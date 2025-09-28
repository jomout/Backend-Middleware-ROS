import asyncio
import logging
import uuid
import asyncio
import redis.asyncio as redis

from .config import settings
from .db import SessionLocal
from .models import Device, DeviceStatus, TaskStack, TaskStackStatus

from .logging import get_logger

logger = get_logger(__name__)


# ROS settings
ROS_ENABLED = settings.ros_enabled

# Redis stream settings
STREAM_NAME = settings.task_stack_stream
# Use shared consumer group and name. This instance processes events for all devices
CONSUMER_GROUP = settings.task_stack_consumer_group
CONSUMER_NAME = settings.task_stack_consumer_name
REDIS_URL = settings.redis_url


async def ensure_group(r: redis.Redis):
    """
    Ensure the Redis consumer group exists. Create if not.
    """
    try:
        await r.xgroup_create(name=STREAM_NAME, groupname=CONSUMER_GROUP, id="0-0", mkstream=True)
    except Exception as e:
        # Group already exists
        if "BUSYGROUP" in str(e):
            return
        raise


async def consume_stream():
    """
    Consume messages from the Redis stream and process task stacks.
    """
    # Create Redis connection
    r = redis.from_url(REDIS_URL, decode_responses=True)

    # Ensure consumer group exists
    await ensure_group(r)

    # Wait for messages
    while True:
        try:
            # Read messages from the stream
            response = await r.xreadgroup(groupname=CONSUMER_GROUP, consumername=CONSUMER_NAME, streams={STREAM_NAME: ">"}, count=10, block=5000)
            if not response:
                continue

            # Process messages from response
            for _, messages in response:
                for msg_id, fields in messages:
                    stack = None
                    success = False

                    device_id = fields.get("deviceId")
                    stack_id = fields.get("stackId")
                    event = fields.get("event")

                    if not device_id or not stack_id or not event:
                        logger.warning(f"Invalid message fields: {fields}")
                        # Ack and skip
                        await r.xack(STREAM_NAME, CONSUMER_GROUP, msg_id)
                        continue

                    with SessionLocal() as db:
                        try:
                            if event != "task_stack.created":
                                logger.warning(f"Unsupported event: {event}")
                                raise Exception(f"Unsupported event: {event}")

                            # Lookup device
                            device = db.query(Device).filter(
                                Device.device_id == uuid.UUID(device_id)
                            ).first()

                            # Ensure device is online
                            if not device or device.status != DeviceStatus.online:
                                logger.warning(f"Device {device_id} is not online")
                                # Mark task stack failed if present
                                stack = db.query(TaskStack).filter(
                                    TaskStack.stack_id == uuid.UUID(stack_id),
                                    TaskStack.device_id == uuid.UUID(device_id),
                                ).first()
                                if stack:
                                    stack.status = TaskStackStatus.failed
                                    db.commit()
                                # Ack and skip further work
                                await r.xack(STREAM_NAME, CONSUMER_GROUP, msg_id)
                                continue

                            # Lookup task stack
                            stack = db.query(TaskStack).filter(
                                TaskStack.stack_id == uuid.UUID(stack_id),
                                TaskStack.device_id == uuid.UUID(device_id),
                                TaskStack.status == TaskStackStatus.pending,
                            ).first()

                            if not stack:
                                logger.warning(
                                    f"Task stack {stack_id} for device {device_id} not found or not pending"
                                )
                                await r.xack(STREAM_NAME, CONSUMER_GROUP, msg_id)
                                continue

                            # Mark in progress before dispatching it
                            stack.status = TaskStackStatus.in_progress
                            db.commit()

                            # Process stack
                            if ROS_ENABLED:
                                from .ros_worker import process_task_stack
                                success = await process_task_stack(device.name, stack)
                            else:
                                from .worker import process_task_stack
                                success = await process_task_stack(device.name, stack)

                        except Exception as e:
                            logger.error(f"Error processing stack {stack_id} for device {device_id}: {e}")
                            success = False
                        finally:
                            # Update stack status if we have it
                            if stack:
                                stack.status = (
                                    TaskStackStatus.completed if success else TaskStackStatus.failed
                                )
                                db.commit()

                            # Always ack once per message
                            await r.xack(STREAM_NAME, CONSUMER_GROUP, msg_id)

        except Exception as e:
            logger.error("Redis consume error: %s", e)
            await asyncio.sleep(2)



