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
                    try:
                        # Get event fields
                        device_id = fields.get("deviceId")
                        stack_id = fields.get("stackId")
                        event = fields.get("event")
                        
                        # Process only supported events. Handle all devices
                        if event not in ("task_stack.created", "task_stack.pending"):
                            logger.warning(f"[Robot {device_id}] Unsupported event: {event}")
                            # Acknowledge unsupported event
                            await r.xack(STREAM_NAME, CONSUMER_GROUP, msg_id)
                            continue
                        
                        # Process task stack
                        with SessionLocal() as db:
                            
                            # Check device status
                            device_status = db.query(Device).filter(
                                Device.device_id == uuid.UUID(device_id)
                            ).first().status
                            
                            if device_status != DeviceStatus.online:
                                logger.warning(f"[Robot {device_id}] Device is not online")
                                await r.xack(STREAM_NAME, CONSUMER_GROUP, msg_id)
                                continue

                            # Fetch the task stack
                            stack = db.query(TaskStack).filter(
                                TaskStack.stack_id == uuid.UUID(stack_id),
                                TaskStack.device_id == uuid.UUID(device_id),
                                TaskStack.status == TaskStackStatus.pending,
                            ).first()
                            
                            if stack:
                                if ROS_ENABLED:
                                    # Process the task stack via ROS
                                    from .ros_worker import process_task_stack
                                
                                    await process_task_stack(stack, db)
                                else:
                                    # Process the task stack without ROS
                                    from .worker import process_task_stack

                                    await process_task_stack(stack, db)

                        # Acknowledge message processed
                        await r.xack(STREAM_NAME, CONSUMER_GROUP, msg_id)
                        
                    except Exception as e:
                        # Do not acknowledge, will be retried
                        logger.error("Error processing message %s: %s", msg_id, e)
                        
        except Exception as e:
            logger.error("Redis consume error: %s", e)
            await asyncio.sleep(2)



