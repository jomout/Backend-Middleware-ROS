import asyncio
import logging
import uuid
import asyncio
import redis.asyncio as redis  # async client

from .config import settings
from .db import SessionLocal
from .models import TaskStack, TaskStackStatus

STREAM_NAME = settings.task_stack_stream
# Use shared consumer group and name; this instance processes events for all devices
CONSUMER_GROUP = settings.task_stack_consumer_group
CONSUMER_NAME = settings.task_stack_consumer_name
REDIS_URL = settings.redis_url

logger = logging.getLogger(__name__)

async def ensure_group(r: redis.Redis):
    try:
        await r.xgroup_create(name=STREAM_NAME, groupname=CONSUMER_GROUP, id="0-0", mkstream=True)
    except Exception as e:
        # Group already exists
        if "BUSYGROUP" in str(e):
            return
        raise

async def consume_stream():
    r = redis.from_url(REDIS_URL, decode_responses=True)
    await ensure_group(r)
    while True:
        try:
            resp = await r.xreadgroup(groupname=CONSUMER_GROUP, consumername=CONSUMER_NAME, streams={STREAM_NAME: ">"}, count=10, block=5000)
            if not resp:
                continue
            for _, messages in resp:
                for msg_id, fields in messages:
                    try:
                        # Align with backend publisher (camelCase keys)
                        device_id = fields.get("deviceId")
                        stack_id = fields.get("stackId")
                        event = fields.get("event")
                        
                        # Process only supported events; handle all devices
                        if event not in ("task_stack.created", "task_stack.pending"):
                            await r.xack(STREAM_NAME, CONSUMER_GROUP, msg_id)
                            continue
                        
                        # Process
                        with SessionLocal() as db:
                            stack = db.query(TaskStack).filter(
                                TaskStack.stack_id == uuid.UUID(stack_id),
                                TaskStack.device_id == uuid.UUID(device_id),
                                TaskStack.status == TaskStackStatus.pending,
                            ).first()
                            if stack:
                                from .worker import process_task_stack
                                
                                # Process the task stack
                                await process_task_stack(stack, db)
                                
                        await r.xack(STREAM_NAME, CONSUMER_GROUP, msg_id)
                        
                    except Exception as e:
                        logger.error("Error processing message %s: %s", msg_id, e)
                        # Do not ack, will be retried
        except Exception as e:
            logger.error("Redis consume error: %s", e)
            await asyncio.sleep(2)
