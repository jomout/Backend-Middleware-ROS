import asyncio
import logging
from fastapi import FastAPI
from contextlib import asynccontextmanager
from .core.redis_consumer import consume_stream
from .core.ros_bridge import bridge
from .core.config import settings

from .core.logging import get_logger

logger = get_logger(__name__)

worker_task = None

ROS_ENABLED = settings.ros_enabled

@asynccontextmanager
async def lifespan(app: FastAPI):
    global worker_task

    # Starting Redis consumer worker
    logger.info("Starting Redis consumer worker...")
    worker_task = asyncio.create_task(consume_stream())
    
    if ROS_ENABLED:
        # Start ROS bridge (background ROS2 executor)
        logger.info("Starting ROS bridge...")
        loop = asyncio.get_running_loop()
        bridge.start(loop)

    # App runs here
    yield 

    # Shutdown Redis consumer
    logger.info("Shutting down Redis consumer...")
    if worker_task:
        worker_task.cancel()
        try:
            await worker_task
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"Worker crashed during shutdown: {e}")

    if ROS_ENABLED:
        # Shutdown ROS bridge
        logger.info("Shutting down ROS bridge...")
        bridge.stop()

app = FastAPI(lifespan=lifespan)
