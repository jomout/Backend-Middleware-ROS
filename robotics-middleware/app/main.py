import asyncio
import logging
from fastapi import FastAPI
from contextlib import asynccontextmanager
from .core.redis_consumer import consume_stream
from .core.ros_bridge import bridge

logging.basicConfig(level=logging.INFO)

worker_task = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global worker_task

    # Starting Redis consumer worker
    logging.info("Starting Redis consumer worker...")
    worker_task = asyncio.create_task(consume_stream())
    
    # Start ROS bridge (background ROS2 executor)
    logging.info("Starting ROS bridge...")
    loop = asyncio.get_running_loop()
    bridge.start(loop)

    # App runs here
    yield 

    # Shutdown Redis consumer
    logging.info("Shutting down Redis consumer...")
    if worker_task:
        worker_task.cancel()
        try:
            await worker_task
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logging.error(f"Worker crashed during shutdown: {e}")
    
    # Shutdown ROS bridge
    logging.info("Shutting down ROS bridge...")
    bridge.stop()

app = FastAPI(lifespan=lifespan)
