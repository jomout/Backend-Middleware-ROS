import asyncio
from fastapi import FastAPI
from contextlib import asynccontextmanager
from .core.redis_consumer import consume_stream

worker_task = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    global worker_task

    # Startup
    print("Starting Redis consumer worker...")
    worker_task = asyncio.create_task(consume_stream())

    yield  # <-- App runs here

    # Shutdown
    print("Shutting down worker...")
    if worker_task:
        worker_task.cancel()
        try:
            await worker_task
        except asyncio.CancelledError:
            pass

app = FastAPI(lifespan=lifespan)
