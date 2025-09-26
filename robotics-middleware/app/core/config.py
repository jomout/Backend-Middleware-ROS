import uuid
from typing import Optional
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # Database settings
    database_url: str

    # Redis connection and stream settings
    redis_url: str = "redis://localhost:6379"
    task_stack_stream: str = "task_stacks"
    task_stack_consumer_group: str = "robots"
    task_stack_consumer_name: Optional[str] = None

    class Config:
        env_file = ".env"

settings = Settings()
