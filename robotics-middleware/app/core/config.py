import json
from typing import Dict
from uuid import UUID
from pydantic import BaseModel, field_validator
from pydantic_settings import BaseSettings

class DeviceConfig(BaseModel):
    command_topic: str
    feedback_topic: str

class Settings(BaseSettings):
    """
    Application configuration settings.
    Loaded from environment variables or a .env file.
    """
    # Database settings
    database_url: str

    # Redis connection and stream settings
    redis_url: str = "redis://localhost:6379"
    task_stack_stream: str = "task_stacks"
    task_stack_consumer_group: str = "robots"
    task_stack_consumer_name: str = "worker-1"
    
    # ROS settings
    ros_enabled: bool = False

    # Devices as a dict keyed by UUID
    devices: Dict[UUID, DeviceConfig] = {}

    @field_validator("devices", mode="before")
    @classmethod
    def parse_devices(cls, v):
        # If it comes in as a JSON string, parse it
        if isinstance(v, str):
            try:
                v = json.loads(v)
            except Exception:
                raise ValueError("Invalid JSON for devices configuration")

        # If it's already a list, convert it to a dict
        if isinstance(v, list):
            try:
                return {
                    UUID(item["device_id"]): {
                        "command_topic": item["command_topic"],
                        "feedback_topic": item["feedback_topic"],
                    }
                    for item in v
                }
            except Exception as e:
                raise ValueError(f"Invalid devices list format: {e}")

        # If it's already a dict, let Pydantic handle conversion
        return v

    class Config:
        env_file = ".env"

settings = Settings()
