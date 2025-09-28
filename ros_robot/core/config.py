import json
from uuid import UUID
from pydantic import BaseModel, field_validator
from pydantic_settings import BaseSettings

class DeviceConfig(BaseModel):
    device_id: UUID
    command_topic: str
    feedback_topic: str

class Settings(BaseSettings):
    """
    Robot configuration settings.
    Loaded from environment variables or a .env file.
    """
    device: DeviceConfig

    @field_validator("device", mode="before")
    @classmethod
    def parse_device(cls, v):
        if isinstance(v, str):
            try:
                return json.loads(v)
            except Exception as e:
                raise ValueError(f"Invalid JSON for device configuration: {e}")
        return v

    class Config:
        env_file = ".env"

settings = Settings()