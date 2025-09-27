import enum
import uuid
from sqlalchemy import Column, String, ForeignKey, JSON, Enum, TIMESTAMP
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class DeviceStatus(enum.Enum):
    """Represents the status of a device."""
    online = "online"
    offline = "offline"

class TaskStackStatus(enum.Enum):
    """Represents the status of a task stack."""
    pending = "pending"
    in_progress = "in_progress"
    completed = "completed"
    failed = "failed"

class User(Base):
    """
    Represents a user in the system.
    """
    __tablename__ = "users"

    user_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String, unique=True, nullable=False)
    name = Column(String, nullable=False)
    password = Column(String, nullable=False)
    created_at = Column(TIMESTAMP, nullable=False)
    updated_at = Column(TIMESTAMP, nullable=False)

class Device(Base):
    """
    Represents a robotic device.
    """
    __tablename__ = "devices"

    device_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    name = Column(String, nullable=False)
    status = Column(Enum(DeviceStatus), nullable=False)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.user_id"), nullable=False)
    created_at = Column(TIMESTAMP, nullable=False)
    updated_at = Column(TIMESTAMP, nullable=False)

class TaskStack(Base):
    """
    Represents a stack of tasks for a robotic device.
    """
    __tablename__ = "task_stacks"

    stack_id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    device_id = Column(UUID(as_uuid=True), ForeignKey("devices.device_id"), nullable=False)
    tasks = Column(JSON, nullable=False)
    status = Column(Enum(TaskStackStatus), default=TaskStackStatus.pending, nullable=False)
    created_at = Column(TIMESTAMP, nullable=False)
    updated_at = Column(TIMESTAMP, nullable=False)
