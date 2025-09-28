from __future__ import annotations

from typing import Any, Dict, Literal, Optional
from uuid import UUID
from pydantic import BaseModel, Field, field_validator


class Command(BaseModel):
	"""
	Command message sent by the middleware to a robot command topic.

	- event: must be "task.execute"
	- stack_id: UUID of the task stack
	- task_index: 0-based index
	- task: an object. Current types include {"type": "pick"|"place", ...}
	"""

	event: Literal["task.execute"]
	stack_id: UUID
	task_index: int = Field(ge=0)
	task: Dict[str, Any]

	@field_validator("task")
	@classmethod
	def validate_task(cls, v: Dict[str, Any]):
		ttype = v.get("type")
		if ttype not in ("pick", "place"):
			raise ValueError(f"Unsupported task.type: {ttype}")
		return v


class Feedback(BaseModel):
	"""
	Feedback message sent by a robot to its feedback topic.

	- event: "task.completed" | "task.failed"
	- stack_id: UUID matching the command
	- task_index: integer matching the command
	- error: optional details when failed
	"""

	event: Literal["task.completed", "task.failed"]
	stack_id: UUID
	task_index: int = Field(ge=0)
	error: Optional[str] = None
