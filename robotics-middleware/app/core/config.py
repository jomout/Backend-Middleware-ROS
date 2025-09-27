from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    """
    Application configuration settings.
    Loaded from environment variables or a .env file.
    Use 'settings' instance to access configuration values.
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

    # ROS topics for different robots
    command_topic_1: str = "/robot_1/commands"
    feedback_topic_1: str = "/robot_1/feedback"
    command_topic_2: str = "/robot_2/commands"
    feedback_topic_2: str = "/robot_2/feedback"

    class Config:
        env_file = ".env"

settings = Settings()
