import logging


def get_logger(name: str) -> logging.Logger:
    """
    Get Uvicorns logger
    """
    return logging.getLogger("uvicorn.error").getChild(name)