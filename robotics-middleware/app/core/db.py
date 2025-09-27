from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from .config import settings

"""
Database session management.
Provides a SQLAlchemy engine and session factory for database interactions.
"""

engine = create_engine(settings.database_url, future=True)
SessionLocal = sessionmaker(bind=engine, autocommit=False, autoflush=False)
