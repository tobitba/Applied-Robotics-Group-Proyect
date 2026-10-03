from dataclasses import dataclass, fields
from typing import Dict, Type, Any


class ConfigRegistry:
    _registry: Dict[str, Type] = {}

    @classmethod
    def register(cls, name: str):
        def wrapper(config_cls: Type):
            cls._registry[name] = config_cls
            return config_cls

        return wrapper
