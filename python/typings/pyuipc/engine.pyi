class Engine(IEngine):
    def __init__(self, backend_name: str, workspace: str = ..., config: json = ...) -> None: ...

class IEngine:
    def __init__(self, *args, **kwargs) -> None: ...
    def advance(self) -> None: ...
    def dump(self) -> bool: ...
    def frame(self) -> int: ...
    def init(self, arg0) -> None: ...
    def recover(self) -> bool: ...
    def retrieve(self) -> None: ...
    def sync(self) -> None: ...
    def to_json(self) -> json: ...
