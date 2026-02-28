from dataclasses import dataclass


@dataclass
class AppArgs:
    file_prefix: str
    config: str | None
    verbose: bool = False
