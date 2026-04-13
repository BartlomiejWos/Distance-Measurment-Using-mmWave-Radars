from dataclasses import dataclass


@dataclass
class AppArgs:
    file_prefix: str | None
    auto_prefix: str | None
    cfg_file_path: str
    verbose: bool = False
