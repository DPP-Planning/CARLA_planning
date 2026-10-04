import os
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PLANNING_DIR = PROJECT_ROOT / "grp planning"
CARLA_PYTHONAPI = os.getenv("CARLA_PYTHONAPI")

if CARLA_PYTHONAPI:
    carla_pythonapi_dir = Path(CARLA_PYTHONAPI).expanduser().resolve()
    if not carla_pythonapi_dir.is_dir():
        raise FileNotFoundError(f"CARLA_PYTHONAPI is not a directory: {carla_pythonapi_dir}")
    sys.path.insert(0, str(carla_pythonapi_dir))
    for carla_egg in sorted((carla_pythonapi_dir / "dist").glob("carla-*.egg")):
        sys.path.insert(0, str(carla_egg))

if str(PLANNING_DIR) not in sys.path:
    sys.path.insert(0, str(PLANNING_DIR))


def create_carla_client(carla_module, default_port=9000):
    host = os.getenv("CARLA_HOST", "localhost")
    port = int(os.getenv("CARLA_PORT", str(default_port)))
    timeout = float(os.getenv("CARLA_TIMEOUT", "10"))

    client = carla_module.Client(host, port)
    client.set_timeout(timeout)
    return client