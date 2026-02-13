# Docker Files Summary for CARLA Planning Project

This document summarizes all the Docker-related files created for running the CARLA planning project.

## Files Created

### 1. Core Docker Files

#### `Dockerfile`
- Creates the Python client environment
- Based on Python 3.8-slim
- Installs CARLA Python API (carla==0.9.13)
- Installs dependencies from requirements.txt
- Sets up the agents/navigation directory structure
- Creates minimal agents.tools.misc module for required imports

#### `docker-compose.yml`
- Orchestrates both CARLA server and client containers
- **carla-server**: CARLA simulator (port 4000 on host → 2000 in container)
- **carla-client**: Your Python planning code
- Includes volume mounts for live code editing
- Network bridge for container communication

#### `requirements.txt`
- Python dependencies:
  - numpy (array operations)
  - pygame (visualization, optional)
  - networkx (graph operations for planning)
  - shapely (geometry operations)
  - matplotlib (plotting)
  - carla (CARLA Python API)

#### `.dockerignore`
- Excludes unnecessary files from Docker build
- Reduces image size and build time
- Excludes: cache, logs, documentation, git files, IDEs

#### `.env.example`
- Template for environment variables
- Documents CARLA connection settings
- Can be copied to `.env` for custom configuration

### 2. Helper Scripts

#### `run-docker.bat` (Windows)
- Convenience script for common Docker operations
- Commands: up, up-d, client, run, shell, logs, down, rebuild, help
- Checks Docker availability before running
- Color-coded output for better UX

#### `run-docker.sh` (Linux/Mac)
- Same functionality as run-docker.bat but for Unix systems
- Needs execute permission: `chmod +x run-docker.sh`

### 3. Documentation

#### `README_DOCKER.md`
- Complete Docker setup documentation
- Prerequisites and system requirements
- Quick start guide with docker-compose
- Alternative setup methods
- Configuration options (GPU, ports, quality settings)
- Troubleshooting common issues
- Development workflow tips
- Project structure overview

#### `DOCKER_QUICKSTART.md`
- TL;DR quick reference guide
- Common commands cheat sheet
- Basic troubleshooting
- Points to full README_DOCKER.md for details

## Usage Examples

### Start Everything
```batch
# Windows
run-docker.bat up

# Linux/Mac
./run-docker.sh up
```

### Run in Background
```batch
run-docker.bat up-d
```

### Use Existing CARLA Server
```batch
run-docker.bat client
```

### Run Different Script
```batch
run-docker.bat run "grp planning/simple-vehicle-3.py"
```

### Access Container Shell
```batch
run-docker.bat shell
```

### Stop Everything
```batch
run-docker.bat down
```

## Key Features

### 1. Flexibility
- Can run both server and client in Docker
- Can run just client with external server
- Environment variables for easy configuration
- Volume mounts for live code editing

### 2. Compatibility
- Works on Windows, Linux, and Mac
- Helper scripts for each platform
- Port mapping compatible with existing setup (4000)

### 3. Development-Friendly
- Code changes reflect immediately (volume mounts)
- Easy to access container shell for debugging
- Logs easily viewable
- Can run different scripts without rebuilding

### 4. Self-Contained
- All dependencies installed in container
- Consistent environment across systems
- No need to install Python packages locally
- No CARLA installation required on host

## Architecture

```
┌─────────────────────────────────────────┐
│          Host Machine                    │
│                                          │
│  ┌────────────────────────────────────┐ │
│  │  Docker Network: carla-network     │ │
│  │                                    │ │
│  │  ┌──────────────────────────────┐ │ │
│  │  │  carla-server                │ │ │
│  │  │  - CARLA Simulator 0.9.13   │ │ │
│  │  │  - Port 2000 (internal)     │ │ │
│  │  │  - Mapped to 4000 (host)    │ │ │
│  │  └────────────┬─────────────────┘ │ │
│  │               │                   │ │
│  │               │ Network           │ │
│  │               │                   │ │
│  │  ┌────────────▼─────────────────┐ │ │
│  │  │  carla-client                │ │ │
│  │  │  - Python 3.8                │ │ │
│  │  │  - Your planning code        │ │ │
│  │  │  - Volume: ./grp planning    │ │ │
│  │  │  - Connects to carla-server  │ │ │
│  │  └──────────────────────────────┘ │ │
│  └────────────────────────────────────┘ │
│                                          │
│  Your Code (mounted as volumes)         │
│  - grp planning/                         │
│  - dLite/                                │
└─────────────────────────────────────────┘
```

## Modified Files

### `grp planning/simple-vehicle.py`
**Changes Made:**
- Added `import os` for environment variables
- Reads CARLA_HOST and CARLA_PORT from environment
- Prints connection info for debugging
- Backwards compatible (defaults to localhost:4000)

**Before:**
```python
client = carla.Client("localhost", 4000)
```

**After:**
```python
CARLA_HOST = os.getenv('CARLA_HOST', 'localhost')
CARLA_PORT = int(os.getenv('CARLA_PORT', '4000'))
print(f"Connecting to CARLA server at {CARLA_HOST}:{CARLA_PORT}")
client = carla.Client(CARLA_HOST, CARLA_PORT)
```

## Directory Structure Created

```
CARLA_planning/
├── Dockerfile                      # Client container
├── docker-compose.yml              # Service orchestration
├── requirements.txt                # Python dependencies
├── .dockerignore                   # Build exclusions
├── .env.example                    # Environment template
├── run-docker.bat                  # Windows helper
├── run-docker.sh                   # Linux/Mac helper
├── README_DOCKER.md                # Full documentation
├── DOCKER_QUICKSTART.md            # Quick reference
├── DOCKER_SUMMARY.md               # This file
└── grp planning/
    └── simple-vehicle.py           # Modified for Docker
```

## Important Notes

### Port Configuration
- **Host**: 4000 (matches your original setup)
- **Container**: 2000 (CARLA default)
- Mapping: `4000:2000` in docker-compose.yml

### Python Path
- Container sets PYTHONPATH to `/app`
- `sys.path.append('../')` in code works correctly
- agents/navigation modules created in container build

### CARLA Version
- Default: 0.9.13
- To change: Update both Dockerfile and docker-compose.yml
- Must match server and client versions

### Volumes
- `./grp planning` → `/app/grp planning` (read/write)
- `./dLite` → `/app/dLite` (read/write)
- Changes on host reflect in container immediately

## Next Steps for Users

1. **Install Docker**: Docker Desktop (Windows/Mac) or Docker Engine (Linux)
2. **Run Quick Start**: `run-docker.bat up` or `./run-docker.sh up`
3. **Wait**: CARLA server takes 30-60 seconds to start
4. **Verify**: Check logs with `docker-compose logs -f`
5. **Develop**: Edit code locally, restart client container to apply changes

## Support Different Scenarios

### Scenario 1: Complete Isolated Environment
```bash
docker-compose up --build
```
Both server and client run in Docker, completely isolated.

### Scenario 2: External CARLA Server
```bash
# Run only client
docker-compose up carla-client
```
Uses existing CARLA on host machine.

### Scenario 3: Development with GPU
Uncomment GPU settings in docker-compose.yml for better performance.

### Scenario 4: Custom Scripts
```bash
run-docker.bat run "path/to/script.py"
```
Run any Python script in the project.

## Troubleshooting Resources

All common issues documented in:
- **README_DOCKER.md**: Detailed troubleshooting section
- **DOCKER_QUICKSTART.md**: Quick fixes
- **Helper scripts**: Check Docker availability automatically

## Benefits of This Setup

1. **Reproducible**: Same environment on any machine
2. **No Installation**: No need to install CARLA/Python locally
3. **Version Locked**: Exact Python/CARLA versions specified
4. **Easy Sharing**: Send Docker files, others get same environment
5. **Clean**: No pollution of host system
6. **Flexible**: Multiple configuration options
7. **Documented**: Comprehensive documentation provided

---

**Created**: 2026-02-12
**CARLA Version**: 0.9.13
**Python Version**: 3.8
**Docker Compose Version**: 3.8
