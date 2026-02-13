# 🐳 Docker Setup for CARLA Planning Project

> **Quick Start**: Run `run-docker.bat up` (Windows) or `./run-docker.sh up` (Linux/Mac)

This Docker setup provides a complete, containerized environment for running the CARLA planning project. Someone can clone your repository and run the entire setup without installing Python, CARLA, or any dependencies locally.

---

## 📋 Table of Contents

1. [Prerequisites](#prerequisites)
2. [Quick Start](#quick-start)
3. [What's Included](#whats-included)
4. [Usage Guide](#usage-guide)
5. [Configuration](#configuration)
6. [Troubleshooting](#troubleshooting)
7. [Development Workflow](#development-workflow)
8. [Files Overview](#files-overview)

---

## Prerequisites

### Required
- **Docker Desktop** (Windows/Mac) or **Docker Engine** (Linux)
  - Download: https://www.docker.com/products/docker-desktop
  - Version: 20.10+
- **Docker Compose**
  - Included with Docker Desktop
  - Linux: `sudo apt-get install docker-compose`
  - Version: 1.29+

### Recommended
- 8GB+ RAM available for Docker
- 10GB+ free disk space
- Multi-core CPU (4+ cores recommended)

### Check Installation
```bash
docker --version
docker-compose --version
```

---

## Quick Start

### Windows

1. **Open Command Prompt or PowerShell** in the project directory
2. **Run the helper script:**
   ```batch
   run-docker.bat up
   ```
3. **Wait** for CARLA server to start (~30-60 seconds)
4. **Watch** as your planning code runs automatically

### Linux/Mac

1. **Open Terminal** in the project directory
2. **Make script executable:**
   ```bash
   chmod +x run-docker.sh
   ```
3. **Run the helper script:**
   ```bash
   ./run-docker.sh up
   ```
4. **Wait** for CARLA server to start (~30-60 seconds)
5. **Watch** as your planning code runs automatically

### Verify Setup

Test that everything is working:
```bash
# Windows
run-docker.bat test

# Linux/Mac
./run-docker.sh test
```

---

## What's Included

This Docker setup provides:

✅ **CARLA Simulator Server** (v0.9.13)
- Runs in headless mode (no GUI)
- Accessible on port 4000 (matches your original setup)
- Pre-configured for optimal performance

✅ **Python Client Environment** (Python 3.8)
- All required dependencies pre-installed
- CARLA Python API
- Navigation agents (basic_agent, global_route_planner, etc.)
- Your custom planning algorithms

✅ **Development Tools**
- Live code reloading (edit locally, changes reflect in container)
- Easy access to container shell
- Log viewing utilities
- Setup verification tests

✅ **Helper Scripts**
- `run-docker.bat` (Windows)
- `run-docker.sh` (Linux/Mac)
- Common operations simplified to single commands

---

## Usage Guide

### Basic Commands

| Command | Description |
|---------|-------------|
| `up` | Start server and client (foreground) |
| `up-d` | Start in background (detached) |
| `down` | Stop all containers |
| `logs` | View client logs |
| `shell` | Open interactive shell in container |
| `test` | Verify Docker setup |
| `rebuild` | Rebuild containers from scratch |
| `help` | Show all commands |

### Examples

#### Start Everything
```bash
# Foreground (see logs in real-time)
run-docker.bat up

# Background (runs silently)
run-docker.bat up-d
```

#### View Logs
```bash
# Client logs
run-docker.bat logs

# Server logs
docker-compose logs carla-server

# All logs
docker-compose logs -f
```

#### Stop Everything
```bash
run-docker.bat down
```

#### Run Different Script
```bash
run-docker.bat run "grp planning/simple-vehicle-3.py"
```

#### Interactive Development
```bash
# Open shell in container
run-docker.bat shell

# Then inside container:
python "grp planning/simple-vehicle.py"
python test_docker_setup.py
```

#### Use Existing CARLA Server
If you already have CARLA running locally:
```bash
run-docker.bat client
```

---

## Configuration

### Port Mapping

The default setup maps:
- **Host Port 4000** → Container Port 2000 (CARLA RPC)
- **Host Port 4001** → Container Port 2001 (CARLA Streaming)
- **Host Port 4002** → Container Port 2002 (CARLA Streaming)

This matches your original setup where CARLA runs on port 4000.

### Change Ports

Edit `docker-compose.yml`:
```yaml
ports:
  - "YOUR_PORT:2000"  # Change YOUR_PORT
```

### CARLA Server Settings

Adjust in `docker-compose.yml`:
```yaml
command: /bin/bash -c "SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl -carla-rpc-port=2000 -quality-level=Low -fps=20"
```

Common options:
- `-quality-level=Low|Medium|High` - Graphics quality
- `-fps=20` - Target frame rate
- `-benchmark` - Benchmark mode
- `-nosound` - Disable audio

### Environment Variables

Copy `.env.example` to `.env` and modify:
```bash
CARLA_HOST=carla-server
CARLA_PORT=2000
```

### GPU Acceleration

Uncomment in `docker-compose.yml`:
```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu]
```

**Note**: Requires nvidia-docker2 installed.

---

## Troubleshooting

### "Connection Refused" Error

**Cause**: CARLA server not fully started yet

**Solution**:
1. Wait 30-60 seconds after starting
2. Check server status:
   ```bash
   docker ps | grep carla-server
   docker logs carla-server
   ```
3. Restart client:
   ```bash
   docker-compose restart carla-client
   ```

### "Port Already in Use"

**Cause**: Another CARLA instance is running

**Solution**:
1. Stop other CARLA instances
2. Or change port in `docker-compose.yml`

### "Out of Memory"

**Cause**: Docker doesn't have enough RAM

**Solution**:
1. **Docker Desktop**: Settings → Resources → Increase memory to 8GB+
2. **Linux**: No limit by default
3. Reduce CARLA quality: `-quality-level=Low`

### "Module Not Found" Error

**Cause**: Import path issues

**Solution**:
1. Verify file structure:
   ```bash
   run-docker.bat shell
   ls -la agents/navigation/
   ```
2. Rebuild container:
   ```bash
   run-docker.bat rebuild
   ```

### CARLA Server Won't Start

**Cause**: Various (GPU, memory, etc.)

**Solution**:
1. Check logs:
   ```bash
   docker logs carla-server
   ```
2. Try software rendering:
   ```yaml
   command: ... -opengl ...  # Already included
   ```
3. Reduce resources:
   ```yaml
   command: ... -quality-level=Low -fps=10 ...
   ```

---

## Development Workflow

### Edit Code Locally

1. **Edit files** in your favorite editor (VS Code, PyCharm, etc.)
2. **Save changes**
3. **Restart client** to apply changes:
   ```bash
   docker-compose restart carla-client
   ```

Your code is mounted as a volume, so changes are immediately available in the container.

### Run Tests

```bash
run-docker.bat test
```

This verifies:
- Python packages installed correctly
- CARLA connection works
- Agent modules available
- File structure correct

### Debug Issues

1. **Open shell in container:**
   ```bash
   run-docker.bat shell
   ```

2. **Run Python interactively:**
   ```python
   python
   >>> import carla
   >>> client = carla.Client('carla-server', 2000)
   >>> world = client.get_world()
   >>> print(world.get_map().name)
   ```

3. **Check Python path:**
   ```bash
   python -c "import sys; print('\n'.join(sys.path))"
   ```

### Add Dependencies

1. **Edit `requirements.txt`:**
   ```
   new-package==1.0.0
   ```

2. **Rebuild container:**
   ```bash
   run-docker.bat rebuild
   ```

---

## Files Overview

### Docker Files

| File | Purpose |
|------|---------|
| `Dockerfile` | Client container definition |
| `docker-compose.yml` | Orchestrates server + client |
| `requirements.txt` | Python dependencies |
| `.dockerignore` | Excludes files from build |
| `.env.example` | Environment variable template |

### Helper Scripts

| File | Purpose |
|------|---------|
| `run-docker.bat` | Windows helper script |
| `run-docker.sh` | Linux/Mac helper script |
| `test_docker_setup.py` | Verification tests |

### Documentation

| File | Purpose |
|------|---------|
| `DOCKER_GUIDE.md` | This comprehensive guide (you are here) |
| `DOCKER_QUICKSTART.md` | Quick reference |
| `README_DOCKER.md` | Detailed technical docs |
| `DOCKER_SUMMARY.md` | Architecture overview |

### Modified Files

| File | Changes |
|------|---------|
| `grp planning/simple-vehicle.py` | Added environment variable support for CARLA connection |

---

## Architecture

```
┌──────────────────────────────────────┐
│         Your Host Machine            │
│                                      │
│  ┌────────────────────────────────┐ │
│  │    Docker Environment          │ │
│  │                                │ │
│  │  ┌──────────────────────────┐ │ │
│  │  │  carla-server            │ │ │
│  │  │  (CARLA Simulator)       │ │ │
│  │  │  Port: 2000 → 4000       │ │ │
│  │  └──────────┬───────────────┘ │ │
│  │             │ Network          │ │
│  │  ┌──────────▼───────────────┐ │ │
│  │  │  carla-client            │ │ │
│  │  │  (Python Environment)    │ │ │
│  │  │  Your Planning Code      │ │ │
│  │  └──────────────────────────┘ │ │
│  │             ▲                  │ │
│  │             │ Volume Mounts    │ │
│  └─────────────┼──────────────────┘ │
│                │                     │
│   ./grp planning/  (editable)       │
│   ./dLite/         (editable)       │
└──────────────────────────────────────┘
```

---

## Common Workflows

### First Time Setup
```bash
# 1. Clone repository
git clone <your-repo>
cd CARLA_planning

# 2. Run setup
run-docker.bat up

# 3. Verify (in another terminal)
run-docker.bat test
```

### Daily Development
```bash
# Start in background
run-docker.bat up-d

# Edit code in your favorite editor
# ...

# Restart to apply changes
docker-compose restart carla-client

# View logs
run-docker.bat logs

# Stop when done
run-docker.bat down
```

### Testing Different Scripts
```bash
# Start server only
docker-compose up -d carla-server

# Run different scripts
run-docker.bat run "grp planning/simple-vehicle.py"
run-docker.bat run "grp planning/simple-vehicle-3.py"
run-docker.bat run "dLite/CarlaDLiteMain.py"
```

### Clean Restart
```bash
# Stop everything
run-docker.bat down

# Rebuild from scratch
run-docker.bat rebuild

# Start again
run-docker.bat up
```

---

## System Requirements

### Minimum
- 4GB RAM (8GB recommended)
- 2 CPU cores (4+ recommended)
- 5GB disk space
- Docker 20.10+

### Recommended
- 8GB+ RAM
- 4+ CPU cores
- 10GB disk space
- SSD storage
- GPU (optional, for better performance)

---

## Support & Resources

### Documentation
- **Quick Start**: `DOCKER_QUICKSTART.md`
- **Technical Details**: `README_DOCKER.md`
- **Architecture**: `DOCKER_SUMMARY.md`
- **This Guide**: `DOCKER_GUIDE.md`

### External Links
- [CARLA Documentation](https://carla.readthedocs.io/)
- [Docker Documentation](https://docs.docker.com/)
- [CARLA Docker Hub](https://hub.docker.com/r/carlasim/carla)

### Getting Help

1. **Check logs**:
   ```bash
   docker-compose logs
   ```

2. **Run tests**:
   ```bash
   run-docker.bat test
   ```

3. **Check Docker status**:
   ```bash
   docker ps
   docker stats
   ```

---

## Next Steps

After getting the basic setup running:

1. ✅ Verify with `run-docker.bat test`
2. 📝 Review your code in `grp planning/`
3. 🚗 Run your planning algorithms
4. 🔧 Customize CARLA settings as needed
5. 📊 Add visualization or logging as desired

---

## Credits

**Project**: CARLA Planning with D* Lite
**CARLA Version**: 0.9.13
**Python Version**: 3.8
**Docker Setup Created**: 2026-02-12

---

**Happy Planning! 🚗💨**
