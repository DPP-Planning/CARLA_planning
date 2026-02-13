# Docker Setup for CARLA Planning

This Docker setup allows you to run the CARLA planning project in a containerized environment, ensuring consistent behavior across different systems.

## Prerequisites

- Docker Engine 20.10+ or Docker Desktop
- Docker Compose 1.29+
- At least 8GB of available RAM
- (Optional) NVIDIA GPU with nvidia-docker2 for GPU acceleration

## Quick Start

### Option 1: Using Docker Compose (Recommended)

1. **Build and start both CARLA server and client:**
   ```bash
   docker-compose up --build
   ```

2. **To run in detached mode (background):**
   ```bash
   docker-compose up -d --build
   ```

3. **View logs:**
   ```bash
   docker-compose logs -f carla-client
   ```

4. **Stop the containers:**
   ```bash
   docker-compose down
   ```

### Option 2: Using Separate Commands

1. **Start CARLA server first:**
   ```bash
   docker run --rm -p 2000:2000 -p 2001:2001 -p 2002:2002 \
     --name carla-server \
     carlasim/carla:0.9.13 \
     /bin/bash -c "SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl -carla-rpc-port=2000 -quality-level=Low"
   ```

2. **Build the client image:**
   ```bash
   docker build -t carla-planning-client .
   ```

3. **Run the client:**
   ```bash
   docker run --rm \
     --network host \
     -e CARLA_HOST=localhost \
     -e CARLA_PORT=2000 \
     carla-planning-client
   ```

## Configuration

### Connecting to Existing CARLA Server

If you already have a CARLA server running (not in Docker):

1. **Modify the client connection in `simple-vehicle.py`:**
   ```python
   # Change from:
   client = carla.Client("localhost", 4000)
   
   # To:
   client = carla.Client(os.getenv("CARLA_HOST", "localhost"), 
                         int(os.getenv("CARLA_PORT", "2000")))
   ```

2. **Run only the client:**
   ```bash
   docker-compose up carla-client
   ```

3. **Or set environment variables:**
   ```bash
   docker run --rm \
     --network host \
     -e CARLA_HOST=localhost \
     -e CARLA_PORT=4000 \
     carla-planning-client
   ```

### Using GPU Acceleration

To enable GPU support for the CARLA server:

1. **Install nvidia-docker2:**
   ```bash
   # Ubuntu/Debian
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
     sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   sudo apt-get update && sudo apt-get install -y nvidia-docker2
   sudo systemctl restart docker
   ```

2. **Uncomment GPU settings in `docker-compose.yml`:**
   ```yaml
   deploy:
     resources:
       reservations:
         devices:
           - driver: nvidia
             count: 1
             capabilities: [gpu]
   ```

### Customizing CARLA Server Settings

Edit the `docker-compose.yml` file to adjust CARLA server parameters:

```yaml
command: /bin/bash -c "SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl -carla-rpc-port=2000 -quality-level=Low -fps=20"
```

Common options:
- `-quality-level=Low|Medium|High` - Graphics quality
- `-fps=20` - Target frame rate
- `-benchmark` - Run in benchmark mode
- `-nosound` - Disable audio

## Project Structure

```
CARLA_planning/
├── Dockerfile              # Client container definition
├── docker-compose.yml      # Orchestration configuration
├── requirements.txt        # Python dependencies
├── .dockerignore          # Files to exclude from build
├── README_DOCKER.md       # This file
├── grp planning/          # Planning algorithms
│   └── simple-vehicle.py  # Main script
└── agents/                # CARLA agents (navigation)
    └── navigation/
        ├── global_route_planner.py
        ├── basic_agent.py
        └── local_planner.py
```

## Troubleshooting

### Connection Refused Error

If you get a connection refused error:

1. **Check if CARLA server is running:**
   ```bash
   docker ps | grep carla-server
   ```

2. **Check server logs:**
   ```bash
   docker logs carla-server
   ```

3. **Wait for server to fully start** (can take 30-60 seconds)

### Port Already in Use

If port 2000 is already in use:

1. **Change the port mapping in `docker-compose.yml`:**
   ```yaml
   ports:
     - "4000:2000"  # Maps host port 4000 to container port 2000
   ```

2. **Update client environment:**
   ```yaml
   environment:
     - CARLA_PORT=2000  # Keep internal port
   ```

### Out of Memory

If Docker runs out of memory:

1. **Increase Docker memory limit** (Docker Desktop → Settings → Resources)
2. **Reduce CARLA quality settings** in docker-compose.yml
3. **Run server locally** and only use Docker for client

### Version Mismatch

If you get API version mismatch errors:

1. **Check your CARLA version:**
   - The default is 0.9.13
   - Update the Docker image tag in `docker-compose.yml`
   - Update the pip install version in `Dockerfile`

## Development Workflow

### Live Code Updates

The docker-compose.yml mounts your local code, so changes are reflected immediately:

1. **Edit your code locally**
2. **Restart the client container:**
   ```bash
   docker-compose restart carla-client
   ```

### Running Different Scripts

To run a different Python script:

```bash
docker-compose run --rm carla-client python "grp planning/simple-vehicle-3.py"
```

### Interactive Mode

To get a shell inside the container:

```bash
docker-compose run --rm carla-client bash
```

Then run scripts manually:
```bash
python "grp planning/simple-vehicle.py"
```

## Additional Resources

- [CARLA Documentation](https://carla.readthedocs.io/)
- [CARLA Docker Hub](https://hub.docker.com/r/carlasim/carla)
- [Docker Documentation](https://docs.docker.com/)

## Notes

- The CARLA server container runs in headless mode (no GUI)
- Default ports: 2000 (RPC), 2001 (streaming), 2002 (streaming)
- The client waits for the server to be ready before connecting
- Server startup can take 30-60 seconds depending on your system
