# CARLA Planning - Docker Quick Start Guide

## TL;DR - Get Running Fast

### Windows
```batch
run-docker.bat up
```

### Linux/Mac
```bash
chmod +x run-docker.sh
./run-docker.sh up
```

## What Gets Created

This Docker setup creates:
- **CARLA Server**: Running in a container on port 4000
- **Python Client**: Your planning code running in another container
- **Network**: Containers can communicate with each other

## Common Commands

### Windows (run-docker.bat)
```batch
run-docker.bat up          # Start everything
run-docker.bat up-d        # Start in background
run-docker.bat logs        # View logs
run-docker.bat down        # Stop everything
run-docker.bat shell       # Open terminal inside client
run-docker.bat help        # See all commands
```

### Linux/Mac (run-docker.sh)
```bash
./run-docker.sh up         # Start everything
./run-docker.sh up-d       # Start in background
./run-docker.sh logs       # View logs
./run-docker.sh down       # Stop everything
./run-docker.sh shell      # Open terminal inside client
./run-docker.sh help       # See all commands
```

## Using Your Existing CARLA Server

If you already have CARLA running on your machine (not in Docker):

1. **Start only the client:**
   ```batch
   # Windows
   run-docker.bat client
   
   # Linux/Mac
   ./run-docker.sh client
   ```

2. **The script will connect to:** `localhost:4000`

## File Structure

```
CARLA_planning/
├── Dockerfile                    # Client container setup
├── docker-compose.yml            # Service orchestration
├── requirements.txt              # Python packages
├── .dockerignore                 # Files to exclude
├── run-docker.bat                # Windows helper script
├── run-docker.sh                 # Linux/Mac helper script
├── README_DOCKER.md              # Full documentation
├── DOCKER_QUICKSTART.md          # This file
└── grp planning/
    └── simple-vehicle.py         # Your main script
```

## Troubleshooting

### "Connection refused"
Wait 30-60 seconds for CARLA server to fully start, then restart the client:
```batch
docker-compose restart carla-client
```

### "Port already in use"
Another CARLA is running. Either:
- Stop it, or
- Edit `docker-compose.yml` and change the port numbers

### "Out of memory"
CARLA needs ~6-8GB RAM. Check Docker Desktop → Settings → Resources

### Want to modify code?
Your code is mounted as a volume, so just edit locally and restart:
```batch
docker-compose restart carla-client
```

## What's Next?

- See `README_DOCKER.md` for complete documentation
- Run different scripts: `run-docker.bat run "grp planning/simple-vehicle-3.py"`
- Access container terminal: `run-docker.bat shell`

## Need Help?

1. Check if Docker is running: `docker --version`
2. View logs: `docker-compose logs carla-server`
3. View client logs: `docker-compose logs carla-client`
4. Full documentation: Read `README_DOCKER.md`
