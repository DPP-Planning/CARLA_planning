#!/bin/bash
# run-docker.sh - Helper script to run CARLA Planning with Docker

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}CARLA Planning Docker Setup${NC}"
echo "================================"

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}Error: Docker is not running. Please start Docker and try again.${NC}"
    exit 1
fi

# Check if docker-compose is available
if ! command -v docker-compose &> /dev/null; then
    echo -e "${RED}Error: docker-compose is not installed.${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Docker is running${NC}"

# Parse command line arguments
MODE=${1:-"up"}
SCRIPT=${2:-"grp planning/simple-vehicle.py"}

case $MODE in
    "up")
        echo -e "\n${GREEN}Starting CARLA server and client...${NC}"
        docker-compose up --build
        ;;
    "up-d")
        echo -e "\n${GREEN}Starting CARLA server and client in background...${NC}"
        docker-compose up -d --build
        echo -e "${GREEN}✓ Containers started${NC}"
        echo -e "\nTo view logs: ${YELLOW}docker-compose logs -f carla-client${NC}"
        echo -e "To stop: ${YELLOW}docker-compose down${NC}"
        ;;
    "client")
        echo -e "\n${GREEN}Starting only the client (expecting external CARLA server)...${NC}"
        docker-compose up --build carla-client
        ;;
    "run")
        echo -e "\n${GREEN}Running script: $SCRIPT${NC}"
        docker-compose run --rm carla-client python "$SCRIPT"
        ;;
    "shell")
        echo -e "\n${GREEN}Opening interactive shell in client container...${NC}"
        docker-compose run --rm carla-client bash
        ;;
    "logs")
        echo -e "\n${GREEN}Showing client logs...${NC}"
        docker-compose logs -f carla-client
        ;;
    "down")
        echo -e "\n${GREEN}Stopping containers...${NC}"
        docker-compose down
        echo -e "${GREEN}✓ Containers stopped${NC}"
        ;;
    "rebuild")
        echo -e "\n${GREEN}Rebuilding containers...${NC}"
        docker-compose down
        docker-compose build --no-cache
        echo -e "${GREEN}✓ Rebuild complete${NC}"
        ;;
    "help")
        echo -e "\nUsage: $0 [command] [script]"
        echo -e "\nCommands:"
        echo -e "  ${YELLOW}up${NC}        - Start server and client (foreground)"
        echo -e "  ${YELLOW}up-d${NC}      - Start server and client (background)"
        echo -e "  ${YELLOW}client${NC}    - Start only client (use existing server)"
        echo -e "  ${YELLOW}run${NC}       - Run a specific script (default: simple-vehicle.py)"
        echo -e "  ${YELLOW}shell${NC}     - Open interactive shell in client container"
        echo -e "  ${YELLOW}logs${NC}      - Show client logs"
        echo -e "  ${YELLOW}down${NC}      - Stop all containers"
        echo -e "  ${YELLOW}rebuild${NC}   - Rebuild containers from scratch"
        echo -e "  ${YELLOW}test${NC}      - Run Docker setup verification tests"
        echo -e "  ${YELLOW}help${NC}      - Show this help message"
        echo -e "\nExamples:"
        echo -e "  $0 up"
        echo -e "  $0 run \"grp planning/simple-vehicle-3.py\""
        echo -e "  $0 shell"
        echo -e "  $0 test"
        ;;
    "test")
        echo -e "\n${GREEN}Running Docker setup verification tests...${NC}"
        docker-compose --profile test run --rm test
        ;;
    *)
        echo -e "${RED}Unknown command: $MODE${NC}"
        echo -e "Use ${YELLOW}$0 help${NC} to see available commands"
        exit 1
        ;;
esac
