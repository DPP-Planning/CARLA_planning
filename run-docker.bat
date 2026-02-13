@echo off
REM run-docker.bat - Helper script to run CARLA Planning with Docker on Windows

setlocal enabledelayedexpansion

echo CARLA Planning Docker Setup
echo ================================

REM Check if Docker is running
docker info >nul 2>&1
if errorlevel 1 (
    echo Error: Docker is not running. Please start Docker and try again.
    exit /b 1
)

REM Check if docker-compose is available
docker-compose --version >nul 2>&1
if errorlevel 1 (
    echo Error: docker-compose is not installed.
    exit /b 1
)

echo [OK] Docker is running

REM Parse command line arguments
set MODE=%1
set SCRIPT=%2

if "%MODE%"=="" set MODE=up
if "%SCRIPT%"=="" set SCRIPT=grp planning/simple-vehicle.py

if "%MODE%"=="up" (
    echo.
    echo Starting CARLA server and client...
    docker-compose up --build
    goto :eof
)

if "%MODE%"=="up-d" (
    echo.
    echo Starting CARLA server and client in background...
    docker-compose up -d --build
    echo [OK] Containers started
    echo.
    echo To view logs: docker-compose logs -f carla-client
    echo To stop: docker-compose down
    goto :eof
)

if "%MODE%"=="client" (
    echo.
    echo Starting only the client (expecting external CARLA server)...
    docker-compose up --build carla-client
    goto :eof
)

if "%MODE%"=="run" (
    echo.
    echo Running script: %SCRIPT%
    docker-compose run --rm carla-client python "%SCRIPT%"
    goto :eof
)

if "%MODE%"=="shell" (
    echo.
    echo Opening interactive shell in client container...
    docker-compose run --rm carla-client bash
    goto :eof
)

if "%MODE%"=="logs" (
    echo.
    echo Showing client logs...
    docker-compose logs -f carla-client
    goto :eof
)

if "%MODE%"=="down" (
    echo.
    echo Stopping containers...
    docker-compose down
    echo [OK] Containers stopped
    goto :eof
)

if "%MODE%"=="rebuild" (
    echo.
    echo Rebuilding containers...
    docker-compose down
    docker-compose build --no-cache
    echo [OK] Rebuild complete
    goto :eof
)

if "%MODE%"=="help" (
    echo.
    echo Usage: %0 [command] [script]
    echo.
    echo Commands:
    echo   up        - Start server and client (foreground^)
    echo   up-d      - Start server and client (background^)
    echo   client    - Start only client (use existing server^)
    echo   run       - Run a specific script (default: simple-vehicle.py^)
    echo   shell     - Open interactive shell in client container
    echo   logs      - Show client logs
    echo   down      - Stop all containers
    echo   rebuild   - Rebuild containers from scratch
    echo   test      - Run Docker setup verification tests
    echo   help      - Show this help message
    echo.
    echo Examples:
    echo   %0 up
    echo   %0 run "grp planning/simple-vehicle-3.py"
    echo   %0 shell
    echo   %0 test
    goto :eof
)

if "%MODE%"=="test" (
    echo.
    echo Running Docker setup verification tests...
    docker-compose --profile test run --rm test
    goto :eof
)

echo Unknown command: %MODE%
echo Use "%0 help" to see available commands
exit /b 1
