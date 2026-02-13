# Dockerfile for CARLA Planning Client
FROM python:3.8-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    unzip \
    libpng16-16 \
    libjpeg62-turbo \
    libtiff5 \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for better caching
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Install CARLA Python API
# This downloads the CARLA 0.9.13 Python API wheel
# Adjust version as needed for your CARLA server version
RUN pip install --no-cache-dir carla==0.9.13 || \
    (wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz && \
     tar -xzf CARLA_0.9.13.tar.gz && \
     pip install --no-cache-dir PythonAPI/carla/dist/carla-0.9.13-cp38-cp38-linux_x86_64.whl && \
     rm -rf CARLA_0.9.13.tar.gz PythonAPI)

# Copy the entire project
COPY . .

# Create the agents/navigation directory structure to match expected imports
RUN mkdir -p /app/agents/navigation && \
    mkdir -p /app/agents/tools && \
    # Copy navigation files from grp planning to agents/navigation
    cp "/app/grp planning/basic_agent.py" /app/agents/navigation/ 2>/dev/null || true && \
    cp "/app/grp planning/global_route_planner.py" /app/agents/navigation/ 2>/dev/null || true && \
    cp "/app/grp planning/local_planner.py" /app/agents/navigation/ 2>/dev/null || true && \
    cp "/app/grp planning/collision.py" /app/agents/navigation/ 2>/dev/null || true && \
    # Create empty __init__.py files for proper Python modules
    touch /app/agents/__init__.py && \
    touch /app/agents/navigation/__init__.py && \
    touch /app/agents/tools/__init__.py

# Create a minimal misc.py for agents.tools.misc imports
RUN echo "import carla\nimport numpy as np\n\
def get_speed(vehicle):\n\
    vel = vehicle.get_velocity()\n\
    return 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)\n\
\n\
def is_within_distance(target_location, current_location, orientation, max_distance, d_angle_th_up=90, d_angle_th_low=0):\n\
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])\n\
    norm_target = np.linalg.norm(target_vector)\n\
    if norm_target > max_distance:\n\
        return False\n\
    return True\n\
\n\
def get_trafficlight_trigger_location(traffic_light):\n\
    return traffic_light.get_transform().location\n\
\n\
def compute_distance(location_1, location_2):\n\
    x = location_2.x - location_1.x\n\
    y = location_2.y - location_1.y\n\
    z = location_2.z - location_1.z\n\
    return np.sqrt(x*x + y*y + z*z)\n" > /app/agents/tools/misc.py

# Set Python path to include the project root
ENV PYTHONPATH="${PYTHONPATH}:/app"

# Default command (can be overridden)
CMD ["python", "grp planning/simple-vehicle.py"]
