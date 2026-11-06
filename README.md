# Running Carla on Nautlius
General Nautlius UI guide - https://github.com/oliverc1623/ceriad/blob/main/nautilus-files/README.md <br />
Specific AIEA setup - https://github.com/cruz-control/nautilus/blob/main/sample-nautilus-desktop-gui/README.md


# CARLA_planning
To run A*: python3.8 astar.py

# Carla Motion Planning:
There is a specific file structure to follow to ensure all imports and files run properly as they should. <br />
Noted here are the locations of the files to be replaced (from grp planning directory) and commands to run for testing:

```Carla/PythonAPI <br />
├───carla
│   ├───astar.py
│   ├───agents
│   │   ├───navigation
│   │   │   └───local_planner.py
│   │   │    ───global_route_planner.py
│   │   │    ───basic_agent.py
|   |   |    ───collision.py
|   |   |    ───controller.py
|   |   |    ───d_agent.py
|   |   |    ───Generate_map.py
│   │   ├───tools
|   |   |    ───misc.py
│   │   └───dLite
│   │   │   └───dlite.py
│   │   │    ───PriorityQueueDLite.py
│   │   └───simple-vehicle.py
├───examples
```

If it's ensured that all proper dependencies and modules are downloaded, then with `python3 simple-vehicle.py`, a simple test of the start to end of the vehicle agent should run.


Command to run Carla: `.\CarlaUE4.exe -carla-rpc-port=4000 (optional) -windowed -ResY=1000`
Replace with: `./CarlaUE4.sh` on mac

## Running test scenarios

Each scenario requires a running CARLA server and a CARLA Python API compatible with that server release. Use Python 3.8 as specified by `test_scenarios/.python-version`, and make the matching CARLA Python API available to Python. If you use CARLA's bundled Python API instead of installing it in your environment, set `CARLA_PYTHONAPI` to its `PythonAPI/carla` directory; the helper adds its `agents` package and any bundled CARLA egg to the import path.

From this directory, run a scenario with:

```sh
python test_scenarios/single_lane.py
```

The scripts can also be launched from another working directory. By default, they connect to `localhost`; set `CARLA_HOST` to the server's hostname or IP to connect remotely. Set `CARLA_PORT` to override the port. The default is `9000`, except `blocked_lane.py` and `multiple_obstacles.py`, which default to `4000`. Set `CARLA_TIMEOUT` to change the connection timeout (default `10` seconds). The CARLA installation must also contain the project-compatible `agents.navigation` modules described above.

For macOS or Linux:

```sh
export CARLA_PYTHONAPI="/opt/CARLA/PythonAPI/carla"
export CARLA_HOST="192.168.1.20"
export CARLA_PORT="9000"
python test_scenarios/single_lane.py
```

For PowerShell:

```powershell
$env:CARLA_PYTHONAPI = "C:\CARLA\PythonAPI\carla"
$env:CARLA_HOST = "192.168.1.20"
$env:CARLA_PORT = "9000"
python .\test_scenarios\single_lane.py
```
