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
│   │   ├───tools
|   |   |    ───misc.py
│   │   └───dLite
│   │   │   └───dlite.py
│   │   │    ───PriorityQueueDLite.py
│   │   └───Generate_map.py
│   │    ───simple-vehicle.py
├───examples
```

If it's ensured that all proper dependencies and modules are downloaded, then with `python3 simple-vehicle.py`, a simple test of the start to end of the vehicle agent should run.


Command to run Carla: `.\CarlaUE4.exe -carla-rpc-port=4000 (optional) -windowed -ResY=1000`
Replace with: `./CarlaUE4.sh` on mac
