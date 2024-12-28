# Solution to the Rviz Simulation Error

## RViz Simulationo Error

To fix the error related to the missing ```gym``` module and ensure the simulation runs correctly in RViz, the following steps were taken:

**1. nstalling the ```gym``` module:** Since the simulation depended on the ```gym``` pmodule, it was installed using the following Python command:

```
pip3 install gym
```

This command was executed within the Python environment in the workspace directory ```(/sim_ws/src)```, ensuring that the simulation could access the required library.

**2. Installing ```tmux```:** Additionally, ```tmux``` was installed as a useful tool for managing multiple terminal sessions within a single window. This step was also performed within the workspace directory using the following command:

```
sudo apt-get install tmux
```

**3. Building the workspace:** Luego de instalar las dependencias necesarias, el workspace de ROS 2 fue compilado usando el siguiente comando en el directorio raíz del workspace ```(/sim_ws)```:

```
colcon build
```

**4. Launching the simulation:**
```
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
