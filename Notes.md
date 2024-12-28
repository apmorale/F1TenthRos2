# Solucciòn a errores experimentados

## Error de simulaciòn en RViz

Para corregir el error relacionado con el módulo gym y permitir la ejecución de la simulación en RViz, se realizaron los siguientes pasos:

**1. Instalación del módulo ```gym```**: Dado que la simulación dependía del módulo ```gym``` para funcionar correctamente, se instaló este módulo utilizando el siguiente comando de Python:

```
pip3 install gym
```

Este comando se ejecutó dentro del entorno de Python en el directorio del workspace ```(/sim_ws/src)```, asegurando que la simulación pudiera acceder a la librería necesaria.

**2. Instalación de ```tmux```**: Además, se instaló ```tmux```, una herramienta útil para gestionar múltiples terminales en una sola ventana. Este paso se hizo para mejorar la experiencia de trabajo con múltiples procesos dentro de contenedores o simulaciones. La instalación de tmux se realizó también en el directorio del workspace utilizando:

```
sudo apt-get install tmux
```

**Compilación del workspace**: Luego de instalar las dependencias necesarias, el workspace de ROS 2 fue compilado usando el siguiente comando en el directorio raíz del workspace ```(/sim_ws)```:

**Lanzar simulaciòn**
```
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
