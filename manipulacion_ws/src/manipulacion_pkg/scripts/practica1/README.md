# Práctica 1: Manipulación Robótica

Este directorio contiene los scripts y configuraciones necesarios para la ejecución de la **Práctica 1**, enfocada en la manipulacion de un objeto mediante un gripper flotante en Gazebo.

---

## 📂 Estructura del Proyecto

La organización de los archivos es la siguiente:

* **`practica1.py`**: Es el script principal (punto de entrada). Coordina el flujo de la práctica, desde la carga de poses hasta la ejecución del movimiento en el simulador o robot real.
* **`auxiliar.py`**: Contiene funciones de soporte esenciales. Aquí se gestionan:
    * La lectura y filtrado de archivos YAML.
    * El ordenamiento de agarres por métricas de calidad (`epsilon_quality`, `volume_quality`).
    * Generación de trayectorias y control del gripper.
* **`configuration.py`**: Centraliza los parámetros de configuración, como nombres de los joints, offsets, rutas de archivos y parámetros del entorno para evitar "hardcodear" valores en el código principal.
* **`grasp_poses/`**: Carpeta que almacena los datos técnicos de los agarres.
    * `grasp_poses_robotiq_driller_small.yaml`: Contiene las definiciones de los agarres para la herramienta *driller*, incluyendo sus matrices de transformación y métricas de calidad.

---

## 🛠️ Ejecución
Para ejecutar esta práctica, utiliza el siguiente comando:
```bash
# Terminal 1 -> Lanzar el simulador Gazebo
roslaunch manipulacion_pkg main_simulation.launch

# Terminal 2 -> Ejecutar el script de la práctica
python3 -m src.manipulacion_pkg.scripts.practica1.practica1
```
> Nota: Se usa -m para ejecutar el módulo directamente, asegurando que las importaciones relativas funcionen correctamente.