# Práctica 1: Manipulación Robótica

Este directorio contiene los scripts y configuraciones necesarios para la ejecución de la **Práctica 1**, enfocada en la manipulacion de un objeto mediante un gripper flotante en Gazebo.

---

## 📂 Estructura del Proyecto

La organización de los archivos es la siguiente:

* **`practica1.py`**: Es el script principal (punto de entrada). Coordina el flujo de la práctica, desde la carga de poses hasta la ejecución del movimiento en el simulador o robot real.
* **`auxiliar.py`**: Contiene funciones de soporte esenciales. Aquí se gestionan:
    * función (wrapper) para cálculo del plan usando la pose objetivo y la configuración del robot.

---

## 🛠️ Ejecución
Para ejecutar esta práctica, utiliza el siguiente comando:
```bash
# Terminal 1 -> Lanzar el simulador Gazebo
roslaunch manipulacion_pkg robot_simulation.launch

# Terminal 2 -> Ejecutar el script de la práctica
python3 -m src.manipulacion_pkg.scripts.practica2.practica2
```
> Nota: Se usa -m para ejecutar el módulo directamente, asegurando que las importaciones relativas funcionen correctamente.

> Nota2: Se puede seleccionar el método de selección de agarres mediante el argumento `--grasp_selector` 
`python3 -m src.manipulacion_pkg.scripts.practica2.practica2 --grasp_selector best `

> Nota3: Si se ha seleccionado el método de selección `best`, se puede elegir el indice de calidad mediante el argumento `--quality_selector` 
`python3 -m src.manipulacion_pkg.scripts.practica2.practica2 --quality_selector volume `
```