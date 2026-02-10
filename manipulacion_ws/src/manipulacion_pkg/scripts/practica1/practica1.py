from practica1.configuration import NOMBRE_GRIPPER_GAZEBO,NOMBRE_GRIPPER,NOMBRE_OBJETO,NOMBRE_OBSTACULO_GAZEBO
import manipulacion_lib 
import rospy 
import PyKDL 
from practica1.auxiliar import set_gripper_pos, generate_trajectory
from practica1.auxiliar import get_grasps_list
from pathlib import Path
import sys
import argparse

# ======================================
#       Argumentos de lanzamiento
# ======================================
parser = argparse.ArgumentParser(description="Control de gripper en simulación gazebo")

parser.add_argument("--grasp_id", 
                    type=int, 
                    default=0, 
                    help="Id del agarre a usar del yaml de agarres válidos (Default=0)")

parser.add_argument("--trajectory", 
                    type=str, 
                    default="square", 
                    choices=["square", "triangle"],
                    help="Trayectoria a seguir por el gripper para superar el objeto (Default=square)")

parser.add_argument("--check_collisions", 
                    action="store_true", 
                    help="Comprueba las colisiones de los agarres válidos")

args = parser.parse_args()

print(f"Ejecutando script con los siguientes argumentos:" \
      f"\t - grasp id: {args.grasp_id}" \
      f"\t - trayectoria: {args.trajectory}" \
      f"\t - chequeo de colisiones: {args.check_collisions}")

# ======================================
#       Ejecución del programa
# ======================================
# Inicializar el nodo de ROS si aún no se ha inicializado
if not rospy.get_node_uri():
    rospy.init_node('gripper_flotante_gazebo', anonymous=True, 
                    log_level=rospy.WARN)

# Creación de una instancia para controlar un gripper flotante en Gazebo
simulacion_gripper_flotante = manipulacion_lib.SimulacionGripperFlotante(
                                nombre_gripper_gazebo=NOMBRE_GRIPPER_GAZEBO)

# Esta instancia permite interactuar con un gripper en la simulación de Gazebo,
# especificando su nombre
# Independientemente del gripper asignado para la práctica, se utiliza el 
# nombre "gripper" para referenciarlo en la simulación de Gazebo.


# Obtener la pose (posición y orientación) de un objeto específico en el Gazebo
# Se obtiene la pose del objeto con respecto al sistema de referencia global (world)
pose_objeto_world = simulacion_gripper_flotante.obtener_pose_objeto(
                    nombre_objeto_gazebo=NOMBRE_OBJETO)

print("Pose objeto con respecto a world")
print(pose_objeto_world)

# Fijamos una pose relativa del gripper con respecto al objeto
# Pose del gripper con respecto al objeto, especificando una
# traslación de 0.2m en X y 0.1m en Z
pose_gripper_objeto = PyKDL.Frame(PyKDL.Rotation.Quaternion(1.0, 0, 0.0, 0.0), 
                                  PyKDL.Vector(0.2, 0, 0.1))

print("Pose gripper con respecto a objeto")
print(pose_gripper_objeto)

# Calcular la pose del gripper en el sistema de referencia global a partir 
# de su pose relativa al objeto
# Se realiza una transformación de coordenadas para obtener la pose del 
# gripper en el sistema global
pose_gripper_world = pose_objeto_world * pose_gripper_objeto
print("Pose gripper con respecto a world: ")
print( pose_gripper_world)



GRASP_POSES_YAML_PATH = Path(__file__).parent / "grasp_poses" / "grasp_poses_robotiq_driller_small.yaml"

# Definición de obstáculos
obstaculo_rojo = manipulacion_lib.Obstaculo('cubo', [0.75,0,0,1,0,0,0], [0.3,1,0.6], 'obstaculo')
suelo = manipulacion_lib.Obstaculo('cubo', [0,0,0,1,0,0,0], [0.01,0.003,-0.01], 'suelo')
obj_x, obj_y, obj_z = pose_objeto_world.p
obj_qx, obj_qy, obj_qz, obj_qw = pose_objeto_world.M.GetQuaternion() 
objeto = manipulacion_lib.Obstaculo('cubo', [obj_x, obj_y, obj_z, obj_qx, obj_qy, obj_qz, obj_qw], [0.141,0.095,0.226], 'objeto')
obstaculos = [obstaculo_rojo, suelo, objeto]

best_grasp = None
first_best_grasp = True

# 1. Iterar sobre el yaml de posiciones
valid_grasps = []
for grasp in get_grasps_list(GRASP_POSES_YAML_PATH):
    if args.check_collisions:
        # 2. Mover gripper 
        x, y, z = grasp.pose[0:3]
        qx, qy, qz, qw = grasp.pose[3:7]

        pose_gripper_objeto = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(qx, qy, qz, qw),
            PyKDL.Vector(x, y, z)
        )
        pose_gripper_world = pose_objeto_world * pose_gripper_objeto
        simulacion_gripper_flotante.fijar_pose_gripper(
                        pose_gripper_world=pose_gripper_world)
    
        # 3. Detectar colisiones
        detector_colisiones = manipulacion_lib.DetectorColisionesGripperFlotante(NOMBRE_GRIPPER, obstaculos)

        rotation = pose_gripper_world.M
        qx, qy, qz, qw = rotation.GetQuaternion()
        pose_gripper = [pose_gripper_world.p.x(), pose_gripper_world.p.y(),
                        pose_gripper_world.p.z(),
                        qx, qy, qz, qw]
        colision = detector_colisiones.hay_colision(pose_gripper)
        print(f"Colisiona en la pose actual: {colision}")

        if not colision:
            valid_grasps.append(grasp)
        rospy.sleep(0.5)

    else:
        valid_grasps.append(grasp)
    
rospy.sleep(1)

if not valid_grasps:
    print("No se ha encontrado ningun agarre válido")
    sys.exit(1)
else:
    print(f"Número de agarres válidos: {len(valid_grasps)}")
    used_grasp = valid_grasps[args.grasp_id]

    print("Agarre utilizado:\n" \
          f"\t - pose: {used_grasp.pose}\n" \
          f"\t - dofs: {used_grasp.dofs}\n" \
          f"\t - epsilon_quality: {used_grasp.epsilon_quality}\n" \
          f"\t - volume_quality: {used_grasp.volume_quality}")
    

simulacion_gripper_flotante.configurar_gripper()


set_gripper_pos(simulacion_gripper_flotante, mode="open")

x, y, z = used_grasp.pose[0:3]
qx, qy, qz, qw = used_grasp.pose[3:7]

pose_gripper_objeto = PyKDL.Frame(
    PyKDL.Rotation.Quaternion(qx, qy, qz, qw),
    PyKDL.Vector(x, y, z)
)
pose_gripper_world = pose_objeto_world * pose_gripper_objeto
simulacion_gripper_flotante.fijar_pose_gripper(
                pose_gripper_world=pose_gripper_world)
    
set_gripper_pos(simulacion_gripper_flotante, mode="custom", articulaciones=used_grasp.dofs)
rospy.sleep(1)

SQUARE_TRAJECTORY = [(0,0,0.5),
                    (1,0,0),
                    (0,0,-0.5)]

TRIANGLE_TRAJECTORY = [(0.5,0,1),
                       (0.5,0,-1)]

if args.trajectory == "square":
    used_trajectory = SQUARE_TRAJECTORY
elif args.trajectory == "triangle":
    used_trajectory = TRIANGLE_TRAJECTORY

initial_pose = pose_gripper_world
ini_qx, ini_qy, ini_qz, ini_qw = pose_gripper_world.M.GetQuaternion() 

for increment in used_trajectory:
    fin_x, fin_y, fin_z = tuple(x + y for x, y in zip(initial_pose.p, increment))
    next_pose = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(ini_qx, ini_qy, ini_qz, ini_qw),  # Misma orientación
        PyKDL.Vector(fin_x, fin_y, fin_z)
    )
    print(f"Moviendo gripper a: {next_pose.p}")

    for pose in generate_trajectory(initial_pose, next_pose, num_frames=200):
        new_x, new_y, new_z = pose.p
        new_pose = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(ini_qx, ini_qy, ini_qz, ini_qw),  # Misma orientación
            PyKDL.Vector(new_x, new_y, new_z)
        )
        simulacion_gripper_flotante.fijar_pose_gripper(
            pose_gripper_world=new_pose)
        rospy.sleep(0.05)

    # Update initial_pose
    initial_pose = next_pose

set_gripper_pos(simulacion_gripper_flotante, mode="open")
new_pose = PyKDL.Frame(
    PyKDL.Rotation.Quaternion(ini_qx, ini_qy, ini_qz, ini_qw),  # Misma orientación
    PyKDL.Vector(new_x, new_y, new_z + 1)
)
simulacion_gripper_flotante.fijar_pose_gripper(
    pose_gripper_world=new_pose)