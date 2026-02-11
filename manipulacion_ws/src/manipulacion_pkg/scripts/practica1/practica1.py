from practica1.configuration import NOMBRE_GRIPPER_GAZEBO,NOMBRE_GRIPPER,NOMBRE_OBJETO,NOMBRE_OBSTACULO_GAZEBO
import manipulacion_lib 
import rospy 
import PyKDL 
from practica1.auxiliar import set_gripper_pos, generate_trajectory
from practica1.auxiliar import get_grasp_list
from pathlib import Path
import sys
import argparse
import math

# ======================================
#       Argumentos de lanzamiento
# ======================================
parser = argparse.ArgumentParser(description="Control de gripper en simulación gazebo")

parser.add_argument("--trajectory", 
                    type=str, 
                    default="square", 
                    choices=["square", "triangle", "square2"],
                    help="Trayectoria a seguir por el gripper para superar el objeto (Default=square)")

args = parser.parse_args()

print(f"Ejecutando script con los siguientes argumentos:" \
      f"\n\t - trayectoria: {args.trajectory}\n")

# ======================================
#       Inicialización de nodo ROS
# ======================================
# Inicializar el nodo de ROS si aún no se ha inicializado
if not rospy.get_node_uri():
    rospy.init_node('gripper_flotante_gazebo', anonymous=True, 
                    log_level=rospy.WARN)

# Creación de una instancia para controlar un gripper flotante en Gazebo
simulacion_gripper_flotante = manipulacion_lib.SimulacionGripperFlotante(
                                nombre_gripper_gazebo=NOMBRE_GRIPPER_GAZEBO)

# Se obtiene la pose del objeto con respecto al sistema de referencia global (world)
pose_objeto_world = simulacion_gripper_flotante.obtener_pose_objeto(
                    nombre_objeto_gazebo=NOMBRE_OBJETO)

print("Pose objeto con respecto a world")
print(pose_objeto_world)

# ======================================
#       Elección de agarre
# ======================================
print("="*80)
print("Iniciando proceso elección de agarre")

GRASP_POSES_YAML_PATH = Path(__file__).parent / "grasp_poses" / "grasp_poses_robotiq_driller_small.yaml"
print(f"Path al yaml de agarres: {GRASP_POSES_YAML_PATH}")

# Definición de obstáculos
obstaculo_rojo = manipulacion_lib.Obstaculo('cubo', [0.75,0,0,1,0,0,0], [0.3,1,0.6], 'obstaculo')
suelo = manipulacion_lib.Obstaculo('cubo', [0,0,0,1,0,0,0], [0.01,0.003,-0.01], 'suelo')
obj_x, obj_y, obj_z = pose_objeto_world.p
obj_qx, obj_qy, obj_qz, obj_qw = pose_objeto_world.M.GetQuaternion() 
objeto = manipulacion_lib.Obstaculo('cubo', [obj_x, obj_y, obj_z, obj_qx, obj_qy, obj_qz, obj_qw], [0.141,0.095,0.226], 'objeto')
obstaculos = [obstaculo_rojo, suelo, objeto]

used_grasp = None

# 1. Iterar sobre el yaml de posiciones
for grasp in get_grasp_list(GRASP_POSES_YAML_PATH, sort=True, sort_by="epsilon"):

    # 2. Chequear que el agarre es válido (métricas de calidad mayores que 0)
    if not grasp.epsilon_quality > 0 or not grasp.volume_quality > 0:
        continue

    # 3. Mover gripper 
    x, y, z = grasp.pose[0:3]
    qx, qy, qz, qw = grasp.pose[3:7]

    pose_gripper_objeto = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(qx, qy, qz, qw),
        PyKDL.Vector(x, y, z)
    )
    pose_gripper_world = pose_objeto_world * pose_gripper_objeto
    simulacion_gripper_flotante.fijar_pose_gripper(
                    pose_gripper_world=pose_gripper_world)

    # 4. Detectar colisiones
    detector_colisiones = manipulacion_lib.DetectorColisionesGripperFlotante(NOMBRE_GRIPPER, obstaculos)

    rotation = pose_gripper_world.M
    qx, qy, qz, qw = rotation.GetQuaternion()
    pose_gripper = [pose_gripper_world.p.x(), pose_gripper_world.p.y(),
                    pose_gripper_world.p.z(),
                    qx, qy, qz, qw]
    colision = detector_colisiones.hay_colision(pose_gripper)
    print(f"Colisiona en la pose actual: {colision}")

    if not colision:
        used_grasp = grasp
        break

    rospy.sleep(0.5)

    
rospy.sleep(1)

if not used_grasp:
    print("No se ha encontrado ningun agarre válido")
    sys.exit(1)
else:
    print("Agarre utilizado:\n" \
          f"\t - pose: {used_grasp.pose}\n" \
          f"\t - dofs: {used_grasp.dofs}\n" \
          f"\t - epsilon_quality: {used_grasp.epsilon_quality}\n" \
          f"\t - volume_quality: {used_grasp.volume_quality}")
    
    print("Fin proceso selección de agarre")
    print("="*80)
    
# ======================================
#       Agarre y movimiento de objeto
# ======================================
print("Iniciando proceso de movimiento de objeto")

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

SQUARE_TRAJECTORY   = [(0.0,  0.0,   0.5),
                       (1.0,  0.0,   0.0),
                       (0.0,  0.0,  -0.5)]

SQUARE2_TRAJECTORY  = [(0.0,  0.0,  0.1),
                       (0.0,  0.6,  0.0),
                       (1.0,  0.0,  0.0),
                       (0.0, -0.6,  0.0),
                       (0.0,  0.0, -0.1)]

TRIANGLE_TRAJECTORY = [(0.5,  0.0,  1.0),
                       (0.5,  0.0, -1.0)]

if args.trajectory == "square":
    used_trajectory = SQUARE_TRAJECTORY
elif args.trajectory == "triangle":
    used_trajectory = TRIANGLE_TRAJECTORY
elif args.trajectory == "square2":
    used_trajectory = SQUARE2_TRAJECTORY
    radio = 5  

initial_pose = pose_gripper_world
ini_qx, ini_qy, ini_qz, ini_qw = pose_gripper_world.M.GetQuaternion() 

start_of_traj_pose = PyKDL.Frame(pose_gripper_world.M, pose_gripper_world.p)

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
            PyKDL.Rotation.Quaternion(ini_qx, ini_qy, ini_qz, ini_qw),  
            PyKDL.Vector(new_x, new_y, new_z)
        )
        simulacion_gripper_flotante.fijar_pose_gripper(
            pose_gripper_world=new_pose)
        rospy.sleep(0.05)

    initial_pose = next_pose

set_gripper_pos(simulacion_gripper_flotante, mode="open")
new_pose = PyKDL.Frame(
    PyKDL.Rotation.Quaternion(ini_qx, ini_qy, ini_qz, ini_qw),  
    PyKDL.Vector(new_x, new_y, new_z + 1)
)
simulacion_gripper_flotante.fijar_pose_gripper(
    pose_gripper_world=new_pose)