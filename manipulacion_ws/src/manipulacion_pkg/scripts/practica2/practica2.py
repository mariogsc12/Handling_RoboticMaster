import argparse
import manipulacion_lib 
import rospy 
from practica1.configuration import NOMBRE_GRIPPER_GAZEBO,NOMBRE_GRIPPER,NOMBRE_OBJETO,NOMBRE_OBSTACULO_GAZEBO
from practica1.auxiliar import get_grasp_list, set_gripper_pos
from practica2.auxiliar import frame2pose, get_joint_limits, get_plan_with_rrt
import PyKDL 
import numpy as np
import sys
from pathlib import Path

# ======================================
#       Argumentos de lanzamiento
# ======================================
parser = argparse.ArgumentParser(description="Control de gripper en simulación gazebo")

parser.add_argument("--trajectory", 
                    type=str, 
                    default="square", 
                    choices=["square", "triangle", "square2"],
                    help="Trayectoria a seguir por el gripper para superar el objeto (Default=square)")

parser.add_argument("--quality_selector", 
                    type=str, 
                    default="volume", 
                    choices=["epsilon", "volume"],
                    help="Indice escogido para la selección del agarre (Default=volume)")

parser.add_argument('--first_grasp', 
                    action='store_true', 
                    help="Flag para usar el primer agarre válido encontrado")

args = parser.parse_args()

print(f"Ejecutando script con los siguientes argumentos:" \
      f"\n\t - trayectoria:      {args.trajectory}",
      f"\n\t - quality_selector: {args.quality_selector}",
      f"\n\t - first_grasp:      {args.first_grasp}\n")

MAX_ITERATIONS = 10

# ======================================
#       Inicialización de nodo ROS
# ======================================
# Inicializar el nodo de ROS si aún no se ha inicializado
if not rospy.get_node_uri():
    rospy.init_node('gripper_flotante_gazebo', anonymous=True, 
                    log_level=rospy.WARN)

# Creación de una instancia para controlar el robot en Gazebo
ur10 = manipulacion_lib.GazeboRobot()

# Creación de una instancia para controlar el gripper en Gazebo
simulacion_gripper = manipulacion_lib.SimulacionGripper(nombre_gripper_gazebo="gripper")

# Se obtiene la pose del objeto con respecto al sistema de referencia global (world)
pose_objeto_world = simulacion_gripper.obtener_pose_objeto(
                    nombre_objeto_gazebo=NOMBRE_OBJETO)

# Creación de una instancia para controlar el robot en Gazebo                 
gazebo_robot = manipulacion_lib.GazeboRobot(nombres_articulaciones=[
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
])

posiciones_articulares_iniciales = [1.25, -0.86, 0.75, -0.8, -0.1, 1.06]
print(f"Moviendo robot a posicion inicial: {posiciones_articulares_iniciales}")
gazebo_robot.command_posicion_articulaciones(posiciones_articulares_iniciales, time_from_start=2)
rospy.sleep(2)
print("="*50)

# ======================================
#       Elección de agarre
# ======================================
print("="*80)
print("Iniciando proceso elección de agarre")

GRASP_POSES_YAML_PATH = Path(__file__).parent.parent / "practica1" / "grasp_poses" / "grasp_poses_robotiq_driller_small.yaml"
print(f"Path al yaml de agarres: {GRASP_POSES_YAML_PATH}")

# Definición obstáculos
obstaculo_rojo = manipulacion_lib.Obstaculo('cubo', [0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], [0.8, 0.2, 0.6], 'obstaculo')
suelo = manipulacion_lib.Obstaculo('cubo', [0,0,0,1,0,0,0], [0.01,0.003,-0.01], 'suelo')
obj_x, obj_y, obj_z = pose_objeto_world.p
obj_qx, obj_qy, obj_qz, obj_qw = pose_objeto_world.M.GetQuaternion() 
objeto = manipulacion_lib.Obstaculo('cubo', [obj_x, obj_y, obj_z, obj_qx, obj_qy, obj_qz, obj_qw], [0.141,0.095,0.226], 'objeto')
obstaculos = [obstaculo_rojo]

used_grasp = None

detector_colisiones = manipulacion_lib.DetectorColisiones(
    usa_brazo_robotico = True,
    usa_gripper = True,
    gripper_dimensions=[0.1, 0.1, 0.1],
    obstaculos=obstaculos
)

wrapper_cinematica = manipulacion_lib.Cinematica(robot=ur10, 
                                                frame_base='base_link',  
                                                frame_final='tool0')

joint_limits = get_joint_limits(ur10)

# 1. Iterar sobre el yaml de posiciones
grasp_id = 1
for grasp in get_grasp_list(GRASP_POSES_YAML_PATH, sort=True, sort_by="epsilon"):
    print("-"*80)
    print(f"Using grasp {grasp_id}")

    # 2. Mover gripper 
    x, y, z = grasp.pose[0:3]
    qx, qy, qz, qw = grasp.pose[3:7]

    pose_gripper_objeto = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(qx, qy, qz, qw),
        PyKDL.Vector(x, y, z)
    )
    pose_gripper_world = pose_objeto_world * pose_gripper_objeto
    pose_gripper = frame2pose(pose_gripper_world)

    iteracion = 0
    while iteracion < MAX_ITERATIONS:
        print(f"--------- iteracion {iteracion+1} ---------")
        valida, posiciones_articulares_deseadas = wrapper_cinematica.calcular_ci(
                    posiciones_articulaciones_actuales=ur10.obtener_posiciones_articulaciones(), 
                    pose_deseada=pose_gripper)
        
        if valida:
            print("Posiciones articulares deseadas: ", posiciones_articulares_deseadas)
            hay_colision = detector_colisiones.hay_colision(posiciones_articulares_deseadas)
            if not hay_colision:
                print("NO hay colision")
                print("Usando RRT para planificar...")
                
                # Configuración del Algoritmo RRT
                rrt = manipulacion_lib.BiRRTJointSpace(start=ur10.obtener_posiciones_articulaciones(), 
                                                        goal=posiciones_articulares_deseadas, 
                                                        joint_limits=joint_limits, 
                                                        collision_detector=detector_colisiones)
            
                # Planificación de la Trayectoria
                planned_path = rrt.plan()
                if planned_path:
                    if used_grasp:
                        if args.quality_selector == "epsilon" and grasp.epsilon_quality > used_grasp.epsilon_quality:
                            used_grasp = grasp
                            print(f"Used grasp actualizado. Nuevo epsilon quality: {used_grasp.epsilon_quality}")
                        elif args.quality_selector == "volume" and grasp.volume_quality > used_grasp.volume_quality:
                            used_grasp = grasp
                            print(f"Used grasp actualizado. Nuevo volume quality: {used_grasp.volume_quality}")
                        else:
                            print(f"Se mantiene el grasp antiguo debido a {args.quality_selector} quality")
                    else:
                        used_grasp = grasp
                        print(f"Used grasp encontrado")
                    break
                else:
                    print("Fallo en la planificación con RRT, skipping...")
                    iteracion += 1
                    continue

            else:
                print("Colisión encontrada, skipping...")
                iteracion += 1
                continue

        # Uso del primer grasp válido (si el flag esta definido)
        if args.first_grasp and (used_grasp and planned_path):
            break
        
        else:
            print("Resultado ci no válido, skipping...")
            iteracion += 1
            continue

    grasp_id += 1
    planned_path = None
    if not used_grasp:
        print("No se ha encontrado ningun agarre válido")
        continue


if not used_grasp:
    print("No se ha encontrado ningun agarre válido en toda la lista de grasps")
    sys.exit(1)
else:
    print("Agarre utilizado:\n" \
        f"\t - pose: {used_grasp.pose}\n" \
        f"\t - dofs: {used_grasp.dofs}\n" \
        f"\t - epsilon_quality: {used_grasp.epsilon_quality}\n" \
        f"\t - volume_quality: {used_grasp.volume_quality}")
    
    print("Fin proceso selección de agarre")
    print("="*80)

rospy.sleep(1)


# ======================================
#       Movimiento del robot
# ======================================
print("="*80)
print("Iniciando proceso de movimiento de objeto")

simulacion_gripper.configurar_gripper()
set_gripper_pos(simulacion_gripper,mode="open")

x, y, z = used_grasp.pose[0:3]
qx, qy, qz, qw = used_grasp.pose[3:7]

rot_grasp = PyKDL.Rotation.Quaternion(qx, qy, qz, qw)
pos_grasp = PyKDL.Vector(x, y, z)
pose_gripper_objeto = PyKDL.Frame(rot_grasp, pos_grasp)

# Calcular pose final en el mundo
pose_final_world = pose_objeto_world * pose_gripper_objeto

# 1. Mover a pose de agarre
print("Planificando movimiento final al objeto...")
plan = get_plan_with_rrt(wrapper_cinematica, detector_colisiones, ur10, pose_final_world)

if plan:
    ur10.command_path_posicion_articulaciones(plan, 0.5, 1.0)
    rospy.sleep(1) 

    # Agarrar objeto
    set_gripper_pos(simulacion_gripper, mode="close")
else:
    print("ERROR: No se pudo encontrar un camino sin colisiones al objeto")

# 2. Iniciar trayectoria

