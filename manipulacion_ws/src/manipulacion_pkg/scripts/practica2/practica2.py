import argparse
import manipulacion_lib 
import rospy 
from practica1.configuration import NOMBRE_GRIPPER_GAZEBO,NOMBRE_GRIPPER,NOMBRE_OBJETO,NOMBRE_OBSTACULO_GAZEBO
from practica1.auxiliar import get_grasp_list, set_gripper_pos, generate_trajectory, GraspInfo
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
                    help="Trayectoria a seguir por el gripper para superar el objeto (Default='square')")

parser.add_argument("--quality_selector", 
                    type=str, 
                    default="volume", 
                    choices=["epsilon", "volume"],
                    help="Indice escogido para la selección del agarre (Default='volume')")

parser.add_argument("--grasp_selector", 
                    type=str, 
                    default="fixed", 
                    choices=["fixed", "first", "best"],
                    help="Modo de selección del agarre (Default='fixed')")

args = parser.parse_args()

print(f"Ejecutando script con los siguientes argumentos:" \
      f"\n\t - trayectoria:      {args.trajectory}",
      f"\n\t - quality_selector: {args.quality_selector}",
      f"\n\t - grasp_selector:   {args.grasp_selector}\n")

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
simulacion_gripper.configurar_gripper()

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

posiciones_articulares_iniciales = [1.25, -1.5, 2, 0, 1, 0]
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
if args.grasp_selector == "fixed":
    used_grasp = GraspInfo(pose=[-0.09872406361048086, 0.04486521939505443, 0.04072782742149465, -0.003734014932421467, 0.017646499563194584, 0.9887218668400154, 0.14867322629863688],
                            dofs=[0.47335584787458446, 0.3800000000000001, -0.5833558478745844, 0.06973696902984415, 0.7758558478745846, 0.0, -0.7758558478745846, -0.06973696902984415, 0.8333558478745846, 0.0, -0.5733558478745844],     
                            epsilon_quality=0.05960620872025142,
                            volume_quality=0
                            )
else:
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
                            
                            # Uso del primer grasp válido (si el flag esta definido)
                            if args.grasp_selector == "first":
                                break
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

    x, y, z = used_grasp.pose[0:3]
    qx, qy, qz, qw = used_grasp.pose[3:7]
    
    pose_gripper_objeto = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(qx, qy, qz, qw),
        PyKDL.Vector(x, y, z)
    )
    
rospy.sleep(1)


# ======================================
#       Movimiento del robot (SEGURO)
# ======================================
print("="*80)
print("Iniciando aproximación segura...")

# 1. Aseguramos que el gripper esté abierto al máximo
set_gripper_pos(simulacion_gripper, mode="open")
rospy.sleep(1.0)

# 2. Calculamos la pose de AGARRE final
pose_final_world = pose_objeto_world * pose_gripper_objeto

# 3. Calculamos el PRE-AGARRE desplazado en el eje de aproximación (Z local)
distancia_aproximacion = 0.15 
pose_pre_world = pose_final_world * PyKDL.Frame(PyKDL.Vector(0, 0, distancia_aproximacion))

# --- PASO A: IR AL PRE-AGARRE (RRT) ---
# El RRT buscará un camino que no choque con nada para ponerse "frente" al objeto
print("Moviendo a posición de espera (Pre-agarre)...")
plan_pre = get_plan_with_rrt(wrapper_cinematica, detector_colisiones, ur10, pose_pre_world)

if plan_pre:
    ur10.command_path_posicion_articulaciones(plan_pre, 0.5, 1.0)
    rospy.sleep(len(plan_pre) * 0.5 + 2.0)
else:
    print("ERROR: RRT no pudo encontrar camino al pre-agarre")
    sys.exit(1)

# --- PASO B: ENTRAR AL OBJETO (APROXIMACIÓN FINAL) ---
print("Entrando al objeto para el agarre...")

# IMPORTANTE: Para que no de error de colisión al acercarse al taladro
detector_colisiones.obstaculos = [obstaculo_rojo] # Quitamos el objeto de la lista

valida, joints_final = wrapper_cinematica.calcular_ci(
    ur10.obtener_posiciones_articulaciones(), 
    frame2pose(pose_final_world)
)

if valida:
    # Entramos LENTO (3 segundos) para que la interpolación sea casi lineal
    # y los dedos no golpeen el objeto.
    ur10.command_posicion_articulaciones(joints_final, time_from_start=3.0)
    rospy.sleep(1) 
    
    print("¡Objeto alcanzado! Cerrando gripper...")
    set_gripper_pos(simulacion_gripper, mode="close")
    rospy.sleep(1)
else:
    print("ERROR: CI no válida para el punto de contacto")

# --- 4. INICIAR TRAYECTORIA DE DESPLAZAMIENTO ---
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

initial_pose = pose_final_world
ini_qx, ini_qy, ini_qz, ini_qw = initial_pose.M.GetQuaternion() 

for increment in used_trajectory:
    fin_x, fin_y, fin_z = tuple(x + y for x, y in zip(initial_pose.p, increment))

    next_pose = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(ini_qx, ini_qy, ini_qz, ini_qw),  # Misma orientación
        PyKDL.Vector(fin_x, fin_y, fin_z)
    )
    print(f"Moviendo gripper a: {next_pose.p}")

    for pose in generate_trajectory(initial_pose, next_pose):
        new_x, new_y, new_z = pose.p
        new_pose = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(ini_qx, ini_qy, ini_qz, ini_qw),  
            PyKDL.Vector(new_x, new_y, new_z)
        )
        
        print("Planificando siguiente movimiento...")
        plan = get_plan_with_rrt(wrapper_cinematica, detector_colisiones, ur10, new_pose)

        if plan:
            print("Ejecutando movimiento")
            ur10.command_path_posicion_articulaciones(plan, 0.5, 1.0)
            rospy.sleep(1) 

        else:
            print("ERROR: No se pudo encontrar un camino sin colisiones al objeto")
            initial_pose = next_pose

set_gripper_pos(simulacion_gripper, mode="open")