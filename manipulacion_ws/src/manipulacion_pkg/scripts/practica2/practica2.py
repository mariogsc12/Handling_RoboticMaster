import argparse
import manipulacion_lib 
import rospy 
from practica1.configuration import NOMBRE_GRIPPER_GAZEBO,NOMBRE_GRIPPER,NOMBRE_OBJETO,NOMBRE_OBSTACULO_GAZEBO
import PyKDL 
import numpy as np
from geometry_msgs.msg import Pose
from practica2.auxiliar import frame2pose

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

# Creación de una instancia para controlar el robot en Gazebo                 
gazebo_robot = manipulacion_lib.GazeboRobot(nombres_articulaciones=[
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
])

# Leer posiciones articulares actuales del robot
posiciones_actuales = gazebo_robot.obtener_posiciones_articulaciones()
print("Posiciones articulares: %s", posiciones_actuales)

# ======================================
#       Pose de agarre deseada
# ======================================
GRASP_POSE = [0.12195302186029333,
              0.04723724582321043,
              0.050673038234956586,
              0.02845181941721514,
              0.1698750166227473,
              0.9634979487183867,
              0.2049504220896671]

x, y, z = GRASP_POSE[0:3]
qx, qy, qz, qw = GRASP_POSE[3:7]

pose_gripper_objeto = PyKDL.Frame(
    PyKDL.Rotation.Quaternion(qx, qy, qz, qw),
    PyKDL.Vector(x, y, z)
)

pose_gripper_world = pose_objeto_world * pose_gripper_objeto
print("Pose gripper con respecto al mundo: ")
print(pose_gripper_world)

# ======================================
#       Detección de obstáculos
# ======================================

ur10 = manipulacion_lib.GazeboRobot()
obstaculo_rojo = manipulacion_lib.Obstaculo('cubo', [0.75,0,0,1,0,0,0], [0.3,1,0.6], 'obstaculo')
suelo = manipulacion_lib.Obstaculo('cubo', [0,0,0,1,0,0,0], [0.01,0.003,-0.01], 'suelo')
obj_x, obj_y, obj_z = pose_objeto_world.p
obj_qx, obj_qy, obj_qz, obj_qw = pose_objeto_world.M.GetQuaternion() 
objeto = manipulacion_lib.Obstaculo('cubo', [obj_x, obj_y, obj_z, obj_qx, obj_qy, obj_qz, obj_qw], [0.141,0.095,0.226], 'objeto')
obstaculos = [obstaculo_rojo, suelo, objeto]

# Detectar colisiones usando la función de la librería
detector_colisiones = manipulacion_lib.DetectorColisiones(
    usa_brazo_robotico = True,
    usa_gripper = True,
    gripper_dimensions=[0.1, 0.1, 0.1],
    obstaculos=obstaculos
)

#Obtener los Límites de las Articulaciones
min_limits = ur10.get_limites_inferiores()
max_limits = ur10.get_limites_superiores()
joint_limits = []
for i in range(len(min_limits)):
    joint_limits.append((min_limits[i], max_limits[i]))

print("Posiciones actuales de las articulaciones:", posiciones_actuales)
hay_colision = detector_colisiones.hay_colision(posiciones_actuales)

# Imprime el resultado de la detección de colisiones
print(f"¿Hay colisión?: {'Sí' if hay_colision else 'No'}")


# ======================================
#       Movimiento del robot
# ======================================
wrapper_cinematica = manipulacion_lib.Cinematica(robot=ur10, 
                                                frame_base='base_link',  
                                                frame_final='tool0')

pose_gripper = frame2pose(pose_gripper_world)

while True:
    valida, posiciones_articulares_deseadas = wrapper_cinematica.calcular_ci(
                posiciones_articulaciones_actuales=ur10.obtener_posiciones_articulaciones(), 
                pose_deseada=pose_gripper)
    
    if valida:
        print("Posiciones articulares deseadas: ", posiciones_articulares_deseadas)
        hay_colision = detector_colisiones.hay_colision(posiciones_articulares_deseadas)
        if not hay_colision:
            print("NO hay colision")
            
            # Configuración del Algoritmo RRT*
            rrt = manipulacion_lib.RRTStarJointSpace(ur10.obtener_posiciones_articulaciones(), posiciones_articulares_deseadas, joint_limits, 0.4, 300,1.0, detectorColisiones)
        
            # Planificación de la Trayectoria
            path = rrt.plan()
            if path:
                print(path)
                ur10.command_path_posicion_articulaciones(path, 0.4, 1.0)
            
            #gazebo_robot.command_posicion_articulaciones([0.5490487683843837, -0.45702793026260485, 1.1701852104646282, -1.1908789269716504, 3.747946643233648, 1.0926705278650608], time_from_start=2)            
            #rospy.sleep(2) 
            break
        else:
            print("Hay colisión, recalculando...")
            rospy.sleep(0.1) 