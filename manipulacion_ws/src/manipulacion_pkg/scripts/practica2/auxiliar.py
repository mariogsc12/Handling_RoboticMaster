from geometry_msgs.msg import Pose
from manipulacion_lib import DetectorColisiones, Cinematica, GazeboRobot, BiRRTJointSpace
from typing import Optional, Union
import PyKDL

def frame2pose(frame):
    rotation = frame.M
    qx, qy, qz, qw = rotation.GetQuaternion()
    pose = Pose()
    pose.position.x = frame.p.x()
    pose.position.y = frame.p.y()
    pose.position.z = frame.p.z()
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

def get_joint_limits(ur10) -> list:
    min_limits = ur10.get_limites_inferiores()
    max_limits = ur10.get_limites_superiores()
    joint_limits = []
    for i in range(len(min_limits)):
        joint_limits.append((min_limits[i], max_limits[i]))

    return joint_limits

def get_plan_with_rrt(
        cinematica: Cinematica, 
        detector_colisiones: DetectorColisiones, 
        gazebo_robot: GazeboRobot, 
        pose_deseada: Union[Pose, PyKDL.Frame], 
        joint_limits: Optional[list] = None,
        max_iteration:int = 20,
        debug:bool = False) -> Optional[list]:
    
    class Logger:
        def __init__(self, debug:bool):
            self.debug = debug
        def print(self, msg: str):
            if self.debug:
                print(msg)

    logger = Logger(debug)

    print("Iniciando planificación con RRT a: ", pose_deseada)
    
    if not joint_limits:
        joint_limits = get_joint_limits(gazebo_robot)

    if isinstance(pose_deseada, PyKDL.Frame):
        pose_deseada = frame2pose(pose_deseada)

    iteracion = 0
    while iteracion < max_iteration:
        valida, posiciones_articulares_deseadas = cinematica.calcular_ci(
                    posiciones_articulaciones_actuales=gazebo_robot.obtener_posiciones_articulaciones(), 
                    pose_deseada=pose_deseada)
        
        if valida:
            hay_colision = detector_colisiones.hay_colision(posiciones_articulares_deseadas)
            if not hay_colision:
                logger.print("NO hay colision")
                logger.print("Usando RRT para planificar...")
                
                # Configuración del Algoritmo RRT
                rrt = BiRRTJointSpace(start=gazebo_robot.obtener_posiciones_articulaciones(), 
                                                        goal=posiciones_articulares_deseadas, 
                                                        joint_limits=joint_limits, 
                                                        collision_detector=detector_colisiones)
            
                # Planificación de la Trayectoria
                planned_path = rrt.plan()
                if planned_path:
                    print(f"Plan creado correctamente")
                    return planned_path
                else:
                    logger.print("Fallo en la planificación con RRT, skipping...")

            else:
                logger.print("Colisión encontrada, skipping...")

        else:
            logger.print("Resultado ci no válido, skipping...")
    
    print("Solución no encontrada")
    return None