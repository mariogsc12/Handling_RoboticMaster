from manipulacion_lib import SimulacionGripperFlotante
from typing import Optional, List, Union, Literal
from dataclasses import dataclass
import yaml
from pathlib import Path
from typing import List
import PyKDL 

GRIPPER_MODES = ["open", "close", "custom"]
def set_gripper_pos(gripper: SimulacionGripperFlotante, mode: str = "open", articulaciones:Optional[List[Union[float, int]]]  = None):
    if not mode or mode not in GRIPPER_MODES:
        raise ValueError(f"El modo del gripper seleccionado es invalido: {mode}")
    
    posicion_articulaciones_open = [0.0] * 11
    posicion_articulaciones_close = [0.4, 0.4, 0.4, 0, 0.4, 0.4, 0.4, 0, 0.4, 0.4, 0.4]

    if mode=="open":
        posicion_articulaciones = posicion_articulaciones_open
        print("Abriendo gripper...")
    elif mode=="close":
        posicion_articulaciones = posicion_articulaciones_close
        print("Cerrando gripper...")
    elif mode=="custom":
        if articulaciones is not None:
            posicion_articulaciones = articulaciones
            print(f"Moviendo gripper con la siguiente configuración: {posicion_articulaciones}")
        else:
            raise ValueError("El valor de las articulaciones no ha sido definido")
        
    gripper.set_posicion_articulaciones(posicion_articulaciones)

@dataclass
class GraspInfo:
    dofs: List
    pose: List
    epsilon_quality: float
    volume_quality: float


def get_grasp_list(grasp_poses_path: Path, sort: bool = False, sort_by: Literal["epsilon", "volume", "both"] = "epsilon") -> List[GraspInfo]:
    with open(grasp_poses_path, "r") as f:
        data = yaml.safe_load(f)

    grasp_list: List[GraspInfo] = []

    for grasp in data["grasps"]:
        grasp = GraspInfo(dofs=grasp["dofs"],
                         pose=grasp["pose"],
                         epsilon_quality=grasp['epsilon_quality'],
                         volume_quality=grasp['volume_quality'])
        grasp_list.append(grasp)

    if sort:
        return sort_grasp_list(grasp_list, sort_by)
    else:
        return grasp_list

def sort_grasp_list(grasp_list: List[GraspInfo], sort_by: Union[Literal["epsilon", "volume", "both"], None] = None) -> List[GraspInfo]:
    if sort_by == "epsilon":
        grasp_list.sort(key=lambda x: x.epsilon_quality, reverse=True)
    
    elif sort_by == "volume":
        grasp_list.sort(key=lambda x: x.volume_quality, reverse=True)
    
    elif sort_by == "both":
        grasp_list.sort(key=lambda x: x.epsilon_quality + x.volume_quality, reverse=True)

    return grasp_list


def generate_trajectory(initial_pose, final_pose, num_frames):
    """ Genera una trayectoria de punto a punto utilizando un número de pasos """
    poses = []
    for i in range(num_frames):
      alpha = i / (num_frames - 1)  # Interpolación lineal
      new_position = initial_pose.p * (1 - alpha) + final_pose.p * alpha
      new_pose = PyKDL.Frame(initial_pose.M, new_position)
      poses.append(new_pose)
    return poses