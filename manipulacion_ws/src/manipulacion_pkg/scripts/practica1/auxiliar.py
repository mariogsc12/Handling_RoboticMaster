from manipulacion_lib import SimulacionGripperFlotante
from typing import Optional, List, Union
from dataclasses import dataclass
import yaml
from pathlib import Path
from typing import List
 
GRIPPER_MODES = ["open", "close", "custom"]
def set_gripper_pos(gripper: SimulacionGripperFlotante, mode: str = "open", articulaciones:Optional[List[Union[float, int]]]  = None):
    if not mode or mode not in GRIPPER_MODES:
        raise ValueError(f"El modo del gripper seleccionado es invalido: {mode}")
    
    posicion_articulaciones_open = [0.0] * 11
    posicion_articulaciones_close = [0.4, 0.4, 0.4, 0, 0.4, 0.4, 0.4, 0, 0.4, 0.4, 0.4]
    breakpoint()
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
    dofs: list
    pose: list
    epsilon_quality: float
    volume_quality: float


def get_grasps_list(grasp_poses_path: Path):
    with open(grasp_poses_path, "r") as f:
        data = yaml.safe_load(f)

    grasp_list: List[GraspInfo] = []

    for grasp in data["grasps"]:
        grasp = GraspInfo(dofs=grasp["dofs"],
                         pose=grasp["pose"],
                         epsilon_quality=grasp['epsilon_quality'],
                         volume_quality=grasp['volume_quality'])
        grasp_list.append(grasp)

    return grasp_list
