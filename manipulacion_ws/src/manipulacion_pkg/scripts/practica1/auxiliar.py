from manipulacion_lib import SimulacionGripperFlotante
from typing import Optional, List, Union

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