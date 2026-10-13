"""lfsim — simulador do seguidor de linha (pista real extraída da imagem + núcleo C++ via ctypes)."""
from .core_bridge import Core, Gains, MapEntry, SEG_NAMES, MODE_NAMES
from .track import Track
from .robot import Robot, RobotParams
from .simulate import run, Result
