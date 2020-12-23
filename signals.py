from carla import VehicleLightState
from scenic.domains.driving.roads import ManeuverType
import enum

@enum.unique
class SignalType(enum.Enum):
    """Turn signal at an intersection."""
    OFF = enum.auto()
    LEFT = enum.auto()
    RIGHT = enum.auto()

    @classmethod
    def from_maneuver(cls, maneuver):
        if maneuver.type is ManeuverType.STRAIGHT:
            return SignalType.OFF
        if maneuver.type is ManeuverType.LEFT_TURN:
            return SignalType.LEFT
        if maneuver.type is ManeuverType.RIGHT_TURN:
            return SignalType.RIGHT
        if maneuver.type is ManeuverType.U_TURN:
            return SignalType.LEFT

def vehicleLightState_from_maneuverType(maneuverType):
    if maneuverType is ManeuverType.STRAIGHT:
        return VehicleLightState.NONE
    if maneuverType is ManeuverType.LEFT_TURN:
        return VehicleLightState.LeftBlinker
    if maneuverType is ManeuverType.RIGHT_TURN:
        return VehicleLightState.RightBlinker
    if maneuverType is ManeuverType.U_TURN:
        return VehicleLightState.LeftBlinker

def signalType_from_vehicleLightState(vehicleLightState):
    if vehicleLightState is VehicleLightState.NONE:
        return SignalType.OFF
    if vehicleLightState is VehicleLightState.LeftBlinker:
        return SignalType.LEFT
    if vehicleLightState is VehicleLightState.RightBlinker:
        return SignalType.RIGHT