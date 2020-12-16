from carla import VehicleLightState
from scenic.domains.driving.roads import ManeuverType

def vehicleLightState_from_maneuverType(maneuverType):
    if maneuverType is ManeuverType.STRAIGHT:
        return VehicleLightState.NONE
    if maneuverType is ManeuverType.LEFT_TURN:
        return VehicleLightState.LeftBlinker
    if maneuverType is ManeuverType.RIGHT_TURN:
        return VehicleLightState.RightBlinker
    if maneuverType is ManeuverType.U_TURN:
        return VehicleLightState.LeftBlinker