import sys
import socket
from dronekit import Command, connect, VehicleMode, mavutil
import time

droneLink = '127.0.0.1:14550'
class Coordinate:
    lat: float
    lon: float
    alt: float

def main():
    vehicle = connectToDrone()
    cmds = getMission(vehicle)
    waitForMissionEnd(vehicle, cmds)
    lzCoord = getTargetPosition()
    executeLanding(vehicle, cmds, lzCoord)

def connectToDrone():
    return connect(droneLink, wait_ready=True)

def getMission(vehicle):
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return cmds

def waitForMissionEnd(vehicle, cmds):
    readyToLand = False
    while(not readyToLand):
        #loiter at mission end
        if vehicle.mode.name == VehicleMode('LOITER'):
            #ready to land when num of mission items = next
            #TODO verify waypoint start id (0 or 1)
            if cmds.next == cmds.count-1:
                readyToLand = True

def getTargetPosition():
    #TODO 1. get drone gps coordinate
    #TODO 2. get image
    #TODO 3. find target
    #TODO 3.1 find blue
    #TODO 3.2 find target area
    #TODO 3.3 find target center
    #TODO 4 calculate target gps coord



    lzCoord = Coordinate
    #[48.51079433248238, -71.64953214980738, 0]      # Alma
    lzCoord.lat = 48.51079433248238     #TODO get better default values! Takeoff point?
    lzCoord.lon = -71.64953214980738
    lzCoord.alt = 15
    return lzCoord

def executeLanding(vehicle, cmds, lzCoord):
    landCmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
                               0, mavutil.mavlink.PRECISION_LAND_MODE_DISABLED, 0, 0, lzCoord.lat, lzCoord.lon, lzCoord.alt)

    #TODO check if its better to
    # 1. add a waypoint at the end and continue mission
    # 2. clear and create new mission with one point : land
    #TODO make sure that MIS_RESTART is properly set to restart mission from start
    cmds.clear()
    cmds.add(landCmd)
    cmds.upload()

    #continue mission with landing nav point
    vehicle.mode = VehicleMode("AUTO")

if __name__ == "__main__":
    main()