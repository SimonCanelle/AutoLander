import sys
import socket
import dronekit
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
    return dronekit.connect(droneLink, wait_ready=True)

def getMission(vehicle):
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return cmds

def waitForMissionEnd(vehicle, cmds):
    readyToLand = False
    while(not readyToLand):
        #loiter at mission end
        if vehicle.mode.name == dronekit.VehicleMode('LOITER'):
            #ready to land when num of mission items = next
            #TODO verify waypoint start id (0 or 1)
            if cmds.next == cmds.count-1:
                readyToLand = True



def getTargetPosition():
    #do camera shit
    lzCoord = Coordinate
    #[48.51079433248238, -71.64953214980738, 15]      # Alma
    lzCoord.lat = 48.51079433248238     #TODO get better default values! Takeoff point?
    lzCoord.lon = -71.64953214980738
    lzCoord.alt = 0
    return lzCoord

def executeLanding(vehicle, cmds, lzCoord):
    #do landing
    #TODO verify if mission item start at 0 or 1
    landCmd = dronekit.Command(0, 0, 0, dronekit.mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               dronekit.mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
                               0, dronekit.mavutil.mavlink.PRECISION_LAND_MODE_DISABLED, 0, 0, lzCoord.lat, lzCoord.lon, lzCoord.alt)

    #TODO make sure that MIS_RESTART is properly set
    cmds.clear()
    cmds.add(landCmd)
    cmds.upload()

    #continue mission with landing nav point
    vehicle.mode = dronekit.VehicleMode("AUTO")

if __name__ == "__main__":
    main()