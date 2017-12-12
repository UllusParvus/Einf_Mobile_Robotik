from HTWG_Robot_Simulator_AIN_V1.World import *

def buildWorld():
    world = World(20, 10)

    world.addLine(1, 5, 10, 5)
    world.addLine(10, 5, 10, 1)
    world.addLine(1, 1, 10, 1)

    return world