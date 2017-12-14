from HTWG_Robot_Simulator_AIN_V1.World import *

def buildWorld():
    world = World(20, 10)

    world.addLine(3, 5, 10, 5)
    world.addLine(10, 5, 10, 8)
    world.addLine(10, 8, 14, 8)
    world.addLine(14, 8, 14, 2)
    world.addLine(14, 2, 3, 2)
    world.addLine(3, 2, 3, 5)

    return world