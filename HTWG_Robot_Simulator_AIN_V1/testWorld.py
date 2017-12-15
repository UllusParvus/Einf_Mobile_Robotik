from HTWG_Robot_Simulator_AIN_V1.World import *

def buildWorld():
    world = World(30, 20)

    world.addLine(4, 11, 10, 11)
    world.addLine(10, 11, 10, 14)
    world.addLine(10, 14, 15, 14)
    world.addLine(15, 14, 15, 11)
    world.addLine(15, 11, 18, 11)
    world.addLine(18, 11, 18, 16)
    world.addLine(18, 16, 23, 16)
    world.addLine(23, 16, 23, 8)
    world.addLine(23, 8, 16, 8)
    world.addLine(16, 8, 16, 5)
    world.addLine(16, 5, 10, 5)
    world.addLine(10, 5, 4, 11)

    return world