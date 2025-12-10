"""robot2 controller."""

# You may need to import some classes of the controller module. Ex:
"""robot1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


def run_robot(robot):

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    emitter=robot.getDevice("emitter")
    
    receiver=robot.getDevice("receiver")
    receiver.enable(p=1)
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
    
        emitter.send("hello robot1")
        
        if(receiver.getQueueLength()>=1):
            receiver.nextPacket()
        

# Enter here exit cleanup code.

if __name__ == "__main__":
    
    robot2=Robot()
    run_robot(robot2)
    