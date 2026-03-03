"""manager controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import DistanceSensor

import numpy as np



def run_robot(robot):
    #starting opinion
    name=robot.getName()
    #state of driving or turning
    state="driving"
    
    cycle=0
    clear_cycles=0
    #parameters for levy flight
    alpha=1.0
    scale=1.5
    def levy_step(alpha=alpha):
        return np.random.pareto(alpha) * scale
    
    #turning a random amount
    startTurnTime="x"
    turning=False
    def random_turn():
        nonlocal startTurnTime
        nonlocal turning
        if(turning==False):
            #print("started turning", robot.getTime())
            speed=np.random.uniform(-max_speed, max_speed)
            left_motor.setVelocity(speed)
            right_motor.setVelocity(-speed)
            startTurnTime=robot.getTime()
            turning=True
    
    #moving for a certain amount of time + obstacle avoidance
    startDriveTime="x"
    driving=False
    def drive(speed):
        
        nonlocal startDriveTime
        nonlocal driving
        
        if(driving==False):
            #print(speed)
            if(speed>max_speed):
                speed=max_speed
            if(speed<1):
                speed=1
            
            left_motor.setVelocity(speed)
            right_motor.setVelocity(speed)
            startDriveTime=robot.getTime()
            driving=True
            print(name,"current queue:",opinions)
    
    
    
    def update_time_functions():
    
        nonlocal startTurnTime
        nonlocal turning
        
        nonlocal startDriveTime
        nonlocal driving
        
        nonlocal cycle
        nonlocal clear_cycles
        
        nonlocal state
        
        #stop turning after the set time
        if(startTurnTime!="x"):
            if(robot.getTime()-startTurnTime>=1.5):
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                startTurnTime="x"
                #print("stop turning")
                turning=False
                state="driving"
                cycle+=1
                if(len(opinions)<1):
                    clear_cycles+=1
                    if(clear_cycles>10):
                        print(name, "process end assumed, time",robot.getTime() )
                        exit()
               
        
        #stop driving after the set time and broadcast opinion                 
        if(startDriveTime!="x"):
            if(robot.getTime()-startDriveTime>=3):
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                startDriveTime="x"
                
                #print("stop turning")
                driving=False    
                state="turning"
        

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
    #setting up sensors
    emitter=robot.getDevice("emitter")
    emitter.setRange(0.3)#set this to 0.3 in the actual simulation
    
    receiver=robot.getDevice("receiver")
    receiver.enable(p=1)
    
    irsensor=robot.getDevice("gs1")
    irsensor.disable()
    
    prox_sensors=[]
    for i in range(8):
        sensor_name = "ps" + str(i)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[i].enable(p=1)
    
    #setting up motors
 
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    max_speed = 6.24
      
    opinions=[]
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
   
       

        
        #read proximity sensors
        right_front=prox_sensors[0].getValue() > 80
        left_front=prox_sensors[7].getValue() > 80
        
        


        if(state=="driving"):  
            drive(levy_step())
            if(right_front):
                right_motor.setVelocity(max_speed/2)
                left_motor.setVelocity(-max_speed/2)
                left_front=False 
            if(left_front):
                right_motor.setVelocity(-max_speed/2)
                left_motor.setVelocity(max_speed/2)
        else:
            random_turn()
            
        
        if(not receiver.getQueueLength()<1):
                opinions.append(receiver.getString())
                #print(name,"collected:",receiver.getString())
                clear_cycles=0
                receiver.nextPacket()   
                
        if(round(robot.getTime(), 3)%0.1<=0.01) and len(opinions)>0:    
                #print("broadcasting, currenttime",robot.getTime())        
                emitter.send(opinions[0])
                #print(name,"emitting",opinions[0])
                opinions.pop(0)
                
                
        
        update_time_functions()



if __name__ == "__main__":
    
    robot1=Robot()
    run_robot(robot1)
    