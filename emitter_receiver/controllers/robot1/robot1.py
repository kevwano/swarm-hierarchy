"""robot1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import DistanceSensor

import numpy as np



def run_robot(robot):
    #starting opinion
    
    opinion= ["white", "black"][np.random.randint(2)]
    print(opinion)
    samples_collected=0
    opinion_quality=0
    
    #state of driving or turning
    state="driving"
    
    #state of exploring or disseminating
    state2="exploring"
        
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
            
    def update_opinion_quality():
        nonlocal samples_collected
        nonlocal opinion_quality
        nonlocal opinion
         
        #reading the colour detected by the ground sensor
        value=irsensor.getValue()
        print(irsensor.getValue())
        if(value>800):
            if(opinion=="white"):
                opinion_quality=((opinion_quality*samples_collected)+1)/(samples_collected+1)
            else:
                opinion_quality=opinion_quality*samples_collected/(samples_collected+1)
        if(value<400):
            if(opinion=="black"):
                opinion_quality=(opinion_quality*samples_collected+1)/(samples_collected+1)
            else:
                opinion_quality=opinion_quality*samples_collected/(samples_collected+1)
        print("Data collected, new opinion quality:", opinion_quality)
        samples_collected+=1
        #print(opinion_quality)
    
    #broadcast opinion for a time based on opinion quality
    broadcastStartTime="x"
    broadcasting=False
    global broadcast_time
    broadcast_time=1 #write modulation code but this is a placeholder
    def spread_opinion(time):
    
        global broadcast_time
        nonlocal broadcastStartTime
        nonlocal broadcasting
        nonlocal opinion_quality   
           
        if(broadcasting==False):
            print("current opinion quality:", opinion_quality)
            print("start dissemination",robot.getTime())
            broadcastStartTime=robot.getTime()
            broadcasting=True
            broadcast_time=time
        else:
            if(round(robot.getTime()-broadcastStartTime, 3)%0.1<=0.01):    
                print("ping",robot.getTime())        
                emitter.send(opinion)
                
    opinions=[]
    collecting=False
    collectionStartTime="x"
    def update_opinion():
    
        nonlocal opinions
        nonlocal collecting
        nonlocal collectionStartTime
    
        if(collecting==False):
            print("start polling")
            collecting=True
            collectionStartTime=robot.getTime()
            receiver.enable(p=1)
        else:
            #read the message at the front of the queue and then remove it
            if(not receiver.getQueueLength()<1):
                opinions.append(receiver.getString())
                print("received:",receiver.getString())
                receiver.nextPacket()
    
    exploring=False
    exploreStartTime="x"
    def explore_phase():
        
        nonlocal exploring
        nonlocal exploreStartTime
  
        if(exploring==False):
            print("start exploring",robot.getTime())
            exploring=True
            exploreStartTime=robot.getTime()
    
    
    def update_time_functions():
    
        nonlocal startTurnTime
        nonlocal turning
        
        nonlocal startDriveTime
        nonlocal driving
        
        nonlocal broadcastStartTime
        nonlocal broadcasting
        
        nonlocal collecting
        nonlocal collectionStartTime
        nonlocal opinions
        nonlocal opinion
        
        nonlocal exploring
        nonlocal exploreStartTime
        
        nonlocal state
        nonlocal state2
        
        #stop turning after the set time
        if(startTurnTime!="x"):
            if(robot.getTime()-startTurnTime>=1.5):
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                startTurnTime="x"
                #print("stop turning")
                turning=False
                state="driving"
        
        #stop driving after the set time and broadcast opinion                 
        if(startDriveTime!="x"):
            if(robot.getTime()-startDriveTime>=3):
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                startDriveTime="x"
                
                #print("stop turning")
                if(state2=="exploring"):
                    update_opinion_quality()
                
                driving=False    
                state="turning"
        
        #stop broadcasting after the set time
        global broadcast_time

        if(broadcastStartTime!="x"):
            if(robot.getTime()-broadcastStartTime>=broadcast_time):
                print("finish dissemination")
                broadcasting=False
                broadcastStartTime="x"
                
                state2="collecting"
            
        #stop collecting opinions after the set time
        if(collectionStartTime!="x"):
            if(robot.getTime()-collectionStartTime>=7):
                print("finish polling, nothing collected")
                collecting=False
                collectionStartTime="x"
                receiver.disable()
                state2="exploring"
            elif(robot.getTime()-collectionStartTime>=3 and len(opinions)>0):
                print("finish polling, collected:", opinions)
                collecting=False
                collectionStartTime="x"
                receiver.disable()
                opinion=opinions[np.random.randint(len(opinions))]
                opinions=[]
                state2="exploring"
                print("new opinion:", opinion)

        #stop polling robots after the set time
        if(exploreStartTime!="x"):
            if(robot.getTime()-exploreStartTime>=18):
                print("finish exploring ",robot.getTime())
                exploring=False
                exploreStartTime="x"
                state2="broadcasting"

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
    receiver.disable()
    
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
      
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
   
       

        
        #read proximity sensors
        right_front=prox_sensors[0].getValue() > 80
        left_front=prox_sensors[7].getValue() > 80
        
        


        if(state=="driving"):  
            drive(levy_step())
        else:
            random_turn()
            
        if(state2=="exploring"):
            irsensor.enable(p=1)
            explore_phase()
        elif(state2=="broadcasting"):
            irsensor.disable()
            spread_opinion(np.random.randint(10))#write modulation function later
        else:
            update_opinion()
            
    
        if(right_front):
            right_motor.setVelocity(max_speed/2)
            left_motor.setVelocity(-max_speed/2)
        if(left_front):
            right_motor.setVelocity(-max_speed/2)
            left_motor.setVelocity(max_speed/2)
        
        update_time_functions()


# Enter here exit cleanup code.

if __name__ == "__main__":
    
    robot1=Robot()
    run_robot(robot1)
    