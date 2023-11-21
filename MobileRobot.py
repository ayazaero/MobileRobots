import matplotlib.pyplot as plt
import random
import numpy as np


# Shared data storage for position, speed, angular speed, and timestamps
def data_meas(self):
    # get data from vicon and store it for other's to use
    return 0;

class Robot:
    def __init__(self, robot_id,max_speed,max_angular_velocity,max_position,battDrainRate,delta_t):
        self.robot_id = robot_id
        self.max_position=max_position
        self.max_speed = max_speed
        self.max_angular_velocity = max_angular_velocity
        self.battDrainRate=battDrainRate
        self.delta_t=delta_t
        self.plot_initialized = False  # To check if the plot has been initialized
        self.plot_object = None  # To store the plot object
        
    def update_data(self, x, y, velocity, angular_velocity, theta, battery_level, timestamp):
        self.x = x
        self.y=y
        self.velocity = velocity
        self.angular_velocity = angular_velocity
        self.theta = theta
        self.battery_level = battery_level
        self.timestamp = timestamp
        
    def step(self, v, om):
        # Update the robot's pose
        self.x += v * np.cos(self.theta) * self.delta_t      
        self.y += v * np.sin(self.theta) * self.delta_t
        self.theta += om * self.delta_t
        self.battery_level -= v*self.battDrainRate*self.delta_t

        # Apply position constraints
        self.x = min(self.max_position, max(-self.max_position, self.x))
        self.y = min(self.max_position, max(-self.max_position, self.y))
        
    def plot(self,col,xdes,ydes):
        if not self.plot_initialized:
            plt.ion()  # Enable interactive mode if the plot hasn't been initialized yet
            self.plot_object = plt.figure()
            plt.xlabel('X Position')
            plt.ylabel('Y Position')
            plt.title('Jonny Robot')
            plt.grid()
            #plt.xlim([-self.max_position, self.max_position])
            #plt.ylim([-self.max_position, self.max_position])
            plt.scatter(xdes,ydes,color='black',marker='s')
            self.plot_initialized = True

        # Plot the current position and orientation of the robot
        if self.plot_object is not None:
            plt.scatter(self.x, self.y, color = col,s=4,marker='o', label='Robot Position')
            
            
            

# Initialize robot objects RobotID, MaxSpeed,MaxAngularSpeed, MaxPosition
num_robots = 20  # Number of robots you want to create

# Create an empty list to hold robot objects
jonny = []

# Initialize robots in a loop
for i in range(num_robots):
    # Define parameters for each robot
    robot_id = i  # Assign a unique ID to each robot
    max_speed = 10  # Define the maximum speed for each robot (random value between 3 and 7)
    max_angular_velocity = 1  # Define the maximum angular velocity for each robot
    max_position = 50  # Define the maximum position for each robot
    batt_drain_rate = 1  # Define the battery drain rate for each robot
    delta_t = 0.05  # Define the time step
    
    # Create a new robot object and append it to the list
    new_robot = Robot(robot_id, max_speed, max_angular_velocity, max_position, batt_drain_rate, delta_t)
    jonny.append(new_robot)



#jonny1=Robot(0,5,1,100,1,0.05)
#jonny2=Robot(1,5,2,8,1,0.05)

# Update robot data(x,y,v,w,tht,bat,time)
for i in range(num_robots):
    jonny[i].update_data(random.uniform(-10, 10), random.uniform(-10, 10), 0, 0, random.uniform(-2*np.pi, 2*np.pi), 98, 0)
    
xdes=np.zeros(num_robots);
ydes=np.zeros(num_robots);

err=0.1*np.ones(num_robots);
Kpv=0.5;
Kiv=0.0001;
Kdv=0.3;

Kpw=2;
Kiw=0.0001;
Kdw=0.03;

i=1;
n_iter=400
ierr=np.zeros(num_robots)
ithterr=np.zeros(num_robots)
err_old=np.zeros(num_robots)
thterr_old=np.zeros(num_robots)

thterr=np.zeros(num_robots)

X_old=np.zeros((2,num_robots));
X_new=np.zeros((2,num_robots))

xt=np.zeros((n_iter,num_robots))
yt=np.zeros((n_iter,num_robots))
thtt=np.zeros((n_iter,num_robots))
t=np.zeros((n_iter,num_robots))

for j in range(num_robots):
    xt[0,j]=jonny[j].x
    yt[0,j]=jonny[j].y
    thtt[0,j]=jonny[j].theta
    t[0,j]=0


while np.linalg.norm(err)>0.05:
    
    for j in range(num_robots-1):
        xdes[j+1]=jonny[j].x
        ydes[j+1]=jonny[j].y
    
    for j in range(num_robots):
        err[j]=np.sqrt(np.power(xdes[j]-jonny[j].x,2)+np.power(ydes[j]-jonny[j].y,2))
        X_old[0,j]=xdes[j]-jonny[j].x
        X_old[1,j]=ydes[j]-jonny[j].y
        
        X_new[0,j]=(np.cos(jonny[j].theta)*X_old[0,j])+(np.sin(jonny[j].theta)*X_old[1,j])
        X_new[1,j]=-(np.sin(jonny[j].theta)*X_old[0,j])+(np.cos(jonny[j].theta)*X_old[1,j])
   
        thterr[j]=np.arctan2(X_new[1,j],X_new[0,j])
        
        
    ierr += err
    ithterr += thterr
    
    derr=err-err_old
    dthterr=thterr-thterr_old
    
    vel=np.maximum(np.zeros(num_robots),(Kpv*err)+(Kdv*derr)+(Kiv*ierr));
    omega=(Kpw*thterr)+(Kdw*dthterr)+(Kiw*ithterr);

    for j in range(num_robots):
        jonny[j].step(vel[j],omega[j])
        xt[i,j]=jonny[j].x
        yt[i,j]=jonny[j].y
        thtt[i,j]=jonny[j].theta
        t[i,j]=t[i-1,j]+jonny[j].delta_t
        
    
    err_old=err;
    thterr_old=thterr;
    
    '''
    for j in range(num_robots):
        color = (j/num_robots, j/num_robots, j/num_robots)
        jonny[j].plot(color,xdes[j],ydes[j])
        '''
    
    
    if i==n_iter-1:
        print('Break')
        break
    i += 1

   
for j in range(num_robots):
    plt.scatter(xt[0,j],yt[0,j],marker='s')
    plt.scatter(xt[:,j],yt[:,j],s=2,label=f'Jonny {j + 1}')

#plt.legend()
plt.grid('True')
plt.show()        
