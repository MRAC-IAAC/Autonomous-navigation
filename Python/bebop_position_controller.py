#!/usr/bin/env python
import rospy
import math
import threading
import sys, select, tty, termios
import way_points
import state_manager
import tf
from  std_msgs.msg import String 
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

last_time = None


class ourWayPoints(object):
    @classmethod
    def InitClass(cls):
        cls.way_points = []


def gh_callback(data):
    
    poseString = data.data.split(",")
    Nextpoint = [float(co) for co in poseString]
    ourWayPoints.way_points.append(Nextpoint)


class NonBlockingConsole(object):
    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def get_data(self):
        try:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                return sys.stdin.read(1)
        except:
            return '[CTRL-C]'
        return False

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

#reads the current state of the drone: Landed, hovering, etc.
copter_state_manager = state_manager.StateManager()
odom_current = Odometry()
bebop_pose = Pose()

def odometry_callback(Odometry):
    global odom_current
    global odom_time
    odom_current = Odometry
    odom_time = rospy.get_time()

def pose_callback(Pose):
    global bebop_pose
    bebop_pose = Pose

#reads input from keyboard to send commands to bebop
def key_reader():
    global is_running
    global copter_state_manager
    global is_keyreader_finished
    global speed_x, speed_y, speed_z
    global pub_takeoff, pub_landing
    global time_current

    message_empty = Empty()
    with NonBlockingConsole() as nbc:
        while is_running:
            c = nbc.get_data()
            time_current = rospy.get_time()
            # x1b is ESC, if key ESC is pressed publish a landing command
            if c == '\x1b':
                is_keyreader_finished = True
                pub_landing.publish(message_empty)
                break
            # if key 's' is pressed send a stop command to bebop
            elif c == 's':
                speed_x = 0.0
                speed_y = 0.0
                speed_z = 0.0
                speed_yaw = 0.0
                print "zero position"
            # if 'l' is pressed send a landing command to bebop
            elif c == 'l':
                copter_state_manager.set_state(state_manager.COPTER_STATE_LANDING, time_current)
                speed_x = 0.0
                speed_y = 0.0
                speed_z = 0.0
                speed_yaw = 0.0
                pub_landing.publish(message_empty)
                print "land"
            # if 'n' is pressed change the state to navigation mode
            elif c == 'n':
                current_state = copter_state_manager.get_state(time_current)
                if current_state == state_manager.COPTER_STATE_HOVERING:
                    print "navigate"
                    copter_state_manager.set_state(state_manager.COPTER_STATE_NAVIGATING, time_current)
                else:
                    print "Not in Hovering State yet"
            # if 't' is pressed send a command to bebop to take off
            elif c == 't':
                copter_state_manager.set_state(state_manager.COPTER_STATE_TAKING_OFF, time_current)
                pub_takeoff.publish(message_empty)
                print "take off"

def get_yaw_from_quaternion(quaternion):
    global Pose
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation_list)
    return yaw


#
ourWayPoints.InitClass()
points = way_points.points


#reads trajectory points from way_points.py and follows the target points, length is the length of error vector
def way_point(length):
    global points
    num_points = len(points)
    #if we are closer to the target point less than 0.2 meters go to next point
    if length <= 0.2 and counter < num_points:
        print "going to next point"
        pub_arrived.publish("**True**")


        #counter = counter + 1
        return [True, max(counter-1,0)]
    else:
        return [False, max(counter-1,0)]
        pub_arrived.publish("**False**")

def odomUpdated(time_current,max_delay):
    global odom_time
    delay = time_current - odom_time
    if delay <= max_delay:
        updated = True
    else:
        updated = False
    # print "delay: ", delay
    return updated

# kp : increase to increase speed of response
# ki : decrease to increase speed of response, affects stability
# kd : predicting error, Its output depends on rate of change of error with respect to time
now = {}
last_time = {'x':0,'y':0,'z':0}
err_sum = {}
last_err = {}
time_change = {}
dErr = {}
output = {}
def pid (name,error,kp,ki,kd,limit):
    global time_current, last_time, last_err, err_sum
    # if "last_time" not in globals():

    now [name] = rospy.get_time()
    if last_time[name] == 0:
        last_time [name] = now[name]
        err_sum [name] = 0
        last_err [name] = 0
    time_change[name] = now[name] - last_time[name]
    err_sum[name] = (error * time_change[name]) + err_sum[name]
    if err_sum [name] > limit:
        err_sum [name] == limit
    elif err_sum [name] < (limit * -1):
        err_sum [name] == (limit * -1)
    if time_change [name] == 0:
        dErr[name] = 0
    else:
        dErr[name] = (error - last_err[name]) / time_change[name]
    output[name] = kp * error + ki * err_sum[name] + kd * dErr[name]

    last_err[name] = error
    last_time[name] = now[name]
    return output[name]

def bebop_position_controller():
    global pub_landing, copter_state_manager
    global is_keyreader_finished
    global current_state, time_current
    global twist_current, odom_current
    global speed_x, speed_y, speed_z

    length = 0
    Pyaw = 0.2
    rate = rospy.Rate(20)

    message_empty = Empty()
    set_pt = []
    graph = []
    while not rospy.is_shutdown() and not is_keyreader_finished:
        print ("ggg")
        points = ourWayPoints.way_points
        ourWayPoints.way_points = []

        time_current = rospy.get_time()
        current_state = copter_state_manager.get_state(time_current)
        if current_state == state_manager.COPTER_STATE_NAVIGATING:
            _set_pt = way_point(length)
            current_index = _set_pt[1]
            if _set_pt[0]:
                set_pt = points[current_index]
            else:
                set_pt = points[current_index]
            current_x = odom_current.pose.pose.position.x
            current_y = odom_current.pose.pose.position.y
            current_z = odom_current.pose.pose.position.z
            bebop_yaw = get_yaw_from_quaternion(odom_current.pose.pose.orientation)
            if bebop_yaw > math.pi:
                bebop_yaw = 2*math.pi - bebop_yaw

            error_x = set_pt[0] - current_x
            error_y = set_pt[1] - current_y
            error_z = set_pt[2] - current_z
            error_yaw = 0 - bebop_yaw

            #calculates the length of error vector (target pose - current pose)
            length = math.sqrt(error_x **2 + error_y **2 + error_z **2)

            speed_x = pid('x',error_x,0.08,0.01,0.05,2)
            speed_y = pid('y',error_y,0.08,0.01,0.05,2)
            speed_z = pid('z',error_z,0.08,0.01,0.05,2)
            speed_yaw = error_yaw * Pyaw
            graph.append([error_x,error_y,error_z])


            print "----------------------------------------"
            # print "target: ",   set_pt
            print "current: ",  round(current_x,3), round(current_y,3), round(current_z,3), round(bebop_yaw,3)
            # print "error: ",    round(error_x,3),   round(error_y,3),   round(error_z,3),   round(error_yaw,3)
            print "speed: ",    round(speed_x,3),   round(speed_y,3),   round(speed_z,3),   round(speed_yaw,3)
            # print "length: ",   length
            # print "----"
            # print "pid_x = ",   pid('x',error_x,0.2,0.2,0.2)
            # print "pid_y = ",   pid('y',error_y,0.2,0.2,0.2)
            # print "pid_z = ",   pid('z',error_z,0.2,0.2,0.2)

            #if last update was within 1 second continue flying, otherwise land
            if odomUpdated(time_current,1) == True:
                twist_current = Twist(Vector3(speed_x,speed_y,speed_z),Vector3(0.0, 0.0, speed_yaw))
                # twist_current = Twist(Vector3(0,0,0),Vector3(0.0, 0.0, 0.0))
            else:
                twist_current = Twist(Vector3(0,0,0),Vector3(0.0, 0.0, 0.0))
                pub_landing.publish(message_empty)
                print "Cannot Update Odometry >> Landing"

            pub.publish(twist_current)
            poseString = [str(current_x),str(current_y),str(current_z)]
            poseString = ','.join(poseString)
            print (poseString)
            pub_pos.publish(poseString)
        rate.sleep()

if __name__ == '__main__':
    try:
        # initialize a node called bebop_position_controller
        rospy.init_node('bebop_position_controller', anonymous=True)
        # subscribes to bebop/odometry
        rospy.Subscriber("bebop/odom", Odometry, odometry_callback)
        # subscribe to a topic "setpoint" using geometry_msgs/poseStamped
        rospy.Subscriber("setpoint", Pose, pose_callback)
        rospy.Subscriber("soft2/NextPoints",String,gh_callback)
        #publishes position message
        pub_pos= rospy.Publisher("soft2/pose", String,queue_size=10)
        pub_pos.publish("0,0,0")
        #publishes arrived message
        pub_arrived =rospy.Publisher("soft2/Arrived", String,queue_size=10)
        # publish cmd_vel
        pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        twist_current = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        # additional options
        pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        pub_landing = rospy.Publisher('/bebop/land', Empty, queue_size=10)
        is_running = True
        thread_reader = threading.Thread(target=key_reader)
        thread_reader.start()
        is_keyreader_finished = False
        pub_pos.publish("0,0,0")
        
        bebop_position_controller()
        is_running = False
        
    except rospy.ROSInterruptException:
        print ("ROS ERROR")
        pass
    
