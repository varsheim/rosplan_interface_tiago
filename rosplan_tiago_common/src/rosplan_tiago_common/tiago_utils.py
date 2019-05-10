# ------------------------------------------------------------------
import rospy
import time
# ------------------------------------------------------------------
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg  import PoseStamped
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import quaternion_from_euler
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
# ------------------------------------------------------------------
# action status enum
# http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
ACTION_STATUS_PENDING   = 0
ACTION_STATUS_ACTIVE    = 1
ACTION_STATUS_SUCCEEDED = 3
ACTION_STATUS_ABORTED   = 4
ACTION_STATUS_REJECTED  = 5
# ------------------------------------------------------------------
# action_*** connected with the process of 'homing' the arm of TIAGo
action_current_status = ACTION_STATUS_PENDING
action_current_id = "unknown"
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

def action_status_callback(msg):

    global action_current_status
    global action_current_id

    status_list = GoalStatus()
    status_list = msg.status_list
    # print(str("*** CB --- ACTION --- LENGTH OF A GOAL STATUS LIST IS:" + str(len(status_list))))

    if ( len(status_list) != 1 ):
        # assuming that there will always be 1 goal status
        # print(str("*** CB --- ACTION --- LENGTH OF A GOAL STATUS LIST IS NOT 1 but " + str(len(status_list))))
        return
    
    id_goal_str = status_list[0].goal_id.id
    # print(str("*** CB --- id_goal_str:" + str(id_goal_str) ))

    # tuck_arm action is executed first, robot will wait until its finish
    idx = id_goal_str.find("tuck_arm")
    if ( idx != -1 ):
        # found
        action_current_id = "tuck_arm"
        action_current_status = status_list[0].status
        # print("========== tuck_arm WAS FOUND ======= \n")
        # print("========== tuck_arm STATUS " + str(action_current_status) + " ======= \n")

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

def wait_for_tiago_init():

    global action_current_status
    global action_current_id

    # to check the status of the tuck_arm action
    rospy.Subscriber('/play_motion/status', GoalStatusArray, action_status_callback)

    # for a moment after node launching the time is 'off'
    while ( rospy.get_time() < 1 ):
        pass

    print( "\t ============= TIAGo Initialization... ================ ")

    start_time = rospy.get_time()

    while ( action_current_id == "unknown" ):
        # wait for the first callback...
        print( "\t ===== Waiting for the first action callback... =======")
        rospy.sleep(1.)
        if ( rospy.get_time() - start_time > 5.0 ):
            print( "\t ====== Assuming  TIAGo is already initialized ========")
            return

    while ( action_current_id == "tuck_arm" ):

        # wait until manipulator's "homing" is finished
        if ( action_current_status == 3 ):
            break

        print( "\t ============ Tucking arm... Waiting... ===============")
        rospy.sleep(1.)

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

#def create_move_base_goal(pos_x=0, pos_y=0, pos_z=0, orient_r=0, orient_p=0, orient_y=0):
def create_move_base_goal(pose):

    goal = MoveBaseGoal()

    '''
    goal.target_pose.pose.position.x = pos_x
    goal.target_pose.pose.position.y = pos_y
    goal.target_pose.pose.position.z = pos_z
    '''

    goal.target_pose.pose = pose
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    return goal

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

def move_base_set_goal(pose):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal = create_move_base_goal(pose)
    client.wait_for_server()
    client.send_goal(goal)

    return client

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

def move_base_cancel_goals():

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    client.cancel_all_goals()

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

def play_motion_start_action(act_name):

    client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)

    goal = PlayMotionGoal()

    if ( act_name == "head_down" or act_name == "head_tilt_down" ):
        goal.motion_name = "head_tilt_down"
    elif ( act_name == "head_normal" or act_name == "head_straight" ):
        goal.motion_name = "head_normal"
    elif ( act_name == "look_around" or act_name == "head_tour" ):
        goal.motion_name = "head_pan_tilted"
    elif ( act_name == "give_hand" or act_name == "offer_hand" ):
        goal.motion_name = "reach_floor"
    elif ( act_name == "home" ):
        goal.motion_name = "home"
    else:
        # at the moment no more will be handled but there are plenty of more movements
        print( "\t Wrong play_motion name...")
        return

    goal.skip_planning = False
    goal.priority = 0

    client.wait_for_server()
    client.send_goal(goal)
    
    '''
    action_ok = client.wait_for_result(rospy.Duration(2.5))
    state = client.get_state()

    if action_ok:
        rospy.loginfo("Action finished succesfully with state: " + str(get_status_string(state)))
    else:
        rospy.logwarn("Action failed with state: " + str(get_status_string(state)))
    '''

    return client

# ------------------------------------------------------------------

