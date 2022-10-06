#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from reactive_sequence import RSequence

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead
from geometry_msgs.msg import PoseStamped

cube_pose = None
repeat_detection = True
wait_for_completion = False

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")
		aruco_pose_topic = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')

		# tuck the arm
		b1 = tuckarm()

		# look down
		b2 = movehead("down")

		# detect the cube
		b3 = detect_cube("cube detection", aruco_pose_topic)

		# pick up the cube
		b4 = pickup_cube("cube pick up")

		# turn around
		b5 = pt.composites.Selector(
			name="Turn 180 degrees",
			children=[counter(30, "Turned around?"), go("Turn around!", 0, -1)]
		)

		# move forward
		b6 = pt.composites.Selector(
			name="Go to table",
			children=[counter(11, "At other table?"), go("Go to the other table!", 1, 0)]
		)

		# place the cube
		b7 = place_cube("cube placement")

		# check if the cube is placed
		b8 = check_task_complete("check task completed", aruco_pose_topic)

		# become the tree
		tree = RSequence(name="Main sequence", children=[b1, b2, b3, b4, b5, b6, b7, b8])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour treensm___(name)
		while not rospy.is_shutdown(): self.tick_tock(1)


class check_task_complete(pt.behaviour.Behaviour):

	"""
	*explaination about this class*

	check if the cube is placed on the second table:
		- if it is placed: return SUCCESS
		- else: return FAILURE and set repeat_detection = True
	"""

	def __init__(self, name, aruco_pose_topic):
		self.aruco_pose_top = aruco_pose_topic
		self.found = False

		super(check_task_complete, self).__init__()

	def update(self):
		global cube_pose
		global repeat_detection

		if self.found:
			return pt.common.Status.SUCCESS

		cube_pose = None
		try:
			cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 5)
		except:
			cube_pose = None

		if cube_pose is None:
			repeat_detection = True
		else:
			repeat_detection = False
			self.found = True
			return pt.common.Status.SUCCESS

		return pt.common.Status.FAILURE


class pickup_cube(pt.behaviour.Behaviour):

	"""
	pick up the cube from the table
	"""

	def __init__(self, name):
		self.pick_srv = rospy.get_param(rospy.get_name() + "/pick_srv")

		rospy.wait_for_service(self.pick_srv, timeout=30)

		super(pickup_cube, self).__init__("Picking up the cube.")

	def update(self):
		global repeat_detection

		if not repeat_detection:
			return pt.common.Status.SUCCESS

		cube_pick_srv = rospy.ServiceProxy(self.pick_srv, SetBool)
		cube_pick_req = cube_pick_srv(True)

		if cube_pick_req.success:
			rospy.loginfo("Picking up the cube succeedd!")
			return pt.common.Status.SUCCESS
		else:
			rospy.loginfo("Picking up the cube failed!")
			return pt.common.Status.FAILURE


class place_cube(pt.behaviour.Behaviour):

	"""
	pick up the cube from the table
	"""

	def __init__(self, name):
		self.place_srv = rospy.get_param(rospy.get_name() + "/place_srv")
		rospy.wait_for_service(self.place_srv, timeout=30)

		super(place_cube, self).__init__("Placing up the cube.")

	def update(self):
		global repeat_detection

		if not repeat_detection:
			return pt.common.Status.SUCCESS

		place_srv_ = rospy.ServiceProxy(self.place_srv, SetBool)
		place_req = place_srv_()

		if place_req.success:
			rospy.loginfo("Placing the cube succeedd!")
			return pt.common.Status.SUCCESS
		else:
			rospy.loginfo("Placing the cube failed!")
			return pt.common.Status.FAILURE


class detect_cube(pt.behaviour.Behaviour):
	
	"""
	*explaination about this class* 
	"""

	def __init__(self, name, aruco_pose_topic):
		self.aruco_pose_top = aruco_pose_topic
		self.detected = False

		super(detect_cube, self).__init__("Detecting the cube.")

	def update(self):
		global cube_pose
		global repeat_detection
		if not repeat_detection or self.detected:
			return pt.common.Status.SUCCESS
		else:
			cube_pose = None

		try:
			cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 5)
		except:
			cube_pose = None

		if cube_pose is None:
			rospy.loginfo("Cube detection failed.")
			return pt.common.Status.FAILURE
		else:
			rospy.loginfo("Cube detected!")
			self.detected = True
			return pt.common.Status.SUCCESS


class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n
        super(counter, self).__init__(name)


    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising go behaviour.")

        # action space
        #self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_top = "/key_vel"
        #rospy.loginfo(self.cmd_vel_top)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular


        # become a behaviour
        super(go, self).__init__(name)

    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()

        # tell the tree that you're running
        return pt.common.Status.RUNNING


class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING


class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")

    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
