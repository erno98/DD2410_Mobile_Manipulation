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
cube_picked_up = False
cube_placed = False
cube_detected = False
reset = True
reset_completed = [False for _ in range(8)]

def reset():
	global reset
	global reset_completed

	reset = True
	reset_completed = [False for _ in range(8)]

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")
		self.aruco_pose_topic = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')


		# tuck the arm
		b1 = tuckarm()

		# localize the robot in the environment
		b2 = localize("localization")

		# navigate the robot to the goal
		b3 = navigate("navigation", kind="pickup")

		# detect the cube
		b4 = detect_cube("cube detection", aruco_pose_topic)

		# pick up the cube
		b5 = pickup_cube("cube pick up")

		# move head up to localize environment
		b6 = movehead("up")

		# navigate the robot to the goal
		b7 = navigate("navigation", kind="place")

		# detect the cube
		b8 = check_cube_placed("check cube placement", aruco_pose_topic)

		# become the tree
		tree = RSequence(name="Main sequence", children=[b1, b2, b3, b4, b5, b6, b7, b8])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour treensm___(name)
		while not rospy.is_shutdown(): self.tick_tock(1)


class localize(pt.behaviour.Behaviour):

	"""
	handles the localization of the robot
	"""

	def __init__(self, name):
		
		super(localize, self).__init__("Localization started")

	def update(self):
		pass


class navigate(pt.behaviour.Behaviour):

	"""
	handles the navigation of the robot
	"""

	def __init__(self, name, kind):
		
		self.kind = kind
		super(navigate, self).__init__(f"Navigaton started for [{self.kind}]")


		if self.kind == "pickup":
			pass

		elif self.kind == "place":
			pass

	def update(self):
		pass


class pickup_cube(pt.behaviour.Behaviour):

	"""
	pick up the cube from the table
	"""

	def __init__(self, name):
		self.pick_srv = rospy.get_param(rospy.get_name() + "/pick_srv")

		rospy.wait_for_service(self.pick_srv, timeout=30)

		super(pickup_cube, self).__init__("Picking up the cube.")

	def update(self):
		global reset
		global reset_completed
		global cube_picked_up

		if reset and not reset_completed[3]:
			cube_picked_up = False
			reset_completed[3] = True

		if cube_picked_up:
			return pt.common.Status.SUCCESS

		cube_pick_srv = rospy.ServiceProxy(self.pick_srv, SetBool)
		cube_pick_req = cube_pick_srv(True)
		cube_picked_up = True

		if cube_pick_req.success:
			rospy.loginfo("Picking up the cube succeeded!")
			return pt.common.Status.RUNNING
		else:
			rospy.loginfo("Picking up the cube failed!")
			return pt.common.Status.FAILURE



class place_cube(pt.behaviour.Behaviour):

	"""
	place the cube on the table
	"""

	def __init__(self, name):
		self.place_srv = rospy.get_param(rospy.get_name() + "/place_srv")
		rospy.wait_for_service(self.place_srv, timeout=30)
		

		super(place_cube, self).__init__("Placing up the cube.")

	def update(self):
		global reset
		global reset_completed
		global cube_placed

		rospy.sleep(3)

		if reset and not reset_completed[5]:
			cube_placed = False
			reset_completed[5] = True

		if cube_placed:
			return pt.common.Status.SUCCESS

		place_srv_ = rospy.ServiceProxy(self.place_srv, SetBool)
		place_req = place_srv_()
		cube_placed = True

		if place_req.success:
			rospy.loginfo("Placing the cube succeedd!")
			return pt.common.Status.SUCCESS
		else:
			rospy.loginfo("Placing the cube failed!")
			return pt.common.Status.FAILURE


class check_cube_placed(pt.behaviour.Behaviour):
	
	"""
	*explaination about this class* 
	"""

	def __init__(self, name, aruco_pose_topic):
		self.aruco_pose_top = aruco_pose_topic
		self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
		self.detected = False
		self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)


		super(check_cube_placed, self).__init__("Detecting the cube.")

	def update(self):
		global cube_pose
		global cube_picked_up
		global repeat_detection
		global reset
		global reset_completed
		global cube_detected

		if reset and not reset_completed[6]:
			self.detected = False
			reset_completed[6] = True

		cube_detected = self.detected

		if self.detected:
			reset = False
			return pt.common.Status.SUCCESS

		try:
			cube_pose = rospy.wait_for_message(self.aruco_pose_top, PoseStamped, 5)
		except:
			cube_pose = None

		if cube_pose is None:
			rospy.loginfo("Cube not found. Going to table 1.")

			rospy.loginfo("Moving towards another desk")

			# tuck arm
			
			rospy.loginfo(" Tucking the arm...")
			goal = PlayMotionGoal()
			goal.motion_name = 'home'
			goal.skip_planning = True
			self.play_motion_ac.send_goal(goal)
			success_tucking = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

			if success_tucking:
				self.state = 1
			else:
				self.play_motion_ac.cancel_goal()
				rospy.logerr("play_motion failed to tuck arm, reset simulation")
				self.state = -1

			rospy.sleep(1)

			# moving 
			move_msg = Twist()
			move_msg.angular.z = -1

			rate = rospy.Rate(10)
			cnt = 0
			while not rospy.is_shutdown() and cnt < 30:
				self.cmd_vel_pub.publish(move_msg)
				rate.sleep()
				cnt = cnt + 1

			rospy.sleep(5)

			move_msg.linear.x = 1
			move_msg.angular.z = 0
			cnt = 0
			while not rospy.is_shutdown() and cnt < 10:
				self.cmd_vel_pub.publish(move_msg)
				rate.sleep()
				cnt = cnt + 1

			self.cmd_vel_pub.publish(Twist())
			self.state = 3
			rospy.sleep(1)
			reset = True
			reset_completed = [False for _ in range(8)]
			return pt.common.Status.SUCCESS
		else:
			rospy.loginfo("Cube detected!")
			self.detected = True
			cube_detected = self.detected

			return pt.common.Status.SUCCESS

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
		global cube_picked_up
		global repeat_detection
		global reset
		global reset_completed

		if reset and not reset_completed[2]:
			self.detected = False
			cube_pose = None

			reset_completed[2] = True

		if self.detected:
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
		global reset
		global reset_completed

		if reset and not reset_completed[0]:
			self.finished = False
			self.sent_goal = False
			reset_completed[0] = True
		# already tucked the arm		global reset
		global reset_completed
		if self.finished: 
			return pt.common.Status.SUCCESS

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
		global reset
		global reset_completed

		if reset and not reset_completed[1]:
			self.done = False
			self.tried = False
			reset_completed[1] = True

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