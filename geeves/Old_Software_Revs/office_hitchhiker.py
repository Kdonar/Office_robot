# office_hitchhiker.py

# This software is for use with a turtlebot robotics platform using ROS.
# Office coordinates are stored in an Amazon AWS DynamoDB database.
# Users ask Amazon Alexa to have Office Hitchhiker show them the office of a particular person.
# The Alexa request must include the First Name and Last Name of the person the user is seeking.
# An Amazon Alexa skill called Office Hitchhiker has been created.
# Software to handle the Amazon Alexa input is hosted by Amazon AWS Lambda.  
# Software on Amazon AWS Lambda is saved as hitchhiker_lambda.py.

# Begin ROS related items
import rospy # ROS python API
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import roslib
# End ROS related items

# Begin Kobuki robot base related items
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, SensorState
from smart_battery_msgs.msg import SmartBatteryStatus
# End Kobuki robot base related items

# Begin Amazon Lambda related items
import boto3 # Amazon Web Services SDK for Python
from boto3.dynamodb.conditions import Key, Attr
from botocore.exceptions import ClientError
# End Amazon Lambda related items

# Begin general purpose related items
from __future__ import print_function # Python 2/3 compatibility
import json # JSON encoder and decoder
import decimal # Capability to use Python decimal function
import time
from tf import transformations
import math
# End general purpose related items

# Helper class to convert a DynamoDB item to JSON.
class DecimalEncoder(json.JSONEncoder):
	def default(self, o):
	  if isinstance(o, decimal.Decimal):
	      if o % 1 > 0:
		return float (o)
	      else:
		return int (o)
	  return super(DecimalEncoder, self).default(o)

class office_hitchhiker():

    # Docking station X & Y coordinates and robot angle
    docking_station_x = 1.88
    docking_station_y = 0.50
    docking_station_angle = 90

    # Kobuki battery max charge level found by running rostopic echo /mobile_base/sensors/core on netbook
    kobuki_base_max_charge = 162

    # Amazon DynamoDB keys required to access database
    ACCESS_KEY='AKIAJE3KLYOD2MKHQFCA'
    SECRET_KEY='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'

    #Default values
    move_base = False  # Converted to True by __init__ and sets the goals using MoveBaseAction
    kobuki_low_battery = False
    netbook_low_battery = False
    netbook_previous_battery_level = 100
    kobuki_previous_battery_level = 1000 # Value > actual maximum, just makes script start assuming battery is ok
    at_docking_station = False
    wait_near_docking_station = 90
    wait_server_to_start = 30
    wait_autodock_server_to_start = 180
    count_locate_office = 0
    wait_db_count = 2
    locate_office_id = 'IDO001A'

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('hitchhiker', anonymous=False)
    
        # Indstruction on what to do if shutdown (Ctrl + C or failure)
        rospy.on_shutdown(self.shutdown)

        # Start move base thread using the action client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Allow time for the action server to start
        self.move_base.wait_for_server(rospy.Duration(self.wait_server_to_start))

        # Monitor Kobuki power and charging status
        rospy.Subscriber("/mobile_base/sensors/core", SensorState, self.SensorPowerEventCallback)

        # Monitor netbook battery power
        rospy.Subscriber("/laptop_charge/", SmartBatteryStatus, self.NetbookPowerEventCallback)

    def escort_user():

        # Determine if robot is at charging station and if it needs charging
        if (self.at_docking_station and self.NeedsCharging()):
            time.sleep(30)
            return True
	elif (self.at_docking_station and not self.NeedsCharging()):
	    return self.CountLocateOfficeRequests()
	elif (self.NeedsCharging() and not self.at_docking_station):
	    return self.ReturnToDockingStation()
	else:
	    return self.WaitForNextRequest()

    def NeedsCharging(self):

	# Determine if Kobuki battery has less than 30% charge.  If so, indicate it needs charging.	
	if ( round(float(data.battery) / float(self.kobuki_base_max_charge) * 100) < 30):
	    
	    if (not self.kobuki_low_battery):
		rospy.loginfo("Kobuki battery is low")
		
	    self.kobuki_low_battery = True

	# Determine if Kobuki battery has more than 50% charge.  If so, indicate the battery is ok.	
	elif ( round(float(data.battery) / float(self.kobuki_base_max_charge) * 100) > 50):

	    if (self.kobuki_low_battery):
		rospy.loginfo("Kobuki battery is fine")
	
	    self.kobuki_low_battery = False

	# Determine if the Netbook battery has less than 30% charge.  If so, indicate it needs charging.

	if (int(data.percentage) < 30):

	    if (not self.netbook_low_battery):
		rospy.loginfo("Netbook battery is low")
	    self.netbook_low_battery = True
	
	# Determine if Netbook has more than 50% charge.  If so, indicate the battery is fine.
	
	elif (int(data.percentage) > 50):

	    if (self.netbook_low_battery):
		rospy.loginfo("Netbook battery is ok")
	    self.netbook_low_battery = False

	if (self.netbook_low_battery or self.kobuki_low_battery):
		return True
	
	return False

    def ReturnToDockingStation(self):

	# Instructs robot to go near to the docking station so it can self dock

	rospy.loginfo("Going near the docking station")

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.heater.stamp = rospy.Time.now()

	# Transform the robot angle to a quaternion for robot orientation

	base_quat = transformations.quaternion_from_euler(0, 0, math.radians(self.docking_station_angle))

	# Set the goal to have the coordinates and angle required to be near the docking station

	goal.target_pose.pose = Pose(Point(float(self.docking_station_x), float(self.docking_station_y), float(0)), Quaternion (float(base_quat[0]), float(base_quat[1]), float(base_quat[2]), float(base_quat[4])))

	# Direct the robot to begin moving
	
	self.move_base.send_goal(goal)

	# Give the robot up to time to get close to the docking station

	success = self.move_base.wait_for_result(rospy.Duration(self.wait_near_docking_station))

	if success:

	    state = self.move_base.get_state()
	
	    if state == GoalStatus.SUCCEEDED:
		rospy.loginfo("The robot made it near the docking station")
		return self.DockWithDockingStation()

	else:

	    self.move_base.cancel_goal()
	    rospy.loginfo("The robot did not make it to the destination near the docking station")
	    return False

    def DockWithDockingStation(self):

        # Automatically docks robot with docking station if the robot is close to the docking station

        self._client = actionlib.SimpleActionClient('/dock_drive_action', AutoDockingAction)
        rospy.loginfo("Waiting for Auto Docking server")
        self._client.wait_for_server()
        rospy.loginfo("auto_docking server found")
        goal = AutoDockingGoal()
        rospy.loginfo("Sending auto docking goal and waiting for results")
    
        success = self._client.wait_for_result(rospy.Duration(self.wait_autodock_server_to_start))

        if success:

	    rospy.loginfo("Auto_docking succeeded")
	    self.at_docking_station = True

        else:

	    rospy.loginfo("Auto_docking failed")
            return False

    def CountLocateOfficeRequests(self):

	# Reference the appropriate AWS service
	dynamodb = boto3.resource(
	         'dynamodb',
	          aws_access_key_id=self.ACCESS_KEY,
	          aws_secret_access_key=self.SECRET_KEY,
	)

	table = dynamodb.Table('IDO_Studio')
	locate_office_field = 'TRUE'

	response = table.query(
	         TableName='IDO_Studio',
	         IndexName='Locate_Office-index',
	         KeyConditionExpression=Key('Locate_Office').eq(locate_office_field)
        )

        locate_office_list = response[u'Items'][0][u'Office']
        self.count_locate_office = response['Count']
        self.locate_office_id = response[u'Items'][0][u'Office']

	# Counting requests, if none present wait and try again.
	if (self.count_locate_office == 0):
	    time.sleep(self.wait_db_count)
	    return True

	# If exactly one request, start navigating toward the office
	elif (self.count_locate_office == 1):
	    	
	    if (self.at_docking_station):
	        return self.BackAwayFromDock()
			    
	    return self.LocateOffice()

	# There is more than one request.  Reset the database table and restart program.	
	else:

	    #### ADD CODE HERE FOR RESETTING THE DATABASE ####
	    #### USE DYNAMO_QUERY7.PY AS GUIDE FOR REMOVING "TRUE" ENTRIES ####
	    #### WILL NEED TO CREATE A WHILE LOOP UNTIL COUNT = 0 ####

    def BackAwayFromDock(self):

	    #### USE DOWENEEDTOBACKUPFROMCHARGINGSTATION IN COFFEEBOT AS A GUIDE ####

    def LocateOffice(self):

	    #### USE DELIVER_COFFEE SECTION IN COFFEEBOT AS A GUIDE ####
	    #### INCORPORATE DYNAMODB FUNCTIONS FOR DETERMINING COORDINATES ####

    def WaitForNextRequest(self):

    	    #### AWAY FROM DOCKING STATION, SO ADD SOME SHORT AMOUNT OF TIME BEFORE RETURNING ####

	    




	

	

	
		
	

	

        

	

            
        

    
