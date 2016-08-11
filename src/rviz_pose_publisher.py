#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import time
import numpy as np

# Array that keeps information aboutgoals and completed goals
goals = []
completed_goals = []

# Clear markers from the map
def clear_map():
    newMarkerArray = MarkerArray()
    newMarkerArray.markers.append(Marker())
    newMarkerArray.markers[0].header.frame_id = "map"
    newMarkerArray.markers[0].action = 3
    goal_pub.publish(newMarkerArray)

# Update the
def update_markers(goals, complete_goals, publisher):
    newMarkerArray = MarkerArray()
    i = 0
    for j in [1,2]:
        if j == 1:
            tmp_arr = goals
            color = [0.0, 1.0 ,0.0]
        else:
            tmp_arr = complete_goals
            color = [0.0, 0.0 ,1.0]
        for goal in tmp_arr:
            newMarkerArray.markers.append(Marker())
            newMarkerArray.markers[i].header.frame_id = "map"
            newMarkerArray.markers[i].id = i;
            newMarkerArray.markers[i].type = 2
            newMarkerArray.markers[i].scale.x = 0.3
            newMarkerArray.markers[i].scale.z = 0.1
            newMarkerArray.markers[i].scale.y = 0.3
            newMarkerArray.markers[i].color.a = 1.0
            newMarkerArray.markers[i].color.r = color[0]
            newMarkerArray.markers[i].color.g = color[1]
            newMarkerArray.markers[i].color.b = color[2]
            newMarkerArray.markers[i].pose.position.x = goal[0]
            newMarkerArray.markers[i].pose.position.y = goal[1]
            i = i+1
    publisher.publish(newMarkerArray)

def update_pose_list(goals,publisher):
    i = 0
    newPoseArray = PoseArray()
    newPoseArray.header.frame_id = "map"
    if(len(goals)!=0):
        for goal in goals:
            newPoseArray.poses.append(Pose())
            newPoseArray.poses[i].position.x = goal[0]
            newPoseArray.poses[i].position.y = goal[1]
            i = i+1

        publisher.publish(newPoseArray)

def completed_cb(data):
    global goals, completed_goals
    element_num = data.data
    if(element_num < len(goals)):
        completed_goals.append(goals[element_num])
        goals.pop(element_num)
        clear_map()
        update_markers(goals, completed_goals, goal_pub)
        update_pose_list(goals, poseArray_pub)
    else:
        print("OUT OF RANGE")

def click_cb(data):
    global goals
    x = data.point.x
    y = data.point.y
    z = 0
    goals.append([x,y])
    clear_map()
    update_markers(goals, completed_goals, goal_pub)
    update_pose_list(goals, poseArray_pub)

def start():
    global goal_pub, poseArray_pub
    # publishing to "turtle1/cmd_vel" to control
    rospy.init_node('rviz_input_test')
    goal_pub = rospy.Publisher("/goal_markers", MarkerArray, queue_size = 1)
    poseArray_pub = rospy.Publisher("/list_of_goals", PoseArray, queue_size = 1)
    rospy.Subscriber("/clicked_point", PointStamped, click_cb)
    rospy.Subscriber("/goal_completed", Int16, completed_cb)

    time.sleep(.300)
    clear_map()
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
