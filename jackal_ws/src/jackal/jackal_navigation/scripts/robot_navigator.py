#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
import rclpy
#from tf_transformation import quaternion_from_euler
from waypoint_follower import BasicNavigator, NavigationResult

'''
Basic navigation demo to go to poses.
''' 
def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0

    navigator.setInitialPose(initial_pose)
    #exit(0)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # onavigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    #[('A', [1.984, -0.5875]), ('B', [4.6645, 3.795]), 
    #('C', [8.128499999999999, 17.355]), ('D', [3.5845, 16.178]), 
    #('E', [7.2205, 15.8535]), ('F', [12.6115, 10.519]), 
    #('G', [8.9695, 4.898]), ('H', [3.3785, 8.292]), ('I', [3.9425, 14.646]), 
    #('J', [7.4855, 10.973500000000001])]
    #Finish point to be added
    # order of waypoints A,B,G, F,J,H,I,D,E,C

    # set our demo's goal poses to follow
    #('A', [1.984, -0.5875])
    goal_poses = []
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.984
    goal_pose1.pose.position.y = -0.5875
    goal_pose1.pose.orientation.w = 1.0
    goal_pose1.pose.orientation.z = 0.0
    goal_poses.append(goal_pose1)

    # additional goals can be appended
    #('B', [4.6645, 3.795])
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 4.6645
    goal_pose2.pose.position.y = 3.795
    goal_pose2.pose.orientation.w = 1.0
    goal_pose2.pose.orientation.z = 0.0
    goal_poses.append(goal_pose2)

    #('G', [8.9695, 4.898])
    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 8.9695
    goal_pose3.pose.position.y = 4.898
    goal_pose3.pose.orientation.w = 1.0
    goal_pose3.pose.orientation.z = 0.0
    goal_poses.append(goal_pose3)

     #('F', [12.6115, 10.519])
    goal_pose4 = PoseStamped()
    goal_pose4.header.frame_id = 'map'
    goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose4.pose.position.x = 12.6115
    goal_pose4.pose.position.y = 10.519
    goal_pose4.pose.orientation.w = 1.0
    goal_pose4.pose.orientation.z = 0.0
    goal_poses.append(goal_pose4)

    #('J', [7.4855, 10.973500000000001])
    goal_pose5 = PoseStamped()
    goal_pose5.header.frame_id = 'map'
    goal_pose5.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose5.pose.position.x = 7.4855
    goal_pose5.pose.position.y = 10.973500000000001
    goal_pose5.pose.orientation.w = 1.0
    goal_pose5.pose.orientation.z = 0.0
    goal_poses.append(goal_pose5)

    #('H', [3.3785, 8.292])
    goal_pose6 = PoseStamped()
    goal_pose6.header.frame_id = 'map'
    goal_pose6.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose6.pose.position.x = 3.3785
    goal_pose6.pose.position.y = 8.292
    goal_pose6.pose.orientation.w = 1.0
    goal_pose6.pose.orientation.z = 0.0
    goal_poses.append(goal_pose6)

    #('I', [3.9425, 14.646])
    goal_pose7 = PoseStamped()
    goal_pose7.header.frame_id = 'map'
    goal_pose7.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose7.pose.position.x = 3.9425
    goal_pose7.pose.position.y = 14.646
    goal_pose7.pose.orientation.w = 1.0
    goal_pose7.pose.orientation.z = 0.0
    goal_poses.append(goal_pose7)

    #('D', [3.5845, 16.178])
    goal_pose8 = PoseStamped()
    goal_pose8.header.frame_id = 'map'
    goal_pose8.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose8.pose.position.x = 3.5845
    goal_pose8.pose.position.y = 16.178
    goal_pose8.pose.orientation.w = 1.0
    goal_pose8.pose.orientation.z = 0.0
    goal_poses.append(goal_pose8)

    #('E', [7.2205, 15.8535])
    goal_pose9 = PoseStamped()
    goal_pose9.header.frame_id = 'map'
    goal_pose9.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose9.pose.position.x = 7.2205
    goal_pose9.pose.position.y = 15.8535
    goal_pose9.pose.orientation.w = 1.0
    goal_pose9.pose.orientation.z = 0.0
    goal_poses.append(goal_pose9)

    #('C', [8.128499999999999, 17.355])
    goal_pose10 = PoseStamped()
    goal_pose10.header.frame_id = 'map'
    goal_pose10.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose10.pose.position.x = 8.128499999
    goal_pose10.pose.position.y = 17.355
    goal_pose10.pose.orientation.w = 1.0
    goal_pose10.pose.orientation.z = 0.0
    goal_poses.append(goal_pose10)

    #print(">>>", len(goal_poses))

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose1)

    nav_start = navigator.get_clock().now()

    navigator.followWaypoints(goal_poses)
    #navigator.goToPose(goal_poses[0])
    
    i = 0
    while not navigator.isNavComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            '''
            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelNav()

            # Some follow waypoints request change to demo preemption
            if now - nav_start > Duration(seconds=35.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = 'map'
                goal_pose4.header.stamp = now.to_msg()
                goal_pose4.pose.position.x = -5.0
                goal_pose4.pose.position.y = -4.75
                goal_pose4.pose.orientation.w = 0.707
                goal_pose4.pose.orientation.z = 0.707
                goal_poses = [goal_pose4]
                nav_start = now
                navigator.followWaypoints(goal_poses)
            '''
    '''

    while not navigator.isNavComplete():
        feedback = navigator.getFeedback()
        #print('Executing current waypoint: ' 
        #    #+ str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses))
        #    )
        print('.', end="")
        now = navigator.get_clock().now()

        # Some navigation timeout to demo cancellation
        if now - nav_start > Duration(seconds=600.0):
            print(">>> Cancelling navigation")
            navigator.cancelNav()        
    '''
    
    # Do something depending on the return code
    result = navigator.getResult()
    print("navigator.getResult(): ", navigator.getResult())
    if result == NavigationResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    
    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
