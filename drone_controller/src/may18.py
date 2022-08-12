#!usr/bin/env python

from mav_msgs.msg import CommandTrajectory
import rospy
from csv import reader
from time import sleep
# from subprocess import Popen, call


class waypointsPub(object):
    def __init__(self):
        self.fly_pub = rospy.Publisher('/Navigator/command/waypoint', CommandTrajectory, queue_size=1)
        self.fly_msg = CommandTrajectory()
        self.dt = 1

    def read_csv(self, filename):
        waypoints = []
        with open(filename, 'r') as f:
            waypoints_csv = reader(f)
            for row in waypoints_csv:
                waypoints.append(row)
        return waypoints

    def start_loop(self):
        waypoints = self.read_csv('/home/kjaebye/UoS_swarm_project/src/drone_controller/src/Navigator_traj.csv')

        for _ in range(len(waypoints)):
            if rospy.is_shutdown():
                break
            else:
                self.fly_msg.position.x = float(waypoints[_][0])
                self.fly_msg.position.y = float(waypoints[_][1])
                self.fly_msg.position.z = float(waypoints[_][2])
                self.fly_pub.publish(self.fly_msg)
                # print(waypoints[_])
                rate=rospy.Rate(5)
                rate.sleep()
                # rospy.sleep(self.dt)
                # sleep(self.dt)


if __name__ == '__main__':
    # Popen('roscore',shell=True)
    rospy.init_node('waypoints_pub_node')
    pub = waypointsPub()
    pub.start_loop()
    # call('killall -9 rosmaster',shell=True)
