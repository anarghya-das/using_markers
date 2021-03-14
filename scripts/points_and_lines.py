#!/usr/bin/env python
import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

if __name__ == '__main__':
    rospy.init_node("points_and_lines")
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)
    r = rospy.Rate(30)

    f = float(0)
    while not rospy.is_shutdown():
        points, line_strip, line_list = Marker(), Marker(), Marker()

        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame"
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = rospy.Time.now()

        points.ns = line_strip.ns = line_list.ns = "points_and_lines"
        points.action = line_list.action = line_list.action = Marker.ADD

        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0

        points.id = 0
        line_strip.id = 1
        line_list.id = 2

        points.type = Marker.POINTS
        line_strip.type = Marker.LINE_STRIP
        line_list.type = Marker.LINE_LIST

        # POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2
        points.scale.y = 0.2

        # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1
        line_list.scale.x = 0.1

        # Points are green
        points.color.g = 1.0
        points.color.a = 1.0

        # Line strip is blue
        line_strip.color.b = 1.0
        line_strip.color.a = 1.0

        # Line list is red
        line_list.color.r = 1.0
        line_list.color.a = 1.0

        for i in range(100):
            y = 5 * math.sin(f+i/float(100)*2*math.pi)
            z = 5 * math.cos(f + i / float(100) * 2 * math.pi)
            p = Point()
            p.x = i - 50
            p.y = y
            p.z = z

            points.points.append(p)
            line_strip.points.append(p)

            line_list.points.append(p)
            p.z += 1
            line_list.points.append(p)

        marker_pub.publish(points)
        marker_pub.publish(line_strip)
        marker_pub.publish(line_list)

        r.sleep()
        f += 0.04
