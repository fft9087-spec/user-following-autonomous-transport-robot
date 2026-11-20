#!/usr/bin/env python3
"""Run a sequential Nav2 delivery mission from a CSV waypoint list."""

import argparse
import csv
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def load_waypoints(path):
    waypoints = []
    with open(path, newline='', encoding='utf-8') as file:
        for row in csv.DictReader(line for line in file if not line.lstrip().startswith('#')):
            waypoints.append((row['name'], float(row['x']), float(row['y']), float(row['yaw_radians'])))
    if not waypoints:
        raise ValueError('Waypoint file contains no destinations.')
    return waypoints


def main():
    parser = argparse.ArgumentParser(description='Run a sequential Nav2 delivery mission.')
    parser.add_argument('--waypoints', required=True, help='CSV file with name,x,y,yaw_radians columns')
    args = parser.parse_args()

    try:
        waypoints = load_waypoints(args.waypoints)
    except (OSError, ValueError, KeyError) as error:
        print(f'Waypoint file error: {error}', file=sys.stderr)
        return 2

    rclpy.init()
    navigator = BasicNavigator()
    print('Waiting for Nav2. Set the initial pose in RViz first.')
    navigator.waitUntilNav2Active()

    for number, (name, x, y, yaw) in enumerate(waypoints, start=1):
        print(f'[{number}/{len(waypoints)}] Sending goal: {name} at ({x:.2f}, {y:.2f})')
        navigator.goToPose(make_pose(navigator, x, y, yaw))

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                distance = getattr(feedback, 'distance_remaining', float('nan'))
                print(f'  {name}: {distance:.2f} m remaining', end='\r', flush=True)

        result = navigator.getResult()
        print()
        if result != TaskResult.SUCCEEDED:
            print(f'Mission stopped at {name}: Nav2 result = {result}', file=sys.stderr)
            rclpy.shutdown()
            return 1
        print(f'Arrived at {name}.')

    print('Mission completed: all delivery locations visited.')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
