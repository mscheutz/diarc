#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading

import numpy as np
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e=e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
                or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()

    return check


def twist_command(base, action):
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 10

    twist = command.twist
    twist.linear_x = action[0]
    twist.linear_y = action[1]
    twist.linear_z = action[2]
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 0

    base.SendTwistCommand(command)
    time.sleep(0.01)
    return True


def cartesian_movement(base, base_cyclic, pose):
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = pose[0]
    cartesian_pose.y = pose[1]
    cartesian_pose.z = pose[2]
    if len(pose) == 6:
        cartesian_pose.theta_x = pose[3]
        cartesian_pose.theta_y = pose[4]
        cartesian_pose.theta_z = pose[5]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def compute_twist(pos_initial, pos_final, total_time=0.1):
    pos_initial = np.array(pos_initial)
    pos_final = np.array(pos_final)
    twist = (pos_final - pos_initial) / total_time
    return twist


def go_to(base, base_cyclic, action):
    init_pos = get_current_pos(base_cyclic)
    twist = compute_twist(init_pos, action)
    twist_command(base, twist)
    time.sleep(0.1)

def get_current_pos(base_cyclic: BaseCyclicClient):
    feedback = base_cyclic.RefreshFeedback()
    cartesian_pose = [0] * 3
    cartesian_pose[0] = feedback.base.tool_pose_x  # (meters)
    cartesian_pose[1] = feedback.base.tool_pose_y  # (meters)
    cartesian_pose[2] = feedback.base.tool_pose_z  # (meters)
    # cartesian_pose[3] = feedback.base.tool_pose_theta_x  # (degrees)
    # cartesian_pose[4] = feedback.base.tool_pose_theta_y  # (degrees)
    # cartesian_pose[5] = feedback.base.tool_pose_theta_z  # (degrees)
    return cartesian_pose

