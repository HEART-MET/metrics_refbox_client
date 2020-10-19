#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import std_msgs.msg
import os
import json
import uuid
import datetime
import metrics_refbox_msgs.msg
from metrics_refbox_msgs.msg import Command
from metrics_refbox_msgs.msg import ObjectDetectionResult, HumanRecognitionResult, ActivityRecognitionResult
from metrics_refbox_msgs.msg import GestureRecognitionResult, HandoverObjectResult, ReceiveObjectResult
from metrics_refbox_msgs.msg import BoundingBox2D


class MetricsBenchmarkMockup(object):
    """
    """
    def __init__(self):
        config_file = rospy.get_param('~config_file')
        self.config_file_path = rospy.get_param('~config_file_path')
        refbox_client_ns = rospy.get_param("~refbox_client_ns")

        stream = open(os.path.join(self.config_file_path, config_file), 'r')
        self.config = json.load(stream)

        self.refbox_command = None
        self.start_time = None
        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Subscribers
        rospy.Subscriber("~refbox_command", metrics_refbox_msgs.msg.Command, self.command_cb)

        # Publishers
        self.result_publishers = {}
        for key in self.config['benchmarks'].keys():
            msg_type = getattr(metrics_refbox_msgs.msg, self.config['benchmarks'][key]['type'])
            self.result_publishers[key] = rospy.Publisher("%s/%s" % (refbox_client_ns, self.config['benchmarks'][key]['topic']),
                                                            msg_type, queue_size = 1)

    def command_cb(self, msg):
        self.refbox_command = msg
        self.benchmark_duration = rospy.Duration.from_sec(5.0)
        self.start_time = rospy.Time.now()
        rospy.logdebug('[mockup] received refbox command %d %d' % (msg.task, msg.command))

    def run(self):
        while not rospy.is_shutdown():
            if self.refbox_command is not None:
                if (rospy.Time.now() - self.start_time) > self.benchmark_duration:
                    if self.refbox_command.task == Command.OBJECT_DETECTION:
                        self.send_object_detection_result()
                    elif self.refbox_command.task == Command.HUMAN_RECOGNITION:
                        self.send_human_recognition_result()
                    elif self.refbox_command.task == Command.ACTIVITY_RECOGNITION:
                        self.send_activity_recognition_result()
                    elif self.refbox_command.task == Command.GESTURE_RECOGNITION:
                        self.send_gesture_recognition_result()
                    elif self.refbox_command.task == Command.HANDOVER_OBJECT:
                        self.send_handover_object_result()
                    elif self.refbox_command.task == Command.RECEIVE_OBJECT:
                        self.send_receive_object_result()
                    self.start_time = None
                    self.refbox_command = None
            self.loop_rate.sleep()

    def send_object_detection_result(self):
        result = ObjectDetectionResult()
        result.result_type = ObjectDetectionResult.BOUNDING_BOX_2D
        result.object_found = True
        #result.box2d.min_x = 5.0
        #result.box2d.min_y = 10.0
        #result.box2d.max_x = 28.0
        #result.box2d.max_y = 50.0
        self.result_publishers['object_detection'].publish(result)

    def send_human_recognition_result(self):
        result = HumanRecognitionResult()
        result.identity = "Alice"
        self.result_publishers['human_recognition'].publish(result)

    def send_activity_recognition_result(self):
        result = ActivityRecognitionResult()
        result.activity = "Drinking from a cup"
        self.result_publishers['activity_recognition'].publish(result)

    def send_gesture_recognition_result(self):
        result = GestureRecognitionResult()
        result.gesture = "Waving for attention"
        self.result_publishers['gesture_recognition'].publish(result)

    def send_handover_object_result(self):
        result = HandoverObjectResult()
        result.human_pose = HandoverObjectResult.HUMAN_POSE_STANDING
        result.human_reach_out_result = HandoverObjectResult.HUMAN_REACHED_OUT
        result.grasp_result = HandoverObjectResult.GRASP_SUCCESSFUL
        result.post_grasp_result = HandoverObjectResult.OBJECT_NOT_DROPPED_AFTER_GRASP

        self.result_publishers['handover_object'].publish(result)

    def send_receive_object_result(self):
        result = ReceiveObjectResult()
        result.human_pose = ReceiveObjectResult.HUMAN_POSE_STANDING
        result.human_reach_out_result = ReceiveObjectResult.HUMAN_REACHED_OUT
        result.pre_grasp_result = ReceiveObjectResult.OBJECT_NOT_DROPPED_BEFORE_GRASP
        result.grasp_result = ReceiveObjectResult.GRASP_SUCCESSFUL
        result.post_grasp_result = ReceiveObjectResult.OBJECT_RELEASED
        self.result_publishers['receive_object'].publish(result)



def main():
    client = MetricsBenchmarkMockup()
    client.run()
