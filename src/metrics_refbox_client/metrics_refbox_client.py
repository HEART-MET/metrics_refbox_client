#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import std_msgs.msg
import os
import json
import uuid
import datetime
import metrics_refbox_msgs.msg
from metrics_refbox_msgs.msg import Command, Confirm


class MetricsRefboxClient(object):
    """
    """
    def __init__(self):
        config_file = rospy.get_param('~config_file')
        self.config_file_path = rospy.get_param('~config_file_path')
        refbox_ns = rospy.get_param("~refbox_ns")
        self.team_name = rospy.get_param("~team_name")

        stream = open(os.path.join(self.config_file_path, config_file), 'r')
        self.config = json.load(stream)

        self.refbox_command = None
        self.refbox_filename = None
        self.rosbag_event = None
        self.result = None
        self.result_type = None
        self.wait_for_recording_start_time = None
        self.recording_timeout = 10.0

        self.state = 'IDLE'

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Subscribers
        rospy.Subscriber("~refbox_command", metrics_refbox_msgs.msg.Command, self.command_cb)
        rospy.Subscriber("~rosbag_event_out", std_msgs.msg.String, self.rosbag_event_cb)
        rospy.Subscriber("~rosbag_filename", std_msgs.msg.String, self.rosbag_filename_cb)
        rospy.Subscriber("/test_topic", std_msgs.msg.String, self.test_topic_cb)

        # Publishers
        self.command_confirm_pub = rospy.Publisher("~refbox_command_confirmation", metrics_refbox_msgs.msg.Confirm, queue_size = 1)
        self.rosbag_event_pub = rospy.Publisher("~rosbag_event_in", std_msgs.msg.String, queue_size = 1)
        self.command_pub = rospy.Publisher("~command", metrics_refbox_msgs.msg.Command, queue_size = 1)

        self.result_publishers = {}
        for key in self.config['benchmarks'].keys():
            msg_type = getattr(metrics_refbox_msgs.msg, self.config['benchmarks'][key]['type'])
            self.result_publishers[key] = rospy.Publisher("%s/%s" % (refbox_ns, self.config['benchmarks'][key]['topic']),
                                                            msg_type, queue_size = 1)
            rospy.Subscriber("~%s" % self.config['benchmarks'][key]['topic'], msg_type, self.result_cb, key)
        self.test_pub = rospy.Publisher('/test_topic_ack', std_msgs.msg.String, queue_size = 1)
        self.show_env_var('ROS_MASTER_URI')
        self.show_env_var('ROS_IP')
        self.show_env_var('ROS_HOSTNAME')

    def show_env_var(self, var):
        env_str = os.getenv(var) if os.getenv(var) is not None else ''
        msg = var + ': ' + env_str
        print(msg)

    def command_cb(self, msg):
        self.refbox_command = msg
        rospy.logdebug('[client] received refbox command %d %d' % (msg.task, msg.command))

    def rosbag_event_cb(self, msg):
        self.rosbag_event = msg.data
        rospy.logdebug('[client] received rosbag event %s' % msg.data)

    def rosbag_filename_cb(self, msg):
        self.rosbag_filename = msg.data
        rospy.logdebug('[client] received rosbag filename %s' % msg.data)

    def result_cb(self, msg, result_type):
        self.result = msg
        self.result_type = result_type
        rospy.logdebug('[client] received result of type %s' % result_type)

    def test_topic_cb(self, msg):
        self.test_pub.publish(self.team_name)

    def run(self):
        while not rospy.is_shutdown():
            if self.state == 'IDLE':
                self.state = self.idle_state()
            elif self.state == 'WAIT_FOR_START_RECORDING':
                self.state = self.wait_for_start_recording()
            elif self.state == 'BENCHMARK_RUNNING':
                self.state = self.benchmark_running()
            elif self.state == 'BENCHMARK_COMPLETE':
                self.state = self.benchmark_complete()
            self.loop_rate.sleep()

    def idle_state(self):
        '''
        Wait for a command from the refbox.
        We're expecting a START command
        but will also respond to a STOP command

        If a START command is received, send start to rosbag recorder
        and transition to WAIT_FOR_START_RECORDING
        '''
        if self.refbox_command is not None:
            if self.refbox_command.command == Command.START:
                self.rosbag_event = None
                self.rosbag_filename = None
                self.rosbag_event_pub.publish('e_start')
                self.wait_for_recording_start_time = rospy.Time.now()
                # TODO: start a timer here
                return 'WAIT_FOR_START_RECORDING'
            elif self.refbox_command.command == Command.STOP:
                self.rosbag_event_pub.publish('e_stop')
                self.command_pub.publish(self.refbox_command)
                self.refbox_command = None
        return 'IDLE'

    def wait_for_start_recording(self):
        '''
        Wait until rosbag recorder has started recording and reports the filename
        Once it has started, send the refbox command to the robot,
        and send the confirmation to back to the refbox
        Transition to BENCHMARK_RUNNING.
        '''
        if self.rosbag_event == 'e_started' and self.rosbag_filename is not None:
            self.result = None
            self.result_type = None
            # tell the robot to start benchmark
            self.command_pub.publish(self.refbox_command)

            # send confirmation to refbox that we received the start command
            # and have started the benchmark
            confirm_msg = metrics_refbox_msgs.msg.Confirm()
            confirm_msg.uid = self.refbox_command.uid
            confirm_msg.data = True
            confirm_msg.rosbag_filename = self.rosbag_filename
            self.command_confirm_pub.publish(confirm_msg)

            self.rosbag_filename = None
            self.rosbag_event = None
            self.refbox_command = None
            self.result = None
            self.result_type = None
            self.wait_for_recording_start_time = None
            return 'BENCHMARK_RUNNING'
        elif (rospy.Time.now() - self.wait_for_recording_start_time) > rospy.Duration(self.recording_timeout):
            rospy.logerr("Rosbag recorder not responding")
            # tell the refbox that we received the start command
            # but did not start the benchmark since the recorder is not running
            confirm_msg = metrics_refbox_msgs.msg.Confirm()
            confirm_msg.uid = self.refbox_command.uid
            confirm_msg.data = False
            self.command_confirm_pub.publish(confirm_msg)
            return 'IDLE'
        # TODO: check timer here
        return 'WAIT_FOR_START_RECORDING'

    def benchmark_running(self):
        '''
        Wait until we receive a result from the robot, or a stop command from the refbox
        In either case, transition to IDLE.
        If we receive a result, republish it to the refbox
        If we receive a stop command, republish it to the robot, and stop recording.
        '''
        if self.result is not None and self.result_type is not None:
            if self.result.message_type == self.result.RESULT:
                self.result_publishers[self.result_type].publish(self.result)
                self.rosbag_event_pub.publish('e_stop')
                self.result = None
                self.result_type = None
                return 'IDLE'
            elif self.result.message_type == self.result.FEEDBACK:
                self.result_publishers[self.result_type].publish(self.result)
                self.result = None
                self.result_type = None
                return 'BENCHMARK_RUNNING'

        if self.refbox_command is not None:
            if self.refbox_command.command == Command.STOP:
                self.command_pub.publish(self.refbox_command)
                self.rosbag_event_pub.publish('e_stop')
                self.refbox_command = None
                return 'IDLE'
            elif self.refbox_command.command == Command.START:
                rospy.logerr('received START command from refbox while benchmark is already running')
                self.refbox_command = None
            else:
                rospy.logerr('received invalid command from refbox while benchmark is running')
                self.refbox_command = None
        return 'BENCHMARK_RUNNING'


def main():
    client = MetricsRefboxClient()
    client.run()
