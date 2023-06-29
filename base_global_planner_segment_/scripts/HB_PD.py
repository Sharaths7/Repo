#!/usr/bin/env python

import rospy
import logging
import yaml
from std_msgs.msg import String
from rospy.exceptions import ROSInitException
from collections import defaultdict


class HeartbeatMonitorNode:
    def __init__(self, topics_file):
        self.node_statuses = defaultdict(lambda: 'ERROR')
        self.topic_statuses = defaultdict(lambda: 'ERROR')
        self.topic_publish_rates = {}
        self.liveness_statuses = defaultdict(lambda: 'ERROR')
        self.topics_config = None
        self.liveness_topic = None
        self.liveness_publisher = None
        self.status_publisher = None
        self.is_alive = True

        with open(topics_file, 'r') as f:
            self.topics_config = yaml.load(f, Loader=yaml.FullLoader)
            for topic_name in self.topics_config['topics']:
                rospy.Subscriber(topic_name, rospy.AnyMsg, self.topic_callback, callback_args=topic_name)
                self.topic_publish_rates[topic_name] = 0

        self.liveness_topic = self.topics_config.get('liveness_topic', '/liveness')
        self.liveness_publisher = rospy.Publisher(self.liveness_topic, String, queue_size=10)
        self.status_publisher = rospy.Publisher('/heartbeat/status', String, queue_size=10)
        rospy.Subscriber(self.liveness_topic, String, self.liveness_callback)

        self.logger = logging.getLogger('heartbeat_monitor_node')
        self.logger.setLevel(logging.INFO)
        log_file = '/home/infy/Desktop/FieldRobots/navigation_ws/src/base_trajectory/base_trajectory_interface/src/base_global_planner_segment_/config/heartbeat_report.log'  # Replace with the desired log file path
        log_handler = logging.FileHandler(log_file)
        log_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        log_handler.setFormatter(log_formatter)
        self.logger.addHandler(log_handler)

    def topic_callback(self, data, topic_name):
        self.topic_statuses[topic_name] = 'OK'
        self.topic_publish_rates[topic_name] += 1

    def liveness_callback(self, data):
        self.liveness_statuses[data._connection_header['callerid']] = 'OK'

    def publish_status(self):
        status_msg = String()
        if not self.is_alive:
            status_msg.data = 'CRASHED'
        elif not all(self.node_statuses.values()) or not all(self.topic_statuses.values()) or not all(
                self.liveness_statuses.values()):
            status_msg.data = 'ERROR'
        else:
            status_msg.data = 'OK'
        self.status_publisher.publish(status_msg)
        self.logger.info('Node status: %s', status_msg.data)
        for node_name, node_status in self.node_statuses.items():
            self.logger.info('Node %s status: %s', node_name, node_status)
        for topic_name, topic_status in self.topic_statuses.items():
            self.logger.info('Topic %s status: %s, publish rate: %f', topic_name, topic_status,
                             self.topic_publish_rates[topic_name])

    def publish_liveness(self):
        liveness_msg = String()
        liveness_msg.data = 'ALIVE'
        self.liveness_publisher.publish(liveness_msg)

    def check_publish_rates(self, event):
        for topic_name in self.topics_config['topics']:
            rate = self.topics_config['publish_rates'].get(topic_name, 0)
            if rate > 0 and self.topic_publish_rates[topic_name] < rate:
                self.topic_statuses[topic_name] = 'ERROR'

    def run(self):
        rate = rospy.Rate(1)  # Publish status and liveness once per second
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_publish_rates)
        while not rospy.is_shutdown():
            self.publish_status()
            self.publish_liveness()
            self.is_alive = True
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('heartbeat_monitor_node')
        heartbeat_monitor_node = HeartbeatMonitorNode(
            '/home/infy/Desktop/FieldRobots/navigation_ws/src/base_trajectory/base_trajectory_interface/src/base_global_planner_segment_/config/topics.yaml')  # Replace with the path to your topics.yaml file
        heartbeat_monitor_node.run()
    except ROSInitException as e:
        rospy.logerr('Failed to initialize heartbeat monitor node: %s', str(e))
    except Exception as e:
        rospy.logerr('Heartbeat monitor node crashed: %s', str(e))
        heartbeat_monitor_node.is_alive = False
        crash_topic = rospy.Publisher('/heartbeat/crash', String, queue_size=10)
        crash_msg = String()
        crash_msg.data = rospy.get_name()
        crash_topic.publish(crash_msg)
