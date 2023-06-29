#!/usr/bin/env python

import rospy
import logging
from std_msgs.msg import String
from rospy.exceptions import ROSInitException
from collections import defaultdict

class HeartbeatMonitorNode:
    def __init__(self):
        self.node_statuses = defaultdict(lambda: 'ERROR')
        self.topic_statuses = defaultdict(lambda: 'ERROR')
        self.topic_publish_rates = {}
        self.liveness_statuses = defaultdict(lambda: 'ERROR')
        self.liveness_subscribers = {}
        self.topics_config = {}
        self.liveness_topic = None
        self.status_publisher = rospy.Publisher('/heartbeat/status', String, queue_size=10)
        self.is_alive = True

        self.load_config_from_parameter_server()
        self.subscribe_to_topics()
        self.subscribe_to_liveness_topics()

        self.logger = logging.getLogger('heartbeat_monitor_node')
        self.logger.setLevel(logging.INFO)
        log_handler = logging.StreamHandler()
        log_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        log_handler.setFormatter(log_formatter)
        self.logger.addHandler(log_handler)

    def load_config_from_parameter_server(self):
        topics_param = rospy.get_param('~topics', {})
        self.topics_config['topics'] = topics_param.get('topics', [])
        self.topics_config['publish_rates'] = topics_param.get('publish_rates', {})

        self.liveness_topic = rospy.get_param('~liveness_topic', '/liveness')

    def subscribe_to_topics(self):
        for topic_name in self.topics_config['topics']:
            self.topic_publish_rates[topic_name] = 0
            rospy.Subscriber(topic_name, rospy.AnyMsg, self.topic_callback, callback_args=topic_name)

    def subscribe_to_liveness_topics(self):
        liveness_nodes_param = rospy.get_param('~liveness_nodes', {})
        for node_name, liveness_topic in liveness_nodes_param.items():
            self.liveness_statuses[node_name] = 'ERROR'
            self.liveness_subscribers[node_name] = rospy.Subscriber(liveness_topic, String, self.liveness_callback, callback_args=node_name)

    def topic_callback(self, data, topic_name):
        self.topic_statuses[topic_name] = 'OK'
        self.topic_publish_rates[topic_name] += 1

    def liveness_callback(self, data, node_name):
        self.liveness_statuses[node_name] = 'OK'

    def publish_status(self):
        status_msg = String()
        if not self.is_alive:
            status_msg.data = 'CRASHED'
        elif not all(self.node_statuses.values()) or not all(self.topic_statuses.values()) or not all(self.liveness_statuses.values()):
            status_msg.data = 'ERROR'
        else:
            status_msg.data = 'OK'
        self.status_publisher.publish(status_msg)
        self.logger.info('Node status: %s', status_msg.data)
        for node_name, node_status in self.node_statuses.items():
            self.logger.info('Node %s status: %s', node_name, node_status)
        for topic_name, topic_status in self.topic_statuses.items():
            self.logger.info('Topic %s status: %s, publish rate: %f', topic_name, topic_status, self.topic_publish_rates[topic_name])

    def check_publish_rates(self, event):
        for topic_name in self.topics_config['topics']:
            rate = self.topics_config['publish_rates'].get(topic_name, 0)
            if rate > 0 and self.topic_publish_rates[topic_name] < rate:
                self.topic_statuses[topic_name] = 'ERROR'

    def run(self):
        rate = rospy.Rate(1) # publish status once per second
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_publish_rates)
        while not rospy.is_shutdown():
            self.publish_status()
            self.is_alive = True
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('heartbeat_monitor_node')
        heartbeat_monitor_node = HeartbeatMonitorNode()
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
