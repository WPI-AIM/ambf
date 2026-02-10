#  Author(s):  Anton Deguet
#  Created on: 2023-05-08
#
# Copyright (c) 2023-2024 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import rospy
import sys

class ral:
    """RAL: ROS abstraction layer

    Provides many common parts of rospy/rclpy via a common API, to allow CRTK code to
    be written at a higher level, and to work for both ROS 1 and ROS 2.
    """

    @staticmethod
    def ros_version():
        return 1

    @staticmethod
    def parse_argv(argv):
        # strip ros arguments
        return rospy.myargv(argv)

    def __init__(self, node_name, namespace = '', node = None):
        self._node_name = node_name
        self._namespace = namespace.strip('/')

        self._children = {}
        self._publishers = []
        self._subscribers = []
        self._services = []

        if node is not None:
            self._node = node
        else:
            # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
            try:
                rospy.init_node(self.node_name(), anonymous = True)
            except Exception as e:
                print(e)

    def __del__(self):
        for pub in self._publishers:
            pub.unregister()

        for sub in self._subscribers:
            sub.unregister()

    def node_name(self):
        return self._node_name

    def namespace(self):
        return self._namespace

    def create_child(self, child_namespace):
        if child_namespace in self._children:
            err_msg = 'ral object already has child "{}"!'.format(child_namespace)
            raise RuntimeError(err_msg)

        full_child_namespace = self._add_namespace_to(child_namespace)
        child = ral(self.node_name(), full_child_namespace, self)
        self._children[child_namespace] = child
        return child

    def now_msg(self):
        return rospy.Time.now()

    def now(self):
        return rospy.Time.now()

    def get_timestamp(self, t):
        if hasattr(t, 'header'):
            t = t.header
        if hasattr(t, 'stamp'):
            t = t.stamp

        return t

    def to_sec(self, t):
        t = self.get_timestamp(t)
        return t.to_sec()

    def create_duration(self, d):
        return rospy.Duration(d)

    def create_rate(self, rate_hz):
        return rospy.Rate(rate_hz)

    def create_time(self):
        return rospy.Time()

    def spin(self):
        pass # Not applicable in ROS 1

    def shutdown(self):
        # mimic the rclpy behavior
        raise RuntimeError('crtk.ral.shutdown() has been called')

    def spin_and_execute(self, function, *arguments):
        function(*arguments)

    def on_shutdown(self, callback):
        rospy.on_shutdown(callback)

    def is_shutdown(self):
        return rospy.is_shutdown()

    def _add_namespace_to(self, name):
        name = name.strip('/')
        qualified_name = '{}/{}'.format(self.namespace(), name)
        return qualified_name

    def publisher(self, topic, msg_type, queue_size = 10, latch = False, *args, **kwargs):
        pub = rospy.Publisher(self._add_namespace_to(topic), msg_type,
                              latch = latch, queue_size = queue_size, *args, **kwargs)
        self._publishers.append(pub)
        return pub

    # latch is added to be compatible with ROS2 crtk.ral
    def subscriber(self, topic, msg_type, callback, queue_size = 10, latch = False, *args, **kwargs):
        sub = rospy.Subscriber(self._add_namespace_to(topic), msg_type, callback = callback,
                               queue_size = queue_size, *args, **kwargs)
        self._subscribers.append(sub)
        return sub

    def service_client(self, name, srv_type):
        client = rospy.ServiceProxy(self._add_namespace_to(name), srv_type)
        self._services.append(client)
        return client

    def get_topic_name(self, pubsub):
        return pubsub.name

    def get_published_topics(self):
        return rospy.get_published_topics()

    def _check_connections(self, start_time, timeout_duration, check_children):
        check_rate = self.create_rate(100)
        connected = lambda pubsub: pubsub.get_num_connections() > 0

        # wait at most timeout_seconds for all connections to establish
        while (self.now() - start_time) < timeout_duration:
            pubsubs = self._publishers + self._subscribers
            unconnected = [ps for ps in pubsubs if not connected(ps)]
            if len(unconnected) == 0:
                break

            check_rate.sleep()

        if check_children:
            # recursively check connections in all child ral objects,
            # using same start time so the timeout period doesn't restart
            for _, child in self._children.items():
                child._check_connections(start_time, timeout_duration, check_children)

        # last check of connection status, raise error if any remain unconnected
        unconnected_pubs = [p for p in self._publishers if not connected(p)]
        unconnected_subs = [s for s in self._subscribers if not connected(s)]
        if len(unconnected_pubs) == 0 and len(unconnected_subs) == 0:
            return

        connected_pub_count = len(self._publishers) - len(unconnected_pubs)
        connected_sub_count = len(self._subscribers) - len(unconnected_subs)
        unconnected_pub_names = ', '.join([self.get_topic_name(p) for p in unconnected_pubs])
        unconnected_sub_names = ', '.join([self.get_topic_name(s) for s in unconnected_subs])

        err_msg = \
        (
            'Timed out waiting for publisher/subscriber connections to establish\n'
            '    Publishers:  {} connected,'
            ' {} not connected\n'
            '                 not connected: [{}]\n\n'
            '    Subscribers: {} connected,'
            ' {} not connected\n'
            '                 not connected: [{}]\n\n'
        )
        err_msg = err_msg.format(connected_pub_count, len(unconnected_pubs), unconnected_pub_names,
                                 connected_sub_count, len(unconnected_subs), unconnected_sub_names)
        raise TimeoutError(err_msg.format(connected_pub_count))

    def check_connections(self, timeout_seconds = 5.0, check_children = True):
        """Check that all publishers are connected to at least one subscriber,
        and that all subscribers are connected to at least on publisher.

        If timeout_seconds is zero, no checks will be done.
        If check_children is True, all children will be checked as well.
        """

        if timeout_seconds == 0.0:
            return

        start_time = self.now()
        timeout_duration = self.create_duration(timeout_seconds)

        self._check_connections(start_time, timeout_duration, check_children)
