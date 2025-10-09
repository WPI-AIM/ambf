#  Author(s):  Anton Deguet
#  Created on: 2023-05-08
#
# Copyright (c) 2023-2025 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import rclpy
import rclpy.publisher
import rclpy.qos
import rclpy.subscription
import rclpy.time
import rclpy.utilities
import threading


class ral:
    """RAL: ROS abstraction layer

    Provides many common parts of rospy/rclpy via a common API, to allow CRTK code to
    be written at a higher level, and to work for both ROS 1 and ROS 2.
    """

    @staticmethod
    def ros_version():
        return 2

    @staticmethod
    def parse_argv(argv):
        # strip ros arguments
        rclpy.init(args = argv)
        return rclpy.utilities.remove_ros_args(argv)

    def __init__(self, node_name, namespace = '', node = None):
        self._node_name = node_name
        self._namespace = namespace.strip('/')

        self._children = {}
        self._publishers = []
        self._subscribers = []
        self._services = []

        self._executor_thread = None

        if node is not None:
            self._node = node
        else:
            # initializes rclpy only if it hasn't been already (e.g. via parse_argv()),
            # otherwise does nothing (since rclpy will raise a RuntimeError)
            try:
                rclpy.init()
            except:
                pass
            # ros init node
            self._node = rclpy.create_node(self.node_name())

    def __del__(self):
        for pub in self._publishers:
            self._node.destroy_publisher(pub)

        for sub in self._subscribers:
            self._node.destroy_subscription(sub)

    def node_name(self):
        return self._node_name

    def namespace(self):
        return self._namespace

    def create_child(self, child_namespace):
        if child_namespace in self._children:
            err_msg = 'ral object already has child "{}"!'.format(child_namespace)
            raise RuntimeError(err_msg)

        full_child_namespace = self._add_namespace_to(child_namespace)
        child = ral(self.node_name(), full_child_namespace, self._node)
        self._children[child_namespace] = child
        return child

    def now_msg(self):
        return self._node.get_clock().now().to_msg() ## For backward compat

    def now(self):
        return self._node.get_clock().now()

    def get_timestamp(self, t):
        if hasattr(t, 'header'):
            t = t.header
        if hasattr(t, 'stamp'):
            t = t.stamp

        try:
            return rclpy.time.Time.from_msg(t)
        except:
            return t

    def to_sec(self, t):
        t = self.get_timestamp(t)
        return float(t.nanoseconds)/1e9

    def create_duration(self, d):
        return rclpy.time.Duration(seconds = d)

    def create_rate(self, rate_hz):
        return self._node.create_rate(frequency = rate_hz,
                                      clock = self._node.get_clock())

    def create_time(self):
        return rclpy.time.Time(seconds = 0.0,
                               clock_type = self._node.get_clock().clock_type)

    def _try_spin(self):
        try:
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self._node)
            executor.spin()
        except rclpy.executors.ExternalShutdownException:
            pass
        
    def spin(self):
        if self._executor_thread != None:
            return
        self._executor_thread = threading.Thread(target = self._try_spin,
                                                 daemon = True)
        self._executor_thread.start()

    def shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()

        if self._executor_thread != None:
            self._executor_thread.join()
            self._executor_thread = None

    def spin_and_execute(self, function, *arguments):
        self.spin()
        try:
            function(*arguments)
        except KeyboardInterrupt:
            pass
        self.shutdown()

    def on_shutdown(self, callback):
        rclpy.get_default_context().on_shutdown(callback)

    def is_shutdown(self):
        return not rclpy.ok()

    def _add_namespace_to(self, name):
        name = name.strip('/')
        qualified_name = '{}/{}'.format(self.namespace(), name)
        return qualified_name

    def publisher(self, topic, msg_type, queue_size = 10, latch = False, *args, **kwargs):
        history = rclpy.qos.HistoryPolicy.KEEP_LAST
        qos = rclpy.qos.QoSProfile(depth = queue_size, history = history)
        if latch:
            # publisher will retain queue_size messages for future subscribers
            qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        pub = self._node.create_publisher(msg_type, self._add_namespace_to(topic), qos)
        self._publishers.append(pub)
        return pub

    def subscriber(self, topic, msg_type, callback, queue_size=10, latch = False, *args, **kwargs):
        history = rclpy.qos.HistoryPolicy.KEEP_LAST
        qos = rclpy.qos.QoSProfile(depth = queue_size, history = history)
        if latch:
            # subscriber should get past messages
            qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        sub = self._node.create_subscription(msg_type, self._add_namespace_to(topic),
                                              callback, qos, *args, **kwargs)
        self._subscribers.append(sub)
        return sub

    def service_client(self, name, srv_type):
        client = self._node.create_client(srv_type, self._add_namespace_to(name))
        if not hasattr(client, '__call__'):
            client.__call__ = client.call
        self._services.append(client)
        return client

    def get_topic_name(self, pubsub):
        return pubsub.topic_name

    def get_published_topics(self):
        published_topics = []
        pub_names_and_types = []
        for [node_name, node_namespace] in self._node.get_node_names_and_namespaces():
                pub_names_and_types += self._node.get_publisher_names_and_types_by_node(node_name, node_namespace)
        for i in range(len(pub_names_and_types)):
            published_topics.append([pub_names_and_types[i][0], pub_names_and_types[i][1][0]])
        return published_topics

    def _check_connections(self, start_time, timeout_duration, check_children):
        def connected(pubsub):
            if isinstance(pubsub, rclpy.publisher.Publisher):
                # get_subscription_count() available in >= ROS 2 Dashing
                return pubsub.get_subscription_count() > 0
            elif isinstance(pubsub, rclpy.subscription.Subscription):
                try:
                    return pubsub.get_publisher_count() > 0
                except AttributeError:
                    # get_publisher_count() not available until ROS 2 Iron Irwini
                    return True
            else:
                raise TypeError("pubsub must be an rclpy Publisher or Subscription")

        check_rate = self.create_rate(100)

        # wait at most timeout_seconds for all connections to establish
        while (self._now() - start_time) < timeout_duration:
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
        raise TimeoutError(err_msg)

    def check_connections(self, timeout_seconds = 5.0, check_children = True):
        """Check that all publishers are connected to at least one subscriber,
        and that all subscribers are connected to at least on publisher.

        If timeout_seconds is zero, no checks will be done.
        If check_children is True, all children will be checked as well.
        """

        if timeout_seconds == 0.0:
            return

        start_time = self._now()
        timeout_duration = self.create_duration(timeout_seconds)

        self._check_connections(start_time, timeout_duration, check_children)
