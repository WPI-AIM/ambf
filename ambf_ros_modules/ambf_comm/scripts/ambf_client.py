#!/usr/bin/env python
import rospy
from ambf_msgs.msg import ObjectState, ObjectCmd, WorldState, WorldCmd
import threading
from geometry_msgs.msg import WrenchStamped
from ambf_object import Object
from ambf_world import World


class Client:
    def __init__(self):
        self._ros_topics = []
        self._search_prefix_str = '/ambf/env/'
        self._search_suffix_str = '/State'
        self._string_cmd = '/Command'
        self._sub_list = []
        self._objects_dict = {}
        self._sub_thread = []
        self._pub_thread = []
        self._rate = 0
        self._world_name = ''
        pass

    def create_objs_from_rostopics(self):
        rospy.init_node('ambf_client')
        rospy.on_shutdown(self.clean_up)
        self._rate = rospy.Rate(1000)
        self._ros_topics = rospy.get_published_topics()
        for i in range(len(self._ros_topics)):
            for j in range(len(self._ros_topics[i])):
                prefix_ind = self._ros_topics[i][j].find(self._search_prefix_str)
                if prefix_ind >= 0:
                    search_ind = self._ros_topics[i][j].find(self._search_suffix_str)
                    if search_ind >= 0:
                        # Searching the active topics between the end of prefix:/ambf/env/ and start of /State
                        obj_name = self._ros_topics[i][j][
                                     prefix_ind + len(self._search_prefix_str):search_ind]
                        if obj_name == 'World' or obj_name == 'world':
                            self._world_name = obj_name
                            obj = World(obj_name)
                            obj._sub = rospy.Subscriber(self._ros_topics[i][j], WorldState, obj.ros_cb)
                            pub_topic_str = self._search_prefix_str + obj_name + self._string_cmd
                            obj._pub = rospy.Publisher(name=pub_topic_str, data_class=WorldCmd, queue_size=10)
                        else:
                            obj = Object(obj_name)
                            obj._sub = rospy.Subscriber(self._ros_topics[i][j], ObjectState, obj.ros_cb)
                            pub_topic_str = self._search_prefix_str + obj_name + self._string_cmd
                            obj._pub = rospy.Publisher(name=pub_topic_str, data_class=ObjectCmd, tcp_nodelay=True, queue_size=10)

                        self._objects_dict[obj_name] = obj

    def start(self):
        self._start_pubs()

    def get_obj_names(self):
        obj_names = []
        for key, obj in self._objects_dict.items():
            obj_names.append(obj.get_name())
        return obj_names

    def get_obj_handle(self, a_name):
        obj = self._objects_dict.get(a_name)
        obj.set_active()
        return obj

    def get_obj_pose(self, a_name):
        obj = self._objects_dict.get(a_name)
        if obj is not None:
            return obj.pose
        else:
            return None

    def set_obj_cmd(self, a_name, fx, fy, fz, nx, ny, nz):
        obj = self._objects_dict.get(a_name)
        obj.command(fx, fy, fz, nx, ny, nz)

    def _start_pubs(self):
        self._pub_thread = threading.Thread(target=self._run_obj_publishers)
        self._pub_thread.daemon = True
        self._pub_thread.start()

    def _run_obj_publishers(self):
        while not rospy.is_shutdown():
            for key, obj in self._objects_dict.items():
                if obj.is_active():
                    obj.run_publisher()
            self._rate.sleep()

    def print_active_topics(self):
        print self._ros_topics
        pass

    def print_summary(self):
        print '_________________________________________________________'
        print '---------------------------------------------------------'
        print 'CLIENT FOR CREATING OBJECTS FROM ROSTOPICS'
        print 'Searching Object names from ros topics with'
        print 'Prefix: ', self._search_prefix_str
        print 'Suffix: ', self._search_suffix_str
        print 'Number of OBJECTS found', len(self._objects_dict)
        for key, value in self._objects_dict.items():
            print key
        print '---------------------------------------------------------'

    def clean_up(self):
        for key, val in self._objects_dict.iteritems():
            val.pub_flag = False
            print 'Closing publisher for: ', key

