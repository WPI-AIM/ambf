#ifndef _ambf_ral_h
#define _ambf_ral_h

// to define AMBF_ROS1, AMBF_ROS2 , AMBF_ROS_DISTRO
#include <ambf_server/ambf_ral_config.h>

// this file is based on cisst-ros/cisst_ros_bridge cisst_ral.h. we
// should probably try to merge this and distribute in a single
// package (Anton)

#include <string>
#include <regex>

// forward declaration
namespace ambf_ral {
    void clean_namespace(std::string &);
}

#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>

#if AMBF_ROS1

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <ambf_msgs/ObjectState.h>
#include <ambf_msgs/ObjectCmd.h>
#include <ambf_msgs/LightState.h>
#include <ambf_msgs/LightCmd.h>
#include <ambf_msgs/VehicleState.h>
#include <ambf_msgs/VehicleCmd.h>
#include <ambf_msgs/ActuatorState.h>
#include <ambf_msgs/ActuatorCmd.h>
#include <ambf_msgs/CameraState.h>
#include <ambf_msgs/CameraCmd.h>
#include <ambf_msgs/RigidBodyState.h>
#include <ambf_msgs/RigidBodyCmd.h>
#include <ambf_msgs/SensorState.h>
#include <ambf_msgs/SensorCmd.h>
#include <ambf_msgs/ContactData.h>
#include <ambf_msgs/ContactEvent.h>
#include <ambf_msgs/ContactSensorState.h>
#include <ambf_msgs/ContactSensorCmd.h>
#include <ambf_msgs/GhostObjectState.h>
#include <ambf_msgs/GhostObjectCmd.h>
#include <ambf_msgs/WorldState.h>
#include <ambf_msgs/WorldCmd.h>

#define AMBF_RAL_DEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define AMBF_RAL_INFO(...)  ROS_INFO(__VA_ARGS__)
#define AMBF_RAL_WARN(...)  ROS_WARN(__VA_ARGS__)
#define AMBF_RAL_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define AMBF_RAL_FATAL(...) ROS_FATAL(__VA_ARGS__)

#define AMBF_RAL_MSG(package, message) package::message
#define AMBF_RAL_MSG_PTR(package, message) package::message##Ptr
#define AMBF_RAL_MSG_MODIFIER(package, message) package::message##Modifier
#define AMBF_RAL_MSG_ITERATOR(package, message) package::message##Iterator

#define AMBF_RAL_PUBLISHER_PTR(type) std::shared_ptr<ros::Publisher>
#define AMBF_RAL_SUBSCRIBER_PTR(type) std::shared_ptr<ros::Subscriber>

namespace ambf_ral {
    typedef std::shared_ptr<ros::NodeHandle> node_ptr_t;
    typedef ros::Rate rate_t;
    typedef std::shared_ptr<rate_t> rate_ptr_t;
    typedef ros::Time time_t;
    typedef ros::Duration duration_t;

    inline std::string node_namespace(node_ptr_t node) {
        return node->getNamespace();
    }

    inline bool time_is_zero(const ros::Time & time) {
        return time.isZero();
    }

    inline double age_in_seconds(const ros::Time & time,
                                 node_ptr_t) {
        return (ros::Time::now() - time).toSec();
    }

    inline ros::Time now(node_ptr_t) {
        return ros::Time::now();
    }

    inline ros::Duration duration_from_seconds(const double & duration) {
        return ros::Duration(duration);
    }

    inline ros::Time time_from_seconds(const double & seconds) {
        ros::Time t;
        return t.fromSec(seconds);
    }

    inline void spin(node_ptr_t) {
        ros::spin();
    }

    inline void spin_some(node_ptr_t) {
    }

    inline void shutdown(void) {
        ros::shutdown();
    }

    inline ambf_ral::node_ptr_t create_node(const std::string & name) {
        // AMBF ROS1 shouldn't create nodes besides the one in afROSNode
        return nullptr;
    }

    template <typename _ros_t>
    void create_publisher(std::shared_ptr<ros::Publisher> & publisher,
                          node_ptr_t node,
                          const std::string & topic,
                          const size_t queue_size,
                          const bool latched) {
        std::string clean_topic = topic;
        ambf_ral::clean_namespace(clean_topic);
        publisher = std::make_shared<ros::Publisher>(node->advertise<_ros_t>(clean_topic, queue_size, latched));
        if (!publisher) {
            std::cerr << "Failed to create publisher for " << clean_topic << std::endl;
        }
    }

    template <typename _ros_t, typename _object_cb_t>
    void create_subscriber(std::shared_ptr<ros::Subscriber> & subscriber,
                           node_ptr_t node,
                           const std::string & topic,
                           const size_t queue_size,
                           void (_object_cb_t::*cb)(const _ros_t &),
                           _object_cb_t * instance
                           ) {
        std::string clean_topic = topic;
        ambf_ral::clean_namespace(clean_topic);
        subscriber = std::make_shared<ros::Subscriber>(node->subscribe(clean_topic, queue_size, cb, instance));
        if (!subscriber) {
            std::cerr << "Failed to create subscriber for " << clean_topic << std::endl;
        }
    }

    template <typename _ros_t, typename _object_cb_t>
    void create_subscriber(std::shared_ptr<ros::Subscriber> & subscriber,
                           node_ptr_t node,
                           const std::string & topic,
                           const size_t queue_size,
                           void (_object_cb_t::*cb)(const typename _ros_t::Ptr),
                           _object_cb_t * instance
                           ) {
        std::string clean_topic = topic;
        ambf_ral::clean_namespace(clean_topic);
        subscriber = std::make_shared<ros::Subscriber>(node->subscribe(clean_topic, queue_size, cb, instance));
        if (!subscriber) {
            std::cerr << "Failed to create subscriber for " << clean_topic << std::endl;
        }
    }

    template <typename _ros_t>
    void set_parameter(node_ptr_t node,
                       const std::string & name,
                       const _ros_t & value
                       ) {
        node->setParam(name, value);
    }

    template <typename _ros_t>
    bool get_parameter(node_ptr_t node,
                       const std::string & name,
                       _ros_t & value
                       ) {
        return node->getParamCached(name, value);
    }

    template <typename _pub_t>
    inline size_t nb_subscribers(_pub_t publisher) {
        return publisher->getNumSubscribers();
    }

    template <typename _pub_t>
    inline void publisher_shutdown(_pub_t publisher) {
        publisher->shutdown();
    }

    template <typename _pub_t>
    inline void subscriber_shutdown(_pub_t subscriber) {
        subscriber->shutdown();
    }
}

#elif AMBF_ROS2

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#if (AMBF_ROS_DISTRO == AMBF_ROS_GALACTIC)
  #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <ambf_msgs/msg/object_state.hpp>
#include <ambf_msgs/msg/object_cmd.hpp>
#include <ambf_msgs/msg/light_state.hpp>
#include <ambf_msgs/msg/light_cmd.hpp>
#include <ambf_msgs/msg/vehicle_state.hpp>
#include <ambf_msgs/msg/vehicle_cmd.hpp>
#include <ambf_msgs/msg/actuator_state.hpp>
#include <ambf_msgs/msg/actuator_cmd.hpp>
#include <ambf_msgs/msg/camera_state.hpp>
#include <ambf_msgs/msg/camera_cmd.hpp>
#include <ambf_msgs/msg/rigid_body_state.hpp>
#include <ambf_msgs/msg/rigid_body_cmd.hpp>
#include <ambf_msgs/msg/sensor_state.hpp>
#include <ambf_msgs/msg/sensor_cmd.hpp>
#include <ambf_msgs/msg/contact_data.hpp>
#include <ambf_msgs/msg/contact_event.hpp>
#include <ambf_msgs/msg/contact_sensor_state.hpp>
#include <ambf_msgs/msg/contact_sensor_cmd.hpp>
#include <ambf_msgs/msg/ghost_object_state.hpp>
#include <ambf_msgs/msg/ghost_object_cmd.hpp>
#include <ambf_msgs/msg/world_state.hpp>
#include <ambf_msgs/msg/world_cmd.hpp>

#define AMBF_RAL_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define AMBF_RAL_INFO(...)  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define AMBF_RAL_WARN(...)  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define AMBF_RAL_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), __VA_ARGS__)
#define AMBF_RAL_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), __VA_ARGS__)

#define AMBF_RAL_MSG(package, message) package::msg::message
#define AMBF_RAL_MSG_PTR(package, message) package::msg::message::SharedPtr
#define AMBF_RAL_MSG_MODIFIER(package, message) package::msg::message::Modifier
#define AMBF_RAL_MSG_ITERATOR(package, message) package::msg::message::Iterator

#define AMBF_RAL_PUBLISHER_PTR(type) typename rclcpp::Publisher<type>::SharedPtr
#define AMBF_RAL_SUBSCRIBER_PTR(type) typename rclcpp::Subscription<type>::SharedPtr

namespace ambf_ral {
    typedef std::shared_ptr<rclcpp::Node> node_ptr_t;
    typedef rclcpp::Rate rate_t;
    typedef std::shared_ptr<rate_t> rate_ptr_t;
    typedef rclcpp::Time time_t;
    typedef rclcpp::Duration duration_t;

    inline std::string node_namespace(node_ptr_t node) {
        return node->get_namespace();
    }

    inline bool time_is_zero(const rclcpp::Time & time) {
        return ((time.seconds() == 0)
                && (time.nanoseconds() == 0));
    }

    inline double age_in_seconds(const rclcpp::Time & time,
                                 node_ptr_t node) {
        return (node->get_clock()->now() - time).seconds();
    }

    inline rclcpp::Time now(node_ptr_t node) {
        return node->get_clock()->now();
    }

    inline rclcpp::Duration duration_from_seconds(const double & duration) {
        return rclcpp::Duration::from_seconds(duration);
    }

    inline rclcpp::Time time_from_seconds(const double & seconds) {
        return rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
    }

    inline void spin(node_ptr_t node) {
        rclcpp::spin(node);
    }

    inline void spin_some(node_ptr_t node) {
        rclcpp::spin_some(node);
    }

    inline void shutdown(void) {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    inline ambf_ral::node_ptr_t create_node(const std::string & name) {
        if (!rclcpp::ok()) {
            // create fake argc/argv
            std::string context_name = "ambf";
            typedef char * char_pointer;
            char_pointer * argv = new char_pointer[1];
            argv[0]= new char[context_name.size() + 1];
            strcpy(argv[0], context_name.c_str());
            int argc = 1;
            rclcpp::init(argc, argv);
        }
        return std::make_shared<rclcpp::Node>("ambf_" + name);
    }

    template <typename _ros_t>
    void create_publisher(typename rclcpp::Publisher<_ros_t>::SharedPtr & publisher,
                          node_ptr_t node,
                          const std::string & topic,
                          const size_t queue_size,
                          const bool latched) {
        std::string clean_topic = topic;
        ambf_ral::clean_namespace(clean_topic);
        rclcpp::QoS qos(queue_size);
        if (latched) {
            qos.transient_local();
        }
        publisher = node->create_publisher<_ros_t>(clean_topic, qos);
        if (!publisher) {
          std::cerr << "Failed to create publisher for " << clean_topic << std::endl;
        }
    }

    template <typename _ros_t, typename _object_cb_t>
    void create_subscriber(typename rclcpp::Subscription<_ros_t>::SharedPtr & subscriber,
                           node_ptr_t node,
                           const std::string & topic,
                           const size_t queue_size,
                           void (_object_cb_t::*cb)(const _ros_t &),
                           _object_cb_t * instance
                           ) {
        std::string clean_topic = topic;
        ambf_ral::clean_namespace(clean_topic);
        subscriber = node->create_subscription<_ros_t>(clean_topic,
                                                       queue_size,
                                                       std::bind(cb,
                                                                 instance,
                                                                 std::placeholders::_1));
        if (!subscriber) {
            std::cerr << "Failed to create subscriber for " << clean_topic << std::endl;
        }
    }

    template <typename _ros_t, typename _object_cb_t>
    void create_subscriber(typename rclcpp::Subscription<_ros_t>::SharedPtr & subscriber,
                           node_ptr_t node,
                           const std::string & topic,
                           const size_t queue_size,
                           void (_object_cb_t::*cb)(const typename _ros_t::SharedPtr),
                           _object_cb_t * instance
                           ) {
        std::string clean_topic = topic;
        ambf_ral::clean_namespace(clean_topic);
        subscriber = node->create_subscription<_ros_t>(clean_topic,
                                                       queue_size,
                                                       std::bind(cb,
                                                                 instance,
                                                                 std::placeholders::_1));
        if (!subscriber) {
            std::cerr << "Failed to create subscriber for " << clean_topic << std::endl;
        }
    }

    template <typename _ros_t>
    void set_parameter(node_ptr_t node,
                       const std::string & name,
                       const _ros_t & value
                       ) {
      if (!node->has_parameter(name)) {
          node->declare_parameter(name, value);
      }
      node->set_parameter(rclcpp::Parameter(name, value));
    }

    template <typename _ros_t>
    bool get_parameter(node_ptr_t node,
                       const std::string & name,
                       _ros_t & value
                       ) {
      return node->get_parameter(name, value);
    }

    template <typename _pub_t>
    inline size_t nb_subscribers(_pub_t publisher) {
        return publisher->get_subscription_count();
    }

    template <typename _pub_t>
    inline void publisher_shutdown(_pub_t & publisher) {
        publisher.reset();
    }

    template <typename _pub_t>
    inline void subscriber_shutdown(_pub_t & subscriber) {
        subscriber.reset();
    }
}

#endif // ROS2


namespace ambf_ral {

    inline void clean_namespace(std::string & _ros_namespace) {
#if AMBF_ROS1
        _ros_namespace = ros::names::clean(_ros_namespace);
#endif
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), ' ', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '-', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '.', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '(', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), ')', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), '[', '_');
        std::replace(_ros_namespace.begin(), _ros_namespace.end(), ']', '_');
        _ros_namespace = std::regex_replace(_ros_namespace, std::regex("//"), "/");
        _ros_namespace = std::regex_replace(_ros_namespace, std::regex("//"), "/");

    }

      inline void clean_nodename(std::string & _node_name) {
        clean_namespace(_node_name);
        std::replace(_node_name.begin(), _node_name.end(), '/', '_');
        while (_node_name.at(0) == '_') {
          _node_name.erase(0, 1);
        }
    }

    class ral
    {
    public:
        ral(int & argc, char * argv[], const std::string & node_name, bool anonymous_name = true);
        ral(const std::string & node_name, bool anonymous_name = true);
        ~ral();

        inline node_ptr_t node(void) {
            return m_node;
        }

        typedef std::vector<std::string> stripped_arguments_t;
        inline const stripped_arguments_t & stripped_arguments(void) const {
            return m_stripped_arguments;
        }

    protected:
        void init(int & argc,  char * argv[], const std::string & node_name, bool anonymous_name);
        std::string m_node_name;
        node_ptr_t m_node;
        stripped_arguments_t m_stripped_arguments;
    };
}

#endif // _ambf_ral_h
