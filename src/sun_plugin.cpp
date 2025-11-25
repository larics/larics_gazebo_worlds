#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace gazebo {

class SunDirectionPlugin : public WorldPlugin {
public:
    void Load(physics::WorldPtr _world, sdf::ElementPtr) override {
        // Init ROS
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "sun_direction_plugin", ros::init_options::NoSigintHandler);
        }
        nh.reset(new ros::NodeHandle("~"));
        sub = nh->subscribe("/sun_position", 1, &SunDirectionPlugin::sunCallback, this);

        // Init Gazebo transport
        gz_node = transport::NodePtr(new transport::Node());
        gz_node->Init();
        light_pub = gz_node->Advertise<gazebo::msgs::Light>("~/light/modify");

        ROS_INFO("SunDirectionPlugin loaded.");
    }

    void sunCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        
        double az = msg->data[0] * M_PI / 180.0;
        double el = msg->data[1] * M_PI / 180.0;


        // Gazebo expects FROM light TO ground
        ignition::math::Vector3d dir(
            -std::cos(el) * std::sin(az),
            -std::cos(el) * std::cos(az),
            -std::sin(el)
        );

        gazebo::msgs::Light light_msg;
        light_msg.set_name("sun");
        light_msg.set_type(gazebo::msgs::Light::DIRECTIONAL);
        gazebo::msgs::Set(light_msg.mutable_direction(), dir);

        light_pub->Publish(light_msg);
        ROS_INFO_STREAM("Updated sun direction to: " << dir);
    }

private:
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Subscriber sub;

    gazebo::transport::NodePtr gz_node;
    gazebo::transport::PublisherPtr light_pub;
};

GZ_REGISTER_WORLD_PLUGIN(SunDirectionPlugin)

}  // namespace gazebo
