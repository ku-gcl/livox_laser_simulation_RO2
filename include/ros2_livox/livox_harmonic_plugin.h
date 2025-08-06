#ifndef ROS2_LIVOX_HARMONIC_PLUGIN_H
#define ROS2_LIVOX_HARMONIC_PLUGIN_H

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Util.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#ifdef LIVOX_ROS_DRIVER2_AVAILABLE
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

#include "ros2_livox/csv_reader.hpp"

namespace ros2_livox
{
    struct AviaRotateInfo
    {
        double time;
        double azimuth;
        double zenith;
    };

    class LivoxHarmonicPlugin
        : public gz::sim::System,
          public gz::sim::ISystemConfigure,
          public gz::sim::ISystemPostUpdate
    {
    public:
        LivoxHarmonicPlugin();
        ~LivoxHarmonicPlugin() override;

        void Configure(const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_eventMgr) override;

        void PostUpdate(const gz::sim::UpdateInfo &_info,
                       const gz::sim::EntityComponentManager &_ecm) override;

    private:
        void OnLaserScan(const gz::msgs::LaserScan &_msg);
        void PublishPointCloud(const gz::msgs::LaserScan &_msg);
#ifdef LIVOX_ROS_DRIVER2_AVAILABLE
        void PublishCustomMsg(const gz::msgs::LaserScan &_msg);
#endif

        // ROS 2 node
        rclcpp::Node::SharedPtr ros_node_;
        
        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_pub_;
#ifdef LIVOX_ROS_DRIVER2_AVAILABLE
        rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
#endif

        // Gazebo transport
        gz::transport::Node gz_node_;
        std::string laser_scan_topic_;
        
        // Configuration
        std::string frame_id_;
        std::string topic_name_;
        std::vector<AviaRotateInfo> avia_infos_;
        
        size_t samples_step_ = 100;
        size_t down_sample_ = 1;
        size_t max_point_size_ = 1000;
        size_t curr_start_index_ = 0;
        
        double min_range_ = 0.1;
        double max_range_ = 400.0;
        
        bool initialized_ = false;
    };
}

#endif // ROS2_LIVOX_HARMONIC_PLUGIN_H