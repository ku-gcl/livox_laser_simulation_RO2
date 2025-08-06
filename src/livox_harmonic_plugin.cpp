#include "ros2_livox/livox_harmonic_plugin.h"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/common/Console.hh>

using namespace ros2_livox;

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, 
                           std::vector<AviaRotateInfo> &avia_infos)
{
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (const auto &data : datas)
    {
        if (data.size() == 3)
        {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;
        }
        else
        {
            gzwarn << "Data size is not 3!" << std::endl;
        }
    }
}

LivoxHarmonicPlugin::LivoxHarmonicPlugin()
{
}

LivoxHarmonicPlugin::~LivoxHarmonicPlugin()
{
}

void LivoxHarmonicPlugin::Configure(const gz::sim::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  gz::sim::EntityComponentManager &_ecm,
                                  gz::sim::EventManager &_eventMgr)
{
    // Initialize ROS 2 node
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }
    
    ros_node_ = rclcpp::Node::make_shared("livox_harmonic_plugin");

    // Get parameters from SDF
    if (_sdf->HasElement("csv_file_name"))
    {
        std::string csv_file_name = _sdf->Get<std::string>("csv_file_name");
        gzdbg << "Loading CSV file: " << csv_file_name << std::endl;
        
        std::vector<std::vector<double>> datas;
        if (!CsvReader::ReadCsvFile(csv_file_name, datas))
        {
            gzerr << "Cannot read CSV file: " << csv_file_name << std::endl;
            return;
        }
        
        convertDataToRotateInfo(datas, avia_infos_);
        max_point_size_ = avia_infos_.size();
        gzdbg << "Loaded " << avia_infos_.size() << " scan points" << std::endl;
    }

    if (_sdf->HasElement("topic"))
    {
        topic_name_ = _sdf->Get<std::string>("topic");
    }
    else
    {
        topic_name_ = "/livox/lidar";
    }

    if (_sdf->HasElement("samples"))
    {
        samples_step_ = _sdf->Get<size_t>("samples");
    }

    if (_sdf->HasElement("downsample"))
    {
        down_sample_ = _sdf->Get<size_t>("downsample");
        if (down_sample_ < 1)
        {
            down_sample_ = 1;
        }
    }

    // Get sensor name for frame_id
    auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
    if (nameComp)
    {
        frame_id_ = nameComp->Data();
    }
    else
    {
        frame_id_ = "lidar_link";
    }

    // Create ROS 2 publishers
    cloud2_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_name_ + "_PointCloud2", 10);
#ifdef LIVOX_ROS_DRIVER2_AVAILABLE
    custom_pub_ = ros_node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(
        topic_name_, 10);
#endif

    // Set up Gazebo transport to listen for laser scan data
    laser_scan_topic_ = "/world/default/" + frame_id_ + "/laser_scan";
    if (!gz_node_.Subscribe(laser_scan_topic_, &LivoxHarmonicPlugin::OnLaserScan, this))
    {
        gzerr << "Error subscribing to topic [" << laser_scan_topic_ << "]" << std::endl;
    }

    gzdbg << "LivoxHarmonicPlugin configured successfully" << std::endl;
    initialized_ = true;
}

void LivoxHarmonicPlugin::PostUpdate(const gz::sim::UpdateInfo &_info,
                                   const gz::sim::EntityComponentManager &_ecm)
{
    if (!initialized_)
        return;

    // Spin ROS 2 node to process callbacks
    rclcpp::spin_some(ros_node_);
}

void LivoxHarmonicPlugin::OnLaserScan(const gz::msgs::LaserScan &_msg)
{
    if (!initialized_)
        return;

    PublishPointCloud(_msg);
#ifdef LIVOX_ROS_DRIVER2_AVAILABLE
    PublishCustomMsg(_msg);
#endif
}

void LivoxHarmonicPlugin::PublishPointCloud(const gz::msgs::LaserScan &_msg)
{
    sensor_msgs::msg::PointCloud cloud;
    cloud.header.stamp = ros_node_->get_clock()->now();
    cloud.header.frame_id = frame_id_;

    size_t range_count = _msg.ranges_size();
    cloud.points.reserve(range_count);

    for (size_t i = 0; i < range_count; ++i)
    {
        double range = _msg.ranges(i);
        
        // Skip invalid ranges
        if (range <= min_range_ || range >= max_range_)
            continue;

        // Calculate point based on avia info
        size_t avia_index = (curr_start_index_ + i * down_sample_) % max_point_size_;
        if (avia_index >= avia_infos_.size())
            continue;

        const auto &rotate_info = avia_infos_[avia_index];
        
        double x = range * cos(rotate_info.zenith) * cos(rotate_info.azimuth);
        double y = range * cos(rotate_info.zenith) * sin(rotate_info.azimuth);
        double z = range * sin(rotate_info.zenith);

        geometry_msgs::msg::Point32 point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud.points.push_back(point);
    }

    // Convert to PointCloud2 and publish
    sensor_msgs::msg::PointCloud2 cloud2;
    sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
    cloud2.header = cloud.header;
    cloud2_pub_->publish(cloud2);

    curr_start_index_ += samples_step_;
}

#ifdef LIVOX_ROS_DRIVER2_AVAILABLE
void LivoxHarmonicPlugin::PublishCustomMsg(const gz::msgs::LaserScan &_msg)
{
    livox_ros_driver2::msg::CustomMsg custom_msg;
    custom_msg.header.stamp = ros_node_->get_clock()->now();
    custom_msg.header.frame_id = frame_id_;

    size_t range_count = _msg.ranges_size();
    custom_msg.points.reserve(range_count);

    for (size_t i = 0; i < range_count; ++i)
    {
        double range = _msg.ranges(i);
        double intensity = (i < _msg.intensities_size()) ? _msg.intensities(i) : 0.0;
        
        // Skip invalid ranges
        if (range <= min_range_ || range >= max_range_)
            continue;

        // Calculate point based on avia info
        size_t avia_index = (curr_start_index_ + i * down_sample_) % max_point_size_;
        if (avia_index >= avia_infos_.size())
            continue;

        const auto &rotate_info = avia_infos_[avia_index];
        
        double x = range * cos(rotate_info.zenith) * cos(rotate_info.azimuth);
        double y = range * cos(rotate_info.zenith) * sin(rotate_info.azimuth);
        double z = range * sin(rotate_info.zenith);

        livox_ros_driver2::msg::CustomPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.reflectivity = intensity;
        point.offset_time = rotate_info.time;

        custom_msg.points.push_back(point);
    }

    custom_msg.point_num = custom_msg.points.size();
    custom_pub_->publish(custom_msg);
}
#endif

// Register the plugin
GZ_ADD_PLUGIN(ros2_livox::LivoxHarmonicPlugin,
              gz::sim::System,
              ros2_livox::LivoxHarmonicPlugin::ISystemConfigure,
              ros2_livox::LivoxHarmonicPlugin::ISystemPostUpdate)