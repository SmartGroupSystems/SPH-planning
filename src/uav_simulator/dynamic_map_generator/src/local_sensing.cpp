#include <ros/ros.h>
#include <ros/master.h>
#include <regex>
#include <map>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

struct ParticleData {
    ros::Subscriber odom_sub;
    ros::Publisher local_map_pub;
    Eigen::Vector3d position;
};

std::map<int, ParticleData> particles_map;
pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
bool has_map = false;

// 地图参数
double resolution, x_size, y_size, z_size;
Eigen::Vector3d local_range;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int particle_id) {
    particles_map[particle_id].position = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
}

void checkForNewParticles(ros::NodeHandle& nh) {
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);

    std::regex topic_pattern("/particle(\\d+)/odom");
    std::smatch match;

    for (const auto& topic : topic_list) {
        if (std::regex_match(topic.name, match, topic_pattern)) {
            int id = std::stoi(match[1].str());

            // 如果是新粒子，则添加订阅器和发布器
            if (particles_map.find(id) == particles_map.end()) {
                std::string sub_topic = topic.name;
                std::string pub_topic = "/particle" + std::to_string(id) + "/local_map";

                particles_map[id].odom_sub = nh.subscribe<nav_msgs::Odometry>(
                    sub_topic, 1, boost::bind(&odomCallback, _1, id));
                particles_map[id].local_map_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_topic, 1);

                ROS_INFO_STREAM("[local_sensing] New particle detected: id=" << id);
            }
        }
    }
}

void mockMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!has_map)
    {
        pcl::fromROSMsg(*msg, *full_cloud);
        kdtree.setInputCloud(full_cloud);
        has_map = true;
        ROS_INFO("[local_sensing] Mock map received with %lu points.", full_cloud->points.size());
    }
}

void pubLocalMaps() {
    if (!has_map) return;

    for (auto& kv : particles_map) {
        int id = kv.first;
        auto& pdata = kv.second;

        pcl::PointCloud<pcl::PointXYZ> localMap;
        pcl::PointXYZ center(pdata.position.x(), pdata.position.y(), pdata.position.z());

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        double sensing_radius = local_range.norm() / 2.0; // 简单近似

        if (kdtree.radiusSearch(center, sensing_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                localMap.points.push_back(full_cloud->points[pointIdxRadiusSearch[i]]);
            }

            localMap.width = localMap.points.size();
            localMap.height = 1;
            localMap.is_dense = true;

            sensor_msgs::PointCloud2 localMapMsg;
            pcl::toROSMsg(localMap, localMapMsg);
            localMapMsg.header.frame_id = "world";
            localMapMsg.header.stamp = ros::Time::now();

            pdata.local_map_pub.publish(localMapMsg);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_sensing_node");
    ros::NodeHandle nh("~");

    // 获取地图参数
    nh.param("sdf_map/resolution", resolution, -1.0);
    nh.param("sdf_map/map_size_x", x_size, -1.0);
    nh.param("sdf_map/map_size_y", y_size, -1.0);
    nh.param("sdf_map/map_size_z", z_size, -1.0);
    nh.param("sdf_map/local_update_range_x", local_range(0), -1.0);
    nh.param("sdf_map/local_update_range_y", local_range(1), -1.0);
    nh.param("sdf_map/local_update_range_z", local_range(2), -1.0);

    // 订阅全局地图
    ros::Subscriber map_sub = nh.subscribe("/mock_map", 1, mockMapCallback);

    // 动态订阅所有 particle odom
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);

    std::regex topic_pattern("/particle(\\d+)/odom");
    std::smatch match;

    for (const auto& topic : topic_list) {
        if (std::regex_match(topic.name, match, topic_pattern)) {
            int id = std::stoi(match[1].str());
            ROS_INFO_STREAM("Found particle odom: " << topic.name << ", id = " << id);

            // 创建订阅器和发布器
            std::string sub_topic = topic.name;
            std::string pub_topic = "/particle" + std::to_string(id) + "/local_map";

            // 使用 boost::bind 带参数传入回调
            particles_map[id].odom_sub = nh.subscribe<nav_msgs::Odometry>(
                sub_topic, 1, boost::bind(&odomCallback, _1, id));
            particles_map[id].local_map_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_topic, 1);
        }
    }

    ros::Rate rate(10.0); // 10Hz 发布 local_map
    int loop_count = 0;

    while (ros::ok()) {
        ros::spinOnce();
        pubLocalMaps();

        // 每1秒检查一次新增粒子（10Hz -> 每10次）
        if (loop_count % 10 == 0) {
            checkForNewParticles(nh);
        }

        loop_count++;
        rate.sleep();
    }

    return 0;
}
