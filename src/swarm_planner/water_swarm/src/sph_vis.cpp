#include "sph_zhang_3d.h"

// 用于存储粒子轨迹
std::map<int, std::vector<geometry_msgs::Point>> particleTrajectories;
std::map<int, visualization_msgs::Marker> persistentMarkers; // 用于存储所有显示的粒子

// 用于发布的粒子位置和轨迹
ros::Publisher marker_publisher;
ros::Publisher trajectory_publisher;

int global_marker_id = 0; // 用于唯一标识每个粒子标记的全局 ID

// 颜色渐变生成器（用于彩虹色）
std_msgs::ColorRGBA generateRainbowColor(int index, int total) {
    std_msgs::ColorRGBA color;
    float ratio = static_cast<float>(index) / static_cast<float>(total);
    int phase = static_cast<int>(ratio * 360);

    // 彩虹色算法，基于相位选择颜色
    float r, g, b;
    if (phase < 60) {
        r = 1.0;
        g = phase / 60.0;
        b = 0.0;
    } else if (phase < 120) {
        r = (120 - phase) / 60.0;
        g = 1.0;
        b = 0.0;
    } else if (phase < 180) {
        r = 0.0;
        g = 1.0;
        b = (phase - 120) / 60.0;
    } else if (phase < 240) {
        r = 0.0;
        g = (240 - phase) / 60.0;
        b = 1.0;
    } else if (phase < 300) {
        r = (phase - 240) / 60.0;
        g = 0.0;
        b = 1.0;
    } else {
        r = 1.0;
        g = 0.0;
        b = (360 - phase) / 60.0;
    }

    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0;  // 透明度为 1.0
    return color;
}

// 处理收到的粒子信息
void swarmParticlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg) {
    static ros::Time last_update_time = ros::Time::now(); 
    int count = 0;
    for (const auto& particle : msg->particles) {
        // 创建或更新粒子的标记
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "particles";
        marker.id = global_marker_id++;  // 使用全局 ID 确保唯一性
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // 设置粒子的位置
        marker.pose.position.x = particle.position.x;
        marker.pose.position.y = particle.position.y;
        marker.pose.position.z = particle.position.z;
        marker.pose.orientation.w = 1.0;

        // 设置粒子的显示大小和颜色
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color = generateRainbowColor(msg->header.stamp.sec % 10, 10); // 根据时间变化颜色

        // 更新或添加到持久标记集合中
        // 每隔8秒更新持久标记
        if (ros::Time::now() - last_update_time >= ros::Duration(6.0)) {
            count ++;
            persistentMarkers[particle.index] = marker;
            if (count == msg->particles.size())
            {
                last_update_time = ros::Time::now();  // 更新上次更新时间
            } 
        }

        // 记录粒子轨迹
        geometry_msgs::Point point;
        point.x = particle.position.x;
        point.y = particle.position.y;
        point.z = particle.position.z;
        
        // 确保粒子轨迹持久化
        particleTrajectories[particle.index].push_back(point);
    }
    
}

// 定时发布粒子和轨迹
void publishParticlesAndTrajectories() {
    visualization_msgs::MarkerArray marker_array;

    // 添加所有持久粒子标记
    for (const auto& entry : persistentMarkers) {
        marker_array.markers.push_back(entry.second);
    }

    // 发布粒子标记
    marker_publisher.publish(marker_array);

    // 发布粒子轨迹
    visualization_msgs::MarkerArray trajectory_array;
    int total_particles = particleTrajectories.size();
    int color_index = 0;

    for (const auto& entry : particleTrajectories) {
        int index = entry.first;
        const std::vector<geometry_msgs::Point>& trajectory = entry.second;

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "world";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "particle_trajectories";
        line_strip.id = index;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;

        // 设置轨迹的大小
        line_strip.scale.x = 0.10;  // 线条宽度

        // 使用彩虹色显示轨迹
        std_msgs::ColorRGBA color = generateRainbowColor(color_index++, total_particles);
        line_strip.color = color;

        // 设置轨迹的点
        line_strip.points = trajectory;
        line_strip.lifetime = ros::Duration(0);  // 轨迹永久显示

        trajectory_array.markers.push_back(line_strip);
    }

    // 发布轨迹
    trajectory_publisher.publish(trajectory_array);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sph_visualizer");
    ros::NodeHandle nh;

    // 订阅 /swarm_particles 话题
    ros::Subscriber swarm_subscriber = nh.subscribe("/swarm_particles", 10, swarmParticlesCallback);

    // 创建发布器
    marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("/visualization_particles", 10);
    trajectory_publisher = nh.advertise<visualization_msgs::MarkerArray>("/visualization_trajectories", 10);

    // 设置 8 秒的定时器
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent&) {
        // ROS_INFO("Publishing particle markers every 0.1 seconds");
        publishParticlesAndTrajectories();  // 每 0.1 秒发布一次粒子和轨迹
    });

    ros::spin();
    return 0;
}
