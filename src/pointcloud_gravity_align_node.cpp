#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

using namespace ros;

Eigen::Matrix3f R_bw = Eigen::Matrix3f::Identity();
bool imu_inited = 0;

Subscriber cloud_sub;
Subscriber imu_sub;
Publisher cloud_pub;
Publisher imu_pub;

void stdcloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    if (!imu_inited) 
        return;
    
    static bool first_in = 1;
    if (first_in) {
        first_in = 0;
        printf("Point cloud frame id:%s\n", msg->header.frame_id.c_str());
    }

    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    int points_size = pcl_cloud.size();
    for (int i = 0; i < points_size; i++) {
        Eigen::Vector3f t;
        t(0) =  pcl_cloud.at(i).x;
        t(1) =  pcl_cloud.at(i).y;
        t(2) =  pcl_cloud.at(i).z;
        t = R_bw * t;
        pcl_cloud.at(i).x = t(0);
        pcl_cloud.at(i).y = t(1);
        pcl_cloud.at(i).z = t(2);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_pub.publish(cloud_msg);
}

void livoxcloudCB(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    if (!imu_inited) 
        return;

    static bool first_in = 1;
    if (first_in) {
        first_in = 0;
        printf("Point cloud frame id:%s\n", msg->header.frame_id.c_str());
    }

    livox_ros_driver::CustomMsg cloud_msg = *msg;
    int points_size = cloud_msg.point_num;
    for (int i = 0; i < points_size; i++) {
        Eigen::Vector3f t;
        t(0) =  cloud_msg.points[i].x;
        t(1) =  cloud_msg.points[i].y;
        t(2) =  cloud_msg.points[i].z;
        t = R_bw * t;
        cloud_msg.points[i].x = t(0);
        cloud_msg.points[i].y = t(1);
        cloud_msg.points[i].z = t(2);
    }
    cloud_pub.publish(cloud_msg);
}

void imuCB(const sensor_msgs::Imu::ConstPtr &msg) {
    static double first_time = -1.0;
    static Eigen::Vector3f acc_avg(0, 0, 0);
    static int iter = 0;
    // Get first time stamp
    if (first_time < 0) {
        printf("Wait 3 seconds to calibrate the gravity...\n");
        first_time = msg->header.stamp.toSec();
        acc_avg(0) = msg->linear_acceleration.x;
        acc_avg(1) = msg->linear_acceleration.y;
        acc_avg(2) = msg->linear_acceleration.z;
        iter++;
        return;
    }
    // Average filter, get direction of gravity and rotation from body frame to robot frame
    if (!imu_inited && msg->header.stamp.toSec() - first_time < 3.0) {
        acc_avg(0) += msg->linear_acceleration.x;
        acc_avg(1) += msg->linear_acceleration.y;
        acc_avg(2) += msg->linear_acceleration.z;
        iter++;
        return;
    } else if (!imu_inited) {
        imu_inited = 1;
        acc_avg /= iter;
        Eigen::Vector3f G(0, 0, acc_avg.norm());
        Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(acc_avg, G);
        R_bw = q.toRotationMatrix();
        Eigen::Vector3f eulerAngle = R_bw.eulerAngles(2,1,0);
        printf("================Start publishing aligned imu and poitncloud.================\n");
        printf("Graviry: %.2fg\n", G(2));
        std::cout << "RPY Euler angle (rad):\n" << eulerAngle << std::endl;
        printf("IMU frame id:%s\n", msg->header.frame_id.c_str());
    }

    sensor_msgs::Imu imu_msg = *msg;
    // Rotate linear acceleration
    Eigen::Vector3f acc;
    acc(0) = imu_msg.linear_acceleration.x;
    acc(1) = imu_msg.linear_acceleration.y;
    acc(2) = imu_msg.linear_acceleration.z;
    acc = R_bw * acc;
    imu_msg.linear_acceleration.x = acc(0);
    imu_msg.linear_acceleration.y = acc(1);
    imu_msg.linear_acceleration.z = acc(2);
    // Rotate angular velocity
    Eigen::Vector3f vel;
    vel(0) = imu_msg.angular_velocity.x;
    vel(1) = imu_msg.angular_velocity.y;
    vel(2) = imu_msg.angular_velocity.z;
    vel = R_bw * vel;
    imu_msg.angular_velocity.x = vel(0);
    imu_msg.angular_velocity.y = vel(1);
    imu_msg.angular_velocity.z = vel(2);
    
    imu_pub.publish(imu_msg);
}

int main(int argc, char **argv) {

    init(argc, argv, "pointcloud_gravity_align");
    NodeHandle n;

    int lidar_type;
    std::string sub_lidar_topic, sub_imu_topic, pub_lidar_topic, pub_imu_topic;
    n.param<int>("lidar_type", lidar_type, 1);
    n.param<std::string>("sub_lidar_topic", sub_lidar_topic, "/livox/lidar");
    n.param<std::string>("sub_imu_topic", sub_imu_topic, "/livox/imu");
    n.param<std::string>("pub_lidar_topic", pub_lidar_topic, "/livox/lidar_aligned");
    n.param<std::string>("pub_imu_topic", pub_imu_topic, "/livox/imu_aligned");

    imu_sub = n.subscribe(sub_imu_topic, 10, imuCB);
    imu_pub = n.advertise<sensor_msgs::Imu>(pub_imu_topic, 10);
    // Standard point cloud
    if (lidar_type == 0) {
        cloud_sub = n.subscribe(sub_lidar_topic, 10, stdcloudCB);
        cloud_pub = n.advertise<sensor_msgs::PointCloud2>(pub_lidar_topic, 10);
        std::cout << "Point cloud type: standard" << std::endl;
    }
    // Livox point cloud
    else if (lidar_type == 1) {
        cloud_sub = n.subscribe(sub_lidar_topic, 10, livoxcloudCB);
        cloud_pub = n.advertise<livox_ros_driver::CustomMsg>(pub_lidar_topic, 10);
        std::cout << "Point cloud type: livox" << std::endl;
    }
    else {
        ROS_ERROR("Wrong parameter: lidar_type = %d", lidar_type);
    }
    
    spin();
    return 0;
}