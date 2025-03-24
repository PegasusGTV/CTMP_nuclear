#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/filesystem.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <ctmp_single/ctmp_utils.hpp>

void visualizeRegions(const std::vector<region>& regions, ros::Publisher& vis_pub) {
    visualization_msgs::MarkerArray marker_array;
    for (const auto& reg : regions) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom_combined"; // Ensure this matches your RViz fixed frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "regions";
        marker.id = reg.id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = reg.state[0];
        marker.pose.position.y = reg.state[1];
        marker.pose.position.z = reg.state[2];
        marker.scale.x = reg.radius * 2;
        marker.scale.y = reg.radius * 2;
        marker.scale.z = reg.radius * 2;
        marker.color.a = 0.5; // Transparency
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0); // Make the marker persist indefinitely
        marker_array.markers.push_back(marker);
        ROS_INFO("Publishing region marker: id=%d, position=(%f, %f, %f), scale=%f", reg.id, reg.state[0], reg.state[1], reg.state[2], reg.radius * 2);
    }
    vis_pub.publish(marker_array);
}

void visualizeCluster(ros::Publisher& vis_pub) {
    visualization_msgs::MarkerArray marker_array;
    for (int i = 0; i < 10; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom_combined"; // Ensure this matches your RViz fixed frame
        marker.header.stamp = ros::Time::now();
        marker.ns = "cluster";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 1.0 + 0.01 * i;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; // Opaque
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0); // Make the marker persist indefinitely
        marker_array.markers.push_back(marker);
        ROS_INFO("Publishing cluster marker: id=%d, position=(%f, %f, %f)", i, 1.0 + 0.01 * i, 0.0, 0.0);
    }
    vis_pub.publish(marker_array);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "visualize_regions");
    ros::NodeHandle nh;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    std::string current_dir = ros::package::getPath("ctmp_single");
    std::string read_write_dir = current_dir + "/data/";
    std::string read_write_path = read_write_dir + "manipulator_1" + "_place.dat";
    std::string file_path = read_write_path;
    std::vector<region> regions;

    try {
        boost::filesystem::path myFile = file_path;
        boost::filesystem::ifstream ifs(myFile);
        boost::archive::text_iarchive ta(ifs);
        ta >> regions;
    } catch (...) {
        ROS_WARN("Unable to read preprocessed file");
        return -1;
    }

    visualizeRegions(regions, vis_pub);
    visualizeCluster(vis_pub);

    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();

    return 0;
}