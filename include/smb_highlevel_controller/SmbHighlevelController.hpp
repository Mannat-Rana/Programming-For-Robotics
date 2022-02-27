#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <algorithm>
#include <iterator>
#include <cmath>

namespace smb_highlevel_controller {

    class SmbHighlevelController {
    private:
        // variables
        ros::NodeHandle nh_;

        ros::Subscriber scan_sub_;
        ros::Subscriber pc_sub_;
        ros::Publisher vel_pub_;
        ros::Publisher vis_pub_;
        ros::ServiceServer start_stop_server_;
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        std::string scan_topic_;
        int scan_queue_;
        std::string point_cloud_topic_;
        int point_cloud_queue_;
        std::string vel_topic_;
        int vel_queue_;
        std::string vis_topic_;
        int vis_queue_;
        std::string start_stop_service_;
        bool e_stop;
        double kp_linear_;
        double kp_angular_;
        double angular_deadband_;
        
        /**
         * @brief Callback to process laserscan to find closest obstacle
         * 
         */
        void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr&);
        
        /**
         * @brief Callback to process pc msg to print # of points in pc 
         * 
         */
        void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr&);
        
        /**
         * @brief Callback to process start stop requests, set data to true to stop
         * 
         * @return true if service executed properly
         * @return false otherwise
         */
        bool start_stop_callback(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&);
        
        /**
         * @brief Get the params from the server
         * 
         * @return true if params loaded properly
         * @return false otherwise
         */
        bool get_params();
        
        /**
         * @brief Send velocity messages from navigation
         * 
         */
        void send_vel_msg(double, double, double, double);
        
        /**
         * @brief Show pillar as green circle on RViz
         * 
         */
        void show_pillar_marker(double, double);
    public:
        /**
         * @brief Construct a new Smb Highlevel Controller object
         * 
         */
        SmbHighlevelController(ros::NodeHandle&);
    };
}
