#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <algorithm>
#include <iterator>

namespace smb_estop {
    class SmbEStop {
    private:
        ros::NodeHandle nh_;
        ros::ServiceClient start_stop_client_;
        ros::Subscriber scan_sub_;

        std::string scan_topic_;
        int scan_queue_;
        std::string start_stop_service_;
        double distance_limit_;

        /**
         * @brief Callback to process laserscan msg to check if bot is too close
         * 
         */
        void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr&);

        /**
         * @brief Get the params from the server
         * 
         * @return true if params loaded properly 
         * @return false otherwise
         */
        bool get_params();
    public:
        /**
         * @brief Construct a new Smb E Stop object
         * 
         */
        SmbEStop(ros::NodeHandle&);
    };
}