#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/SetBool.h>
#include <string>
#include <algorithm>
#include <iterator>
#include <math.h>

namespace smb_crash_detector {
    class SmbCrashDetector {
    private:
        ros::NodeHandle nh_;
        ros::ServiceClient start_stop_client_;
        ros::Subscriber imu_sub_;

        std::string imu_topic_;
        int imu_queue_;
        std::string start_stop_service_;
        double accel_change_;
        double prev_accel_x_;
        double prev_accel_y_;
        bool crashed_;

        /**
         * @brief Callback to process the imu data to check for crashes
         */
        void imu_callback(const sensor_msgs::Imu::ConstPtr&);
        
        /**
         * @brief Get the params from server
         * 
         * @return true if all params loaded correctly
         * @return false otherwise
         */
        
        bool get_params();
    public:
        /**
         * @brief Construct a new Smb Crash Detector object
         * 
         */
        SmbCrashDetector(ros::NodeHandle&);
    };
}