#include <smb_highlevel_controller/SmbCrashDetector.hpp>

namespace smb_crash_detector {
    SmbCrashDetector::SmbCrashDetector(ros::NodeHandle& nh)
    : nh_{nh}, prev_accel_x_{0}, prev_accel_y_{0}, crashed_{false}
    {
        if (get_params()) 
        {
            imu_sub_ = nh_.subscribe(imu_topic_, imu_queue_, &SmbCrashDetector::imu_callback, this);
            start_stop_client_ = nh_.serviceClient<std_srvs::SetBool>("/smb_highlevel_controller_node/" + start_stop_service_);
            ROS_INFO("Successfully initialized Crash Detector Node");
        }
        else {
            ROS_FATAL("Could not initialize Crash Detector Node");
        }
    }

    void SmbCrashDetector::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_data) {
        double current_accel_x {imu_data->linear_acceleration.x};
        double current_accel_y {imu_data->linear_acceleration.y};
        // Detect crash if there is big change in accel on xy plane
        if (sqrt(pow(current_accel_x - prev_accel_x_, 2) + pow(current_accel_y - prev_accel_y_, 2)) >= accel_change_) {
            crashed_ = true;
        }
        prev_accel_x_ = current_accel_x; prev_accel_y_ = current_accel_y;
        // Call stop service if crash detected
        if (crashed_) {
            ROS_ERROR_THROTTLE(2, "Crash detected, calling stop service!");
            std_srvs::SetBool srv;
            srv.request.data = true;
            start_stop_client_.call(srv);
            if (srv.response.success) {
                ROS_ERROR_STREAM_THROTTLE(2, srv.response.message);
            }
        }
    }

    bool SmbCrashDetector::get_params() {
        bool success {false};
        // Get params from server
        if (nh_.getParam("/smb_crash_detector_node/imu_topic", imu_topic_) &&
            nh_.getParam("/smb_crash_detector_node/imu_queue", imu_queue_) &&
            nh_.getParam("/smb_crash_detector_node/start_stop_srv", start_stop_service_) &&
            nh_.getParam("/smb_crash_detector_node/accel_change", accel_change_)) {
            success = true;
            ROS_INFO("Got all crash detector topics from parameter server!");
        }
        return success;
    }
}