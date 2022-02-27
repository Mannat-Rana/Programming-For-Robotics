#include <smb_highlevel_controller/SmbEStop.hpp>

namespace smb_estop {
    SmbEStop::SmbEStop(ros::NodeHandle& nh)
    : nh_{nh}
    {
        if (get_params()) 
        {
            scan_sub_ = nh_.subscribe(scan_topic_, scan_queue_, &SmbEStop::laserscan_callback, this);
            start_stop_client_ = nh_.serviceClient<std_srvs::SetBool>("/smb_highlevel_controller_node/" + start_stop_service_);
            ROS_INFO("Successfully initialized E-Stop Node");
        }
        else {
            ROS_FATAL("Could not initialize E-Stop Node");
        }
    }

    void SmbEStop::laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        double smallest_distance {scan->ranges.at(0)};
        // Find smallest value in scan
        for (const auto val : scan->ranges) {
            if (val < smallest_distance) {
                smallest_distance = val;
            }
        }
        // Call stop service if too close
        ROS_WARN_STREAM_THROTTLE(2, "Distance: " << smallest_distance);
        if (smallest_distance <= distance_limit_) {
            ROS_WARN_THROTTLE(2, "Obstacle within safe range, calling stop service!");
            std_srvs::SetBool srv;
            srv.request.data = true;
            start_stop_client_.call(srv);
            if (srv.response.success) {
                ROS_ERROR_STREAM_THROTTLE(2, srv.response.message);
            }
        }
    }

    // Get all params from server
    bool SmbEStop::get_params() {
        bool success {false};
        if (nh_.getParam("/smb_estop_node/scan_topic", scan_topic_) &&
            nh_.getParam("/smb_estop_node/scan_queue", scan_queue_) &&
            nh_.getParam("/smb_estop_node/start_stop_srv", start_stop_service_) &&
            nh_.getParam("/smb_estop_node/distance_limit", distance_limit_)) {
            success = true;
            ROS_INFO("Got all e-stop topics from parameter server!");
        }
        return success;
    }
}