#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

    SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nh) 
    :   nh_{nh}, tfListener{tfBuffer}, e_stop{false}
    {
        if (get_params())
        {
            ROS_INFO("Successfully intialized controller");
            scan_sub_ = nh_.subscribe(scan_topic_, scan_queue_, &SmbHighlevelController::laserscan_callback, this);
            pc_sub_ = nh_.subscribe(point_cloud_topic_, point_cloud_queue_, &SmbHighlevelController::pointcloud_callback, this);
            vel_pub_ = nh_.advertise<geometry_msgs::Twist>(vel_topic_, vel_queue_);
            vis_pub_ = nh_.advertise<visualization_msgs::Marker>(vis_topic_, vis_queue_);
            start_stop_server_ = nh_.advertiseService(start_stop_service_, &SmbHighlevelController::start_stop_callback, this);
        }
        else {
            ROS_ERROR("Could not initialize controller");
        }
    }

    bool SmbHighlevelController::start_stop_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp) {
        // If service is called to stop, stop, else, reset e_stop bool
        if (req.data) {
            e_stop = true;
            resp.message = "Stopped SMB!";
        }
        else {
            e_stop = false;
            resp.message = "Re-enabled SMB";
        }
        resp.success = true;
        return true;
    }

    void SmbHighlevelController::send_vel_msg(double angle, double x_distance, double y_distance, double distance) {
        geometry_msgs::Twist vel_msg{};
        // Stop navigating if e-stopped, continue otherwise
        // Start with adjusting angle then go straight once angle is correct
        if (!e_stop) {
            if (angle > angular_deadband_) {
                vel_msg.angular.z = angle * kp_angular_;
            }
            else {
                vel_msg.angular.z = 0;
                vel_msg.linear.x = distance * kp_linear_;
            }  
        }
        else {
            vel_msg.angular.z = 0; 
            vel_msg.linear.x = 0;
        }
        vel_pub_.publish(vel_msg);
    }

    void SmbHighlevelController::show_pillar_marker(double x_distance, double y_distance) {
        geometry_msgs::TransformStamped transform{};
        // Get transform from lidar to odom
        transform = tfBuffer.lookupTransform("odom", "rslidar_base_link", ros::Time(0));
        geometry_msgs::Vector3 original {};
        original.x = x_distance; original.y = y_distance; original.z = 0;
        geometry_msgs::Vector3 converted {};
        // Make the conversion to odom frame
        tf2::doTransform(original, converted, transform);
        visualization_msgs::Marker marker{};
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "smb_highlevel_controller";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = converted.x + transform.transform.translation.x;
        marker.pose.position.y = converted.y + transform.transform.translation.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; 
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        // Publish green circle wrt odom frame
        vis_pub_.publish(marker);
    }

    void SmbHighlevelController::laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {       
        double distance{};
        do {
            // Find min distance in scan
            double smallest_distance{scan->ranges.at(0)};
            int min_index {0};
            int counter {0};
            for (const auto val : scan->ranges) {
                if (val < smallest_distance) {
                    smallest_distance = val;
                    min_index = counter;
                }
                counter++;
            }
            // Find angle by incrementing scan count with angle increment
            double angle {scan->angle_min + (min_index * scan->angle_increment)};
            double x_distance {smallest_distance * cos(angle)};
            double y_distance {smallest_distance * sin(angle)};
            distance = sqrt((x_distance * x_distance) + (y_distance * y_distance));
            send_vel_msg(angle, x_distance, y_distance, distance);
            try {
                show_pillar_marker(x_distance, y_distance);
            }
            catch (tf2::TransformException& e) {
                ROS_WARN("%s", e.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            ros::spinOnce();
        } while (distance > 1);            
    }

    void SmbHighlevelController::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pc) {
        // Print # of points in each point cloud msg
        ROS_INFO_STREAM_THROTTLE(2, "Points in Point Cloud: " << pc->data.size());
    }

    bool SmbHighlevelController::get_params() {
        bool success {false};
        // Get params from server
        if (nh_.getParam("/smb_highlevel_controller_node/scan_topic", scan_topic_) &&
            nh_.getParam("/smb_highlevel_controller_node/scan_queue", scan_queue_) &&
            nh_.getParam("/smb_highlevel_controller_node/point_cloud_queue", point_cloud_queue_) &&
            nh_.getParam("/smb_highlevel_controller_node/point_cloud_topic", point_cloud_topic_) &&
            nh_.getParam("/smb_highlevel_controller_node/vel_topic", vel_topic_) &&
            nh_.getParam("/smb_highlevel_controller_node/vel_queue", vel_queue_) &&
            nh_.getParam("/smb_highlevel_controller_node/vis_topic", vis_topic_) &&
            nh_.getParam("/smb_highlevel_controller_node/vis_queue", vis_queue_) &&
            nh_.getParam("/smb_highlevel_controller_node/start_stop_srv", start_stop_service_) &&
            nh_.getParam("/smb_highlevel_controller_node/kp_linear", kp_linear_) &&
            nh_.getParam("/smb_highlevel_controller_node/kp_angular", kp_angular_) && 
            nh_.getParam("/smb_highlevel_controller_node/angular_deadband", angular_deadband_)) {
            success = true;
            ROS_INFO("Got all controller topics from parameter server!");
        }
        return success;
    }
}

