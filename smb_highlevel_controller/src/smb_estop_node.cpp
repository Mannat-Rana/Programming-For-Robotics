#include <smb_highlevel_controller/SmbEStop.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "smb_estop_node");
    ros::NodeHandle nh{"~"};
    smb_estop::SmbEStop node(nh);
    ros::spin();
}