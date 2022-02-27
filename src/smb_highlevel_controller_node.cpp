#include <smb_highlevel_controller/SmbHighlevelController.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "smb_highlevel_controller_node");
    ros::NodeHandle nh{"~"};
    smb_highlevel_controller::SmbHighlevelController node(nh);
    ros::spin();
}