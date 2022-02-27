#include <smb_highlevel_controller/SmbCrashDetector.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "smb_crash_detector_node");
    ros::NodeHandle nh{"~"};
    smb_crash_detector::SmbCrashDetector node(nh);
    ros::spin();
}