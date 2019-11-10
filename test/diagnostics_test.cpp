/*
 * Author  : Heethesh Vhavle
 * Version : 1.0.0
 * Date    : Nov 10, 2019
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <delta_planning_controls/utils.hpp>

namespace delta {
namespace tests {

namespace {
  ros::Publisher diag_pub;
  utils::FPSLogger fps_logger = utils::FPSLogger("Test");

  void PublishDiagnostics() {
    diagnostic_msgs::DiagnosticArray msg;
    msg.header.stamp = ros::Time::now();
    auto status = utils::MakeDiagnosticsStatus("test", "planning_controls", fps_logger.GetFPS());
    msg.status.push_back(status);
    diag_pub.publish(msg);
  }
}

}  // tests
}  // delta

int main(int argc, char **argv) {
  ros::init(argc, argv, "diagnostics_test");
  ros::NodeHandle n;
  ROS_INFO("diagnostics_test node started.");
  
  delta::tests::diag_pub = n.advertise<diagnostic_msgs::DiagnosticArray>(
    "/delta/cpp/tests/diagnostics_test", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    delta::tests::fps_logger.Lap();
    ros::spinOnce();
    loop_rate.sleep();
    delta::tests::fps_logger.Tick();

    delta::tests::PublishDiagnostics();
  }

  return 0;
}
