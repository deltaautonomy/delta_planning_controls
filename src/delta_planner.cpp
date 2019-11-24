#include <iostream>
#include <math.h>

#include <tf/tf.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <delta_planning_controls/delta_planner.hpp>
// #include <carla_ros_bridge/CarlaVehicleControl.h>
#include <carla_ros_bridge_msgs/CarlaEgoVehicleControl.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <iostream>
#include <ros/console.h>
#include <math.h>
#include <algorithm>

DeltaPlanner::DeltaPlanner(std::string name)
{
    ros::NodeHandle private_nh("~/" + name);
    _private_nh = private_nh;

    _private_nh.param("dt", _dt, 0.1);
    _private_nh.param("/planning_controls/Controller/speed_kp", _speed_kp, 1.0);
    _private_nh.param("/planning_controls/Controller/speed_ki", _speed_ki, 0.01);
    _private_nh.param("/planning_controls/Controller/speed_kd", _speed_kd, 0.1);
    _private_nh.param("/planning_controls/Controller/steer_k1", _steer_k1, 1.0);
    _private_nh.param("/planning_controls/Controller/steer_k2", _steer_k2, 1.0);

    _private_nh.param("/planning_controls/Controller/steer_max", _steer_max, 1.0);
    _private_nh.param("/planning_controls/Controller/steer_min", _steer_min, -1.0);
    _private_nh.param("/planning_controls/Controller/brake_max", _brake_max, 0.0);
    _private_nh.param("/planning_controls/Controller/brake_min", _brake_min, -1.0);
    _private_nh.param("/planning_controls/Controller/throttle_max", _throttle_max, 1.0);
    _private_nh.param("/planning_controls/Controller/throttle_min", _throttle_min, 0.0);

    _private_nh.param("/planning_controls/Planner/planner_frequency", _planner_freq, 20.0);
    _private_nh.param("/planning_controls/Planner/max_acceleration_x", _max_acceleration_x, 4.0);
    _private_nh.param("/planning_controls/Planner/max_decceleration_x", _min_acceleration_x, 6.0);
    _private_nh.param("/planning_controls/Planner/max_acceleration_y", _max_acceleration_y, 2.0);
    _private_nh.param("/planning_controls/Planner/max_decceleration_y", _min_acceleration_y, 3.0);
    // _private_nh.param("/delta_planning_controls/Planner/shoulder_distance", _shoulder_const, -5.0);

    // cout<<_steer_k1<<"\n\n\n\n\n\n\n";

    _ego_state = VehicleState();
    _collision_state = VehicleState();
    _collision_time = 0.0;
    _collision_probability = 0.0;

    _planner = QuinticPolynomialGeneration(_planner_freq, _max_acceleration_x, _min_acceleration_x, _y_final);
    _plan_initialized = false;
    _controller = PIDController(_speed_kp, _speed_kd, _speed_ki, _steer_max, _steer_min, _steer_k1, _steer_k2, _dt, _throttle_max, _brake_min);
}

void DeltaPlanner::publishControl(VehicleControl control)
{
    carla_ros_bridge_msgs::CarlaEgoVehicleControl msg;
    msg.header.stamp = _stamp;
    msg.steer = control.steer;
    msg.brake = abs(control.brake);
    msg.throttle = control.throttle;
    control_pub.publish(msg);
}

void DeltaPlanner::visualizeEvasiveTrajectory(Eigen::MatrixXd trajectory)
{   visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = _stamp;
    marker.ns = "evasive_trajectory";
    marker.type = visualization_msgs::Marker::LINE_STRIP; // check this
    marker.action = visualization_msgs::Marker::ADD;

    for (int i=0; i<trajectory.rows(); i++)
    {
        geometry_msgs::Point point;
        point.x = trajectory(i,0);
        point.y = trajectory(i,1);
        point.z = 0.5;
        marker.points.push_back(point);
    }
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    traj_pub.publish(marker);
}

void DeltaPlanner::run()
{   
    _fps_logger.Lap();

    // Call the planner
    if(!_plan_initialized) {
        if(_ego_state.vx > 0) {
            // _y_final = 0;
            _delta_plan = _planner.getEvasiveTrajectory(_ego_state, _y_final);  
        // cout<<"traj: "<<_delta_plan<<'\n'<<endl;
        }

        // if (_ego_state.vx > 15) {
        if (_collision_probability > 0.5) {
            _controller.setPlan(_delta_plan);
            _plan_initialized = true;
        }
    }

    // Call the controller
    VehicleControl control = _controller.runStep(_ego_state);
    
    _fps_logger.Tick();

    visualizeEvasiveTrajectory(_delta_plan); 
    publishControl(control);
    publishDiagnostics();
}

void DeltaPlanner::egoStateCB(const delta_msgs::EgoStateEstimate::ConstPtr& msg)
{
    _stamp = msg->header.stamp;

    _ego_state.x = msg->pose.position.x;
    _ego_state.y = msg->pose.position.y;

    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    _ego_state.yaw = yaw; //wrong
    _ego_state.vx = (msg->twist.linear.x);
    _ego_state.vy = (msg->twist.linear.y);
    _ego_state.yaw_rate = msg->twist.angular.y;
    _ego_state.acc_x = (msg->accel.linear.x);
    _ego_state.acc_y = (msg->accel.linear.y);
}
void DeltaPlanner::laneMarkingCB(const delta_msgs::LaneMarkingArray::ConstPtr& msg)
{
    vector <double> lane_intercepts;
    vector<delta_msgs::LaneMarking> lanes = msg->lanes;
    for (auto lane : lanes) 
        lane_intercepts.push_back(lane.intercept);
    double max_intercept = *max_element(lane_intercepts.begin(),lane_intercepts.end());
    _y_final = 0.75*_y_final - 0.25*max_intercept;
    // cout<<"Y final: "<<_y_final<<endl;

}

void DeltaPlanner::collisionCB(const delta_msgs::CollisionDetection::ConstPtr& msg) 
{
    _collision_state.x = msg->state.x;
    _collision_state.y = msg->state.y;
    _collision_state.vx = msg->state.vx;
    _collision_state.vy = msg->state.vy;
    _collision_time = msg->time_to_impact.toSec();
    _collision_probability = msg->probability;

}

void DeltaPlanner::publishDiagnostics() {
    diagnostic_msgs::DiagnosticArray msg;
    msg.header.stamp = ros::Time::now();
    auto status = delta::utils::MakeDiagnosticsStatus("planner_controller",
        "planning_controls", _fps_logger.GetFPS());
    msg.status.push_back(status);
    diag_pub.publish(msg);
}

void DeltaPlanner::validateControls()
{
    // cout<<string(30,'')<<In Controls validation<<string(30,'')<<endl;
    _controller.get_validation();
}

void mySigintHandler(int sig)
{ 
  // All the default sigint handler does is call shutdown()
  cout<<"\033[95m"<<string(30,'*')<<" Planning and Controls Shutdown "<<string(30,'*')<<"\033[0m"<<endl;
//   cout << "\033[1;31mbold string(30,'*') HEEEEEE\033[0m\n";
  ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_planning_controls_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ROS_INFO("delta_planning_controls node started.");

    DeltaPlanner planner_obj(std::string("planner"));
    signal(SIGINT, mySigintHandler);

    ros::Subscriber ego_state_sub = nh.subscribe<delta_msgs::EgoStateEstimate>("/delta/prediction/ego_vehicle/state", 10, &DeltaPlanner::egoStateCB, &planner_obj);
    ros::Subscriber lane_markings_sub = nh.subscribe<delta_msgs::LaneMarkingArray>("/delta/perception/lane_detection/markings", 10, &DeltaPlanner::laneMarkingCB, &planner_obj);
    ros::Subscriber collsion_detection_sub = nh.subscribe<delta_msgs::CollisionDetection>("/delta/prediction/collision", 10, &DeltaPlanner::collisionCB, &planner_obj);


    planner_obj.control_pub = nh.advertise<carla_ros_bridge_msgs::CarlaEgoVehicleControl>("/delta/planning_controls/ego_vehicle/controls", 1);
    planner_obj.traj_pub = nh.advertise<visualization_msgs::Marker>("/delta/planning_controls/ego_vehicle/evasive_trajectory", 1);
    planner_obj.diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/delta/planning_controls/ego_vehicle/diagnostics", 5);

    ros::Rate rate(10); // 10 hz
    while (ros::ok())
    {   
        planner_obj.run();
        ros::spinOnce();
        rate.sleep();
    }
    planner_obj.validateControls();
}

// void DeltaPlanner::cfgCB(delta_planning_controls::PIDReconfigureConfig &config, uint32_t level)
// {
//     _dt = config.dt;
// }

    // dynamic_reconfigure::Server<delta_planning_controls::PIDReconfigureConfig> server;
    // dynamic_reconfigure::Server<delta_planning_controls::PIDReconfigureConfig>::CallbackType f;
    // f = boost::bind(&DeltaPlanner::cfgCB, _1, _2);

    // server.setCallback(f);