#include <delta_planning_controls/delta_planner.hpp>
#include <carla_ros_bridge/CarlaVehicleControl.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <iostream>
#include <ros/console.h>
#include <math.h>

DeltaPlanner::DeltaPlanner(std::string name)
{
    ros::NodeHandle private_nh("~/" + name);
    _private_nh = private_nh;

    _private_nh.param("dt", _dt, 0.1);
    _private_nh.param("/delta_planning_controls/Controller/speed_kp", _speed_kp, 1.0);
    _private_nh.param("/delta_planning_controls/Controller/speed_ki", _speed_ki, 0.01);
    _private_nh.param("/delta_planning_controls/Controller/speed_kd", _speed_kd, 0.1);
    _private_nh.param("/delta_planning_controls/Controller/steer_k1", _steer_k1, 1.0);
    _private_nh.param("/delta_planning_controls/Controller/steer_k2", _steer_k2, 1.0);

    _private_nh.param("/delta_planning_controls/Controller/steer_max", _steer_max, 1.0);
    _private_nh.param("/delta_planning_controls/Controller/steer_min", _steer_min, -1.0);
    _private_nh.param("/delta_planning_controls/Controller/brake_max", _brake_max, 0.0);
    _private_nh.param("/delta_planning_controls/Controller/brake_min", _brake_min, -1.0);
    _private_nh.param("/delta_planning_controls/Controller/throttle_max", _throttle_max, 1.0);
    _private_nh.param("/delta_planning_controls/Controller/throttle_min", _throttle_min, 0.0);

    _private_nh.param("/delta_planning_controls/Planner/planner_frequency", _planner_freq, 20.0);
    _private_nh.param("/delta_planning_controls/Planner/max_acceleration_x", _max_acceleration_x, 4.0);
    _private_nh.param("/delta_planning_controls/Planner/max_decceleration_x", _min_acceleration_x, 7.0);
    _private_nh.param("/delta_planning_controls/Planner/max_acceleration_y", _max_acceleration_y, 2.0);
    _private_nh.param("/delta_planning_controls/Planner/max_decceleration_y", _min_acceleration_y, 3.0);

    cout<<_speed_kp<<"\n";

    _ego_state = VehicleState();

    _planner = QuinticPolynomialGeneration(_planner_freq, _max_acceleration_x, _min_acceleration_x);
    _plan_initialized = false;
    _controller = PIDController(_speed_kp, _speed_kd, _speed_ki, _steer_max, _steer_min, _steer_k1, _steer_k2, _dt, _throttle_max, _brake_min);
}

void DeltaPlanner::publishControl(VehicleControl control)
{
    carla_msgs::CarlaEgoVehicleControl msg;
    msg.header.stamp = _stamp;
    msg.steer = control.steer;
    msg.brake = control.brake;
    msg.throttle = control.throttle;
    control_pub.publish(msg);
}

void DeltaPlanner::visualizeEvasiveTrajectory(MatrixXd trajectory)
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
    // cout<<_plan_initialized<<endl;
    if(!_plan_initialized) {
        // call the planner
        cout<<"EGO STATE: "<<_ego_state.acc_x<<" "<<_ego_state.vx<<endl;
        _delta_plan = _planner.getEvasiveTrajectory(_ego_state);  
        cout<<"traj: "<<_delta_plan<<'\n'<<endl;

        if (_delta_plan.rows() > 10) {
            _controller.setPlan(_delta_plan);
            _plan_initialized = true;
        }
    }
    VehicleControl control = _controller.runStep(_ego_state);
    visualizeEvasiveTrajectory(_delta_plan);
    publishControl(control);

}

void DeltaPlanner::egoStateCB(const delta_prediction::EgoStateEstimate::ConstPtr& msg)
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
    _ego_state.vx = (msg->twist.linear.x)*cos(yaw);
    _ego_state.vy = (msg->twist.linear.y)*sin(yaw);
    _ego_state.yaw_rate = msg->twist.angular.y;
    _ego_state.acc_x = (msg->accel.linear.x)*cos(yaw);
    _ego_state.acc_y = (msg->accel.linear.y)*sin(yaw);
}
void DeltaPlanner::laneMarkingCB(const delta_perception::LaneMarkingArray::ConstPtr& msg)
{
    // vector <double> lane_intercepts;
    // for (int i=0; i < sizeof(msg)/sizeof(msg[0]); i++) // how to iterate over Lane marking array
    // {
    //     lane_intercepts.push_back(msg[i]->intercept);
    // }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_planning_controls_node");
    ros::NodeHandle nh;

    DeltaPlanner planner_obj(std::string("planner"));

    ros::Subscriber ego_state_sub = nh.subscribe<delta_prediction::EgoStateEstimate>("/delta/prediction/ego_vehicle/state", 10, &DeltaPlanner::egoStateCB, &planner_obj);
    ros::Subscriber lane_markings_sub = nh.subscribe<delta_perception::LaneMarkingArray>("/delta/perception/lane_detection/markings", 10, &DeltaPlanner::laneMarkingCB, &planner_obj);

    planner_obj.control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/delta/planning/controls", 1);
    planner_obj.traj_pub = nh.advertise<visualization_msgs::Marker>("/delta/planning/evasive_trajectory", 1);
    

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {   
        planner_obj.run();
        ros::spinOnce();
        r.sleep();
    }
}

// void DeltaPlanner::cfgCB(delta_planning_controls::PIDReconfigureConfig &config, uint32_t level)
// {
//     _dt = config.dt;
// }

    // dynamic_reconfigure::Server<delta_planning_controls::PIDReconfigureConfig> server;
    // dynamic_reconfigure::Server<delta_planning_controls::PIDReconfigureConfig>::CallbackType f;
    // f = boost::bind(&DeltaPlanner::cfgCB, _1, _2);

    // server.setCallback(f);