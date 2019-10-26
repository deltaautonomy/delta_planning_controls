#include <delta_planning_controls/delta_planner.hpp>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <tf/tf.h>
#include <iostream>

DeltaPlanner::DeltaPlanner(std::string name)
{
    ros::NodeHandle private_nh("~/" + name);
    _private_nh = private_nh;

    _private_nh.param("dt", _dt, 0.1);
    _private_nh.param("speed_kp", _speed_kp, 1.0);
    _private_nh.param("speed_ki", _speed_ki, 0.01);
    _private_nh.param("speed_kd", _speed_kd, 0.1);
    _private_nh.param("steer_k1", _steer_k1, 1.0);
    _private_nh.param("steer_k2", _steer_k2, 1.0);

    _private_nh.param("steer_max", _steer_max, 1.0);
    _private_nh.param("steer_min", _steer_min, -1.0);
    _private_nh.param("brake_max", _brake_max, 0.0);
    _private_nh.param("brake_min", _brake_min, -1.0);
    _private_nh.param("throttle_max", _throttle_max, 1.0);
    _private_nh.param("throttle_min", _throttle_min, 0.0);

    _ego_state = VehicleState();

    _planner = QuinticPolynomialGeneration(0.1, 1.0);
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

void DeltaPlanner::run()
{
    if(!_plan_initialized) {
        // call the planner
        // _planner.getEvasiveTrajectory(_ego_state, )
        //_controller.setPlan(plan)
        _plan_initialized = true;
    }
    cout<<_speed_kp;
    VehicleControl control = _controller.runStep(_ego_state);

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
    _ego_state.vx = msg->twist.linear.x;
    _ego_state.vy = msg->twist.linear.y;
    _ego_state.yaw_rate = msg->twist.angular.y;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "delta_planning_controls_node");
    ros::NodeHandle nh;

    DeltaPlanner planner_obj(std::string("planner"));

    ros::Subscriber ego_state_sub = nh.subscribe<delta_prediction::EgoStateEstimate>("/delta/prediction/ego_vehicle/state", 10, &DeltaPlanner::egoStateCB, &planner_obj);

    planner_obj.control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/delta/planning/controls", 1);

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