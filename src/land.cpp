#include "drone_contest_2022/land.h"

landAction::landAction(std::string name):
    _as(_nh, name, boost::bind(&landAction::executeCB, this, _1), false), _action_name(name),
    _ac("moveTo", true)
{

    _arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    _pose_sub = _nh.subscribe( "/mavros/vision_pose/pose" , 1, &landAction::pose_cb, this);
    _mavros_state_sub = _nh.subscribe( "/mavros/state", 1, &landAction::mavros_state_cb, this);

    _first_local_pos = false;

    _as.start();
}

void landAction::mavros_state_cb( mavros_msgs::State mstate) {
   _mstate = mstate;
}


void landAction::pose_cb(geometry_msgs::PoseStamped msg){

    Eigen::Vector3d rpy;

    _pos_odom << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

    rpy = utilities::R2XYZ ( utilities::QuatToMat ( Eigen::Vector4d( msg.pose.orientation.w,  msg.pose.orientation.x,  msg.pose.orientation.y,  msg.pose.orientation.z) ) );
    _yaw_odom = rpy(2); 

    _first_local_pos = true;

}

void landAction::executeCB(const drone_contest_2022::landGoalConstPtr &goal){

    bool result;

    //Set up control mode
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //---
    
    result = false;

    while( !_first_local_pos )
        usleep(0.1*1e6);
    ROS_INFO("First local pose arrived!");

    if( _set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
        ROS_INFO("OFFBOARD mode enabled");
    }
    
    ROS_INFO("Waiting for moveTo to start.");
    _ac.waitForServer();
    ROS_INFO("Action server started, sending  moveTo goal.");

    _move_goal.x_setpoint = _pos_odom(0);
    _move_goal.y_setpoint = _pos_odom(1);
    _move_goal.z_setpoint = -0.5;
    _move_goal.yaw_setpoint = _yaw_odom;
    _move_goal.duration = goal->duration;

    _ac.sendGoal(_move_goal);

    //wait for the action to return
    bool finished_before_timeout = _ac.waitForResult(ros::Duration(goal->duration+2.0));
  
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = _ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());

        mavros_msgs::CommandTOL land_srv;
        _land_client.call( land_srv );
        while( _mstate.armed ) usleep(0.1*1e6);
        cout << "Disarmed!" << endl;
        
        ROS_INFO("%s: Succeeded ", _action_name.c_str());
        result = true;

    }
    else
        ROS_INFO("Action did not finish before the time out.");

    _result.finished = result;
    _as.setSucceeded(_result);


}