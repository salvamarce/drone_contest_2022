#include "drone_contest_2022/setpointPublisher.h"

setPointPublisher::setPointPublisher(){

    if( !_nh.getParam("setpoint_topic", _setpoint_topic)) {
        _setpoint_topic = "/drone/setpoint";
    }

    _mavros_sp_pub = _nh.advertise<mavros_msgs::PositionTarget>( "/mavros/setpoint_raw/local" , 1);
    _setpoint_sub = _nh.subscribe(_setpoint_topic.c_str(), 1, &setPointPublisher::setpoint_cb, this);
    _pose_sub = _nh.subscribe( "/mavros/vision_pose/pose" , 1, &setPointPublisher::pose_cb, this);
    _mavros_state_sub = _nh.subscribe( "/mavros/state", 1, &setPointPublisher::mavros_state_cb, this);

    _setpoint << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    _first_local_pos = false;
}

void setPointPublisher::setpoint_cb(std_msgs::Float32MultiArray sp){

    _setpoint << sp.data[0], sp.data[1], sp.data[2], sp.data[3], sp.data[4], sp.data[5], sp.data[6], sp.data[7];
    
}

void setPointPublisher::pose_cb ( geometry_msgs::PoseStamped msg ) {

   Eigen::Vector3d rpy;

   _world_pos_odom << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

   rpy = utilities::R2XYZ ( utilities::QuatToMat ( Eigen::Vector4d( msg.pose.orientation.w,  msg.pose.orientation.x,  msg.pose.orientation.y,  msg.pose.orientation.z) ) );
   _yaw_odom = rpy(2); 
   _quat_odom << msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;

   _first_local_pos = true;

}

void setPointPublisher::publisher(){

    ros::Rate r(50);
    Eigen::Vector3f des_pos;
    Eigen::Vector3f des_vel;
    float des_yaw;
    float des_yaw_rate;

    mavros_msgs::PositionTarget ptarget;
    ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    ptarget.type_mask =
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::FORCE |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    while( !_first_local_pos )
        usleep(0.1*1e6);
    ROS_INFO("First local pose arrived!");

    // *** Everything in odom frame ***
    des_pos = _world_pos_odom;
    des_yaw = _yaw_odom;
    des_vel << 0.0, 0.0, 0.0;
    des_yaw_rate = 0.0;

    while( ros::ok() ){

        if( _mavros_state.mode != "OFFBOARD") {
            des_pos  = _world_pos_odom;
            des_yaw = _yaw_odom;
            des_vel << 0.0, 0.0, 0.0;
            des_yaw_rate = 0.0;
        }
        else if( isnan( _setpoint(0) ) || isnan( _setpoint(1) ) || isnan( _setpoint(2) ) || isnan( _setpoint(3) ) ||
                 isnan( _setpoint(4) ) || isnan( _setpoint(5) ) || isnan( _setpoint(6) ) || isnan( _setpoint(7) )){

            ROS_INFO("NaN detceted in setpoints");
            des_pos  = _world_pos_odom;
            des_yaw = _yaw_odom;
            des_vel << 0.0, 0.0, 0.0;
            des_yaw_rate = 0.0;
        }
        else{
            des_pos << _setpoint(0), _setpoint(1), _setpoint(2);
            des_yaw = _setpoint(3);
            des_vel << _setpoint(4), _setpoint(5), _setpoint(6);
            des_yaw_rate = _setpoint(7);
        }

        ptarget.header.stamp = ros::Time::now();
        ptarget.position.x = des_pos(0);
        ptarget.position.y = des_pos(1);
        ptarget.position.z = des_pos(2); 
        ptarget.velocity.x = des_vel(0);
        ptarget.velocity.y = des_vel(1);
        ptarget.velocity.z = des_vel(2); 
        ptarget.yaw = des_yaw;
        ptarget.yaw_rate = des_yaw_rate;

        _mavros_sp_pub.publish( ptarget );

        r.sleep();
    }
}   

void setPointPublisher::run(){
    boost::thread publisher_t( &setPointPublisher::publisher, this);
}