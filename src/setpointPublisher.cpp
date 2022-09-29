#include "drone_contest_2022/setpointPublisher.h"

setPointPublisher::setPointPublisher(){

    if( !_nh.getParam("setpoint_topic", _setpoint_topic)) {
        _setpoint_topic = "/drone/setpoint";
    }
    if( !_nh.getParam("obs_dist_min", _obs_dist_min)) {
        _obs_dist_min = 0.45;
    }
    if( !_nh.getParam("obs_dist_break", _obs_dist_break)) {
        _obs_dist_break = 1.0;
    }
    if( !_nh.getParam("k_back_vel", _k_back_vel)) {
        _k_back_vel = 0.8;
    }

    _mavros_sp_pub = _nh.advertise<mavros_msgs::PositionTarget>( "/mavros/setpoint_raw/local" , 1);
    _laser_dist_pub = _nh.advertise<std_msgs::Float32MultiArray>( "/laser/distances" , 1);
    _setpoint_sub = _nh.subscribe(_setpoint_topic.c_str(), 1, &setPointPublisher::setpoint_cb, this);
    _pose_sub = _nh.subscribe( "/mavros/vision_pose/pose" , 1, &setPointPublisher::pose_cb, this);
    _lidar_sub = _nh.subscribe( "/laser/scan" , 1, &setPointPublisher::lidar_cb, this);
    _mavros_state_sub = _nh.subscribe( "/mavros/state", 1, &setPointPublisher::mavros_state_cb, this);

    _setpoint << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    _old_pos_setpoint << 0.0, 0.0, 0.0;

    _first_local_pos = false;
    _new_setpoint = false;
}

void setPointPublisher::setpoint_cb(std_msgs::Float32MultiArray sp){

    if (_old_pos_setpoint(0) != sp.data[0] || _old_pos_setpoint(1) != sp.data[1] || _old_pos_setpoint(2) != sp.data[2]){
        
        _new_setpoint = true;
        _setpoint << sp.data[0], sp.data[1], sp.data[2], sp.data[3],
                    sp.data[4], sp.data[5], sp.data[6], sp.data[7],
                    sp.data[8], sp.data[9], sp.data[10];

        cout << "new_sp: " << sp.data[0] << " " << sp.data[1] << " " << sp.data[2] << endl;
        cout << "old_sp: " << _old_pos_setpoint(0) << " " << _old_pos_setpoint(1) << " " << _old_pos_setpoint(2) << endl;


        _old_pos_setpoint << sp.data[0], sp.data[1], sp.data[2];
        
    }
    else
        _new_setpoint = false;
    
}

void setPointPublisher::pose_cb ( geometry_msgs::PoseStamped msg ) {

   Eigen::Vector3d rpy;

   _world_pos_odom << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

   rpy = utilities::R2XYZ ( utilities::QuatToMat ( Eigen::Vector4d( msg.pose.orientation.w,  msg.pose.orientation.x,  msg.pose.orientation.y,  msg.pose.orientation.z) ) );
   _yaw_odom = rpy(2); 
   _quat_odom << msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z;

   _first_local_pos = true;

}

float setPointPublisher::get_cone_measure( int mid_pos,  double angle_incr,  std::vector<float>& ranges){
    
    float ANGLE_CONE = 0.348; //20 degree both sides
    float min = std::numeric_limits<float>::infinity();
    int elems = round( ANGLE_CONE / angle_incr);
    float angle, dist;
    
    if( mid_pos == 0){
        for(int i= 0; i < elems; i++){

            angle = i * angle_incr;
            dist = fabs(ranges[i] * cos(angle));
            
            if(dist < min){
                min = dist;
            }

        }

        for(int i= ranges.size()-1; i > (ranges.size()-1-elems); i--){

            angle = (ranges.size()-1-i) * angle_incr;
            dist = fabs(ranges[i] * cos(angle));

            if(dist < min){
                min = dist;

            }

        }

    }
    else{

        for(int i= (mid_pos-elems); i < (mid_pos+elems); i++){

            angle = (i-mid_pos) * angle_incr;
            dist = fabs(ranges[i] * cos(angle));
            if(dist < min){
                min = dist;
            }

        }

    }

    if( min > 5.0)
        min = 5.0;

    return min;

}

void setPointPublisher::lidar_cb(sensor_msgs::LaserScan msg){

    float angle_min, angle_max, angle_incr;
    int front_elem, left_elem, right_elem, back_elem;

    angle_min = msg.angle_min;
    angle_max = msg.angle_max;
    angle_incr = msg.angle_increment;
    
    front_elem = round((-1 * angle_min) / angle_incr);
    back_elem = 0;
    right_elem = round((-1 * angle_min/2) / angle_incr);
    left_elem = round(( 3*angle_max/2) / angle_incr);

    _range_front = get_cone_measure(front_elem, angle_incr, msg.ranges);
    _range_left  = get_cone_measure(left_elem,  angle_incr, msg.ranges);
    _range_back  = get_cone_measure(back_elem,  angle_incr, msg.ranges);
    _range_right = get_cone_measure(right_elem, angle_incr, msg.ranges);

    std_msgs::Float32MultiArray distances;

    distances.data.resize(4);
    distances.data[0] = _range_front;
    distances.data[1] = _range_right;
    distances.data[2] = _range_back;
    distances.data[3] = _range_left;
    _laser_dist_pub.publish(distances);

    // cout << "front: " << _range_front << " back: " << _range_back << endl;
    // cout << "left: " << _range_left << " right: " << _range_right << endl;

}

std::tuple<float, float, float> setPointPublisher::collisionAvoidance(const float range, float pos, float vel, float acc, float old_pos, int pos_dir, float dt){

    float des_pos, des_vel, des_acc;
    float space = (( (_obs_dist_break-_obs_dist_min) - ( range - _obs_dist_min))/(_obs_dist_break-_obs_dist_min));
    cout << "range: " << range << endl;
    cout << "space: " << space << endl;

    if( range > _obs_dist_min && range > _obs_dist_break || (pos_dir*vel) < 0.0 ){

        des_pos = pos;
        des_vel = vel;
        des_acc = acc;
        cout << "no break \n";
    }
    else if( range > _obs_dist_min && range < _obs_dist_break && (pos_dir*vel) > 0.0 ){
            
        des_acc = 0.0;
        des_vel = vel - vel * space;
        des_pos = old_pos + vel * dt;

        cout << "v_sp: " << vel << " v_break: " << space*vel << endl;
        cout << "des_vel: " << vel << endl;

    }
    else if( range < _obs_dist_min && (pos_dir*vel) > 0.0  ){

        des_acc = 0.0;
        des_vel = -pos_dir * _k_back_vel * space;
        des_pos = old_pos - vel * dt;;
        
        cout << "v_sp: " << vel << " v_back: " << space*vel << endl;
        cout << "des_vel_back: " << vel << endl;
        
    }
    cout << "\n --- \n";

    return std::make_tuple(des_pos, des_vel, des_acc);

}

void setPointPublisher::publisher(){
    
    int rate = 100;
    float dt = 1.0/rate;

    ros::Rate r(rate);
    Eigen::Vector3f des_pos;
    Eigen::Vector3f des_vel;
    Eigen::Vector3f des_acc;
    Eigen::Vector3f old_des_pos;
    float des_yaw;
    float des_yaw_rate;

    std::tuple<float, float, float> x_tuple, y_tuple;

    mavros_msgs::PositionTarget ptarget;
    ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // ptarget.type_mask =
        // mavros_msgs::PositionTarget::IGNORE_PX |
        // mavros_msgs::PositionTarget::IGNORE_PY |
        // mavros_msgs::PositionTarget::IGNORE_PZ |
        // mavros_msgs::PositionTarget::IGNORE_AFX |
        // mavros_msgs::PositionTarget::IGNORE_AFY |
        // mavros_msgs::PositionTarget::IGNORE_AFZ;
        // mavros_msgs::PositionTarget::FORCE ;
        // mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    while( !_first_local_pos )
        usleep(0.1*1e6);
    ROS_INFO("First local pose arrived!");

    // *** Everything in odom frame ***
    des_pos = _world_pos_odom;
    old_des_pos = _world_pos_odom;
    des_yaw = _yaw_odom;
    des_vel << 0.0, 0.0, 0.0;
    des_acc << 0.0, 0.0, 0.0;
    des_yaw_rate = 0.0;

    while( ros::ok() ){

        if( _mavros_state.mode != "OFFBOARD") {
            des_pos  = _world_pos_odom;
            des_yaw = _yaw_odom;
            des_vel << 0.0, 0.0, 0.0;
            des_acc << 0.0, 0.0, 0.0;
            des_yaw_rate = 0.0;
        }
        else if( isnan( _setpoint(0) ) || isnan( _setpoint(1) ) || isnan( _setpoint(2) ) || isnan( _setpoint(3) ) ||
                 isnan( _setpoint(4) ) || isnan( _setpoint(5) ) || isnan( _setpoint(6) ) || isnan( _setpoint(7) )){

            ROS_INFO("NaN detceted in setpoints");
            des_pos  = _world_pos_odom;
            des_yaw = _yaw_odom;
            des_vel << 0.0, 0.0, 0.0;
            des_acc << 0.0, 0.0, 0.0;
            des_yaw_rate = 0.0;
        }
        else{
            
        //     if(_new_setpoint){
        //         cout << "new sp \n";
        //         if( _setpoint(4) >= 0.0) //going forward
        //             x_tuple = collisionAvoidance(_range_front+200, _setpoint(0), _setpoint(4), _setpoint(8), old_des_pos(0), 1, dt);
        //         else // going backward
        //             x_tuple = collisionAvoidance(_range_back+200, _setpoint(0), _setpoint(4), _setpoint(8), old_des_pos(0), -1, dt);


        //         if( (_range_left > 0.45 && _setpoint(5)>0.0) ||
        //             (_range_right > 0.45 && _setpoint(5)<0.0) ){
                    
        //             des_pos(1) = _setpoint(1);
        //             des_vel(1) = _setpoint(5);
        //             des_acc(1) = _setpoint(9);
        //         }
                
        //         des_pos(0) = std::get<0>(x_tuple);
        //         des_vel(0) = std::get<1>(x_tuple);
        //         des_acc(0) = std::get<2>(x_tuple);

        //         des_pos(2) = _setpoint(2);
        //         des_vel(2) = _setpoint(6);
        //         des_acc(2) = _setpoint(10);
        //         des_yaw = _setpoint(3);
        //         des_yaw_rate = _setpoint(7);
        //     }
        //     else{
        //         des_pos = old_des_pos;
        //         des_vel << 0.0, 0.0, 0.0;
        //         des_acc << 0.0, 0.0, 0.0;
        //         des_yaw = _setpoint(3);
        //         des_yaw_rate = _setpoint(7); 
        //     }
            

            des_pos(0) = _setpoint(0);
            des_vel(0) = _setpoint(4);
            des_acc(0) = _setpoint(8);

            des_pos(1) = _setpoint(1);
            des_vel(1) = _setpoint(5);
            des_acc(1) = _setpoint(9);

            des_pos(2) = _setpoint(2);
            des_vel(2) = _setpoint(6);
            des_acc(2) = _setpoint(10);

            des_yaw = _setpoint(3);
            des_yaw_rate = _setpoint(7);

        }

        ptarget.header.stamp = ros::Time::now();
        ptarget.position.x = des_pos(0);
        ptarget.position.y = des_pos(1);
        ptarget.position.z = des_pos(2); 
        ptarget.velocity.x = des_vel(0);
        ptarget.velocity.y = des_vel(1);
        ptarget.velocity.z = des_vel(2); 
        ptarget.acceleration_or_force.x = des_acc(0);
        ptarget.acceleration_or_force.y = des_acc(1);
        ptarget.acceleration_or_force.z = des_acc(2); 
        ptarget.yaw = des_yaw;
        ptarget.yaw_rate = des_yaw_rate;
        
        cout << "des_pos: " << des_pos << endl;
        cout << "old_pos: " << old_des_pos << endl;

        old_des_pos = des_pos;

        _mavros_sp_pub.publish( ptarget );
        

        r.sleep();
    }
}   

void setPointPublisher::run(){
    boost::thread publisher_t( &setPointPublisher::publisher, this);
}