#include "drone_contest_2022/moveTo.h"

moveToAction::moveToAction(std::string name):
_as(_nh, name, boost::bind(&moveToAction::executeCB, this, _1), false), _action_name(name)
{
    if( !_nh.getParam("trajectory_rate", _traj_rate)) {
        _traj_rate = 20.0;
    }
    if( !_nh.getParam("setpoint_topic", _setpoint_topic)) {
        _setpoint_topic = "/drone/setpoint";
    }

    _setpoint_pub = _nh.advertise<std_msgs::Float32MultiArray>( _setpoint_topic.c_str() , 1);
    _setpoint_sub = _nh.subscribe( _setpoint_topic.c_str() , 1, &moveToAction::setpoint_cb, this);

    _dt = 1.0/_traj_rate;

    _old_x_sp = 0.0;
    _old_y_sp = 0.0;
    _old_z_sp = 0.0;
    _old_yaw_sp = 0.0;

    _as.start();
}

void moveToAction::setpoint_cb(std_msgs::Float32MultiArray sp){
    _old_x_sp = sp.data[0];
    _old_y_sp = sp.data[1];
    _old_z_sp = sp.data[2];
    _old_yaw_sp = sp.data[3];
}

void moveToAction::executeCB(const drone_contest_2022::moveToGoalConstPtr &goal){
    ros::Rate r(_traj_rate);
    bool success = true;
    
    _x_0 = _old_x_sp;
    _y_0 = _old_y_sp;
    _z_0 = _old_z_sp;
    _yaw_0 = _old_yaw_sp;
    if( _yaw_0 == 6.28 || _yaw_0 == -6.28)
        _yaw_0 = 0.0;
    ROS_INFO("valore old: %f %f %f %f", _x_0, _y_0, _z_0, _yaw_0);

    _vel_0 = _acc_0 = 0.0;
    _vel_f = _acc_f = 0.0;
    
    _x_f = goal->x_setpoint;
    _y_f = goal->y_setpoint;
    _z_f = goal->z_setpoint;
    _yaw_f = goal->yaw_setpoint;
    if( _yaw_f == 6.28 || _yaw_f == -6.28)
        _yaw_f = 0.0;
    ROS_INFO("valore goal: %f %f %f %f", _x_f, _y_f, _z_f, _yaw_f);

    std::vector<float> vec_x0{_x_0, _vel_0, _acc_0};
    std::vector<float> vec_xf{_x_f, _vel_f, _acc_f};

    std::vector<float> vec_y0{_y_0, _vel_0, _acc_0};
    std::vector<float> vec_yf{_y_f, _vel_f, _acc_f};

    std::vector<float> vec_z0{_z_0, _vel_0, _acc_0};
    std::vector<float> vec_zf{_z_f, _vel_f, _acc_f};

    std::vector<float> vec_yaw0{_yaw_0, _vel_0, _acc_0};
    std::vector<float> vec_yawf{_yaw_f, _vel_f, _acc_f};

    float t0 = 0.0;
    float tf = goal->duration;
    float n_points = tf * (1.0/_dt);
    
    _traj_x_A = computeQuinticCoeff(t0, tf, vec_x0, vec_xf);    
    _traj_y_A = computeQuinticCoeff(t0, tf, vec_y0, vec_yf);
    _traj_z_A = computeQuinticCoeff(t0, tf, vec_z0, vec_zf);
    _traj_yaw_A = computeQuinticCoeff(t0, tf, vec_yaw0, vec_yawf);
    
    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> xdt = computeQuinticTraj(_traj_x_A, t0, tf, n_points);
    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> ydt = computeQuinticTraj(_traj_y_A, t0, tf, n_points);
    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> zdt = computeQuinticTraj(_traj_z_A, t0, tf, n_points);
    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> yawdt = computeQuinticTraj(_traj_yaw_A, t0, tf, n_points);
    
    std::vector<float> xd     = std::get<1>(xdt);
    std::vector<float> d_xd   = std::get<2>(xdt);
    std::vector<float> dd_xd  = std::get<3>(xdt);

    std::vector<float> yd     = std::get<1>(ydt);
    std::vector<float> d_yd   = std::get<2>(ydt);
    std::vector<float> dd_yd  = std::get<3>(ydt);

    std::vector<float> zd     = std::get<1>(zdt);
    std::vector<float> d_zd   = std::get<2>(zdt);
    std::vector<float> dd_zd  = std::get<3>(zdt);

    std::vector<float> yawd     = std::get<1>(yawdt);
    std::vector<float> d_yawd   = std::get<2>(yawdt);

    _setpoint.data.resize(8);

    for( int i=0; i<n_points-1; i++){
        ROS_INFO("%s: Executing, goal_x: %f, feedback_y: %f ", _action_name.c_str(), goal->x_setpoint, &_feedback.actual_x_sp);

        // check that preempt has not been requested by the client
        if (_as.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", _action_name.c_str());
            // set the action state to preempted
            _as.setPreempted();
            success = false;
        }
        _setpoint.data[0] = xd[i];
        _setpoint.data[1] = yd[i];
        _setpoint.data[2] = zd[i];
        _setpoint.data[3] = yawd[i];
        _setpoint.data[4] = d_xd[i];
        _setpoint.data[5] = d_yd[i];
        _setpoint.data[6] = d_zd[i];
        _setpoint.data[7] = d_yawd[i];

        _feedback.actual_x_sp = xd[i];
        _feedback.actual_y_sp = yd[i];
        _feedback.actual_z_sp = zd[i];
        _feedback.actual_yaw_sp = yawd[i];
        _feedback.actual_vx_sp = d_xd[i];
        _feedback.actual_vy_sp = d_yd[i];
        _feedback.actual_vz_sp = d_zd[i];
        _feedback.actual_yaw_rate_sp = d_yawd[i];

        _as.publishFeedback(_feedback);
        ROS_INFO("Feedback: %f %f %f %f", _feedback.actual_x_sp, _feedback.actual_y_sp, _feedback.actual_z_sp, _feedback.actual_yaw_sp);
        _setpoint_pub.publish(_setpoint);

        r.sleep();
    }

    if(success){
        _result.finished = true;
        ROS_INFO("%s: Succeeded ", _action_name.c_str());
        _as.setSucceeded(_result);
    }
    
}
