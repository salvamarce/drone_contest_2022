#include "ros/ros.h"
#include "boost/thread.hpp"
#include "mavros_msgs/PositionTarget.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/State.h"
#include "Eigen/Dense"
#include "drone_contest_2022/utils.h"

using namespace std;

class setPointPublisher{

    protected:
        /** ROS Stuff **/
        ros::NodeHandle _nh;
        ros::Publisher _mavros_sp_pub;
        ros::Subscriber _setpoint_sub;
        ros::Subscriber _pose_sub;
        ros::Subscriber _mavros_state_sub;

        string _setpoint_topic;

        /** Drone variables **/
        //x, y, z, yaw, vx, vy, vz, yaw_rate
        Eigen::Matrix<float, 8, 1> _setpoint; 

        Eigen::Vector3f _world_pos_odom;
        Eigen::Vector4f _quat_odom;
        float _yaw_odom;

        mavros_msgs::State _mavros_state;

        bool _first_local_pos;

        /** Functions **/
        void setpoint_cb(std_msgs::Float32MultiArray sp);
        void pose_cb(geometry_msgs::PoseStamped msg);
        void mavros_state_cb(mavros_msgs::State state) { _mavros_state = state; }
        void publisher();

    public:
        setPointPublisher();
        void run();
        
};