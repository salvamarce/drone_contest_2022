#include "ros/ros.h"
#include "boost/thread.hpp"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/State.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "Eigen/Dense"
#include "drone_contest_2022/utils.h"

using namespace std;

class setPointPublisher{

    protected:
        /** ROS Stuff **/
        ros::NodeHandle _nh;
        ros::Publisher _mavros_sp_pub;
	ros::Publisher _laser_dist_pub;
        ros::Subscriber _setpoint_sub;
        ros::Subscriber _pose_sub;
        ros::Subscriber _lidar_sub;
        ros::Subscriber _mavros_state_sub;

        string _setpoint_topic;

        /** Drone variables **/
        //x, y, z, yaw, vx, vy, vz, yaw_rate, ax, ay, az
        Eigen::Matrix<float, 11, 1> _setpoint; 
        Eigen::Matrix<float, 3, 1> _old_pos_setpoint;

        Eigen::Vector3f _world_pos_odom;
        Eigen::Vector4f _quat_odom;
        float _yaw_odom;

        mavros_msgs::State _mavros_state;

        bool _first_local_pos;
        bool _new_setpoint;

        float _range_front, _range_left, _range_right, _range_back;
        float _obs_dist_break, _obs_dist_min, _k_back_vel;

        /** Functions **/
        void setpoint_cb(std_msgs::Float32MultiArray sp);
        void pose_cb(geometry_msgs::PoseStamped msg);
        void mavros_state_cb(mavros_msgs::State state) { _mavros_state = state; }
        void lidar_cb(sensor_msgs::LaserScan msg);
        float get_cone_measure( int mid_pos,  double angle_incr,  std::vector<float>& ranges);
        std::tuple<float, float, float> collisionAvoidance(const float range, float pos, float vel, float acc, float old_pos, int pos_dir, float dt);
        void publisher();

    public:
        setPointPublisher();
        void run();
        
};

