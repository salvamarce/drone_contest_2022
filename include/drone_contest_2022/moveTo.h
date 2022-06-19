#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "drone_contest_2022/moveToAction.h"
#include "std_msgs/Float32MultiArray.h"
#include "drone_contest_2022/trajectories.h"

using namespace std;

class moveToAction{

    protected:

        void setpoint_cb(std_msgs::Float32MultiArray sp);

        ros::NodeHandle _nh;
        
        std::string _action_name;
        actionlib::SimpleActionServer<drone_contest_2022::moveToAction> _as;
        drone_contest_2022::moveToFeedback _feedback;
        drone_contest_2022::moveToResult _result;

        ros::Publisher _setpoint_pub;
        ros::Subscriber _setpoint_sub;
        
        Eigen::Matrix<float, 6,1> _traj_x_A;
        Eigen::Matrix<float, 6,1> _traj_y_A;
        Eigen::Matrix<float, 6,1> _traj_z_A;
        Eigen::Matrix<float, 6,1> _traj_yaw_A;

        float _vel_0, _acc_0;
        float _vel_f, _acc_f;
        float _x_0, _y_0, _z_0, _yaw_0;
        float _x_f, _y_f, _z_f, _yaw_f;
        float _old_x_sp, _old_y_sp, _old_z_sp, _old_yaw_sp;
        std_msgs::Float32MultiArray _setpoint;

        float _dt;

        string _setpoint_topic;
        float _traj_rate;

    public:
        moveToAction(std::string name);

        ~moveToAction(void) {}

        void executeCB(const drone_contest_2022::moveToGoalConstPtr &goal);

};