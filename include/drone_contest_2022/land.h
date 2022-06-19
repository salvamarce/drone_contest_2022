#include "ros/ros.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandTOL.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/terminal_state.h"
#include "drone_contest_2022/moveToAction.h"
#include "drone_contest_2022/landAction.h"
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"
#include "drone_contest_2022/utils.h"

using namespace std;

class landAction{

    protected:
        ros::NodeHandle _nh;
        std::string _action_name;
        actionlib::SimpleActionServer<drone_contest_2022::landAction> _as;
        actionlib::SimpleActionClient<drone_contest_2022::moveToAction> _ac;

        ros::Subscriber _pose_sub;
        ros::Subscriber _mavros_state_sub;
        ros::ServiceClient _arming_client;
		ros::ServiceClient _set_mode_client;
        ros::ServiceClient _land_client;

        mavros_msgs::State _mstate;

        drone_contest_2022::landFeedback _feedback;
        drone_contest_2022::landResult _result;

        drone_contest_2022::moveToGoal _move_goal;

        Eigen::Vector3f _pos_odom;
        float _yaw_odom;

        bool _first_local_pos;

        void mavros_state_cb( mavros_msgs::State mstate);
        void pose_cb(geometry_msgs::PoseStamped msg);

    public:
        landAction(std::string name);
        ~landAction(void) {}

        void executeCB(const drone_contest_2022::landGoalConstPtr &goal);

};