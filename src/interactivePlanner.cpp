#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "drone_contest_2022/moveToAction.h"
#include "drone_contest_2022/takeOffAction.h"
#include "drone_contest_2022/landAction.h"

using namespace std;

int main(int argc, char** argv){

    ros::init(argc, argv, "interactive_planner");

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<drone_contest_2022::moveToAction> moveTo_client("moveTo", true);
    actionlib::SimpleActionClient<drone_contest_2022::takeOffAction> takeOff_client("takeOff", true);
    actionlib::SimpleActionClient<drone_contest_2022::landAction> land_client("land", true);

    string key_in;
    double takeOff_height;
    double takeOff_duration;
    double land_duration;
    int n_sp = 0;

    std::vector<double> x_sp;
    std::vector<double> y_sp;
    std::vector<double> z_sp;
    std::vector<double> yaw_sp;
    std::vector<double> duration_sp;

    if( !nh.getParam("takeOff_height", takeOff_height)) {
        takeOff_height = 1.5;
    }
    if( !nh.getParam("takeOff_duration", takeOff_duration)) {
        takeOff_duration = 12.0;
    }
    if( !nh.getParam("land_duration", land_duration)) {
        land_duration = 10.0;
    }
    nh.getParam("x_setpoints", x_sp);
    nh.getParam("y_setpoints", y_sp);
    nh.getParam("z_setpoints", z_sp);
    nh.getParam("yaw_setpoints", yaw_sp);
    nh.getParam("sp_duration", duration_sp);

    if( x_sp.size() != y_sp.size() || x_sp.size() != z_sp.size() || x_sp.size() != yaw_sp.size() ||
        x_sp.size() != duration_sp.size() || y_sp.size() != z_sp.size() || y_sp.size() != yaw_sp.size() ||
        y_sp.size() != duration_sp.size() || z_sp.size() != yaw_sp.size() || z_sp.size() != duration_sp.size() ||
        yaw_sp.size() != duration_sp.size())
    {
        ROS_ERROR("Problems in setpoints dimensions !");
        return 0;
    }
    else{
        n_sp = x_sp.size();
    }

    moveTo_client.waitForServer();
    takeOff_client.waitForServer();
    land_client.waitForServer();
    ROS_INFO("Action server ready!");

    drone_contest_2022::takeOffGoal takeOff_goal;
    drone_contest_2022::moveToGoal moveTo_goal;
    drone_contest_2022::landGoal land_goal;

    takeOff_goal.altitude_setpoint = takeOff_height;
    takeOff_goal.duration = takeOff_duration;

    cout << "Press 's' to start: ";
    getline(cin, key_in);
    fflush(stdin);

	if(key_in == "s" && ros::ok() ){

        takeOff_client.sendGoal(takeOff_goal);
        bool takeOff_finished_before_timeout = takeOff_client.waitForResult(ros::Duration(takeOff_duration+2.0));
        if (takeOff_finished_before_timeout){

            int k=0;
            bool moveTo_finished_before_timeout;
            while ( k < n_sp && ros::ok() ){

                cout << "Press 'p' for the next setpoint: " << x_sp[k] << " " << y_sp[k] << " " << z_sp[k] 
                     << " " << yaw_sp[k] << " dur: " << duration_sp[k] <<" s \n";
                cout << "Or x to exit and land \n";
                getline(cin, key_in);
                fflush(stdin);

                if((key_in == "p")){
                    moveTo_goal.x_setpoint = x_sp[k];
                    moveTo_goal.y_setpoint = y_sp[k];
                    moveTo_goal.z_setpoint = z_sp[k];
                    moveTo_goal.yaw_setpoint = yaw_sp[k];
                    moveTo_goal.duration = duration_sp[k];

                    moveTo_client.sendGoal(moveTo_goal);

                    moveTo_finished_before_timeout = moveTo_client.waitForResult(ros::Duration(duration_sp[k]+2.0));

                    if(moveTo_finished_before_timeout){
                        k++;
                    }
                    else
                        return 0;
                }
                else if (key_in == "x" && ros::ok() ){
                
                    land_goal.duration = land_duration;

                    land_client.sendGoal(land_goal);
                }

            }
            land_goal.duration = land_duration;

            land_client.sendGoal(land_goal);

        }
        else
            return 0;          
    }

    return 0;

}