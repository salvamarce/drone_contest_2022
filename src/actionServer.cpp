#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <drone_contest_2022/moveToAction.h>
#include <drone_contest_2022/takeOffAction.h>
#include <drone_contest_2022/moveTo.h>
#include <drone_contest_2022/takeOff.h>
#include <drone_contest_2022/land.h>
#include <drone_contest_2022/setpointPublisher.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "action_server");

    moveToAction moveTo("moveTo");
    takeOffAction takeOff("takeOff");
    landAction land("land");

    setPointPublisher sp_pub;
    sp_pub.run();
    
    ros::Rate r(100);
    
    while(ros::ok()){
    
        ros::spinOnce();
        r.sleep();
    }

    return 0;

}
