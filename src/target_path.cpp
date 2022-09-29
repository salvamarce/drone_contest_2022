#include "ros/ros.h"
#include "Eigen/Dense"
#include "gazebo_msgs/ModelState.h"

using namespace std;

int main(int argc, char** argv){

    ros::init(argc, argv, "target_path");

    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state",1);

    gazebo_msgs::ModelState path_sp;

    std::vector<double> wps;
	Eigen::MatrixXd wps_;
	string name;

	name = "path";

	if (nh.hasParam(name.c_str())){

		nh.getParam(name.c_str(), wps);
        
		int col = wps.size()/4;
        // cout << "### SIZE: " << wps.size() << endl;
		wps_.resize(4,col);
		
		for (int i = 0; i < (4*col - 3); i+=4)
		{
			wps_(0, i/4) = wps[i];
			wps_(1, i/4) = wps[i+1];
			wps_(2, i/4) = wps[i+2];
            wps_(3, i/4) = wps[i+3];
			
		}
		
	}

    ros::Rate r(10);
    int i=0;
    path_sp.model_name = "robot";

    while(ros::ok()){

        path_sp.pose.position.x = wps_(i);
        path_sp.pose.position.y = wps_(i+1);
        path_sp.twist.linear.x = wps_(i+2);
        path_sp.twist.linear.y = wps_(i+3);

        path_pub.publish(path_sp);
        
        i+=4;
        if(i>wps.size()-5)
            i = 0;

        r.sleep();
    }
	 
    return 0;
}