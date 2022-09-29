
#include <opencv2/aruco.hpp>
#include <signal.h>
#include <execinfo.h>
#include <opencv2/highgui.hpp>
#include <cstdio>
#include <math.h> 
#include <opencv2/core.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "boost/thread.hpp"

#include "std_msgs/Int32.h"
#include <actionlib/server/simple_action_server.h>


using namespace std;
using namespace cv;

using namespace std;

void faultHandler(int sig){
    void *trace[16];
    size_t size;
    char **messages = (char **)NULL;
  
    // get void*'s for all entries on the stack
    size = backtrace(trace, 16);
  
    // print out all the frames to stderr
    if(sig==11)
        fprintf(stderr, "SEED-ERROR (Segmentation Fault) backtrace:\n");
    else
        fprintf(stderr, "SEED-ERROR (signal %d) backtrace:\n", sig);
  
    //standard backtrace (no error line)
    //backtrace_symbols_fd(trace, size, STDERR_FILENO);
  
    //backtrace
    messages = backtrace_symbols(trace, size);
    for (int i = 1; i < size; ++i) {
        printf("[bt] #%d %s\n", i, messages[i]);
        char syscom[256];
        std::stringstream commLine;
        commLine << "addr2line %p -e " << "~" << "/catkin_ws/devel/lib/seed/seed";
        sprintf(syscom, commLine.str().c_str() , trace[i]); //last parameter is the name of this app
        //std::cout<<syscom<<"\n";
        system(syscom);
    }
  
    exit(1);
}

class Search_Marker {

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _img_sub; 
        cv::Mat _img;      
        std::vector<int> _markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Ptr<cv::aruco::DetectorParameters> parameters;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv_bridge::CvImagePtr cv_ptr;

        bool _img_fill;

    public:

        Search_Marker();
        void imageCb(const sensor_msgs::ImageConstPtr& msg);
        void elab();
        void run();
        cv::Mat getImage();

};


Search_Marker::Search_Marker(){
    

    _img_sub = _nh.subscribe("/camera/rgb/image_raw", 1, &Search_Marker::imageCb, this);

    parameters = cv::aruco::DetectorParameters::create();
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    //cv::namedWindow("img");
   
    _img_fill = false;


}
/*
cv::Mat Search_Marker::getImage() { return _img;}
*/
void Search_Marker::imageCb(const sensor_msgs::ImageConstPtr& msg) {

    try {
        //copy from the sensor_message::image the variable msg, to a cv bridge data
        cout << "try \n";
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
        _img = cv::imread( "/home/salvatore/Pictures/neabotics.jpeg", cv::IMREAD_COLOR);


        //cv_ptr->image.copyTo(_img);
        /*
        if (cv_ptr == NULL)
            cout << "nullo \n";
        else
            cout << "ptr ok \n";

        if (dictionary == NULL)
            cout << "dic nullo \n";
        else
            cout << "dic ok \n";

        cout << "dopo copy \n";
        
        cv::imshow("marker_window", im);
        cv::waitKey();
        */
        /*
        //cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, _markerIds);
        cout << "dopo detect \n";
        // if(_markerIds.size()>0){

        //     //outputImage= _img.clone();
        //     cout << "trovato: " << endl;
        //     // cv::aruco::drawDetectedMarkers(outputImage, markerCorners, _markerIds);
        //     // cv::imshow("marker_window", outputImage);
        // }
        // else
        //     cout << "no id \n";
        */

       
        _img_fill=true; 

    }
    catch (cv_bridge::Exception& e) {
        cout << "cath \n";
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } 
    

}
/*

int main(int argc, char **argv){

    ros::init(argc, argv, "Marker_identifier"); 
    ros::NodeHandle nh;
    Search_Marker mark;
    cv::Mat image;

    ros::Rate r(10);
    while(ros::ok()){

        //image = mark.getImage();

        // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
        // cv::imshow(OPENCV_WINDOW, image);  

        r.sleep();
        ros::spinOnce();
    }


    return 0;
}

*/


void Search_Marker::elab() {

    cv::Mat img;
    while( !_img_fill ) {
        usleep(0.1*1e6);
    }

    ros::Rate r(5);

    while( ros::ok() ) {
        _img.copyTo(img);
        r.sleep();

        cout << "Loop!" << endl;
        /*
        if( img.empty() ) {


            std::cout << "EMPTY!" << std::endl;
        }


        else {
            imshow( "img", img );
            waitKey(1);
        }
        */

        cv::Mat im;
        im = cv::imread( "/home/salvatore/Pictures/neabotics.jpeg", cv::IMREAD_COLOR);

        if( im.empty() ) {
            std::cout << "EMPTY!" << std::endl;
        }
        else {
            imshow( "img", im );
            waitKey();
            
        }
        r.sleep();
    }



}

void Search_Marker::run() {

    boost::thread elab_t( &Search_Marker::elab, this);
    ros::spin();
}

int main(int argc, char** argv ) {

/*
    ros::init(argc, argv, "Marker_identifier"); 
    ros::NodeHandle nh;
    Search_Marker mark;
    mark.run();
*/
  
  
    cv::Mat im;
    im = cv::imread( "/home/salvatore/Pictures/neabotics.jpeg");

    if( im.empty() ) {
        std::cout << "EMPTY!" << std::endl;
    }
    else {
        imshow( "img", im );
        waitKey();
    }
}