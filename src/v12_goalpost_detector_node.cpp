#include <v12_goalpost_detector/v12_goalpost_detector.h>

int main(int argc,char **argv){

    ros::init(argc,argv,"v12_goalpost_detector_node");

    GoalpostDetector goalpost_detector;

    ros::Rate loop_rate(30);

    while(ros::ok()){

        goalpost_detector.process();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
