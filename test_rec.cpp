#include "ros/ros.h"
#include "std_msgs/String.h"
#include "algorithm/position.h"
#include "dji_sdk/AttitudeQuaternion.h"

ros::Subscriber ImageData;
ros::Subscriber quaternion;

void ImageCallback(const algorithm::position::ConstPtr& msg){
    ROS_INFO("I heard image data: [%d]",msg->id);
    ROS_INFO("tx: [%f] ty: [%f] tz: [%f]",msg->x, msg->y,msg->z);
    //ROS_INFO("px: [%d] py: [%d]",msg->px, msg->y);
}

void QuaternionCallback(const dji_sdk::AttitudeQuaternion& msg){
     ROS_INFO("I heard quaternion data\n");
     ROS_INFO("q0:%f q1:%f q2:%f q3:%f wx:%f wy:%f wz:%f",msg.q0,msg.q1,msg.q2,msg.q3,msg.wx,msg.wy,msg.wz);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "greceive");
	ros::NodeHandle n;
    ImageData = n.subscribe("algorithm",1, ImageCallback);
    quaternion = n.subscribe("/dji_sdk/attitude_quaternion",1000, QuaternionCallback);
    while (ros::ok())
       ros::spinOnce();
    //ros::spin();
    //ros::spinOnce();
    //ROS_INFO("SB");
	return 0;
}
