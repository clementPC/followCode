#include <stdio.h>
#include <cstdlib>

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include "algorithm/position.h"
#include "algorithm/control.h"
#include "M100_control.h"

// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>

using namespace DJI::onboardSDK;

ros::Subscriber imagecontrol;
PID pidx,pidy,pidz,pidyaw;
algorithm::control _control;

void ImagecontrolCallback(const algorithm::control& msg){
    //ROS_INFO("I heard quaternion data\n");
    //ROS_INFO("q0:%f q1:%f q2:%f q3:%f wx:%f wy:%f wz:%f",msg.q0,msg.q1,msg.q2,msg.q3,msg.wx,msg.wy,msg.wz);
    pidx.UFO_PID_Control(msg.x);
    pidy.UFO_PID_Control(msg.y);
    pidz.UFO_PID_Control(msg.z);
    pidyaw.UFO_PID_Control(msg.yaw);
    _control.data=msg.data;
    _control.command=msg.command;
    ROS_INFO("Raw Control Data x:%lf y:%lf z:%lf yaw:%lf",msg.x,msg.y,msg.z,msg.yaw);
    ROS_INFO("PID_OUT_X = %lf PID_OUT_Y = %lf PID_OUT_Z = %lf PID_OUT_YAW:%lf",pidx.getValue(),pidy.getValue(),pidz.getValue(),pidyaw.getValue());

}

//float POSITION_X_EXCEPT = 0;
int main(int argc, char **argv)
{

    //init ros
    ros::init(argc, argv, "M100_SDK_Control");
    ROS_INFO("M100_SDK_Control program start");

    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);
    imagecontrol = nh.subscribe("/algorithm/control",10, ImagecontrolCallback);
    int move = 0;

    pidyaw.setKp(0.1);
    pidyaw.setLimit(10);
    pidz.setKp(2);
    pidz.setLimit(2);
    drone->request_sdk_permission_control();
    ROS_INFO("Requesting sdk control");
    sleep(1);
    ROS_INFO("Ready to take off");
    drone->takeoff();
    ROS_INFO("Taking Off");
    sleep(5);
    ROS_INFO("Finish take off");
    drone->attitude_control(0X4B,0,0,0.5,0);
    usleep(20000);
    while(ros::ok())
    {
        //pidx.setValue(0);
        //pidy.setValue(0);
        //pidz.setValue(0);
        _control.command = stay;
        ros::spinOnce();   //no blocking
        //        //algorithm::positionConstPtr position_real= ros::topic::waitForMessage<algorithm::position>("/algorithm");
        //        //              for(int i = 0;i < 5;i++)
        //ROS_INFO("Test");
	
        if(_control.command == go){            
            if(pidx.getValue() < POSI_THRESHOLD && pidy.getValue() < POSI_THRESHOLD && pidyaw.getValue()<5*POSI_THRESHOLD) move++;
            else    move = 0;
            drone->attitude_control( 0x4B,pidx.getValue(),pidy.getValue(),pidz.getValue(),pidyaw.getValue());
            if(move > 5){
                drone->attitude_control( 0x4B,0.3,0,0,0);
                move = 0;
                sleep(1);
            }
//            drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
//                                     Flight::VerticalLogic::VERTICAL_VELOCITY |
//                                     Flight::YawLogic::YAW_ANGLE |
//                                     Flight::HorizontalCoordinate::HORIZONTAL_BODY |
//                                     Flight::SmoothMode::SMOOTH_ENABLE,
//                                     pidx.getValue(), pidy.getValue(), pidz.getValue(),pidyaw.getValue());
           //drone->attitude_control(0X0A,pidx.getValue(), pidy.getValue(), 0,0);//    
	     //drone->attitude_control(0X0A,0,0,0,0);       
//	     drone->attitude_control(0X0B,0,0,0,pidyaw.getValue());
//           drone->attitude_control(0X0B,0,0,pidz.getValue(),pidyaw.getValue());
//           drone->attitude_control(0X0B,0,0,pidz.getValue(),0);
        }
        if(_control.command == stop)    break;
        if(_control.command == ckp){
            pidx.setKp( _control.data);
            pidy.setKp( _control.data);
            pidz.setKp( _control.data);
        }
        if(_control.command == climit){
            pidx.setLimit( _control.data);
            pidy.setLimit( _control.data);
            pidz.setLimit( _control.data);
        }
    }

    ROS_INFO("PID_Control End");
    sleep(2);
    ROS_INFO("Ready to landing");
    drone->landing();
    sleep(6);
    ROS_INFO("Finish landing");
    ROS_INFO("Releasing the control");
    drone->release_sdk_permission_control();
    ROS_INFO("control realeased");
    return 0;
}

/*   void chatcallback(const algorithm::position point_msg)
{
    ROS_INFO("x=%f",point_msg.x);
}  */

/*int main(int argc, char **argv)
{	
    ros::init(argc, argv, "detect");


        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<algorithm::position_2>("algorithm");
    ROS_INFO("Start");

    algorithm::position_2 srv;
    srv.request.a =1; //atoll(argv[1]);
        if (client.call(srv))
     {
    ROS_INFO("x= %f ,y= %f,z=%f", srv.response.x,srv.response.y,srv.response.z);
     }
     else
     {
       ROS_ERROR("Failed to call service ");
       return 1;
     }

    return 0;
}    



/*void chatcallback(const algorithm::position position_real)
{
    ROS_INFO("calback in");
    x= position_real.x;
    ROS_INFO("calback out");
//	ros::ok()=false;

} */

/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect");
    ros::NodeHandle n;

while(ros::ok())
{	
    algorithm::positionConstPtr position_real= ros::topic::waitForMessage<algorithm::position>("/algorithm");
//	ros::Subscriber sub= n.subscribe("algorithm",100,chatcallback);
    ROS_INFO("asdf=%f",position_real->x);
//	ros::spin();

//	ROS_INFO("asd:%f",x);

}
return 0; 

} */

























