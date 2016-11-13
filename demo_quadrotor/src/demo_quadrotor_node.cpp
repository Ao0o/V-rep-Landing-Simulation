#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <sensor_msgs/CameraInfo.h>
#include <demo_quadrotor/Effort.h>
#include <image_process/Distance.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

#include <visp/vpAdaptiveGain.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDot.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <math.h>

using namespace std;
class VS
{
private:
  ros::NodeHandle nh_;
  ros::Publisher  pubTwist_; // cmd_vel
  ros::Publisher  pubEffort_;//torque and thrust
  ros::Subscriber subPose_;  // pose_stamped
  ros::Subscriber subStatus_;  // status_stamped
  ros::Subscriber sub_cam_info; // Camera parameters
  ros::Subscriber sub_quad_pose;//quad_pose
  ros::Subscriber sub_quad_vel;//quad_vel
  ros::Subscriber sub_target_position;//the landing target position in the (quadrotor) body-fixed frame

  //vpServo task;
  // Current and desired visual feature associated to the x coordinate of the point
  vpFeaturePoint s_x, s_xd;
  vpFeatureDepth s_Z, s_Zd;


  vpCameraParameters cam;
  bool Stream_info_camera; //Is equal to one if we received the information about the camera
  double depth;
  double X,Y,Z,Zd;
  //double lambda;

  bool valid_pose;
  bool valid_pose_prev;

  double t_start_loop;
  double tinit;
  geometry_msgs::Twist quad_vel;
  double Vx,Vy,Vz,height;
  double roll,pitch,yaw;

  enum Feedback{
    None,QRcode,Circle}feedback;
  
  //vpColVector v;
  //vpColVector vi;
  //double mu;
  //vpAdaptiveGain lambda_adapt;

public:
  void init_vs();
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void statusCallback(const std_msgs::Int8ConstPtr& msg);
  void CameraInfoCb(const sensor_msgs::CameraInfo& msg);
  void quadPoseCallback(const geometry_msgs::PoseStamped& msg);
  void quadVelCallback(const geometry_msgs::TwistStamped& msg);
  void targetPosCallback(const image_process::Distance& msg);
  VS(int argc, char**argv);
  virtual ~VS() {
  //task.kill();
  };
};

VS::VS(int argc, char**argv)
{
  //init_vs();
  pubTwist_  = nh_.advertise<geometry_msgs::Twist>("vs/quadrotor/cmd_vel", 1000);
  pubEffort_ = nh_.advertise<demo_quadrotor::Effort>("/quadrotor/command",1000);
  
  // Subscribe to the topic Camera info in order to receive the camera paramenter. The callback function will be called only one time.
  sub_cam_info = nh_.subscribe("/camera_info", 1000,&VS::CameraInfoCb,this);
  sub_quad_pose = nh_.subscribe("/vrep/quadrotor/pose",1000,&VS::quadPoseCallback,this);
  sub_quad_vel = nh_.subscribe("/vrep/quadrotor/twist",1000,&VS::quadVelCallback,this);
  sub_target_position = nh_.subscribe("/quadrotor/distance",1000,&VS::targetPosCallback,this);//
  subPose_   = nh_.subscribe("/visp_auto_tracker/object_position", 1000, &VS::poseCallback, this);
  subStatus_ = nh_.subscribe("/visp_auto_tracker/status", 1000, &VS::statusCallback, this);

  depth = 0.2;
  //lambda = 1.;
  valid_pose = false;
  valid_pose_prev = false;

  Stream_info_camera = 0;

  Zd = depth;
  //v.resize(2);
  //vi.resize(2);
  //v = 0; vi = 0;
  //mu = 4;
  Vx=0.0;Vy=0.0;Vz=0.0;
  roll=0.0;pitch=0.0;yaw=0.0;
  t_start_loop = 0.0;
  tinit = 0.0;
  feedback=None;
}

void VS::init_vs()
{


  //cam.initPersProjWithoutDistortion(800, 795, 320, 216);


  //lambda_adapt.initStandard(3, 0.2, 40);


  //task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
  //task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;
  //task.setLambda(lambda_adapt) ;

  //vpVelocityTwistMatrix cVe = robot.get_cVe();
  //vpMatrix eJe = robot.get_eJe();
  //task.set_cVe( cVe );
  //task.set_eJe( eJe );

  //vpImagePoint ip(0,0);

  // Create the current x visual feature
  //vpFeatureBuilder::create(s_x, cam, ip);

  // Create the desired x* visual feature
  //s_xd.buildFrom(0, 0, Zd);

  // Add the feature
  //task.addFeature(s_x, s_xd, vpFeaturePoint::selectX()) ;

  //s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0
  //s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd , 0); // log(Z/Z*) = 0 that's why the last parameter is 0

  // Add the feature
  //task.addFeature(s_Z, s_Zd) ;

}

void VS::statusCallback(const std_msgs::Int8ConstPtr& msg)
{
  if (msg->data == 3)
    valid_pose = true;
  else
    valid_pose = false;
}

void VS::targetPosCallback(const image_process::Distance& msg)
{
    if(feedback==QRcode) {return;}
    else if(feedback==None) {feedback=Circle;}
    
    static float X_error_last=0.0,Vx_error_last=0.0,X_error_sum=0.0,Y_error_last=0.0,Vy_error_last=0.0,Y_error_sum=0.0,Z_error_last=0.0,Vz_error_last=0.0;
    static float roll_error_last=0.0,roll_vel_error_last=0.0,pitch_error_last=0.0,pitch_vel_error_last=0.0;
    double torx=0.0,tory=0.0,torz=0.0,thrust=0.0;
    double Z_error,Vz_error,Vz_d; 
 
    //X轴控制
    double Kp_Vx=0.3,Kd_Vx=0.3,Kp_Ax=0.25,Kd_Ax=0.4,pitch_d=0.0,Ax_d=0.0,X_error,Vx_error,Vx_d=0.0,pitch_error=0.0,pitch_vel_error=0.0,pitch_vel_d=0.0;
    
    
    X_error=msg.x_error;  
    Vx_d=Kp_Vx*X_error+Kd_Vx*(X_error-X_error_last)+0.0005*X_error_sum;

    Vx_error=Vx_d-Vx;
    Ax_d=Kp_Ax*Vx_error+Kd_Ax*(Vx_error-Vx_error_last);

    pitch_d=-1.0*atan(Ax_d/9.81);
    pitch_error=pitch_d-pitch;
    pitch_vel_d=0.7*pitch_error+0.5*(pitch_error-pitch_error_last);
    
    pitch_vel_error=pitch_vel_d-quad_vel.angular.y;
    tory=0.7*pitch_vel_error+0.5*(pitch_vel_error-pitch_vel_error_last);
    tory=tory*0.352971;
    
    X_error_sum+=X_error;
    X_error_last=X_error;
    Vx_error_last=Vx_error;
    pitch_error_last=pitch_error;
    pitch_vel_error_last=pitch_vel_error;


    //Y轴控制
    double Kp_Vy=0.2,Kd_Vy=0.3,Kp_Ay=0.2,Kd_Ay=0.4,roll_d=0.0,Ay_d=0.0,Y_error,Vy_error,Vy_d=0.0,roll_error=0.0,roll_vel_error=0.0,roll_vel_d=0.0;
    
    
    Y_error=msg.y_error;  
    Vy_d=Kp_Vy*Y_error+Kd_Vy*(Y_error-Y_error_last);

    Vy_error=Vy_d-Vy;
    Ay_d=Kp_Ay*Vy_error+Kd_Ax*(Vy_error-Vy_error_last);

    roll_d=atan(Ay_d/9.81);
    roll_error=roll_d-roll;
    roll_vel_d=0.7*roll_error+0.5*(roll_error-roll_error_last);
    
    roll_vel_error=roll_vel_d-quad_vel.angular.x;
    torx=0.7*roll_vel_error+0.5*(roll_vel_error-roll_vel_error_last);
    torx=torx*0.267337;
    
    Y_error_last=Y_error;
    Vy_error_last=Vy_error;
    roll_error_last=roll_error;
    roll_vel_error_last=roll_vel_error;

    
    //高度控制
    static bool descent=false;
    if(fabs(X_error)<0.05&&fabs(Y_error)<0.05)
    {
       descent=true;
       //cout<<"descent"<<endl;
    }
    /*else
    {
       cout<<"x_err="<<X_error<<",y_err="<<Y_error<<",x_err_sum="<<X_error_sum<<",y_err_sum="<<Y_error_sum<<endl;
    }*/   
    if(descent)
    {
		 double Kp_Vz=0.2,Kd_Vz=0.2,Kp_Fz=0.2,Kd_Fz=0.2;

		 Z_error=0.7-msg.height;  
		 Vz_d=Kp_Vz*Z_error+Kd_Vz*(Z_error-Z_error_last);

		 Vz_error=Vz_d+Vz;
		 thrust=Kp_Fz*Vz_error+Kd_Fz*(Vz_error-Vz_error_last);
		 
		 Z_error_last=Z_error;
		 Vz_error_last=Vz_error;
    }

    demo_quadrotor::Effort effort;
    effort.torx=torx;
    effort.tory=tory;
    effort.torz=torz;
    effort.thrust=thrust;
    pubEffort_.publish(effort);     
}


void VS::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (!Stream_info_camera ) // We check if the streaming of images is started or not
  {
    std::cout << "Waiting for the camera parameters."<<std::endl;
    return;
  }
  if(height>2.1)
  {
     feedback=Circle;
     return;
  }
  else 
  {
  	  feedback=QRcode;
  	  //cout<<"x="<<msg->pose.position.x<<",y="<<msg->pose.position.y<<",z="<<msg->pose.position.z<<",height="<<height<<endl;
  }

  static float X_error_last=0.0,Vx_error_last=0.0,X_error_sum=0.0,Y_error_last=0.0,Vy_error_last=0.0,Z_error_last=0.0,Vz_error_last=0.0;
  static float roll_error_last=0.0,roll_vel_error_last=0.0,pitch_error_last=0.0,pitch_vel_error_last=0.0,yaw_error_last=0.0,yaw_vel_error_last=0.0;
  geometry_msgs::Twist out_cmd_vel;
  double torx=0.0,tory=0.0,torz=0.0,thrust=0.0;
  double Z_error,Vz_error,Vz_d;

  try 
  {
    t_start_loop = vpTime::measureTimeMs();
    std::ostringstream strs;
    strs << "Receive a new pose" << std::endl;
    std::string str;
    str = strs.str();
    ROS_DEBUG("%s", str.c_str());

    vpHomogeneousMatrix cMo = visp_bridge::toVispHomogeneousMatrix(msg->pose);

    double tem1,tem2,yaw2;
    vpPoint origin;
    origin.setWorldCoordinates(0,0,0);//Set the point world coordinates. We mean here the coordinates of the point in the object frame. 
    tf::Quaternion bq(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3(bq).getRPY(tem1,tem2,yaw2);//得到的欧拉角以弧度为单位
    origin.project(cMo);//
    X = origin.get_X();//Get the point X coordinate in the camera frame
    Y = origin.get_Y();
    Z = origin.get_Z();

    
    /*std::cout<<"yaw="<<yaw2<<std::endl;
    std::cout<<"X="<<X<<std::endl;
    std::cout<<"Y="<<Y<<std::endl;
    std::cout<<"Z="<<Z<<std::endl;*/

    if (Z <= 0)
      ROS_DEBUG("Z <= 0");

    if (! valid_pose || Z <= 0) {
      ROS_DEBUG("not valid pose");

      out_cmd_vel.linear.x = 0;
      out_cmd_vel.linear.y = 0;
      out_cmd_vel.linear.z = 0;
      out_cmd_vel.angular.x = 0;
      out_cmd_vel.angular.y = 0;
      out_cmd_vel.angular.z = 0;
      pubTwist_.publish(out_cmd_vel);

      valid_pose = false;
      valid_pose_prev = valid_pose;

      return;
    }
    //X轴控制
    double Kp_Vx=0.3,Kd_Vx=0.3,Kp_Ax=0.25,Kd_Ax=0.4,pitch_d=0.0,Ax_d=0.0,X_error,Vx_error,Vx_d=0.0,pitch_error=0.0,pitch_vel_error=0.0,pitch_vel_d=0.0;
    
    
    X_error=-1.0*Y;  
    Vx_d=Kp_Vx*X_error+Kd_Vx*(X_error-X_error_last)+0.0005*X_error_sum;

    Vx_error=Vx_d-Vx;
    Ax_d=Kp_Ax*Vx_error+Kd_Ax*(Vx_error-Vx_error_last);

    pitch_d=-1.0*atan(Ax_d/9.81);
    pitch_error=pitch_d-pitch;
    pitch_vel_d=0.7*pitch_error+0.5*(pitch_error-pitch_error_last);
    
    pitch_vel_error=pitch_vel_d-quad_vel.angular.y;
    tory=0.7*pitch_vel_error+0.5*(pitch_vel_error-pitch_vel_error_last);
    tory=tory*0.352971;
    
    X_error_sum+=X_error;
    X_error_last=X_error;
    Vx_error_last=Vx_error;
    pitch_error_last=pitch_error;
    pitch_vel_error_last=pitch_vel_error;

    //Y轴控制
    double Kp_Vy=0.2,Kd_Vy=0.3,Kp_Ay=0.2,Kd_Ay=0.4,roll_d=0.0,Ay_d=0.0,Y_error,Vy_error,Vy_d=0.0,roll_error=0.0,roll_vel_error=0.0,roll_vel_d=0.0;
    
    
    Y_error=X;  
    Vy_d=Kp_Vy*Y_error+Kd_Vy*(Y_error-Y_error_last);

    Vy_error=Vy_d-Vy;
    Ay_d=Kp_Ay*Vy_error+Kd_Ax*(Vy_error-Vy_error_last);

    roll_d=atan(Ay_d/9.81);
    roll_error=roll_d-roll;
    roll_vel_d=0.7*roll_error+0.5*(roll_error-roll_error_last);
    
    roll_vel_error=roll_vel_d-quad_vel.angular.x;
    torx=0.7*roll_vel_error+0.5*(roll_vel_error-roll_vel_error_last);
    torx=torx*0.267337;
    
    Y_error_last=Y_error;
    Vy_error_last=Vy_error;
    roll_error_last=roll_error;
    roll_vel_error_last=roll_vel_error;

    //Z轴控制
    double yaw_error=0.0,yaw_vel_d=0.0,yaw_vel_error=0.0;
    if(yaw2>-1.5708&&yaw2<1.5708)
       yaw_error=yaw2-1.5708;
    else if(yaw2<-1.5708)
       yaw_error=4.7124-yaw2;
    else if(yaw2>1.5708)
       yaw_error=yaw2-1.5708;
   
    yaw_vel_d=0.2*yaw_error+0.15*(yaw_error-yaw_error_last);
    yaw_vel_error=yaw_vel_d-quad_vel.angular.z;
    torz=0.2*yaw_vel_error+0.15*(yaw_vel_error-yaw_vel_error_last);
    torz=torz*0.3;
    
    yaw_error_last=yaw_error;
    yaw_vel_error=yaw_vel_error_last;
     
    //高度控制
    double Kp_Vz=0.4,Kd_Vz=0.4,Kp_Fz=0.5,Kd_Fz=0.5;

    Z_error=Zd-Z;  
    Vz_d=Kp_Vz*Z_error+Kd_Vz*(Z_error-Z_error_last);

    Vz_error=Vz_d+Vz;
    thrust=Kp_Fz*Vz_error+Kd_Fz*(Vz_error-Vz_error_last);
    
    Z_error_last=Z_error;
    Vz_error_last=Vz_error;
    
    demo_quadrotor::Effort effort;
    effort.torx=torx;
    effort.tory=tory;
    effort.torz=torz;
    effort.thrust=thrust;
    pubEffort_.publish(effort);
    //cout<<"QRcode"<<endl;
     
    // Update the current x feature
    //s_x.set_xyZ(origin.p[0], origin.p[1], Z);

    // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
    //s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd)) ;

    //vpPioneer robot; // Pas top ! devrait etre vpRobotPioneer
    //vpVelocityTwistMatrix cVe = robot.get_cVe();
    //vpMatrix eJe = robot.get_eJe();
    //task.set_cVe( cVe );
    //task.set_eJe( eJe );

    // Compute the control law. Velocities are computed in the mobile robot reference frame
    //v = task.computeControlLaw() ;



   // v = v - vi*exp(-mu*(t_start_loop - tinit)/1000.);
    //double max_linear_vel = 0.5;
    //double max_angular_vel = vpMath::rad(50);

    //if (std::abs(v[0]) > max_linear_vel || std::abs(v[1]) > max_angular_vel) {
      //ROS_INFO("Vel exceed max allowed");
      //for (unsigned int i=0; i< v.size(); i++)
        //ROS_INFO("v[%d]=%f", i, v[i]);
      //v = 0;
    //}

    //out_cmd_vel.linear.x = v[0];
    //out_cmd_vel.linear.y = 0;
    //out_cmd_vel.linear.z = 0;
    //out_cmd_vel.angular.x = 0;
    //out_cmd_vel.angular.y = 0;
    //out_cmd_vel.angular.z = v[1];

    //pubTwist_.publish(out_cmd_vel);
    valid_pose_prev = valid_pose;
    valid_pose = false;
  }
  catch(...) {
    ROS_INFO("Catch an exception: set vel to 0");
    out_cmd_vel.linear.x = 0;
    out_cmd_vel.linear.y = 0;
    out_cmd_vel.linear.z = 0;
    out_cmd_vel.angular.x = 0;
    out_cmd_vel.angular.y = 0;
    out_cmd_vel.angular.z = 0;
    pubTwist_.publish(out_cmd_vel);
  }
}


void VS::CameraInfoCb(const sensor_msgs::CameraInfo& msg)
 {   
     std::cout << "Received Camera INFO"<<std::endl;
     // Convert the paramenter in the visp format
     cam = visp_bridge::toVispCameraParameters(msg);
     cam.printParameters();

     // Stop the subscriber (we don't need it anymore)
     this->sub_cam_info.shutdown();

     Stream_info_camera = 1;
     init_vs();
 }

void VS::quadPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    tf::Quaternion bq(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
    tf::Matrix3x3(bq).getRPY(roll,pitch,yaw);//得到的欧拉角以弧度为单位
    /*std::cout<<"roll="<<roll<<std::endl;
    std::cout<<"pitch="<<pitch<<std::endl;
    std::cout<<"yaw="<<yaw<<std::endl;*/ 
    Vx=quad_vel.linear.x*cos(yaw)+quad_vel.linear.y*sin(yaw);
    Vy=quad_vel.linear.y*cos(yaw)-quad_vel.linear.x*sin(yaw);
    Vz=quad_vel.linear.z;
    height=-1.0*msg.pose.position.z;
    //cout<<height<<endl;
    /*if(Vx>0.01||Vx<-0.01||Vy>0.01||Vy<-0.01)
    {std::cout<<"Vx="<<Vx<<std::endl;
    std::cout<<"Vy="<<Vy<<std::endl;
    std::cout<<"Vz="<<Vz<<std::endl; }*/
}

void VS::quadVelCallback(const geometry_msgs::TwistStamped& msg)
{
     quad_vel.linear=msg.twist.linear;
     quad_vel.angular=msg.twist.angular;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_quadrotor_node");

  VS vs(argc, argv);

  ros::spin();
}

