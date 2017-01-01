#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <sensor_msgs/CameraInfo.h>
#include <demo_quadrotor/Effort.h>
#include <demo_quadrotor/info.h>
#include <image_process/Distance.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <sensor_msgs/JointState.h>

#include <sensor_msgs/Imu.h>
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
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <PID_lib/pid.h>

#include <yaml-cpp/yaml.h>


#define PI 3.1415926
//position PID
//PID pid_x(0.01);
//PID pid_y(0.01);
//PID pid_z(0.01);
//PID pid_phi(0.01);

demo_quadrotor::info quad_info;

nav_msgs::Odometry quad_odom;
//Vel PID
PID pid_vx(1, 0, 0.1, -1, 1, -10, 10, false);
PID pid_vy(0.7, 0, 0.08, -1, 1, -10, 10, false);

PID pid_vz(1.35, 0, 0.05, -5, 5, -5, 5, false);
PID pid_vphi(5.0, 0.0, 0.3, -1, 1, -30, 30, false);

//gimbal Vel PID
PID pid_gimbal_vel(0.015, 0, 0.0, -1, 1, -3, 3, false);

//ACC PID
PID pid_ax(1.9, 0, 0.25, -10, 10, -5, 5, false);
PID pid_ay(1.9, 0, 0.25, -10, 10, -0.5, 0.5, false);

PID pid_az(1.35, 0, 0.05, -1, 1, -2, 2, false);
//PID pid_aphi(5.0, 0, 0.11, -1, 1, -30, 30, false);
PID pid_aphi(6.9, 0, 0.2, -1, 1, -30, 30, false);
//angle vel PID
PID pid_pitch_vel(3.5, 0, 0.18, -2, 2, -6, 6, false);
PID pid_roll_vel(3.5, 0, 0.18, -2, 2, -6, 6, false);


float gimbal_vel;
//torque PID
PID pid_torx(1.9, 0, 0.03, -1, 1, -30, 30, false);
PID pid_tory(1.9, 0, 0.03, -1, 1, -30, 30, false);
//PID pid_torz(0.01);
//PID pid_thrust(0.01);
using namespace std;
class VS
{
private:
  ros::NodeHandle nh_;
  ros::Publisher  pubTwist_; // cmd_vel

  ros::Publisher  pubEffort_;//torque and thrust

  ros::Publisher pubJointVel_;//gimbal vel
  /*TO plot and adjust pid param*/

  ros::Publisher  pub_vel_d_; // desire vel
  ros::Publisher  pub_vel_actual_; // actual vel

  ros::Publisher  pub_acc_d_; // desire acc
  ros::Publisher  pub_acc_actual_; // actual acc
  //ros::Publisher  pub_acc_d_; // desire acc
  ros::Publisher pubPIDmsg_;

  ros::Publisher pubquad_pose_;

  ros::Publisher quad_info_;

  ros::Publisher quad_odom_;

  ros::Subscriber subPID_;
  ros::Subscriber subPose_;  // pose_stamped
  ros::Subscriber subStatus_;  // status_stamped
  ros::Subscriber sub_cam_info; // Camera parameters
  ros::Subscriber sub_quad_pose;//quad_pose
  ros::Subscriber sub_quad_vel;//quad_vel
  ros::Subscriber sub_target_position;//the landing target position in the (quadrotor) body-fixed frame
  ros::Subscriber sub_quad_acc;

  ros::Subscriber sub_joint_staus;//receive joint status
  //TO get time
  double dt_;
  double first_time_;
  ros::Time timeStamp_;
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

  // caulate simtime
  bool simtime;
  double start_time;

  double t_start_loop;
  double tinit;
  geometry_msgs::Twist quad_vel;

  geometry_msgs::Twist quad_acc;
  geometry_msgs::Twist quad_acc_d;

  double Vx,Vy,Vz,height;
  double roll,pitch,yaw;

  enum Feedback{
    None,QRcode,Circle}feedback;

  //test
  //  Eigen::Vector3f a;
  //  Eigen::Matrix b;
  //  Eigen::Quaternion c;

  //vpColVector v;
  //vpColVector vi;
  //double mu;
  //vpAdaptiveGain lambda_adapt;

public:
  geometry_msgs::Vector3 pidmsg;
  geometry_msgs::PoseStamped quad_pose;
  //position in body frame
  float X_b;
  float Y_b;

  //find pic stablity
  int stab_count;
  int lose_count;
  double simt_;

  float pid_p_;
  float pid_i_;
  float pid_d_;
  float gimbal_angle;
  void init_vs();
  //  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void statusCallback(const std_msgs::Int8ConstPtr& msg);
  //  void CameraInfoCb(const sensor_msgs::CameraInfo& msg);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  void quadVelCallback(const geometry_msgs::TwistStamped& msg);

  void targetPosCallback(const image_process::Distance &msg);


  void quadAccCallback(const sensor_msgs::Imu &msg);

  void jointStatusCallback(const sensor_msgs::JointState& msg);

  void PIDValueCallback(const geometry_msgs::Vector3& msg);
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
  pub_acc_d_ = nh_.advertise<geometry_msgs::Twist>("vs/quadrotor/acc_desire", 1000);
  pub_acc_actual_ = nh_.advertise<geometry_msgs::Twist>("vs/quadrotor/acc_actual", 1000);
  pubJointVel_ = nh_.advertise<sensor_msgs::JointState>("/vrep/gimbal_Plane/jointCommand", 1000);
  pubquad_pose_ =   nh_.advertise<geometry_msgs::PoseStamped>("/quadrotor_pose",1000);
  //for pid
  pubPIDmsg_  = nh_.advertise<geometry_msgs::Vector3>("vs/pidmsg", 1000);

  quad_info_ =  nh_.advertise<demo_quadrotor::info>("/quad_info", 1000);
  quad_odom_ =  nh_.advertise<nav_msgs::Odometry>("/quad_odom", 1000);
  // Subscribe to the topic Camera info in order to receive the camera paramenter. The callback function will be called only one time.
  //  sub_cam_info = nh_.subscribe("/camera_info", 2000,&VS::CameraInfoCb,this);
  sub_quad_pose = nh_.subscribe("/vrep/quadrotor/pose",1000,&VS::quadPoseCallback,this);
  sub_quad_vel = nh_.subscribe("/vrep/quadrotor/twist",1000,&VS::quadVelCallback,this);
  sub_target_position = nh_.subscribe("/quadrotor/distance",1000,&VS::targetPosCallback,this);//
  /*/quadrotor/distance*/

  //subPose_   = nh_.subscribe("/visp_auto_tracker/object_position", 1000, &VS::poseCallback, this);
  subStatus_ = nh_.subscribe("/visp_auto_tracker/status", 1000, &VS::statusCallback, this);
  sub_quad_acc = nh_.subscribe("/vrep/imu", 1000, &VS::quadAccCallback,this);
  subPID_ = nh_.subscribe("/pid_param", 1000, &VS::PIDValueCallback,this);
  sub_joint_staus = nh_.subscribe("/vrep/gimbal_Plane/jointStatus", 1000, &VS::jointStatusCallback,this);


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

  //t = 0.0;
  first_time_ = 0.0;
  dt_ = 0.05;
  simt_ = 0.0;
  //cout << "init wooo" << endl;

  pid_p_ = 0.0;
  pid_i_ = 0.0;
  pid_d_ = 0.0;
  gimbal_angle = 0.0;

  simtime = false;
  start_time = 0;

  //positioin in the body frame
  X_b = 0.0;
  Y_b = 0.0;

  stab_count = 0;
  lose_count = 0;
}

void VS::init_vs()
{
}

void VS::statusCallback(const std_msgs::Int8ConstPtr& msg)
{
  if (msg->data == 3)
    valid_pose = true;
  else
    valid_pose = false;
}

void VS::targetPosCallback(const image_process::Distance &msg)
{



  cout << "x, y " << X_b << "\t" << Y_b <<endl;
  this->dt_ = ros::Time::now().toSec() - this->first_time_;

  this->first_time_ = ros::Time::now().toSec();



  double torx=0.0,tory=0.0,torz=0.0,thrust=0.0;

  float x_error , y_error, z_error, phi_error, pix_error;
  float vx_d, vy_d, vz_d, vphi_d;
  float vx_error, vy_error, vz_error, vphi_error;
  //  float accx_error, accy_error, accz_error;
  float acc_x_d, acc_y_d, acc_z_d, acc_phi_d;
  float pitch_y_d, roll_x_d, pitch_error,
      roll_error, pitch_vel_d, roll_vel_d,
      pitch_vel_error,roll_vel_error;
  cout << "stabcount : " << stab_count << endl;

  if(!msg.find_pic){
    lose_count++;
    stab_count = 0;
    //    pix_error = 0 -gimbal_angle;
 //    gimbal_vel = 0;
    if(lose_count > 5){

            pix_error  = 0 - gimbal_angle;
            pid_gimbal_vel.set_Kp(0.5);
            gimbal_vel = pid_gimbal_vel.update(pix_error,dt_);

      //gimbal_angle = 0;
      cout << "gimbal_angle" << gimbal_angle << endl;
      phi_error = atan(1.7/5) * 180/PI -yaw;
      if(abs(phi_error) < 5 ){

        x_error = 0.6- X_b;// - gimbal_angle;//msg.x_error;
        y_error = 0;//0.5- Y_b;//0 - msg.y_error;
        z_error = 0;//4 - height;//msg.height;

      }
      else
      {
        pid_vphi.set_Kp(1);
        pid_vphi.set_Ki(0);
        pid_vphi.set_Kd(0.0);

        x_error = 0.0 - X_b;
        y_error = 0.0 - Y_b;
        z_error = 0;
        phi_error = 0 -yaw;
      }

    }

  }

  else{
    stab_count++;
    lose_count = 0;
    //gimbal control

    cout << "msg.pix-x" << msg.pix_x << endl;

    if (stab_count > 5)
    {

      cout << "find pic stab and " << endl;
      cout << "pid_gimbal" << pid_gimbal_vel.get_KP() << endl;
      pid_gimbal_vel.set_Kp(-0.02);
      pid_gimbal_vel.set_Kd(-0.001);
      pix_error = 320 - msg.pix_x;

      cout << "pix_x" << msg.pix_x << endl;

      gimbal_vel = -pid_gimbal_vel.update(pix_error,dt_);


      if (abs(pix_error) < 10)
      {
        gimbal_vel = 0.0;
      }
      pid_vphi.set_Kp(1);
      pid_vphi.set_Ki(0);
      pid_vphi.set_Kd(0.0);
      //    (5.0, 0.0, 0.3, -1, 1, -30, 30, false);
      phi_error =  0 - yaw;

      if (abs(phi_error) < 5)
      {

        cout << "phi_error < 5" << endl;
        x_error = 0.0 - X_b;// - gimbal_angle;//msg.x_error;
        y_error = -0.02 - Y_b;//0 - msg.y_error;

        if(abs(gimbal_angle) < 5 && abs(msg.y_error) < 5)
        {
          z_error = 1.7 - height;//msg.height;
        }

      }
      else
      {
        cout << "phi_error > 5" << endl;

        x_error = 0;
        y_error = 0;
        z_error = 0;
      }
    }
    else
    {
      gimbal_vel = 0;
      phi_error = atan(1.7/6) * 180/PI -yaw;
      //      if(abs(phi_error) < 5 ){

      x_error = 0.6- X_b;// - gimbal_angle;//msg.x_error;
      y_error = 0;//0.5- Y_b;//0 - msg.y_error;
      z_error = 0;//4 - height;//msg.height;

      //      }
      //      else
      //      {
      //        x_error = 0;
      //        y_error = 0;
      //        z_error = 0;
      //      }

    }
  }





  //pix_x
//  pid_gimbal_vel.set_Kp(pid_p_);
//  pid_gimbal_vel.set_Ki(pid_i_);
//  pid_gimbal_vel.set_Kd(pid_d_);
//  pidmsg.x = gimbal_angle;// msg.pix_x;
//  pidmsg.y = pix_error;
//  pidmsg.z = gimbal_vel;
//  pubPIDmsg_.publish(pidmsg);

  //    pid_ay.set_Kp(pid_p_);
  //    pid_ay.set_Ki(pid_i_);
  //    pid_ay.set_Kd(pid_d_);

  //    pid_pitch_vel.set_Kp(pid_p_);
  //    pid_pitch_vel.set_Ki(pid_i_);
  //    pid_pitch_vel.set_Kd(pid_d_);

  //    pid_tory.set_Kp(pid_p_);
  //    pid_tory.set_Ki(pid_i_);
  //    pid_tory.set_Kd(pid_d_);

  //phi
  //     simt += dt_;
  //       pid_vphi.set_Kp(pid_p_);
  //       pid_vphi.set_Ki(pid_i_);
  //       pid_vphi.set_Kd(pid_d_);
  //     pidmsg.x = 30;
  //     pidmsg.y =  yaw;
  //     pidmsg.z = simt_;
  //     //pidmsg.z = phi_error;
  //     pubPIDmsg_.publish(pidmsg);


  //vy
  //    pid_vy.set_Kp(pid_p_);
  //    pid_vy.set_Ki(pid_i_);
  //    pid_vy.set_Kd(pid_d_);
  //    pidmsg.x =  msg.y_error;
  //    pidmsg.y = y_error;
  //    pidmsg.z = vy_d;
  //    pubPIDmsg_.publish(pidmsg);
  //  pidmsg.x = quad_vel.angular.y;
  //  pidmsg.y = pitch_vel_error;
  //  pidmsg.z = tory;
  //  pubPIDmsg_.publish(pidmsg);

  //  pidmsg.x = quad_vel.angular.y;
  //  pidmsg.y = pitch_vel_error;
  //  pidmsg.z = tory;
  //  pubPIDmsg_.publish(pidmsg);


  //  if(!msg.find_pic)
  //  {
  //position  yaw error
  //  x_error = 0.8- X_b;// - gimbal_angle;//msg.x_error;
  //  y_error = 0;//0.5- Y_b;//0 - msg.y_error;
  //  z_error = 0;//4 - height;//msg.height;

  //  phi_error = atan(1.7/3.9) * 180/PI -yaw;
  //  }
  //  else{
  //    //position  yaw error
  //    x_error = 0;// - gimbal_angle;//msg.x_error;
  //    y_error = 0;//0 - msg.y_error;
  //    z_error = 0;//msg.height;

  //    phi_error = 0 ;
  //  }



  //      pid_vx.set_Kp(pid_p_);
  //      pid_vx.set_Ki(pid_i_);
  //      pid_vx.set_Kd(pid_d_);

  //  if (abs(phi_error) < 3)
  //  {
  //    phi_error = 0;
  //  }
  if(height < 1.8)
  {

    z_error = 1.43 - height;

  }
  //  else

//    x_error = 1- X_b;// - gimbal_angle;//msg.x_error;
//    y_error = 0.5- Y_b;//0 - msg.y_error;
//    z_error = 4 - height;//msg.height;
//        pidmsg.x = Vx;
//        pidmsg.y = Vy;
//        pidmsg.z = Vz;
//            pubPIDmsg_.publish(pidmsg);

  phi_error = PI * phi_error/180;
  cout << "p error : " << x_error << '\t' <<
          y_error << '\t' <<
          z_error << 't' <<
          phi_error << endl;
  // //desire vel
  vx_d = pid_vx.update(x_error, dt_);
  vy_d = pid_vy.update(y_error, dt_);
  vz_d = pid_vz.update(z_error, dt_);
  vphi_d = pid_vphi.update(phi_error,dt_);


  if(height < 1.8)
  {
    cout<< "height" << height <<endl;
    vx_d = 0;
    vy_d = 0;
    vphi_d = 0;
    z_error = 1.43 - height;
    gimbal_vel = 0.0;
  }


  //pid_vz
  //    pidmsg.x = 4;
  //    pidmsg.y = height;
  //    pidmsg.z = simt_;
  //    pubPIDmsg_.publish(pidmsg);
  //vphi

  //vx
  //    simt_ += 0.2;

  //  pidmsg.x = X_b;
  //  pidmsg.y = Y_b;
  //  pidmsg.z = height;
  //  pubPIDmsg_.publish(pidmsg);

  //simt_ += dt_;
  //  cout << "phi_error " << phi_error<< "vphi_d "<< vphi_d <<endl;

//  pidmsg.x = 20;
//  pidmsg.y =  yaw;//this->quad_vel.angular.z;
//  //      pidmsg.z = simt_;
//  pidmsg.z = vphi_error;
//  pubPIDmsg_.publish(pidmsg);

  //  cout << "v error : " << vx_error << '\t' <<
  //          vy_error << '\t' <<
  //          vz_error << 't' <<
  //          vphi_error << endl;
  //cout << "yaw" << yaw<< endl;
  //phi
  //     simt += dt_;
  //    pid_vphi.set_Kp(pid_p_);
  //    pid_vphi.set_Ki(pid_i_);
  //    pid_vphi.set_Kd(pid_d_);
  //    pidmsg.x = 1;
  //    pidmsg.y =  yaw;
  //   // pidmsg.z = simt;
  //    pidmsg.z = phi_error;
  //  pubPIDmsg_.publish(pidmsg);

  //  //vx
  //    pid_vx.set_Kp(pid_p_);
  //    pid_vx.set_Ki(pid_i_);
  //    pid_vx.set_Kd(pid_d_);
  //    pidmsg.x =  1;//gimbal_angle;
  //    pidmsg.y = x_error;
  //    pidmsg.z = vx_d;
  //    pubPIDmsg_.publish(pidmsg);


  //  //gimbal angle and  Y error during in the desire value
  //  if(gimbal_angle <72 && gimbal_angle > 68 &&  y_error < 0.1){


  //   vz_d = -0.1;
  //}



  //Vel error
  vx_error = vx_d - Vx;
  vy_error = vy_d - Vy;
  vz_error = vz_d + this->quad_vel.linear.z;
  vphi_error = vphi_d - this->quad_vel.angular.z;

  //  cout << "v error : " << vx_error << '\t' <<
  //         vy_error << '\t' <<
  //         vz_error << '\t' <<
  //         vphi_error << endl;
  //  cout << "v actual : " << this->quad_vel.linear.x << '\t' <<
  //          this->quad_vel.linear.y << '\t' <<
  //          this->quad_vel.linear.z << '\t' <<
  //          this->quad_vel.angular.z << endl;

  //    pid_ay.set_Kp(pid_p_);
  //    pid_ay.set_Ki(pid_i_);
  //    pid_ay.set_Kd(pid_d_);

  //      pid_ax.set_Kp(pid_p_);
  //      pid_ax.set_Ki(pid_i_);
  //      pid_ax.set_Kd(pid_d_);

  //desire Acc
  acc_x_d =  pid_ax.update(vx_error, dt_);
  acc_y_d  = pid_ay.update(vy_error, dt_);
  acc_z_d  = pid_az.update(vz_error, dt_);
  acc_phi_d  = pid_aphi.update(vphi_error, dt_);

  //  pid_aphi.set_Kp(pid_p_);
  //  pid_aphi.set_Ki(pid_i_);
  //  pid_aphi.set_Kd(pid_d_);
  //  pidmsg.x = 1;
  //  pidmsg.y =  this->quad_vel.angular.z;
  //  //pidmsg.z = simt;
  //  pidmsg.z = vphi_error;
  //  cout << "desire acc : " << acc_x_d << '\t' <<
  //          acc_y_d << '\t' <<
  //          acc_z_d << '\t' <<
  //          acc_phi_d << endl;
  //  simt_ += 0.2;
  // //    pidmsg.x = 1;//quad_vel.linear.y;
  // //    pidmsg.y = Vx;
  // //    pidmsg.z = simt_;
  // //    pubPIDmsg_.publish(pidmsg);
  //      pidmsg.x = 1;//quad_vel.linear.y;
  //      pidmsg.y = Vy;
  //      pidmsg.z = simt_;
  //      pubPIDmsg_.publish(pidmsg);
  //Acc error
  //  accx_error = acc_x_d - this->quad_acc.linear.x;
  //  accy_error = acc_y_d - this->quad_acc.linear.y;
  //  accz_error = acc_z_d - this->quad_acc.linear.z;


  //  cout << "actural acc : " << this->quad_acc.linear.x << '\t' <<
  //          quad_acc.linear.y << '\t' <<
  //          quad_acc.linear.z << '\t' <<endl;
  //  cout << "Acc error : " << accx_error << '\t' <<
  //          accy_error << '\t' <<
  //          accz_error << '\t' << endl;




  //desire pitch roll
  pitch_y_d = -1.0 * atan(acc_x_d/9.81);
  roll_x_d = atan(acc_y_d/9.81);



  pitch_error = pitch_y_d - pitch;
  roll_error = roll_x_d - roll;

  cout << "pitch " << pitch  << "error " << pitch_error <<endl;



  pitch_vel_d = pid_pitch_vel.update(pitch_error,dt_);
  roll_vel_d = pid_roll_vel.update(roll_error, dt_);

  pitch_vel_error = pitch_vel_d - quad_vel.angular.y;
  roll_vel_error = roll_vel_d - quad_vel.angular.x;


  if (abs(x_error) < 0.1 && abs(y_error) < 0.1 ){

  }


  //torque
  torx = pid_tory.update(roll_vel_error,dt_);
  tory = pid_tory.update(pitch_vel_error,dt_);
  torz = acc_phi_d * 0.35;
  //  thrust = pid_thrust.update(0.01);
  thrust = acc_z_d * 5.47;

  //  cout << "torque : " << torx << '\t' <<
  //          tory << '\t' <<
  //          torz << '\t' << endl;
  //  cout << "pitch_vel" << quad_vel.angular.y << endl;
  cout << "dt" << dt_ <<endl;


  sensor_msgs::JointState gimbalVel;
  gimbalVel.header.stamp = ros::Time::now();
  gimbalVel.header.frame_id = "joint";
  gimbalVel.name.push_back("joint");
  gimbalVel.velocity.push_back(gimbal_vel);
  gimbalVel.position.push_back(0);
  gimbalVel.effort.push_back(0);
  //   gimbalVel.velocity.data[0] = 1.0;
  pubJointVel_.publish(gimbalVel);

  demo_quadrotor::Effort effort;

  effort.torx = torx;
  effort.tory = tory;
  effort.torz = torz;
  effort.thrust = thrust;
  pubEffort_.publish(effort);

  quad_info.cam_x = msg.x_error;
  quad_info.cam_y = msg.y_error;
  quad_info.gimbal_angle = gimbal_angle;

 //  quad_info.iscatched = msg.find_pic;
  if (!msg.find_pic)
    quad_info.iscatched = 0;
  else
    quad_info.iscatched =1;
  quad_info.pose_x = X_b;
  quad_info.pose_y = Y_b;
  quad_info.pose_z = height;
  quad_info.relative_psi = msg.phi;
  quad_info.vel_x = Vx;
  quad_info.vel_y = Vy;
  quad_info.vel_z = Vz;

  quad_info_.publish(quad_info);
  quad_odom_.publish(quad_odom);
}



//void VS::CameraInfoCb(const sensor_msgs::CameraInfo& msg)
//{
//  std::cout << "Received Camera INFO"<<std::endl;
//  // Convert the paramenter in the visp format
//  cam = visp_bridge::toVispCameraParameters(msg);
//  cam.printParameters();

//  // Stop the subscriber (we don't need it anymore)
//  this->sub_cam_info.shutdown();

//  Stream_info_camera = 1;
//  init_vs();
//}

void VS::quadPoseCallback(const geometry_msgs::PoseStamped &msg)
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
  yaw = yaw*180/PI;
  X_b = msg.pose.position.x;
  Y_b = msg.pose.position.y;
  quad_pose = msg;


  quad_pose.pose.position.z *= -1;
  quad_pose.header.frame_id = "/mytf";
  pubquad_pose_.publish(quad_pose);

  quad_odom.child_frame_id = "/mytf";
  quad_odom.header = quad_pose.header;
  quad_odom.pose.pose = quad_pose.pose;

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
  quad_odom.twist.twist = msg.twist;

}

void VS::quadAccCallback(const sensor_msgs::Imu &msg)

{
  quad_acc.linear = msg.linear_acceleration;
  quad_acc.angular = msg.angular_velocity;
  pub_acc_actual_.publish(quad_acc);
}

void VS::PIDValueCallback(const geometry_msgs::Vector3& msg)
{
  pid_p_ = msg.x;
  pid_i_ = msg.y;
  pid_d_ = msg.z;
}

void VS::jointStatusCallback(const sensor_msgs::JointState& msg)
{
  gimbal_angle = msg.position[0] *180/3.1415926;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_quadrotor_node");




  VS vs(argc, argv);
  ros::spin();
  //    ros::Rate loop_rate(5);
  //    while(ros::ok()){
  //      ros::spinOnce();
  //      loop_rate.sleep();
  //    }



}

