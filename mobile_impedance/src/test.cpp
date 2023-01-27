#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include "gazebo_msgs/LinkStates.h"
#include <mobile_impedance/velocity.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
class AdmittanceController
{
public:
  AdmittanceController()
  {
    force_sub = n_.subscribe("/published_force", 1, &AdmittanceController::AdmittanceCallback, this);
    state_sub = n_.subscribe("/gazebo/link_states", 1, &AdmittanceController::StateCallback, this);
    joint_state_sub = n_.subscribe("/mobile_base/joint_states", 1, &AdmittanceController::JointStateCallback, this);
    fl_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_l_ew_joint_velocity_controller/command", 1000);
    fr_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/f_r_ew_joint_velocity_controller/command", 1000);
    rl_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_l_ew_joint_velocity_controller/command", 1000);
    rr_velocity_pub = n_.advertise<std_msgs::Float64>("/mobile_base/r_r_ew_joint_velocity_controller/command", 100);;

  }

  void AdmittanceCallback(const geometry_msgs::Wrench::ConstPtr& force_msg)
  {
    
    Force << force_msg->force.x, force_msg->force.y, force_msg->force.z, force_msg->torque.x, force_msg->torque.y, force_msg->torque.z;
  
  }

  void StateCallback(const gazebo_msgs::LinkStates::ConstPtr& state_msg)
  {
    past_act_angle = act_angle;
                    
    tf::Quaternion actual_orientation(
          state_msg->pose[1].orientation.x,
          state_msg->pose[1].orientation.y,
          state_msg->pose[1].orientation.z,
          state_msg->pose[1].orientation.w);

    tf::Matrix3x3 ma(actual_orientation);
    ma.getRPY(roll_actual, pitch_actual, yaw_actual);

    current_pose << state_msg->pose[1].position.x, state_msg->pose[1].position.y, state_msg->pose[1].position.z, roll_actual,pitch_actual,yaw_actual;
    current_vel << state_msg->twist[1].linear.x, state_msg->twist[1].linear.y, state_msg->twist[1].linear.z, state_msg->twist[1].angular.x, 
                    state_msg->twist[1].angular.y, state_msg->twist[1].angular.z;

    //model position & orientation
    Vx  = state_msg->pose[1].position.x;
    Vy  = state_msg->pose[1].position.y;
    // steer_angle = yaw_actual;
    act_angle = yaw_actual;


    
  }

  void JointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
  {
    // each wheel angle

    past_wheel_theta_fl = wheel_theta_fl;
    past_wheel_theta_fr = wheel_theta_fr;
    past_wheel_theta_rl = wheel_theta_rl;
    past_wheel_theta_rr = wheel_theta_rr;

    wheel_theta_fl = joint_state_msg->position[0];
    wheel_theta_fr = joint_state_msg->position[1];
    wheel_theta_rl = joint_state_msg->position[2];
    wheel_theta_rr = joint_state_msg->position[3];

    wheel_theta_dot_fl_ew = joint_state_msg->velocity[0];
    wheel_theta_dot_rl_ew = joint_state_msg->velocity[2];
    wheel_theta_dot_rr_ew = joint_state_msg->velocity[3];
    wheel_theta_dot_fr_ew = joint_state_msg->velocity[1];


  }

  void run(){
    ros::Rate loop_rate(100);
    while(ros::ok)
    {
      M =   50;
      B =   1;
      K =   10; //20

      accel = 1/M*(Force - B * current_vel - K * current_pose);
    // desired_pose << 0,0,0,0,0,0;
    // chassis_vel = 1/B * (Force - K*current_pose);

      chassis_accel << accel(0),accel(1),accel(5);

      double s1 = sin(0);
      double c1 = cos(0);
      // W2
      double s2 = sin(0);
      double c2 = cos(0);
      // W3
      double s3 = sin(0);
      double c3 = cos(0);
      // W4
      double s4 = sin(0);
      double c4 = cos(0);

      // p_W1
      double ps1 = sin(0);
      double pc1 = cos(0);
      // p_W2
      double ps2 = sin(0);
      double pc2 = cos(0);
      // p_W3
      double ps3 = sin(0);//not bad, go back
      double pc3 = cos(0);  
      // p_W4
      double ps4 = sin(0);
      double pc4 = cos(0);


      double w1 = ((-y_w_1*c1 + x_w_1*s1)/(4.0*(x_w_1)*(x_w_1)+4.0*(y_w_1)*(y_w_1)));
      double w2 = ((-y_w_2*c2 + x_w_2*s2)/(4.0*(x_w_2)*(x_w_2)+4.0*(y_w_2)*(y_w_2)));
      double w3 = ((-y_w_3*c3 + x_w_3*s3)/(4.0*(x_w_3)*(x_w_3)+4.0*(y_w_3)*(y_w_3)));
      double w4 = ((-y_w_4*c4 + x_w_4*s4)/(4.0*(x_w_4)*(x_w_4)+4.0*(y_w_4)*(y_w_4)));

      double pw1 = ((-y_w_1*pc1 + x_w_1*s1)/(4.0*(x_w_1)*(x_w_1)+4.0*(y_w_1)*(y_w_1)));
      double pw2 = ((-y_w_2*pc2 + x_w_2*s2)/(4.0*(x_w_2)*(x_w_2)+4.0*(y_w_2)*(y_w_2)));
      double pw3 = ((-y_w_3*pc3 + x_w_3*s3)/(4.0*(x_w_3)*(x_w_3)+4.0*(y_w_3)*(y_w_3)));
      double pw4 = ((-y_w_4*pc4 + x_w_4*s4)/(4.0*(x_w_4)*(x_w_4)+4.0*(y_w_4)*(y_w_4)));


      past_jacobian<<  pc1/4.0, pc2/4.0, pc3/4.0, pc4/4.0,
                      ps1/4.0, ps2/4.0, ps3/4.0, ps4/4.0,
                      pw1,   pw2,   pw3,   pw4;

      jacobian<<  c1/4.0, c2/4.0, c3/4.0, c4/4.0,
                  s1/4.0, s2/4.0, s3/4.0, s4/4.0,
                    w1,   w2,   w3,   w4;

      Eigen::MatrixXd jacobian_dot = (jacobian - past_jacobian)/dt;
        

      Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();
    
      joint_vel << wheel_theta_dot_fl_ew, wheel_theta_dot_rl_ew, wheel_theta_dot_rr_ew, wheel_theta_dot_fr_ew;
      dummy<< chassis_accel(0),chassis_accel(1),chassis_accel(2);

      ddthetalist = jacobian_pinv * (dummy - jacobian_dot*joint_vel);
      dthetalist  = dthetalist + ddthetalist*dt; //?
    
      // dummy<< chassis_vel(0),chassis_vel(1),chassis_vel(5);
      
      // joint_vel = jacobian_pinv * dummy;
      joint_vel = dthetalist;

      velocity1.data = joint_vel(0);
      velocity2.data = joint_vel(1);
      velocity3.data = -joint_vel(2);
      velocity4.data = -joint_vel(3);
      
      // velocity1.data = 2;
      // velocity2.data = 2;
      // velocity3.data = -2;
      // velocity4.data = -2;


      fl_velocity_pub.publish(velocity1);
      rl_velocity_pub.publish(velocity2);
      rr_velocity_pub.publish(velocity3);
      fr_velocity_pub.publish(velocity4);
      
      ROS_WARN_STREAM( velocity1 << ", " << velocity2 << ", " << velocity3 << ", " << velocity4);
      loop_rate.sleep();
      
    }
    
  }



private: 
  ros::NodeHandle n_; 
  ros::Subscriber force_sub;
  ros::Subscriber state_sub;
  ros::Subscriber joint_state_sub;
  ros::Publisher fl_velocity_pub;
  ros::Publisher fr_velocity_pub;
  ros::Publisher rl_velocity_pub;
  ros::Publisher rr_velocity_pub;

  Eigen::VectorXd Force = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd current_pose = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd current_vel = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd chassis_accel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd accel = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd dthetalist = Eigen::VectorXd::Zero(4);

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3,4);
  Eigen::MatrixXd past_jacobian = Eigen::MatrixXd::Zero(3,4);
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd chassis_vel = Eigen::VectorXd::Zero(3);


  Eigen::VectorXd dummy = Eigen::VectorXd::Zero(3);
  
  Eigen::VectorXd V = Eigen::VectorXd::Zero(8);
  double roll_actual, pitch_actual, yaw_actual;
  double dt = 0.01;
  double M,B,K;

  std_msgs::Float64 velocity1;
  std_msgs::Float64 velocity2;
  std_msgs::Float64 velocity3;
  std_msgs::Float64 velocity4;
 
  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "admittance_node");
  ros::AsyncSpinner spinner(10);
  spinner.start();

  AdmittanceController admittance; 
  admittance.run();
  return 0;
}