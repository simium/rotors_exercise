

#include "estimator.h"

// Define dimensions
const int n = 6;
const int p = 3;
const int m = 3;

// Define KF matrices like in Slide 7
Eigen::VectorXd x(n);  // State vector: x (n by 1)
Eigen::VectorXd x_pred(n);  // State vector: x (n by 1)
Eigen::VectorXd u(p);  // Input vector: u (p by 1)
Eigen::VectorXd z(m);  // Measurement vector: z (m by 1)
Eigen::VectorXd z_pred(m);  // Measurement vector: z (m by 1)
Eigen::MatrixXd F(n,n);  // Transition matrix: F (n by n)
Eigen::MatrixXd G(n,p);  // Input matrix: G (n by p)
Eigen::MatrixXd H(m,n);  // Measurement matrix: H (m by n)
Eigen::MatrixXd Q(n,n);  // Process noise covariance matrix: Q (n by n)
Eigen::MatrixXd R(m,m);  // Measurement noise covariance matrix: R (m by m)
Eigen::MatrixXd P(n,n);  // System covariance matrix: P (n by n)
Eigen::MatrixXd P_pred(n,n);  // System covariance matrix: P (n by n)
Eigen::MatrixXd K(n,m);  // Kalman filter gain: K (n by m)

double deltaT, deltaTsq, deltaTcu, lastTime;
double sigmaX, sigmaXsq;

EstimatorNode::EstimatorNode() {


  ros::NodeHandle nh;

  //pose_sub_ = nh.subscribe("/xaircraft/fake_gps/pose",  1, &EstimatorNode::PoseCallback, this);
  //imu_sub_  = nh.subscribe("/xaircraft/imu",            1, &EstimatorNode::ImuCallback, this);

  // Needed according to email from DSerrano (April 15th)
  //ground_truth_sub_  = nh.subscribe("/xaircraft/ground_truth/pose",    1, &EstimatorNode::GroundTruthCallback, this);

  //pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/xaircraft/pose", 1);

  pose_sub_ = nh.subscribe("/firefly/fake_gps/pose",  1, &EstimatorNode::PoseCallback, this);
  imu_sub_  = nh.subscribe("/firefly/imu",            1, &EstimatorNode::ImuCallback, this);

  // Needed according to email from DSerrano (April 15th)
  ground_truth_sub_  = nh.subscribe("/firefly/ground_truth/pose",    1, &EstimatorNode::GroundTruthCallback, this);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/pose", 1);

  timer_ = nh.createTimer(ros::Duration(0.1), &EstimatorNode::TimedCallback, this);

  deltaT = 0;
  deltaTsq = deltaT*deltaT;
  deltaTcu = deltaT*deltaT*deltaT;
  lastTime = ros::Time::now().toSec();

  sigmaX = 0.1;
  sigmaXsq = sigmaX*sigmaX;

  x(0) = 0;
  x(1) = 0;
  x(2) = 0;
  x(3) = 0;
  x(4) = 0;
  x(5) = 0;

  u(0) = 0;
  u(1) = 0;
  u(2) = 0;

  z(0) = 0;
  z(1) = 0;
  z(2) = 0;

  // See slide 16
  F << 1, 0, 0, deltaT, 0, 0,
       0, 1, 0, 0, deltaT, 0,
       0, 0, 1, 0, 0, deltaT,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;

  // See slide 18
  G << deltaTsq/2, 0, 0,
       0, deltaTsq/2, 0,
       0, 0, deltaTsq/2,
       deltaT, 0, 0,
       0, deltaT, 0,
       0, 0, deltaT;

  H << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0;

  // See slide 19
  Q << deltaTcu/2, 0, 0, deltaTsq/2, 0, 0,
       0, deltaTcu/2, 0, 0, deltaTsq/2, 0,
       0, 0, deltaTcu/2, 0, 0, deltaTsq/2,
       deltaTsq/2, 0, 0, deltaTsq/2, 0, 0,
       0, deltaTsq/2, 0, 0, deltaTsq/2, 0,
       0, 0, deltaTsq/2, 0, 0, deltaTsq/2;

  Q = sigmaXsq * Q;

  R << sigmaXsq, 0, 0,
       0, sigmaXsq, 0,
       0, 0, sigmaXsq;

  P << sigmaX, 0, 0, 0, 0, 0,
       0, sigmaX, 0, 0, 0, 0,
       0, 0, sigmaX, 0, 0, 0,
       0, 0, 0, sigmaX, 0, 0,
       0, 0, 0, 0, sigmaX, 0,
       0, 0, 0, 0, 0, sigmaX;
}

EstimatorNode::~EstimatorNode() { }

void EstimatorNode::Publish()
{
  //publish your data
  msgPose_.pose.position.x = x(0);
  msgPose_.pose.position.y = x(1);
  msgPose_.pose.position.z = x(2);
  msgPose_.pose.orientation.x = ground_truth_pose(0);
  msgPose_.pose.orientation.y = ground_truth_pose(1);
  msgPose_.pose.orientation.z = ground_truth_pose(2);
  msgPose_.pose.orientation.w = ground_truth_pose(3);

  pose_pub.publish(msgPose_);
}

void EstimatorNode::PoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {

  ROS_INFO_ONCE("Estimator got first pose message.");
  msgPose_.header.stamp = pose_msg->header.stamp;
  msgPose_.header.seq = pose_msg->header.seq;
  msgPose_.header.frame_id =pose_msg->header.frame_id;

  // Update measurement vector
  z(0) = pose_msg->pose.position.x;
  z(1) = pose_msg->pose.position.y;
  z(2) = pose_msg->pose.position.z;

  // Slide 6: Update or correction (based on measurements)
  K = P_pred*H.transpose()*((H*P_pred*H.transpose() + R)).inverse();
  x = x_pred + K*(z-z_pred);
  P = P_pred - K*H*P_pred;
}

void EstimatorNode::ImuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg) {

  ROS_INFO_ONCE("Estimator got first IMU message.");

  msgPose_.header.stamp = imu_msg->header.stamp;
  msgPose_.header.seq = imu_msg->header.seq;
  msgPose_.header.frame_id =imu_msg->header.frame_id;

  // Update input vector
  u(0) = imu_msg->linear_acceleration.x;
  u(1) = imu_msg->linear_acceleration.y;

  // Fix gravity or the altitude goes rock&roll crazy :)
  u(2) = imu_msg->linear_acceleration.z - 9.81;

  // Prediction (based on model)
  x_pred = F*x + G*u;
  P_pred = F*P*F.transpose() + Q;
  z_pred = H*x_pred;
}

void EstimatorNode::TimedCallback(
      const ros::TimerEvent& e){
   ROS_INFO_ONCE("Timer initiated.");

   deltaT = ros::Time::now().toSec() - lastTime;
   deltaTsq = deltaT*deltaT;
   deltaTcu = deltaT*deltaT*deltaT;

   // See slide 16
   F << 1, 0, 0, deltaT, 0, 0,
        0, 1, 0, 0, deltaT, 0,
        0, 0, 1, 0, 0, deltaT,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

   // See slide 18
   G << deltaTsq/2, 0, 0,
        0, deltaTsq/2, 0,
        0, 0, deltaTsq/2,
        deltaT, 0, 0,
        0, deltaT, 0,
        0, 0, deltaT;

   // See slide 19
   Q << deltaTcu/2, 0, 0, deltaTsq/2, 0, 0,
        0, deltaTcu/2, 0, 0, deltaTsq/2, 0,
        0, 0, deltaTcu/2, 0, 0, deltaTsq/2,
        deltaTsq/2, 0, 0, deltaTsq/2, 0, 0,
        0, deltaTsq/2, 0, 0, deltaTsq/2, 0,
        0, 0, deltaTsq/2, 0, 0, deltaTsq/2;

   Q = sigmaXsq * Q;

   lastTime = ros::Time::now().toSec();

   Publish();
}

void EstimatorNode::GroundTruthCallback(
      const geometry_msgs::PoseStampedConstPtr &pose_msg){

    ROS_INFO_ONCE("Ground truth initiated.");

    ground_truth_pose(0) = pose_msg->pose.orientation.x;
    ground_truth_pose(1) = pose_msg->pose.orientation.y;
    ground_truth_pose(2) = pose_msg->pose.orientation.z;
    ground_truth_pose(3) = pose_msg->pose.orientation.w;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "estimator");

  EstimatorNode estimator_node;

  ros::spin();

  return 0;
}
