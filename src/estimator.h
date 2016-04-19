
#ifndef ESTIMATOR_NODE_H
#define ESTIMATOR_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <ros/ros.h>

//#include <ros/TimerEvent.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>


class EstimatorNode {
 public:
  EstimatorNode();
  ~EstimatorNode();

  void Publish();

 private:

  // Define here the matrices and vectors of the Kalman filter
  Eigen::Matrix3d example;  

  // subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber imu_sub_;

  ros::Publisher pose_pub;
  geometry_msgs::PoseStamped msgPose_;
  

  ros::Timer timer_;

  void PoseCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void ImuCallback(
      const sensor_msgs::ImuConstPtr& imu_msg);

  void TimedCallback(
      const ros::TimerEvent& e);
};

#endif // ESTIMATOR_NODE_H
