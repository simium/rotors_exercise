
#ifndef PATH_NODE_H
#define PATH_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <ros/ros.h>

//#include <ros/TimerEvent.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>

class PathNode {
 public:
  PathNode();
  ~PathNode();

  void Publish();

 private:

  // subscribers
  ros::Subscriber gps_sub_;
  ros::Subscriber ground_truth_sub_;
  ros::Subscriber estimator_sub_;

  geometry_msgs::PoseStamped msgPose_;

  // publishers
  ros::Publisher PathKalmanFilter_pub_;
  ros::Publisher PathFakeGPS_pub_;
  ros::Publisher PathGroundTruth_pub_;

  nav_msgs::Path msgFakeGPSPath_;
  nav_msgs::Path msgGroundTruthPath_;
  nav_msgs::Path msgKalmanFilterPath_;

  void FakeGPSCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void GroundTruthCallback(
      const geometry_msgs::PoseStampedConstPtr &pose_msg);

  void EstimatorCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);
};

#endif // PATH_NODE_H
