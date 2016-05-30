#include "path_node.h"

PathNode::PathNode() {

  ros::NodeHandle nh;

  gps_sub_  = nh.subscribe("/firefly/fake_gps/pose", 1, &PathNode::FakeGPSCallback, this);
  ground_truth_sub_  = nh.subscribe("/firefly/ground_truth/pose", 1, &PathNode::GroundTruthCallback, this);
  estimator_sub_ = nh.subscribe("/firefly/pose", 1, &PathNode::EstimatorCallback, this);
  //gps_sub_  = nh.subscribe("/xaircraft/fake_gps/pose", 1, &PathNode::FakeGPSCallback, this);
  //ground_truth_sub_  = nh.subscribe("/xaircraft/ground_truth/pose", 1, &PathNode::GroundTruthCallback, this);
  //estimator_sub_ = nh.subscribe("/xaircraft/pose", 1, &PathNode::EstimatorCallback, this);

  PathKalmanFilter_pub_  =  nh.advertise<nav_msgs::Path>("/firefly/path/kf", 1);
  PathFakeGPS_pub_ =  nh.advertise<nav_msgs::Path>("/firefly/path/gps", 1);
  PathGroundTruth_pub_  =  nh.advertise<nav_msgs::Path>("/firefly/path/ground_truth", 1);
  //PathKalmanFilter_pub_  =  nh.advertise<nav_msgs::Path>("/xaircraft/path/kf", 1);
  //PathFakeGPS_pub_ =  nh.advertise<nav_msgs::Path>("/xaircraft/path/gps", 1);
  //PathGroundTruth_pub_  =  nh.advertise<nav_msgs::Path>("/xaircraft/path/ground_truth", 1);
}

PathNode::~PathNode() { }

void PathNode::EstimatorCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {

  ROS_INFO_ONCE("Estimator initiated.");
  msgPose_.header.stamp = pose_msg->header.stamp;
  msgPose_.header.seq = pose_msg->header.seq;
  msgPose_.header.frame_id = "world";
  msgPose_.pose = pose_msg->pose;

  msgKalmanFilterPath_.header.stamp = pose_msg->header.stamp;
  msgKalmanFilterPath_.header.seq = pose_msg->header.seq;
  msgKalmanFilterPath_.header.frame_id = "world";

  msgKalmanFilterPath_.poses.push_back(msgPose_);

  PathKalmanFilter_pub_.publish(msgKalmanFilterPath_);
}

void PathNode::GroundTruthCallback(
      const geometry_msgs::PoseStampedConstPtr &pose_msg){

    ROS_INFO_ONCE("Ground truth initiated.");
    msgPose_.header.stamp = pose_msg->header.stamp;
    msgPose_.header.seq = pose_msg->header.seq;
    msgPose_.header.frame_id = "world";
    msgPose_.pose = pose_msg->pose;

    msgGroundTruthPath_.header.stamp = pose_msg->header.stamp;
    msgGroundTruthPath_.header.seq = pose_msg->header.seq;
    msgGroundTruthPath_.header.frame_id = "world";

    msgGroundTruthPath_.poses.push_back(msgPose_);

    PathGroundTruth_pub_.publish(msgGroundTruthPath_);
}

void PathNode::FakeGPSCallback(
      const geometry_msgs::PoseStampedConstPtr &pose_msg){

    ROS_INFO_ONCE("Fake GPS initiated.");
    msgPose_.header.stamp = pose_msg->header.stamp;
    msgPose_.header.seq = pose_msg->header.seq;
    msgPose_.header.frame_id = "world";
    msgPose_.pose = pose_msg->pose;

    msgFakeGPSPath_.header.stamp = pose_msg->header.stamp;
    msgFakeGPSPath_.header.seq = pose_msg->header.seq;
    msgFakeGPSPath_.header.frame_id = "world";

    msgFakeGPSPath_.poses.push_back(msgPose_);

    PathFakeGPS_pub_.publish(msgFakeGPSPath_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "Path");

  PathNode path_node;

  ros::spin();

  return 0;
}
