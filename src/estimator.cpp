

#include "estimator.h"

EstimatorNode::EstimatorNode() {


  ros::NodeHandle nh;

  pose_sub_ = nh.subscribe("/firefly/fake_gps/pose",  1, &EstimatorNode::PoseCallback, this);
  imu_sub_  = nh.subscribe("/firefly/imu",            1, &EstimatorNode::ImuCallback, this);


  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/firefly/pose", 1);

  timer_ = nh.createTimer(ros::Duration(0.1), &EstimatorNode::TimedCallback, this);
}

EstimatorNode::~EstimatorNode() { }

void EstimatorNode::Publish()
{
  //publish your data
  //ROS_INFO("Publishing ...");

  pose_pub.publish(msgPose_);
}

void EstimatorNode::PoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {

  ROS_INFO_ONCE("Estimator got first pose message.");
  msgPose_.header.stamp = pose_msg->header.stamp;
  msgPose_.header.seq = pose_msg->header.seq;
  msgPose_.header.frame_id =pose_msg->header.frame_id;



}

void EstimatorNode::ImuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg) {

  ROS_INFO_ONCE("Estimator got first IMU message.");

  msgPose_.header.stamp = imu_msg->header.stamp;
  msgPose_.header.seq = imu_msg->header.seq;
  msgPose_.header.frame_id =imu_msg->header.frame_id;

}

void EstimatorNode::TimedCallback(
      const ros::TimerEvent& e){
   ROS_INFO_ONCE("Timer initiated.");
   Publish();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "estimator");

  EstimatorNode estimator_node;

  ros::spin();

  return 0;
}
