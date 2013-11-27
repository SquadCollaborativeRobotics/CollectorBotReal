#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>


ros::NodeHandle nh;

std::vector<std::string> landmark_frames;
std::vector<std::string> april_frames;

void init()
{
  landmark_frames.push_back(std::string("/landmark_3"));
  landmark_frames.push_back(std::string("/landmark_5"));
  april_frames.push_back(std::string("/april_tag[3]"));
  april_frames.push_back(std::string("/april_tag[5]"));
}

// Publisher that sends out an april tag that is a possible goal node
ros::Publisher tags_pub = nh.advertise<geometry_msgs::PoseStamped>("/apriltags", 1000);
ros::Publisher new_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);

bool AprilTagLocalize(tf::TransformListener &listener)
{
  for (int i = 0; i < landmark_frames.size(); i++)
  {
    tf::StampedTransform mapLandmarkTransform;
    tf::StampedTransform differenceTransform;
    try{
      if (listener.canTransform (landmark_frames[i], april_frames[i], ros::Time(0)))
      {
        listener.lookupTransform(landmark_frames[i], april_frames[i],
                                 ros::Time(0), differenceTransform);
        listener.lookupTransform("/map", landmark_frames[i],
                                 ros::Time(0), mapLandmarkTransform);

        tf::Quaternion differenceQuat = differenceTransform.getRotation();
        tf::Quaternion mapLandmarkQuat = mapLandmarkTransform.getRotation();

        ROS_INFO("Found transform for tag %d:\ndx = %lf, dy = %lf, dz = %lf", i,
          differenceTransform.getOrigin().x(), differenceTransform.getOrigin().y(), differenceTransform.getOrigin().z());


        //Create a pose based in the /map frame
        geometry_msgs::PoseWithCovariance poseWithCovariance;
        geometry_msgs::Pose pose;

        pose.position.x = mapLandmarkTransform.getOrigin().x() - differenceTransform.getOrigin().x();
        pose.position.y = mapLandmarkTransform.getOrigin().y() - differenceTransform.getOrigin().y();
        pose.position.z = mapLandmarkTransform.getOrigin().z() - differenceTransform.getOrigin().z();

        // Quaternion for the rotation transform between the landmark and april tag's pose
        geometry_msgs::Quaternion q;
        q.x = mapLandmarkQuat.x() - differenceQuat.x();
        q.y = mapLandmarkQuat.y() - differenceQuat.y();
        q.z = mapLandmarkQuat.z() - differenceQuat.z();

        pose.orientation = q;

        geometry_msgs::PoseWithCovarianceStamped newRobotPose;
        newRobotPose.header.frame_id = "/map";
        newRobotPose.header.stamp = differenceTransform.stamp_;

        poseWithCovariance.pose = pose;
        newRobotPose.pose = poseWithCovariance;

        new_pose_pub.publish(newRobotPose);
      }
    }
    catch (tf::TransformException ex){
      ROS_INFO("could not transform from %s to %s", landmark_frames[i].c_str(), april_frames[i].c_str());
      return false;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");

  tf::TransformListener listener(nh);

  init();

  // Initialize variables for loop
  ros::Rate r(10.0);

  while(nh.ok()){

    AprilTagLocalize(listener);


    //get the pose of the robot in the map frame and publish
    tf::StampedTransform transform; //transform from landmark to where the camera sees the april tag

    geometry_msgs::PoseStamped ps;
    tf::Quaternion quat;

    for (int i=0; i<10; i++)
    {
      std::stringstream ss;
      ss<<"april_tag["<<i<<"]";
      if (listener.canTransform ("/map", ss.str().c_str(), ros::Time(0)))
      {
        listener.lookupTransform("/map", ss.str().c_str(),
                                 ros::Time(0), transform);
        quat = transform.getRotation();

        char buf[2];
        ps.header.frame_id = sprintf(buf, "%d", i);
        ps.pose.position.x = transform.getOrigin().x();
        ps.pose.position.y = transform.getOrigin().y();
        geometry_msgs::Quaternion q;
        q.x=quat.x();
        q.y=quat.y();
        q.z=quat.z();
        ps.pose.orientation = q;
        tags_pub.publish(ps);
      }
    }

    // Update callbacks after the fact, for next loop iteration.
    ros::spinOnce();
    r.sleep();
  }
}
