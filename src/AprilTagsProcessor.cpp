#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
//#include <tf/Vector3.h>

#include <math.h>

std::vector<std::string> landmark_frames;
std::vector<std::string> april_frames;

// Publisher that sends out an april tag that is a possible goal node?
ros::Publisher tags_pub;
// Publisher that reinitializes the pose of the robot (relocalizes)
ros::Publisher new_pose_pub;
// Publisher that reinitializes the pose of the robot (relocalizes)
ros::Publisher new_initial_pose_pub;

void init(ros::NodeHandle nh)
{
  landmark_frames.push_back(std::string("/landmark_3"));
  landmark_frames.push_back(std::string("/landmark_5"));
  april_frames.push_back(std::string("/april_tag[3]"));
  april_frames.push_back(std::string("/april_tag[5]"));

  tags_pub = nh.advertise<geometry_msgs::PoseStamped>("/apriltags", 100);
  new_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/new_pose", 100);
  new_initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);
}


bool AprilTagLocalize(tf::TransformListener &listener)
{
  for (int i = 0; i < landmark_frames.size(); i++)
  {
    tf::StampedTransform mapLandmarkTransform;
    tf::StampedTransform differenceTransform;
    tf::StampedTransform tagToBaseTransform;
    try{
      if (listener.canTransform (landmark_frames[i], april_frames[i], ros::Time(0)))
      {
        listener.lookupTransform(april_frames[i], "/base_link",
                                 ros::Time(0), tagToBaseTransform);
        listener.lookupTransform(landmark_frames[i], april_frames[i],
                                ros::Time(0), differenceTransform);
        listener.lookupTransform("map", landmark_frames[i],
                                 ros::Time(0), mapLandmarkTransform);

        // ROS_INFO("Found transform for tag %s:\ndx = %lf, dy = %lf, dz = %lf", april_frames[i].c_str(),
        //   differenceTransform.getOrigin().x(), differenceTransform.getOrigin().y(), differenceTransform.getOrigin().z());


        //Create a pose based in the /map frame
        geometry_msgs::PoseWithCovariance poseWithCovariance;
        geometry_msgs::Pose pose;

        tf::Transform tf;
        tf.setOrigin(tf::Vector3(mapLandmarkTransform.getOrigin().x() + tagToBaseTransform.getOrigin().x(),
          mapLandmarkTransform.getOrigin().y() + tagToBaseTransform.getOrigin().y(),
          0));

        // tf.setRotation(mapLandmarkTransform.getRotation().x() + tagToBaseTransform.getRotation().x(),
        //   mapLandmarkTransform.getRotation().y() + tagToBaseTransform.getRotation().y(),
        //   mapLandmarkTransform.getRotation().z() + tagToBaseTransform.getRotation().z());

        std::cout << getYaw(mapLandmarkTransform.getRotation())<<std::endl;

        // tf.setRotation(tf::createQuaternionFromYaw(atan2(tagToBaseTransform.getOrigin().y(),
        //                                                  tagToBaseTransform.getOrigin().x()) - getYaw(mapLandmarkTransform.getRotation())));

        double rotationDueToXY = atan2(tagToBaseTransform.inverse().getOrigin().y(),tagToBaseTransform.inverse().getOrigin().x());
        tf.setRotation(tf::createQuaternionFromYaw(getYaw(mapLandmarkTransform.getRotation()) + rotationDueToXY));

        // tf.setRotation(tf::createQuaternionFromYaw(differenceTransform.getRotation().z()));

        geometry_msgs::Transform msg_tf;
        tf::transformTFToMsg(tf, msg_tf);

        /*
        pose.position.x = mapLandmarkTransform.getOrigin().x() + tagToBaseTransform.getOrigin().x();
        pose.position.y = mapLandmarkTransform.getOrigin().y() + tagToBaseTransform.getOrigin().y();
        pose.position.z = 0;//mapLandmarkTransform.getOrigin().z() - differenceTransform.getOrigin().z() + tagToBaseTransform.getOrigin().z();

        // Quaternion for the rotation transform between the landmark and april tag's pose
        geometry_msgs::Quaternion q;
        q.x = mapLandmarkTransform.getRotation().x() + tagToBaseTransform.getRotation().x();
        q.y = mapLandmarkTransform.getRotation().y() + tagToBaseTransform.getRotation().y();
        q.z = mapLandmarkTransform.getRotation().z() + tagToBaseTransform.getRotation().z();

        pose.orientation = q;
        */

        pose.position.x = msg_tf.translation.x;
        pose.position.y = msg_tf.translation.y;
        pose.position.z = msg_tf.translation.z;
        pose.orientation = msg_tf.rotation;

        ROS_INFO("New Position:\ndx = %lf, dy = %lf, dz = %lf",
          pose.position.x, pose.position.y, pose.position.z);

        /*
        ROS_INFO("New Quaternion:\ndx = %lf, dy = %lf, dz = %lf",
          q.x, q.y, q.z);
        */

        geometry_msgs::PoseWithCovarianceStamped newRobotPose;
        newRobotPose.header.frame_id = "map";
        newRobotPose.header.stamp = differenceTransform.stamp_;

        poseWithCovariance.pose = pose;

        //create the covariance array (the values are based on what rviz sends)
        boost::array<double, 36> covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

        poseWithCovariance.covariance = covariance;
        newRobotPose.pose = poseWithCovariance;

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "map";
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.pose = pose;

        new_pose_pub.publish(poseStamped);
        new_initial_pose_pub.publish(newRobotPose);
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

  ros::NodeHandle nh;

  tf::TransformListener listener(nh);

  init(nh);

  // Initialize variables for loop
  ros::Rate r(10.0);

  while(nh.ok()){

    // Update callbacks
    ros::spinOnce();

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
        ps.pose.position.z = transform.getOrigin().z();
        geometry_msgs::Quaternion q;
        q.x=quat.x();
        q.y=quat.y();
        q.z=quat.z();
        ps.pose.orientation = q;
        tags_pub.publish(ps);
      }
    }

    r.sleep();
  }
}
