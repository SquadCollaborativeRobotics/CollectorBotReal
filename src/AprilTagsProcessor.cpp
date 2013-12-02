#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>

#include <math.h>

std::vector<std::string> landmark_frames;
std::vector<std::string> april_frames;

// Publisher that sends out an april tag that is a possible goal node?
ros::Publisher tags_pub;
// Publisher that publishes a pose for visualization purposes to the "/new_pose"
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
  static ros::Time lastPoseUpdateTime = ros::Time::now();

  if (ros::Time::now() - lastPoseUpdateTime > ros::Duration(5))
  {
    for (int i = 0; i < landmark_frames.size(); i++)
    {
      try{
        // if it can find a transformation between a landmark frame and april tag frame (i.e. both are in the tree)
        // watch out for tags still in tree but not seen in the camera
        if (listener.canTransform (landmark_frames[i], april_frames[i], ros::Time(0)))
        {
          tf::StampedTransform mapLandmarkTransform;
          tf::StampedTransform differenceTransform;
          tf::StampedTransform tagToBaseTransform;

          listener.lookupTransform(april_frames[i], "/base_link",
          ros::Time(0), tagToBaseTransform);
          listener.lookupTransform("map", landmark_frames[i],
          ros::Time(0), mapLandmarkTransform);
          listener.lookupTransform(landmark_frames[i], april_frames[i],
          ros::Time(0), differenceTransform);

          if (ros::Time::now() - tagToBaseTransform.stamp_ < ros::Duration(1.0))
          {

            //Create a pose based in the /map frame
            geometry_msgs::PoseWithCovariance poseWithCovariance;
            geometry_msgs::Pose pose;

            tf::Vector3 lm_vec = mapLandmarkTransform.getOrigin();
            tf::Vector3 april_vec = tagToBaseTransform.getOrigin();
            tf::Matrix3x3 rot_vec = mapLandmarkTransform.getBasis();

            // std::cout << tagToBaseTransform.getOrigin().x() <<" "<< tagToBaseTransform.getOrigin().y() <<" "<< tagToBaseTransform.getOrigin().z() <<" "<< std::endl;
            // std::cout << mapLandmarkTransform.getOrigin().x() <<" "<< mapLandmarkTransform.getOrigin().y() <<" "<< mapLandmarkTransform.getOrigin().z() <<" "<< std::endl;

            tf::Transform tf;
            tf.setOrigin(lm_vec + (april_vec * rot_vec));

            double yawMap = getYaw(mapLandmarkTransform.getRotation());
            double yawTag = getYaw(tagToBaseTransform.getRotation());

            tf.setRotation(tf::createQuaternionFromYaw(yawMap + yawTag));

            geometry_msgs::Transform msg_tf;
            tf::transformTFToMsg(tf, msg_tf);

            pose.position.x = msg_tf.translation.x;
            pose.position.y = msg_tf.translation.y;
            pose.position.z = 0;
            pose.orientation = msg_tf.rotation;

            ROS_INFO("New Robot Position:\nx = %lf, y = %lf, z = %lf",
            pose.position.x, pose.position.y, pose.position.z);

            geometry_msgs::PoseWithCovarianceStamped newRobotPose;
            newRobotPose.header.frame_id = "map";
            newRobotPose.header.stamp = tagToBaseTransform.stamp_;

            poseWithCovariance.pose = pose;

            // create the covariance array (the values are based on what rviz sends)
            // TODO: change based on confidence (distance to tag?)
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

            lastPoseUpdateTime = ros::Time::now();
          }
        }
      }
      catch (tf::TransformException ex){
        ROS_INFO("could not transform from %s to %s", landmark_frames[i].c_str(), april_frames[i].c_str());
        return false;
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AprilTagsProcessor_node");

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
