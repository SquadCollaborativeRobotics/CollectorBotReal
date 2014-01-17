#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <global_planner/GarbagePosition.h>

#include <math.h>

// Signifies that that time variable has not yet been set
std::vector<std::string> landmark_frames;
std::vector<std::string> april_frames;

// Hard Coded Hack, probably need a data structure to hold
ros::Time last_pose_update_time;
bool last_pose_update_time_exists = false;

ros::Time first_seen_tag;
bool first_seen_tag_exists = false;

ros::Duration localization_delay(30.0);
ros::Duration tag_delay(1.5);
ros::Duration tag_timeout(5.0);

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

  tags_pub = nh.advertise<global_planner::GarbagePosition>("/garbageposition", 100);
  new_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/new_pose", 100);
  new_initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);

}

/**
 * Localize the robot using the april tags seen by the robot
 *
 * @param  listener [description]
 * @return          [description]
 */
bool AprilTagLocalize(tf::TransformListener &listener)
{
  for (int i = 0; i < landmark_frames.size(); i++)
  {
    try
    {
      // if it can find a transformation between a landmark frame and april tag frame (i.e. both are in the tree)
      // watch out for tags still in tree but not seen in the camera
      if (listener.canTransform (landmark_frames[i], april_frames[i], ros::Time(0)))
      {
        // shouldUpdate ASSUMES IT ONLY GETS CALLED WHEN AN APRIL TAG IS SEEN
        // Maybe introduce a parameter that tracks which april tag is seen?
        // ROS_INFO("Could update %d %d", first_seen_tag_exists, last_pose_update_time_exists)

        //The transform from the global map to the landmark
        tf::StampedTransform map_landmark_transform;
        //The transformation from tag to base
        tf::StampedTransform tag_to_base_transform;

        // transform from april tag to base link at the time the camera image was taken
        listener.lookupTransform(april_frames[i], "/base_link",
        tag_to_base_transform.stamp_, tag_to_base_transform);

        ROS_INFO_STREAM("Time of tag transform: "<<tag_to_base_transform.stamp_);

        // transform from april tag to base link at the time the camera image was taken
        listener.lookupTransform("/map", landmark_frames[i],
        tag_to_base_transform.stamp_, map_landmark_transform);

        // If tiem when transform was generated was less than 0.1 seconds ago.
        if (ros::Time::now() - tag_to_base_transform.stamp_ < ros::Duration(0.5))
        {
          tf::Vector3 lm_vec = map_landmark_transform.getOrigin();
          tf::Vector3 april_vec = tag_to_base_transform.getOrigin();
          tf::Matrix3x3 rot_vec = map_landmark_transform.getBasis();

          //Transform from map to robot!
          tf::Transform new_pose_tf;
          new_pose_tf.setOrigin(lm_vec + (april_vec * rot_vec));

          double yawMap = getYaw(map_landmark_transform.getRotation());
          double yawTag = getYaw(tag_to_base_transform.getRotation());

          new_pose_tf.setRotation(tf::createQuaternionFromYaw(yawMap + yawTag));

          // Find the distance the bot has moved while the camera image was processing
          // This will be added on after we figure out where the robot was at the time
          // of the image being taken
          tf::StampedTransform distance_moved_since_camera_image_taken;
          listener.lookupTransform("/base_link", tag_to_base_transform.stamp_,
          "/base_link", ros::Time::now(),
          "/map", distance_moved_since_camera_image_taken);

          //update the new pose with the distance moved since image taken
          new_pose_tf.setOrigin(new_pose_tf.getOrigin() + distance_moved_since_camera_image_taken.getOrigin());
          new_pose_tf.setRotation(new_pose_tf.getRotation() + tf::createQuaternionFromYaw(getYaw(distance_moved_since_camera_image_taken.getRotation())));

          // Convert to a geometry message, which AMCL accepts
          geometry_msgs::Transform msg_tf;
          tf::transformTFToMsg(new_pose_tf, msg_tf);

          //AMCL takes the pose as a PoseWithCovarianceStamped. Build the posewithcovariance first
          geometry_msgs::PoseWithCovariance poseWithCovariance;
          poseWithCovariance.pose.position.x = msg_tf.translation.x;
          poseWithCovariance.pose.position.y = msg_tf.translation.y;
          poseWithCovariance.pose.position.z = 0;
          poseWithCovariance.pose.orientation = msg_tf.rotation;

          ROS_INFO("New Robot Position:\nx = %lf, y = %lf, z = %lf",
          poseWithCovariance.pose.position.x, poseWithCovariance.pose.position.y, poseWithCovariance.pose.position.z);

          // Create a "stamped" message
          geometry_msgs::PoseWithCovarianceStamped newRobotPose;
          newRobotPose.header.frame_id = "map";
          newRobotPose.header.stamp = tag_to_base_transform.stamp_;

          // create the covariance array (the values are based on what rviz sends)
          // TODO: change based on confidence (distance to tag, speed of turning, etc)
          // row order covariance.... first row (0-5) = covariance from x, (6-12) = cov. from y, etc.
          // in this order: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
          boost::array<float, 36> covariance = {
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0, // there is some variance in x due to moving in x
            0.0, 0.05,0.0, 0.0, 0.0, 0.0,  // there is some variance in y due to moving in y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // No z motion occurs... no variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // no x rotation occurs
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // no y rotation ccurs
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06  // z rotation is fairly uncertain???
          };

          poseWithCovariance.covariance = covariance;
          newRobotPose.pose = poseWithCovariance;

          geometry_msgs::PoseStamped poseStamped;
          poseStamped.header.frame_id = "map";
          poseStamped.header.stamp = ros::Time::now();
          poseStamped.pose = poseWithCovariance.pose;

          // TODO: add code to check if the AMCL current pose is very different than the predicted one.
          // (ex: AMCL current pose differs more than 10 cm?)
          // if it is within a threshold don't update. Let AMCL keep working its magic until we are fairly
          // sure it's lost.
          ROS_INFO("PUBLISHING NEW POSE ESTIMATE!");
          new_pose_pub.publish(poseStamped);
          new_initial_pose_pub.publish(newRobotPose);

          last_pose_update_time = ros::Time::now();
        } //if time
      } //if cantransform
    } //try
    catch (tf::TransformException ex)
    {
      ROS_INFO("could not transform from %s to %s", landmark_frames[i].c_str(), april_frames[i].c_str());
      return false;
    } //catch
  }//for
}//func

ros::Time last_time;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AprilTagsProcessor_node");
  ROS_INFO("STARTING APRIL TAG PROCESSOR");

  ros::NodeHandle nh;

  tf::TransformListener listener(nh);

  last_time = ros::Time::now();

  init(nh);

// Initialize variables for loop
  ros::Rate r(10.0);

  while(nh.ok()){

  // Update callbacks
    ros::spinOnce();

    AprilTagLocalize(listener);

    //get the pose of the robot in the map frame and publish
    tf::StampedTransform transform; //transform from landmark to where the camera sees the april tag

    // Publish the goal positions on a topic in the map frame
    for (std::vector<std::string>::iterator it = april_frames.begin(); it != april_frames.end(); ++it)
    {
      if (listener.canTransform ("/map", it->c_str(), ros::Time(0)))
      {
        // Publish at max rate of 1 Hz
        // not tied to ros::rate since want to see most current up to 10Hz
        if (ros::Time::now() - ros::Duration(1.0) > last_time) {
          global_planner::GarbagePosition gp;
          geometry_msgs::PoseStamped ps;
          tf::Quaternion quat;

          listener.lookupTransform("/map", it->c_str(),
            ros::Time(0), transform);
          quat = transform.getRotation();

          // Get pose in map frame
          ps.header.frame_id = "/map";

          //Set garbage pose based on the transform
          ps.pose.position.x = transform.getOrigin().x();
          ps.pose.position.y = transform.getOrigin().y();
          ps.pose.position.z = transform.getOrigin().z();

          //Set garbage rotation from the transform
          geometry_msgs::Quaternion q;
          q.x=quat.x();
          q.y=quat.y();
          q.z=quat.z();
          ps.pose.orientation = q;

          //Finish creating message & publish
          gp.pose = ps;
          tags_pub.publish(ps);
          last_time = ros::Time::now();
        }
      }
    }

    r.sleep();
  }
}