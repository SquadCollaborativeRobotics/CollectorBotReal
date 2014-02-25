#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>

#include <math.h>

// Signifies that that time variable has not yet been set
std::vector<std::string> landmark_frames;
std::vector<std::string> april_frames;

// Hard Coded Hack, probably need a data structure to hold
ros::Time last_pose_update_time;
bool last_pose_update_time_exists = false;

ros::Time first_seen_tag;
bool first_seen_tag_exists = false;

ros::Duration localization_delay(5.0);
ros::Duration tag_delay(1.5);
ros::Duration tag_timeout(3.0);

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

// If we see a tag X 
//  If haven't seen it before
//   check tag X seen at time A
//  else if time since last seen > t1
//   update
//  else if time since last seen > tdead
//   cancel/reset
//  else
//   update pose estimate with tag X
// do this not more than once every Y seconds

bool doneOnce = false;

bool shouldUpdate(){
  
  // Check for existence of whether or not we've localized before.
  if (!last_pose_update_time_exists){
    // Never localized.
    if (!first_seen_tag_exists){
      // First time seeing a tag, set first seen tag.
      first_seen_tag = ros::Time::now();
      first_seen_tag_exists = true;
      return false;
    }
    else if (ros::Time::now() - first_seen_tag > tag_timeout){
      // Robot sees for less than duration then drives away and comes back
      // Set first seen tag again.
      first_seen_tag = ros::Time::now();
      first_seen_tag_exists = true;
      return false;
    }
    else if (ros::Time::now() - first_seen_tag < tag_delay){
      // Haven't seen tag long enough, carry on.
      return false;
    }
    else{
      // Never localized and haven't seen tag long enough
      last_pose_update_time = ros::Time::now();
      // Reset tags for next usage
      first_seen_tag_exists = false;
      last_pose_update_time_exists = false;
      return true;
    }
  }
  else{
    // We've April Tag Localized Before
    if (ros::Time::now() - last_pose_update_time < localization_delay){
      // Not long enough to localize again
      return false;
    }
    else if (!first_seen_tag_exists){
      // Previously localized, first time seeing a tag
      first_seen_tag = ros::Time::now();
      first_seen_tag_exists = true;
      return false;
    }
    else if (ros::Time::now() - first_seen_tag > tag_timeout){
      // Robot sees for less than duration then drives away and comes back
      first_seen_tag = ros::Time::now();
      first_seen_tag_exists = true;
      return false;
    }
    else if (ros::Time::now() - first_seen_tag < tag_delay){
      // Not long enough seeing tag yet.
      return false;
    }
    else{
      // Previously Localized, but has been long enough and seen tag long enough
      last_pose_update_time = ros::Time::now();
      // Reset to false for next usage
      first_seen_tag_exists = false;
      last_pose_update_time_exists = false;
      return true;
    }
  }
}

bool AprilTagLocalize(tf::TransformListener &listener)
{
    
  for (int i = 0; i < landmark_frames.size(); i++)
  {
    
    try{
      // if it can find a transformation between a landmark frame and april tag frame (i.e. both are in the tree)
      // watch out for tags still in tree but not seen in the camera
      if (listener.canTransform (landmark_frames[i], april_frames[i], ros::Time(0)))
      {
        // shouldUpdate ASSUMES IT ONLY GETS CALLED WHEN AN APRIL TAG IS SEEN
        // Maybe introduce a parameter that tracks which april tag is seen?
        // ROS_INFO("Could update %d %d", first_seen_tag_exists, last_pose_update_time_exists);
        //
        // if (false){
        if (shouldUpdate() && !doneOnce){
          tf::StampedTransform map_landmark_transform;
          tf::StampedTransform difference_transform;
          tf::StampedTransform tag_to_base_transform;

          listener.lookupTransform(april_frames[i], "/base_link",
          ros::Time(0), tag_to_base_transform);
          listener.lookupTransform("map", landmark_frames[i],
          ros::Time(0), map_landmark_transform);
          listener.lookupTransform(landmark_frames[i], april_frames[i],
          ros::Time(0), difference_transform);

          // If tiem when transform was generated was less than 0.1 seconds ago.
          if (ros::Time::now() - tag_to_base_transform.stamp_ < ros::Duration(0.1))
          {

            //Create a pose based in the /map frame
            geometry_msgs::PoseWithCovariance poseWithCovariance;
            geometry_msgs::Pose pose;

            tf::Vector3 lm_vec = map_landmark_transform.getOrigin();
            tf::Vector3 april_vec = tag_to_base_transform.getOrigin();
            tf::Matrix3x3 rot_vec = map_landmark_transform.getBasis();

            // std::cout << tag_to_base_transform.getOrigin().x() <<" "<< tag_to_base_transform.getOrigin().y() <<" "<< tag_to_base_transform.getOrigin().z() <<" "<< std::endl;
            // std::cout << map_landmark_transform.getOrigin().x() <<" "<< map_landmark_transform.getOrigin().y() <<" "<< map_landmark_transform.getOrigin().z() <<" "<< std::endl;

            tf::Transform tf;
            tf.setOrigin(lm_vec + (april_vec * rot_vec));

            double yawMap = getYaw(map_landmark_transform.getRotation());
            double yawTag = getYaw(tag_to_base_transform.getRotation());

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
            newRobotPose.header.stamp = tag_to_base_transform.stamp_;

            poseWithCovariance.pose = pose;

            // create the covariance array (the values are based on what rviz sends)
            // TODO: change based on confidence (distance to tag?)
            // boost::array<double, 36> covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25,
            // 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            // 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};
            boost::array<double, 36> covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025};

            poseWithCovariance.covariance = covariance;
            newRobotPose.pose = poseWithCovariance;

            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.frame_id = "map";
            poseStamped.header.stamp = ros::Time::now();
            poseStamped.pose = pose;

            ROS_INFO("PUBLISHING NEW POSE ESTIMATE!");
            new_pose_pub.publish(poseStamped);
            new_initial_pose_pub.publish(newRobotPose);
            doneOnce = true;

            last_pose_update_time = ros::Time::now();
          }
        }
      }
    }

    catch (tf::TransformException ex){
      ROS_INFO("could not transform from %s to %s", landmark_frames[i].c_str(), april_frames[i].c_str());
      return false;
    }
  }
}

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
        sprintf(buf, "%d\n", i);
        ps.header.frame_id = buf;
        ps.pose.position.x = transform.getOrigin().x();
        ps.pose.position.y = transform.getOrigin().y();
        ps.pose.position.z = transform.getOrigin().z();

        geometry_msgs::Quaternion q;
        q.x=quat.x();
        q.y=quat.y();
        q.z=quat.z();
        ps.pose.orientation = q;
        // Publish at max rate of 1 Hz
        // not tied to ros::rate since want to see most current up to 10Hz
        if (ros::Time::now() - ros::Duration(1.0) > last_time) {
          tags_pub.publish(ps);
          last_time = ros::Time::now();
        }
      }
    }

    r.sleep();
  }
}
