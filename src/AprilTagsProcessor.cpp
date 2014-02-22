#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <global_planner/GoalMsg.h>

#include <math.h>

//*****************************************
// Landmark and frame vectors
//*****************************************
std::vector<std::string> landmark_frames;
std::vector<std::string> april_frames;
std::vector<std::string> goal_frames;

//*****************************************
// variables used to control rate of updates
//*****************************************
// Hard Coded Hack, probably need a data structure to hold
ros::Time last_pose_update_time;

//*****************************************
// Ros Publishers
//*****************************************
// Publisher that sends out an april tag that is a possible goal node?
ros::Publisher tags_pub;
// Publisher that publishes a pose for visualization purposes to the "/new_pose"
ros::Publisher new_pose_pub;
// Publisher that reinitializes the pose of the robot (relocalizes)
ros::Publisher new_initial_pose_pub;

//*****************************************
// A few constant (static) transforms
//*****************************************
//The transformation from camera_rgb to camera frame
tf::StampedTransform camera_rgb_to_camera_link_transform;
// transform from camera to base_link. first we have to go from landmark to camera, then to
// base_link
tf::StampedTransform camera_to_base_link_transform;

//Current velocity
ros::Subscriber odom_sub;
nav_msgs::Odometry curr_odom;

//Variables for helping the accuracy of the robot
bool needAnotherLook;
ros::Time timeOfLastLook;

//Variables dealing with the global planner state
ros::Subscriber cmd_state_sub;
ros::Publisher cmd_state_pub;
std_msgs::Int32 curr_state_value;
std_msgs::Int32 stored_state;

//get amcl_pose
ros::Subscriber amcl_pose_sub;
geometry_msgs::PoseWithCovarianceStamped amcl_pose;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  curr_odom = *msg;
}

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  amcl_pose.pose = msg->pose;
  amcl_pose.header = msg->header;
  ROS_INFO_STREAM_THROTTLE(1,"Got amcl pose");
}

void currStateCallback(const std_msgs::Int32::ConstPtr& msg){
  curr_state_value.data = msg->data;
  ROS_ERROR_STREAM("Received cmd_state: "<<curr_state_value.data);
}

/**
 * Prints out a StampedTransform class
 * @param transform
 */
void PrintTransform(tf::StampedTransform& transform)
{
  ROS_INFO_STREAM("Time: "<<transform.stamp_<<" | Parent: "<<transform.frame_id_<<" | Child?: "<<transform.child_frame_id_<<"\nTransform:");
  tf::Matrix3x3 mat = transform.getBasis();
  for (int i=0 ; i<3; i++)
  {
    std::cout << "[";
    for (int j=0; j<3; j++)
    {
      std::cout<<mat[i][j]<<" ";
    }
    std::cout << transform.getOrigin()[i] << "]" << std::endl;
  }
  std::cout << "]";
}

/**
 * Communicates with the global planner to stop the robot so the system can
 * @return [description]
 */
void pauseRobot()
{
  ROS_INFO_STREAM("Stopping robot: Current state = "<<curr_state_value.data);

  std_msgs::Int32 stop_msg;
  stop_msg.data = 99; // pause state = 99
  cmd_state_pub.publish(stop_msg);
  ros::spinOnce();
}

void resumeRobot()
{
  ROS_INFO_STREAM("Resuming Robot");
  std_msgs::Int32 resume_msg;
  resume_msg.data = 100; //resume state = 100
  cmd_state_pub.publish(resume_msg);
  ros::spinOnce();
}

/**
 * initialize variables & setup publishers/subscribers
 * @param nh
 */
void init(ros::NodeHandle nh)
{
  ros::spinOnce();
  landmark_frames.push_back(std::string("/landmark_3"));
  landmark_frames.push_back(std::string("/landmark_5"));
  april_frames.push_back(std::string("/april_tag[3]"));
  april_frames.push_back(std::string("/april_tag[5]"));
  goal_frames.push_back(std::string("/april_tag[6]"));

  tags_pub = nh.advertise<global_planner::GoalMsg>("garbageCan", 100);
  new_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/new_pose", 100);
  new_initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);
  cmd_state_pub = nh.advertise<std_msgs::Int32>("/cmd_state", 100);

  odom_sub = nh.subscribe("odom", 10, odomCallback);
  cmd_state_sub = nh.subscribe("curr_cmd_state", 10, currStateCallback);
  amcl_pose_sub = nh.subscribe("amcl_pose", 10, amclPoseCallback);

  needAnotherLook = true;
  timeOfLastLook = ros::Time(0);
}

/**
 * Get the magnitude of the transform in the x,y plane
 * @param  tf transform to calculate
 * @return    magnitude of the TF x,y coordinates
 */
double GetDistance(tf::StampedTransform &tf)
{
  return sqrt(tf.getOrigin()[0]*tf.getOrigin()[0] + tf.getOrigin()[1]*tf.getOrigin()[1]);
}

/**
 * Compares 2 poses and returns the abs(distance) between them (taking into account covariance...)
 * @param  pose1
 * @param  pose2
 * @return true if a significant difference between the 2 poses
 */
bool PosesDiffer(geometry_msgs::PoseWithCovariance& poseWithCovariance1, geometry_msgs::PoseWithCovariance& poseWithCovariance2)
{
  geometry_msgs::Pose pose1 = poseWithCovariance1.pose;
  geometry_msgs::Pose pose2 = poseWithCovariance2.pose;

  double diffX = abs(pose1.position.x - pose2.position.x);
  double diffY = abs(pose1.position.x - pose2.position.x);
  double diffTheta = abs(pose1.orientation.z - pose2.orientation.z);

  //Check linear distance
  static const double considerableDistance = 0.15; //15 cm difference is too much
  if (sqrt(diffX*diffX + diffY*diffY) > considerableDistance)
    return true;

  //Check difference in theta
  static const double considerableDifferenceTheta = 0.26; //15 degrees is too much
  if (diffTheta > considerableDifferenceTheta)
    return true;
}

/**
 * Localize the robot using the april tags seen by the robot
 * @param  listener - Transform listener. passed because this should be converted to a more
 * object oriented program later
 * @return true if localization occured, false otherwise
 */
bool AprilTagLocalize(tf::TransformListener &listener)
{
  static const double minTimeBetweenUpdates = 10.5;
  if (ros::Time::now() - last_pose_update_time < ros::Duration(minTimeBetweenUpdates))
  {
    ROS_INFO_STREAM_THROTTLE(1,"Not checking... not going to update quicker than once every "<<minTimeBetweenUpdates<<" seconds");
    return false;
  }

  //Look for all landmarks
  for (int i = 0; i < landmark_frames.size(); i++)
  {
    try
    {
      // if it can find a transformation between a landmark frame and april tag frame (i.e. both are in the tree)
      // watch out for tags still in tree but not seen in the camera
      if (listener.canTransform(landmark_frames[i], april_frames[i], ros::Time(0)))
      {
        //The transform from the global map to the landmark
        tf::StampedTransform map_to_landmark_transform;
        //The transformation from tag to camera rgb frame
        tf::StampedTransform tag_to_camera_rgb_transform;

        ros::Time now = ros::Time::now();

        // transform from the global map coordinates to the landmark
        listener.lookupTransform("/map", landmark_frames[i],
          ros::Time(0), map_to_landmark_transform);

        // transform from april tag to the camera optical frame
        listener.lookupTransform(april_frames[i], "/camera_rgb_frame",
          ros::Time(0), tag_to_camera_rgb_transform);

        listener.lookupTransform("/camera_rgb_frame", "/camera_link",
          ros::Time(0), camera_rgb_to_camera_link_transform);
        // transform from camera to base link
        listener.lookupTransform("/camera_link", "/base_link",
          ros::Time(0), camera_to_base_link_transform);

        // If time when tag was seen was less than 1 second ago.
        if (now - tag_to_camera_rgb_transform.stamp_ < ros::Duration(1))
        {
          if (GetDistance(tag_to_camera_rgb_transform) > 2.2)
          {
            ROS_WARN_STREAM_THROTTLE(3,"Distance to tag is too far for accurate localization: "
              <<GetDistance(tag_to_camera_rgb_transform)<<" m");
            continue;
          }

          //Do we need to stop and get a more accurate reading from the tag
          if (needAnotherLook)
          {
            ROS_INFO("Need another look...");
            timeOfLastLook = tag_to_camera_rgb_transform.stamp_;
            pauseRobot();
            //We have stopped, let's get a good reading
            needAnotherLook = false;
            return false;
          }
          else
          {
            ROS_INFO_STREAM_THROTTLE(1,"Looking again now that we're stopped and primed to receive");
            //If we get stuck in a position where we can't see any tags and have no motion, we should just continue on with the planner
            if (now - timeOfLastLook > ros::Duration(5))
            {
              ROS_ERROR_STREAM("Got stuck... resuming plan?");
              needAnotherLook=true;
              continue;
            }

            //Check to make sure we are stopped...
            bool stillMoving = false;
            if (curr_odom.twist.twist.linear.x > 0.01 || curr_odom.twist.twist.linear.y > 0.01
              || curr_odom.twist.twist.angular.z > 0.01)
            {
              //needAnotherLook = true;
              ROS_WARN_STREAM_THROTTLE(1,"Going too fast for accurate localization");
              ROS_INFO_STREAM(curr_odom.twist.twist.linear.x<<' '<<curr_odom.twist.twist.linear.y<<' '<<curr_odom.twist.twist.angular.z);
              stillMoving = true;
            }

            // Now that we're stopped, make sure the robot has had time to settle
            // and the camera has a fresh, clear image
            if ((tag_to_camera_rgb_transform.stamp_ - timeOfLastLook  > ros::Duration(1.5)) && !stillMoving)
            {
              ROS_INFO_STREAM("Performing pose update");
              //Transform from landmark to camera
              tf::Transform new_pose_tf;

              new_pose_tf = map_to_landmark_transform * tag_to_camera_rgb_transform * camera_rgb_to_camera_link_transform * camera_to_base_link_transform;

              // Find the distance the bot has moved while the camera image was processing
              // This will be added on after we figure out where the robot was at the time
              // of the image being taken
              tf::StampedTransform distance_moved_since_camera_image_taken;

              // past is used for determining the time at which to go back for calculating distance traveled since camera image was taken
              ros::Time past = tag_to_camera_rgb_transform.stamp_;

              listener.waitForTransform("base_link", now,
                                        "base_link", past,
                                        "map", ros::Duration(0.5));
              listener.lookupTransform("base_link", now,
                                      "base_link", past,
                                      "map", distance_moved_since_camera_image_taken);

              PrintTransform(distance_moved_since_camera_image_taken);

              if (distance_moved_since_camera_image_taken.getOrigin()[0] < 1.0 && distance_moved_since_camera_image_taken.getOrigin()[1] < 1.0)
              {
                //update the new pose with the distance moved since image taken
                //new_pose_tf = new_pose_tf * distance_moved_since_camera_image_taken;
              }

              // Convert to a geometry message, which is what AMCL accepts
              geometry_msgs::Transform msg_tf;
              tf::transformTFToMsg(new_pose_tf, msg_tf);

              //AMCL takes the pose as a PoseWithCovarianceStamped. Build the poseWithCovariance first
              geometry_msgs::PoseWithCovariance poseWithCovariance;
              poseWithCovariance.pose.position.x = msg_tf.translation.x;
              poseWithCovariance.pose.position.y = msg_tf.translation.y;
              poseWithCovariance.pose.position.z = msg_tf.translation.z;
              poseWithCovariance.pose.orientation = msg_tf.rotation;

              ROS_INFO("New Robot Position:\nx = %lf, y = %lf, z = %lf",
              poseWithCovariance.pose.position.x, poseWithCovariance.pose.position.y, poseWithCovariance.pose.position.z);

              // create the covariance array (the values are based on what rviz sends)
              // TODO: change based on confidence (distance to tag, speed of turning, etc)
              // row order covariance.... first row (0-5) = covariance from x, (6-12) = cov. from y, etc.
              // in this order: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
              boost::array<float, 36> covariance = {
                0.03, 0.0, 0.0, 0.0, 0.0, 0.0, // there is some variance in x due to moving in x
                0.0, 0.03,0.0, 0.0, 0.0, 0.0,  // there is some variance in y due to moving in y
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // No z motion occurs... no variance
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // no x rotation occurs
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // no y rotation ccurs
                0.0, 0.0, 0.0, 0.0, 0.0, 0.05  // z rotation is fairly uncertain???
              };

              // finish creating message to send to AMCL

              // Create a "stamped" message
              geometry_msgs::PoseWithCovarianceStamped newRobotPose;
              newRobotPose.header.frame_id = "map";
              newRobotPose.header.stamp = tag_to_camera_rgb_transform.stamp_;

              poseWithCovariance.covariance = covariance;
              newRobotPose.pose = poseWithCovariance;

              //Use for display purposes...
              geometry_msgs::PoseStamped poseStamped;
              poseStamped.header.frame_id = "map";
              poseStamped.header.stamp = now;
              poseStamped.pose = poseWithCovariance.pose;

              // check if the AMCL current pose is very different than the predicted one.
              // (i.e.: AMCL current pose differs more than threshold)
              // if it is within a threshold don't update. Let AMCL keep working its magic until we are fairly
              // sure it's lost.

              if (PosesDiffer(amcl_pose.pose, newRobotPose.pose))
              {
                new_pose_pub.publish(poseStamped);
                new_initial_pose_pub.publish(newRobotPose);

                //wait for amcl to reinitialize
                ROS_INFO_STREAM("Waiting for AMCL to reinitialize");

                sleep(2.0);
              }
              else
              {
                ROS_WARN_STREAM("Poses do not differ enough to need april tag localization");
              }

              last_pose_update_time = now;

              needAnotherLook = true; // reset the trigger to require looking again next time
              ros::spinOnce(); //send the pose

              ROS_INFO("Resuming planner");
              sleep(0.5);
              resumeRobot();
              return true;
            }
          }
        } //if time
      } //if cantransform
    } //try
    catch (tf::TransformException ex)
    {
      ROS_ERROR_THROTTLE(2, "could not transform from %s to %s", landmark_frames[i].c_str(), april_frames[i].c_str());
      ROS_ERROR_STREAM_THROTTLE(2,ex.what());
    } //catch
  }//for
  //ROS_INFO_STREAM_THROTTLE(3, "No tags seen");
  return false;
}//func

ros::Time last_goal_send_time;

void PublishGoalPoses(tf::TransformListener& listener)
{
  // Publish the goal positions on a topic in the map frame
  for (std::vector<std::string>::iterator it = goal_frames.begin(); it != goal_frames.end(); ++it)
  {
    if (listener.canTransform ("/map", it->c_str(), ros::Time(0)))
    {
      // Publish at max rate of 1 Hz
      // not tied to ros::rate since want to see most current up to 10Hz
      if (ros::Time::now() - ros::Duration(5.0) > last_goal_send_time) {
        tf::StampedTransform transform;

        global_planner::GoalMsg can;
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
        geometry_msgs::Quaternion quatMsg;
        quatMsg.x=quat.x();
        quatMsg.y=quat.y();
        quatMsg.z=quat.z();
        quatMsg.w=quat.w();
        ps.pose.orientation = quatMsg;

        //Finish creating message & publish
        can.pose = ps.pose;
        //TODO: make more correct
        can.id = 6;

        tags_pub.publish(can);
        last_goal_send_time = ros::Time::now();

        ROS_ERROR("Sent goal pose");
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AprilTagsProcessor_node");
  ROS_INFO("STARTING APRIL TAG PROCESSOR");

  ros::NodeHandle nh;

  tf::TransformListener listener(nh);

  last_goal_send_time = ros::Time::now();
  init(nh);

// Initialize variables for loop
  ros::Rate r(10.0);

  while(nh.ok()){
    //ROS_INFO_STREAM_THROTTLE(5,"Main loop");
  // Update callbacks
    ros::spinOnce();

    AprilTagLocalize(listener);

    PublishGoalPoses(listener);

    ros::spinOnce();
    r.sleep();
  }
}
