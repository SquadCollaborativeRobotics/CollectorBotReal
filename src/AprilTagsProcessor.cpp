#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <global_planner/GarbagePosition.h>

#include <math.h>

//*****************************************
// Landmark and frame vectors
//*****************************************
std::vector<std::string> landmark_frames;
std::vector<std::string> april_frames;

//*****************************************
// variables used to control rate of updates
//*****************************************
// Hard Coded Hack, probably need a data structure to hold
ros::Time last_pose_update_time;
// Signifies that that time variable has not yet been set
bool last_pose_update_time_exists = false;

ros::Time first_seen_tag;
bool first_seen_tag_exists = false;

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


//*****************************************
// Used for testing to tell the robot to stop once a tag is seen
//*****************************************
#ifdef TEST_TAGS_STOP
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel_test_msg;
#endif

//Current velocity estimate
ros::Subscriber cmd_vel_sub;
ros::Subscriber cmd_state_sub;
ros::Publisher cmd_state_pub;
geometry_msgs::Twist curr_cmd_vel;
std_msgs::Int32 curr_state_value;
std_msgs::Int32 stored_state;

//Variables for controlling the state of the bot in terms of moving while april tag is being read
enum DetectionState
{
  NoTagsSeen = 0,
  TagInFrame = 1
};

DetectionState current_detection_state;
bool needAnotherLook;
ros::Time timeOfLastLook;


void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
  curr_cmd_vel = *msg;
}

void commandCallback(const std_msgs::Int32::ConstPtr& msg){
  curr_state_value.data = msg->data;
  ROS_ERROR_STREAM("Received cmd_state: "<<curr_state_value.data);

  //account for a race condition where the goal is met while we are waiting for a second look at a tag
  if (curr_state_value.data != 0)
  {
    stored_state.data = curr_state_value.data;
    ROS_INFO_STREAM("Stored state = "<<stored_state.data);
  }
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

int stopRobot()
{
  stored_state = curr_state_value;
  int curr_state = curr_state_value.data;
  ROS_INFO_STREAM("Stopping robot: Last state = "<<stored_state);

  std_msgs::Int32 stop_msg;
  stop_msg.data = 0;
  cmd_state_pub.publish(stop_msg);
  return curr_state;
}

int resumePlanner()
{
  ROS_INFO_STREAM("Resuming state to: "<<stored_state.data);
  cmd_state_pub.publish(stored_state);
  return stored_state.data;
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

  tags_pub = nh.advertise<global_planner::GarbagePosition>("/garbageposition", 100);
  new_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/new_pose", 100);
  new_initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 100);
  cmd_state_pub = nh.advertise<std_msgs::Int32>("/cmd_state", 100);

  cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmdVelCallback);
  cmd_state_sub = nh.subscribe("cmd_state", 10, commandCallback);

  #ifdef TEST_TAGS_STOP
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  cmd_vel_test_msg.linear.x = 0;
  cmd_vel_test_msg.linear.y = 0;
  cmd_vel_test_msg.linear.z = 0;
  cmd_vel_test_msg.angular.x = 0;
  cmd_vel_test_msg.angular.y = 0;
  cmd_vel_test_msg.angular.z = 0;
  #endif

  current_detection_state = NoTagsSeen;
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
 * Localize the robot using the april tags seen by the robot
 * @param  listener - Transform listener. passed because this should be converted to a more
 * object oriented program later
 * @return true if localization occured, false otherwise
 */
bool AprilTagLocalize(tf::TransformListener &listener)
{
  static const double minTimeBetweenUpdates = 9.0;
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
          if (GetDistance(tag_to_camera_rgb_transform) > 2.1)
          {
            ROS_WARN_STREAM_THROTTLE(1,"Distance to tag is too far for accurate localization: "
              <<GetDistance(tag_to_camera_rgb_transform)<<" m");
            continue;
          }

          //Do we need to stop and get a more accurate reading from the tag
          if (needAnotherLook)
          {
            ROS_INFO("Need another look...");
            timeOfLastLook = tag_to_camera_rgb_transform.stamp_;
            stopRobot();
            //We have stopped, let's get a good reading
            needAnotherLook = false;
            return false;
          }
          else
          {
            //If we get stuck in a position where we can't see any tags and have no motion, we should just continue on with the planner
            if (now - timeOfLastLook > ros::Duration(5))
            {
              ROS_ERROR_STREAM("Got stuck... resuming plan?");
              needAnotherLook=true;
              resumePlanner();
              continue;
            }

            if (curr_cmd_vel.linear.x > 0.02 || curr_cmd_vel.linear.y > 0.02
              || curr_cmd_vel.angular.z > 0.025)
            {
              //needAnotherLook = true;
              ROS_WARN_STREAM_THROTTLE(1,"Going too fast for accurate localization");
              continue;
            }
            
            ROS_INFO_STREAM_THROTTLE(1,"Looking again now that we're stopped and primed to receive");
            // Now that we're stopped, make sure the robot has had time to settle
            // and the camera has a fresh, clear image
            if (tag_to_camera_rgb_transform.stamp_ - timeOfLastLook  > ros::Duration(1.0))
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

              listener.waitForTransform("/base_link", now,
                                        "/base_link", past,
                                        "map", ros::Duration(0.5));
              listener.lookupTransform("/base_link", now,
                                      "/base_link", past,
                                      "/map", distance_moved_since_camera_image_taken);

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

              // Create a "stamped" message
              geometry_msgs::PoseWithCovarianceStamped newRobotPose;
              newRobotPose.header.frame_id = "map";
              newRobotPose.header.stamp = tag_to_camera_rgb_transform.stamp_;

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
              poseWithCovariance.covariance = covariance;
              newRobotPose.pose = poseWithCovariance;

              geometry_msgs::PoseStamped poseStamped;
              poseStamped.header.frame_id = "map";
              poseStamped.header.stamp = now;
              poseStamped.pose = poseWithCovariance.pose;

              // TODO: add code to check if the AMCL current pose is very different than the predicted one.
              // (ex: AMCL current pose differs more than 10 cm?)
              // if it is within a threshold don't update. Let AMCL keep working its magic until we are fairly
              // sure it's lost.

              new_pose_pub.publish(poseStamped);
              new_initial_pose_pub.publish(newRobotPose);

              last_pose_update_time = now;

              needAnotherLook = true; // reset the trigger to require looking again next time
              ros::spinOnce(); //send the pose

              //wait for amcl to reinitialize
              ROS_INFO_STREAM("Waiting for AMCL to reinitialize");

              sleep(1);
              ROS_INFO("Resuming planner");
              resumePlanner();
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
  for (std::vector<std::string>::iterator it = april_frames.begin(); it != april_frames.end(); ++it)
  {
    if (listener.canTransform ("/map", it->c_str(), ros::Time(0)))
    {
      // Publish at max rate of 1 Hz
      // not tied to ros::rate since want to see most current up to 10Hz
      if (ros::Time::now() - ros::Duration(1.0) > last_goal_send_time) {
        tf::StampedTransform transform;
        
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
        tags_pub.publish(gp);
        last_goal_send_time = ros::Time::now();
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
