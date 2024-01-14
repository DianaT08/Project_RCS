// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  // Setting pre-grasp approach

  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat

  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of eef during grasp
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");

  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& group, const std::string& table)
{
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;

  if(table == "table2")
  {
          orientation.setRPY(0, 0, tau/4);  // A quarter turn about the z-axis
          place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

          /* For place location, we set the value to the exact location of the center of the object. */
          place_location[0].place_pose.pose.position.x = 0;
          place_location[0].place_pose.pose.position.y = 0.5;
          place_location[0].place_pose.pose.position.z = 0.5;
  }

  else if (table == "table3")
  {
        orientation.setRPY(0, 0, 0);  // A quarter turn about the z-axis
        place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

        /* For place location, we set the value to the exact location of the center of the object. */
        place_location[0].place_pose.pose.position.x = 0;
        place_location[0].place_pose.pose.position.y = -0.5;
        place_location[0].place_pose.pose.position.z = 0.5;
  }

  // Setting pre-place approach
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2 or 3.
  group.setSupportSurfaceName(table);
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  collision_objects[0].primitives[0].dimensions.resize(2);
  collision_objects[0].primitives[0].dimensions[0] = 0.4;
  collision_objects[0].primitives[0].dimensions[1] = 0.2;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  collision_objects[1].primitives[0].dimensions.resize(2);
  collision_objects[1].primitives[0].dimensions[0] = 0.4; // Height
  collision_objects[1].primitives[0].dimensions[1] = 0.2; // Radius

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[2].ADD;

  // Add the third table where we will be placing the cube.
  collision_objects[3].id = "table3";
  collision_objects[3].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  collision_objects[3].primitives[0].dimensions.resize(2);
  collision_objects[3].primitives[0].dimensions[0] = 0.4; // Height
  collision_objects[3].primitives[0].dimensions[1] = 0.2; // Radius

  /* Define the pose of the table. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0;
  collision_objects[3].primitive_poses[0].position.y = -0.5;
  collision_objects[3].primitive_poses[0].position.z = 0.2;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;

  collision_objects[3].operation = collision_objects[3].ADD;

  // Apply the collision objects to the scene
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void resetObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                 const std::string& object_id)
{
    // Remove the object from its current position
    std::vector<std::string> object_ids = {object_id};
    planning_scene_interface.removeCollisionObjects(object_ids);

    // Define the object at its initial position
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "panda_link0";
    collision_object.id = object_id;
    collision_object.primitives.resize(1);
    collision_object.primitives[0].type = collision_object.primitives[0].BOX;
    collision_object.primitives[0].dimensions.resize(3);
    collision_object.primitives[0].dimensions[0] = 0.02;
    collision_object.primitives[0].dimensions[1] = 0.02;
    collision_object.primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_object.primitive_poses.resize(1);
    collision_object.primitive_poses[0].position.x = 0.5;
    collision_object.primitive_poses[0].position.y = 0;
    collision_object.primitive_poses[0].position.z = 0.5;
    collision_object.primitive_poses[0].orientation.w = 1.0;

    collision_object.operation = collision_object.ADD;

    // Add the object back to the scene at the initial position
    planning_scene_interface.applyCollisionObject(collision_object);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  while(ros::ok())
  {
    pick(group);

    ros::WallDuration(1.0).sleep();

    std::string table = (rand() % 2 == 0) ? "table2" : "table3";
    place(group, table);
    ros::WallDuration(2.0).sleep();
    resetObject(planning_scene_interface, "object");
    ros::WallDuration(2.0).sleep();
  }


  ros::waitForShutdown();
  return 0;
}