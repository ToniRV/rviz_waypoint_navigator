/*
 * Copyright (c) 2017, Antoni Rosinol Vidal.
 * All rights reserved. *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rviz_waypoint_navigator/rviz_waypoint_navigator.h>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <std_srvs/Empty.h>

#include <waypoint_navigator/GoToWaypoint.h>
#include <waypoint_navigator/GoToWaypoints.h>

#include <math.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
ros::ServiceClient send_waypoint_;
ros::ServiceClient plan_waypoints_;
ros::ServiceClient execute_waypoints_;
ros::ServiceClient abort_path_;
ros::Publisher pose_pub_;
std::string frame_id_;
float scale_;
int num_rotors_;
float arm_len_;
float body_width_;
float body_height_;
float r_color_;
float g_color_;
float b_color_;

void processFeedback(
    const InteractiveMarkerFeedbackConstPtr& feedback) {
  static geometry_msgs::PoseStamped command_pose;
  static waypoint_navigator::GoToWaypoints::Request wp_request;
  static bool first_pose_update = false;

  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if(feedback->mouse_point_valid)
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch (feedback->event_type) {
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM(s.str() << ": button click" << mouse_point_ss.str() << ".");
      break;

    case InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM(s.str()
               << ": " << ((feedback->menu_entry_id == 1)?
                             "\e[1mSending command!\e[0m":"Cancel") << '\n'
               << " Using frame with id " << frame_id_ << ".");
      // Danger, potentially destructive commands in real life, handle with care.
      switch (feedback->menu_entry_id) {
      case 1:
        if (first_pose_update == true) {
          pose_pub_.publish(command_pose);
        } else {
          ROS_ERROR("No pose update received, please move around the copter.");
        }
        break;
      case 2:
      {
        if (first_pose_update == true) {
          // Do Safety checks, such as avoiding sending commands if the distance btw
          // drone and goal are too far away.
          waypoint_navigator::GoToWaypoint::Request wp_request;
          wp_request.point.x = command_pose.pose.position.x;
          wp_request.point.y = command_pose.pose.position.y;
          wp_request.point.z = command_pose.pose.position.z;
          waypoint_navigator::GoToWaypoint::Response wp_response;
          send_waypoint_.call(wp_request, wp_response);
        } else {
          ROS_ERROR("No pose update received, please move around the copter.");
        }
      }
        break;
      case 3:
      {
        ROS_INFO("Recording current pose as a waypoint.");
        waypoint_navigator::GoToWaypoint::Request wp;
        wp.point.x = command_pose.pose.position.x;
        wp.point.y = command_pose.pose.position.y;
        wp.point.z = command_pose.pose.position.z;
        wp_request.points.push_back(wp.point);
        ROS_INFO_STREAM("Current number of waypoints: " << wp_request.points.size());
        waypoint_navigator::GoToWaypoints::Response wp_response;
        plan_waypoints_.call(wp_request, wp_response);
      }
        break;
      case 4:
      {
        if (wp_request.points.size() > 0) {
          std_srvs::Empty::Request request;
          std_srvs::Empty::Response response;
          execute_waypoints_.call(request, response);
          wp_request.points.clear();
        } else {
          ROS_ERROR("No stored waypoints, please save some waypoints before.");
        }
      }
        break;
      case 5:
      {
        std_srvs::Empty::Request request;
        std_srvs::Empty::Response response;
        abort_path_.call(request, response);
        wp_request.points.clear();
      }
      }

      break;

    case InteractiveMarkerFeedback::POSE_UPDATE:
      first_pose_update = true;
      ROS_INFO_STREAM(s.str() << ": pose changed\n"
          << "position = \n"
          << "x: " << feedback->pose.position.x << '\n'
          << "y: " << feedback->pose.position.y << '\n'
          << "z: " << feedback->pose.position.z << '\n'
          << "orientation = \n"
          << "w: " << feedback->pose.orientation.w << '\n'
          << "x: " << feedback->pose.orientation.x << '\n'
          << "y: " << feedback->pose.orientation.y << '\n'
          << "z: " << feedback->pose.orientation.z << '\n'
          << " frame: " << feedback->header.frame_id << '\n'
          << " time: "  << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec. \n"
          << "Storing new command pose...");
      command_pose.header = feedback->header;
      command_pose.header.frame_id = frame_id_;
      command_pose.pose = feedback->pose;
      break;

    case InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM(s.str() << ": mouse down" << mouse_point_ss.str() << ".");
      break;

    case InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
      break;
  }

  server->applyChanges();
}

void alignMarker(
    const InteractiveMarkerFeedbackConstPtr &feedback) {
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  ROS_INFO_STREAM(feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z);

  server->setPose(feedback->marker_name, pose);
  server->applyChanges();
}

void saveMarker(InteractiveMarker int_marker) {
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

double rand(double min, double max) {
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

//////////////////////////// Makers ////////////////////////////////////////////
Marker makeBox(const InteractiveMarker& msg) {
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl(InteractiveMarker& msg) {
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);
  return msg.controls.back();
}

void makeRandomDofMarker(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale_;

  int_marker.name = "6dof_random_axes";
  int_marker.description = "6-DOF\n(Arbitrary Axes)";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  for (int i=0; i<3; i++)
  {
    control.orientation.w = rand(-1,1);
    control.orientation.x = rand(-1,1);
    control.orientation.y = rand(-1,1);
    control.orientation.z = rand(-1,1);
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void makeViewFacingMarker(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale_;

  int_marker.name = "view_facing";
  int_marker.description = "View Facing 6-DOF";

  InteractiveMarkerControl control;

  // make a control that rotates around the view axis
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation.w = 1;
  control.name = "rotate";

  int_marker.controls.push_back(control);

  // create a box in the center which should not be view facing,
  // but move in the camera plane.
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;
  control.name = "move";

  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;

  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void makeQuadrocopterMarker(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale_;

  int_marker.name = "quadcopter";
  int_marker.description = "Quadcopter";

  InteractiveMarkerControl control;
  control.name="copter_viz";
  control.always_visible = true;
  control.interaction_mode = InteractiveMarkerControl::NONE;
  if (num_rotors_ <= 0) num_rotors_ = 2;

  /** Hexacopter marker code adapted from libsfly_viz
   *  thanks to Markus Achtelik.
   */

  // rotor marker template
  float marker_scale = 1.0;
  Marker rotor;
  rotor.ns = "vehicle_rotor";
  rotor.action = Marker::ADD;
  rotor.type = Marker::CYLINDER;
  rotor.scale.x = 0.2 * marker_scale;
  rotor.scale.y = 0.2 * marker_scale;
  rotor.scale.z = 0.01 * marker_scale;
  rotor.color.r = 0.4;
  rotor.color.g = 0.4;
  rotor.color.b = 0.4;
  rotor.color.a = 0.8;
  rotor.pose.position.z = 0;

  // arm marker template
  Marker arm;
  arm.ns = "vehicle_arm";
  arm.action = Marker::ADD;
  arm.type = Marker::CUBE;
  arm.scale.x = arm_len_ * marker_scale;
  arm.scale.y = 0.02 * marker_scale;
  arm.scale.z = 0.01 * marker_scale;
  arm.color.r = 0.0;
  arm.color.g = 0.0;
  arm.color.b = 1.0;
  arm.color.a = 1.0;
  arm.pose.position.z = -0.015 * marker_scale;

  float angle_increment = 2 * M_PI / num_rotors_;

  for (float angle = angle_increment / 2; angle <= (2 * M_PI); angle += angle_increment)
  {
    rotor.pose.position.x = arm_len_ * cos(angle) * marker_scale;
    rotor.pose.position.y = arm_len_ * sin(angle) * marker_scale;
    rotor.id++;

    arm.pose.position.x = rotor.pose.position.x / 2;
    arm.pose.position.y = rotor.pose.position.y / 2;
    arm.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    arm.id++;

    control.markers.push_back(rotor);
    control.markers.push_back(arm);
  }

  // body marker template
  Marker body;
  body.ns = "vehicle_body";
  body.action = Marker::ADD;
  body.type = Marker::CUBE;
  body.scale.x = body_width_ * 0.75 *marker_scale;
  body.scale.y = body_width_ * 0.75 *marker_scale;
  body.scale.z = body_height_ * 0.75 *marker_scale;
  body.color.r = r_color_;
  body.color.g = g_color_;
  body.color.b = b_color_;
  body.color.a = 0.8;

  control.markers.push_back(body);
  // The above is only for visualization
  int_marker.controls.push_back(control);

  // These are for proper control
  control.markers.clear();
  control.always_visible = false;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.name = "rotate_z";
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  int_marker.controls.push_back(control);

  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  InteractiveMarkerControl control_menu;

  control_menu.interaction_mode = InteractiveMarkerControl::MENU;
  control_menu.name = "menu_only_control";

  control_menu.always_visible = true;
  int_marker.controls.push_back(control_menu);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply(*server, int_marker.name);
}

void makeChessPieceMarker(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale_;

  int_marker.name = "chess_piece";
  int_marker.description = "Chess Piece\n(2D Move + Alignment)";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back(makeBox(int_marker));
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  server->setCallback(int_marker.name, &alignMarker, InteractiveMarkerFeedback::POSE_UPDATE);
}

void makePanTiltMarker(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale_;

  int_marker.name = "pan_tilt";
  int_marker.description = "Pan / Tilt";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control.orientation_mode = InteractiveMarkerControl::INHERIT;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void makeMenuMarker(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale_;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  Marker marker = makeBox(int_marker);
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  menu_handler.apply(*server, int_marker.name);
}

void makeButtonMarker(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale_;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  Marker marker = makeBox(int_marker);
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void makeMovingMarker(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "moving_frame";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale_;

  int_marker.name = "moving";
  int_marker.description = "Marker Attached to a\nMoving Frame";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back(makeBox(int_marker));
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void make6DofMarker(bool fixed, unsigned int interaction_mode,
                    const tf::Vector3& position, bool show_6dof) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale_;

  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if (fixed) {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != InteractiveMarkerControl::NONE) {
      std::string mode_text;
      if(interaction_mode ==
         InteractiveMarkerControl::MOVE_3D)         mode_text = "MOVE_3D";
      if(interaction_mode ==
         InteractiveMarkerControl::ROTATE_3D)       mode_text = "ROTATE_3D";
      if(interaction_mode ==
         InteractiveMarkerControl::MOVE_ROTATE_3D)  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") +
                               (show_6dof ? " + 6-DOF controls" : "") +
                               "\n" + mode_text;
  }

  if (show_6dof) {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != InteractiveMarkerControl::NONE)
    menu_handler.apply(*server, int_marker.name);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n("~");

  n.param<std::string>("frame_id", frame_id_, "base_frame");
  n.param<float>("scale", scale_, 1.0);
  n.param<int>("num_rotors", num_rotors_, 4);
  n.param<float>("arm_len", arm_len_, 0.22);
  n.param<float>("body_width", body_width_, 0.15);
  n.param<float>("body_height", body_height_, 0.10);
  n.param<float>("r_color", r_color_, 0.0);
  n.param<float>("g_color", g_color_, 1.0);
  n.param<float>("b_color", b_color_, 0.0);
  ROS_INFO_STREAM("Set frame_id to " << frame_id_ <<".");

  ros::NodeHandle n_pub;
  send_waypoint_ = n_pub.serviceClient
                   <waypoint_navigator::GoToWaypoint::Request>(
                     "go_to_waypoint");
  plan_waypoints_ = n_pub.serviceClient
                    <waypoint_navigator::GoToWaypoints::Request>(
                      "plan_to_waypoints");
  execute_waypoints_ = n_pub.serviceClient<std_srvs::Empty::Request>(
                         "execute_waypoints");
  abort_path_ = n_pub.serviceClient<std_srvs::Empty::Request>("abort_path");
  pose_pub_ = n_pub.advertise<geometry_msgs::PoseStamped>("command/pose", 10);

  server.reset(new interactive_markers::InteractiveMarkerServer(
                 "rviz_waypoint_navigator","",false));

  ros::Duration(0.1).sleep();

  menu_handler.insert("Send pose as step command.", &processFeedback);
  menu_handler.insert("Plan path to this pose.", &processFeedback);
  menu_handler.insert("Store pose as waypoint.", &processFeedback);
  menu_handler.insert("Plan pose through stored waypoints.", &processFeedback);
  menu_handler.insert("Abort path", &processFeedback);

  tf::Vector3 position;
  position = tf::Vector3(0, 0, 0);
  makeQuadrocopterMarker(position);

  server->applyChanges();

  ros::spin();

  server.reset();
}
