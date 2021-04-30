// Fill out your copyright notice in the Description page of Project Settings.

#include "VoronoiAIController.h"
#include "voronoi_visual_utils.hpp"
#include <ostream>
#include <boost/math/constants/constants.hpp>

AVoronoiAIController::AVoronoiAIController()
{
	marker_pub = ros_node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

std::pair<double,double> AVoronoiAIController::GetSpeedAndSteering(const std::vector<segment_type>& Walls)
{
	// Get the plan as list of line segments and draw it
	Planner.MakeRoadmap(Walls, allowed_obs_dist);

	std::vector<point_type> Plan;
	Planner.GetPlan(Plan, Walls);

	point_type rear_axle = RearAxleToLidar(point_type(0.f, 0.f)); // Coordinates of rear axle in lidar's frame
	point_type PurePursuitGoal; // Coordinates of the goal point in rear axle's frame
	if (Plan.size() == 0)
	{
		std::cout << "No plan found!";
	}
	else if (euclidean_distance(Plan[0], rear_axle) >= PurepursuitLookahead)
	{
		std::cout << "Plan does not start inside of lookahead circle!";
		PurePursuitGoal = LidarToRearAxle(Plan[0]);
	}
	else if (euclidean_distance(Plan[Plan.size() - 1], rear_axle) <= PurepursuitLookahead)
	{
		std::cout << "Plan does not end outside of lookahead circle!";
		PurePursuitGoal = LidarToRearAxle(Plan[Plan.size() - 1]);
	}
	else
	{
		PurePursuitGoal = get_plan_at_lookahead(Plan);
	}
	float steering_ratio = pure_pursuit(PurePursuitGoal);
	float speed = get_speed(Plan);

	// Visualizations
	// DrawLaser();
	DrawWalls();
	DrawRoadmap();
	DrawPlan(Plan);
	DrawPurepursuit(PurePursuitGoal);

	return std::pair<float,float>(speed, steering_ratio);
}


/// Assumes that Plan starts inside the lookahead circle and ends outside.
/// The returned value is in rear_axle's coordinates.
point_type AVoronoiAIController::get_plan_at_lookahead(const std::vector<point_type>& Plan)
{
	point_type rear_axle = RearAxleToLidar(point_type(0.f, 0.f)); // Plan is in lidar's coordinates
	size_t end_index = 1;
	while (euclidean_distance(Plan[end_index], rear_axle) <= PurepursuitLookahead)
		++end_index;
	point_type p_in = LidarToRearAxle(Plan[end_index - 1]);
	point_type p_out = LidarToRearAxle(Plan[end_index]);
	double x1 = p_in.x(); 
	double y1 = p_in.y();
	double x2 = p_out.x();
	double y2 = p_out.y();
	double dx = x2 - x1;
	double dy = y2 - y1;
	double A = dx * dx + dy * dy;
	double B = x1 * dx + y1 * dy;
	double C = x1 * x1 + y1 * y1 - PurepursuitLookahead * PurepursuitLookahead;
	double t = (-B + sqrt(B*B - A*C)) / A;
	return point_type(x1 + t * dx, y1 + t * dy);
}

/// Returns a steering "percentage" value between 0.0 (left) and 1.0
/// 	(right) that is as close as possible to the requested degrees. The car's
/// 	wheels can't turn more than max_turn_degrees in either direction.
float AVoronoiAIController::pure_pursuit(point_type goal_point)
{
	// goal_point is in rear axle's coordinates.
	// goal_point must have a positive x coordinate.

	double d = euclidean_distance(goal_point, point_type(0.f, 0.f));
	double d2 = d*d;
	double steering_angle_rad = atan(2 * wheelbase * goal_point.y() / d2);
	double steering_angle_deg = steering_angle_rad * 180.f / boost::math::constants::pi<double>();

		// The wheels cannot physically turn more than 34 degrees
	if (steering_angle_deg < -max_turn_degrees)
		return 0.f;
	else if (steering_angle_deg > max_turn_degrees)
		return 1.f;
	else
		return 1.0 - ((steering_angle_deg + max_turn_degrees) / (2 * max_turn_degrees));
}

float AVoronoiAIController::get_speed(const std::vector<point_type>& Plan)
{
	return 0.2;
}

/// Assumes that orientation of lidar's frame and the rear axle's frame are the same
point_type AVoronoiAIController::LidarToRearAxle(const point_type& point)
{
	return point_type(point.x() + lidar_to_rearAxle, point.y());
}

/// Assumes that orientation of lidar's frame and the rear axle's frame are the same
point_type AVoronoiAIController::RearAxleToLidar(const point_type& point)
{
	return point_type(point.x() - lidar_to_rearAxle, point.y());
}

void AVoronoiAIController::DrawWalls()
{
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/laser";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	// Line width
	line_list.scale.x = 0.1;
    // Walls are green
    line_list.color.g = 1.0;
   	line_list.color.a = 1.0;

	for (auto& wall : Walls)
	{
		geometry_msgs::Point p0, p1;
		p0.x = wall.low().x();
		p0.y = wall.low().y();
		p0.z = 0.f;
		p1.x = wall.high().x();
		p1.y = wall.high().y();
		p1.z = 0.f;
		line_list.points.push_back(p0);
		line_list.points.push_back(p1);
	}
	marker_pub.publish(line_list);
}

void AVoronoiAIController::DrawRoadmap()
{
	std::vector<segment_type> segments;
	Planner.GetRoadmapSegments(segments);

	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/laser";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 1;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	// Line width
	line_list.scale.x = 0.3;
    // Roadplan is black
   	line_list.color.a = 1.0;

	for (const segment_type& segment : segments)
	{
		geometry_msgs::Point p0, p1;
		p0.x = segment.low().x();
		p0.y = segment.low().y();
		p0.z = 0.f;
		p1.x = segment.high().x();
		p1.y = segment.high().y();
		p1.z = 0.f;
		line_list.points.push_back(p0);
		line_list.points.push_back(p1);
	}
	marker_pub.publish(line_list);
}

void AVoronoiAIController::DrawPlan(const std::vector<point_type>& Plan)
{
	if (Plan.size() == 0)
		return;
	
	visualization_msgs::Marker line_strip;
	line_strip.header.frame_id = "/laser";
	line_strip.header.stamp = ros::Time::now();
	line_strip.ns = "points_and_lines";
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	line_strip.id = 2;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	// Line width
	line_strip.scale.x = 0.1;
    // Plan is white
	line_strip.color.r = 1.0;
	line_strip.color.g = 1.0;
    line_strip.color.b = 1.0;
   	line_strip.color.a = 1.0;

	for (auto& point : Plan)
	{
		geometry_msgs::Point p;
		p.x = point.x();
		p.y = point.y();
		p.z = 0.f;
		line_strip.points.push_back(p);
	}
	marker_pub.publish(line_strip);
}

void AVoronoiAIController::DrawPurepursuit(const point_type& goal)
{
	visualization_msgs::Marker points;
	points.header.frame_id = "/laser";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 3;
	points.type = visualization_msgs::Marker::POINTS;
	// point width/height
	points.scale.x = 0.3;
	points.scale.y = 0.3;
    // Goalpoint is red
	points.color.r = 1.0;
   	points.color.a = 1.0;

	point_type goal_lidar = RearAxleToLidar(goal);
	geometry_msgs::Point p;
	p.x = goal_lidar.x();
	p.y = goal_lidar.y();
	p.z = 0.f;
	points.points.push_back(p);

	marker_pub.publish(points);

	// Draw the lookahead circle around the rear axle
	visualization_msgs::Marker marker;
    marker.header.frame_id = "/laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -lidar_to_rearAxle; // TODO: change the constant to a member variable
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = PurepursuitLookahead*2.f;
    marker.scale.y = PurepursuitLookahead*2.f;
    marker.scale.z = 0.01;
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 0.3;

	marker_pub.publish(marker);

	// TODO change chape to arrow and publish again

}