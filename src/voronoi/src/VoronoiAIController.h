// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <set>
#include "VoronoiPlanner.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

/**
 * 
 */
class AVoronoiAIController
{
public:
	AVoronoiAIController();
	
	std::pair<double,double> GetSpeedAndSteering(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
	point_type LidarToRearAxle(const point_type& point);
	point_type RearAxleToLidar(const point_type& point);
	float pure_pursuit(point_type goal_point);
	point_type get_plan_at_lookahead(const std::vector<point_type>& Plan);
	float get_speed(const std::vector<point_type>& Plan);

	// Controller properties
	float MinTrackWidth = 1.5; // in meters
	float PurepursuitLookahead = 1; // Distance (in meters) between the rear axel and the goal point
	float allowed_obs_dist = 0.3f; // in meters
	float discontinuity_threshold = 0.5f;

	// Vehicle properties
	float wheelbase = 0.3; // Distance (in meters) of rear axle to front axel
	float max_turn_degrees = 34;
	float lidar_to_rearAxle = 0.38;

	// VoronoiGraph properties
	VoronoiPlanner Planner;
	std::vector<segment_type> Walls;

	// Lidar
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
	float LidarMinDegree = -135;
	float LidarMaxDegree = 135;
	std::vector<float> Distances;

	// Visualizations (in laser frame)
	// void DrawLaser();
	void DrawWalls();
	void DrawRoadmap();
	void DrawPlan(const std::vector<point_type>& Plan);
	void DrawPurepursuit(const point_type& goal);

	ros::NodeHandle ros_node;
	ros::Publisher marker_pub;
};