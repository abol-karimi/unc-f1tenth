// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <set>
#include "VoronoiGraph.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

// TODO: replace with boost types (geometry, polygon, or uBLAS::vector)
struct PointFloat {
	float x;
	float y;
	PointFloat(float x0, float y0) : x(x0), y(y0) {}
};

struct SegmentFloat {
	PointFloat p0;
	PointFloat p1;
	SegmentFloat(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
};

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

	void Polylinize(std::vector<segment_type>& OutLineSegments, float DiscontinuityThreshold); // Convert raw distances to line segments
	bool GetSegment(SegmentFloat& OutSegment, float& OutStartAngle, float StepAngle, float DiscontinuityThreshold);
	bool GetPointAtAngle(PointFloat& OutPoint, float angle_deg); // Calculates the lidar point at angle_deg in Distances[1081] 
	bool GetDistanceAtAngle(float& OutDistance, float angle_deg); // Returns the corresponding distance in Distances[1081] 
	float Distance(PointFloat p0, PointFloat p1);
	float DistanceToLine(PointFloat point, PointFloat p0, PointFloat p1);

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
	VoronoiGraph Planner;
	std::vector<segment_type> Walls;

	// Lidar
	float AngularResolution = 0.25; // 4 measurements per angle
	float Range = 17; // Maximum detectable distance in meters
	float OutOfRange = 65.533; // Value to return if distance > LidarRange
	float LidarMinDegree = -135;
	float LidarMaxDegree = 135; // (LidarMaxDegree - LidarMinDegree)*AngularResolution + 1 = 1081
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