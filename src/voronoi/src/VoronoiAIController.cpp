// Fill out your copyright notice in the Description page of Project Settings.

#include "VoronoiAIController.h"
#include "voronoi_visual_utils.hpp"
#include <ostream>
#include <boost/math/constants/constants.hpp>

std::pair<double,double> AVoronoiAIController::GetSpeedAndSteering(std::vector<float> ranges)
{
	Distances = ranges;

	// Make a set of polylines out of lidar 2D point cloud.
	Polylinize(Walls, discontinuity_threshold);

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

	return std::pair<float,float>(0.45, steering_ratio);

	// Visualizations
	// DrawLaser();
	// DrawWalls();
	// DrawRoadmap();
	// DrawPlan(Plan);
	// // visualize the purepusuit goalpoint
	// DrawDebugSphere(GetWorld(), LidarToWorldLocation(RearAxleToLidar(PurePursuitGoal)),
	// 	9.f, 5.f, FColor(100, 10, 10), false, 0.f, 30.f, 2.2f);
	// // Draw circle corresponding to pure_pursuit lookahead distance (to rear axle)
	// DrawDebugCircle(GetWorld(),
	// 	LidarToWorldLocation(RearAxleToLidar(point_type(0.f, 0.f))),
	// 	PurepursuitLookahead*100.f, 72, FColor(0, 0, 0), false, 0.f, 30, 2.f, FVector(0, 1, 0), FVector(1, 0, 0));
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

/// Assumes that orientation of lidar's frame and the rear axle's frame are the same
point_type AVoronoiAIController::LidarToRearAxle(const point_type& point)
{
	return point_type(point.x() + 0.4, point.y()); // TODO: replace 0.4 with a member variable
}

/// Assumes that orientation of lidar's frame and the rear axle's frame are the same
point_type AVoronoiAIController::RearAxleToLidar(const point_type& point)
{
	return point_type(point.x() - 0.4, point.y()); // TODO: replace 0.4 with a member variable
}

void AVoronoiAIController::Polylinize(std::vector<segment_type>& OutLineSegments, float DiscontinuityThreshold)
{
	OutLineSegments.clear();

	SegmentFloat NewSegment(0, 0, 10, 10);
	float NewStartAngle = -135;
	float StepAngle = 2; // Unit is degrees.
	while (NewStartAngle < 135)
	{
		// Search for segments counterclockwise starting from NewStartAngle.
		// GetSegment() updates NewStartAngle for the next search.
		bool FoundNewSegment = GetSegment(NewSegment, NewStartAngle, StepAngle, DiscontinuityThreshold);
		if (FoundNewSegment)
		{
			// Convert SegmentFloat to segment_type, and meters to millimeters
			double x1, y1, x2, y2;
			x1 = NewSegment.p0.x;
			y1 = NewSegment.p0.y;
			x2 = NewSegment.p1.x;
			y2 = NewSegment.p1.y;
			OutLineSegments.push_back(segment_type(point_type(x1, y1), point_type(x2, y2))); // TODO what lp and hp? Any requiremtns on the order of points?
		}
	}
}

bool AVoronoiAIController::GetSegment(SegmentFloat& OutSegment, float& OutStartAngle, float StepAngle, float DiscontThreshold)
{
	PointFloat StartPoint(0, 0);
	float StartAngle = OutStartAngle;
	//UE_LOG(LogTemp, Warning, TEXT("Starting at angle: %f"), StartAngle);
	while (!GetPointAtAngle(StartPoint, StartAngle))
	{
		StartAngle += StepAngle;
		if (StartAngle > 135)
		{
			// OutStartAngle can be used for the next call to GetSegment. Here it is already out of bounds.
			OutStartAngle = StartAngle;
			return false; // No segment found
		}
	}
	PointFloat InterPoint(0, 0);
	float InterAngle = StartAngle + StepAngle; // Angle of the intermediate point of the segment (if any).
	if (!GetPointAtAngle(InterPoint, InterAngle)) // TODO: Report discontinuity
	{
		// No point at InterAngle, so skip it for the next call to GetSegment().
		OutStartAngle = InterAngle + StepAngle;
		//UE_LOG(LogTemp, Warning, TEXT("!GetPointAtAngle(InterPoint, InterAngle)"));
		return false; // No segment found
	}
	if (Distance(StartPoint, InterPoint) > DiscontThreshold) // TODO Report discontinuity
	{
		// Setup the next search from the beginning of the discontinuity.
		OutStartAngle = InterAngle;
		//UE_LOG(LogTemp, Warning, TEXT("Distance(StartPoint, InterPoint) > DiscontinuityThreshold"));
		return false; // No segment found
	}

	// If reached here, a segment exists starting at StartPoint and passing through SegmentInterPoint.
	// Now search how much the track extends along this segment.
	float EndAngle = InterAngle;
	PointFloat EndPoint = InterPoint;
	float CandidEndAngle = EndAngle;
	PointFloat CandidEndPoint = EndPoint;
	// While-loop invariant:
	//  If EndAngle and EndPoint are valid before the loop,
	//  then they are valid after the loop.
	while (CandidEndAngle + StepAngle <= 135)
	{
		CandidEndAngle += StepAngle;
		bool FoundNewPoint = GetPointAtAngle(CandidEndPoint, CandidEndAngle);
		if (!FoundNewPoint) // Point at CandidEndAngle is OutOfRange TODO: Report discontinuity.
		{
			// Return with the current EndPoint and EndAngle
			OutSegment.p0 = StartPoint;
			OutSegment.p1 = EndPoint;
			OutStartAngle = CandidEndAngle + StepAngle; // Skip the OutOfRange angle for next call to GetSegment()
			//UE_LOG(LogTemp, Warning, TEXT("!FoundNewPoint"));
			return true; // Found a segment
		}
		else if (Distance(EndPoint, CandidEndPoint) > DiscontThreshold) // TODO Report discontinuity
		{
			// Finalize the segment with the current EndAngle and EndPoint
			OutSegment.p0 = StartPoint;
			OutSegment.p1 = EndPoint;
			OutStartAngle = CandidEndAngle; // Skip the discontinuity
			//UE_LOG(LogTemp, Warning, TEXT("Distance(EndPoint, CandidEndPoint) > DiscontinuityThreshold"));
			return true; // Found a segment
		}
		else if (DistanceToLine(CandidEndPoint, StartPoint, InterPoint) > 0.1) // TODO: Expose 0.1 as a class property
		{
			// Finalize the segment with the current EndAngle and EndPoint
			OutSegment.p0 = StartPoint;
			OutSegment.p1 = EndPoint;
			OutStartAngle = EndAngle;
			//UE_LOG(LogTemp, Warning, TEXT("DistanceToLine(CandidEndPoint, StartPoint, InterPoint) > 0.4"));
			return true; // Found a segment
		}
		else // CandidEndPoint is close enough to the interpolated line (imposing curvature threshold)
		{
			// Extend the segment to CandidEndPoint
			EndAngle = CandidEndAngle;
			EndPoint = CandidEndPoint;
		}
	}
	OutSegment.p0 = StartPoint;
	OutSegment.p1 = EndPoint;
	OutStartAngle = CandidEndAngle + StepAngle;
	return true;
}

bool AVoronoiAIController::GetPointAtAngle(PointFloat& OutPoint, float angle_deg)
{
	float angle_rad = angle_deg * boost::math::constants::pi<double>() / 180.0;
	float Distance;
	if (GetDistanceAtAngle(Distance, angle_deg))
	{
		OutPoint.x = Distance * cos(angle_rad);
		OutPoint.y = Distance * sin(angle_rad);
		return true;
	}
	else { return false; } // Out of range distance
}

bool AVoronoiAIController::GetDistanceAtAngle(float& OutDistance, float angle_deg)
{
	// Linearly interpolate between (-135, 0) and (135, 1080) on the angle-index coordinates
	// Slope is (1080-0)/(135-(-135)) = 1080/270
	// Index-intercept is 1080/2 = 540
	// Interpolation function is slope*angle_deg + intercept
	if (angle_deg < LidarMinDegree || angle_deg > LidarMaxDegree)
	{
		return false;
	}
	int index = static_cast<int>(1080 * angle_deg / 270 + 540);
	float Distance = Distances[index];
	if (Distance < OutOfRange)
	{
		OutDistance = Distance;
		return true;
	}
	else { return false; }
}

float AVoronoiAIController::Distance(PointFloat p0, PointFloat p1)
{
	float DeltaX = p1.x - p0.x;
	float DeltaY = p1.y - p0.y;
	return sqrt(DeltaX * DeltaX + DeltaY * DeltaY);
}

// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points
// Calculate distance of p0 to the line passing through p0 and p1
float AVoronoiAIController::DistanceToLine(PointFloat point, PointFloat p0, PointFloat p1)
{
	float delta_y = p1.y - p0.y;
	float delta_x = p1.x - p0.x;
	float denominator = Distance(p0, p1);
	float numerator_const_term = p1.x * p0.y - p1.y * p0.x;
	float numerator = abs(delta_y * point.x - delta_x * point.y + numerator_const_term);
	return (numerator / denominator);
}
















// void AVoronoiAIController::DrawLaser()
// {
// 	std::vector<point_type> points;
// 	Lidar->GetLidarData(points);
// 	for (const auto& point : points)
// 	{
// 		double distance = euclidean_distance(point, point_type(0.f, 0.f))*5.f;
// 		point_type End = point;
// 		point_type Start = point_type(point.x()/distance, point.y()/distance);
// 		DrawDebugLine(GetWorld(), 
// 			LidarToWorldLocation(Start), 
// 			LidarToWorldLocation(End), 
// 			FColor(255, 0, 0), false, 0.f, 0.f, 0.f);
// 	}	
// }

// void AVoronoiAIController::DrawWalls()
// {
// 	for (auto& wall : Walls)
// 	{
// 		DrawDebugLine(
// 			GetWorld(),
// 			LidarToWorldLocation(wall.low()),
// 			LidarToWorldLocation(wall.high()), FColor(0, 255, 0), false, 0.f, 1.f, 10.f);
// 	}
// }

// void AVoronoiAIController::DrawRoadmap()
// {
// 	std::list<point_type> points;
// 	Planner.GetRoadmapPoints(points);
// 	for (const point_type& point : points)
// 	{
// 		DrawDebugSphere(GetWorld(), LidarToWorldLocation(point),
// 			15.f, 5.f, FColor(0, 0, 0), false, 0.f, 10.f, 1.f);
// 	}
// 	std::vector<segment_type> segments;
// 	Planner.GetRoadmapSegments(segments);
// 	for (const segment_type& segment : segments)
// 	{
// 		DrawDebugLine(GetWorld(), LidarToWorldLocation(segment.low()), LidarToWorldLocation(segment.high()),
// 			FColor(64, 64, 255), false, 0.f, 5.f, 10.f);
// 	}
// }

// void AVoronoiAIController::DrawPlan(std::vector<point_type>& Plan)
// {
// 	if (Plan.size() > 1)
// 	{
// 		for (auto si = Plan.begin(); si != Plan.end() - 1; ++si)
// 		{
// 			DrawDebugLine(GetWorld(),
// 				LidarToWorldLocation(*si), LidarToWorldLocation(*(si + 1)), 
// 				FColor(255, 255, 255), false, 0.f, 20.f, 7.f);
// 		}
// 	}
// }
