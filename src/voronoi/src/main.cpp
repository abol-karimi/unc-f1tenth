#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
// #include <std_msgs/Float64.h>
#include "ac_msgs/drive_params.h"

#include "Controller.h"
#include "Perception.h"
#include <ostream>
#include <signal.h>
#include <functional>

class ManageROS
{
public:
    float allowed_obs_dist = 0.3f; // in meters

    ManageROS()
    {
        sub = ros_node.subscribe<sensor_msgs::LaserScan>("scan", 1, &ManageROS::LaserCallback, this);
        ac_pub = ros_node.advertise<ac_msgs::drive_params>("drive_parameters", 1);
        marker_pub = ros_node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        Perception perception;
        const std::vector<segment_type>& walls = perception.GetWalls(msg);

        VoronoiPlanner planner;
        const std::vector<point_type>& plan = planner.GetPlan(walls, allowed_obs_dist);

        Controller controller;
        double speed, steering;
        std::tie(speed, steering) = controller.GetSpeedAndSteering(plan); // Steeing in (-1.0, 1.0)

        // Send the control commands
        Publish(speed, steering);

        // Visualize the walls and the plan
        DrawWalls(walls);
        DrawPlan(plan);
    }

    void Publish(double speed, double steering) const
    {
        ac_msgs::drive_params msg;
        msg.velocity = speed;
        msg.angle = steering;
        ac_pub.publish(msg);
        std::cout << "Published: speed: " << speed << "\t Steering: " << steering << std::endl;
    }
private:
	// Visualizations (in laser frame)
    void DrawWalls(const std::vector<segment_type>& walls)
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

        for (auto& wall : walls)
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

    void DrawPlan(const std::vector<point_type>& Plan)
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

    ros::NodeHandle ros_node;
    ros::Subscriber sub;
    ros::Publisher ac_pub;
	ros::Publisher marker_pub;
};

namespace {
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voronoi_controller");
    ManageROS manageROS;

    // Capture Ctr+C to stop the car.
    signal(SIGINT, signal_handler);
    shutdown_handler = [manageROS](int sig)
    {
        manageROS.Publish(0.f, 0.5f);
        usleep(1000);
        ros::shutdown();
    };

    ros::spin();
    manageROS.Publish(0.f, 0.f);

	return EXIT_SUCCESS;
}