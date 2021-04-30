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
        sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &ManageROS::LaserCallback, this);
        ac_pub = n.advertise<ac_msgs::drive_params>("drive_parameters", 1);
    }

    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // Make a set of polylines out of lidar 2D point cloud.
        Perception perception;
        const std::vector<segment_type>& walls = perception.GetWalls(msg);

        VoronoiPlanner Planner;
        // Get the plan as list of line segments
        std::vector<point_type>& plan =	Planner.GetPlan(walls, allowed_obs_dist);

        double speed, steering;
        std::tie(speed, steering) = controller.GetSpeedAndSteering(plan); // Steeing in (-1.0, 1.0)
        Publish(speed, steering);
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
    Controller controller;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher ac_pub;
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