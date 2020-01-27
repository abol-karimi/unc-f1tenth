#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
// #include <std_msgs/Float64.h>
#include "ac_msgs/drive_params.h"

#include "VoronoiAIController.h"
#include <ostream>
#include <signal.h>
#include <functional>

class ManageROS
{
public:
    ManageROS()
    {
        sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &ManageROS::LaserCallback, this);
        ac_pub = n.advertise<ac_msgs::drive_params>("drive_parameters", 1);
    }

    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        double speed, steering;
        std::tie(speed, steering) = controller.GetSpeedAndSteering(msg->ranges); // Steeing in (-1.0, 1.0)
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
    AVoronoiAIController controller;
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