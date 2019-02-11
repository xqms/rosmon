#include "resource_checker.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rosmon_diagnostics");

    ros::spin();
    return 0;
}
