#include "rosmon_to_diagnostic.h"
#include <ros/ros.h>
#include <rosmon_msgs/State.h>

using namespace rosmon_diagnostics;

void onNewStateMessage(rosmon_msgs::StateConstPtr state, RosmonToDiagnostic& rosmonToDiag,
                       ros::Publisher& publisher)
{
    rosmonToDiag.onNewStateMessage(*state);
    publisher.publish(rosmonToDiag.getCurrentDiagnosticArray());
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rosmon_diagnostics");
    ros::NodeHandle nh;
    RosmonToDiagnostic rosmonToDiagnostic;

    auto pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1, true);

    auto sub = nh.subscribe<rosmon_msgs::State>(
        "rosmon_state", 1,
        boost::bind(onNewStateMessage, _1, boost::ref(rosmonToDiagnostic),
                    boost::ref(pub)));

    ros::spin();
    return 0;
}
