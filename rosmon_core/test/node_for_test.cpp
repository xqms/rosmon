#include <ros/ros.h>
#include <std_srvs/Empty.h>

std::vector<uint8_t> bigVector;

bool crashThisNode(std_srvs::EmptyRequest& , std_srvs::EmptyResponse& )
{
    int *a = nullptr;
    *a = 2;

    return true;
}

bool increaseMemoryBy1MB(std_srvs::EmptyRequest& , std_srvs::EmptyResponse& )
{
    bigVector.resize(bigVector.size() + 1e6);
    return true;
}

bool useCpuDuring10s(std_srvs::EmptyRequest& , std_srvs::EmptyResponse& )
{
    ros::Time start = ros::Time::now();
    int i = 0;
    while(ros::Time::now() - start < ros::Duration(10.)){
        ++i;
    }
    return true;
}

bool useCpuDuring1s(std_srvs::EmptyRequest& , std_srvs::EmptyResponse& )
{
    ros::Time start = ros::Time::now();
    int i = 0;
    while(ros::Time::now() - start < ros::Duration(1.)){
        ++i;
    }
    return true;
}

bool useCpuDuring100ms(std_srvs::EmptyRequest& , std_srvs::EmptyResponse& )
{
    ros::Time start = ros::Time::now();
    int i = 0;
    while(ros::Time::now() - start < ros::Duration(.1)){
        ++i;
    }
    return true;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "node_tester");
    ros::NodeHandle nh("~");
    auto serv1 = nh.advertiseService("use_cpu", &useCpuDuring10s);
    auto serv2 = nh.advertiseService("use_cpu_1s", &useCpuDuring1s);
    auto serv3 = nh.advertiseService("use_cpu_100ms", &useCpuDuring100ms);
    auto serv4 = nh.advertiseService("increase_memory", &increaseMemoryBy1MB);
    auto serv5 = nh.advertiseService("crash_this_node", &crashThisNode);
    ros::spin();
    return 0;
}
