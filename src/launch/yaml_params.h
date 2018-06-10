// Methods for converting YAML into XmlRpcValues for the parameter server
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#ifndef ROSMON_LAUNCH_YAML_PARAMS_H
#define ROSMON_LAUNCH_YAML_PARAMS_H

#include <XmlRpc.h>
#include <yaml-cpp/yaml.h>

namespace rosmon
{
namespace launch
{

class ParseContext;

XmlRpc::XmlRpcValue yamlToXmlRpc(const ParseContext& ctx, const YAML::Node& n);

}
}

#endif
