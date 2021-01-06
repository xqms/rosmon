// Small utility that prints a ROS parameter type and value
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <ros/ros.h>

#include <rosfmt/rosfmt.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dump_param", ros::init_options::AnonymousName);

	if(argc < 2 || std::string(argv[1]) == "--help")
	{
		fmt::print(stderr, "Usage: dump_param <name>\n");
		return 1;
	}

	ros::NodeHandle nh;

	using Value = XmlRpc::XmlRpcValue;
	Value value;

	if(!nh.getParam(argv[1], value))
	{
		fmt::print(stderr, "Could not retrieve parameter '{}', does it exist?", argv[1]);
		return 1;
	}

	switch(value.getType())
	{
		case Value::TypeString:
			fmt::print("Type: String\n");
			fmt::print("Value: '{}'\n", static_cast<std::string>(value));
			break;

		case Value::TypeBoolean:
			fmt::print("Type: Bool\n");
			fmt::print("Value: {}\n", static_cast<bool>(value));
			break;

		case Value::TypeDouble:
		{
			fmt::print("Type: Double\n");

			double v = value;
			fmt::print("Value: {}\n", v);

			uint64_t vhex = 0;
			std::memcpy(&vhex, &v, sizeof(vhex));

			fmt::print("Hex: 0x{:08X}\n", vhex);
			break;
		}

		case Value::TypeInt:
			fmt::print("Type: Int\n");
			fmt::print("Value: {}\n", static_cast<int>(value));
			break;

		case Value::TypeArray:
			fmt::print("Type: Array\n");
			break;

		case Value::TypeBase64:
			fmt::print("Type: Base64\n");
			break;

		case Value::TypeDateTime:
			fmt::print("Type: DateTime\n");
			break;

		case Value::TypeInvalid:
			fmt::print("Type: Invalid\n");
			break;

		case Value::TypeStruct:
			fmt::print("Type: Struct\n");
			break;

		default:
			fmt::print("Type: Unknown\n");
			break;
	}
}
