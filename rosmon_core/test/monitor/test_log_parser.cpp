// Test log parser
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <catch_ros/catch.hpp>

#include "../../src/monitor/log_parser.h"

using namespace rosmon::monitor;

TEST_CASE("LogParser", "[log_parser]")
{
	LogParser parser;

	int captures = 0;
	LogParser::Event lastEvent;

	auto cb = [&](LogParser::Event&& event){
		captures++;
		lastEvent = std::move(event);
	};
	parser.setCallback(cb);

	SECTION("simple")
	{
		captures = 0;

		parser.processString("\e[0mThis is an info message\e[0m\n");
		CHECK(captures == 1);
		CHECK(lastEvent.severity == rosmon::LogEvent::Type::Info);
		CHECK(lastEvent.message == "This is an info message");

		captures = 0;

		parser.processString("\e[31mThis is an error message\e[0m\n");
		CHECK(captures == 1);
		CHECK(lastEvent.severity == rosmon::LogEvent::Type::Error);
		CHECK(lastEvent.message == "This is an error message");

		captures = 0;

		parser.processString("\e[32mThis is a debug message\e[0m\n");
		CHECK(captures == 1);
		CHECK(lastEvent.severity == rosmon::LogEvent::Type::Debug);
		CHECK(lastEvent.message == "This is a debug message");

		captures = 0;

		parser.processString("\e[33mThis is a warning message\e[0m\n");
		CHECK(captures == 1);
		CHECK(lastEvent.severity == rosmon::LogEvent::Type::Warning);
		CHECK(lastEvent.message == "This is a warning message");
	}

	SECTION("raw")
	{
		captures = 0;

		parser.processString("This is a raw \e[31mred\e[0m message\n");
		CHECK(captures == 1);
		CHECK(lastEvent.severity == rosmon::LogEvent::Type::Raw);
		CHECK(lastEvent.message == "This is a raw \e[31mred\e[0m message");
	}
}
