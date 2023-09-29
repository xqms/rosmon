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

		captures = 0;

		parser.processString("\e[33mThis is a warning message with an end\e[0m and another start \e[33m  and another end!\e[0m\n");
		CHECK(captures == 1);
		CHECK(lastEvent.severity == rosmon::LogEvent::Type::Raw);
		CHECK(lastEvent.message == "\e[33mThis is a warning message with an end\e[0m and another start \e[33m  and another end!\e[0m");
	}

	SECTION("timeout")
	{
		captures = 0;

		auto t0 = std::chrono::steady_clock::time_point();
		auto t1 = t0 + std::chrono::milliseconds(500);

		parser.processString("\e[33mThis is a warning message without its end!\n", t0);
		CHECK(captures == 0);

		parser.checkPending(t0);
		CHECK(captures == 0);

		parser.checkPending(t1);
		CHECK(captures == 1);
		CHECK(lastEvent.severity == rosmon::LogEvent::Type::Raw);
		CHECK(lastEvent.message == "\e[33mThis is a warning message without its end!");
	}

	SECTION("flush")
	{
		captures = 0;

		auto t0 = std::chrono::steady_clock::time_point();

		parser.processString("\e[33mThis is a warning message without its end!\n", t0);

		CHECK(captures == 0);

		parser.checkPending(t0);
		CHECK(captures == 0);

		parser.flush();
		CHECK(captures == 1);
		CHECK(lastEvent.severity == rosmon::LogEvent::Type::Raw);
		CHECK(lastEvent.message == "\e[33mThis is a warning message without its end!");
	}
}
