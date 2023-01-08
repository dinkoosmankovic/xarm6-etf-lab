#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include <AbstractPlanner.h>
#include <RBTConnect.h>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <CommandLine.h>
#include <glog/logging.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

	std::string root_path = "/home/nermin/RPMPLv2";
	// std::string scenario_file_path = root_path + "/data/xarm6/scenario_test/scenario_test.yaml";
	std::string scenario_file_path = root_path + "/data/xarm6/scenario1/scenario1.yaml";
	// std::string scenario_file_path = root_path + "/data/xarm6/scenario2/scenario2.yaml";

	bool print_help = false;
	CommandLine args("Test RBTConnect command line parser.");
	args.addArgument({"-s", "--scenario"}, &scenario_file_path, "Scenario .yaml description file path");
	args.addArgument({"-h", "--help"},     &print_help,
      "Use --scenario scenario_yaml_file_path to "
      "run with different scenario");

	try
	{
		args.parse(argc, argv);
	}
	catch (std::runtime_error const &e)
	{
		LOG(INFO) << e.what() << std::endl;
		return -1;
	}

	// When oPrintHelp was set to true, we print a help message and exit.
	if (print_help)
	{
		args.printHelp();
		return 0;
	}

	ConfigurationReader::initConfiguration(root_path);
	scenario::Scenario scenario(scenario_file_path, root_path);
	std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();

	LOG(INFO) << "Using scenario: " << scenario_file_path;
	LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
	LOG(INFO) << "Dimensions: " << ss->getDimensions();
	LOG(INFO) << "State space type: " << ss->getStateSpaceType();
	LOG(INFO) << "Start: " << scenario.getStart();
	LOG(INFO) << "Goal: " << scenario.getGoal();

	try
	{
		std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RBTConnect>(ss, scenario.getStart(), scenario.getGoal());
		bool res = planner->solve();
		LOG(INFO) << "RBTConnect planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
		LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
		LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
			
		if (res)
		{
			std::vector<std::shared_ptr<base::State>> path = planner->getPath();
			for (int i = 0; i < path.size(); i++)
				LOG(INFO) << path.at(i)->getCoord().transpose() << std::endl;
		}
		planner->outputPlannerData(scenario_file_path.substr(0, scenario_file_path.size()-5) + "_planner_data.log");

	}
	catch (std::domain_error &e)
	{
		LOG(ERROR) << e.what();
	}
	google::ShutDownCommandLineFlags();
	return 0;
}
