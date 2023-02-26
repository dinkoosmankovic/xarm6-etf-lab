//
// Created by nermin on 25.02.23.
//

#include "RealTimeDRGBTConnect.h"
#include <glog/logging.h>

planning::drbt::RealTimeDRGBTConnect::RealTimeDRGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_,
                                           std::shared_ptr<base::State> goal_) : DRGBTConnect(ss_, start_, goal_) {}

bool planning::drbt::RealTimeDRGBTConnect::computingNextState()
{
    // Generate a horizon
    generateHorizon();
    // LOG(INFO) << "Initial horizon consists of " << horizon.size() << " states: ";
    // for (int i = 0; i < horizon.size(); i++)
    //     LOG(INFO) << i << ". state:\n" << horizon[i]; 

    // Moving from 'q_current' towards 'q_next' for step 'DRGBTConnectConfig::STEP', where 'q_next' may change
    do
    {
        // LOG(INFO) << "\n\nIteration num. " << planner_info->getNumIterations()
        //           << " -----------------------------------------------------------------------------------------";

        // Update obstacles and check if the collision occurs
        // LOG(INFO) << "Updating obstacles...";
        ss->env->updateObstacles();
        if (!ss->isValid(q_current))
        {
            LOG(INFO) << "Collision has been occured!!! ";
            planner_info->setSuccessState(false);
            planner_info->setPlanningTime(planner_info->getIterationTimes().back());
            return false;
        }
        
        // Compute the horizon and the next state
        computeHorizon();
        computeNextState();
        // LOG(INFO) << "Horizon consists of " << horizon.size() << " states: ";
        // for (int i = 0; i < horizon.size(); i++)
        //     LOG(INFO) << i << ". state:\n" << horizon[i];

        // Update the robot current state
        updateCurrentState();
        // LOG(INFO) << "Status: " << (status == base::State::Status::Advanced ? "Advanced" : "")
        //                         << (status == base::State::Status::Trapped  ? "Trapped"  : "")
        //                         << (status == base::State::Status::Reached  ? "Reached"  : "");
        
    } while (status == base::State::Status::Advanced);

    // LOG(INFO) << "Next state is " << (status == base::State::Status::Reached ? "reached!" : "not reached!") 
    //     << "\n***************************************************************************";
}

void planning::drbt::RealTimeDRGBTConnect::replanning()
{
    // // Replanning procedure assessment
    // if (replanning || whetherToReplan()) // Da li ce biti ovdje ili negdje drugo??
    // {
    //     try
    //     {
    //         // LOG(INFO) << "Trying to replan...";
    //         time_current = std::chrono::steady_clock::now();
    //         RGBTConnectConfig::MAX_PLANNING_TIME = DRGBTConnectConfig::MAX_ITER_TIME - getElapsedTime(time_iter_start, time_current) - 1; // 1 [ms] is reserved for the following code lines
    //         RGBTConnect = std::make_unique<planning::rbt::RGBTConnect>(ss, q_current, goal);
    //         if (RGBTConnect->solve())   // New path is found, thus update predefined path to the goal
    //         {
    //             // LOG(INFO) << "The path has been replanned in " << RGBTConnect->getPlannerInfo()->getPlanningTime() << " [ms]. ";
    //             // LOG(INFO) << "Predefined path is: ";
    //             predefined_path = RGBTConnect->getPath();
    //             // for (int i = 0; i < predefined_path.size(); i++)
    //             //     std::cout << predefined_path.at(i) << std::endl;
    //             replanning = false;
    //             status = base::State::Status::Reached;
    //             q_next = std::make_shared<HorizonState>(predefined_path.front(), 0);
    //             horizon.clear();
    //             planner_info->addRoutineTime(RGBTConnect->getPlannerInfo()->getPlanningTime(), 4);
    //         }
    //         else    // New path is not found
    //             throw std::runtime_error("New path is not found! ");
    //     }
    //     catch (std::exception &e)
    //     {
    //         // LOG(INFO) << "Replanning is required. " << e.what();
    //         replanning = true;
    //     }
    // }
    // // else
    // //     LOG(INFO) << "Replanning is not required! ";
}

bool planning::drbt::RealTimeDRGBTConnect::checkingTerminatingCondition()
{
    // // Planner info and terminating condition
    // planner_info->setNumIterations(planner_info->getNumIterations() + 1);
    // planner_info->addIterationTime(getElapsedTime(time_alg_start, time_current));
    // if (checkTerminatingCondition())
    // {
    //     planner_info->setPlanningTime(planner_info->getIterationTimes().back());
    //     return planner_info->getSuccessState();
    // }      
}