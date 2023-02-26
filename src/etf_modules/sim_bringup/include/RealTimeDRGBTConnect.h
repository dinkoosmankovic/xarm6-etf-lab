//
// Created by nermin on 25.02.23.
//
#ifndef RPMPL_REALTIMEDRGBTCONNECT_H
#define RPMPL_REALTIMEDRGBTCONNECT_H

#include <DRGBTConnect.h>

namespace planning
{
    namespace drbt
    {
        class RealTimeDRGBTConnect : public planning::drbt::DRGBTConnect
        {
        public:
            RealTimeDRGBTConnect() {}
            RealTimeDRGBTConnect(std::shared_ptr<base::StateSpace> ss_, std::shared_ptr<base::State> start_, std::shared_ptr<base::State> goal_);

            bool computingNextState();
            void replanning();
            bool checkingTerminatingCondition();
        };
    }
}

#endif RPMPL_REALTIMEDRGBTCONNECT_H