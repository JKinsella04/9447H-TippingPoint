#include "control/mobileGoal.hpp"

MobileGoalState mobileGoal_mode = MobileGoalState::IN;

MobileGoal& MobileGoal::run(){

    switch (mobileGoal_mode){
        case MobileGoalState::IN:
            break;

        case MobileGoalState::OUT:
            break;

    }
    return *this;
}

