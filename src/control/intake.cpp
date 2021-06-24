#include "control/intake.hpp"

IntakeState intake_mode = IntakeState::OFF;

Intake& Intake::run(){

    switch (intake_mode){
        case IntakeState::IN:
            break;
        case IntakeState::OUT:
            break;
        case IntakeState::OFF:
            break;
    }

    return *this;
}