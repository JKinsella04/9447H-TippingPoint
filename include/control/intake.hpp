#pragma once
#include "main.h"
#include "globals.hpp"

enum class IntakeState {
    IN, OUT, OFF
};

class Intake {
    public:

    /*
    Runs the intake state machine.
    */
    Intake& run();

    private:
};