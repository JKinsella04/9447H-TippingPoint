#include "main.h"
#include "globals.hpp"

/*
@param IN Mobile goal in starting position.
@param OUT Mobile goal extended out.
*/
enum class MobileGoalState {
    IN, OUT
}; 

class MobileGoal {
    public:

        /*
        Runs the mobile goal state machine. 
        */
        MobileGoal& run();

        void reset();
        
    private:
};