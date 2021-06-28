#pragma once
#include "main.h"
#include "globals.hpp"


// struct setAuton{
//     std::string name;
//     int id;
// };

class Autonomous{
    public:

        Autonomous();

        /*
        Returns current selected auton.
        */
        std::string getAuton();

        /*
        Sets Id to selected auton.
        */
        void setId(int id_);
        
        /*
        Runs current auton
        */
        void runAuton();

    private:
        static std::string name;
        static int id;
};

/*===========================================
  AUTON DECLARATIONS
===========================================*/

void test();

void homeRow();
void oneGoal();
void twoGoal();
