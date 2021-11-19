#pragma once
#include "main.h"
#include "globals.hpp"

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

void awp();
void elim();
void skills();
void test();