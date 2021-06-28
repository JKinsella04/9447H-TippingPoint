#include "control/auton.hpp"

// struct setAuton runningAuton;
std::string Autonomous::name;

int Autonomous::id;

Autonomous::Autonomous(){ }

std::string Autonomous::getAuton(){
    return name;
}

void Autonomous::setId(int id_){
    id = id_;
    switch (id){
        case 1: name = "Home Row"; break;
        case 2: name = "One Goal"; break;
        case 3: name = "Two Goal"; break;
        default: name = "ERROR NO AUTON FOUND"; break;
    }
}

void Autonomous::runAuton(){
    switch(id){
        case 1:{
            homeRow();
            break;
        }
        case 2:{
            oneGoal();
            break;
        }
        case 3:{
            twoGoal();
            break;
        }

    }
}

void homeRow(){ }

void oneGoal() { }

void twoGoal(){ }

void test(){ }