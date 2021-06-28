#include "control/auton.hpp"
#include "control/chassis.hpp"

// Class Init
Chassis Chassis;

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
        default: name = "ERROR INVALID AUTON"; break;
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

        default:{
            break;
        }

    }
}

// Match Autons
void homeRow(){
//   Chassis.drive(20).withGains(.3, 0.01, .15).withTol(100).waitUntilSettled();
  Chassis.turn(90).withGains(133, 0.01, 66).withTol(1).waitUntilSettled();
 }

void oneGoal() { 
  Chassis.turn(-90).withGains(133, 0.01, 66).withTol(1).waitUntilSettled();

}

void twoGoal(){ }

// Skills 
void skills();

// Testing 
void test(){
//   Chassis.drive(20).withGains(.3, 0.01, .15).withTol(100).waitUntilSettled();
//   Chassis.turn(90).withGains(133, 0.01, 66).withTol(1).waitUntilSettled();
}