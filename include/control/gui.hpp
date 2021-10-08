#pragma once
#include "main.h"
#include "globals.hpp"
#include "gif-pros/gifclass.hpp"


//TODO: Debug Screen
//TODO: Reset Screen

LV_IMG_DECLARE(destiny) // background image is a destiny 2 emblem 
LV_IMG_DECLARE(noAuton_IMG)
LV_IMG_DECLARE(homeRow_IMG)
LV_IMG_DECLARE(twoGoal_IMG)
LV_IMG_DECLARE(skills_IMG)
LV_IMG_DECLARE(AWP_IMG)

class Display{
    public:

    Display();

    void cleanUp();

    static void start(void* ignore);

    void run();

    void stop();

    private:

    static bool isRunning, isInitalized;

    static int currScr;

    lv_obj_t * tab1;
    lv_obj_t * tab2;
    lv_obj_t * tab3;
    
    void tabAuton(lv_obj_t * parent);
    void tabDebug(lv_obj_t * parent);
    void tabSettings(lv_obj_t * parent);

    // Macros From 7K - Tower Takeover
    lv_obj_t * createLabel(int x, int y, std::string text, lv_obj_t * parent);
    lv_obj_t * createButton(int id, int x, int y, int width, int height, std::string text, lv_obj_t * parent, lv_action_t action, lv_style_t * btn_pr = nullptr, lv_style_t * btn_rel = nullptr);
};