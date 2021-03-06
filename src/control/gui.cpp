#include "gui.hpp"
#include "auton.hpp"
#include "chassis.hpp"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_label.h"
#include "frontLift.hpp"
#include "backLift.hpp"
#include "positionTracking.hpp"
#include "misc.hpp"
#include "units.hpp"

static Autonomous auton;
static Chassis chassis;
static FrontLift frontLift;
static BackLift backLift;
PositionTracker *Display::robot;

bool Display::isRunning = false, Display::isInitalized = false;

static lv_style_t style_btn;
static lv_style_t style_btn_released;

static lv_style_t switch_red;
static lv_style_t switch_blue;

static lv_obj_t *scr;
static lv_obj_t *tab;

static lv_obj_t *autonName;
static lv_obj_t *autonGraphic;

static lv_obj_t *posVals;
static lv_obj_t *chassisVals;
static lv_obj_t *printValue;
static lv_obj_t *driveErr;
static lv_obj_t *turnErr;

bool clampIsToggled = false, draggerIsToggled = false;

double chassisSpeed = 0;
int currAWP = 0, currElim = 0;

lv_res_t Display::btn_click_action(lv_obj_t *btn) {
  int id = lv_obj_get_free_num(btn);
  switch (id) { // Controls both tab 2 + 3.
  case 1:       // Tab 2 control
    pros::delay(2500);
    autonomous();
    break;
  case 2: // Tab 3 control
    robot->Odom::calibrateGyro()->Odom::zero();
    break;
  case 3:
    robot->Odom::tarePosition()->Odom::reset();
    break;
  case 4:
    frontLift.reset();
    break;
  case 5:{
    frontLift.toggleClamp();
    backLift.toggleClamp();
    break;
  }
  case 6: {
    if (chassis.getState() == ChassisState::IDLE)
      chassis.setState(ChassisState::DEBUG);
    else
      chassis.setState(ChassisState::IDLE);
    break;
  }
  default:
    break;
  }
  return LV_RES_OK;
}

static lv_res_t btn_auton_action(lv_obj_t *btn) {
  if (lv_obj_get_free_num(btn) == 1) {
    currAWP++;
    if (currAWP > 3) currAWP = 1;
    switch (currAWP) {
    case 1:{
      auton.setId(1);
      lv_img_set_src(autonGraphic, &leftAWP_IMG);
      break;}
    case 2:{
      auton.setId(2);
      lv_img_set_src(autonGraphic, &rightAWP_IMG);
      break;}
    case 3:{
      auton.setId(3);
      lv_img_set_src(autonGraphic, &AWP);
      break;}
    }
  } else if (lv_obj_get_free_num(btn) == 2) {
    currElim++;
    if (currElim > 2) currElim = 1;
    switch (currElim) {
    case 1:{
      lv_img_set_src(autonGraphic, &middleGoal_IMG);
      auton.setId(4);
      break;}
    case 2:{
      lv_img_set_src(autonGraphic, &twoGoal_IMG);
      auton.setId(5);
      break;}
    }
  } else if(lv_obj_get_free_num(btn) == 3){
    auton.setId(6);
    lv_img_set_src(autonGraphic, &skills_IMG);
  }
  return LV_RES_OK;
}

Display::Display() {
  if (!isInitalized) {
    // Loading Screen
    lv_obj_t *obj = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(obj, 480, 240);
    lv_obj_set_style(obj, &lv_style_transp); // make the container invisible
    lv_obj_align(obj, NULL, LV_ALIGN_CENTER, 0, 25);

    Gif gif("/usd/Logo.gif", obj);

    // Themes & Styles
    lv_theme_t *th = lv_theme_material_init(180, NULL);

    th->bg->body.main_color = LV_COLOR_HEX(0x4a4a4a);
    th->bg->body.grad_color = LV_COLOR_HEX(0x4a4a4a);

    th->tabview.btn.bg->body.main_color = LV_COLOR_HEX(0x4a4a4a);
    th->tabview.btn.bg->body.grad_color = LV_COLOR_HEX(0x4a4a4a);
    th->tabview.btn.bg->body.shadow.width = 0;

    th->tabview.btn.bg->text.color = LV_COLOR_WHITE;

    lv_theme_set_current(th);

    lv_style_plain.body.radius = 1;

    // Btn Style
    lv_style_copy(&style_btn, &lv_style_plain);
    style_btn.body.main_color = LV_COLOR_GRAY;
    style_btn.body.grad_color = LV_COLOR_GRAY;
    style_btn.body.border.color = LV_COLOR_WHITE;
    style_btn.body.border.width = 2;
    style_btn.text.color = LV_COLOR_WHITE;

    lv_style_copy(&style_btn_released, &lv_style_plain);
    style_btn_released.body.main_color = LV_COLOR_MAKE(48, 48, 48);
    style_btn_released.body.grad_color = LV_COLOR_MAKE(48, 48, 48);
    style_btn_released.body.border.color = LV_COLOR_GRAY;
    style_btn_released.body.border.width = 2;
    style_btn_released.text.color = LV_COLOR_WHITE;

    pros::delay(1000);

    gif.clean(); // Destroy loading screen gif.

    scr = lv_cont_create(NULL, NULL);
    lv_scr_load(scr);

    // Background
    lv_obj_t *background = lv_img_create(scr, NULL);
    lv_obj_set_pos(background, 0, 0);
    lv_img_set_src(background, &destiny);

    // Overlay
    autonName = createLabel(10, 0, "Auton Selected: ", scr);
    lv_label_set_style(autonName, &style_btn);

    // Tabs
    tab = lv_tabview_create(scr, NULL);
    lv_obj_set_pos(tab, 0, 0);
    lv_obj_set_size(tab, 480, 250);
    lv_obj_set_style(tab, &lv_style_transp);
    lv_tabview_set_btns_pos(tab, LV_TABVIEW_BTNS_POS_BOTTOM);
    lv_tabview_set_sliding(tab, false);

    tab1 = lv_tabview_add_tab(tab, "Auton");
    tab2 = lv_tabview_add_tab(tab, "Debug");
    tab3 = lv_tabview_add_tab(tab, "Settings");

    tabAuton(tab1);
    tabDebug(tab2);
    tabSettings(tab3);

    isInitalized = true;
  }
}

void Display::cleanUp() { 
    lv_obj_clean(lv_layer_sys()); 
}

void Display::tabAuton(lv_obj_t *parent) {
  lv_obj_t *AWP = createButton(1, 0, 20, 200, 40, "AWP", parent, btn_auton_action, &style_btn, &style_btn_released);
  lv_obj_t *Elim = createButton(2, 0, 70, 200, 40, "Elim", parent, btn_auton_action, &style_btn, &style_btn_released);
  lv_obj_t *skills = createButton(3, 0, 120, 200, 40, "Skills", parent, btn_auton_action, &style_btn, &style_btn_released);

  autonGraphic = lv_img_create(parent, NULL);
  lv_obj_set_pos(autonGraphic, 250, 50);
  lv_img_set_src(autonGraphic, &noAuton_IMG);
}

void Display::tabDebug(lv_obj_t *parent) {
  lv_obj_t *startAuton = createButton(1, 0, 20, 200, 40, "Start Auton", parent, btn_click_action, &style_btn, &style_btn_released);
  
  posVals = createLabel(10, 70, "RobotPos = (0,0) ", parent);
  lv_label_set_style(posVals, &style_btn);

  chassisVals = createLabel(10, 90, "Chassis State: ", parent);
  lv_label_set_style(chassisVals, &style_btn);

  printValue = createLabel(10, 120, "Value: NONE", parent);
  lv_label_set_style(printValue, &style_btn);

  driveErr = createLabel(10, 140, "Value: NONE", parent);
  lv_label_set_style(driveErr, &style_btn);

  turnErr = createLabel(10, 160, "Value: NONE", parent);
  lv_label_set_style(turnErr, &style_btn);


 }

void Display::tabSettings(lv_obj_t *parent) {
  lv_obj_t *resetIMU = createButton(2, 0, 20, 200, 40, "Reset IMU", parent, btn_click_action, &style_btn, &style_btn_released);
  lv_obj_t *resetpos = createButton(3, 0, 70, 200, 40, "Reset robotPos", parent, btn_click_action, &style_btn, &style_btn_released);
  lv_obj_t *resetLift = createButton(4, 0, 120, 200, 40, "Reset Lift", parent, btn_click_action, &style_btn, &style_btn_released);

  lv_obj_t *toggleDragger = createButton(5, 250, 70, 200, 40, "Toggle Clamp", parent, btn_click_action, &style_btn, &style_btn_released);
  lv_obj_t *spinChassis = createButton(6, 250, 120, 200, 40, "Spin Chassis", parent, btn_click_action, &style_btn, &style_btn_released);
}

void Display::start(void *ignore) {
  if(!isRunning) {
    pros::delay(500);
    Display *that = static_cast<Display *>(ignore);
    that->run();
  }
}

void Display::run() {
  isRunning = true;

  // Tiny logo in top right corner
  lv_obj_t *s_logo = lv_obj_create(lv_layer_sys(), NULL);
  lv_obj_set_size(s_logo, 480, 240);
  lv_obj_set_style(s_logo, &lv_style_transp); // make the container invisible
  lv_obj_align(s_logo, NULL, LV_ALIGN_CENTER, 0, 25);
  lv_obj_set_pos(s_logo, 380, 0);
  Gif small_gif("/usd/small_logo.gif", s_logo);

  while (isRunning) {
    // Current Auton Label
    std::string temp = "Auton Selected: " + auton.getAuton();
    lv_label_set_text(autonName, temp.c_str());

    // Print (X,Y) Coordinates
    std::ostringstream posX, posY;
    posX << floor(robot->Odom::getX().convert(inch) * 100) /100;
    posY << floor(robot->Odom::getY().convert(inch) * 100) /100;
    std::string posTemp = "Curr Pos: (" + posX.str() + "," + posY.str() + ")";
    lv_label_set_text(posVals, posTemp.c_str());

    // Placeholder for quickly adding a sensor value to screen.
    std::ostringstream value;
    value << floor(robot->Odom::getTheta().convert(degree) *100) /100;
    std::string tempValue = "IMU: " + value.str();
    lv_label_set_text(printValue, tempValue.c_str());

    // Print Current drive Error.
    std::ostringstream driveErrValue;
    driveErrValue << floor(chassis.getDriveError().convert(inch) *100) / 100;
    std::string tempDriveErr = "Drive Error: " + driveErrValue.str();
    lv_label_set_text(driveErr, tempDriveErr.c_str());

    // Print Current Turn Error.
    std::ostringstream turnErrValue;
    turnErrValue << floor(chassis.getTurnError().convert(degree) *100) / 100;
    std::string tempTurnErr = "Turn Error: " + turnErrValue.str();
    lv_label_set_text(turnErr, tempTurnErr.c_str());

    std::string cstate;
    switch(chassis.getState()){
      case ChassisState::DRIVE:{ cstate = "Chassis State: DRIVE"; break; }
      case ChassisState::POINT:{ cstate = "Chassis State: POINT"; break; }
      case ChassisState::TURN:{ cstate = "Chassis State: TURN"; break; }     
      case ChassisState::OPCONTROL:{ cstate = "Chassis State: OPCONTROL"; break; }
      case ChassisState::BALANCE:{ cstate = "Chassis State: BALANCE"; break; } 
      case ChassisState::DEBUG: { cstate = "Chassis State: DEBUG"; break; }     
      case ChassisState::IDLE:{ cstate = "Chassis State: IDLE"; break; }
    }
    lv_label_set_text(chassisVals, cstate.c_str());
    
    pros::delay(10);
  }
}

void Display::stop() { isRunning = false; }

lv_obj_t *Display::createLabel(int x, int y, std::string text_, lv_obj_t *parent) {
  lv_obj_t *label = lv_label_create(parent, NULL);
  lv_obj_set_pos(label, x, y);
  lv_label_set_text(label, text_.c_str());

  return label;
}

lv_obj_t *Display::createButton(int id, int x, int y, int width, int height, std::string text, lv_obj_t *parent, lv_action_t action, lv_style_t *btn_pr,lv_style_t *btn_rel) {
  lv_obj_t *button = lv_btn_create(parent, NULL);
  lv_obj_set_pos(button, x, y);
  lv_obj_set_size(button, width, height);
  lv_obj_set_free_num(button, id);
  lv_btn_set_action(button, LV_BTN_ACTION_CLICK, action);

  if (btn_pr != nullptr || btn_rel != nullptr) {
    lv_btn_set_style(button, LV_BTN_STYLE_PR, btn_pr);
    lv_btn_set_style(button, LV_BTN_STYLE_REL, btn_rel);
  } else {
    lv_btn_set_style(button, LV_BTN_STYLE_PR, lv_theme_get_current()->btn.pr);
    lv_btn_set_style(button, LV_BTN_STYLE_REL, lv_theme_get_current()->btn.rel);
  }

  lv_obj_t *buttonLabel = createLabel(0, 0, text, button);
  lv_obj_align(buttonLabel, button, LV_ALIGN_CENTER, 0, 0);

  return button;
}