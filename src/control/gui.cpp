#include "control/gui.hpp"
#include "control/auton.hpp"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_label.h"

static Autonomous Auton;

bool Display::isRunning = false, Display::isInitalized = false;

static lv_style_t style_btn;
static lv_style_t style_btn_released;

static lv_style_t switch_red;
static lv_style_t switch_blue;

static lv_obj_t *scr;
static lv_obj_t *tab;

static lv_obj_t *autonName;
static lv_obj_t *autonGraphic;

static lv_res_t btn_click_action(lv_obj_t *btn) {
  int id = lv_obj_get_free_num(btn);
  //TODO: RESET BUTTONS
  switch (id) {
  case 1:
    break;
  case 2:
    break;
  case 3:
    break;
  default:
    break;
  }
  return LV_RES_OK;
}

static lv_res_t btn_auton_action(lv_obj_t *btn) {
  int id = lv_obj_get_free_num(btn);
  Auton.setId(id);

  switch (id) {
  case 1: lv_img_set_src(autonGraphic, &homeRow_IMG); break;
  case 2: lv_img_set_src(autonGraphic, &oneGoal_IMG); break;
  case 3: lv_img_set_src(autonGraphic, &twoGoal_IMG); break;
  
  default: break;
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
  lv_obj_t *homeRow = createButton(1, 0, 20, 200, 40, "Home Row", parent, btn_auton_action, &style_btn, &style_btn_released);
  lv_obj_t *oneGoal = createButton(2, 0, 70, 200, 40, "One Goal", parent, btn_auton_action, &style_btn, &style_btn_released);
  lv_obj_t *twoGoal = createButton(3, 0, 120, 200, 40, "Two Goal", parent, btn_auton_action, &style_btn, &style_btn_released);

  autonGraphic = lv_img_create(parent, NULL);
  lv_obj_set_pos(autonGraphic, 250, 50);
  lv_img_set_src(autonGraphic, &noAuton_IMG);
}

void Display::tabDebug(lv_obj_t *parent) { }

void Display::tabSettings(lv_obj_t *parent) {
  lv_obj_t *resetIMU = createButton(1, 0, 20, 200, 40, "Reset IMU", parent, btn_click_action, &style_btn, &style_btn_released);
  lv_obj_t *resetOdom = createButton(2, 0, 70, 200, 40, "Reset Odom", parent, btn_click_action, &style_btn, &style_btn_released);
  lv_obj_t *resetLift = createButton(3, 0, 120, 200, 40, "Reset Lift", parent, btn_click_action, &style_btn, &style_btn_released);
}
void Display::start(void *ignore) {
  pros::delay(500);
  Display *that = static_cast<Display *>(ignore);
  that->run();
}

void Display::run() {
  isRunning = true;

  lv_obj_t *s_logo = lv_obj_create(lv_layer_sys(), NULL);
  lv_obj_set_size(s_logo, 480, 240);
  lv_obj_set_style(s_logo, &lv_style_transp); // make the container invisible
  lv_obj_align(s_logo, NULL, LV_ALIGN_CENTER, 0, 25);
  lv_obj_set_pos(s_logo, 380, 0);
  Gif small_gif("/usd/small_logo.gif", s_logo);
  // Tiny logo in top right corner

  while (isRunning) {
    // Current Auton Label
    std::string temp = "Auton Selected: " + Auton.getAuton();
    lv_label_set_text(autonName, temp.c_str());

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