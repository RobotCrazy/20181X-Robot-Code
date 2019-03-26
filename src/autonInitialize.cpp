#include "autonInitialize.h"
#include "main.h"

void autonSelector()
{

  master.print(0, 4, "Select Auton: ");
  pros::delay(2000);

  //Documentation for this library: https://docs.littlevgl.com

  lv_style_t backgroundStyle;
  lv_style_copy(&backgroundStyle, &lv_style_plain);
  backgroundStyle.body.main_color = LV_COLOR_BLACK;

  lv_obj_t *autonSelectorParent = lv_obj_create(lv_scr_act(), NULL);
  lv_obj_set_style(autonSelectorParent, &backgroundStyle);
  lv_obj_set_size(autonSelectorParent, 480, 272);

  lv_obj_t *redFront = lv_btn_create(autonSelectorParent, NULL);
  lv_obj_align(redFront, NULL, LV_ALIGN_IN_TOP_LEFT, 20, 10);

  lv_obj_t *redBack = lv_btn_create(autonSelectorParent, NULL);
  lv_obj_align(redBack, NULL, LV_ALIGN_IN_TOP_RIGHT, -20, 20);

  lv_obj_t *blueFront = lv_btn_create(autonSelectorParent, NULL);
  lv_obj_align(blueFront, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 30, -45);

  lv_obj_t *blueBack = lv_btn_create(autonSelectorParent, NULL);
  lv_obj_align(blueBack, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -30, -50);
}