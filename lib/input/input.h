#pragma once

#include <Arduino.h>

// Joystick pin configuration
constexpr int JOY_X_PIN = 19;
constexpr int JOY_BTN_PIN = 46;

extern int cursor_x_position;
extern bool joy_btn_clicked;

bool joy_btn_click();
void calibrate_joy();
int get_joy_x(bool logical = false);
