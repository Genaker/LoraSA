#include "input.h"

int cursor_x_position = 0;
bool joy_btn_clicked = false;

static int joyXMid = 0;

bool joy_btn_click()
{
    joy_btn_clicked = true;
    return digitalRead(JOY_BTN_PIN) == HIGH ? false : true;
}

void calibrate_joy()
{
    int cal_X = 0;
    for (int i = 0; i < 100; i++)
    {
        cal_X += analogRead(JOY_X_PIN);
    }
    joyXMid = cal_X / 100;
}

static const int MID = 100;

int get_joy_x(bool logical)
{
    int joyX = analogRead(JOY_X_PIN);

    if (logical)
    {
        if (joyX < joyXMid - MID)
        {
            return -1;
        }
        else if (joyX > joyXMid + MID)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    return joyX;
}
