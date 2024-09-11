// Joystick integration
constexpr int JOY_X_PIN = 19;
int cursor_x_position = 0;
// Not integrated yet constexpr int JOY_Y_PIN = N/A;
constexpr int JOY_BTN_PIN = 46;
bool joy_btn_clicked = false;

// Joystick integration
bool joy_btn_click()
{
    joy_btn_clicked = true;
    // is the output from the pushbutton inside the joystick. It’s normally open. If we
    // use a pull-up resistor in this pin, the SW pin will be  HIGH
    // when it is not pressed, and LOW when Pressed.
    return digitalRead(JOY_BTN_PIN) == HIGH ? false : true;
}

int joyXMid = 0;
int cal_X = 0, cal_Y = 0;

void calibrate_joy()
{
    for (int i = 0; i < 100; i++)
    {
        cal_X += analogRead(JOY_X_PIN);
    }
    // calibrate center
    joyXMid = cal_X / 100;
}

int MID = 100; // 10 mid point delta arduino, use 4 for attiny
int get_joy_x(bool logical = false)
{
    int joyX = analogRead(JOY_X_PIN);

    /*
        Serial.print("Calibrated_X_Voltage = ");
        Serial.print(joyXMid);
        Serial.print("X_Voltage = ");
        Serial.print(joyX);
        Serial.print("\t");
    */

    if (logical)
    {
        // 4095
        if (joyX < joyXMid - MID)
        {
            return -1;
        }
        // 0-5
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
