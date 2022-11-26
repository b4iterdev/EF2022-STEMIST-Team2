#include <PS2X_lib.h>

//Set pinout to your own, but ENABLE Pin MUST be PWM pin.
#define EN_A 3
#define IN1_A 5
#define IN2_A 6

#define IN1_B 7
#define IN2_B 8
#define EN_B 9

PS2X ps2x; // create PS2 Controller Class

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        13  //14    
#define PS2_CMD        11  //15
#define PS2_SEL        10  //16
#define PS2_CLK        12  //17

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
#define pressures   false
//#define rumble      true
#define rumble      false

int motor_right_speed = 0;
int motor_left_speed = 0;

void setupPS2controller()
{
  ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);
  //  ps2x.read_gamepad(false, 0); // disable vibration of the controller
}
void PS2control()
{
    ps2x.read_gamepad(false,0);

    int nJoyX = ps2x.Analog(PSS_RX); // read x-joystick
    int nJoyY = ps2x.Analog(PSS_LY); // read y-joystick
    
    nJoyX = map(nJoyX, 0, 255, -1023, 1023);
    nJoyY = map(nJoyY, 0, 255, 1023, -1023);
  
    // OUTPUTS
    int nMotMixL; // Motor (left) mixed output
    int nMotMixR; // Motor (right) mixed output
  
    // CONFIG
    // - fPivYLimt  : The threshold at which the pivot action starts
    //                This threshold is measured in units on the Y-axis
    //                away from the X-axis (Y=0). A greater value will assign
    //                more of the joystick's range to pivot actions.
    //                Allowable range: (0..+127)
    float fPivYLimit = 1023.0;
        
    // TEMP VARIABLES
    float   nMotPremixL;    // Motor (left) premixed output
    float   nMotPremixR;    // Motor (right) premixed output
    int     nPivSpeed;      // Pivot Speed
    float   fPivScale;      // Balance scale between drive and pivot
  
    // Calculate Drive Turn output due to Joystick X input
    if (nJoyY >= 0) {
      // Forward
      nMotPremixL = (nJoyX>=0)? 1023.0 : (1023.0 + nJoyX);
      nMotPremixR = (nJoyX>=0)? (1023.0 - nJoyX) : 1023.0;
    } else {
      // Reverse
      nMotPremixL = (nJoyX>=0)? (1023.0 - nJoyX) : 1023.0;
      nMotPremixR = (nJoyX>=0)? 1023.0 : (1023.0 + nJoyX);
    }
  
    // Scale Drive output due to Joystick Y input (throttle)
    nMotPremixL = nMotPremixL * nJoyY/1023.0;
    nMotPremixR = nMotPremixR * nJoyY/1023.0;
  
    // Now calculate pivot amount
    // - Strength of pivot (nPivSpeed) based on Joystick X input
    // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
    nPivSpeed = nJoyX;
    fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0 : (1.0 - abs(nJoyY)/fPivYLimit);
  
    // Calculate final mix of Drive and Pivot
    nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
    nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);
    
    motor_left_speed = nMotMixL;
    motor_right_speed = nMotMixR;
  
    if (motor_right_speed > 50) {
      digitalWrite(IN1_B,HIGH);
      digitalWrite(IN2_B,LOW);
    }
    else if (motor_right_speed < -50) {
      digitalWrite(IN1_B,LOW);
      digitalWrite(IN2_B, HIGH);
    }
    else {
      digitalWrite(IN1_B, LOW);
      digitalWrite(IN2_B, LOW);
    }
  
    if (motor_left_speed > 50) {
      digitalWrite(IN1_A, LOW);
      digitalWrite(IN2_A, HIGH);
    }
    else if (motor_left_speed < -50) {
      digitalWrite(IN1_A,HIGH);
      digitalWrite(IN2_A,LOW);
    }
    else {
      digitalWrite(IN1_A, LOW);
      digitalWrite(IN2_A, LOW);
    }
    analogWrite(EN_A, abs(motor_left_speed));
    analogWrite(EN_B, abs(motor_right_speed));
    if (abs(motor_left_speed > 50) || abs(motor_left_speed > 50)) {
    }
    delay(50);
}
