/*
 **********************************************************************************************************************
  Kamen-Lab01.ino version 3
   Cade Jin and Billy York 12/5/2017
   This program will demonstate basic function of the robot movement. It use encoder to help reduce odometry problem
   but it is a limit function version and not able to handle odometry due to sliding. The encoder is used to adjust
   the step counting as well as reduce speed to gave a higher touque for robot to overcome static friction.
  *********************************************************************************************************************
  encoder links:
    https://www.dfrobot.com/wiki/index.php/Wheel_Encoders_for_DFRobot_3PA_and_4WD_Rovers_(SKU:SEN0038)
    http://image.dfrobot.com/image/data/SEN0038/encoderSketch.zip
  mega links:
    pin mappings: https://www.arduino.cc/en/Hacking/PinMapping2560
***********************************************************************************************************************
  Hardware Connection:
  digital pin 2 - left wheel encoder
  digital pin 3 - right wheel encoder
  digital pin 4 - start button
  digital pin 13 - enable LED on microcontroller
  digital pin 48 - enable PIN on A4988 Stepper Motor Driver StepSTICK
  digital pin 50 - left stepper motor step pin
  digital pin 51 - left stepper motor direction pin
  digital pin 52 - right stepper motor step pin
  digital pin 53 - right stepper motor direction pin
***********************************************************************************************************************
  Key Variables:
  ecount[2]        //variable to hold number of encoder counts (left, right)
  steps[2]      //variable to hold number of step counts (left, right)
  stepecount[2]      //variable to hold number of step counts during one encoder cycle (left, right)
        Note: the encode reading is uneven compare to the step motor count but it always follow the parrten that flip
        between 50 and 30 which provide an even distribute reading of 80 steps over 2 encounder count which we defined as
        Encoder cycle
  speedreduction[2]      //variable to hold number of reducing speed for wheel overcome touque (left, right)
 *********************************************************************************************************************
  Key Functions of the program:
  wheelinterrupt(int which_wheel) // interrupt for the wheel encoder for the step and speed correction
  clear_encoder() //set the global variable before the movement
  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  Main function of the program:
  movedifferentratio(int leftratio, int rightratio, int[] desire_steps) //move wheel in different ratio prociesely
  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  Main application of the program:
*/

//define pin numbers and variables
//##################################################################################################################################################################################################
//Define pin numbers
const int startbutton = 4;     // start button pin

const int ltEncoder = 2;       // left encoder pin
const int rtEncoder = 3;       // right encoder pin

const int enableLED = 13;      // stepper enabled LED

const int rtStepPin = 46;      // right stepper motor step pin
const int rtDirPin = 53;       // right stepper motor direction pin
const int ltStepPin = 44;      // left stepper motor step pin
const int ltDirPin = 49;       // left stepper motor direction pin
const int stepperEnable = 48;  // stepper enable pin on stepStick

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//define global program constants
#define debug false                       // const for debug mode

#define max_stepTime  500                 // max delay time between high and low on step pin
#define min_stepTime  2500                // min delay time between high and low on step pin
#define max_reducespeed 2000              // the speed reduction for wheel overcome rough surface
#define begin_reduction_speed 500         // the speed reduction for wheel when program start
#define speed_increase 300                // the speed increase for correct action
#define speed_reduce 30                   // the speed reduction for wrong action

#define debounce_time 600                 // time to wait for botton debouncing
#define start_time 1000                   // time to wait before programe start
#define stop_time 500                     // delay between each main application

#define one_rot 800                       // number of counts for one wheel rotation
#define one_ecount 80                     // number of counts for steps while two encoder count
#define one_ecycle 2                      // number of encoder count for a complete encoder cycle
#define significant_error 2               // step of error that need adjustment if over that

#define LEFT  0                           // constant for left wheel
#define RIGHT 1                           // constant for right wheel

#define FWD HIGH                          // move constant for forward
#define REV LOW                           // move constant for reverse

#define baud 9600                         // serial communication baud rate

#define wheelcircumference 10.5625        // inches
#define wheelwidth 3.4375                 // inches
#define robotwidth 8.3125                 // inches

//define global variable
volatile long ecount[2] = {0, 0};         // interrupt variable to hold number of encoder counts (eft, right)
volatile int steps[2] = {0, 0};           // global variable to hold number of steps counts (left, right)
volatile int stepecount[2] = {0, 0};      // global variable to hold number of step counts of each two encoder counts (left, right)
volatile int speedreduction[2] = {0, 0};  // global variable for speed reduction

//#########################################################################################################################
void setup() {
  //initial step motor
  pinMode(rtStepPin, OUTPUT);                                              // sets right stepper pin as output
  pinMode(rtDirPin, OUTPUT);                                               // sets right stepper direction pin as output
  pinMode(ltStepPin, OUTPUT);                                              // sets left stepper pin as output
  pinMode(ltDirPin, OUTPUT);                                               // sets left stepper director pin as output
  pinMode(stepperEnable, OUTPUT);                                          // sets stepper enable pin as output
  digitalWrite(stepperEnable, LOW);                                        // turns on the stepper motor driver
  digitalWrite(ltDirPin, FWD);                                             // sets left wheel to go forward
  digitalWrite(rtDirPin, FWD);                                             // sets right wheel to go forward

  //initial step motor enable light
  pinMode(enableLED, OUTPUT);                                              // sets pin 13 enable LED on microcontroller as output
  digitalWrite(enableLED, HIGH);                                           // turn on enable LED

  //initial the encoder
  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);  // init the interrupt mode for the digital pin 2
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);  // init the interrupt mode for the digital pin 3

  //initial the start button
  pinMode(startbutton, INPUT);

  //initial the Serial communication if debuging
  if (debug) {
    Serial.begin(baud);                                                    // init the Serial port for debug mode
    while (! Serial);                                                      // Wait untilSerial is ready
    Serial.println("Serial is Open");
    Serial.println("System stand by");
  }

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  //waiting for start by botton push
  bool pressed = false;                                                 // boolean for debounce
  unsigned long debouncetimer = millis();                               // timer for debounce
  while (! (pressed && (millis() - debouncetimer) > debounce_time) ) {  // escape only when botton is pressed after debounce time(ms)
    if (digitalRead(startbutton) == LOW && ! pressed) {                 // when first cycle catch press
      digitalWrite(enableLED, LOW);                                     // LED on pin 13 off for indication
      debouncetimer = millis();                                         // record the time
      pressed = true;                                                   // record the press
      if (debug) {                                                      // serial print for debuging
        Serial.println("Pressed");
        Serial.println(debouncetimer);
        Serial.println(millis());
      }
    } else if (digitalRead(startbutton) == HIGH) {                      // when not pressed
      digitalWrite(enableLED, HIGH);                                    // LED on pin 13 on for indication
      if (debug && pressed) {                                           // serial print for debuging (only print at first cycle catch release
        Serial.println("released");
      }
      pressed = false;                                                  // record the release
    }
  }

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  //Programe start after release
  if (debug) {                                                 //  serial print for debuging
    Serial.println("Start the program after button release");
    Serial.println(debouncetimer);
    Serial.println(millis());
  }
  digitalWrite(enableLED, HIGH);                               // LED on pin 13 on for indication
  while (digitalRead(startbutton) == LOW);                     // wait till the botton is released
  delay(start_time);                                           // Delay before start
  
  moveSquare();
  stopmoving();
  movecircle(LEFT,2);
  stopmoving();
  movefigure8(2);
  stopmoving();
  goToGoal(1,2);
  stopmoving();
  goToGoal(-2, 1);
  stopmoving();
  goToAngle(-45);
  stopmoving();
  goToAngle(60);
  stopmoving();
  
  digitalWrite(stepperEnable, HIGH);                           // turns on the stepper motor driver
}

//##################################################################################################################################################################################################
void loop() {
  // put your main code here, to run repeatedly:

}
//##################################################################################################################################################################################################
//interrupt function trigered by left encoder
void LwheelSpeed() {
  wheelinterrupt(LEFT);                                                                                                       // for simplify code
}

//interrupt function trigered by right encoder
void RwheelSpeed()
{
  wheelinterrupt(RIGHT);                                                                                                      // for simplify code
}

//genrallized interruption function
void wheelinterrupt(int i) {
  bool addspeed = false;                                                                                                      // boolean to store if the motor need to slow down
  ecount[i]++;
  if (ecount[i] == one_ecycle) {                                                                                              // check if it is a complete encoder cycle
    if (steps[i] > one_ecount) {                                                                                              // check if the step is greater than on encoder cycle(deal with initialize problem
      if (stepecount[i] < (one_ecount / 4)) {                                                                                 // if encoder cycle step coutn is less than 25% (assume not moving)
        steps[i] -= stepecount[i];                                                                                            // clean step as the wheel is not moving
        addspeed = true;                                                                                                      // reduce speed
      } else if ((stepecount[i] < (one_ecount - (one_ecount / 40))) || (stepecount[i] > (one_ecount + (one_ecount / 40)))) {  // check if the ecount is 2.5% offset from the ideal number
        steps[i] -= (stepecount[i] - 80);                                                                                     // correct the step
        addspeed = true;                                                                                                      // reduce speed
      } else {                                                                                                                // if nothing going on increase the speed
        speedreduction[i] = (speedreduction[i] - speed_increase);
        speedreduction[i] = speedreduction[i] > 0 ? speedreduction[i] : 0;
      }
    }
    if (debug) {
      Serial.print(i);
      Serial.print(" esteps =");
      Serial.println(stepecount[i]);
      Serial.print(i);
      Serial.print(" steps =");
      Serial.println(steps[i]);
    }
    ecount[i] = 0;                                                                                                            // clear encorder counting
    stepecount[i] = 0;                                                                                                        // clear step counting for enconder cycle
  }
  if (stepecount[i] >= (one_ecount * 7 / 10) ) {                                                                              // for any possibility the step encorder won't over 70% of the entire step. Thus if so, need correct the step for that
    addspeed = true;                                                                                                          // reduce speed
    steps[i] -= (stepecount[i] - one_ecount * 7 / 10);                                                                        // correct the step reading
    stepecount[i] = one_ecount * 7 / 10;
  }
  if (addspeed) {                                                                                                             // reduce speed until reach the limitation
    speedreduction[i] = speedreduction[i] < max_reducespeed ? (speedreduction[i] + speed_reduce) : max_reducespeed;
  }
}
//##################################################################################################################################################################################################
//initial function for the movement
void clear_encoder() {
  for (int i = 0; i < 2; i++) {
    ecount[i] = 0;                              // clear the encorder counting
    speedreduction[i] = begin_reduction_speed;  // gave a little speed reduction for wheel initialize
    steps[i] = -1;                              // clear steps record the actual step the robot move (-1 for the initial miss count issue
    stepecount[i] = -1;                         // esteps record the step between two encoder reading (-1 for the initial miss count issue)
  }
}

//##################################################################################################################################################################################################
// function for handle the step motor moving at different ratio
void movedifferentratio(int leftratio, int rightratio, int desire_step[]) {
  clear_encoder();                                                                                 // clear the global variable
  unsigned long timer[2] = {micros(), micros()};                                                   // for the step switch timer.
  int pinsets[2] = {LOW, LOW};                                                                     // record for current motor step pin stats
  int steppins[2] = {ltStepPin, rtStepPin};                                                        // record for motor step pin
  bool steppause[2] = {false, false};                                                              // local variable for the ratio correctenese
  int motor_speed[2] = {max_stepTime, max_stepTime};                                               // desire delay for each motor in us ( limited between 500 to infinity due to hardware limitation)

  digitalWrite(steppins[LEFT], pinsets[LEFT]);                                                     // preset the motor to readay for the movement
  digitalWrite(steppins[RIGHT], pinsets[RIGHT]);

  if (leftratio > 0 && rightratio > 0) {                                                           // deal with divided by zero
    if (leftratio > rightratio) {                                                                  // calculate the step time according to the ratio(let the fast one run at the faster speed)
      // only support up to 5 : 1 ratio at maximum for better performance
      motor_speed[RIGHT] = leftratio * motor_speed[LEFT] / rightratio;
      motor_speed[RIGHT] = motor_speed[RIGHT] > min_stepTime ? min_stepTime : motor_speed[RIGHT];
    } else {
      motor_speed[LEFT] = rightratio * motor_speed[RIGHT] / leftratio;
      motor_speed[LEFT] = motor_speed[LEFT] > min_stepTime ? min_stepTime : motor_speed[LEFT];
    }
  }
  while (steps[LEFT] < desire_step[LEFT] || steps[RIGHT] < desire_step[RIGHT]) {                   // while the steps cunt below the desires step
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //moving part
    for (int i = 0; i < 2; i ++) {                                                                 // combine for left (0) and right(1)
      if ((micros() - timer[i]) >= (motor_speed[i] + speedreduction[i]) || micros() < timer[i]) {  // check if timer is over the desire value. or chip timer overflow
        if (pinsets[i] == HIGH) {                                                                  // basicly flip the step pin for motor to next step, count step and reset timer
          pinsets[i] = LOW;
          steppause[i] = false;
        } else {
          pinsets[i] = HIGH;
          steps[i] = steps[i] + (steppause[i] ? 0 : 1);
          stepecount[i] = stepecount[i] + (steppause[i] ? 0 : 1);
        }
        if (!steppause[i]) {                                                                       // if the ratio is significant different, pause for one step to fix it
          digitalWrite(steppins[i], pinsets[i]);
        }

        timer[i] = micros();                                                                       // reset the timer
      }
      //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //adjust speed by encoder reading
      if (stepecount[i] > (one_ecount * 13 / 10)) {                                                     //enter if step is over 130% of desire step of the encoder cycle
        int edifferent = (stepecount[i] - (one_ecount * 13 / 10));                                      //correct the step count
        steps[i] = steps[i] - edifferent;
        stepecount[i] = stepecount[i] - edifferent;
        speedreduction[i] = speedreduction[i] + one_ecount * 2 / 10;                                    //reduce the speed
        speedreduction[i] = speedreduction[i] > max_reducespeed ? max_reducespeed : speedreduction[i];
      }
    }
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //adjust speed for different ratio
    if (steps[LEFT] >= 0 && steps[RIGHT] >= 0) {                                       // prevent divide by zero problem
      int sdifferent = 0;                                                              // calculate the error from desired

      if (leftratio > 0 && rightratio > 0) {                                           // deal with divide by zero
        if (leftratio > rightratio) {                                                  // always convert smaller ratio to bigger ratio for expending the difference
          sdifferent = (((long)steps[RIGHT] * leftratio) / rightratio) - steps[LEFT];
        } else {
          sdifferent = steps[RIGHT] - (((long)steps[LEFT] * rightratio) / leftratio);
        }
      }
      steppause[LEFT] = leftratio == 0 || sdifferent < -significant_error;             // pause the next step if the error is significant
      steppause[RIGHT] = rightratio == 0 || sdifferent > significant_error;
    }
  }
}

//##################################################################################################################################################################################################
// function to let robot move forward
void forward(double distanceinfeet) {
  digitalWrite(ltDirPin, FWD);
  digitalWrite(rtDirPin, FWD);
  double distanceinin = distanceinfeet * 12;                              // unit convertion
  int stepcount = (int) (distanceinin / wheelperimeter * one_rot + 0.5);
  int desire_steps[2] = {stepcount, stepcount};
  movedifferentratio(1, 1, desire_steps);
}

//##################################################################################################################################################################################################
// function to let robot move reverse
void reverse(double distanceinfeet) {
  digitalWrite(ltDirPin, REV);
  digitalWrite(rtDirPin, REV);
  double distanceinin = distanceinfeet * 12;                              // unit convertion
  int stepcount = (int) (distanceinin / wheelperimeter * one_rot + 0.5);
  int desire_steps[2] = {stepcount, stepcount};
  movedifferentratio(1, 1, desire_steps);
}

//##################################################################################################################################################################################################
// function to stop robot moving
void stopmoving() {
  digitalWrite(ltDirPin, FWD);
  digitalWrite(rtDirPin, FWD);
  digitalWrite(rtStepPin, LOW);
  digitalWrite(ltStepPin, LOW);
  delay(stop_time);
}

//##################################################################################################################################################################################################
// function to let robot spin
void spin(int toward, int deg) {
  digitalWrite(ltDirPin, (toward == RIGHT ? FWD : REV));
  digitalWrite(rtDirPin, (toward == LEFT ? FWD : REV));
  double perimetertotravel = (robotwidth / 2 * PI * 2) * deg / 360 ;           // unit convertion
  int stepcount = (int) (perimetertotravel / wheelperimeter * one_rot + 0.5);
  int desire_steps[2] = {stepcount, stepcount};
  movedifferentratio(1, 1, desire_steps);
}
//##################################################################################################################################################################################################
// function to let robot pivot
void pivot(int toward, int deg) {
  digitalWrite(ltDirPin, FWD);
  digitalWrite(rtDirPin, FWD);
  double perimetertotravel = (robotwidth * PI * 2) * deg / 360 ;               // unit convertion
  int stepcount = (int) (perimetertotravel / wheelperimeter * one_rot + 0.5);
  int desire_steps[2] = {stepcount, stepcount};
  desire_steps[toward] = 0;
  movedifferentratio((toward == LEFT ? 0 : 1), (toward == RIGHT ? 0 : 1), desire_steps);
}
//##################################################################################################################################################################################################
// function to let robot turn
void turn(int toward, int deg, double radius) {
  digitalWrite(ltDirPin, FWD);
  digitalWrite(rtDirPin, FWD);
  double leftratio;
  double rightratio;
  int leftsteps;
  int rightsteps;
  double radiusin = radius * 12;
  if (radiusin > (robotwidth / 2)) {
    if (toward == LEFT) {
      leftratio = ((radiusin - (robotwidth / 2)) * 2 * PI / wheelperimeter );
      rightratio = ((radiusin + (robotwidth / 2)) * 2 * PI / wheelperimeter);
    } else {
      rightratio = ((radiusin - (robotwidth / 2)) * 2 * PI / wheelperimeter);
      leftratio = ((radiusin + (robotwidth / 2)) * 2 * PI / wheelperimeter);
    }
    leftsteps = (int) round(leftratio / 360 * deg * one_rot + 0.5);
    rightsteps = (int) round(rightratio / 360 * deg * one_rot + 0.5);
    int desiredsteps[2] = { leftsteps, rightsteps };
    movedifferentratio((int) leftratio, (int) rightratio, desiredsteps);
  }
}
//##################################################################################################################################################################################################
// function to let robot move circle
void movecircle(int toward, double diameter){
  turn(toward, 310, diameter / 2);
}
//##################################################################################################################################################################################################
// function to let robot move figure 8
void movefigure8(double diameter){
  movecircle(LEFT , diameter);
  movecircle(RIGHT , diameter);
}
//##################################################################################################################################################################################################
// function to let robot turn to specific angle
void goToAngle(int Angle) {
  spin(Angle > 0 ? LEFT : RIGHT, abs(Angle));
}
//##################################################################################################################################################################################################
// function to let robot go to specific goal
void goToGoal(int x, int y) {
  double angle = 0;
  if(x != 0){
    angle = atan(((double) abs(x)) / abs(y)) * 180 / PI;
    angle = y < 0 ? 180 - angle : angle;
    angle = abs(x) / x * angle * -1;
    goToAngle(angle);
    stopmoving();
  }
  forward(sqrt(sq(x) + sq(y)));
  if(angle!=0){
  stopmoving();
    goToAngle(angle * -1);
    
  }
}
//##################################################################################################################################################################################################
// function to let robot move a squre
void moveSquare(){
  for(int i = 0; i < 4; i++){
    forward(2);
  stopmoving();
    pivot(RIGHT, 90);
  stopmoving();
  }
}

void randomWander
