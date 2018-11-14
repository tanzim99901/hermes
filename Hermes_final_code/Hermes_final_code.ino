#include <Mouse.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/////// MOTOR PINS ////////
#define RPWM_left 6
#define LPWM_left 5

#define RPWM_right 11
#define LPWM_right 10

#define L_EN_left 3
#define R_EN_left 4

#define L_EN_right 8
#define R_EN_right 9

#define forward 1
#define backward 2
#define left 3
#define right 4
////////////////////////////

////// HAND CONTROLLER PINS /////////////
#define leftbuttonPin 40   //left steer rocker switch   (RED)
#define rightbuttonPin 41 //right steer rocker switch   (YELLOW)
#define forwardtrimbuttonPin 42  //trim rocker switch   (GREEN)
#define backtrimbuttonPin 43 //trim rocker switch other way   (WHITE)
#define deadmanbuttonPin 45  // deadman button , push to make switch. If you let go, motors both stop permamently
//////////////////////////////////////

////// LED PINS ///////
#define ledonePin 50
#define ledtwoPin 51
///////////////////////

MPU6050 mpu;
int counter = 0;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float pitch;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



//setup all variables. Terms may have strange names but this software has evolved from bits and bobs done by other segway clone builders

// code that keeps loop time at 10ms per cycle of main program loop xxxxxxxxxxxxxxx
int STD_LOOP_TIME = 9;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

//XXXXXXXXXXXXXXXXXXXXXXXXXXX USER ADJUSTABLE VARIABLES XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
float gyroscalingfactor = 1.0;  //strange constant. It is best thought of as the "balance gyro scaling factor"
//Increase the value of Start_Balance_Point to bring the initial balance point further backwards
float Start_Balance_point = 0;

float P_constant = 40;  //previously 4.5
float D_constant = 0; //previously 0.5
float I_constant = 2;  //previously 1.0

float overallgaintarget = 0.6;   //previously 0.6
float overallgainstart = 0.3; //starting value before softstart 0.3
const int AvgAngles = 5;
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX END OF USER ADJUSTABLE VARIABLES XXXXXXXXXXXXXX

float overallgain;

float level = 0;
int cSpeedVal_Motor1 = 0;
int cSpeedVal_Motor2 = 0;

float Steering;
float SteerValue;
float SteerCorrect;
float steersum;
int Steer = 0;

int firstloop;

float g;
float s;

float gangleratedeg;

int cut = 100;

float gangleraterads;

int k4;
int k5;
int k6;
int k7;
int k8;

float gyroangledt;
float angle;
float anglerads;
float balance_torque;
float cur_speed;
float balancetrim;

int i;
int j;
int tipstart;

signed char Motor1percent;
signed char Motor2percent;


float prevTargetAngle = 0;
float targetAngle = 0;

float angles[5];

float currAngle = 0, prevAngle = 0;
float prevAngles[3];
int prevAngleI = 0;

// time vars
int currTime = 0;
int prevTime = 0;

void setup()
{
  //digital inputs
  pinMode(deadmanbuttonPin, INPUT);
  pinMode(rightbuttonPin, INPUT);
  pinMode(leftbuttonPin, INPUT);
  pinMode(forwardtrimbuttonPin, INPUT);
  pinMode(backtrimbuttonPin, INPUT);
  //digital outputs
  pinMode(ledonePin, OUTPUT);
  pinMode(ledtwoPin, OUTPUT);

  pinMode(RPWM_left, OUTPUT);
  pinMode(LPWM_left, OUTPUT);

  pinMode(RPWM_right, OUTPUT);
  pinMode(LPWM_right, OUTPUT);

  pinMode(R_EN_left, OUTPUT);
  pinMode(L_EN_left, OUTPUT);

  pinMode(R_EN_right, OUTPUT);
  pinMode(L_EN_right, OUTPUT);

  digitalWrite(deadmanbuttonPin, HIGH);  //enables the Arduino internal pullup. when button pressed it connects it to ground giving a value of zero when pressed
  digitalWrite(rightbuttonPin, HIGH);
  digitalWrite(leftbuttonPin, HIGH);
  digitalWrite(forwardtrimbuttonPin, HIGH);
  digitalWrite(backtrimbuttonPin, HIGH);
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  //kill motors when first switched on
  motor_stop();
  //read_gyro();
}

void loop()
{
  //read_gyro();
  tipstart = 0;
  overallgain = 0;
  cur_speed = 0;
  level = 0;
  Steer = 0;
  balancetrim = 0;

  digitalWrite(ledonePin, HIGH);
  Serial.println("Tilt board");

  //Tilt board one end on floor. Turn it on and let go i.e. stop wobbling it about
  //as now the software will read the gyro values 200 times when there is no rotational movement to find the average zero point for each gyro.
  //delay(2000);
  //Serial.println("Sampling inputs");

  //delay(100);
  for (i = 0; i < 200; i++) {
    //read_gyro();
    sample_inputs();
  }
  //delay(2000);
  //read_gyro();
  Serial.println("Sampling finished");
  //delay(2000);
  digitalWrite(ledonePin, HIGH);
  digitalWrite(ledtwoPin, HIGH);

  for (i = 0; i < 50; i++) {
    //XXXXXXXXXXXXXXXXXXXXX TIMEKEEPER      loop timing control keeps it at 100 cycles per second XXXXXXXXXXXXXXX
    lastLoopUsefulTime = millis() - loopStartTime;
    if (lastLoopUsefulTime < STD_LOOP_TIME) {
      delay(STD_LOOP_TIME - lastLoopUsefulTime);
    }
    lastLoopTime = millis() - loopStartTime;
    loopStartTime = millis();
    //XXXXXXXXXXXXXXXXXXXXXX end of loop timing control XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    sample_inputs();

  }
  Serial.println("Loop timing done");
  //delay(2000);
  while (tipstart < 5) {  //don't know why I chose 5 but there we are
    //XXXXXXXXXXXXXXXXXXXXX TIMEKEEPER      loop timing control keeps it at 100 cycles per second XXXXXXXXXXXXXXX
    lastLoopUsefulTime = millis() - loopStartTime;
    if (lastLoopUsefulTime < STD_LOOP_TIME) {
      delay(STD_LOOP_TIME - lastLoopUsefulTime);
    }
    lastLoopTime = millis() - loopStartTime;
    loopStartTime = millis();
    //XXXXXXXXXXXXXXXXXXXXXX end of loop timing control XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    sample_inputs();
    Serial.println(" INSIDE WHILE TIPSTART 5");

    if ((currAngle < (Start_Balance_point - 1)) || (currAngle > (Start_Balance_point + 1))) {

      tipstart = 0;
      overallgain = 0;
      cur_speed = 0;
      level = 0;
      Steer = 0;
      balancetrim = 0;
    }
    else {
      tipstart = 5;

      digitalWrite(ledonePin, LOW);
      digitalWrite(ledtwoPin, LOW);
    }

  } //end of while tipstart < 5

  overallgain = overallgainstart;  //softstart value. Gain will now rise to final of 0.6 at rate of 0.005 per program loop.
  //i.e. it will go from 0.3 to 0.6 over the first 4 seconds or so after tipstart has been activated
  //Serial.println(overallgain);
  //delay(2000);
  angle = 0;
  cur_speed = 0;
  Steering = 512;
  SteerValue = 512;
  balancetrim = 0;

  firstloop = 1;
  Serial.println("END OF TILT START");
  //delay(2000);
  //end of tiltstart code. If go beyond this point then machine is active
  //main balance routine, just loops forever. Machine is just trying to stay level. You "trick" it into moving by tilting one end down
  //works best if keep legs stiff so you are more rigid like a broom handle is if you are balancing it vertically on end of your finger
  //if you are all wobbly, the board will go crazy trying to correct your own flexibility.
  //NB: This is why a segway has to have vertical handlebar otherwise ankle joint flexibility in fore-aft direction would make it oscillate wildly.
  //NB: This is why the handlebar-less version of Toyota Winglet still has a vertical section you jam between your knees.

  while (1)
  {
    /*if (counter == 100)
      {
      //sample_inputs();
      //while (1);
      //set_motor();
      Serial.println("MOTOR STATUS");
      Serial.println(level);
      Serial.println(Steer);
      Serial.println(cSpeedVal_Motor1);
      Serial.println(cSpeedVal_Motor2);
      Serial.println(Motor1percent);
      Serial.println(Motor2percent);
      while (1);
      }*/

    /*motor_drive(left, forward, 255);
      motor_drive(right, forward, 255);
      delay(3000);
      motor_drive(left, backward, 255);
      motor_drive(right, backward, 255);
      delay(3000);*/
      drive_mode();
  }
}

void stop_mode()
{
  motor_stop();
}

void drive_mode()
{
  Serial.println("ENTER MODE");
  //delay(200000);
  sample_inputs();
  Serial.println("Sample function");
  //delay(2000);
  set_motor();

  //XXXXXXXXXXXXXXXXXXXXX loop timing control keeps it at 100 cycles per second XXXXXXXXXXXXXXX
  lastLoopUsefulTime = millis() - loopStartTime;
  if (lastLoopUsefulTime < STD_LOOP_TIME) {
    delay(STD_LOOP_TIME - lastLoopUsefulTime);
  }
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
  //XXXXXXXXXXXXXXXXXXXXXX end of loop timing control XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  Serial.println("DONE");
  //delay(2000);
  serialOut_timing();//updates the LED status every now and then (one means sending >50% of full power to motors.
  //Two LED's lit means sending > 75% of full power to motors and you need to slow down.

  //XXXXXXXXXXXXXXXXXXXX softstart function: board a bit squishy when you first bring it to balanced point,
  //then ride becomes firmer over next 4 seconds as value for overallgain increases from starting value of 0.3 to 0.6(overallgaintarget) set in user adjustable variables at start.
  if (overallgain < overallgaintarget) {
    overallgain = (float)overallgain + 0.005;
  }
  if (overallgain > overallgaintarget) {
    overallgain = overallgaintarget;
  }
  //XXXXXXXXXXXXXXX end of softstart code XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  Serial.println(overallgain);
  //counter++;
  //delay(5000);

}
// ================================================================
// ===               READ GYROSCOPE                             ===
// ================================================================
void read_gyro()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }


  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    delay(3000);
    //read_gyro();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    // display Yaw, Pitch and Roll
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
    Serial.print("ypr\t");
    Serial.print(ypr[0]);
    Serial.print("\t");
    Serial.print(ypr[1]);
    Serial.print("\t");
    Serial.println(ypr[2]);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void motor_stop()
{
  digitalWrite(R_EN_left, LOW);
  digitalWrite(L_EN_left, LOW);
  analogWrite(RPWM_left, 0);
  analogWrite(LPWM_left, 0);

  digitalWrite(R_EN_right, LOW);
  digitalWrite(L_EN_right, LOW);
  analogWrite(RPWM_right, 0);
  analogWrite(LPWM_right, 0);
}

void updateAngle() {
  read_gyro();
  //sixDOF.getYawPitchRoll(angles);
  /*prevAngles[prevAngleI] = ypr[1];
    prevAngleI = (prevAngleI + 1) % AvgAngles;
    float sum = 0;
    for (int i = 0; i < AvgAngles; i++)
    sum += prevAngles[i];
    currAngle = sum / AvgAngles;
    prevAngle = currAngle;
    Serial.println(AvgAngles);
    Serial.println(sum);

    Serial.println(currAngle);
    Serial.println(prevAngleI);
    //Serial.println(AvgAngles);
    delay(20000);*/
  prevAngles[prevAngleI] = ypr[1];
  prevAngle = currAngle;
  currAngle = prevAngles[prevAngleI] - AvgAngles;
  //Serial.println(currAngle);
  //Serial.println(prevAngleI);
  //delay(20000);
}

void sample_inputs()
{
  //int leftbuttonPin = 40;
  k4 = digitalRead(deadmanbuttonPin); //1 when not pressed and 0 when is being pressed
  k5 = digitalRead(leftbuttonPin);
  k6 = digitalRead(rightbuttonPin);
  k7 = digitalRead(forwardtrimbuttonPin);
  k8 = digitalRead(backtrimbuttonPin);
  updateAngle();


  //adjust balance trim
  if (k7 == 0) { //is 0 when it IS being pressed
    balancetrim = balancetrim - 0.005;
  }
  if (k8 == 0) {
    balancetrim = balancetrim + 0.005;
  }




  if (balancetrim < -8) balancetrim = -8; //stops you going too far with this
  if (balancetrim > 8) balancetrim = 8; //stops you going too far the other way



  //STEERING
  if (k5 == 0) {

    SteerValue = (float)SteerValue + 0.5;
  }

  if (k6 == 0) {

    SteerValue = (float)SteerValue - 0.5;
  }

  if ((k6 == 1) && (k5 == 1)) {
    SteerValue = 512;
  }  //if steering switch not being flicked either way then you want to go straight on


  if (SteerValue < 362) {
    SteerValue = 362;  //limiting max rate of turning (512 is no turning)
  }
  if (SteerValue > 662) {
    SteerValue = 662;   //limiting max rate of turning
  }
  SteerCorrect = 0;
  //XXXXXXXXXXX  End of steering code




  if (firstloop == 1) {
    lastLoopTime = 10;
    firstloop = 0;

    gyroangledt = 0;
    gangleraterads = 0;
  }
  else {
    gangleratedeg = (float) (currAngle - prevAngle) * (1 / (lastLoopTime * 0.001)); //angle change rate in degrees per second
    gyroangledt = (float) (gyroscalingfactor * lastLoopTime * 0.001 * gangleratedeg);
    gangleraterads = (float) (gangleratedeg * 0.017453); // just a scaling issue from history
  }

  angle = (float) currAngle + balancetrim;

  anglerads = (float) angle * 0.017453;

  balance_torque = (float) (P_constant * anglerads) + (D_constant * gangleraterads);

  cur_speed = (float) (cur_speed + (I_constant * anglerads * 0.001 * lastLoopTime));

  level = (float)(balance_torque + cur_speed) * overallgain;
  Serial.println(SteerValue);
  Serial.println(angle);
  Serial.println(anglerads);
  Serial.println(balance_torque);
  Serial.println(cur_speed);
  Serial.println(level);
  Serial.println(overallgain);
  //while(1);
}

void set_motor()   {
  //unsigned char cSpeedVal_Motor1 = 0;
  //unsigned char cSpeedVal_Motor2 = 0;

  level = level * 200; //changes it to a scale of about -100 to +100
  if (level < -100) {
    level = -100;
  }
  if (level > 100) {
    level = 100;
  }



  Steer = (float) SteerValue - SteerCorrect;  //at this point is on the 0-1023 scale
  //SteerValue is either 512 for dead ahead or bigger/smaller if you are pressing steering switch left or right

  Steer = (Steer - 512) * 0.19;   //gets it down from 0-1023 (with 512 as the middle no-steer point) to -100 to +100 with 0 as the middle no-steer point on scale




  //set motors using the simplified serial Sabertooth protocol (same for smaller 2 x 5 Watt Sabertooth by the way)

  Motor1percent = (signed char) level + Steer;
  Motor2percent = (signed char) level - Steer;


  if (Motor1percent > 100) Motor1percent = 100;
  if (Motor1percent < -100) Motor1percent = -100;
  if (Motor2percent > 100) Motor2percent = 100;
  if (Motor2percent < -100) Motor2percent = -100;

  /*if (k4 == 1)
    {
    cut = cut - 1;
    if (cut < 0)
    {
      cut = 0;
    }
    }

    if (cut == 0)
    {
    level = 0;
    Steer = 0;
    motor_stop();


    while (1)
    { //loops endlessly until reset
      delay(500);
      pinMode(ledonePin, HIGH);
      pinMode(ledtwoPin, HIGH);

      delay(500);
      pinMode(ledonePin, LOW);
      pinMode(ledtwoPin, LOW);
    } // end of while 1
    }*/
  /*
    //if not pressing deadman button on hand controller - cut everything
    if (k4 == 0) { //is 0 when you ARENT pressing the deadman button
    cut = cut - 1;
    if (cut < 0) {
      cut = 0;
    }
    }
    if (k4 == 1) { //is 1 when you ARE pressing deadman button
    cut = cut + 1;
    if (cut > 50) {
      cut = 50; //if cut is 100 takes 1 second off the deadman before motors actually cut
    }
    }


    if (cut == 0) {
    level = 0;
    Steer = 0;
    motor_stop();


    while (1) {   //loops endlessly until reset


      delay(500);
      pinMode(ledonePin, HIGH);
      pinMode(ledtwoPin, HIGH);

      delay(500);
      pinMode(ledonePin, LOW);
      pinMode(ledtwoPin, LOW);



    } // end of while 1
    }   //end of if cut == 0

    //cut is not 0 so we therefore enact the specified command to the motors
  */

  cSpeedVal_Motor1 = map (Motor1percent, -100, 100, -255, 255);
  cSpeedVal_Motor2 = map (Motor2percent, -100, 100, -255, 255);

  ////// MOTOR 1  ////////
  if (cSpeedVal_Motor1 == 0)
  {
    motor_stop();
  }
  else if (cSpeedVal_Motor1 > 0)
  {
    motor_drive(left, backward, cSpeedVal_Motor1);
  }
  else if (cSpeedVal_Motor1 < 0)
  {
    motor_drive(left, forward, abs(cSpeedVal_Motor1));
  }

  ////// MOTOR 2  //////////
  if (cSpeedVal_Motor2 == 0)
  {
    motor_stop();
  }
  else if (cSpeedVal_Motor2 > 0)
  {
    motor_drive(right, backward, cSpeedVal_Motor2);
  }
  else if (cSpeedVal_Motor2 < 0)
  {
    motor_drive(right, forward, abs(cSpeedVal_Motor2));
  }
  Serial.println(cSpeedVal_Motor1);
  Serial.println(cSpeedVal_Motor2);

  //delay(2000);
}

void motor_drive(int motor, int direct, int power)
{
  if (motor == left)
  {
    digitalWrite(R_EN_left, HIGH);
    digitalWrite(L_EN_left, HIGH);
    if (direct == forward)
    {
      analogWrite(LPWM_left, power);
      analogWrite(RPWM_left, 0);
    }
    else if (direct == backward)
    {
      analogWrite(RPWM_left, power);
      analogWrite(LPWM_left, 0);
    }
  }
  else if (motor == right)
  {
    digitalWrite(R_EN_right, HIGH);
    digitalWrite(L_EN_right, HIGH);
    if (direct == forward)
    {
      analogWrite(LPWM_right, power);
      analogWrite(RPWM_right, 0);
    }
    else if (direct == backward)
    {
      analogWrite(RPWM_right, power);
      analogWrite(LPWM_right, 0);
    }
  }
}

void serialOut_timing()
{
  static int skip = 0;

  if (skip++ == 20)
  { //display every 2000ms (at 100Hz)
    skip = 0;
  }
  if ((Motor1percent > 50)  || (Motor1percent < -50))
  {
    digitalWrite(ledonePin, HIGH);
  }
  if ((Motor1percent > 75) || (Motor1percent < -75))
  {
    digitalWrite(ledtwoPin, HIGH);
  }
  if ((Motor1percent <= 50) && (Motor1percent >= -50))
  {
    digitalWrite(ledtwoPin, LOW);
    digitalWrite(ledonePin, LOW);

  }
}
