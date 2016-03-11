#include <I2C.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.


/*********************************************************
 * LIMIT SWITCH
 */
int ledPin = 13; // LED connected to digital pin 13
int inAPin0 = A0;
int inAPin1 = A1;


/*********************************************************
 * PROGRAM STATES
 */
enum State {
  EXIT_CART,
  RIGHT_SCAN,
  LEFT_SCAN,
  STRAIGHT,
  STOP
};

State state;


/*********************************************************
 * ENCODER
 * 
 * 22 cm per rev. of wheel
 * 1 wheel rev. = 298 motor rev.
 * 12 pulses per motor rev.
 * 12*298 = 3576 ticks/wheel rev., 3576/22cm ~= 163 pulses/cm
 */
volatile long left_pulses, right_pulses;

void resetEncoder() {
   left_pulses = 0;
   right_pulses = 0;
}

// called for every L encoder change
void L_CHANGE() {
  left_pulses++;
}

// called for every R encoder change
void R_CHANGE() {
  right_pulses++;
}


/*********************************************************
 * LIDAR
 */
int S_t = 0;
double alpha = 0.6;
int start = 1;

// Scanning Vars
int max_dist = 250;
int curr_dist = 0; 
int last_dist = 0; // Keep track of previous measurement to see change
int det_right = 0; // flag to notify detection of ramp
int seen = 0;
unsigned long t = 0;
unsigned long tDelta = 0;
int cap_dist = 0;

int exponentialSmoothing (int dist){
  if(start) {
    S_t = dist;
    start = 0;
  } else{
    S_t = (int)(alpha*dist + (1 - alpha)*S_t);
  }
  return (S_t);  
}

int lidar_distance() {
  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses   
    
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }
  
  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses    
   
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  
  int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  distance = exponentialSmoothing(distance);

  return distance;
}


/*********************************************************
 * MOTOR CONTROLLER
 */
// Motor Controller PINS
int M_Enable = 52;
int MR_Dir = 50;
int ML_PWM = 10;
int ML_Dir = 48;
int MR_PWM = 11;

void setupMotorController() {
  pinMode(M_Enable, OUTPUT);
  pinMode(MR_Dir, OUTPUT);
  pinMode(ML_Dir, OUTPUT);
  pinMode(MR_PWM, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, HIGH);
  digitalWrite(MR_Dir, LOW);
  analogWrite(MR_PWM, 0);
  analogWrite(ML_PWM, 0);
}

// Movement Functions
void stopMoving()
{
  digitalWrite(M_Enable, LOW);
  analogWrite(MR_PWM, 0);
  analogWrite(ML_PWM, 0);
}

void rotateLeft(uint16_t pwm) {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, LOW);
  digitalWrite(MR_Dir, LOW);
  analogWrite(MR_PWM, pwm);
  analogWrite(ML_PWM, pwm);
}

void rotateRight(uint16_t pwm) {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, HIGH);
  digitalWrite(MR_Dir, HIGH);
  analogWrite(MR_PWM, pwm);
  analogWrite(ML_PWM, pwm);
}

// Drive Straight Vars w/ Accel.
int travel_dist = 0;
int rampSpeed = 0;
int maxSpeed = 150;
int startAccel = 15;
int startAccelTime = 0;
int accelDone = 0;

void driveStraight(uint16_t pwm) {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, LOW);
  digitalWrite(MR_Dir, HIGH);
  analogWrite(MR_PWM, pwm);
  analogWrite(ML_PWM, pwm);
}

void updateAccel() {
  if (rampSpeed != maxSpeed && accelDone != 1) {
    Serial.print("updateAccel: ");
    Serial.print(rampSpeed);
    Serial.println("");
    rampSpeed = startAccel*((millis() - startAccelTime)/1000);
    if (rampSpeed > maxSpeed) {
      rampSpeed = maxSpeed;
      accelDone = 1;
    }
    driveStraight(rampSpeed);
  }
}


/*********************************************************
 * PID CONTROLLER
 */
float kP = 2, kI = 0, kD = 0;
int32_t Actual = 0;
int32_t Error = 0;
int32_t SetPt = 0;
int32_t IntThresh = 20;
int32_t ScaleFactor = 1;
int32_t Integral = 0;
int32_t Last = 0;
int32_t Drive = 0;
int P, I, D;

void pid_control () {
  Actual = right_pulses - left_pulses;
   
  Error = SetPt - Actual;
  
  if (abs(Error) < IntThresh){ // prevent integral 'windup'
    Integral = Integral + Error; // accumulate the error integral
  } else {
    Integral = 0; // zero it if out of bounds
  }
  
  P = Error*kP; // proportional term
  I = Integral*kI; // integral term
  D = (Last-Actual)*kD; // derivative term
  
  Drive = P + I + D; // Total drive = P+I+D
  Drive = Drive*ScaleFactor; // scale Drive to be in the range 0-255

  if ((rampSpeed + Drive) > 255) {
    Drive = 255 - rampSpeed;
  } else if ((rampSpeed + Drive) < 0) {
    Drive = -rampSpeed;
  }
  
  analogWrite (MR_PWM, rampSpeed + Drive); // send PWM command to motor board
  Last = Actual; // save current value for next time 
}


/*********************************************************
 * MAIN SETUP
 */
void setup() {
  Serial.begin(9600); //Opens serial connection at 9600bps.     
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  setupMotorController();

  // Setup Interrupts for encoder pulses
  attachInterrupt(0, L_CHANGE, RISING); // Interrupt pin 2
  attachInterrupt(1, R_CHANGE, RISING); // Interrupt pin 3
  
  pinMode(ledPin, OUTPUT); // sets the digital pin 13 as output
  pinMode(inAPin0, INPUT_PULLUP); // sets the digital pin 7 as input
  pinMode(inAPin1, OUTPUT);
  digitalWrite(inAPin1, LOW);
  
  digitalWrite(ledPin, digitalRead(inAPin0));
  while (digitalRead(inAPin0)); // For debugging (press then release SW to activate code)
  delay(1000);
  while (!digitalRead(inAPin0)); // Wait until limit switch is released
  
  delay(500);

  state = EXIT_CART;
  resetEncoder();
  rampSpeed = 30;
  driveStraight(rampSpeed);
}


/*********************************************************
 * MAIN LOOP
 */
void loop() {
  
  switch (state) {
    
    case EXIT_CART:
      // Drive out of cart for 10 cm
      if (left_pulses >= 1630 || right_pulses >= 1630) {
        stopMoving();
        rampSpeed = 25;
        delay(200);
        state = RIGHT_SCAN;
        resetEncoder();
        rotateRight(rampSpeed);
      }
      pid_control();
      break;
      
    
    case RIGHT_SCAN:
      if (right_pulses >= 290 || left_pulses >= 290) {
        stopMoving();
        rampSpeed = 0;
        delay(100);
        state = LEFT_SCAN;
        resetEncoder();
        rotateLeft(20);
      }
      pid_control();
      break;


    case LEFT_SCAN:
      curr_dist = lidar_distance();
      Serial.print("Laser Distance: ");
      Serial.print(curr_dist);
      Serial.println("");
      if (curr_dist >= max_dist) {
        if (seen == 1) {
          if (abs(last_dist - curr_dist) >= 35) {
              seen = 2;
              tDelta = (millis() - t);
              t = millis();
              rotateRight(20);
              while((millis() - t) < ((int)(0.5*tDelta) + (0.1*tDelta))) {
                delay(1);
              }
              stopMoving();
              delay(200);
              state = STRAIGHT;
              resetEncoder();
              driveStraight(rampSpeed);
              startAccelTime = millis();
              break;
            }
        } else {
          if (last_dist == 0)
            last_dist = curr_dist;
  
          if (abs(last_dist - curr_dist) >= 35) {
            cap_dist = curr_dist;
            t = millis();
            seen = 1;
          }
        }
        last_dist = curr_dist;
      }
      break;

      
    case STRAIGHT:
      travel_dist = cap_dist*163;
      if (left_pulses >= travel_dist || right_pulses >= travel_dist) {
        stopMoving();
        delay(200);
        state = STOP;
        resetEncoder();
        while(1);
      } else {
        updateAccel();
        pid_control();
      }
    
    default: 
    break;
  } 
}
