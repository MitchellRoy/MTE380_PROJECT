#include <I2C.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

// Motor Controller PINS
int M_Enable = 52;
int MR_Dir = 50;
int ML_PWM = 10;
int ML_Dir = 48;
int MR_PWM = 11;
int seen = 0;

int S_t = 0;
double alpha = 0.6;
int start = 1;

unsigned long t;
unsigned long tstart;
unsigned long tDelta;

long left_pulses, right_pulses;
long enc_timer = 0;

enum State {
  LASER,
  STRAIGHT,
  STOP
};

State state;


// MotorController Functions
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


// Limit Switch Vars
int ledPin = 13; // LED connected to digital pin 13
int val = 0;
int inAPin0 = A0;
int inAPin1 = A1;


// Encoder Functions
void resetEncoder() {
   left_pulses = 0;
   right_pulses = 0;
   enc_timer = 0;
}

// called for every L encoder change
void L_CHANGE() {
  left_pulses++;
}

// called for every R encoder change
void R_CHANGE() {
  right_pulses++;
}


// Movement Functions
void stopMoving()
{
  digitalWrite(M_Enable, LOW);
  analogWrite(MR_PWM, 0);
  analogWrite(ML_PWM, 0);
}

void rotateLeft() {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, LOW);
  digitalWrite(MR_Dir, LOW);
  analogWrite(MR_PWM, 15);
  analogWrite(ML_PWM, 15);
}

void rotateRightSlow() {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, HIGH);
  digitalWrite(MR_Dir, HIGH);
  analogWrite(MR_PWM, 15);
  analogWrite(ML_PWM, 15);
}

void rotateRight() {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, HIGH);
  digitalWrite(MR_Dir, HIGH);
  analogWrite(MR_PWM, 20);
  analogWrite(ML_PWM, 20);
}

int startSpeed = 240;

void driveStraight() {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, LOW);
  digitalWrite(MR_Dir, HIGH);
  analogWrite(MR_PWM, startSpeed);
  analogWrite(ML_PWM, startSpeed);
}

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
int cap_dist = 0;

void pid_control () {
   Actual = right_pulses - left_pulses;
   
//   Serial.print(startSpeed + Drive);
//   Serial.print("  ");
  Serial.print(left_pulses);
  Serial.print(" ");
  Serial.print(right_pulses);
  Serial.print(" ");
   Serial.print(Actual);
   Serial.print("  ");
   Serial.println("");
   
   Error = SetPt - Actual;
  
   if (abs(Error) < IntThresh){ // prevent integral 'windup'
    Integral = Integral + Error; // accumulate the error integral
   }
   else {
    Integral=0; // zero it if out of bounds
   }
   
   P = Error*kP; // calc proportional term
   I = Integral*kI; // integral term
   D = (Last-Actual)*kD; // derivative term
   
   Drive = P + I + D; // Total drive = P+I+D
   Drive = Drive*ScaleFactor; // scale Drive to be in the range 0-255
   
//     if (Drive < 0){ // Check which direction to go.
//      digitalWrite (Direction,LOW); // change direction as needed
//     }
//     else { // depending on the sign of Error
//      digitalWrite (Direction,HIGH);
//     }
   if ((startSpeed + Drive) > 255) {
      Drive = 255 - startSpeed;
   } else if ((startSpeed + Drive) < 0) {
      Drive = -startSpeed;
   }
   
   analogWrite (MR_PWM, startSpeed + Drive); // send PWM command to motor board
   Last = Actual; // save current value for next time 
}


// Laser Sensor Functions
int exponentialSmoothing (int dist){
  if(start){
    S_t = dist;
    start = 0;
  } else{
    S_t = (int)(alpha*dist + (1 - alpha)*S_t);
  }
  return (S_t);  
}


// Setup Function
void setup() {
  Serial.begin(9600); //Opens serial connection at 9600bps.     
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  setupMotorController();
  
  pinMode(ledPin, OUTPUT);      // sets the digital pin 13 as output
  pinMode(inAPin0, INPUT_PULLUP);      // sets the digital pin 7 as input
  pinMode(inAPin1, OUTPUT);
  
  resetEncoder();
  
  // Setup Interrupts for encoder pulses
  attachInterrupt(0, L_CHANGE, RISING); // Interrupt pin 2
  attachInterrupt(1, R_CHANGE, RISING); // Interrupt pin 3

  // Wait for 3 Seconds
  delay(3000);

  // Rotate right for fixed angle
  rotateRight();

  while (left_pulses < 50) {
    Serial.print(left_pulses);
    Serial.print(right_pulses);
    Serial.println();
  };
  stopMoving();

  // After as 2 second delay, rotate left
  delay(2000);
  resetEncoder();
  rotateLeft();

  state =  LASER;
  
  tstart = millis();
}





void loop() {
//  val = digitalRead(inAPin1);   // read the input pin
digitalWrite(ledPin, HIGH);    // sets the LED to the button's value
//  digitalWrite(inAPin1, LOW); 
  
  switch (state) {
    case LASER:
      while (left_pulses < 150 || seen == 1) {
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

        if (seen == 1) {
          if (distance > 265) {
            Serial.println("Target detected the second time");
            seen = 2;
            tDelta = (millis() - t);
            t = millis();
            rotateRightSlow();
            while((millis() - t) < ((int)(0.5*tDelta) + (0.1*tDelta))) {
              delay(1);
            }
            stopMoving();
            break;
          }
        } else {
          Serial.println("Constant loop ");
          if (distance < 250) {
            Serial.println("Target detected the first time");
            cap_dist = distance;
            t = millis();
            seen = 1;
          }
        }
      };
      Serial.print("Change state ");
      stopMoving();
      delay(2000);
      resetEncoder();
      Serial.println("move straight");
      state = STRAIGHT;
      driveStraight();

      
    case STRAIGHT:
//    Serial.print(cap_dist);
//    Serial.print("   ");
//    Serial.print(left_pulses);
//    Serial.print("   ");
//    Serial.print(right_pulses);
//    Serial.println("");
      if (left_pulses > 7500) { //(cap_dist - 5)*50
        stopMoving();
        resetEncoder();
        state = STOP;
        while(1);
      } else {
        pid_control();
      }
    
    default: 
    break;
  } 
}
