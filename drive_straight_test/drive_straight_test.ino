// Motor Controller PINS
int M_Enable = 52;
int MR_Dir = 50;
int ML_PWM = 10;
int ML_Dir = 48;
int MR_PWM = 11;

long left_pulses, right_pulses;
long enc_timer = 0;

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

int startSpeed = 240;

void rotateLeft() {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, LOW);
  digitalWrite(MR_Dir, LOW);
  analogWrite(MR_PWM, startSpeed);
  analogWrite(ML_PWM, startSpeed);
}

void rotateRightSlow() {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, HIGH);
  digitalWrite(MR_Dir, HIGH);
  analogWrite(MR_PWM, startSpeed);
  analogWrite(ML_PWM, startSpeed);
}

void rotateRight() {
  digitalWrite(M_Enable, HIGH);
  digitalWrite(ML_Dir, HIGH);
  digitalWrite(MR_Dir, HIGH);
  analogWrite(MR_PWM, startSpeed);
  analogWrite(ML_PWM, startSpeed);
}

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







// Setup Function
void setup() {
  Serial.begin(9600); //Opens serial connection at 9600bps.     
  
  setupMotorController();
  
  // Setup Interrupts for encoder pulses
  attachInterrupt(0, L_CHANGE, CHANGE); // Interrupt pin 2
  attachInterrupt(1, R_CHANGE, CHANGE); // Interrupt pin 3

  delay(2000);
  driveStraight();
  resetEncoder();
}


void loop() {
  if (millis() < 10000) {
    pid_control();
  } else {
    stopMoving();
    while(1);
  }

  Serial.print("Left: ");
  Serial.print(left_pulses);
  Serial.print("  Right: ");
  Serial.print(right_pulses);
  Serial.print("  PError: ");
  Serial.print(Actual);
  Serial.println();
}
