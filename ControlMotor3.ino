#include <util/atomic.h>

// Pins
#define SENSOR_PIN 2
#define PWM 6
#define IN2 7

// Use the "volatile" directive for variables
// used in an interrupt
volatile float velocity_interrupt = 0;
volatile long previousTime = 0;

//Ref increase
long timeToIncrease = 20000; //ms
long previous_t_increase = 0;

//Control parameters

//float Kp = 4.253;
//float Ki = 3.838;
//float Kd = 0.0471;
//int N = 100;
//float Cf = 0.01;
float Kp = 5.253;
float Ki = 3.838;
float Kd = 0.0971;
int N = 100;
float  radius = 0.58;
float maxDeltaTime = 60;//[ms]

long startControlTime = 500;//[ms]
long sampleTime = 10;
float Ref = 0.4;//cm/s

//Control variables
float linearVelocity = 0;
unsigned int pwmDuty = 0;
float directCmd = 6;
float CmdPID = 0.0;
float Cmd = 0.0;
float CmdLim = 0.0;
float KiLim = Ki;
float E = 0;
float Ep = 0;
float CmdIp = 0;
float CmdDp = 0;



//Filter
#define WINDOW_SIZE 100
long INDEX = 0;
float VALUE = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float velocityFilter = 0;

//Conversion
float pulses2Rad = (2*PI) / 65.0;
float voltage2PWM = 255.0 / 12;
float deg2Rad = PI / 180;

void setup() {
  
  Serial.begin(115200);

  pinMode(SENSOR_PIN, INPUT);
  pinMode(PWM, OUTPUT);
  //pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN),
                  readEncoder, RISING);
  CmdLim = directCmd;
}

void loop() {
  //digitalWrite(IN2,HIGH);
  // read the position in an atomic block
  // to avoid potential misreads
  float velocityPulses = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    velocityPulses = velocity_interrupt;
  }

  if((micros() - previousTime) >= maxDeltaTime * 1000){
    velocityPulses = 0;
  }

  linearVelocity = velocityPulses * pulses2Rad * radius;

  filter();
  if (millis() >= startControlTime) {
    control();
//    if ((millis() - previous_t_increase) >= timeToIncrease && Ref < 140) {
//      Ref += 10;
//      previous_t_increase = millis();
//    }
  }

  float pwmDuty = CmdLim * voltage2PWM;

  analogWrite(PWM, pwmDuty);
  //Serial.print("Referencia: ");
  Serial.print(Ref);
  Serial.print(" ");
  //Serial.print(changePercentage * 100);
  //Serial.println(" ");
  //Serial.print(velocityDeg);
  //Serial.print(',');
  Serial.println(velocityFilter);
  //Serial.print(CmdPID);
  //Serial.print(" ");
  //Serial.println(Ref - velocityFilter);
  delay(sampleTime);
}

void filter() {
  SUM = SUM - READINGS[INDEX];  // Remove the oldest entry from the sum and add the newest reading
  VALUE = linearVelocity;   // Read the next sensor value as a float
  READINGS[INDEX] = VALUE;              // Add the newest reading to the window
  SUM = SUM + VALUE;                    // Add the newest reading to the sum
  INDEX = (INDEX + 1) % WINDOW_SIZE;    // Increment the index, and wrap to 0 if it exceeds the window size
  velocityFilter = SUM / (float)WINDOW_SIZE;  // Divide the sum of the window by the window size for the result
}
void control() {
  E = (Ref - velocityFilter) / radius;
  //Tustin
  float CmdP = Kp * E;
  float CmdI = (KiLim * sampleTime * 0.5 * 1e-3) * (E + Ep) + CmdIp;
  float CmdD = ((Kd * N) * (E - Ep) + (1 - N * sampleTime * 0.5 * 1e-3) * CmdDp) / (1 + N * sampleTime * 0.5 * 1e-3);

  //  float CmdP = Kp * E;
  //  float CmdI = (KiLim * sampleTime * 1e-3) * E + CmdIp;
  //  float CmdD = (Kd  * (E - Ep) + CmdDp * (Cf)) / (Cf + sampleTime * 1e-3);

  //Euler Forward
  //      double CmdP = Kp * E;
  //      double CmdI = (Ki * (Ts * 1e-3)*Ep) + CmdIp;
  //      double CmdD = Kd * N * (E - Ep) + CmdDp * (1 - (N * Ts * 1e-3));


  CmdPID = CmdP + CmdI + CmdD;

  CmdIp = CmdI;
  CmdDp = CmdD;
  Ep = E;

  //Calculate control signal
  Cmd = directCmd + CmdPID;
  CmdLim = min(max(Cmd, 5), 12); // Saturated Control Output

  //Anti-Windup
  if (CmdLim <= 0 || CmdLim >= 12) {
    KiLim = 0;
  } else {
    KiLim = Ki;
  }
}


void readEncoder() {
  // Compute velocity with method 2
  long currentTime = micros();
  float deltaTime = ((float) (currentTime - previousTime)) * 1.0e-6;
  velocity_interrupt = 1 / deltaTime;
  previousTime = currentTime;
}
