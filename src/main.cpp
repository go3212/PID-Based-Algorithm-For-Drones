#include <Arduino.h>
#include <Wire.h>

#define F_CPU 16000000L
#define IMU_POWER_REGISTRY 0x6B
#define GYROSCOPE_CONFIGURATION_REGISTRY 0x1B
#define ACCELEROMETER_CONFIGURATION_REGISTRY 0x1C
#define DATA_SEQUENCE_FIRST_BYTE 0x3B //note: there are 14 bytes in total

//DECLARING FUNCTIONS
void gyroscope_gather_data();

//MAIN VARIABLES
unsigned long timer[1];

boolean start = true;

float raw_gyro_pitch_calibration, raw_gyro_roll_calibration, raw_gyro_yaw_calibration;
int raw_gyro_pitch, raw_gyro_roll, raw_gyro_yaw;
int raw_acc_pitch, raw_acc_roll, raw_acc_yaw;
int raw_temperature;

float angular_pitch_velocity, angular_roll_velocity, angular_yaw_velocity;
float angle_pitch, angle_roll, angle_yaw;
float angle_pitch_acceleration, angle_roll_acceleration, angle_yaw_acceleration;

float PITCH_SET_ANGLE, ROLL_SET_ANGLE, YAW_SET_ANGLE;

float pitch_error, roll_error, yaw_error;
float p_pitch_error, p_roll_error, p_yaw_error;
float PID_PITCH, PID_ROLL, PID_YAW;
float i_pitch, i_roll, i_yaw;

float pitch_proportional_gain, roll_proportional_gain, yaw_proportional_gain;
float pitch_integral_gain, roll_integral_gain, yaw_integral_gain;
float pitch_derivative_gain, roll_derivative_gain, yaw_derivative_gain;

unsigned long esc_1, esc_2, esc_3, esc_4;
unsigned long esc_timer_1, esc_timer_2, esc_timer_3, esc_timer_4, esc_timer_def;
unsigned long timer_phase;

//CONTROLLER VARIABLES
int CHANNEL1, CHANNEL2, CHANNEL3, CHANNEL4, CHANNEL5, CHANNEL6;
int CHANNEL_STATE_1, CHANNEL_STATE_2, CHANNEL_STATE_3, CHANNEL_STATE_4, CHANNEL_STATE_5, CHANNEL_STATE_6;
unsigned long ICRR1, ICRR2, ICRR3, ICRR4, ICRR5, ICRR6;
unsigned long ISR_REFRESH_RATE;

int landing;
float i_landing, e_landing, p_landing;
float landing_proportional_gain, landing_integral_gain, landing_derivative_gain;
float landing_acceleration;

void setup() {
  Serial.begin(2000000);

  Wire.begin();
  TWBR = 12;   //TWBR = ((F_CPU / frequency) - 16) / 2;
  
  PCICR = 0b00000001;
  PCMSK0 = 0b00011111;

  DDRD |= B11110000;

  for (int i = 0; i < 1250 ; i++){                      
    PORTD |= B11110000;                                         
    delayMicroseconds(1000);                                            
    PORTD &= B00001111;                                                  
    delayMicroseconds(3000);                                               
  }

  //The next chain of code tweaks the MPU6050 parameters to fit with everything else
  Wire.beginTransmission(0x68);
  Wire.write(IMU_POWER_REGISTRY);
  Wire.write(0b00000000); //8Mhz Oscillation Clock
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(GYROSCOPE_CONFIGURATION_REGISTRY);
  Wire.write(0b00001000); //FS_SEL = 1 +-500º/s
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(ACCELEROMETER_CONFIGURATION_REGISTRY);
  Wire.write(0b00010000); //AFS_SEL = 2 +- 8g
  Wire.endTransmission();

  PCICR |= (1 << PCIE0);                                      
  PCMSK0 |= (1 << PCINT0);                            
  PCMSK0 |= (1 << PCINT1);                             
  PCMSK0 |= (1 << PCINT2);                      
  PCMSK0 |= (1 << PCINT3); 
  PCMSK0 |= (1 << PCINT4);   

  //The next lines will calibrate the gyroscope drift.
  for(int i = 0; i < 2000; i++){
    gyroscope_gather_data();
    raw_gyro_pitch_calibration += raw_gyro_pitch;
    raw_gyro_roll_calibration += raw_gyro_roll;
    raw_gyro_yaw_calibration += raw_gyro_yaw;

    delayMicroseconds(4000);
  }

  raw_gyro_pitch_calibration /= 2000;
  raw_gyro_roll_calibration /= 2000;
  raw_gyro_yaw_calibration /= 2000;

  //////////////////////////////
  //PID VALUES
  pitch_proportional_gain = 3.4;
  pitch_integral_gain = 0.006;
  pitch_derivative_gain = 18;
  
  roll_proportional_gain = pitch_proportional_gain;
  roll_integral_gain = pitch_integral_gain;
  roll_derivative_gain = pitch_derivative_gain;

  yaw_proportional_gain = 3;
  yaw_integral_gain = 0.01;
  yaw_derivative_gain = 0;
/* WIP
  landing_proportional_gain = 0;
  landing_integral_gain = 0;
  landing_derivative_gain = 0;

  landing_acceleration = 0;
*/
}

void loop() {
  timer[0] = micros();

  gyroscope_gather_data();

  if(start == true){
      angle_pitch = atan(raw_acc_roll/sqrt(pow(raw_acc_pitch,2)+pow(raw_acc_yaw,2)))*57.296;
      angle_roll = atan(raw_acc_pitch/sqrt(pow(raw_acc_roll,2)+pow(raw_acc_yaw,2)))*-57.296;

      start = false;
  }

  //////////////////////////////////
  //RIGID BODY PROPERTIES AND STATUS
  raw_gyro_pitch -= raw_gyro_pitch_calibration;
  raw_gyro_roll -= raw_gyro_roll_calibration;
  raw_gyro_yaw -= raw_gyro_yaw_calibration;

  angular_pitch_velocity = angular_pitch_velocity * 0.7 + (raw_gyro_pitch / 65.5)* 0.3;   //raw_gyro/FS_SEL=2 -> raw_value/65.5 -> degrees/second (º/s)
  angular_roll_velocity = angular_roll_velocity * 0.7 + (raw_gyro_roll / 65.5) * 0.3;
  angular_yaw_velocity = angular_yaw_velocity * 0.7 + (raw_gyro_yaw / 65.5) * 0.3;   

  angle_pitch += angular_pitch_velocity * 0.004;  //Integration ∫wt = += w*t -> angular_velocity*cycle_time
  angle_roll += angular_roll_velocity * 0.004;
  angle_yaw += angular_yaw_velocity * 0.004;

  //LINEAR TRANSFORMATIONS
  angle_pitch -= (-angle_roll)*sin(angular_yaw_velocity*0.000069813); //the sin function is in radians 
  angle_roll += (-angle_pitch)*sin(angular_yaw_velocity*0.000069813);

  angle_pitch_acceleration = atan(raw_acc_roll/sqrt(pow(raw_acc_pitch,2)+pow(raw_acc_yaw,2)))*57.296;
  angle_roll_acceleration = atan(raw_acc_pitch/sqrt(pow(raw_acc_roll,2)+pow(raw_acc_yaw,2)))*-57.296;

  //LOW PASS FILTERING THE ANGLES AND THE ANGULAR VELOCITY
  angle_pitch = angle_pitch*0.9996 + angle_pitch_acceleration*0.0004;
  angle_roll = angle_roll*0.9996 + angle_roll_acceleration*0.0004;

  //////////////////////////////////
  //CHANNEL DATA FILTER
  PITCH_SET_ANGLE = 0;
  ROLL_SET_ANGLE = 0;
  YAW_SET_ANGLE = 0;

  if(CHANNEL1 > 1540) PITCH_SET_ANGLE = (CHANNEL1 - 1540)/4;
  else if(CHANNEL1 < 1460) PITCH_SET_ANGLE = (CHANNEL1 - 1460)/4;

  if(CHANNEL2 > 1540) ROLL_SET_ANGLE = (CHANNEL2 - 1540)/4;
  else if(CHANNEL2 < 1460) ROLL_SET_ANGLE = (CHANNEL2 - 1460)/4; 

  if(CHANNEL4 > 1540) YAW_SET_ANGLE = (CHANNEL4 - 1540)/4;
  else if(CHANNEL4 < 1460) YAW_SET_ANGLE = (CHANNEL4 - 1460)/4;

  PITCH_SET_ANGLE -= (angle_pitch*6);
  ROLL_SET_ANGLE -= (angle_roll*6);

  //////////////////////////////////
  //PID CONTROLLER
  pitch_error = (angular_pitch_velocity - PITCH_SET_ANGLE);

  i_pitch += pitch_error * pitch_integral_gain;

  PID_PITCH = pitch_error * pitch_proportional_gain + i_pitch + (pitch_error - p_pitch_error) * pitch_derivative_gain;

  p_pitch_error = pitch_error;

  roll_error = (angular_roll_velocity - ROLL_SET_ANGLE);

  i_roll += roll_error * roll_integral_gain;

  PID_ROLL = roll_error * roll_proportional_gain + i_roll + (roll_error - p_roll_error) * roll_derivative_gain;

  p_roll_error = roll_error;

  yaw_error = (angular_yaw_velocity - YAW_SET_ANGLE);

  i_yaw += yaw_error * yaw_integral_gain;

  PID_YAW = yaw_error * yaw_proportional_gain + i_yaw + (yaw_error - p_yaw_error) * yaw_derivative_gain;

  p_yaw_error = yaw_error;

  /////////////////
  //PID CORRECTIONS
  if(PID_ROLL >= 350) PID_ROLL = 350;
  else if(PID_ROLL <= -350) PID_ROLL = -350;
  if(PID_PITCH >= 350) PID_PITCH = 350;
  else if(PID_PITCH <= -350) PID_PITCH = -350;
  if(PID_YAW >= 350 ) PID_YAW = 400;
  else if(PID_YAW <= -350 ) PID_YAW = -350;  

  //////////////////
  //ESC PULSE LENGHT
  esc_1 = CHANNEL3 - PID_PITCH - PID_ROLL - PID_YAW;
  esc_2 = CHANNEL3 - PID_PITCH + PID_ROLL + PID_YAW;
  esc_3 = CHANNEL3 + PID_PITCH + PID_ROLL - PID_YAW;
  esc_4 = CHANNEL3 + PID_PITCH - PID_ROLL + PID_YAW;

  ///////////////////
  //PULSE CORRECTION
  if(esc_1 <= 1088) esc_1 = 1088;
  else if(esc_1 >= 2000) esc_1 = 2000;

  if(esc_2 <= 1088) esc_2 = 1088;
  else if(esc_2 >= 2000) esc_2 = 2000;

  if(esc_3 <= 1088) esc_3 = 1088;
  else if(esc_3 >= 2000) esc_3 = 2000;

  if(esc_4 <= 1088) esc_4 = 1088; 
  else if(esc_4 >= 2000) esc_4 = 2000;

  //////////////////////////
  /*LANDING PID CALCULATIONS (WIP)
  e_landing = (sqrt(pow(raw_acc_pitch, 2) + pow(raw_acc_roll, 2) + pow(raw_acc_yaw, 2))*0.00239 - 9);
  i_landing += e_landing * landing_integral_gain;
  landing = e_landing * landing_proportional_gain + i_landing + (e_landing - p_landing) * landing_derivative_gain;
  p_landing = e_landing;
  */
  /////////////
  //SEND PULSES
  timer_phase = micros();
  PORTD |= B11110000;  
  
  if(CHANNEL5 < 1100){
    esc_timer_def = 1000 + timer_phase;
    while(PORTD >= 16){
      timer_phase = micros();
      if(esc_timer_def <= timer_phase) PORTD &= B00001111;
    }
  }else{
    esc_timer_1 = esc_1 + timer_phase;
    esc_timer_2 = esc_2 + timer_phase;
    esc_timer_3 = esc_3 + timer_phase;
    esc_timer_4 = esc_4 + timer_phase;
    while(PORTD >= 16){
      timer_phase = micros();
      if(esc_timer_1 <= timer_phase) PORTD &= B11101111;
      if(esc_timer_2 <= timer_phase) PORTD &= B11011111;
      if(esc_timer_3 <= timer_phase) PORTD &= B10111111;
      if(esc_timer_4 <= timer_phase) PORTD &= B01111111;
    }
  }

  timer[1] = micros();
  while(timer[1] - timer[0] <= 4000) timer[1] = micros();
}

void gyroscope_gather_data(){
  Wire.beginTransmission(0x68);
  Wire.write(DATA_SEQUENCE_FIRST_BYTE);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 14);

  if(Wire.available() <= 14){
    raw_acc_pitch = Wire.read() << 8 | Wire.read();
    raw_acc_roll = Wire.read() << 8 | Wire.read();
    raw_acc_yaw = Wire.read() << 8 | Wire.read();

    raw_temperature = Wire.read() << 8 | Wire.read();

    raw_gyro_pitch = Wire.read() << 8 | Wire.read();
    raw_gyro_roll = Wire.read() << 8 | Wire.read();
    raw_gyro_yaw = Wire.read() << 8 | Wire.read();
  }
}

ISR(PCINT0_vect){
  ISR_REFRESH_RATE = micros();
  //CHANNEL 1
  if(PINB & B00000001){
    if(CHANNEL_STATE_1 == 0){
      CHANNEL_STATE_1 = 1;
      ICRR1 = ISR_REFRESH_RATE;
    }
  }else if(CHANNEL_STATE_1 == 1){
     CHANNEL_STATE_1 = 0;
     CHANNEL1 = ISR_REFRESH_RATE - ICRR1;
  }
  //CHANNEL 2
  if(PINB & B00000010){
    if(CHANNEL_STATE_2 == 0){
      CHANNEL_STATE_2 = 1;
      ICRR2 = ISR_REFRESH_RATE;
    }
  }else if(CHANNEL_STATE_2 == 1){
    CHANNEL_STATE_2 = 0;
    CHANNEL2 = ISR_REFRESH_RATE - ICRR2;
  }
  //CHANNEL 3
  if(PINB & B00000100){
    if(CHANNEL_STATE_3 == 0){
      CHANNEL_STATE_3 = 1;
      ICRR3 = ISR_REFRESH_RATE;
    }
  }else if(CHANNEL_STATE_3 == 1){
    CHANNEL_STATE_3 = 0;
    CHANNEL3 = ISR_REFRESH_RATE - ICRR3;
  }
  //CHANNEL 4
  if(PINB & B00001000){
    if(CHANNEL_STATE_4 == 0){
      CHANNEL_STATE_4 = 1;
      ICRR4 = ISR_REFRESH_RATE;
    }
  }else if(CHANNEL_STATE_4 == 1){
    CHANNEL_STATE_4 = 0;
    CHANNEL4 = ISR_REFRESH_RATE - ICRR4;
  }
  //CHANNEL 5
  if(PINB & B00010000){
    if(CHANNEL_STATE_5 == 0){
      CHANNEL_STATE_5 = 1;
      ICRR5 = ISR_REFRESH_RATE;
    }
  }else if(CHANNEL_STATE_5 == 1){
    CHANNEL_STATE_5 = 0;
    CHANNEL5 = ISR_REFRESH_RATE - ICRR5;
  }
}

