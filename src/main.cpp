#include <Arduino.h>
#include <Wire.h>

#define F_CPU 16000000L
#define RWPOWER 0x6B
#define GYRO_CONFIG 0x1B
#define ACC_CONFIG 0x1C
#define BitHIGH_Start 0x3B

////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS DECLARATION
////////////////////////////////////////////////////////////////////////////////////
void giroscopio_salida();

////////////////////////////////////////////////////////////////////////////////////
// VARIABLES
////////////////////////////////////////////////////////////////////////////////////
//GLOBALES//
unsigned long timer[2];

boolean start = false;

//GIROSCOPIO//
int MPU6050 = 0x68;
float acceleration[4];
float gyroscope[4];
float temperature;
long gyroscope_calibracion[4];
float modulo_vector_aceleracion;
float giroscopio_angular[4];

float PID_GYRO[4];

//VARIABLES INFLUENCIADAS POR EL TIEMPO//
float angle[4];
float angle_acceleration[4];
float PID_ANGLE[4];

//CANALES//
unsigned long ISR_REFRESH_RATE;                   //TIEMPO ENTRE PULSOS
int CH1, CH2, CH3, CH4;                           //CHANNEL 1, 2, 3, 4
int IA1, IA2, IA3, IA4;                           //INPUT ANTERIOR
unsigned long ICRR1, ICRR2, ICRR3, ICRR4;         //ISR_CHANNEL_REFRESH_RATE

//PID//
float DPS_ROLL;                                   //ALMACENA LOS VALORES EN GRADOS POR SEGUNDO EN ROLL DETERMINADOS POR EL PULSO DEL MANDO
float DPS_YAW;                                    //ALMACENA LOS VALORES EN GRADOS POR SEGUNDO EN YAW DETERMINADOS POR EL PULSO DEL MANDO
float DPS_PITCH;                                  //ALMACENA LOS VALORES EN GRADOS POR SEGUNDO EN PITCH DETERMINADOS POR EL PULSO DEL MANDO
float pitch_proporcional;
float pitch_integral;
float pitch_integral_temp;
float pitch_derivada;
float pitch_ganancia_proporcional = 2.25;
float pitch_ganancia_integral = 0.02;
float pitch_ganancia_derivada = 15;
float DPS_PITCH_TEMP;
float gyro_x_temp;
float PID_PITCH;
float roll_proporcional;
float roll_integral;
float roll_integral_temp;
float roll_derivada;
float roll_ganancia_proporcional = pitch_ganancia_proporcional;
float roll_ganancia_integral = pitch_ganancia_integral;
float roll_ganancia_derivada = pitch_ganancia_derivada;
float DPS_ROLL_TEMP;
float gyro_y_temp;
float PID_ROLL;
float yaw_proporcional;
float yaw_integral;
float yaw_integral_temp;
float yaw_derivada;
float yaw_ganancia_proporcional = 3;
float yaw_ganancia_integral = 0.01;
float yaw_ganancia_derivada = 0;
float DPS_YAW_TEMP;
float gyro_z_temp;
float PID_YAW;

//ESC's//
unsigned long esc[5];
unsigned long esc_timer[5];
unsigned long fase_timer;

void setup() {
  ////////////////////////////////////////////////////////////////////////////////////
  // ISR HABILITAR
  ////////////////////////////////////////////////////////////////////////////////////
  PCICR = 0b00000001;   //ESTE REGISTRO, HABILITA EL Pin Change Interrupt Control
  PCMSK0 = 0b00001111;  //ESTE REGISTRO, PONE EN MODO Pin Change Interrupt Control LOS PINES 11-8, ESTO SIGNIFICA QUE CADA VEZ QUE HAYA UN CAMBIO EN ESTOS PINES, OTRA PARTE DEL CODIGO SE EJECUTARA AUTOMATICAMENTE
  
  ////////////////////////////////////////////////////////////////////////////////////
  // PINES
  ////////////////////////////////////////////////////////////////////////////////////
  DDRD |= B11110000;

  ////////////////////////////////////////////////////////////////////////////////////
  // RUTINA PRINCIPAL
  ////////////////////////////////////////////////////////////////////////////////////
  Wire.begin();
  TWBR = 12;

  ////////////////////////////////////////////////////////////////////////////////////
  // GIROSCOPIO
  ////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(MPU6050);       //HABILITA CONEXION EN EL BUS I2C AL ESCLAVO 0x68
  Wire.write(RWPOWER);                   //ESCRIBE EN EL REGISTRO DE "POWER0"
  Wire.write(0b00000000);                //ESTABLECE SUS PARAMETROS
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050);
  Wire.write(GYRO_CONFIG);               //ESCRIBE EN EL REGISTRO DE "GYRO_CONFIG"
  Wire.write(0b00001000);                //ESTABLECE SUS PARAMETROS
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050);
  Wire.write(ACC_CONFIG);                //ESCRIBE EN EL REGISTRO DE "ACC_CONFIG"
  Wire.write(0b00000000);                //ESTABLECE SUS PARAMETROS
  Wire.endTransmission();                //PARA LA TRANSMISION Y CONFIGURACION

  for(int i = 0; i <= 2000; i++){
    giroscopio_salida();                        //RECLAMA TODAS LOS VALORES ACTUALES DEL GIROSCOPIO
    gyroscope_calibracion[1] += gyroscope[1];   //SUMA TODAS LAS OSCILACIONES EN LA VELOCIDAD ANGULAR X
    gyroscope_calibracion[2] += gyroscope[2];   //SUMA TODAS LAS OSCILACIONES EN LA VELOCIDAD ANGULAR Y
    gyroscope_calibracion[3] += gyroscope[3];   //SUMA TODAS LAS OSCILACIONES EN LA VELOCIDAD ANGULAR Z

    delayMicroseconds(4000);                    //SIMULA LA VELOCIDAD DE ACTUALIZACION DEL LOOP PRINCIPAL
  }
  gyroscope_calibracion[1] /= 2000;      //DIVIDE EL SUMATORIO DE LAS OSCILACIONES X POR EL NUMERO DE CICLOS "for"
  gyroscope_calibracion[2] /= 2000;      //DIVIDE EL SUMATORIO DE LAS OSCILACIONES Y POR EL NUMERO DE CICLOS "for"
  gyroscope_calibracion[3] /= 2000;      //DIVIDE EL SUMATORIO DE LAS OSCILACIONES Z POR EL NUMERO DE CICLOS "for"

  ////////////////////////////////////////////////////////////////////////////////////
  // ESC's
  ////////////////////////////////////////////////////////////////////////////////////
  PORTD |= B11110000;                      //PONEMOS LOS PINES 4-7 EN MODO HIGH (+5V)
  delayMicroseconds(2000);                //ESPERAMOS 2000us PARA ESTABLECER EL RANGO MAXIMO (2000us)
  PORTD &= B00000000;                     //PONEMOS LOS PINES 4-7 EN MODO LOW (+5V)

  PORTD |= B11110000;                     //PONEMOS LOS PINES 4-7 EN MODO HIGH (+5V)
  delayMicroseconds(1500);                //ESPERAMOS 1500us PARA ESTABLECER EL RANGO MEDIO (1500us)
  PORTD &= B00000000;                     //PONEMOS LOS PINES 4-7 EN MODO LOW (+5V)

  PORTD |= B11110000;                     //PONEMOS LOS PINES 4-7 EN MODO HIGH (+5V)
  delayMicroseconds(1000);                //ESPERAMOS 1000us PARA ESTABLECER EL RANGO MINIMO (1000us)
  PORTD &= B00000000;                     //PONEMOS LOS PINES 4-7 EN MODO LOW (+5V)

  //Serial.begin(2000000);
}

void loop() {
  timer[1] = micros();

  if(start != true){
    giroscopio_salida();
    modulo_vector_aceleracion = sqrt(acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]+acceleration[3]*acceleration[3]);
    angle_acceleration[1] = asin(acceleration[2]/modulo_vector_aceleracion)*57.296;
    angle_acceleration[2] = asin(acceleration[1]/modulo_vector_aceleracion)*-57.296;
    angle[1] = angle_acceleration[1];
    angle[2] = angle_acceleration[2];

    start = true;
  }

  ////////////////////////////////////////////////////////////////////////////////////
  // CORRECCIONES Y TRANSFORMACIONES GIROSCOPIO
  ////////////////////////////////////////////////////////////////////////////////////
  giroscopio_salida();                    //RECLAMA LOS VALORES DEL GIROSCOPIO ACTUALES
  giroscopio_angular[1] = (gyroscope[1] - gyroscope_calibracion[1])/65.5;   //LE RESTA LA CALIBRACION PARA OBTENER UN VALOR MAS PRECISO EN EL TIEMPO
  giroscopio_angular[2] = (gyroscope[2] - gyroscope_calibracion[2])/65.5;   //LE RESTA LA CALIBRACION PARA OBTENER UN VALOR MAS PRECISO EN EL TIEMPO
  giroscopio_angular[3] = (gyroscope[3] - gyroscope_calibracion[3])/65.5;   //LE RESTA LA CALIBRACION PARA OBTENER UN VALOR MAS PRECISO EN EL TIEMPO

  PID_GYRO[1] = PID_GYRO[1]*0.7 + giroscopio_angular[1]*0.3;
  PID_GYRO[2] = PID_GYRO[2]*0.7 + giroscopio_angular[2]*0.3;
  PID_GYRO[3] = PID_GYRO[3]*0.7 + giroscopio_angular[3]*0.3;

  ////////////////////////////////////////////////////////////////////////////////////
  // CALCULOS V2 DRONE
  ////////////////////////////////////////////////////////////////////////////////////
  angle[1] += (gyroscope[1] - gyroscope_calibracion[1])*(0.0000611);    //PITCH
  angle[2] += (gyroscope[2] - gyroscope_calibracion[2])*(0.0000611);    //ROLL

  angle[1] -= angle[2]*sin((gyroscope[3]-gyroscope_calibracion[3])*0.000001066);
  angle[2] += angle[1]*sin((gyroscope[3]-gyroscope_calibracion[3])*0.000001066);

  modulo_vector_aceleracion = sqrt(acceleration[1]*acceleration[1]+acceleration[2]*acceleration[2]+acceleration[3]*acceleration[3]);

  angle_acceleration[1] = asin(acceleration[2]/modulo_vector_aceleracion)*57.296;
  angle_acceleration[2] = asin(acceleration[1]/modulo_vector_aceleracion)*-57.296;

  angle[1] = angle[1]*0.999 + angle_acceleration[1]*0.001;
  angle[2] = angle[2]*0.999 + angle_acceleration[2]*0.001;

  PID_ANGLE[1] = angle[2]*15;
  PID_ANGLE[2] = angle[1]*15;
  ////////////////////////////////////////////////////////////////////////////////////
  // CORRECCIONES CANALES Y CONVERSION A GRADOS POR SEGUNDO
  ////////////////////////////////////////////////////////////////////////////////////
  //DPS_PITCH//
  if(CH2 > 1540){                                 //INICIA LA CONDICION DETERMINANDO LOS VALORES POSITIVOS
      DPS_PITCH = (CH2 - 1540)/6;
  } else if(CH2 < 1460){                          //SI SON INFERIORES,  SERAN NEGATIVOS
      DPS_PITCH = (CH2 - 1460)/6;
  } else {                                        //PARA PREVENIR FALLOS DE CORRECCION DEBIDO A LA FRECUENCIA, SI NO
    //DPS_PITCH = 0;                                //SE ENCUENTRA EN EL RANGO DE MOVIMIENTO, QUE SEA CERO.
  }
  ////////////
  //DPS_ROLL//
  if(CH1 > 1540){                                 //INICIA LA CONDICION DETERMINANDO LOS VALORES POSITIVOS
      DPS_ROLL = (CH1 - 1540)/6;
  } else if(CH1 < 1460){                          //SI SON INFERIORES,  SERAN NEGATIVOS
      DPS_ROLL = (CH1 - 1460)/6;
  } else {                                        //PARA PREVENIR FALLOS DE CORRECCION DEBIDO A LA FRECUENCIA, SI NO
    //DPS_ROLL = 0;                                 //SE ENCUENTRA EN EL RANGO DE MOVIMIENTO, QUE SEA CERO.
  }
  ///////////
  //DPS_YAW//
    if(CH4 > 1540){                               //INICIA LA CONDICION DETERMINANDO LOS VALORES POSITIVOS
      DPS_YAW = (CH4 - 1540)/6;
  } else if(CH4 < 1460){                          //SI SON INFERIORES,  SERAN NEGATIVOS
      DPS_YAW = (CH4 - 1460)/6;
  } else {                                        //PARA PREVENIR FALLOS DE CORRECCION DEBIDO A LA FRECUENCIA, SI NO
    //DPS_YAW = 0;                                  //SE ENCUENTRA EN EL RANGO DE MOVIMIENTO, QUE SEA CERO.
  }
  /////////////////////
  //CORRECCION THRUST//
  if (CH3 >= 2000) CH3 = 2000;
  if (CH3 <= 1000) CH3 = 1000;

  ///////////////////////
  //AJUSTES VERSION 2.0//
  DPS_ROLL -= (PID_ANGLE[1]/10);
  DPS_PITCH -= (PID_ANGLE[2]/10);

  ////////////////////////////////////////////////////////////////////////////////////
  // SERIAL PARA ACTIVAR HABILITAR EL Serial.begin();
  ////////////////////////////////////////////////////////////////////////////////////
  Serial.print(" angle[1] ");
  Serial.print(angle[1]);
  Serial.print(" angle[2] ");
  Serial.print(angle[2]);
  Serial.print(" gyro[1] ");
  Serial.print(giroscopio_angular[1]);
  Serial.print(" gyrp[2] ");
  Serial.println(giroscopio_angular[2]);

  ////////////////////////////////////////////////////////////////////////////////////
  // CONTROLADOR PID
  ////////////////////////////////////////////////////////////////////////////////////
  //PID_ROLL//
  roll_proporcional = (PID_GYRO[2] - DPS_ROLL) * roll_ganancia_proporcional;
  roll_integral += (PID_GYRO[2] - DPS_ROLL) * roll_ganancia_integral;
  roll_derivada = ((PID_GYRO[2] - DPS_ROLL) - (gyro_y_temp - DPS_ROLL_TEMP)) * roll_ganancia_derivada;

  PID_ROLL = roll_proporcional + roll_integral + roll_derivada;

  PID_GYRO[2] = gyro_y_temp;
  DPS_ROLL = DPS_ROLL_TEMP;
  /////////////
  //PID_PITCH//
  pitch_proporcional = (PID_GYRO[1] - DPS_PITCH) * pitch_ganancia_proporcional;
  pitch_integral += (PID_GYRO[1] - DPS_PITCH) * pitch_ganancia_integral;
  pitch_derivada = ((PID_GYRO[1] - DPS_PITCH) - (gyro_x_temp - DPS_PITCH_TEMP)) * pitch_ganancia_derivada;

  PID_PITCH = pitch_proporcional + pitch_integral + pitch_derivada;

  pitch_integral = pitch_integral_temp;
  PID_GYRO[1] = gyro_x_temp;
  DPS_PITCH = DPS_PITCH_TEMP;

  ///////////
  //PID_YAW//
  yaw_proporcional = (PID_GYRO[3] - DPS_YAW) * yaw_ganancia_proporcional;
  yaw_integral += (PID_GYRO[3] - DPS_YAW) * yaw_ganancia_integral;
  yaw_derivada = ((PID_GYRO[3] - DPS_YAW) - (gyro_z_temp - DPS_YAW_TEMP)) * yaw_ganancia_derivada;

  PID_YAW = yaw_proporcional + yaw_integral + yaw_derivada;

  yaw_integral = yaw_integral_temp;
  PID_GYRO[3] = gyro_z_temp;
  DPS_YAW = DPS_YAW_TEMP;

  /////////////////////////////////////////////////////////////
  //CORRECCIONES EN LOS VALORES PID_YAW; PID_PITCH; PID_ROLL;// 
  if(PID_ROLL >= 400) PID_ROLL = 400;
  if(PID_ROLL <= -400) PID_ROLL = -400;
  if(PID_PITCH >= 400) PID_PITCH = 400;
  if(PID_PITCH <= -400) PID_PITCH = -400;
  if(PID_YAW >= 400 ) PID_YAW = 400;
  if(PID_YAW <= -400 ) PID_YAW = -400;

  ////////////////////////////////////////////////////////////////////////////////////
  // PULSOS ESC's Y ESC's
  ////////////////////////////////////////////////////////////////////////////////////
  esc[1] = CH3 - PID_PITCH - PID_ROLL - PID_YAW;  //PC4
  esc[2] = CH3 - PID_PITCH + PID_ROLL + PID_YAW;  //PC5
  esc[3] = CH3 + PID_PITCH + PID_ROLL - PID_YAW;  //PC6
  esc[4] = CH3 + PID_PITCH - PID_ROLL + PID_YAW;  //PC7

  ////////////////////////////
  //CORRECCION DE LOS PULSOS//
  if(esc[1] <= 1000) esc[1] = 1000;
  if(esc[2] <= 1000) esc[2] = 1000;
  if(esc[3] <= 1000) esc[3] = 1000;
  if(esc[4] <= 1000) esc[4] = 1000; 

  if(esc[1] >= 2000) esc[1] = 2000;
  if(esc[2] >= 2000) esc[2] = 2000;
  if(esc[3] >= 2000) esc[3] = 2000;
  if(esc[4] >= 2000) esc[4] = 2000;

  //////////////////////////
  //ENVIAR PULSOS ESC[1-4]//
  fase_timer = micros();
  PORTD |= B11110000;
  esc_timer[1] = esc[1] + fase_timer;
  esc_timer[2] = esc[2] + fase_timer;
  esc_timer[3] = esc[3] + fase_timer;
  esc_timer[4] = esc[4] + fase_timer;

  while(PORTD >= 16){
    fase_timer = micros();
    if(esc_timer[1] <= fase_timer) PORTD &= B11101111;
    if(esc_timer[2] <= fase_timer) PORTD &= B11011111;
    if(esc_timer[3] <= fase_timer) PORTD &= B10111111;
    if(esc_timer[4] <= fase_timer) PORTD &= B01111111;
  }

  timer[2] = micros();
  while(timer[2] - timer[1] <= 4000) timer[2] = micros();
}

////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////////////////////////
void giroscopio_salida(){
  Wire.beginTransmission(MPU6050);
  Wire.write(BitHIGH_Start);
  Wire.endTransmission();

  Wire.requestFrom(MPU6050, 14);

  if(Wire.available() <= 14){
    acceleration[1] = Wire.read() << 8 | Wire.read();
    acceleration[2] = Wire.read() << 8 | Wire.read();
    acceleration[3] = Wire.read() << 8 | Wire.read();

    temperature = Wire.read() << 8 | Wire.read();

    gyroscope[1] = (Wire.read() << 8 | Wire.read());
    gyroscope[2] = (Wire.read() << 8 | Wire.read());
    gyroscope[3] = (Wire.read() << 8 | Wire.read());
  }
}

/* WIP
void Checker(int x){
  boolean Check = true;
  while (Check == true){
  fase_timer = micros();
  PORTD |= B11110000;
  esc_timer[1] = esc[1] + fase_timer;
  esc_timer[2] = esc[2] + fase_timer;
  esc_timer[3] = esc[3] + fase_timer;
  esc_timer[4] = esc[4] + fase_timer;

  while(PORTD >= 16){
    fase_timer = micros();
    if(esc_timer[1] <= fase_timer) PORTD &= B11101111;
    if(esc_timer[2] <= fase_timer) PORTD &= B11011111;
    if(esc_timer[3] <= fase_timer) PORTD &= B10111111;
    if(esc_timer[4] <= fase_timer) PORTD &= B01111111;
  }

  timer[2] = micros();
  while(timer[2] - timer[1] <= 4000) timer[2] = micros();
  }
}
*/

////////////////////////////////////////////////////////////////////////////////////
// SUBRUTINAS ISR
////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
    ISR_REFRESH_RATE = micros();
    //CHANNEL 1
    if(PINB & B00000001){
      if(IA1 == 0){
        IA1 = 1;
        ICRR1 = ISR_REFRESH_RATE;
      }
    }else if(IA1 == 1){
      IA1 = 0;
      CH1 = ISR_REFRESH_RATE - ICRR1;
    }
    //CHANNEL 2
    if(PINB & B00000010){
      if(IA2 == 0){
        IA2 = 1;
        ICRR2 = ISR_REFRESH_RATE;
      }
    }else if(IA2 == 1){
      IA2 = 0;
      CH2 = ISR_REFRESH_RATE - ICRR2;
    }
    //CHANNEL 3
    if(PINB & B00000100){
      if(IA3 == 0){
        IA3 = 1;
        ICRR3 = ISR_REFRESH_RATE;
      }
    }else if(IA3 == 1){
      IA3 = 0;
      CH3 = ISR_REFRESH_RATE - ICRR3;
    }
    //CHANNEL 4
    if(PINB & B00001000){
      if(IA4 == 0){
        IA4 = 1;
        ICRR4 = ISR_REFRESH_RATE;
      }
    }else if(IA4 == 1){
      IA4 = 0;
      CH4 = ISR_REFRESH_RATE - ICRR4;
    }
  }