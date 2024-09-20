#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

TaskHandle_t Task1;


#include <EEPROM.h>
#include <Wire.h>


/*  ESPINO  */

const int P0=12;
const int P1=13;

const int P2=14;
const int P5=16;
const int P6=4;
const int P7=5;

#define tamtur 500

#define MPU 0x68

int16_t ax,ay,az;
int16_t temperatura;
int16_t gx,gy,gz;

double girox,giroy,giroz;

float angx=0;
float angy=0;
float angz=0;

float dt=0;
float tiempo_prev=0;

float girosc_ang_z=0;
float girosc_ang_z_prev=0;

float calibracion=0;

long tiempo=0;

boolean semaforogz=false;



///MADGWICK


#define sampleFreqDef   512.0f          // sample frequency in Hz
#define betaDef         0.1f            // 2 * proportional gain

double delta_t = 0; // Used to control display output rate
uint32_t now = 0;        // used to calculate integration interval
uint32_t last_update = 0; // used to calculate integration interval
float beta;       // algorithm gain
float q0;
float q1;
float q2;
float q3; // quaternion of sensor frame relative to auxiliary frame
float roll;
float pitch;
float yaw;
char anglesComputed=0;


void iniciaMadgwick(){
  beta = betaDef;
  q0 = 1.0f;
  q1 = 0.0f;
  q2 = 0.0f;
  q3 = 0.0f;
  now = micros();
  anglesComputed = 0;
}

void Madgwick_BMX160_updateIMU(float gx2, float gy2, float gz2, float ax2, float ay2, float az2) {
 
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;


  
  ax2=convertRawAccel(ax2);
  ay2=convertRawAccel(ay2);
  az2=convertRawAccel(az2);
  
  gx2=convertRawGyro(gx2);
  gy2=convertRawGyro(gy2);
  gz2=convertRawGyro(gz2);
 
  
  // Convert gyroscope degrees/sec to radians/sec
  
  gx2 *= 0.0174533f;
  gy2 *= 0.0174533f;
  gz2 *= 0.0174533f;
  
  now = micros();
  //  Set integration time by time elapsed since last filter update
  delta_t = ((now - last_update) / 1000000.0f);
  last_update = now;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx2 - q2 * gy - q3 * gz2);
  qDot2 = 0.5f * (q0 * gx2 + q2 * gz - q3 * gy2);
  qDot3 = 0.5f * (q0 * gy2 - q1 * gz + q3 * gx2);
  qDot4 = 0.5f * (q0 * gz2 + q1 * gy - q2 * gx2);


  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax2 == 0.0f) && (ay2 == 0.0f) && (az2 == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = Madgwick_BMX160_invSqrt(ax2 * ax2 + ay2 * ay2 + az2 * az2);
    ax2 *= recipNorm;
    ay2 *= recipNorm;
    az2 *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax2 + _4q0 * q1q1 - _2q1 * ay2;
    s1 = _4q1 * q3q3 - _2q3 * ax2 + 4.0f * q0q0 * q1 - _2q0 * ay2 - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az2;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax2 + _4q2 * q3q3 - _2q3 * ay2 - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az2;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax2 + 4.0f * q2q2 * q3 - _2q2 * ay2;
    recipNorm = Madgwick_BMX160_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
    
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * delta_t;
  q1 += qDot2 * delta_t;
  q2 += qDot3 * delta_t;
  q3 += qDot4 * delta_t;

  // Normalise quaternion
  recipNorm = Madgwick_BMX160_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  anglesComputed = 0;
  
}


float Madgwick_BMX160_invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void Madgwick_BMX160_computeAngles(){
  roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
  pitch = asinf(-2.0f * (q1*q3 - q0*q2));
  yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
  anglesComputed = 1;
}


   float getRoll() {
        if (!anglesComputed) Madgwick_BMX160_computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) Madgwick_BMX160_computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) Madgwick_BMX160_computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) Madgwick_BMX160_computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) Madgwick_BMX160_computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) Madgwick_BMX160_computeAngles();
        return yaw;
    }
    
  void getQuaternion(float *w, float *x, float *y, float *z) {
       *w = q0;
       *x = q1;
       *y = q2;
       *z = q3;
   }


float convertRawGyro(int gRaw) {
  
  // since we are using 2000 degrees/seconds range
  
  float g = (gRaw * 2000.0) / 32768.0;

  return g;
  
}

float convertRawAccel(int aRaw) {
 
  float a = (aRaw * (16.0 / 32768.0));

  return a;
}


void calibracionderiva(){
    
    for(int i=0;i<10000;i++){
         captura();
         calibracion=(calibracion*0.5)+(0.5*((float)gz));      
    }  
}


void captura(){

  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Pedir el registro 0x47 - corresponde al giroZ
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU,14,true); //A partir del 0x3B, se piden 6 registros
  
  while(Wire.available()<14){}

  ax=Wire.read()<<8|Wire.read();
  ay=Wire.read()<<8|Wire.read();
  az=Wire.read()<<8|Wire.read();
  temperatura=Wire.read()<<8|Wire.read(); 
  gx=Wire.read()<<8|Wire.read();
  gy=Wire.read()<<8|Wire.read();
  gz=Wire.read()<<8|Wire.read();

  if(abs(gz)<8){ gz=0; }
     
}

void angulo(){

   /*
   Madgwick_BMX160_updateIMU(gx,gy,gz,ax,ay,az);

   angx=getRollRadians(); 
   angy=getPitchRadians(); 
   angz=getYawRadians();  
   */

   dt=micros()-tiempo_prev;
   tiempo_prev = ((float)micros());

   girosc_ang_z=(((float)gz)*(0.06103515625)*(dt/1000000))+girosc_ang_z_prev;   ///0.06103515625  sale de dividir 2000/32768
   
   girosc_ang_z_prev=girosc_ang_z;
   
}



void setup() {

  int i=0;
  
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT); 
  pinMode(14, OUTPUT); 
  pinMode(16, OUTPUT);    
  pinMode(15, OUTPUT);    

  digitalWrite(15,HIGH);

  Wire.begin(21,22);
  Wire.setClock(400000);
    
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1A);   //CONFIG
  Wire.write(0x00);   // Se supone que a 8Khz
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x19);   //Sample Rate
  Wire.write(0x00);   //  8000 / 1 + 0
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);   //Setting Accel
  Wire.write(0x18);      //  + / - 16g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);   //Setting Giro
  Wire.write(0x18);      //  2000dps
  Wire.endTransmission(true);
  
 
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(LED_BUILTIN, LOW); //Este led se ilumina al contrario

  tiempo=millis();

  delay(5000);

  Serial.println("Vamos a calibrar gz");
  
  calibracionderiva();

  Serial.println(calibracion);

  Serial.println("");

  delay(5000);

  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    0,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 
                    

  iniciaMadgwick();
  
}

void Task1code( void * pvParameters ){    // en este Core recogemos las peticiones web


  tiempo_prev = ((float)micros());

  for(;;){   

    captura();
    angulo();
    
    esp_task_wdt_init(30,false);
    esp_task_wdt_reset();
    
    delayMicroseconds(10);
    
  } 
  
}



void loop() {

  int i=0;

  if(Serial.available()){ 

      String val="";
      while(Serial.available()){
          char c=Serial.read();
          if((c=='\r')||(c=='\n')){ break; }
          val+=c;          
          delay(10);                  
      }
      while(Serial.available()){  char c=Serial.read();  }
      
      calibracion=val.toFloat();

      Serial.print("Calibracion : ");Serial.println(val);

      girosc_ang_z_prev=0.0;
      
      delay(5000);
      
      
  }

  if((millis()-tiempo)>10) {   
        
    //Serial.print("Angulo : ");
    
    //Serial.print(ax);Serial.print(",");Serial.print(ay);Serial.print(",");Serial.print(az);Serial.print(",");
    //Serial.print(gx);Serial.print(",");Serial.print(gy);Serial.print(",");Serial.print(gz);Serial.print(",");
    //Serial.println(temperatura);

    //Serial.print(q0);Serial.print(",");Serial.print(q1);Serial.print(",");Serial.print(q2);Serial.print(",");Serial.println(q3);

    Serial.println(girosc_ang_z_prev);
    //Serial.println(gz);
    
    tiempo=millis();
    
  }

   
   esp_task_wdt_init(30,false);
   esp_task_wdt_reset();
   

}
