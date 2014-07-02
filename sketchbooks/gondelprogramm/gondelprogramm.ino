#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <Mirf.h>
#include <MirfSpiDriver.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//für Magnetometer
#include "MPU6050_9Axis_MotionApps41.h"

MPU6050 accelgyro(0x69);  //aus dem sleep mode wecken
 //////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////IMUU/////////////////////////////////////////////////////
// Declare device MPU6050 class
MPU6050 mpu;

#define GyroMeasError PI * (40.0f / 180.0f) // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f) // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)

#define beta sqrt(3.0f / 4.0f) * GyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3; // raw data arrays reading
uint16_t count = 0; // used to control display output rate
uint16_t delt_t = 0; // used to control display output rate
uint16_t mcount = 0; // used to control display output rate
uint8_t MagRate; // read rate for magnetometer datax^xx^xX

float pitch, yaw, roll;
float deltat = 0.0f; // integration interval for both filter schemes
uint16_t lastUpdate = 0; // used to calculate integration interval
uint16_t now = 0; // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f}; // vector to hold integral error for Mahony method
        
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm; // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm; // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex; // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f; // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
 
        }

 //////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////IMUU/////////////////////////////////////////////////////
byte data[12];
byte sent = 0;
boolean imu_init = true;
double imu_winkel_x = 0;
double imu_winkel_y = 0;
double imu_winkel_z = 0;
double imu_time;
int my360;

//Höhenregelung
int16_t h_tn, h_tm;
int16_t tm, tn, t_tm, t_tn;

int16_t height_soll = 130;

float   P_h = 4,
        I_h = .04,
        D_h = 0,
    
        Drehmoment = .02;
    
  int height=0;
  int height2=0;
  int height3=0;
  
  int N_speed, Rot_speed, Z_speed;      
  bool N_direction,
       Rot_direction,
       Z_direction;
    

#define LED_PIN 13
bool blinkState = false;



//Base station v0.9.1 20130704 1300
#define VERSION "v0.9"
#define DEVICEID 54

#define VERSIONMAJOR 0
#define VERSIONMINOR 9

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

// NRF24 settings
#define RFADDR "alex0"
#define RFBASE "alex1"
#define RFCHANNEL 3

//constants
const byte MYID = DEVICEID;
const byte Version[2] = {VERSIONMAJOR,VERSIONMINOR};
const byte MAXSTATIONS = 15;

// Global Variables
int PID = 5;
unsigned long t;  // needed for IPS
unsigned long time;  //needed for IPS
int signal = 10; //needed for IPS

class Packet {
  //byte time;
  unsigned long time;
  byte sent;
  byte data[12];
  
  public:
    Packet (byte dest, byte type, byte *payload) {
      sent = 0;
      //time = millis() % 128;
      time = millis();
      data[0] = dest;
      data[1] = MYID;
      data[2] = type;
      data[3] = (byte)((PID & 0xFF00) >> 8);
      data[4] = (byte)(PID & 0x00FF);
      PID++;
      for (int i = 0; i<7;i++){
        data[i+5] = payload[i];
      }
    }
    ~Packet () {
    }
    byte send (void){
      if(millis() - time < data[2]) {
        if (millis()-time >= sent*5){
          data[11] = sent;
          Mirf.setTADDR((byte *)RFBASE);
          Mirf.send(data);
          sent++;
        }
        if(data[2] == 192) return 1;
        return 0;
      } else {
        return 1;
      }
    }
    
    unsigned int getPID() {
      return (unsigned int)((unsigned int)data[3]<< 8 + (unsigned int)data[4]);
    }
};
Packet *Packages[5];
byte busy=0;
int devicePid [MAXSTATIONS][5] = {{0}};
int deviceLastPid [MAXSTATIONS] = {0};

void sendPackages(void) {
    if ((busy&1)) {
      if (Packages[0]->send()) {
        busy &= 0b11111110;
        delete Packages[0];
      }
    } else if ((busy&2)) {
      if (Packages[1]->send()) {
        busy &= 0b11111101;
        delete Packages[1];
      }
    } else if ((busy&4)) {
      if (Packages[2]->send()) {
        busy &= 0b11111011;
        delete Packages[2];
      }
    } else if ((busy&8)) {
      if (Packages[3]->send()) {
        busy &= 0b11110111;
        delete Packages[3];
      }
    } else if ((busy&16)) {
      if (Packages[4]->send()) {
        busy &= 0b11101111;
        delete Packages[4];
      }
    }
}
boolean newPacket (byte dest, byte type, byte *payload) {
  if (busy<31){
    if (!(busy&1)) {
      Packages[0] = new Packet (dest, type,payload);
      busy |= 1;
    } else if (!(busy&2)) {
      Packages[1] = new Packet (dest, type,payload);
      busy |= 2;
    } else if (!(busy&4)) {
      Packages[2] = new Packet (dest, type,payload);
      busy |= 4;
    } else if (!(busy&8)) {
      Packages[3] = new Packet (dest, type,payload);
      busy |= 8;
    } else if (!(busy&16)) {
      Packages[4] = new Packet (dest, type,payload);
      busy |= 16;
    }
    return true;
  } else return false;
}

void deletePID (int pid) {
  Serial.println(busy);
    if ((busy&1)) {
      if (Packages[0]->getPID() == pid) {
        busy &= 0b11111110;
        delete Packages[0];
      }
    } else if ((busy&2)) {
      if (Packages[1]->getPID() == pid) {
        busy &= 0b11111101;
        delete Packages[1];
      }
    } else if ((busy&4)) {
      if (Packages[2]->getPID() == pid) {
        busy &= 0b11111011;
        delete Packages[2];
      }
    } else if ((busy&8)) {
      if (Packages[3]->getPID() == pid) {
        busy &= 0b11110111;
        delete Packages[3];
      }
    } else if ((busy&16)) {
      if (Packages[4]->getPID() == pid) {
        busy &= 0b11101111;
        delete Packages[4];
      }
    }
    Serial.println(busy);
}
byte parseMsg() {
  byte data[12];
  Mirf.getData(data);
  unsigned int pid;
  pid = ((((unsigned int)data[3])<< 8) + (unsigned int)data[4]);
  byte from = data[1];
  
  if(data[0] == MYID ) {
    if (isNewPid(from,pid)) {
      switch(data[2]) {
        case 192: //ACK
          unsigned int ackpid;
          ackpid = ((unsigned int)data[5]<< 8 + (unsigned int)data[6]);
          deletePID(ackpid);
          break;
        
        
                  //LED TEST
        case 10:{
          
            if(data[5]){
              Serial.println("an ");
              digitalWrite(21,HIGH);
            }else{
              Serial.println("aus ");
              digitalWrite(21,LOW);
            }
        }
        break;
        
        case 30:{
          
          P_h = data[5]/10.0;  Serial.print(" P: ");Serial.print(P_h);
          I_h = data[6]/100.0;  Serial.print(" I: ");Serial.print(I_h);
          D_h = data[7]/100.0;  Serial.print(" D: ");Serial.print(D_h);
          
          height_soll = data[8]; Serial.print(" soll: ");Serial.print(height_soll);
          
          Drehmoment = data[9]/100.0;  Serial.print(" Drehmoment: ");Serial.println(Drehmoment);
          
        }
        break;
        
                  //MOTOREN
        case 33:{
          N_speed = data[5]*2;        bool N_direction = data[6];
          Rot_speed = data[7]*2;      bool Rot_direction = data[8];
          //Z_speed = data[9]*(-2.2);   bool Z_direction = data[10];
         
 //        Serial.print("N: ");Serial.print(N_speed); Serial.print(" N_dir: ");Serial.print(N_direction);
 //        Serial.print("  Rot: ");Serial.print(Rot_speed);Serial.print(" Rot_dir: ");Serial.print(Rot_direction);
 //        Serial.print("  Z: ");Serial.println(Z_speed);Serial.print(" Z_dir: ");Serial.print(Z_direction);
         
        }
        break;
          
        case 101: {
            int16_t x= (data[5]<<8) + data[6];
            int16_t y= (data[7]<<8) + data[8];
            int16_t z= (data[9]<<8) + data[10];
            Serial.println("aX: ");Serial.print(x);
            Serial.println("aY: ");Serial.print(y);
            Serial.println("aZ: ");Serial.print(z);
          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
          }
          break;
        case 102: {
            int16_t x= (data[5]<<8) + data[6];
            int16_t y= (data[7]<<8) + data[8];
            int16_t z= (data[9]<<8) + data[10];
            Serial.print("gX: ");Serial.print(x);
            Serial.print("gY: ");Serial.print(y);
            Serial.print("gZ: ");Serial.print(z);
            byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
          }
          break;
        case 103: {
            int16_t x= (data[5]<<8) + data[6];
            int16_t y= (data[7]<<8) + data[8];
            int16_t z= (data[9]<<8) + data[10];
            Serial.print("mX: ");Serial.print(x);
            Serial.print("mY: ");Serial.print(y);
            Serial.print("mZ: ");Serial.print(z);
          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
          }
          break;
          
        default:
          return 0; //kenne Datentyp nicht
      }
    } else {// Paket wurde bereits empfangen, wird nicht verarbeitet sondern nur bestätigt
      byte pid[2];
      pid[0] = data[3];
      pid[1] = data[4];
      newPacket(from, (byte)192, pid);
    }
  } else {
    return 0; //Nachricht nicht für mich
  }
  return 1;
}

// Prüft ob PID neu oder bereits empfangen wurde.
// Rückgabewert 0, falls bereits empfangen
// 1, falls neue PID
byte isNewPid(byte from, unsigned int pid) {
  unsigned int minpid = 65535;
  byte minpidi =0;
  for(byte i = 0 ; i<5 ; i++) {
    if(devicePid[from][i] == pid) {
      return 0; //PID bereits empfangen
    }
    if(devicePid[from][i] < minpid) {
      minpid = devicePid[from][i];
      minpidi = i;
    }
  }
  if (minpid < pid) {
    devicePid[from][minpidi] = pid;
    deviceLastPid[from] = pid;
    return 1;
  } else if (pid <15) {
    for(byte i = 0 ; i<5 ; i++) {
      if(devicePid[from][i] > 15) {
        devicePid[from][i] = 0;
        minpidi = i;
      }
    }
    devicePid[from][minpidi] = pid;
    deviceLastPid[from] = pid;
    return 1;
  } else if (deviceLastPid[from] < minpid && deviceLastPid[from] != pid){
    devicePid[from][minpidi] = pid;
    deviceLastPid[from] = pid;
    return 1;
  }
  deviceLastPid[from] = pid;
  return 2;
}




//############Höhensensor

 int i2cGetMeasurement (byte address) {
  int reading = -1;
  Wire.beginTransmission(address); // transmit to device #112
  
  Wire.write(0x02);      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting

  Wire.requestFrom(address, byte(2));    // request 2 bytes from slave device #112
  if(1 < Wire.available())    // if two bytes were received
  {
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
  return reading;
}

void i2cStartMeasurement (byte address) {
    Wire.beginTransmission(address);
    Wire.write(byte(0x00));
    Wire.write(byte(0x51));  //0x50  Real Ranging Mode - Result in inches
                             //0x51  Real Ranging Mode - Result in centimeters
                             //0x52  Real Ranging Mode - Result in micro-seconds
                             //0x56  Fake Ranging Mode - Result in inches
                             //0x57  Fake Ranging Mode - Result in centimeters
                             //0x58  Fake Ranging Mode - Result in micro-seconds
                             //0x5C  Transmit an 8 cycle 40khz burst
    Wire.endTransmission();
}

//##############

//############## IPS Ultraschall- und Infrarotsignal (200us lang 40kHz Impulse, alle 200ms)
void ips_signal()
{
  if(micros()-time>200000)
  {
    time = micros();
    
    
    for(t=0; t<=200;)
    {
      t = micros()-time;
  
      digitalWrite(signal, HIGH);
   //   delayMicroseconds(3);
   //   delayMicroseconds(2);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);
      
      digitalWrite(signal, LOW);
   //   delayMicroseconds(3);
   //   delayMicroseconds(2);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);
      
    }
  }  
}


//TEAM2 HÖHENREGELUNG**************************************************
  
  float derivation(float tm,float tn){
   
  float dt = 1000.0*((tm-tn)/(t_tm-t_tn));
  return dt;
}
 
 
float integral(float tm,float tn){
   
  // trapezoidal method
  float dtau   = ((tm+tn)/2) * (t_tm-t_tn) * 0.001;
     
  return dtau;
}
  
  int hoehenregelung(float H_p,float H_i,float H_d,int height){
   
  // Save values from previous cycle
  h_tn = h_tm;
  // Get current values
  h_tm = height;
   
  // Refresh time value
  t_tn = t_tm;
  t_tm = millis();
   
   
  // calculate slope
  float h_slope = derivation(h_tm, h_tn);
  float h_int   = integral(h_tm, h_tn);
   
  // calculates difference
  int diff = height_soll - height;
  
  // calculates integral
  int sum_h = sum_h + h_int; 
  float f = f + diff;
 
  int mspeed_h= H_p * diff + H_i * sum_h + H_d * h_slope;
  
  //Serial.print(" stellwert_H: ");Serial.print(mspeed_h);
  
 
  // PID-controller
  if(mspeed_h > 0)
  {
    Z_direction = 0;
    return mspeed_h;
  }
  else{
  //  Z_direction = 1;
  //  return mspeed_h*0.8;
  return 0;
  }
  }
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////IMU aus MPU9150////////////////////////////////////////////////////




//*********************************************************************

void setup() {
  Serial.begin(9600);
 
 delay(2000);

  Wire.begin();

  Serial.print("t1");
  //init NRF24
  Mirf.spi = &MirfHardwareSpi;
  Mirf.cePin = 19;
  Mirf.csnPin = 18;
  Mirf.init();
  Mirf.setRADDR((byte *)RFADDR);
  Mirf.setTADDR((byte *)RFBASE);
  Mirf.payload = 12;
  Mirf.channel = RFCHANNEL;
  Mirf.config();
  Serial.print("t2");
  //PID = readPID();
  
  //newPacket((byte)1, (byte)193, Version);
  
  //PINMODES**********************************
  
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(signal, OUTPUT);  //40kHz signal for IPS
  time = micros();
  imu_time = time;
  Serial.println(imu_time);
  Serial.print("t3");
  i2cStartMeasurement(byte(240));
  Serial.print("t4");
  delay(70);
  height= i2cGetMeasurement(byte(240));
  Serial.print("t5");
  h_tn = height;
  h_tm = height;
  Serial.print("/setup");
 //////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////IMUU/////////////////////////////////////////////////////

{
  

  Serial.begin(9600); // Start serial at 38400 bps
;
    // initialize MPU6050 device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU9150 connection successful") : F("MPU9150 connection failed"));

// Set up the accelerometer, gyro, and magnetometer for data output

   mpu.setRate(7); // set gyro rate to 8 kHz/(1 * rate) shows 1 kHz, accelerometer ODR is fixed at 1 KHz

   MagRate = 10; // set magnetometer read rate in Hz; 10 to 100 (max) Hz are reasonable values

  mpu.setDLPFMode(4); // set bandwidth of both gyro and accelerometer to ~20 Hz

// Full-scale range of the gyro sensors:
// 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
  mpu.setFullScaleGyroRange(0); // set gyro range to 250 degrees/sec

// Full-scale accelerometer range.
// The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
  mpu.setFullScaleAccelRange(0); // set accelerometer to 2 g range

  mpu.setIntDataReadyEnabled(true); // enable data ready interrupt
}
}

long previousMillis = 0;


//**************************************************************************
//**************************************************************************
void loop(){

  
  int height=0;
 //Serial.println("t1 ");
  //Höhe******************************************************
  
  height3 = height2;
  height2 = height;
  i2cStartMeasurement(byte(240));
  delay(70);
  

  height= (i2cGetMeasurement(byte(240))) + (height2*0.4) +(height3*0.3);

  //IMU*******************************************************
  //Serial.print("Sleep Enabled: ");
  //Serial.println(accelgyro.getSleepEnabled());
      accelgyro.setSleepEnabled(false);
   
//Serial.println("t2 ");

  //SEND*****************************************************	
  sendPackages();
  while(Mirf.isSending()) {};

  //Serial.println("t3 ");
  //SEND*****************************************************
  //sendPackages();
  //while(Mirf.isSending()) {};


  //SEND*****************************************************	
  sendPackages();
  while(Mirf.isSending()) {};


  if(Mirf.dataReady()){
    parseMsg();
  }
  //Serial.println("t4 ");
/*
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   

    
  }*/
  
 // if((abs(height - height3) > 40) && (height3 != 0)) height_soll += (height - height3);
  
 //    Serial.print(" korrektur: ");Serial.print(height-height3);
 //    Serial.print(" soll: ");Serial.print(height_soll);
     
  
//      Serial.print(" hoehe: ");
//    Serial.println(height);


//Motoren Steuerung*****************************************

Z_speed = hoehenregelung(P_h,I_h,D_h,height);

//Serial.println("t5 ");
  int rotSpeed;
  bool rotDir;

 rotSpeed = ((1-(2*Rot_direction))*Rot_speed) - (((1-(2*Z_direction))*Z_speed) * Drehmoment);

 if(rotSpeed > 0)rotDir=0;
 else rotDir=1;

 rotSpeed = abs(rotSpeed);

  
  
  //Serial.print(" dreh:");Serial.println(Drehmoment);
         digitalWrite(4,N_direction);    analogWrite(5,N_speed);
         digitalWrite(7,rotDir);  analogWrite(6,rotSpeed);

 //        digitalWrite(8,Z_direction);    analogWrite(9,Z_speed);
 
 //int hoehenregelung(float H_p,float H_i,float H_d,int height){
         digitalWrite(8,Z_direction);    analogWrite(9,Z_speed);
         
         //Serial.println("t6 ");
         
//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////IMUU/////////////////////////////////////////////////////

if(mpu.getIntDataReadyStatus() == 1) { // wait for data ready status register to update all data registers
            mcount++;
           // read the raw sensor data
            mpu.getAcceleration ( &a1, &a2, &a3 );
            ax = a1*2.0f/32768.0f; // 2 g full range for accelerometer
            ay = a2*2.0f/32768.0f;
            az = a3*2.0f/32768.0f;

            mpu.getRotation ( &g1, &g2, &g3 );
            gx = g1*250.0f/32768.0f; // 250 deg/s full range for gyroscope
            gy = g2*250.0f/32768.0f;
            gz = g3*250.0f/32768.0f;
// The gyros and accelerometers can in principle be calibrated in addition to any factory calibration but they are generally
// pretty accurate. You can check the accelerometer by making sure the reading is +1 g in the positive direction for each axis.
// The gyro should read zero for each axis when the sensor is at rest. Small or zero adjustment should be needed for these sensors.
// The magnetometer is a different thing. Most magnetometers will be sensitive to circuit currents, computers, and
// other both man-made and natural sources of magnetic field. The rough way to calibrate the magnetometer is to record
// the maximum and minimum readings (generally achieved at the North magnetic direction). The average of the sum divided by two
// should provide a pretty good calibration offset. Don't forget that for the MPU9150, the magnetometer x- and y-axes are switched
// compared to the gyro and accelerometer!
            if (mcount > 1000/MagRate) { // this is a poor man's way of setting the magnetometer read rate (see below)
            mpu.getMag ( &m1, &m2, &m3 );
            mx = m1*10.0f*1229.0f/4096.0f + 18.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
            my = m2*10.0f*1229.0f/4096.0f + 70.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
            mz = m3*10.0f*1229.0f/4096.0f + 270.0f;
            mcount = 0;
            }
         }
   
  now = micros();
  deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
 // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);
 MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate
/*
    Serial.print("ax = "); Serial.print((int)1000*ax);
    Serial.print(" ay = "); Serial.print((int)1000*ay);
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2);
    Serial.print(" gy = "); Serial.print( gy, 2);
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mx );
    Serial.print(" my = "); Serial.print( (int)my );
    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]);
    Serial.print(" qy = "); Serial.print(q[2]);
    Serial.print(" qz = "); Serial.println(q[3]);
*/                   
    
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth.
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw *= 180.0f / PI - 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll *= 180.0f / PI;

   // Serial.print("Yaw, Pitch, Roll: ");
   // Serial.print(yaw, 2);
   // Serial.print(", ");
   // Serial.print(pitch, 2);
   // Serial.print(", ");
   // Serial.println(roll, 2);
    
   // Serial.print("rate = "); Serial.print((float)1.0f/deltat, 2); Serial.println(" Hz");
    

    }
    //Serial.println("t7 ");
    uint16_t Y;
    byte w[6];
    
    Y = (int)yaw*10;
    Serial.print("Yaw: ");
    Serial.println(yaw, 2);
    w[0] = Y & 0xFF;
    w[1] = (Y >> 8) & 0xFF;
    
    if(newPacket(55, 100, w)){
  sendPackages();
    }

    }
            
    

