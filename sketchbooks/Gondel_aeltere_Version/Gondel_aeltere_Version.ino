// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro(0x69);

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
byte data[12];
byte sent = 0;
boolean imu_init = true;
double imu_winkel_x = 0;
//double imu_winkel_y = 0;
//double imu_winkel_z = 0;
int imu_time;

int16_t h_tn, h_tm;
int16_t tm, tn, t_tm, t_tn;

int16_t height_soll = 130;

float P_h = 4,
        I_h = .04,
        D_h = 0,
    
        Drehmoment = .05;
    
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
unsigned long t; // needed for IPS
unsigned long time; //needed for IPS
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
      //if(millis() - time < data[2]) {
      if(millis() - time < 30) {
        if (millis()-time >= sent*3){
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
        
          //einlesen der gesendeten Parameter
        case 30:{    
          
          P_h = data[5]/10.0; Serial.print(" P: ");Serial.print(P_h);
          I_h = data[6]/100.0; Serial.print(" I: ");Serial.print(I_h);
          D_h = data[7]/100.0; Serial.print(" D: ");Serial.print(D_h);
          
          height_soll = data[8]; Serial.print(" soll: ");Serial.print(height_soll);
          
          Drehmoment = data[9]/100.0; Serial.print(" Drehmoment: ");Serial.println(Drehmoment);
          
        }
        break;
        
          //einlesen der Steuerungswerte für die MOTOREN
        case 33:{
          N_speed = data[5]*2;  N_direction = data[6];
          Rot_speed = data[7]*2;  Rot_direction = data[8];
          //Z_speed = data[9]*(-2.2); bool Z_direction = data[10];
         
         //Serial.print("N: ");Serial.print(N_speed); Serial.print(" N_dir: ");Serial.print(N_direction);
         //Serial.print(" Rot_roh: ");Serial.print(data[8]);//Serial.print(" Rot_dir: ");Serial.print(Rot_direction);
         //Serial.print(" Rot_dir: ");Serial.print(Rot_direction);
 // Serial.print(" Z: ");Serial.println(Z_speed);Serial.print(" Z_dir: ");Serial.print(Z_direction);
         
        }
        break;
         /* 
             //lesen der Sensorwerte aus dem IMU-Board
         case 100:{
          ist_winkel = ((data[5]) + (data[6] << 8))/10;
          //Serial.print("Winkel_yaw: ");Serial.print(yaw);
          float pitch =((data[7]) + (data[8] << 8))/10;
          //Serial.print("  Winkel_pitch: ");Serial.print(pitch);
          float roll =((data[9]) + (data[10] << 8))/10;
          Serial.print("  Winkel_ist: ");Serial.println(ist_winkel);
          
          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
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
          */
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




    //Funktionen zum auslesen der Höhensensorwerte
 int i2cGetMeasurement (byte address) {
  int reading = -1;
  Wire.beginTransmission(address); // transmit to device #112
  
  Wire.write(0x02); // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission(); // stop transmitting

  Wire.requestFrom(address, byte(2)); // request 2 bytes from slave device #112
  if(1 < Wire.available()) // if two bytes were received
  {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
  }
  return reading;
}

void i2cStartMeasurement (byte address) {
    Wire.beginTransmission(address);
    Wire.write(byte(0x00));
    Wire.write(byte(0x51)); //0x50 Real Ranging Mode - Result in inches
                             //0x51 Real Ranging Mode - Result in centimeters
                             //0x52 Real Ranging Mode - Result in micro-seconds
                             //0x56 Fake Ranging Mode - Result in inches
                             //0x57 Fake Ranging Mode - Result in centimeters
                             //0x58 Fake Ranging Mode - Result in micro-seconds
                             //0x5C Transmit an 8 cycle 40khz burst
    Wire.endTransmission();
}




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
   // delayMicroseconds(3);
   // delayMicroseconds(2);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);
      
      digitalWrite(signal, LOW);
   // delayMicroseconds(3);
   // delayMicroseconds(2);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);
      
    }
  }
}


    //TEAM2 PID-Regeler angewandt für HÖHENREGELUNG
  
  float derivation(float tm,float tn){
   
  float dt = 1000.0*((tm-tn)/(t_tm-t_tn));
  return dt;
}
 
 
float integral(float tm,float tn){
   
  // trapezoidal method
  float dtau = ((tm+tn)/2) * (t_tm-t_tn) * 0.001;
     
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
  float h_int = integral(h_tm, h_tn);
   
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
    //Z_direction = 1;
    //return mspeed_h*0.8;
    return 0;
  }
  
  }
  


    //Anfang des Hauptprogramms*********************************************************************

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.print("Team Propellerman Gondel");

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
  pinMode(signal, OUTPUT); //40kHz signal for IPS
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
  
}

long previousMillis = 0;


//***************************************************************************
//***************************************************************************
void loop(){
 
  int ax_old, ay_old, az_old, gx_old, gy_old, gz_old, mx_old, my_old, mz_old;
  static long h_time = 0;
  
  
   ips_signal();  //senden des 40kHz Signals für die IPS bodenstationen
   
   
  //Berechnung der Höhenwerte************************************************
  
  height3 = height2;
  height2 = height;
  
  i2cStartMeasurement(byte(240));
  
  if(millis() - h_time > 70){
    height= (i2cGetMeasurement(byte(240))*0.3) + (height2*0.4) +(height3*0.3);
    h_time = millis();
  }

  

  //auslesen und weitersenden der IMU-Daten***********************************
  
  //Serial.print("Sleep Enabled: ");
  //Serial.println(accelgyro.getSleepEnabled());
      accelgyro.setSleepEnabled(false);
      
  // read raw accel/gyro measurements from device
      accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
      
  
      
      //Serial.print("ax:");Serial.print(ax-ax_old);Serial.print(" ay:");Serial.print(ay-ax_old);Serial.print(" az:");Serial.print(az-ax_old);
      //Serial.print(" mx:");Serial.print(mx-mx_old);
      Serial.print(" my:");Serial.print(my);
      //Serial.print(" mz:");Serial.print(mz-mz_old);
      //Serial.print(" gx:");Serial.print(gx-gx_old);Serial.print(" gy:");Serial.print(gy-gy_old);Serial.print(" gz:");Serial.println(gz-gz_old);
      

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    //accelgyro.getMag(&mx, &my, &mz);
  
  byte a[6], g[6], m[6];
//    a[0]= (ax & 0xFF00) >> 8;a[1]= (ax & 0x00FF);
//    a[2]= (ay & 0xFF00) >> 8;a[3]= (ay & 0x00FF);
//    a[4]= (az & 0xFF00) >> 8;a[5]= (az & 0x00FF);
//    newPacket((byte)55, (byte)101, a);
//    sendPackages();
//    while(Mirf.isSending()) {};
   
    //m[0]= (mx & 0xFF00) >> 8;m[1]= (mx & 0x00FF);
    m[0]= (gz & 0xFF00) >> 8;m[1]= (gz & 0x00FF);
    m[2]= (my & 0xFF00) >> 8;m[3]= (my & 0x00FF);
    m[4]= (mx & 0xFF00) >> 8;m[5]= (mx & 0x00FF);
    //m[4]= (mz & 0xFF00) >> 8;m[5]= (mz & 0x00FF);
    newPacket((byte)55, (byte)103, m);
    sendPackages();
    while(Mirf.isSending()) {};
   /* 
    g[0]= (gx & 0xFF00) >> 8;g[1]= (gx & 0x00FF);
    g[2]= (gy & 0xFF00) >> 8;g[3]= (gy & 0x00FF);
    g[4]= (gz & 0xFF00) >> 8;g[5]= (gz & 0x00FF);
    newPacket((byte)55, (byte)102, g);
    sendPackages();
    while(Mirf.isSending()) {};*/
 /*
Serial.print("a/g/m:\t");
Serial.print(ax); Serial.print("\t");
Serial.print(ay); Serial.print("\t");
Serial.print(az); Serial.print("\t");
Serial.print(gx); Serial.print("\t");
Serial.print(gy); Serial.print("\t");
Serial.print(gz); Serial.print("\t");
Serial.print(mx); Serial.print("\t");
Serial.print(my); Serial.print("\t");
Serial.println(mz);
*/


//  sendPackages();
//  if (newPacket (55, 101, a))
//  sendPackages();
//  if (newPacket (55, 102, g))
//    sendPackages();
//  if (newPacket (55, 103, m))
//    sendPackages();




  //SEND**********************************************************************
  sendPackages();
  while(Mirf.isSending()) {};


  if(Mirf.dataReady()){
    parseMsg();
  }
/*
unsigned long currentMillis = millis();
if(currentMillis - previousMillis > interval) {
// save the last time you blinked the LED
previousMillis = currentMillis;

}*/
  
 // if((abs(height - height3) > 40) && (height3 != 0)) height_soll += (height - height3);
  
 // Serial.print(" korrektur: ");Serial.print(height-height3);
  Serial.print(" soll: ");Serial.print(height_soll);
     
  
  //Serial.print(" hoehe: ");
  //Serial.println(height);


//Motoren Steuerung***********************************************************

Z_speed = hoehenregelung(P_h,I_h,D_h,height);

//Serial.print(" rot in");Serial.print(Rot_speed);
//Serial.print(" dir: ");Serial.print(Rot_direction);
  int rotSpeed;
  bool rotDir;
 rotSpeed = ((1-(2*Rot_direction))*Rot_speed) - (Z_speed * Drehmoment);
 if(rotSpeed > 0)rotDir=0;
 else rotDir=1;

//Serial.print(" rot out:");Serial.println(rotSpeed);
   rotSpeed = abs(rotSpeed);
  
  Serial.print(" dreh:");Serial.println(Drehmoment);
         digitalWrite(4,N_direction); analogWrite(5,N_speed);
         digitalWrite(7,rotDir); analogWrite(6,rotSpeed);

 // digitalWrite(8,Z_direction); analogWrite(9,Z_speed);
 
 //int hoehenregelung(float H_p,float H_i,float H_d,int height){
         digitalWrite(8,Z_direction); analogWrite(9,Z_speed);
    
}
