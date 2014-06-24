//Base station v0.9.1 20130704 1300
#define VERSION "v0.9"
#define DEVICEID 55

#define VERSIONMAJOR 0
#define VERSIONMINOR 9

#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <math.h>

// NRF24 settings
#define RFADDR "alex1"
#define RFBASE "alex0"
#define RFCHANNEL 3


//manuell - automatik*********************************************************
//#define Manuelle_Steuerung

//constants
const byte MYID = DEVICEID;
const byte Version[2] = {VERSIONMAJOR,VERSIONMINOR};
const byte MAXSTATIONS = 15;

// Global Variables
int PID = 5;

// IMU
int a[3], g[3], m[3], a_old[3], g_old[3], m_old[3];

int16_t winkel_tn, winkel_tm, t_tm, t_tn, x_alt=0, y_alt=0, z_alt=0,

x,y,z,
y_WP= 1500,
x_WP= 500;


//regel parameter
float N_P    = 5,
      Rot_p  = 3,
      Rot_i  = 0.02,
      Rot_d  = 0.0;

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


int lowpassFilter( int x_new, int x_old, float alpha)
{ 
  return alpha*x_new + (1-alpha)*x_old;
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
        case 193: {//REG
          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
          }
          break;
        case 64: {//deltat
          unsigned long deltat;
          deltat = (unsigned long)data[8];
          deltat += (unsigned long)data[7]<<8;
          deltat += (unsigned long)data[6]<<16;
          deltat += (unsigned long)data[5]<<24;
          byte from = data[1];
          sync(from,data[9]);
          Serial.print("t");
          if(from<10) Serial.print('0');
          Serial.print(from);
          if(deltat<10) Serial.print('0');
          if(deltat<100) Serial.print('0');
          if(deltat<1000) Serial.print('0');
          if(deltat<10000) Serial.print('0');
          if(deltat<100000) Serial.print('0');
          if(deltat<1000000) Serial.print('0');
          if(deltat<10000000) Serial.print('0');
          if(deltat<100000000) Serial.print('0');
          if(deltat<1000000000) Serial.print('0');
          Serial.print(deltat);
          Serial.print("\n");
          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          
          
          newPacket(from, (byte)192, pid);
          }
          break;
          
        case 101: {

            a_old[0] = a[0];
            a_old[1] = a[1];
            a_old[2] = a[2];
            a[0] = lowpassFilter(((data[5]<<8) + data[6])/16.384, a_old[0], 0.5);
            a[1] = lowpassFilter(((data[7]<<8) + data[8])/16.384, a_old[1], 0.5);
            a[2]= lowpassFilter(((data[9]<<8) + data[10])/16.384, a_old[2], 0.5);

            int16_t x= (data[5]<<8) + data[6];
            int16_t y= (data[7]<<8) + data[8];
            int16_t z= (data[9]<<8) + data[10];

            //Serial.print("aX: ");Serial.print(x);Serial.print("\t");
            //Serial.print("aY: ");Serial.print(y);Serial.print("\t");
            //Serial.print("aZ: ");Serial.print(z);Serial.println("\t");



          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
          }
          break;
        case 102: {
          
            g_old[0] = g[0];
            g_old[1] = g[1];
            g_old[2] = g[2];
            g[0] = lowpassFilter(((data[5]<<8) + data[6])/131, g_old[0], 0.5);
            g[1] = lowpassFilter(((data[7]<<8) + data[8])/131, g_old[1], 0.5);
            g[2] = lowpassFilter(((data[9]<<8) + data[10])/131, g_old[2], 0.5);

            int16_t x= (data[5]<<8) + data[6];
            int16_t y= (data[7]<<8) + data[8];
            int16_t z= (data[9]<<8) + data[10];


            //Serial.print("gX: ");Serial.print(x);Serial.print("\t");
            //Serial.print("gY: ");Serial.print(y);Serial.print("\t");
            //Serial.print("gZ: ");Serial.print(z);Serial.println("\t");

            byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
          }
          break;
        case 103: {

            m_old[0] = m[0];
            m_old[1] = m[1];
            m_old[2] = m[2];
            m[0] = lowpassFilter(((data[5]<<8) + data[6]), m_old[0], 0.5);
            m[1] = lowpassFilter(((data[7]<<8) + data[8]), m_old[1], 0.5);
            m[2] = lowpassFilter(((data[9]<<8) + data[10]), m_old[2], 0.5);

            int16_t x= (data[5]<<8) + data[6];
            int16_t y= (data[7]<<8) + data[8];
            int16_t z= (data[9]<<8) + data[10];


            //Serial.print("mX: ");Serial.print(x);Serial.print("\t");
            //Serial.print("mY: ");Serial.print(y);Serial.print("\t");
            //Serial.print("mZ: ");Serial.print(z);Serial.println("\t");

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


byte devicesync[MAXSTATIONS] = {0};
// sync
void sync(byte from, byte syn) {
  if (devicesync[from]) {
    Serial.println("sync");
    for (int i=0;i<MAXSTATIONS;i++) {
      devicesync[i] = 0;
    }
  }
  devicesync[from] = 1;
}

//IPS daten****************************************************************

float xy_winkel(){
  float winkel= (float) atan((double)(x-x_alt)/(double)(y-y_alt));
  x_alt=x; y_alt=y;
  return winkel;
}

float WP_winkel(){
  float winkel= (float) atan((double)(x_WP-x)/(double)(y_WP-y));
  return winkel;
}

void lese_position(){
  if(Serial.available() > 0){
    while(Serial.read() ==  2){
      delay(7);
/*
      x_alt = x;
      y_alt = y;
      z_alt = z;
      x= lowpassFilter((Serial.read() << 8) + Serial.read(), x_alt, 0.5);
      y= lowpassFilter((Serial.read() << 8) + Serial.read(), y_alt, 0.5);
      z= lowpassFilter((Serial.read() << 8) + Serial.read(), z_alt, 0.5);
*/
      x= ((Serial.read() << 8) + Serial.read())*0.5 + x*0.5;
      y= ((Serial.read() << 8) + Serial.read())*0.5 + y*0.5;
      z= ((Serial.read() << 8) + Serial.read())*0.5 + z*0.5;

    }
    
    //Serial.println("Position");
    Serial.print("x:");Serial.print(x);
    Serial.print(" ");
    Serial.print("y:");Serial.print(y);
    Serial.print(" ");
    Serial.print("z:");Serial.print(z);
    Serial.println(" ");
  }
}


//erweiterte Team2 Regelung************************************************


float derivation(float tm,float tn){
   
  float dt = 1000.0*((tm-tn)/(t_tm-t_tn));
  return dt;
}
 
 
float integral(float tm,float tn){
   
  // trapezoidal method
  float dtau   = ((tm+tn)/2) * (t_tm-t_tn) * 0.001;
     
  return dtau;
}

int vorwaertsregelung(float P, float ist_winkel, float soll_winkel){
  
  int N_speed = 250 - abs((ist_winkel-soll_winkel) * P);
  
  if(N_speed > 0) return -N_speed;
  else return 0;
  
}

int drehregelung(float Rot_p,float Rot_i,float Rot_d, float ist_winkel, float soll_winkel){
  
  // Refresh time value
  t_tn = t_tm;
  t_tm = millis();
  
  winkel_tn = winkel_tm;
  winkel_tm = ist_winkel;
  
  // calculates difference
  float diff = (ist_winkel-soll_winkel);
  
  // calculate slope
  float winkel_slope = derivation(winkel_tm, winkel_tn);
  float winkel_int   = integral(winkel_tm, winkel_tn);
  
  // calculates integral
  int sum_winkel = sum_winkel + winkel_int; 
  //float f = f + diff_winkel;
 
  int mspeed_Rot= Rot_p * diff + Rot_i * sum_winkel + Rot_d * winkel_slope;
  return mspeed_Rot;
}


//*************************************************************************
//******************************************************************************************
byte Motor[6], regel_param[6];

  //motoren ansteuerungen
  int8_t Motor_N, Motor_Rot,  Motor_Z,
    P_h = 400,                //*100
    I_h = 4,                  //
    D_h = 0,                  //
    drehmomentausgleich = 8,  //
    
    Soll_h = 130;
    

  int regel_faktor =1;
  
  

long previousMillis = 0;
byte led_an[6];


void setup() {
  Serial.begin(9600);
  Serial.print("Team Propellerman Basisstation");
  //init NRF24
  Mirf.spi = &MirfHardwareSpi;
  Mirf.cePin = 9;
  Mirf.csnPin = 10;
  Mirf.init();
  Mirf.setRADDR((byte *)RFADDR);
  Mirf.payload = 12;
  Mirf.channel = RFCHANNEL;
  Mirf.config();
  
  //PID = readPID();
  
  //newPacket((byte)1, (byte)193, Version);
  regel_param[0] = P_h;
  regel_param[1] = I_h;
  regel_param[2] = D_h;
  regel_param[3] = Soll_h;
  regel_param[4] = drehmomentausgleich;
  
  if(newPacket(54, 30, regel_param))
  sendPackages();
  
  pinMode(2,INPUT);
}

//******************************************************************************************

void loop(){
  sendPackages();
  
  lese_position();
  //loop variablen
  float ist_winkel, soll_winkel;
  
  if(x_alt != x){
  ist_winkel= xy_winkel();    Serial.print("  ist_W: ");Serial.print(ist_winkel);
  soll_winkel= WP_winkel();   Serial.print("  soll_W: ");Serial.println(soll_winkel);
  }

  //Manuelle Steuerung*********************************
  #ifdef Manuelle_Steuerung
const int VERT = A0; // analog
const int HORIZ = A1; // analog
const int SEL = 2; // digital
  
   int vertical, horizontal, select;
  
  // read all values from the joystick
  
  vertical = analogRead(VERT) -510; // will be 0-1023
  horizontal = analogRead(HORIZ) -510; // will be 0-1023
  select = digitalRead(SEL); // will be HIGH (1) if not pressed, and LOW (0) if pressed

  /*
  Serial.print("vertical: ");
  Serial.print(vertical,DEC);
  Serial.print(" horizontal: ");
  Serial.print(horizontal,DEC);
  Serial.print(" select: ");*/
  if(select == HIGH){
//    Serial.println("not pressed");
    Motor_Z = -120;
  }else{
//    Serial.println("PRESSED!");
    Motor_Z = 0;
  }
 
  if((-40 > vertical) || (vertical > 40))Motor_N = int8_t(vertical/4.2);
  else Motor_N = 0;
  
  if((-40 > horizontal) || (horizontal > 40))Motor_Rot = int8_t(horizontal/4.6);
  else Motor_Rot = 0;
  
  //Serial.print(Motor_N);Serial.print("  ");Serial.print(Motor_Rot);Serial.print("  ");Serial.println(Motor_Z);
  
  #else
  
  
  
  //Regeln********************************************

  Motor_N =     vorwaertsregelung(N_P, ist_winkel, soll_winkel);
  
  Motor_Rot =   drehregelung(Rot_p, Rot_i, Rot_d, ist_winkel, soll_winkel);
//  Serial.print("Motor_N: "); Serial.println(Motor_N);
//  Serial.print("Motor_: "); Serial.println(Motor_N); 
 #endif

  

  
  Motor[0] = abs(Motor_N);      Serial.print("Motor_N  "); Serial.println(Motor_N);

  if(Motor_N > 0) Motor[1] = 0;
  else Motor[1] = 1;
  
  Motor[2] = abs(Motor_Rot * regel_faktor);
  if(Motor_Rot > 0) Motor[3] = 0;
  else Motor[3] = 1;
 
 
  Motor[4] = abs(Motor_Z * regel_faktor);
  if(Motor_Z > 0)Motor[5] = 0;
  else Motor[5] = 1;
  
  if(newPacket(54, 33, Motor))
  sendPackages();
  
  while(Mirf.isSending()) {};
  if(Mirf.dataReady()){
    parseMsg();
  }

  unsigned long currentMillis = millis();
 /*
  if(currentMillis - previousMillis > 1000) {
    previousMillis = currentMillis;   
    
    Serial.print("0:  ");  Serial.println(Motor[0]);
    //Serial.print("1:  ");  Serial.println(Motor[1]);
    
    led_an[0]= !led_an[0];
    newPacket( 54,10,led_an);
    
  }
  */
  
}
