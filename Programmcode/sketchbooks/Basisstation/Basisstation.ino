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
#define Manuelle_Steuerung

//constants
const byte MYID = DEVICEID;
const byte Version[2] = {VERSIONMAJOR,VERSIONMINOR};
const byte MAXSTATIONS = 15;

// Global Variables
int PID = 5,
  WP_nr = 0,
  abwurf = 0,
  box = 0;

int16_t winkel_tn, winkel_tm, t_tm, t_tn, x_alt=0, y_alt=0, z_alt=0,x_uralt = 0, y_uralt = 0,

gx, gy, gz,
x,y,z,
WP[16][2]= {{-2000,-2000},{-500,100},{0,400},{1000,700},{1500,300}};
  float ist_winkel, soll_winkel, WP_WP_winkel, WP_time;

//regel parameter
float N_P = 5,
      Rot_p = .9,
      Rot_i = 0.2,
      Rot_d = 0.1,
      
      Pgy = .3;
      

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
  Mirf.getData(data);//Serial.print(" paket: ");Serial.println(data[2]);
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
          
            //lesen der Sensorwerte aus dem IMU-Board
        case 100:{
          //ist_winkel = ((data[5]) + (data[6] << 8))/10;
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
            //int16_t x= (data[5]<<8) + data[6];
            //int16_t y= (data[7]<<8) + data[8];
            //int16_t z= (data[9]<<8) + data[10];

 Serial.print("aX: ");Serial.print(x);Serial.print("\t");
 Serial.print("aY: ");Serial.print(y);Serial.print("\t");
 Serial.print("aZ: ");Serial.print(z);Serial.println("\t");


          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
          }
          break;
          
        case 102: {
             gx= ((data[5]<<8) + data[6])/10;
             gy= ((data[7]<<8) + data[8])/10;
             gz= (((data[9]<<8) + data[10])/10)*0.3 + gz*0.7;
             Serial.print("gZ: ");Serial.print(gz);Serial.println("\t");
             gz-=gz*0.1; 

 //Serial.print("gX: ");Serial.print(gx);Serial.print("\t");
 //Serial.print("gY: ");Serial.print(gy);Serial.print("\t");
 //Serial.print("gZ: ");Serial.print(gz);Serial.println("\t");

          byte pid[2];
          pid[0] = data[3];
          pid[1] = data[4];
          byte from = data[1];
          newPacket(from, (byte)192, pid);
          }
          break;
        case 103: {
           static int16_t y2,y_sum=0,x_sum=0;
           static int i=0, mini = 50, maxi=180;
          
            gz= (((data[5]<<8) + data[6])/10)*0.3 + gz*0.7;

            int16_t x= (data[5]<<8) + data[6];
            int16_t y= (data[7]<<8) + data[8];
            //int16_t z= (data[9]<<8) + data[10];
            
              //berechnung eines kompasses aus den magnetwerten
            if(y<mini)mini=y*0.3 + mini*0.7;
            if(y>maxi)maxi=y*0.3 + mini*0.7;
            
            y -= mini;
            y=y*180/(maxi-mini);
            
            x_sum += x;
            y_sum += y;
            i++;
            if(i>3){
              y= y_sum/i;
              x= x_sum/i;
              i= 0 ; y_sum = 0;x_sum = 0;
            /*
              y2= y - y2;
            
              if( (y2/abs(y2)) == (gz/abs(gz)))ist_winkel=y;
              else ist_winkel = -y;
            
              y2 = y;
              */
              ist_winkel= y *(x/abs(x));
            }
//Serial.print("gZ: ");Serial.print(gz);Serial.println("\t");
 //Serial.print("mX: ");Serial.print(x);Serial.print("\t");
 //Serial.print("mY: ");Serial.print(ist_winkel);Serial.print("\t");
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




//********************************************************************
//IPS daten
//********************************************************************

  //differenz zweier Winkel
float winkelDiff(float W1,float W2){
  float W =W2-W1;
  if(W < -180){
    W= -(360 + W);
  }else{
    if(W > 180)W=360-W;
  }
  return W;
}

  //Winkelberechnung aus den IPS-Koordinaten(schlecht!)
float xy_winkel(){
static long XY_time = 0;
static float winkel = 0;

if(millis() - XY_time > 2000){
  float winkel = (float) atan((double)(x-x_uralt)/(double)(y-y_uralt));
  winkel = winkel*360/(2*3.14159);
  XY_time = millis();
  x_uralt = x; y_uralt = y;
  }
  return winkel;
}

  //Berechnung des Winkels zwischen Gondel und Wegpunkt
float WP_winkel(){
  float winkel= (float) atan((double)(WP[WP_nr][0]-x)/(double)(WP[WP_nr][1]-y));
  winkel = winkel*360/(2*3.14159);
  return winkel;
}

  //einlesen der Positionsdaten
void lese_position(){
  if(Serial.available() > 0){
    int16_t x_temp, y_temp, z_temp;
    while(Serial.read() == 2){
      delay(9);
      
      x_temp = (Serial.read() << 8) + Serial.read();
      y_temp = (Serial.read() << 8) + Serial.read();
      z_temp = (Serial.read() << 8) + Serial.read();
      
        //sprung und 3fach mittelwert filter
      if(((x - x_temp)*(x - x_temp) + (y - y_temp)*(y - y_temp)) < 900){
      
      x= (x_temp)*0.2 + x*0.4 + x_alt*0.4;
      y= (y_temp)*0.2 + y*0.4 + y_alt*0.4;
      z= (z_temp)*0.2 + z*0.4 + z_alt*0.4;
      }
    }
  
Serial.println("Position");
Serial.print("x:");Serial.print(x);
Serial.print(" ");
Serial.print("y:");Serial.print(y);
Serial.print("   ");
Serial.print("z:");Serial.print(z);
Serial.print(" ");
  }
}


//erweiterte Team2 Regelung************************************************


float derivation(float tm,float tn){
   
  float dt = 1000.0*((tm-tn)/(t_tm-t_tn));
  return dt;
}
 
 
float integral(float tm,float tn){
   
  // trapezoidal method
  float dtau = ((tm+tn)/2) * (t_tm-t_tn) * 0.001;
     
  return dtau;
}

int8_t vorwaertsregelung(float P, float ist_winkel, float soll_winkel){
  
  int8_t N_speed = 127 - abs((winkelDiff(ist_winkel,soll_winkel)) * P);
  //Serial.print(" N_: ");Serial.print(N_speed);
  //Serial.print(" winkel_diff: ");Serial.println(winkelDiff(ist_winkel,soll_winkel));
  if(N_speed > 0) return N_speed;
  else return 0;
  
}  

int8_t drehregelung(float Rot_p,float Rot_i,float Rot_d, float ist_winkel, float soll_winkel){
  
  // Refresh time value
  t_tn = t_tm;
  t_tm = millis();
  
  winkel_tn = winkel_tm;
  winkel_tm = ist_winkel;
  
  // calculates difference
  float diff = winkelDiff(ist_winkel,soll_winkel);
  //Serial.print("winkel differenz:");Serial.println(diff);
  
  // calculate slope
  float winkel_slope = derivation(winkel_tm, winkel_tn);
  float winkel_int = integral(winkel_tm, winkel_tn);
  
  // calculates integral
  int sum_winkel = sum_winkel + winkel_int;
  //float f = f + diff_winkel;
 
  int8_t mspeed_Rot= (Rot_p * diff + Rot_i * sum_winkel + Rot_d * winkel_slope)*.1;
  return mspeed_Rot;
}


//*************************************************************************
//******************************************************************************************

  //motoren ansteuerungen
  int8_t Motor_N, Motor_Rot, Motor_Z,temp_Rot = 0,
    P_h = 20,                   //*10
    I_h = 55,                   //*100
    D_h = 0,                   //
    drehmomentausgleich = 60, //60
    
    Soll_h = 130;          //cm
    
  byte Motor[6], regel_param[6];
  

long previousMillis = 0;
byte led_an[6];
  boolean winkel_flag=0;

void setup() {
  Serial.begin(115200);
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
  
  pinMode(2,INPUT);

    
  regel_param[0] = P_h;
  regel_param[1] = I_h;
  regel_param[2] = D_h;
  regel_param[3] = Soll_h;
  regel_param[4] = drehmomentausgleich;
  
  if(newPacket(54, 30, regel_param))
  sendPackages();
  
}



//******************************************************************************************

void loop(){
 
    //motoren ansteuerungen
  static int8_t Motor_N, Motor_Rot, Motor_Z,temp_Rot = 0,
    P_h = 30,                   //*10
    I_h = 55,                   //*100
    D_h = 0,                   //
    drehmomentausgleich = 60, //60
    
    Soll_h = 130;          //cm
    
  byte regel_param[6];
  
  sendPackages();
  regel_param[0] = P_h;
  regel_param[1] = I_h;
  regel_param[2] = D_h;
  regel_param[3] = Soll_h;
  regel_param[4] = drehmomentausgleich;
  
  if(newPacket(54, 30, regel_param))
  sendPackages();
  
  
  //Manuelle Steuerung**********************************************************
#ifdef Manuelle_Steuerung
const int VERT = A0; // analog
const int HORIZ = A1; // analog
const int SEL = 2; // digital
  
   int vertical, horizontal, select;
  
  // read all values from the joystick
  
  vertical = analogRead(VERT) -510; // will be 0-1023
  horizontal = analogRead(HORIZ) -510; // will be 0-1023
  select = digitalRead(SEL); // will be HIGH (1) if not pressed, and LOW (0) if pressed

  if(select == HIGH){
// Serial.println("not pressed");
    Motor_Z = -120;
  }else{
// Serial.println("PRESSED!");
    Motor_Z = 0;
  }
 
 if((-40 > vertical) || (vertical > 40))Motor_N = int8_t(vertical/4.2);
  else Motor_N = 0;
  
  if((-40 > horizontal) || (horizontal > 40))Motor_Rot = int8_t(horizontal/4.6);
  else Motor_Rot = 0;
  

#else
  
  //autonome Steuerung***********************************************************
  
  lese_position();
  //loop variablen

  
  if((x_alt != x)){   //neue koordinaten
  
  //ist_winkel= xy_winkel();
  soll_winkel= WP_winkel();
  x_alt=x; y_alt=y;
  winkel_flag = 1;
  
  //Serial.print("soll winkel: ");Serial.println(soll_winkel);
  
  //WEGPUNKTE FLUG************************************
  Serial.print(" WPnr: ");Serial.print(WP_nr);
  
    //berechnen eines Winkels zwischen zwei Wegpunkten
  WP_WP_winkel = (float) atan((double)(WP[WP_nr + 1][0]- WP[WP_nr][0])/(double)(WP[WP_nr + 1][1]- WP[WP_nr][1]));
  WP_WP_winkel = WP_WP_winkel *360/(2*3.14159);
  
    //testen ob ein Wegpunkt erreicht wurde
  if((((float)(x - WP[WP_nr][0])*(float)(x - WP[WP_nr][0]) + (float)(y - WP[WP_nr][1])*(float)(y - WP[WP_nr][1])) < 3600)
      && (abs(ist_winkel - WP_WP_winkel) < 60) ){
        
        if(WP_nr == abwurf){      //Wegpunkt ist der abwurfpunkt
          Soll_h = 40; 
        }else if(WP_nr == box){   //an dem wegpunkt befindet sich die kiste -> höhe anpassen
          Soll_h = 90;
        }else Soll_h = 130;
        
        WP_nr++;  //der Wegpunkt wird abgehakt
        Serial.print(" WPnr: ");Serial.print(WP_nr);
        Serial.print(" x: ");Serial.print(WP[WP_nr][0]);Serial.print(" y: ");Serial.println(WP[WP_nr][1]);
      }else Soll_h = 130;
  
//  Serial.print("ist winkel: ");Serial.print(ist_winkel);
//  Serial.print("  WP-WP winkel: ");Serial.println(WP_WP_winkel);
  
  }else{
    winkel_flag = 0;
    Motor_Rot = temp_Rot;
  }

  
  
  //Regeln*********************************************************************************2
  //zum testen!*********
  //Serial.print("  mot_rot");Serial.println(Motor_Rot);

 // if(winkel_flag){
  Motor_N = vorwaertsregelung(N_P, ist_winkel, 40);
  //Motor_Rot = 0;
  Motor_Rot = drehregelung(Rot_p, Rot_i, Rot_d, ist_winkel, 40);
  temp_Rot = Motor_Rot;
  //Serial.print("  rot: ");Serial.println(Motor_Rot);
  
  Serial.print(" ist_W: ");Serial.print(ist_winkel);
  Serial.print(" winkel differenz: ");Serial.println(winkelDiff(ist_winkel,40));
 // Serial.print(" soll_W: ");Serial.print(soll_winkel);
  
  //Serial.print(" Motor_N: "); Serial.print(Motor_N);
  //Serial.print(" Motor_Rot: "); Serial.println(Motor_Rot);
//  }
  
  
  
  //notfall Regelung**********************************
  /*
  if(((float)(x - WP[WP_nr][0])*(float)(x - WP[WP_nr][0]) + (float)(y - WP[WP_nr][1])*(float)(y - WP[WP_nr][1])) < 1600){
    if(Motor_N > 5){
      WP_time = millis(); Motor_N = -100;
  } 
    if((millis() - WP_time) > 2000)Motor_N = 0;
      
      float ist_WP_winkel = atan((float)(x - WP[WP_nr + 1][0])/(float)(y - WP[WP_nr + 1][1])),
          WP_ist_winkel = atan((float)(WP[WP_nr - 1][0] - x)/(float)(WP[WP_nr - 1][1] - y));
      int Rtime = 20 * winkelDiff(WP_ist_winkel,ist_WP_winkel);
    
      if((millis() - WP_time) < Rtime)Motor_Rot = 100;
      else{
      if((millis() - WP_time) < 2*Rtime)Motor_Rot = -100;
      else{
        Motor_Rot = 0; WP_nr++;Motor_N = 100;
      }
    }
    
    
  }*/
  
  
#endif
  
  Motor_Rot += (gz * Pgy);//;Serial.print("gyr: ");Serial.println(gz);
  
  Motor[0] = abs(Motor_N);
  if(Motor_N > 0) Motor[1] = 0;
  else Motor[1] = 1;
  
  Motor[2] = abs(Motor_Rot);
  if(Motor_Rot > 0) Motor[3] = 0;
  else Motor[3] = 1;
 
 
  Motor[4] = abs(Motor_Z);
  if(Motor_Z > 0)Motor[5] = 0;
  else Motor[5] = 1;
  
  if(newPacket(54, 33, Motor))
  sendPackages();
  
  while(Mirf.isSending()) {};
  if(Mirf.dataReady()){
    parseMsg();
  }
  sendPackages();

  unsigned long currentMillis = millis();
 /*
if(currentMillis - previousMillis > 1000) {
previousMillis = currentMillis;
Serial.print("0: "); Serial.println(Motor[0]);
//Serial.print("1: "); Serial.println(Motor[1]);
led_an[0]= !led_an[0];
newPacket( 54,10,led_an);
}
*/
  
}
