
/*
 * Ultraschallsensor(MB1240)
 */
  
long value = 0;                      // Variable für die Zeit vom US-Sensor zum Objekt
long high = 0;                       // Variable für die Entfernung vom US-Sensor zum Objekt
 
const int us    = A2;                // US Pin 2  an Pin A1
int Entfernung = 0;
 
void setup()
{
  Serial.begin(9600);                // Baudrate
  pinMode(us, INPUT);                // US-Pin als Eingang festlegen
   
  Serial.println("DISTANCE");
  Serial.println("cm");
 }
 
void loop()
{
  value = pulseIn(us, HIGH);         // Berechnungen
  high = value*(1000/58);
  Entfernung = high/1000;
   
  Serial.print(Entfernung);
  Serial.println("cm");
   
  delay(20);                         // Zeit bis zur nächsten Messung in ms
 
}


/*
 * Ultraschallsensor(SRF02)
 */
 /*
 #include <Wire.h>
  
void setup()
{
  Serial.begin(9600);                // Baudrate
  Wire.begin();
}
 
 int i2cGetMeasurement (byte address) {
  int reading = -1;
  Wire.beginTransmission(address); // transmit to device #112
  
  Wire.write(0x02);      // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();      // stop transmitting
  delay(6);
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


void loop()
{
  int hight=0;
  i2cStartMeasurement(0xE2);
  delay(100);
  hight= i2cGetMeasurement(0xE2);
  Serial.println(hight);
  
}*/
