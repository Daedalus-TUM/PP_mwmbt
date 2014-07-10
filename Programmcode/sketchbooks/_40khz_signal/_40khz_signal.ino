int signal = 10;
unsigned long t;
unsigned long time;

void setup()
{
 pinMode(signal, OUTPUT);
 time = micros();
}

void loop()
{
  
  if(micros()-time>200000)
  {
    time = micros();
    
    for(t=0; t<=200;)
    {
      t = micros()-time;
    
      digitalWrite(signal, HIGH);
      delayMicroseconds(2);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);
      digitalWrite(signal, LOW);
      delayMicroseconds(2);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);
      delayMicroseconds(1);

    }
      
  /*
  digitalWrite(signal, HIGH);
  delay(50);
  digitalWrite(signal, LOW);  
  */
  }
}
