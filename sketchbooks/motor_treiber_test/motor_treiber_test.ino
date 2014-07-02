void setup()
{
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(7,OUTPUT);
}

void loop()
{
  delay(20);
  digitalWrite(5,HIGH);
  digitalWrite(7,LOW);
  analogWrite(6,100);
  
  delay(200);
  analogWrite(6,0);
  delay(50);
  
  digitalWrite(5,LOW);
  digitalWrite(7,HIGH);
  analogWrite(6,200);
  
  delay(200);
  analogWrite(6,0);
  delay(200);

}
