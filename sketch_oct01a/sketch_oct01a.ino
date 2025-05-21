const int dir =2;
const int pull = 3;
const int enable = 8;
void setup()
{
  pinMode(dir,OUTPUT);
  pinMode(pull,OUTPUT);
  pinMode(enable,OUTPUT);
  digitalWrite(enable,LOW);
  }
void loop()
{
  digitalWrite(dir,HIGH);
  for(int i =1;i<50;i++)
  {
   
    digitalWrite(pull,HIGH);
    delayMicroseconds(500);
    digitalWrite(pull,LOW );
    delayMicroseconds(500);
  }
  digitalWrite(enable,HIGH);
  delay(1000);
  
}
  
  
