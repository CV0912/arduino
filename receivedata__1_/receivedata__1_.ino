void receivedata()
{
  while(Serial2.available()>0)
  {
    Serial.readStringUntil('!');
  }
}
