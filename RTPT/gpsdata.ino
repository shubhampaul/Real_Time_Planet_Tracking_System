
static void smartDelay(unsigned long ms)
{
  //Serial.println("C");
  unsigned long start = millis();
  do
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
  //Serial.println("D");
}

