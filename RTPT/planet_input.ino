int planetInput(int potval)
{
  if ((potval >= 0) && (potval <= 90))
  {
    Serial.print("Mercury");
    return 1;
  }
  if ((potval >= 91) && (potval <= 191))
  {
    Serial.print("Venus");
    return 2;
  }
  if ((potval >= 192) && (potval <= 292))
  {
    Serial.print("Mars");
    return 4;
  }
  if ((potval >= 293) && (potval <= 393))
  {
    Serial.print("Jupiter");
    return 5;
  }
  if ((potval >= 394) && (potval <= 494))
  {
    Serial.print("Saturn");
    return 6;
  }
  if ((potval >= 495) && (potval <= 595))
  {
    Serial.print("Uranus");
    return 7;
  }
  if ((potval >= 596) && (potval <= 696))
  {
    Serial.print("Neptune");
    return 8;
  }
  if ((potval >= 697) && (potval <= 1023))
  {
    Serial.print("Pluto");
    return 9;
  }
}

