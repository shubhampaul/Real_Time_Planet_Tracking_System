/*
RTPT (Real Time Planet Tracking System and Trajectory Prediction)
Copyright © 2016  Shubham Paul , Samhita Ganguly ,Rohit Kumar
This file is part of RTPT.
    RTPT is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    RTPT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with RTPT.  If not, see <http://www.gnu.org/licenses/>.
 */
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

