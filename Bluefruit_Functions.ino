void connectionManager()
{

  if ((ble.isConnected()) && (bleLoopControl == 0))
  { Serial.println( F("Switching to DATA mode!") );
    ble.setMode(BLUEFRUIT_MODE_DATA);

    Serial.println(F("***********************"));
    bluefruitConnection = true;
    bleLoopControl = 1;
  }
  else if (! ble.isConnected())
  { bluefruitConnection = false;
    if (bleLoopControl >= 1)
    {
      bluefruitTimeout = true;//starts the timer to auto turn off
      previousBLEMillis = millis();//once the timer turns off it will loop itself out of its case and
      //cant be accessed again until the bluefruit is connected
      Serial.println(previousBLEMillis);

    }

    bleLoopControl = 0;

  }

}//end connection manager


void bluefruitNeopixelColorPicker()
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {


    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);

    //Serial.println(previousBLEMillis);


    for (uint8_t i = 0; i < NUMPARKINGNEOPIXELSLEFT; i++) {
      leftParkingNeopixels.setPixelColor(i, leftParkingNeopixels.Color(red, green, blue, OFF));

    }
    for (uint8_t i = 0; i < NUMPARKINGNEOPIXELSRIGHT; i++) {
      rightParkingNeopixels.setPixelColor(i, rightParkingNeopixels.Color(red, green, blue, OFF));

    }
    leftParkingNeopixels.show(); // This sends the updated pixel color to the hardware.
    rightParkingNeopixels.show(); // This sends the updated pixel color to the hardware.

  }

}// end bluefruit NeopPixelColorPicker

void neopixelDisconnectedTimer()
{

  //Serial.println(currentBLEMillis);
  currentBLEMillis = millis();

  if ((currentBLEMillis - previousBLEMillis) >= intervalBLE)
  {
    previousBLEMillis = currentBLEMillis;
    bluefruitTimeout = false;
  }


}


