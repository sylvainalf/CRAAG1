/*
* Flight Computer code for CRAAG1 pico payload
* Phil Crump - phildcrump@gmail.com
*
* This code is based heavily on: [ie. forked from]
* PicoAtlas - James Coxon jacoxon@googlemail.com
* See http://ukhas.org.uk/projects:picoatlas for more info
* Latest PicoAtlas code can be found: https://github.com/jamescoxon/PicoAtlas
*
* GPS Code from jonsowman and Joey flight computer CUSF
* https://github.com/cuspaceflight/joey-m/tree/master/firmware

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/  
#include <stdlib.h>
#include <SPI.h>
#include "RFM22.h"
#include <util/crc16.h>

//Setup radio on SPI with NSEL on pin 10
rfm22 radio1(10);

//Variables
int32_t lat = 0, lon = 0, alt = 0;
uint8_t hour = 0, minute = 0, second = 0, lock = 0; // sats = 0;
unsigned long startGPS = 0;
int GPSerror = 0, count = 0, n, gpsstatus, lockcount = 0, total_time = -1, old_total_time = -2;
//Added by Phil
uint16_t battV = 0;
int16_t intTemp = 0, extTemp = 0; // Need to be signed
uint8_t rfmTemp = 0;
int32_t rawBat = 0, rawInt = 0, rawExt = 0;
char state;
//
uint8_t buf[60]; //GPS receive buffer
char superbuffer [80]; //Telem string buffer

// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{

	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/
	char c;
	c = *string++;
	while ( c != '\0')
	{
		rtty_txbyte (c);
		c = *string++;
	}
}

void rtty_txbyte (char c)
{
	/* Simple function to sent each bit of a char to 
	** rtty_txbit function. 
	** NB The bits are sent Least Significant Bit first
	**
	** All chars should be preceded with a 0 and 
	** proceded with a 1. 0 = Start bit; 1 = Stop bit
	**
	** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
	*/
	int i;
	rtty_txbit (0); // Start bit
	// Send bits for for char LSB first	
	for (i=0;i<7;i++)
	{
		if (c & 1) rtty_txbit(1); 
			else rtty_txbit(0);	
		c = c >> 1;
	}
	rtty_txbit (1); // Stop bit
        rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
		if (bit)
		{
		  // high
                  radio1.write(0x073, 0x03);
		}
		else
		{
		  // low
                  radio1.write(0x073, 0x00);
		}
					 // 50 Baud:
                delayMicroseconds(19500); // 10000 = 100 BAUD 20150
                // Theoretical 100 Baud:
                //delayMicroseconds(9500); // (20000)/2 = 10000 - 500 as that seems to be how the one above is calculated
                // Theoretical 300 Baud:
                //delayMicroseconds(2850); // (20000)/6 = 3333.3 - 500 as that seems to be how the one above is calculated

}

void setupRadio(){
  
  //digitalWrite(A3, LOW); // Turn on Radio
  
  delay(1000);
  
  radio1.initSPI();

  radio1.init();
  
  radio1.write(0x71, 0x00); // unmodulated carrier
 
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b,0x12);
  radio1.write(0x0c,0x15);
  
  radio1.setFrequency(434.201);
  
  radio1.write(0x6D, 0x04);// turn tx low power 11db
  
  radio1.write(0x07, 0x08); // turn tx on
  
}
//************Other Functions*****************

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
  //Serial.println();
}

uint16_t gps_CRC16_checksum (char *string)
{
	size_t i;
	uint16_t crc;
	uint8_t c;
 
	crc = 0xFFFF;
 
	// Calculate checksum ignoring the first two $s
	for (i = 2; i < strlen(string); i++)
	{
		c = string[i];
		crc = _crc_xmodem_update (crc, c);
	}
 
	return crc;
}

void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  // Taken from Project Swift (rather than the old way of sending ascii text)
  uint8_t setNMEAoff[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0xA9};
  sendUBX(setNMEAoff, sizeof(setNMEAoff)/sizeof(uint8_t));
  
  //// Check and set the navigation mode (Airborne, 1G)
  //uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  //sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));

  // Check and set the navigation mode (Pedestrian) 67mph vel limit, 9000m altitude limit
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  
  delay(500);
  
}

void gpsPower(int i){
  if(i == 0){
    //turn off GPS
    //  uint8_t GPSoff[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B};
    uint8_t GPSoff[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x16, 0x74};
    sendUBX(GPSoff, sizeof(GPSoff)/sizeof(uint8_t));
    gpsstatus = 0;
  }
  else if (i == 1){
    //turn on GPS
     //uint8_t GPSon[] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37};
    uint8_t GPSon[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x17, 0x76};
    sendUBX(GPSon, sizeof(GPSon)/sizeof(uint8_t));
    gpsstatus = 1;
    delay(1000);
    setupGPS();
  }
}

void PSMgps(){
   setupGPS();
   //set GPS to Eco mode (reduces current by 4mA)
   //Copied from James Coxon's Code - github.com/jamescoxon/
   // ACTUALLY: Seems to set GPS to PowerSave mode, NOT Eco
   //uint8_t setEco[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x04, 0x1D, 0x85};
   //sendUBX(setEco, sizeof(setEco)/sizeof(uint8_t));
   
   uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 }; // Setup for Power Save Mode (Default Cyclic 1s)
   sendUBX(setPSM, sizeof(setPSM)/sizeof(uint8_t));
}

/**
 * Calculate a UBX checksum using 8-bit Fletcher (RFC1145)
 */
void gps_ubx_checksum(uint8_t* data, uint8_t len, uint8_t* cka,
        uint8_t* ckb)
{
    *cka = 0;
    *ckb = 0;
    for( uint8_t i = 0; i < len; i++ )
    {
        *cka += *data;
        *ckb += *cka;
        data++;
    }
}

/**
 * Verify the checksum for the given data and length.
 */
bool _gps_verify_checksum(uint8_t* data, uint8_t len)
{
    uint8_t a, b;
    gps_ubx_checksum(data, len, &a, &b);
    if( a != *(data + len) || b != *(data + len + 1))
        return false;
    else
        return true;
}

/**
 * Get data from GPS, times out after 1 second.
 */
void gps_get_data()
{
    int i = 0;
    unsigned long startTime = millis();
    while (1) {
    // Make sure data is available to read
    if (Serial.available()) {
      buf[i] = Serial.read();
      i++;
    }
    // Timeout if no valid response in 1 seconds
    if (millis() - startTime > 1000) {
      break;
    }
    }
}
/**
 * Check the navigation status to determine the quality of the
 * fix currently held by the receiver with a NAV-STATUS message.
 */
void gps_check_lock()
{
    GPSerror = 0;
    Serial.flush();
    // Construct the request to the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x06, 0x00, 0x00,
        0x07, 0x16};
    sendUBX(request, 8);

    // Get the message back from the GPS
    gps_get_data();
    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 ) {
      GPSerror = 11;
    }
    if( buf[2] != 0x01 || buf[3] != 0x06 ) {
      GPSerror = 12;
    }

    // Check 60 bytes minus SYNC and CHECKSUM (4 bytes)
    if( !_gps_verify_checksum(&buf[2], 56) ) {
      GPSerror = 13;
    }
    
    if(GPSerror == 0){
    // Return the value if GPSfixOK is set in 'flags'
    if( buf[17] & 0x01 )
        lock = buf[16];
    else
        lock = 0;

    //sats = buf[53];
    }
    else {
      lock = 0;
    }
}

/**
 * Poll the GPS for a position message then extract the useful
 * information from it - POSLLH.
 */
void gps_get_position()
{
    GPSerror = 0;
    Serial.flush();
    // Request a NAV-POSLLH message from the GPS
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03,
        0x0A};
    sendUBX(request, 8);
    
    // Get the message back from the GPS
    gps_get_data();

    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        GPSerror = 21;
    if( buf[2] != 0x01 || buf[3] != 0x02 )
        GPSerror = 22;
        
    if( !_gps_verify_checksum(&buf[2], 32) ) {
      GPSerror = 23;
    }
    
    if(GPSerror == 0) {
      // 4 bytes of longitude (1e-7)
      lon = (int32_t)buf[10] | (int32_t)buf[11] << 8 | 
          (int32_t)buf[12] << 16 | (int32_t)buf[13] << 24;
      lon /= 1000;
      
      // 4 bytes of latitude (1e-7)
      lat = (int32_t)buf[14] | (int32_t)buf[15] << 8 | 
          (int32_t)buf[16] << 16 | (int32_t)buf[17] << 24;
      lat /= 1000;
      
      // 4 bytes of altitude above MSL (mm)
      alt = (int32_t)buf[22] | (int32_t)buf[23] << 8 | 
          (int32_t)buf[24] << 16 | (int32_t)buf[25] << 24;
      alt /= 1000;
    }

}

/**
 * Get the hour, minute and second from the GPS using the NAV-TIMEUTC
 * message.
 */
void gps_get_time()
{
    GPSerror = 0;
    Serial.flush();
    // Send a NAV-TIMEUTC message to the receiver
    uint8_t request[8] = {0xB5, 0x62, 0x01, 0x21, 0x00, 0x00,
        0x22, 0x67};
     sendUBX(request, 8);

    // Get the message back from the GPS
    gps_get_data();

    // Verify the sync and header bits
    if( buf[0] != 0xB5 || buf[1] != 0x62 )
        GPSerror = 31;
    if( buf[2] != 0x01 || buf[3] != 0x21 )
        GPSerror = 32;

    if( !_gps_verify_checksum(&buf[2], 24) ) {
      GPSerror = 33;
    }
    
    if(GPSerror == 0) {
      if(hour > 23 || minute > 59 || second > 59)
      {
        GPSerror = 34;
      }
      else {
        hour = buf[22];
        minute = buf[23];
        second = buf[24];
        total_time = hour + minute + second;
      }
    }
}

void pollADC(bool toggle_tx) {
  if (toggle_tx){
    radio1.write(0x07, 0x01); // turn tx off
  }
  rawBat = analogRead(3);
  rawInt = analogRead(2);
  rawExt = analogRead(1);
  if (toggle_tx){
    radio1.write(0x07, 0x08); // turn tx on
  }
}

char* removeWhite(char *str){
  int len;
        while( isspace(*(++str)) ); // Trim leading space
        if( (*str) == '\0' )
        {
                return str;
        }
        len = strlen(str);
        len--;
        while ( isspace(*(str +len) ))
        {
                len--;
        }
        *(str + len + 1) = '\0';
        //return (str-1);
        return str;
}

void prepData() {
  if(gpsstatus == 1){
    gps_check_lock();
    if (lock == 0x03 || lock == 0x04) {
       lockcount = 0;
       state = 'F'; //Flight
       gps_get_position();
       gps_get_time();
     } else {
       delay(500);
       gps_check_lock();
       if (lock == 0x03 || lock == 0x04) {
         lockcount = 0;
         state = 'F'; //Flight
         gps_get_position();
         gps_get_time();
       } else {
         lockcount++;
         state = 'L'; //Lost Lock
         //sats = 0;
       }
    }
  }
  count++;
  // Remember Arduino ADC REF = 1.1V == 1023
  battV = (uint16_t)((rawBat/(1024/1.1)) * 300); // eg. gives 122 for 1.22V
  extTemp = (int16_t)(((rawExt)-465.0)/0.93); // eg. gives 132 for 13.2
  intTemp = (int16_t)(((rawInt)-465.0)/0.93);
  //rfmTemp = temperatureRead( 0x00,0 ) / 2;  //from RFM22
  //rfmTemp = rfmTemp - 64;
  n=sprintf (superbuffer, "$$CRAAG1,%d,%02d%02d%02d,%ld,%ld,%ld,%c,%d,%d,%d", count, hour, minute, second, lat, lon, alt, state, battV, intTemp, extTemp);
  //n=sprintf (superbuffer, "$$CRAAG1,%d,%02d%02d%02d,%ld,%ld,%ld,%d,%c", count, hour, minute, second, lat, lon, alt, sats, state);
  n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
}

void setup() {
  state = 'B'; // Booting
  //pinMode(A3, OUTPUT);
  //digitalWrite(A3, HIGH); //Turn radio off
  Serial.begin(9600);
  delay(500);
  gpsPower(0); // Turn GPS off
  setupRadio();
  radio1.write(0x07, 0x08); // turn tx on
  delay(250);
  radio1.write(0x07, 0x01); // turn tx off
  delay(250);
  radio1.write(0x07, 0x08); // turn tx on
  delay(250);
  radio1.write(0x07, 0x01); // turn tx off
  delay(250);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  analogReference(INTERNAL); //Set 1v1 ADC reference
}

void loop() {
  delay(500);
  
  //If we still haven't got a lock after 5 minutes of searching power down the GPS and 
  // restart the main loop;
  if(millis() - startGPS > 300000){
    gpsPower(0); //turn GPS off
    state = 'T'; //Timeout
  }
  
  if(gpsstatus == 0){ // GPS is off, due to first start or timeout
    //Before we turn on the GPS (and potentially reboot due to too much current draw) transmit a single RTTY string
    pollADC(true);
    prepData();
    radio1.write(0x07, 0x08); // turn tx on
    delay(4000);
    rtty_txstring(superbuffer);
    radio1.write(0x07, 0x01); // turn tx off
    //Turn radio off, sleep for 1 sec in preperation for GPS to be powered on
    delay(1000);
    gpsPower(1); //turn GPS on
    startGPS = millis();
  }
  else { // GPS is on
    gps_check_lock();
  
    if( lock == 0x03 || lock == 0x04 )
    {
      // Set GPS to Power-Saving Mode, reduces power consumption drastically.
      PSMgps();
      while(count < 700 || alt < 2000) { // Only if it's less than 4 hours, or we are on the ground
         if (lockcount > 10){ // more than 10 loops without lock
           gpsPower(0); //turn GPS off
           radio1.write(0x07, 0x01); // turn tx off
           delay(20*1000); // Sleep for 20 seconds
           radio1.write(0x07, 0x08); // turn tx on
           delay(1000);
           rtty_txstring("GPS_REBOOT\n");
           radio1.write(0x07, 0x01); // turn tx off
           gpsPower(1); //turn GPS on
           delay(10*1000);
           PSMgps();
           radio1.write(0x07, 0x08); // turn tx on
           lockcount = 0;
        }
        pollADC(true);
        prepData();
        //Occasionally the GPS doesn't properly respond with useful data, we need to try and filter
        // this out - here we look for a change in latitude (which should occur even at rest due to 
        // GPS drift and also we'll look for a change in time (total_time = hour+mins+secs to create
        // a relatively unique number).
        //if (lat != oldLat && total_time != old_total_time ) {
        //radio1.write(0x07, 0x08); // turn tx on
        //delay(1000);
        rtty_txstring(superbuffer); // Main string transmit
        //}
        //old_total_time = total_time;

      }//End of the normal flight loop
        
      //This is float loop
      state = 'U'; // Up == float
      
      pollADC(false);
      prepData();
      gpsPower(0); //turn GPS off
        
      radio1.write(0x07, 0x08); // turn tx on
      delay(8000);
      rtty_txstring(superbuffer);
      rtty_txstring(superbuffer);
      rtty_txstring(superbuffer);
      radio1.write(0x07, 0x01); // turn tx off
      delay(5*60000); // Sleep for 15 minutes
      radio1.write(0x07, 0x08); // turn tx on
      delay(2000);
      radio1.write(0x07, 0x01); // turn tx off
      gpsPower(1); //turn GPS on
      startGPS = millis();
      delay(30*1000);
    } else { // GPS on, but no lock
      state = 'L'; // No Lock
      radio1.write(0x07, 0x08); // turn tx on
      delay(1000);
      rtty_txstring("LLLL\n");
      radio1.write(0x07, 0x01); // turn tx off
      delay(8*1000);
    }
  }
    
}
