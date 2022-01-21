/*
 * Author: JP Meijers
 * Date: 2016-09-02
 *
 * Transmit GPS coordinates via TTN. This happens as fast as possible, while still keeping to
 * the 1% duty cycle rules enforced by the RN2483's built in LoRaWAN stack. Even though this is
 * allowed by the radio regulations of the 868MHz band, the fair use policy of TTN may prohibit this.
 *
 * CHECK THE RULES BEFORE USING THIS PROGRAM!
 *
 * CHANGE ADDRESS!
 * Change the device address, network (session) key, and app (session) key to the values
 * that are registered via the TTN dashboard.
 * The appropriate line is "myLora.initABP(XXX);" or "myLora.initOTAA(XXX);"
 * When using ABP, it is advised to enable "relax frame count".
 *
 * This sketch assumes you are using the Sodaq One V4 node in its original configuration.
 *
 * This program makes use of the Sodaq UBlox library, but with minor changes to include altitude and HDOP.
 *
 * LED indicators:
 * Blue: Busy transmitting a packet
 * Green waiting for a new GPS fix
 * Red: GPS fix taking a long time. Try to go outdoors.
 *
 * To decode the binary payload, you can use the following
 * javascript decoder function. It should work with the TTN console.
 *
function Decoder(bytes, port) {
  // Decode an uplink message from a buffer CAE230835FC200001D
  // (array) of bytes to an object of fields.
  var decoded = {};

  // if (port === 1) decoded.led = bytes[0];
  decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
  decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;

  decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
  decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;

  var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
  var sign = bytes[6] & (1 << 7);
  if(sign)
  {
    decoded.alt = 0xFFFF0000 | altValue;
  }
  else
  {
    decoded.alt = altValue;
  }

  decoded.hdop = bytes[8] / 10.0;

  return decoded; 
}

Version of message type

function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  
  var bytes_size = bytes.length;
  var decoded = {};

  if(bytes_size > 1){
    
    // if (port === 1) decoded.led = bytes[0];
    decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.lat = (decoded.lat / 0xFFFFFF * 180) - 90;
    decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.lon = (decoded.lon / 0xFFFFFF * 360) - 180;
    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    //var altValue = (((bytes[6]<<8)>>>0) + bytes[7]) / 10.0;
    var sign = bytes[6] & (1 << 7);
    if(sign)
    {
      decoded.alt = 0xFFFF0000 | altValue;
    }
    else
    {
      decoded.alt = altValue;
    }
    decoded.hdop = bytes[8] / 10.0;
  }
  else{
    decoded.ack = bytes[0];
  }
  return decoded;
}

// function Decoder(bytes, port) {
//   // Decode an uplink message from a buffer
//   // (array) of bytes to an object of fields.
  
//   var decoded = "";
  
//   for(i in bytes){
//     decoded += String.fromCharCode(bytes[i]);
//   }
  

//   // if (port === 1) decoded.led = bytes[0];
//   var res = decoded.replace(/'/g, '"');
//   res = res.replace("HDOP", "hdop");
//   var myObj = JSON.parse(res); 
  
//   var decodedd = {};
//   decodedd.altitude = myObj.altitude;
//   decodedd.hdop = myObj.hdop;
//   decodedd.latitude = myObj.latitude;
//   decodedd.longitude = myObj.longitude;
//   decodedd.ttf = myObj.ttf;


//   return decodedd;
// }

*
*/

/* Co Author: Frank Montenij

I, Frank Montenij moved the code in a repository so it is compatible with the arduino IDE to upload the code.
Furthermore I moved the code to connect to TTN to a seperate function and added the timer functionality. 
I also added the function request event 
and changed receive_event so it is able to support the request from the half-duplex system if LoRaWAN has reach.
Another change I made in the receive event function is that it now supports DAB_id's to be bigger than 255.

*/
#include "Custom_GPS.h"
#include "rn2xx3.h"
#include <Wire.h>

// Create an instance of the rn2xx3 library,
// Giving Serial1 as stream to use for communication with the radio
rn2xx3 myLora(Serial1);

String toLog;
uint8_t txAck[2];
uint8_t txGPS[11];
uint8_t txRSSI[4];
uint8_t* rxBuffer;
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
int dr = 4;
boolean flagRx = false;
boolean succes = false;
boolean repeater = false;
bool join_result = false;

void receiveEvent(int howMany);
void requestEvent();

void setup()
{
    SerialUSB.begin(57600);
    Serial1.begin(57600);

    // Make sure usb serial connection is available,
    // or after 10s go on anyway for 'headless' use of the node.
    while ((!SerialUSB) && (millis() < 10000));

    SerialUSB.println("SODAQ LoRaONE TTN Mapper starting");
// Initialize GPS
    sodaq_gps.init(GPS_ENABLE);

    // LED pins as outputs. HIGH=Off, LOW=On
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_BLUE, HIGH);
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, HIGH);
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_GREEN, HIGH);
    
    initialize_radio();

    // Transmit a startup message
    //myLora.tx("TTN Mapper on Sodaq One");

    // Enable next line to enable debug information of the sodaq_gps
    //sodaq_gps.setDiag(SerialUSB);

    // Set the datarate/spreading factor at which we communicate.
    // DR5 is the fastest and best to use. DR0 is the slowest.
    myLora.setDR(dr);

    Wire.begin(4);                // join i2c bus with address #4
    Wire.onReceive(receiveEvent); // register event
    Wire.onRequest(requestEvent);
}

void initialize_radio()
{
  delay(100); //wait for the RN2xx3's startup message
  while(Serial1.available()){
    Serial1.read();
  }

  // Print out the HWEUI so that we can register it via ttnctl
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    SerialUSB.println("Communication with RN2xx3 unsuccessful. Power cycle the Sodaq One board.");
    delay(10000);
    hweui = myLora.hweui();
  }
  SerialUSB.print("When using OTAA, register this DevEUI: ");
  SerialUSB.println(hweui);
  SerialUSB.print("RN2xx3 firmware version:");
  SerialUSB.println(myLora.sysver());

  // Configure your keys and join the network
  SerialUSB.println("Trying to join TTN");

  // By implementing connect_to_TTN with a timer the function can be used for when the raspberry pi wants to know if this device can reach TTN
  // and be used for connecting to TTN in the initialize radio function where it needs to keep trying until it connects. 
  join_result = connect_to_TTN();
  while(!join_result)
  {
    join_result = connect_to_TTN();
  }
  
  SerialUSB.println("Now trying GPS fix");
  // Scan until we have a succesful GPS fix
//  while(!sodaq_gps.scan(true))
//  {
//    digitalWrite(LED_RED, LOW);
//    SerialUSB.println("No GPS fix, doing another scan");
//    digitalWrite(LED_RED, HIGH);
//  }
//
//  SerialUSB.println("Successfully GPS fix");
}

bool connect_to_TTN() 
{
//  const char *devAddr = "";
//  const char *nwkSKey = "";
//  const char *appSKey = "";
    //  ABP: initABP(const char *devAddr, const char *nwkSKey, const char *appSKey);
//  join_result = myLora.initABP(devAddr, appSKey, nwkSKey);

  String AppEUI = "APPEUI FROM TTN APPLICATION";
  String AppKey = "APPKEY FROM TTN APLLICATION";
  String DevEUI = "DEVEUI FROM TTN APPLICATION";
  //  OTAA: initOTAA(String AppEUI, String AppKey, String DevEUI);
  join_result = myLora.initOTAA(AppEUI, AppKey, DevEUI);

  unsigned long current_time = millis();
  unsigned long previous_time = millis();
  unsigned long timeout = 10000; // 10000 is 10000 milliseconds so 10 seconds and it represents the amount of seconds the loop has to connect to lora before it fails.
  
  while(!join_result && (current_time - previous_time < timeout))
  {
    SerialUSB.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(1000); // Delay a second before retry
    digitalWrite(LED_BLUE, LOW);
    join_result = myLora.init();
    digitalWrite(LED_BLUE, HIGH);
    current_time = millis();
  }

  if (join_result) 
  {
    SerialUSB.println("Successfully joined TTN");
    return true;
  } else
  {
    SerialUSB.println("Unsuccesfully joined TTN");
    return false;
  }
}

void loop()
{
  
  if(flagRx) 
  { 
    if(rxBuffer[2] == 1)
    {
      transmitAck();
      SerialUSB.println(F("TEST1"));
    } else if(rxBuffer[2] == 2)
    {
//      transmitGPS();
      SerialUSB.println(F("TEST2"));
    } else if(rxBuffer[2] == 3)
    {
//      while(repeater){
//      transmitGPS();
//      delay(9000); // Delay a minute before retry
//      }
      SerialUSB.println(F("TEST3"));
    } else if(rxBuffer[2] == 4)
    {
//      transmitSignal();
      SerialUSB.println(F("TEST4"));    
    }
    flagRx = false;
    succes = true;
  }
}

uint16_t get_dab_id() {
  uint16_t dab_id = rxBuffer[0] + rxBuffer[1] * 256;
  return dab_id;
}

void receiveEvent(int howMany)
{
  rxBuffer = (uint8_t*)malloc(howMany * sizeof(uint8_t));
  for(int i = 0; i < howMany; i++){
    rxBuffer[i] = Wire.read();
    SerialUSB.print(i);
    SerialUSB.print(": ");
    SerialUSB.println(rxBuffer[i]);
  }

//    //rxBuffer[howMany] = '\0';  // assume it is a string, add zero-terminator
  flagRx = true;    // set flag, we received something.
  succes = false;
  if(rxBuffer[2] == 3)
  {
    repeater = true;
  }else{
    repeater = false;
  }

  if (howMany == 1 && rxBuffer[0] == 1)
  {
    SerialUSB.println("HAS_REACH");
    join_result = connect_to_TTN();
    flagRx = false;
    free(rxBuffer);
  }
}

void requestEvent()
{
  // If the Sodaq One can connect to TTN it has_reach so return true else return false.
  Wire.write(join_result);
  Wire.write(succes);
}

void transmitGPS()
{
  SerialUSB.println("Waiting for GPS fix");

  // Turn off GREEN and turn on RED led to indicate GPS usage
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, LOW);

  //Turn on GPS
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);
  sodaq_gps.init(GPS_ENABLE);
  
  // Scan until we have a succesful GPS fix
  while(!sodaq_gps.scan(true))
  {
    digitalWrite(LED_RED, LOW);
    SerialUSB.println("No GPS fix, doing another scan");
    digitalWrite(LED_RED, HIGH);
  }

  digitalWrite(LED_RED, LOW);

  LatitudeBinary = ((sodaq_gps.getLat() + 90) / 180) * 16777215;
  LongitudeBinary = ((sodaq_gps.getLon() + 180) / 360) * 16777215;


  txGPS[0] = get_dab_id();  //DAB+ ID
  txGPS[1] = rxBuffer[2];  //Message type
  txGPS[2] = ( LatitudeBinary >> 16 ) & 0xFF;
  txGPS[3] = ( LatitudeBinary >> 8 ) & 0xFF;
  txGPS[4] = LatitudeBinary & 0xFF;

  txGPS[5] = ( LongitudeBinary >> 16 ) & 0xFF;
  txGPS[6] = ( LongitudeBinary >> 8 ) & 0xFF;
  txGPS[7] = LongitudeBinary & 0xFF;

  altitudeGps = sodaq_gps.getAlt();
  txGPS[8] = ( altitudeGps >> 8 ) & 0xFF;
  txGPS[9] = altitudeGps & 0xFF;

  hdopGps = sodaq_gps.getHDOP()*10;
  txGPS[10] = hdopGps & 0xFF;

//  toLog = "";
//  for(size_t i = 0; i<sizeof(txBuffer); i++)
//  {
//    char buffer[3];
//    sprintf(buffer, "%02x", txBuffer[i]);
//    toLog = toLog + String(buffer);
//  }
//  SerialUSB.print(" hex ");
//  SerialUSB.println(toLog);

  digitalWrite(LED_RED, HIGH);
  // Turn on blue to indicate Lora usage
  digitalWrite(LED_BLUE, LOW);
  myLora.txBytes(txGPS, sizeof(txGPS));
  digitalWrite(LED_BLUE, HIGH);

  SerialUSB.println("TX done");
  // Cycle between datarate 0 and 5
  //dr = (dr + 1) % 6;
  //myLora.setDR(dr);
  if(!repeater)
  {
    free(rxBuffer);
  }

  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event   
}

void transmitAck()
{

    txAck[0] = get_dab_id();
    txAck[1] = rxBuffer[2];
    
    // Turn on blue to indicate Lora usage
    digitalWrite(LED_BLUE, LOW);
    myLora.txBytes(txAck, sizeof(txAck));
    digitalWrite(LED_BLUE, HIGH);
    SerialUSB.println("TX done");
    // Cycle between datarate 0 and 5
    //dr = (dr + 1) % 6;
    //myLora.setDR(dr);
    free(rxBuffer);
//    Wire.begin(4);                // join i2c bus with address #4
//    Wire.onReceive(receiveEvent); // register event   
}

void transmitSignal()
{
    txRSSI[0] = get_dab_id();
    txRSSI[1] = rxBuffer[2];
    txRSSI[2] = rxBuffer[3];
    txRSSI[3] = 1;

    // Turn on blue to indicate Lora usage
    digitalWrite(LED_BLUE, LOW);
    myLora.txBytes(txRSSI, sizeof(txRSSI));
    digitalWrite(LED_BLUE, HIGH);
    SerialUSB.println("TX done");
    // Cycle between datarate 0 and 5
    //dr = (dr + 1) % 6;
    //myLora.setDR(dr);
    free(rxBuffer);
    Wire.begin(4);                // join i2c bus with address #4
    Wire.onReceive(receiveEvent); // register event   
}

/*
 * function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  
  var bytes_size = bytes.length;
  var decoded = {};

  if(bytes[1] == 2 | bytes[1] == 3){
    
    // if (port === 1) decoded.led = bytes[0];
    decoded.ack = bytes[0];
    decoded.msgtype = bytes[1];
    decoded.lat = ((bytes[2]<<16)>>>0) + ((bytes[3]<<8)>>>0) + bytes[4];
    decoded.lat = (decoded.lat / 0xFFFFFF * 180) - 90;
    decoded.lon = ((bytes[5]<<16)>>>0) + ((bytes[6]<<8)>>>0) + bytes[7];
    decoded.lon = (decoded.lon / 0xFFFFFF * 360) - 180;
    var altValue = ((bytes[8]<<8)>>>0) + bytes[9];
    //var altValue = (((bytes[6]<<8)>>>0) + bytes[7]) / 10.0;
    var sign = bytes[8] & (1 << 7);
    if(sign)
    {
      decoded.alt = 0xFFFF0000 | altValue;
    }
    else
    {
      decoded.alt = altValue;
    }
    decoded.hdop = bytes[10] / 10.0;
  }
  else if(bytes[1] == 1){
    decoded.ack = bytes[0];
    decoded.msgtype = bytes[1];
  }
  else if(bytes[1] == 4){
    decoded.ack = bytes[0];
    decoded.msgtype = bytes[1];
    decoded.rssi = bytes[2];
    decoded.snr = bytes[3];

    
  }
  return decoded;
}
*/
