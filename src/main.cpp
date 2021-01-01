#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include "timer.h"
#include "lsm.h"
#include "motor.h"

int port = 23;  //Port number
WiFiServer server(port);
WiFiClient client;

#define textBuffSize 34 //length of longest command string plus two spaces for CR + LF
char textBuff[textBuffSize]; //someplace to put received text
int charsReceived = 0;

/* boolean connectFlag = 0; //we'll use a flag separate from client.connected so we can recognize when a new connection has been created
unsigned long timeOfLastActivity; //time in milliseconds of last activity
unsigned long allowedConnectTime = 600000; //ten minutes timeout after no activity from client */
 
//Constants
const char *ssid = "CayganRepeater20MBPS";  //Enter your wifi SSID
const char *password = "########";       //Enter your wifi Password
const int eSize = 1024;                  //this is the eeprom size
const int MotorType = FWDREV;     //Please uncomment this line for the L298N DC motor driver.
const int SensorType = LSM303DLHC;  //Please uncomment this line to use the LSM303DLHC sensor.
#define SerialPort Serial           //Please uncomment this line to use the USB port.
#define WINDUP_LIMIT 450            //Sets the total number of degrees azimuth rotation in any direction before resetting to zero
const int enableA = 0; //D3  not use for FWDREV
const int enableB = 2; //D4 not use for FWDREV
const int elFwdPin = 14;   //D5
const int elRevPin = 12;   //D6
const int azFwdPin = 13;   //D7
const int azRevPin = 15;   //D8
//const int gndPin = D4;   //Makes a convenient ground pin adjacent to the speaker pin
//const int spkPin = 9;    //Attach a piezo buzzer to this pin. It beeps when new calibration data arrives.
//Motor drive gains. These set the amount of motor drive close to the set point
const int azGain = 25;   //Azimuth motor gain
const int elGain = 25;   //Elevation motor gain

//Filter constants
const float azAlpha = 0.5; //Alpha value for AZ motor filter: Decrease to slow response time and reduce motor dither.
const float elAlpha = 0.7; // default 0.5 Alpha value for EL motor filter: Decrease to slow response time and reduce motor dither.
const float lsmAlpha = 0.05; //Alpha value for sensor filter: Decrease to slow response time and ease calibration process.

//Modes
enum Modes {tracking, monitoring, demonstrating, calibrating, debugging, pausing , clearing};    //Rotator controller modes

//Global variables
float az;               //Antenna azimuth
float el;               //Antenna elevation
String line;            //Command line
float azSet;            //Antenna azimuth set point
float elSet;            //Antenna elevation set point
float azLast;           //Last antenna azimuth reading
float elLast;           //Last antenna element reading
float azWindup;         //Antenna windup angle from startup azimuth position
float azOffset;         //Antenna azimuth offset for whole revolutions
bool windup;            //Antenna windup condition
float azSpeed;          //Antenna azimuth motor speed
float elSpeed;          //Antenna elevation motor speed
float azError;          //Antenna azimuth error
float elError;          //Antenna elevation error
float azInc;            //AZ increment for demo mode
float elInc;            //EL increment for demo mode
Modes mode;             //Rotator mode

//Objects
//Motor driver object: Mot xxMot(Driver-Type, Filter-Alpha, Gain, Fwd-Pin, Rev/Dir-Pin)
Mot azMot(MotorType, azAlpha, azGain, azFwdPin, azRevPin); //AZ motor driver object
Mot elMot(MotorType, elAlpha, elGain, elFwdPin, elRevPin); //EL motor driver object

//LSM sensor object: Lsm lsm(Sensor-Type, Filter-Alpha)
Lsm lsm(SensorType,lsmAlpha);

//Non-blocking Timer object
Timer t1(100);

//EEPROM
void save() {
  //Save the calibration data to EEPROM
  EEPROM.begin(eSize);
  yield();
  EEPROM.put(0, lsm.cal);
  EEPROM.commit();
}
void restore() {
  //Restore the calibration data from EEPROM
  EEPROM.begin(eSize);
  yield();
  EEPROM.get(0, lsm.cal);
}
void clear() {
  //Clear the EEPROM
  EEPROM.begin(eSize);
  for (int i = 0; i < eSize; i++) {
  EEPROM.write(i, 0);
  yield();
  EEPROM.commit();
  }
}

//Functions
float diffAngle(float a, float b) {
  //Calculate the acute angle between two angles in -180..180 degree format
  float diff = a - b;
  if (diff < -180) diff += 360;
  if (diff > 180) diff -= 360;
  return diff;
}

void printDebug(void) {
  //Print raw sensor data
  client.print(lsm.mx); client.print(",");
  client.print(lsm.my); client.print(",");
  client.print(lsm.mz); client.print(",");
  client.print(lsm.gx); client.print(",");
  client.print(lsm.gy); client.print(",");
  client.println(lsm.gz);
}

void printCal(void) {
  //Print the calibration data
  client.print(lsm.cal.md, 1); client.print(",");
  client.print(lsm.cal.me.i, 1); client.print(",");
  client.print(lsm.cal.me.j, 1); client.print(",");
  client.print(lsm.cal.me.k, 1); client.print(",");
  client.print(lsm.cal.ge.i, 1); client.print(",");
  client.print(lsm.cal.ge.j, 1); client.print(",");
  client.print(lsm.cal.ge.k, 1); client.print(",");
  client.print(lsm.cal.ms.i, 1); client.print(",");
  client.print(lsm.cal.ms.j, 1); client.print(",");
  client.print(lsm.cal.ms.k, 1); client.print(",");
  client.print(lsm.cal.gs.i, 1); client.print(",");
  client.print(lsm.cal.gs.j, 1); client.print(",");
  client.println(lsm.cal.gs.k, 1);
}

void calibrate() {
  //Process raw accelerometer and magnetometer samples
  bool changed = lsm.calibrate();
  //Print any changes and beep the speaker to facilitate manual calibration
  if (changed) {
    //digitalWrite(spkPin, HIGH);     //Sound the piezo buzzer
    printCal();                     //Print the calibration data
  }
}

void reset(bool getCal) {
  //Reset the rotator, initialize its variables and optionally get the stored calibration
  azSet = 100.0; // This is initial AZ where your rotator should point to upon power up, default is 0
  elSet = 60.0; // This is initial EL where your rotator should point to upon power up, default is 0
  line = "";
  azLast = 0.0;
  elLast = 0.0;
  azWindup = 0.0;
  azOffset = 0.0;
  azSpeed = 0.0;
  elSpeed = 0.0;
  mode = tracking;
  windup = false;
  if (getCal) restore();
  azError = 0.0;
  elError = 0.0;
  azInc = 0.05;
  elInc = 0.05;
  t1.reset(100);
  printCal();
  lsm.calStart(); //Reset the axis calibration objects
}

void printMon(float az, float el, float azSet, float elSet, float azWindup, float azError, float elError) {
  //Print the monitor data
  client.print(az, 0); client.print(",");
  client.print(el, 0); client.print(",");
  client.print(azSet, 0); client.print(",");
  client.print(elSet, 0); client.print(",");
  client.print(azWindup, 0); client.print(",");
  client.print(windup); client.print(",");
  client.print(azError, 0); client.print(",");
  client.println(elError, 0);
}

void printAzEl() {
  //Print the rotator feedback data in Easycomm II format
  client.print("AZ");
  client.print((az < 0) ? (az + 360) : az, 1);
  client.print(" EL");
  client.print(el, 1);
  client.print("\n");
}

void getWindup(bool *windup,  float *azWindup, float *azOffset, float *azLast, float *elLast, float az, float elSet) {
  //Get the accumulated windup angle from the home position (startup or last reset position) and set the windup state if greater than the limit.
  //Get the raw difference angle between the current and last azimuth reading from the sensor
  float azDiff = az - *azLast;

  //Detect crossing South: azDiff jumps 360 for a clockwise crossing or -360 for an anticlockwise crossing
  //Increment the azimuth offset accordingly
  if (azDiff < -180) *azOffset += 360;
  if (azDiff > 180) *azOffset -= 360;

  //Save the current azimuth reading for the next iteration
  *azLast = az;

  //Compute the azimuth wind-up angle, i.e. the absolute number of degrees from the home position
  *azWindup = az + *azOffset;

  //Detect a windup condition where the antenna has rotated more than 450 degrees from home
  if (abs(*azWindup) > WINDUP_LIMIT) *windup = true;    //Set the windup condition - it resets later when the antenna nears home

  //Perform the anti-windup procedure at the end of each pass - This is overkill unless you absolutely don't want anti-windup during a pass
   // if (elSet <= 0)
     // if (elLast > 0)
     //   if (mode == tracking) {
      //    *windup = true;
       // }

  //Save the current elevation reading for the next iteration
  *elLast = elSet;
}

void getAzElDemo(float *azSet, float *elSet, float *azInc, float *elInc) {
  //Autoincrement the azimuth and elevation to demo the rotator operation
  if (*azSet > 180.0) *azInc = -*azInc;
  if (*azSet < -180.0) *azInc = -*azInc;
  if (*elSet > 90.0) *elInc = -*elInc;
  if (*elSet < 0.0) *elInc = -*elInc;
  *azSet += *azInc;
  *elSet += *elInc;
  client.print(*azSet, 0); client.print(",");
  client.println(*elSet, 0);
}

void getAzElError(float *azError, float *elError, bool *windup, float *azSet, float elSet, float az, float el) {
  //Compute the azimuth and elevation antenna pointing errors, i.e. angular offsets from set positions
  //Compute the azimuth antenna pointing error: Normally via the shortest path; opposite if windup detected.
  if (*windup) {                               //Check for a windup condition
    //To unwind the antenna set an azError in the appropriate direction to home
    *azError = constrain(azWindup, -180, 180); //Limit the maximum azimuth error to -180..180 degrees
    //Cancel the windup condition when the antenna is within 180 degrees of home (Actually 175 degrees to avoid rotation direction ambiguity)
    //Set a zero home position by default, but return azumith control to the computer if still connected
    if (abs(*azError) < 175) *windup = false; //Cancel windup and permit computer control
  }
  else {
    //Compute the normal azimuth antenna pointing error when there is no windup condition
    *azError = diffAngle(az, *azSet);
  }

  //Compute the elevation antenna pointing error
  *elError = diffAngle(el, elSet);
}

void processPosition() {
  //Perform the main operation of positioning the rotator under different modes
  //Read the accelerometer and magnetometer
  lsm.readGM();
  switch (mode) {
    case debugging:
      printDebug(); //Print the raw sensor data for debug purposes
      break;
    case calibrating:
      calibrate();  //Process calibration data
      break;
    case pausing:
      azMot.halt(); //Stop the AZ motor
      elMot.halt(); //Stop the EL motor
      break;
    case clearing:
      clear();
      break;
    default:
      lsm.getAzEl();  //Get the azimuth and elevation of the antenna               //Get the antenna AZ and EL
      az = lsm.az;
      el = lsm.el;
      getWindup(&windup, &azWindup, &azOffset, &azLast, &elLast, az, elSet);      //Get the AZ windup angle and windup state
      if (mode == demonstrating) getAzElDemo(&azSet, &elSet, &azInc, &elInc);     //Set the AZ and EL automatically if in demo mode
      getAzElError(&azError, &elError, &windup, &azSet, elSet, az, el);           //Get the antenna pointing error
      if (mode == monitoring) printMon(az, el, azSet, elSet, azWindup, azError, elError); //Print the data if in monitor mode
  }
}

void processMotors() {
  //Drive the motors to reduce the azimuth and elevation error to zero
  azMot.drive(azError);
  elMot.drive(elError);
}

void processUserCommands(String line) {
  //Process user commands
  //User command type 1: r, b, m, c, a, d, s, d, h, p or e<decl> followed by a carriage return
  //User command type 2: <az> <el> followed by a carriage return
  String param;                                           //Parameter value
  int firstSpace;                                         //Position of the first space in the command line
  char command = line.charAt(0);                          //Get the first character
  switch (command) {                                      //Process type 1 user commands
    case 'x':                                             //Clear EEPROM content command
      client.println("Clearing EEPROM in progress");
      clear();
      reset(true);
      client.println("Clear complete");
      break;
    case 'r':                                             //Reset command
      client.println("Reset in progress");
      reset(true);  //Reset the rotator and load calibration from EEPROM
      client.println("Reset complete");
      break;
    case 'b':                                             //Debug command
      client.println("Debugging in progress: Press 'a' to abort");
      mode = debugging;
      t1.reset(100);
      break;
    case 'm':                                             //Monitor command
      client.println("Monitoring in progress: Press 'a' to abort");
      mode = monitoring;
      t1.reset(100);
      break;
    case 'c':                                             //Calibrate command
      client.println("Calibration in progress: Press 'a' to abort or 's' to save");
      reset(false); //Reset the rotator, but don't load calibration from EEPROM
      mode = calibrating;
      t1.reset(50);
      break;
    case 'a':                                             //Abort command
      mode = tracking;
      t1.reset(100);
      reset(true);
      client.println("Function aborted");
      break;
    case 'e':                                             //Magnetic declination command
      param = line.substring(1);                          //Get the second parameter
      lsm.cal.md = param.toFloat();
      break;
    case 's':                                             //Save command
      save();
      reset(true);
      client.println("Calibration saved");
      break;
    case 'd':                                             //Demo command
      client.println("Demo in progress: Press 'a' to abort");
      t1.reset(50);
      mode = demonstrating;
      break;
    case 'h':                                             //Help command
      client.println("################################################");
      client.println("");
      client.println("Commands:");
      client.println("az el -- Manual position control (0..360 0..90)");
      client.println("r  -- Reset, warning this will reset the eeprom, or else you will need to recalibrate the sensor");
      client.println("eNN.N -- Magnetic Declination");
      client.println("c -- Calibrate");
      client.println("s -- Save");
      client.println("a -- Abort");
      client.println("d -- Demo");
      client.println("b -- Debug");
      client.println("m -- Monitor");
      client.println("p -- Pause");
      client.println("q -- Restart controller");
      client.println("");
      break;
    case 'q':
      ESP.restart();                                      //Reboot the ESP
    case 'p':                                             //Pause command
      if (mode == pausing) {
        mode = tracking;
      } else {
        mode = pausing;
        client.println("Paused");
      }
      break;
    default:                                              //Process type 2 user commands
      firstSpace = line.indexOf(' ');                     //Get the index of the first space
      param = line.substring(0, firstSpace);              //Get the first parameter
      azSet = param.toFloat();                            //Get the azSet value
      param = line.substring(firstSpace + 1);             //Get the second parameter
      elSet = param.toFloat();                            //Get the elSet value
  }
}

void processEasycommCommands(String line) {
  //Process Easycomm II rotator commands
  //Easycomm II position command: AZnn.n ELnn.n UP000 XXX DN000 XXX\n
  //Easycomm II query command: AZ EL \                    // When you click engage from Gpredict, hamlib will send this
  String param;                                           //Parameter value
  int firstSpace;                                         //Position of the first space in the command line
  int secondSpace;                                        //Position of the second space in the command line
  if (line.startsWith("AZ EL")) {                         //Query command received
    printAzEl();                                          //Send the current Azimuth and Elevation
  } else {
    if (line.startsWith("AZ")) {                          //Position command received: Parse the line.
      firstSpace = line.indexOf(' ');                     //Get the position of the first space
      secondSpace = line.indexOf(' ', firstSpace + 1);    //Get the position of the second space
      param = line.substring(2, firstSpace);              //Get the first parameter
      azSet = param.toFloat();                            //Set the azSet value
      if (azSet > 180) azSet = azSet - 360;               //Convert 0..360 to -180..180 degrees format
      param = line.substring(firstSpace + 3, secondSpace);//Get the second parameter
      elSet = param.toFloat();                            //Set the elSet value
    }
  }
}

void printErrorMessage() {
  client.println("Unrecognized command.  ? for help.");
}

void resetCharsReceived() {
  //timeOfLastActivity = millis();
  delay(20);
  client.flush();
  charsReceived = 0; //count of characters received
  //client.print(">");
}

/* void checkConnectionTimeout() {
  if(millis() - timeOfLastActivity > allowedConnectTime) {
    client.println();
    client.println("Timeout disconnect.");
    client.stop();
    connectFlag = 0;
  }
} */

void getReceivedText() {
  char c;
  int charsWaiting;
  // copy waiting characters into textBuff
  //until textBuff full, CR received, or no more characters
  charsWaiting = client.available();
  do {
    c = client.read();
    textBuff[charsReceived] = c;
    charsReceived++;
    charsWaiting--;
  }
  while(charsReceived <= textBuffSize && c != 0x0d && charsWaiting > 0);

  //if LF found go look at received text and execute command
  // in telnet client, if you press enter it will LF (\n)
  if(c == '\n') {
    line = textBuff;
    processEasycommCommands(line);
    // after completing command, flush client and reset character received
    resetCharsReceived();
  }
  //if CR carriage return found go look at received text and execute command
  // in telnet client, typing A then followed by combination of control+m+m will send CRLF
  if(c == 0x0d) {   // this is the hex of carriage return
    line = textBuff;
    processUserCommands(line);
    // after completing command, flush client and reset character received
    resetCharsReceived();
  }

  // if textBuff full without reaching a CR, print an error message
  if(charsReceived >= textBuffSize) {
    client.println();
    printErrorMessage();
    resetCharsReceived();
  }
  // if textBuff not full and no CR, do nothing else;
  // go back to loop until more characters are received
}

void handleTelnet(){
  if (server.hasClient()){
    SerialPort.println("Client is now connected");

  	// client is connected
    if (!client || !client.connected()){
      if(client) client.stop();          // client disconnected
      //connectFlag = 1;
      client = server.available();
      client.println("\nDV2JB ESP8266 admin control");
      client.println("type 'h' for help");
      resetCharsReceived();
    }
    else {
      server.available().stop();  // have client, block new conections
    }
  }
  if (client && client.connected() && client.available()){
    // client input processing
    while(client.available())
      getReceivedText(); // pass through
      // do other stuff with client input here
  } 
}

/* void checkConnectFlag() {
  // check to see if connection has timed out
  if(connectFlag) checkConnectionTimeout();
} */

////// Setup ///////
void setup() {
  reset(true);                                            //Initialize the rotor system Reset the rotator and load configuration from EEPROM
  SerialPort.begin(19200);                                //Initialize the serial port
  lsm.begin();                                            //Initialize the sensor
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);                        //HIGH is OFF

  WiFi.mode(WIFI_STA);                                    // To avoid esp8266 from going to AP mode
  WiFi.begin(ssid, password); //Connect to wifi
  SerialPort.println("Connecting to Wifi");               // Wait for connection  
  while (WiFi.status() != WL_CONNECTED) {   
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    SerialPort.print("."); 
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(500);
  }
  SerialPort.println("");
  SerialPort.print("Connected to "); SerialPort.println(ssid);
  SerialPort.print("IP address: "); SerialPort.println(WiFi.localIP());  
  digitalWrite(LED_BUILTIN, LOW); // LED is on when connected to wifi
  server.begin();
  SerialPort.print("Open Telnet and connect to IP:"); SerialPort.print(WiFi.localIP()); SerialPort.print(" on port ");
  SerialPort.println(port);

  ArduinoOTA.onStart([]() {                               //ArduinoOTA code starts
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    SerialPort.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    SerialPort.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    SerialPort.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    SerialPort.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      SerialPort.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      SerialPort.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      SerialPort.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      SerialPort.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      SerialPort.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

////// Loop ////////
void loop() { 
  //Repeat continuously - this is for rotator
  t1.execute(&processPosition);                                   //Process position only periodically
  processMotors();                                                //Process motor drive
  handleTelnet();                                                 //Handles the processing of commands
  //checkConnectFlag();                                             //Check connectflag
  ArduinoOTA.handle();                                            //Handles ArduinoOTA
}  