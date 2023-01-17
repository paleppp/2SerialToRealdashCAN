/**
 * ============================================================================
 *  Name        : 2SerialToRealdashCAN
 *  Author      : paleppp
 *  Created     : 15.1.2023
 *  
 * Reading Speeduino Secondary Serial with broader realtime dataset command "n".
 * Then send data to Realdash CAN via UART-USB adapter. You can also add custom inputs to Realdash with this.
 * Code is designed to use ESP32 DEVKIT V1 38PIN Board and 8 channel Optocoupler
 *
 * This code is based of 'Realdash Arduino CAN example' and pazi88's 'Serial3toBMWcan' and modified by paleppp.
 * ============================================================================
**/

#include "Arduino.h"

#define NOTHING_RECEIVED        0
#define R_MESSAGE               1
#define N_MESSAGE               110

// -------------------------------------------------------------------------------------------------------------


// Struct table for storing the incoming data from Speeduino Serial
// -------------------------------------------------------------------------------------------------------------
struct statuses {
  uint8_t secl;
  uint8_t status1;
  uint8_t engine;
  uint8_t dwell;
  uint16_t MAP;
  uint8_t IAT;
  uint8_t coolant;
  uint8_t batCorrection;
  uint8_t battery10;
  uint8_t O2;
  uint8_t egoCorrection;
  uint8_t iatCorrection;
  uint8_t wueCorrection;
  uint16_t RPM;
  uint8_t TAEamount;
  uint8_t corrections;
  uint8_t ve;
  uint8_t afrTarget;
  uint16_t PW1;
  uint8_t tpsDOT;
  uint8_t advance;
  uint8_t TPS;
  uint16_t loopsPerSecond;
  uint16_t freeRAM;
  uint8_t boostTarget;
  uint8_t boostDuty;
  uint8_t spark;
  uint16_t rpmDOT;
  uint8_t ethanolPct;
  uint8_t flexCorrection;
  uint8_t flexIgnCorrection;
  uint8_t idleLoad;
  uint8_t testOutputs;
  uint8_t O2_2;
  uint8_t baro;
  uint16_t CANin_1;
  uint16_t CANin_2;
  uint16_t CANin_3;
  uint16_t CANin_4;
  uint16_t CANin_5;
  uint16_t CANin_6;
  uint16_t CANin_7;
  uint16_t CANin_8;
  uint16_t CANin_9;
  uint16_t CANin_10;
  uint16_t CANin_11;
  uint16_t CANin_12;
  uint16_t CANin_13;
  uint16_t CANin_14;
  uint16_t CANin_15;
  uint16_t CANin_16;
  uint8_t tpsADC;
  uint8_t getNextError;
  uint8_t launchCorrection;
  uint16_t PW2;
  uint16_t PW3;
  uint16_t PW4;
  uint8_t status3;
  uint8_t engineProtectStatus;
  uint8_t fuelLoad;
  uint8_t ignLoad;
  uint8_t injAngle;
  uint8_t idleDuty;
  uint8_t CLIdleTarget;
  uint8_t mapDOT;
  uint8_t vvt1Angle;
  uint8_t vvt1TargetAngle;
  uint8_t vvt1Duty;
  uint8_t flexBoostCorrection;
  uint8_t baroCorrection;
  uint8_t ASEValue;
  uint8_t vss;
  uint8_t gear;
  uint8_t fuelPressure;
  uint8_t oilPressure;
  uint8_t wmiPW;
  uint8_t status4;
  uint8_t vvt2Angle;
  uint8_t vvt2TargetAngle;
  uint8_t vvt2Duty;
  uint8_t outputsStatus;
  uint8_t fuelTemp;
  uint8_t fuelTempCorrection;
  uint8_t VE1;
  uint8_t VE2;
  uint8_t advance1;
  uint8_t advance2;
  uint8_t nitrous_status;
  uint8_t TS_SD_Status;
};
statuses currentStatus;
// -------------------------------------------------------------------------------------------------------------

// Speeduino Serial connection stuff
// -------------------------------------------------------------------------------------------------------------
static uint32_t oldtime=millis(); // for the timeout
uint8_t SpeedyResponse[120]; //The data buffer for the serial3 data. This is longer than needed, just in case
// -------------------------------------------------------------------------------------------------------------

// Variables for inputs 
// -------------------------------------------------------------------------------------------------------------

 unsigned int digitalPins = 0;
 unsigned int HighBeam = 0;
 unsigned int BrakeWarn = 0;
 unsigned int OilPress = 0;
 unsigned int AlternatorWarn = 0;
 unsigned int FuelLevel;
 
 int analogPins[7] = {0};
// -------------------------------------------------------------------------------------------------------------

// Pinout list
// ------------------------------------------------------------------------------------------------------------- 

 #define HIGHBEAM 27
 #define TURNSIGLEFT 32
 #define TURNSIGRIGHT 33
 #define BRAKEWARN 0

 #define OILPRESS 0
 #define ALTERNATORWARN 0
 #define FUELLEVEL 0

// -------------------------------------------------------------------------------------------------------------

bool data_error; //indicator for the data from speeduino being ok.
bool responseSent; // to keep track if we have responded to data request or not.
bool newData; // This tells if we have new data available from speeduino or not.
bool ascMSG; // ASC message received.

bool debug = false; // Debugging Serial monitor on/off

uint8_t SerialState,canin_channel,currentCommand;

uint32_t start;
uint32_t end;

// -------------------------------------------------------------------------------------------------------------

// Speeduino data request command 
void requestData() {

  Serial2.write("n"); // Send n to request real time data
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  // This runs only if Debugging is set = true
  if (debug == true) {
  Serial.begin(115200); // For Debugging in Serial Monitor
    } else { 
  }

  //Init Serials for reading Speeduino and sending data further to Realdash
  Serial1.begin(115200,SERIAL_8N1,14,12); // For Android Tablet via FT312D
  Serial2.begin(115200);  // baudrate for Speeduino is 115200

  // Start with sensible values for some of these variables.
  SerialState = NOTHING_RECEIVED;
  data_error = false;
  responseSent = false;
  newData = false;
  ascMSG = false;

  // Init Pins
  pinMode(HIGHBEAM, INPUT_PULLUP);
  pinMode(TURNSIGLEFT, INPUT_PULLUP);
  pinMode(TURNSIGRIGHT, INPUT_PULLUP);
  pinMode(BRAKEWARN, INPUT_PULLUP);
  pinMode(OILPRESS, INPUT_PULLUP);
  pinMode(ALTERNATORWARN, INPUT_PULLUP);
  pinMode(FUELLEVEL, INPUT);

  Serial.println ("Version date: 15.1.2023"); // To see from debug serial when is used code created.
  requestData(); // all set. Start requesting data from speeduino
}

// display the needed values in serial monitor for debugging
void displayData(){

  Serial.print (" RPM= "); Serial.print (currentStatus.RPM); Serial.print("\t");
  Serial.print (" CLT= "); Serial.print (currentStatus.coolant); Serial.print("\t");
  Serial.print (" TPS="); Serial.print (currentStatus.TPS); Serial.print("\t");
  Serial.print (" O2="); Serial.print (currentStatus.O2); Serial.print("\t");
  Serial.print (" IAT="); Serial.print (currentStatus.IAT); Serial.print("\t");
  Serial.print (" BAT="); Serial.print (currentStatus.battery10); Serial.print("\t");
  Serial.print (" VE1 ="); Serial.print (currentStatus.VE1); Serial.println("\t");



}
// necessary conversion for the data before sending to CAN BUS
void processData(){

  unsigned int tempRPM;
  data_error = false; // set the received data as ok

  currentStatus.secl = SpeedyResponse[0];
  currentStatus.status1 = SpeedyResponse[1];
  currentStatus.engine = SpeedyResponse[2];
  currentStatus.dwell = SpeedyResponse[3];
  currentStatus.MAP = ((SpeedyResponse [5] << 8) | (SpeedyResponse [4]));
  currentStatus.IAT = SpeedyResponse[6];
  currentStatus.coolant = SpeedyResponse[7]-40;
  currentStatus.batCorrection = SpeedyResponse[8];
  currentStatus.battery10 = SpeedyResponse[9];
  currentStatus.O2 = SpeedyResponse[10];
  currentStatus.egoCorrection = SpeedyResponse[11];
  currentStatus.iatCorrection = SpeedyResponse[12];
  currentStatus.wueCorrection = SpeedyResponse[13];

  currentStatus.RPM = ((SpeedyResponse [15] << 8) | (SpeedyResponse [14]));
  
  currentStatus.TAEamount = SpeedyResponse[16];
  currentStatus.corrections = SpeedyResponse[17];
  currentStatus.ve = SpeedyResponse[18];
  currentStatus.afrTarget = SpeedyResponse[19];

  currentStatus.PW1 = ((SpeedyResponse [21] << 8) | (SpeedyResponse [20]));
  
  currentStatus.tpsDOT = SpeedyResponse[22];
  currentStatus.advance = SpeedyResponse[23];
  currentStatus.TPS = SpeedyResponse[24];

  currentStatus.loopsPerSecond = ((SpeedyResponse [26] << 8) | (SpeedyResponse [25]));
  currentStatus.freeRAM = ((SpeedyResponse [28] << 8) | (SpeedyResponse [27]));

  currentStatus.boostTarget = SpeedyResponse[29];
  currentStatus.boostDuty = SpeedyResponse[30];
  currentStatus.spark = SpeedyResponse[31];

  currentStatus.rpmDOT = ((SpeedyResponse [33] << 8) | (SpeedyResponse [32]));
  currentStatus.ethanolPct = SpeedyResponse[34];

  currentStatus.flexCorrection = SpeedyResponse[35];
  currentStatus.flexIgnCorrection = SpeedyResponse[36];
  currentStatus.idleLoad = SpeedyResponse[37];
  currentStatus.testOutputs = SpeedyResponse[38];
  currentStatus.O2_2 = SpeedyResponse[39];
  currentStatus.baro = SpeedyResponse[40];
  currentStatus.CANin_1 = ((SpeedyResponse [42] << 8) | (SpeedyResponse [41]));
  currentStatus.CANin_2 = ((SpeedyResponse [44] << 8) | (SpeedyResponse [43]));
  currentStatus.CANin_3 = ((SpeedyResponse [46] << 8) | (SpeedyResponse [45]));
  currentStatus.CANin_4 = ((SpeedyResponse [48] << 8) | (SpeedyResponse [47]));
  currentStatus.CANin_5 = ((SpeedyResponse [50] << 8) | (SpeedyResponse [49]));
  currentStatus.CANin_6 = ((SpeedyResponse [52] << 8) | (SpeedyResponse [51]));
  currentStatus.CANin_7 = ((SpeedyResponse [54] << 8) | (SpeedyResponse [53]));
  currentStatus.CANin_8 = ((SpeedyResponse [56] << 8) | (SpeedyResponse [55]));
  currentStatus.CANin_9 = ((SpeedyResponse [58] << 8) | (SpeedyResponse [57]));
  currentStatus.CANin_10 = ((SpeedyResponse [60] << 8) | (SpeedyResponse [59]));
  currentStatus.CANin_11 = ((SpeedyResponse [62] << 8) | (SpeedyResponse [61]));
  currentStatus.CANin_12 = ((SpeedyResponse [64] << 8) | (SpeedyResponse [63]));
  currentStatus.CANin_13 = ((SpeedyResponse [66] << 8) | (SpeedyResponse [65]));
  currentStatus.CANin_14 = ((SpeedyResponse [68] << 8) | (SpeedyResponse [67]));
  currentStatus.CANin_15 = ((SpeedyResponse [70] << 8) | (SpeedyResponse [69]));
  currentStatus.CANin_16 = ((SpeedyResponse [72] << 8) | (SpeedyResponse [71]));
  currentStatus.tpsADC = SpeedyResponse[73];
  currentStatus.getNextError = SpeedyResponse[74];
  currentStatus.launchCorrection = SpeedyResponse[75];
  currentStatus.PW2 = ((SpeedyResponse [77] << 8) | (SpeedyResponse [76]));
  currentStatus.PW3 = ((SpeedyResponse [79] << 8) | (SpeedyResponse [78]));
  currentStatus.PW4 = ((SpeedyResponse [81] << 8) | (SpeedyResponse [80]));
  currentStatus.status3 = SpeedyResponse[82];
  currentStatus.engineProtectStatus = SpeedyResponse[83];
  currentStatus.fuelLoad = ((SpeedyResponse [85] << 8) | (SpeedyResponse [84]));
  currentStatus.ignLoad = ((SpeedyResponse [87] << 8) | (SpeedyResponse [86]));
  currentStatus.injAngle = ((SpeedyResponse [89] << 8) | (SpeedyResponse [88]));
  currentStatus.idleDuty = SpeedyResponse[90];
  currentStatus.CLIdleTarget = SpeedyResponse[91];
  currentStatus.mapDOT = SpeedyResponse[92];
  currentStatus.vvt1Angle = SpeedyResponse[93];
  currentStatus.vvt1TargetAngle = SpeedyResponse[94];
  currentStatus.vvt1Duty = SpeedyResponse[95];
  currentStatus.flexBoostCorrection = ((SpeedyResponse [97] << 8) | (SpeedyResponse [96]));
  currentStatus.baroCorrection = SpeedyResponse[98];
  currentStatus.ASEValue = SpeedyResponse[99];
  currentStatus.vss = ((SpeedyResponse [101] << 8) | (SpeedyResponse [100]));
  currentStatus.gear = SpeedyResponse[102];
  currentStatus.fuelPressure = SpeedyResponse[103];
  currentStatus.oilPressure = SpeedyResponse[104];
  currentStatus.wmiPW = SpeedyResponse[105];
  currentStatus.status4 = SpeedyResponse[106];
  currentStatus.vvt2Angle = SpeedyResponse[107];
  currentStatus.vvt2TargetAngle = SpeedyResponse[108];
  currentStatus.vvt2Duty = SpeedyResponse[109];
  currentStatus.outputsStatus = SpeedyResponse[110];
  currentStatus.fuelTemp = SpeedyResponse[111];
  currentStatus.fuelTempCorrection = SpeedyResponse[112];
  currentStatus.VE1 = SpeedyResponse[113];
  currentStatus.VE2 = SpeedyResponse[114];
  currentStatus.advance1 = SpeedyResponse[115];
  currentStatus.advance2 = SpeedyResponse[116];
  currentStatus.nitrous_status = SpeedyResponse[117];
  currentStatus.TS_SD_Status = SpeedyResponse[118];

  // check if received values makes sense and convert those if all is ok.
  if (currentStatus.RPM < 9000 && data_error == false)  // the engine will not probaply rev over 8000 RPM
  {
    tempRPM = currentStatus.RPM; // RPM conversion factor for e46/e39 cluster
  }
}
// Handle the serial "n" command reading and processing and sending data further into Realdash
void HandleN()
{
  //Serial.print ("n "); // Just for debug
  data_error = false;

    //Serial2.read(); // 'n' <-- DEC 110  << this must be uncommented if wanted to use the old method of reading serial.
    Serial2.read(); // 0x32 <-- DEC 50
    uint8_t nLength = Serial2.read(); // Length of data to follow
    for (int i=0; i<nLength ; i++) {
    SpeedyResponse[i] = Serial2.read();
    }

  // do the necessary processing for received data
  processData();
  if (debug == true) {
    displayData(); // only required for debugging
    } else { 
  }

  requestData();  // restart data reading
  oldtime = millis(); // zero the timeout
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.

  ReadDigitalStatuses();
  ReadAnalogStatuses();
  SendCANFramesToSerial();
}
// Not used for now! Can be used to send commands into Speeduino.
void HandleR()
{
  Serial.println ("R ");
  byte tmp0;
  byte tmp1;
  canin_channel = Serial2.read();
  tmp0 = Serial2.read();  // read in lsb of source can address
  tmp1 = Serial2.read();  // read in msb of source can address
  //CanAddress = tmp1<<8 | tmp0 ;
  //SendDataToSpeeduino();  // send the data to speeduino
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}

void ReadSerial()
{
  currentCommand = Serial2.read(); // 'n' <-- DEC 110
  switch (currentCommand)
  {
    case 'n':  // Speeduino sends data in n-message
      SerialState = N_MESSAGE;
    break;
    case 'R':  // Speeduino requests data in n-message
      SerialState = R_MESSAGE;
    break;
    default:
      Serial.print ("Not an N or R message ");
      Serial.println (currentCommand);
    break;
  }
}



void loop()
{
  switch(SerialState) {
    case NOTHING_RECEIVED:
      if (Serial2.available() > 0) { ReadSerial(); }  // read bytes from serial3 to define what message speeduino is sending.
      break;
    case N_MESSAGE:
      if (Serial2.available() >= 118) { HandleN(); }  // read and process the n-message from serial3, when it's fully received.
      break;
    case R_MESSAGE:
      //if (Serial2.available() >= 118) { HandleN(); }  // read and process the A-message from serial3, when it's fully received.
      break;
      }

  if ( (millis()-oldtime) > 500) { // timeout if for some reason reading serial3 fails
    oldtime = millis();
    Serial.println ("Timeout from speeduino!");
    requestData();                // restart data reading
    
  }

// ---- OLD METHOD ---- | This requires to commenting the whole >> switch (SerialState) << out! | Also uncomment >> Serial2.read << from HandleN
  // if (Serial2.available() >= 117) { HandleN(); }  // read and process the A-message from serial3, when it's fully received.

// These are included in HandleN for now.
  // ReadDigitalStatuses();
  // ReadAnalogStatuses();
  // SendCANFramesToSerial();

// ---- OLD METHOD ----
}


void ReadDigitalStatuses()
{
  // read status of digital pins (1-9)
  digitalPins = 0;
  int bitposition = 0;

  for (int i=1; i<9; i++)
  {
    if (digitalRead(i) == HIGH) digitalPins |= (1 << bitposition);
    bitposition++;
  }

// Turn signals readouts, Uses same bit but different position! Bit 1 <-- Left Turn signal | Right Turn signal --> Bit 2 || Emergency signals --> Bit 3
  if (digitalRead(TURNSIGLEFT) == LOW) digitalPins |= (1 << bitposition);
    bitposition++;
  if (digitalRead(TURNSIGRIGHT) == LOW) digitalPins |= (1 << bitposition);
    bitposition+2;


// High beam readout and invert output to LOW -> 1 and HIGH -> 0 // Easier in Realdash!
  if (digitalRead(HIGHBEAM) == LOW) {
    HighBeam = 1;
  } else {
    HighBeam = 0;
  }

// Brake warning light readout and invert output to LOW -> 1 and HIGH -> 0 // Easier in Realdash!
  if (digitalRead(BRAKEWARN) == LOW) {
    BrakeWarn = 1;
  } else {
    BrakeWarn = 0;
  }

// Oil pressure warning light readout and invert output to LOW -> 1 and HIGH -> 0 // Easier in Realdash!
  if (digitalRead(OILPRESS) == LOW) {
    OilPress = 1;
  } else {
    OilPress = 0;
  }

// Alternator warning light readout and invert output to LOW -> 1 and HIGH -> 0 // Easier in Realdash!
  if (digitalRead(ALTERNATORWARN) == LOW) {
    AlternatorWarn = 1;
  } else {
    AlternatorWarn = 0;
  }
}


void ReadAnalogStatuses()
{
  // read analog pins (0-7)
  for (int i=0; i<7; i++)
  {
    analogPins[i] = analogRead(i);
  }
}


void SendCANFramesToSerial()
{
  byte buf[8];

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes containing data in a manner
  // described by the RealDash custom channel description XML file
  // all multibyte values are handled as little endian by default.
  // endianess of the values can be specified in XML file if it is required to use big endian values

  // build first CAN frame

 memcpy(buf, &currentStatus.secl, 2);
 memcpy(buf + 1, &currentStatus.status1, 2);
 memcpy(buf + 2, &currentStatus.engine, 2);
 memcpy(buf + 3, &currentStatus.dwell, 2);
 SendCANFrameToSerial(3200, buf);

// build second CAN frame

 memcpy(buf, &currentStatus.MAP, 2);
 memcpy(buf + 2, &currentStatus.IAT, 2);
 memcpy(buf + 4, &currentStatus.coolant, 2);
 memcpy(buf + 6, &currentStatus.batCorrection, 2);
 SendCANFrameToSerial(3201, buf);

// build third CAN frame

 memcpy(buf, &currentStatus.battery10, 2);
 memcpy(buf + 2, &currentStatus.O2, 2);
 memcpy(buf + 4, &currentStatus.egoCorrection, 2);
 memcpy(buf + 6, &currentStatus.iatCorrection, 2);
 SendCANFrameToSerial(3202, buf);

// build fourth CAN frame

 memcpy(buf, &currentStatus.wueCorrection, 2);
 memcpy(buf + 2, &currentStatus.RPM, 2);
 memcpy(buf + 4, &currentStatus.TAEamount, 2);
 memcpy(buf + 6, &currentStatus.corrections, 2);
 SendCANFrameToSerial(3203, buf);

// build fifth CAN frame

 memcpy(buf, &currentStatus.ve, 2);
 memcpy(buf + 2, &currentStatus.afrTarget, 2);
 memcpy(buf + 4, &currentStatus.PW1, 2);
 memcpy(buf + 6, &currentStatus.tpsDOT, 2);
 SendCANFrameToSerial(3204, buf);

// build sixth CAN frame

 memcpy(buf, &currentStatus.advance, 2);
 memcpy(buf + 2, &currentStatus.TPS, 2);
 memcpy(buf + 4, &currentStatus.loopsPerSecond, 2);
 memcpy(buf + 6, &currentStatus.freeRAM, 2);
 SendCANFrameToSerial(3205, buf);

// build seventh CAN frame

 memcpy(buf, &currentStatus.boostTarget, 2);
 memcpy(buf + 2, &currentStatus.boostDuty, 2);
 memcpy(buf + 4, &currentStatus.spark, 2);
 memcpy(buf + 6, &currentStatus.rpmDOT, 2);
 SendCANFrameToSerial(3206, buf);

// build eighth CAN frame

 memcpy(buf, &currentStatus.ethanolPct, 2);
 memcpy(buf + 2, &currentStatus.flexCorrection, 2);
 memcpy(buf + 4, &currentStatus.flexIgnCorrection, 2);
 memcpy(buf + 6, &currentStatus.idleLoad, 2);
 SendCANFrameToSerial(3207, buf);

// build ninth CAN frame

 memcpy(buf, &currentStatus.testOutputs, 2);
 memcpy(buf + 2, &currentStatus.O2_2, 2);
 memcpy(buf + 4, &currentStatus.baro, 2);
 memcpy(buf + 6, &currentStatus.CANin_1, 2);
 SendCANFrameToSerial(3208, buf);

// build tenth CAN frame

 memcpy(buf, &currentStatus.CANin_2, 2);
 memcpy(buf + 2, &currentStatus.CANin_3, 2);
 memcpy(buf + 4, &currentStatus.CANin_4, 2);
 memcpy(buf + 6, &currentStatus.CANin_5, 2);
 SendCANFrameToSerial(3209, buf);

// build eleventh CAN frame

 memcpy(buf, &currentStatus.CANin_6, 2);
 memcpy(buf + 2, &currentStatus.CANin_7, 2);
 memcpy(buf + 4, &currentStatus.CANin_8, 2);
 memcpy(buf + 6, &currentStatus.CANin_9, 2);
 SendCANFrameToSerial(3210, buf);

// build twelfth CAN frame

 memcpy(buf, &currentStatus.CANin_10, 2);
 memcpy(buf + 2, &currentStatus.CANin_11, 2);
 memcpy(buf + 4, &currentStatus.CANin_12, 2);
 memcpy(buf + 6, &currentStatus.CANin_13, 2);
 SendCANFrameToSerial(3211, buf);

// build thirteenth CAN frame

 memcpy(buf, &currentStatus.CANin_14, 2);
 memcpy(buf + 2, &currentStatus.CANin_15, 2);
 memcpy(buf + 4, &currentStatus.CANin_16, 2);
 memcpy(buf + 6, &currentStatus.tpsADC, 2);
 SendCANFrameToSerial(3212, buf);

// build fourteenth CAN frame

 memcpy(buf, &currentStatus.getNextError, 2);
 memcpy(buf + 2, &currentStatus.launchCorrection, 2);
 memcpy(buf + 4, &currentStatus.PW2, 2);
 memcpy(buf + 6, &currentStatus.PW3, 2);
 SendCANFrameToSerial(3213, buf);

// build fifteenth CAN frame

 memcpy(buf, &currentStatus.PW4, 2);
 memcpy(buf + 2, &currentStatus.status3, 2);
 memcpy(buf + 4, &currentStatus.engineProtectStatus, 2);
 memcpy(buf + 6, &currentStatus.fuelLoad, 2);
 SendCANFrameToSerial(3214, buf);

// build sixteenth CAN frame

 memcpy(buf, &currentStatus.ignLoad, 2);
 memcpy(buf + 2, &currentStatus.injAngle, 2);
 memcpy(buf + 4, &currentStatus.idleDuty, 2);
 memcpy(buf + 6, &currentStatus.CLIdleTarget, 2);
 SendCANFrameToSerial(3215, buf);

// build seventeenth CAN frame

 memcpy(buf, &currentStatus.mapDOT, 2);
 memcpy(buf + 2, &currentStatus.vvt1Angle, 2);
 memcpy(buf + 4, &currentStatus.vvt1TargetAngle, 2);
 memcpy(buf + 6, &currentStatus.vvt1Duty, 2);
 SendCANFrameToSerial(3216, buf);

// build eighteenth CAN frame

 memcpy(buf, &currentStatus.flexBoostCorrection, 2);
 memcpy(buf + 2, &currentStatus.baroCorrection, 2);
 memcpy(buf + 4, &currentStatus.ASEValue, 2);
 memcpy(buf + 6, &currentStatus.vss, 2);
 SendCANFrameToSerial(3217, buf);

// build nineteenth CAN frame

 memcpy(buf, &currentStatus.gear, 2);
 memcpy(buf + 2, &currentStatus.fuelPressure, 2);
 memcpy(buf + 4, &currentStatus.oilPressure, 2);
 memcpy(buf + 6, &currentStatus.wmiPW, 2);
 SendCANFrameToSerial(3218, buf);

// build twentieth CAN frame

 memcpy(buf, &currentStatus.status4, 2);
 memcpy(buf + 2, &currentStatus.vvt2Angle, 2);
 memcpy(buf + 4, &currentStatus.vvt2TargetAngle, 2);
 memcpy(buf + 6, &currentStatus.vvt2Duty, 2);
 SendCANFrameToSerial(3219, buf);

// build twenty-first CAN frame

 memcpy(buf, &currentStatus.outputsStatus, 2);
 memcpy(buf + 2, &currentStatus.fuelTemp, 2);
 memcpy(buf + 4, &currentStatus.fuelTempCorrection, 2);
 memcpy(buf + 6, &currentStatus.VE1, 2);
 SendCANFrameToSerial(3220, buf);

// build twenty-second CAN frame

 memcpy(buf, &currentStatus.VE2, 2);
 memcpy(buf + 2, &currentStatus.advance1, 2);
 memcpy(buf + 4, &currentStatus.advance2, 2);
 memcpy(buf + 6, &currentStatus.nitrous_status, 2);
 SendCANFrameToSerial(3221, buf);

  // Pins next

  // build Arduino pin CAN frame, Arduino digital pins and few spesific digital inputs.
  memcpy(buf, &digitalPins, 2);
  memcpy(buf + 2, &HighBeam, 2);
  memcpy(buf + 4, &BrakeWarn, 2);
  memcpy(buf + 6, &OilPress, 2);

  // write 2nd CAN frame to serial
  SendCANFrameToSerial(3222, buf);

    // build Arduino pin CAN frame, Arduino digital pin and 3 analog values
  memcpy(buf, &AlternatorWarn, 2);
  memcpy(buf + 2, &analogPins[1], 2);
  memcpy(buf + 4, &analogPins[2], 2);
  memcpy(buf + 6, &analogPins[3], 2);

  // write 2nd CAN frame to serial
  SendCANFrameToSerial(3223, buf);
  
/*
  // build 3rd CAN frame, rest of Arduino analog values
  memcpy(buf, &analogPins[3], 2);
  memcpy(buf + 2, &analogPins[4], 2);
  memcpy(buf + 4, &analogPins[5], 2);
  memcpy(buf + 6, &analogPins[6], 2);

  // write 3rd CAN frame to serial
  SendCANFrameToSerial(3223, buf);
*/
//}


/*
  // build 4th frame, this is a text extension frame
  // only send once at 1000 loops
  if (currentStatus.status1 == 0)
  {
    SendTextExtensionFrameToSerial(3224, "Inj 1 Status ON");
  }
  else if (currentStatus.status1 == 4)
  {
    SendTextExtensionFrameToSerial(3225, "DFCO On");
  }
  else if (currentStatus.engine < 1)
  {
    SendTextExtensionFrameToSerial(3226, "Not Running");
  }
  else if (currentStatus.engine == 1)
  {
    SendTextExtensionFrameToSerial(3226, "Running");
  }
  else if (currentStatus.engine == 2)
  {
    SendTextExtensionFrameToSerial(3226, "Cranking");
  }
  */
}


void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial1.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  Serial1.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  Serial1.write(frameData, 8);
}

/*
void SendTextExtensionFrameToSerial(unsigned long canFrameId, const char* text)
{
  if (text)
  {
    // the 4 byte identifier at the beginning of each CAN frame
    // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
    const byte textExtensionBlockTag[4] = { 0x55, 0x33, 0x22, 0x11 };
    Serial1.write(textExtensionBlockTag, 4);

    // the CAN frame id number (as 32bit little endian value)
    Serial1.write((const byte*)&canFrameId, 4);

    // text payload
    Serial1.write(text, strlen(text) + 1);
    // SerialBT.write((const uint8_t *)text, strlen(text) + 1);
  }
}*/