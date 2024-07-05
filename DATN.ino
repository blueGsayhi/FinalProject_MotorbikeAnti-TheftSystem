#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER  1024
#define RADIUS_OF_EARTH 6371000

// Include libraries
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <MPU6050_tockn.h>
#include <ThingSpeak.h>
#include <TinyGPS++.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

// Define UART RX - TX pins
#define A7670C_RX_PIN   16
#define A7670C_TX_PIN   17
#define NEO6M_RX_PIN    26
#define NEO6M_TX_PIN    27

// Define the input pins
#define LOCK 4
#define UNLOCK 0
#define FIND 15
#define ENGINE_CONTROL 2
#define V_SEN 5

// Define the output pins
#define RELAY  12
#define BUZZER 18
    
// Initialize - UART - I2C communication
HardwareSerial A7670C(1);
HardwareSerial NEO6M(2);
MPU6050 MPU6065(Wire);
TinyGPSPlus GPS;
TinyGsm modem(A7670C);
TinyGsmClient client(modem);

// Declare millis variables
unsigned long previousMillisBuzzer = 0;
unsigned long previousMillisBuzzerTilt = 0;
unsigned long lastMillisVibration = 0;
unsigned long lastMillisEnginePress = 0;
unsigned long startMillisTilt = 0;
unsigned long previousMillisGps = 0;
unsigned long lastGpsUpdateTime = 0;
unsigned long previousSendThingspeakMillis = 0;
unsigned long previousReciveThingspeakMillis = 0;
unsigned long lastMillisEngineSms = 0;

// Declare the vibration sensor variable
volatile int vibrationCount = 0;

// Declare state variables
bool buzzerState = LOW;
bool lockState = LOW;
bool engineState = LOW;
bool gpsState = false;
bool replyVibration = false;
bool replyTilt = false;
bool replyDistance = false;
bool confirmEngineRf = false;
bool confirmEngineSms = false;
bool trackingLocation = false;
bool distanceState = false;
bool buzzerPermisson = true;

int flagApp = 0;

// Declare phone number variables
String mainPhone = "";
String subPhone = "";
String trackingPhone = "";

// Declare location coordinate variables
String latitude = "";
String longitude = "";
String latitudeLock = "";
String longitudeLock = "";
double distance;

void setup() {
  Serial.begin(115200);
  A7670C.begin(115200, SERIAL_8N1, A7670C_RX_PIN, A7670C_TX_PIN);
  NEO6M.begin(9600, SERIAL_8N1, NEO6M_RX_PIN, NEO6M_TX_PIN);
  EEPROM.begin(512);
  Wire.begin();
  MPU6065.begin();
  MPU6065.calcGyroOffsets(true);
  attachInterrupt(digitalPinToInterrupt(V_SEN), countVibration, RISING);
  pinMode(LOCK, INPUT);
  pinMode(UNLOCK, INPUT);
  pinMode(FIND, INPUT);
  pinMode(ENGINE_CONTROL, INPUT);
  pinMode(V_SEN, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(RELAY, LOW);
  A7670C.println("AT+CRESET");
  delay(10000);
  A7670C.println("AT+CMGF=1");
  delay(500);
  A7670C.println("AT+CNMI=1,2,0,0,0");
  delay(500);
  A7670C.println("AT+CMGD=1,4");
  delay(500);
  A7670C.println("AT+CSCLK=0");
  delay(500);
  ThingSpeak.begin(client);
  delay(500);
  modem.init();
  delay(500);
  modem.waitForNetwork();
  delay(500);
  modem.gprsConnect("m3-world", "", "");
  delay(500);
  mainPhone = readEEPROM(0, 12);
  subPhone = readEEPROM(13, 25);
  Serial.println("\nDS so dien thoai");
  Serial.println("Chinh: " + mainPhone);
  Serial.println("Phu: " + subPhone);
  ThingSpeak.readMultipleFields(2559134, "KZSMBJYZ2NJT2GVJ");
  flagApp = ThingSpeak.getFieldAsInt(3);
  delay(5000);
}

void loop() {
  find();  
  lockUnlock();
  engineControl();
  antiThief();
  location();
  gsm();
  reciveFromThingspeak();
  sendToThingspeak();
}

void find() {
  if(digitalRead(FIND) == HIGH) {
    while(digitalRead(FIND) == HIGH) {};
    buzzer(4, 100);
    vibrationCount = 0;
  }
}

void lockUnlock() {
  while(digitalRead(LOCK) == HIGH && lockState == LOW) {
    buzzer(2, 100);
    vibrationCount = 0;
    if(latitude == "" && longitude == ""){
      distanceState = false;
    }
    else {
      latitudeLock = latitude;
      longitudeLock = longitude;
      distanceState = true;
      distance = 0;
    }
    lockState = HIGH;
  }
  while(digitalRead(UNLOCK) == HIGH && lockState == HIGH) {
    buzzer(1, 100);
    lockState = LOW;
    replyDistance = false;
  }    
}

void engineControl() {
  if (digitalRead(ENGINE_CONTROL) == HIGH && confirmEngineRf == false) {
    while (digitalRead(ENGINE_CONTROL) == HIGH) {}
    confirmEngineRf = true;
    lastMillisEnginePress = millis();
  }
  if (digitalRead(ENGINE_CONTROL) == HIGH && confirmEngineRf == true) {
    while (digitalRead(ENGINE_CONTROL) == HIGH) {}
    engineState = !engineState;
    digitalWrite(RELAY, engineState);
    confirmEngineRf = false;
    buzzer(3, 400);
    vibrationCount = 0;
  }
  if (millis() - lastMillisEnginePress > 5000 && confirmEngineRf == true) {
    confirmEngineRf = false;
  }
  if (millis() - lastMillisEngineSms > 30000 && confirmEngineSms == true) {
    confirmEngineSms = false;
  } 
}

void antiThief() {
  MPU6065.update();
  float angleX = MPU6065.getAngleX();
  float angleY = MPU6065.getAngleY();

  if((abs(angleX) > 35 || abs(angleY) > 35) && lockState == 1) {
    unsigned long currentMillisGyroscope = millis();
    if(startMillisTilt == 0) {
      startMillisTilt = currentMillisGyroscope;
    } 
    if(currentMillisGyroscope - startMillisTilt >= 5000) {
      if(!replyTilt) {
        if(mainPhone != "" && subPhone == "") {
          Reply("Xe cua ban dang bi nga!\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, mainPhone);
        }
        if(mainPhone != "" && subPhone != "") {
          Reply("Xe cua ban dang bi nga!\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, mainPhone);
          delay(2000);
          Reply("Xe cua ban dang bi nga!\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, subPhone);
        }
        replyTilt = true;  
      }
      if(buzzerPermisson) {
        buzzer();
      }
    } 
    if(currentMillisGyroscope - startMillisTilt >= 15000) {
      vibrationCount = 0;
      startMillisTilt = 0;
      digitalWrite(BUZZER, LOW);
      buzzerPermisson = false;
    }
  } 
  else {
    startMillisTilt = 0;
    replyTilt = false;
    buzzerPermisson = true;
  }
  
  unsigned long currentMillisVibration = millis();
  if(vibrationCount >= 200 && (currentMillisVibration - lastMillisVibration <= 10000) && lockState == 1) {
    if(!replyVibration && replyTilt == false) {
      if(mainPhone != "" && subPhone == "") {
        Reply("Xe cua ban dang bi rung lac!\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, mainPhone);
      }
      if(mainPhone != "" && subPhone != "") {
        Reply("Xe cua ban dang bi rung lac!\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, mainPhone);
        delay(2000);
        Reply("Xe cua ban dang bi rung lac!\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, subPhone);
      }
      replyVibration = true;
    }
    buzzer();
  } 
  else if (currentMillisVibration - lastMillisVibration > 10000) {
    vibrationCount = 0;
    replyVibration = false;
  } 

  if(abs(angleX) < 35 && abs(angleY) < 35 && vibrationCount < 200 && lockState == 1) {
    digitalWrite(BUZZER, LOW);
  }
  if(lockState && distance > 200 && replyDistance == false && distanceState == true) {
    digitalWrite(RELAY, HIGH);
    engineState = HIGH;
    if(mainPhone != "" && subPhone == "") {
      Reply("Xe cua ban da bi dat khoi vi tri do!\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, mainPhone);
    }
    if(mainPhone != "" && subPhone != "") {
      Reply("Xe cua ban da bi dat khoi vi tri do!\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, mainPhone);
      delay(2000);
      Reply("Xe cua ban da bi dat khoi vi tri do!\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, subPhone);
    }    
    replyDistance = true;
  }
}

void gsm() {
  while(A7670C.available()){
    String response = A7670C.readString();
    Serial.println(response);
    if(response.indexOf("+CMT:") > -1){
      String callerID = getCallerID(response);
      String cmd = getMsgContent(response);
      if(cmd.equals("dkc")) {
        if(mainPhone == "") {
          mainPhone = callerID;
          saveEEPROM(0, 12, callerID);
          Reply("DK thanh cong SDT chinh: " + mainPhone,callerID);
        } 
        else {
          Reply("Da ton tai SDT chinh duoi " + mainPhone.substring(9,12) ,callerID);
        } 
      }  
      else if(cmd.indexOf("dkp") > -1 && callerID == mainPhone) {
        if(getNumber(cmd) == mainPhone) {
          Reply("Khong the DK SDT phu trung SDT chinh",callerID);
        } 
        else if(subPhone == "" && cmd.length() == 16 && isPhoneDigits(getNumber(cmd)) == true) {
          subPhone = getNumber(cmd);
          saveEEPROM(13, 25, subPhone);
          Reply("DK thanh cong SDT phu: " + subPhone,callerID);
        }
        else if(cmd.length() != 16 || isPhoneDigits(getNumber(cmd)) == false) {
          Reply("Sai dinh dang DK SDT",callerID);
        } 
        else {
          Reply("Da ton tai SDT phu duoi " + subPhone.substring(9,12) ,callerID);
        } 
      }
      else if(cmd.equals("ds") && (callerID == mainPhone || callerID == subPhone)) {
        Reply("Danh sach SDT \nChinh: " + mainPhone + "\n" + "Phu: " + subPhone,callerID);
      } 
      else if(cmd.equals("xoac") && (callerID == mainPhone || callerID == subPhone)) {
        clearEEPROM(0, 12);
        Reply("Da xoa SDT chinh",callerID);
        mainPhone = "";
      } 
      else if(cmd.equals("xoap") && (callerID == mainPhone || callerID == subPhone)) {
        clearEEPROM(13, 25);
        Reply("Da xoa SDT phu",callerID);
        subPhone = "";
      }
      else if(cmd.equals("xoahet") && (callerID == mainPhone || callerID == subPhone)) {
        clearEEPROM(0, 25);
        Reply("Da xoa tat ca SDT",callerID);
        mainPhone = "";
        subPhone = "";
      }
      else if(cmd.equals("batchongtrom") && (callerID == mainPhone || callerID == subPhone)) {
        lockState = HIGH;
        latitudeLock = latitude;
        longitudeLock = longitude;
        Reply("Da bat che do chong trom",callerID);
      }
      else if(cmd.equals("tatchongtrom") && (callerID == mainPhone || callerID == subPhone)) {
        lockState = LOW;
        Reply("Da tat che do chong trom",callerID);
      }
      else if(cmd.equals("khoadongco") && (callerID == mainPhone || callerID == subPhone)) {
        if(digitalRead(RELAY) == HIGH) {
          Reply("Dong co dang duoc khoa!",callerID);
        }
        else {
          confirmEngineSms = true;
          lastMillisEngineSms = millis();
          Reply("De xac nhan khoa dong co nhan: khoadongco y\nDe huy khoa dong co nhan: khoadongco n",callerID);
        } 
      }
      else if(cmd.equals("khoadongco y") && (callerID == mainPhone || callerID == subPhone) && confirmEngineSms == true) {
        digitalWrite(RELAY, HIGH);
        engineState = HIGH;
        Reply("Da khoa dong co", callerID);
      }
      else if(cmd.equals("khoadongco n") && (callerID == mainPhone || callerID == subPhone) && confirmEngineSms == true) {
        Reply("Da huy yeu cau khoa dong co", callerID);
      } 
      else if(cmd.equals("mokhoadongco") && (callerID == mainPhone || callerID == subPhone)) {
        if(digitalRead(RELAY) == LOW) {
          Reply("Dong co khong bi khoa!",callerID);
        }
        else {
          digitalWrite(RELAY, LOW);
          engineState = LOW;
          Reply("Da mo khoa dong co",callerID);
        }
      }
      else if(cmd.equals("khoidonglai") && (callerID == mainPhone || callerID == subPhone)) {
        Reply("Da gui lenh khoi dong lai",callerID);
        delay(5000);
        ESP.restart();
      }
      else if(cmd.equals("guivitri") && (callerID == mainPhone || callerID == subPhone)) {
        Reply("Toa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, callerID);
      }
      else if(cmd.equals("theodoixe") && (callerID == mainPhone || callerID == subPhone)) {
        trackingLocation = true;
        trackingPhone = callerID; 
      } 
      else if(cmd.equals("ngungtheodoixe") && (callerID == mainPhone || callerID == subPhone) && trackingLocation == true) {
        trackingLocation = false;
        Reply("Da ngung theo doi xe", callerID);
      } 
      else if(cmd.equals("cuphap") && (callerID == mainPhone || callerID == subPhone)) {
        Reply("DKC - DK SDT chinh\nDKP=+84********* - DK SDT phu\nDS - DS SDT da DK\nXOAC - Xoa SDT chinh\nXOAP - Xoa SDT phu\nXOAHET - Xoa het SDT\nTATXE - Gui yeu cau tat xe\n", callerID);
        Reply("TATXE Y - Xac nhan tat xe\nTATXE N - Huy yeu cau tat xe\nBATXE - Bat xe may\nGUIVITRI - Gui vi tri xe\nTHEODOIXE - Theo doi xe\nNGUNGTHEODOIXE - Ngung theo doi xe", callerID);
      }
      else if(cmd.equals("trangthai") && (callerID == mainPhone || callerID == subPhone)) {
        String gpsString, lockString, engineString;
        if(gpsState) {
          gpsString = "GPS : Co tin hieu";
        }
        else {
          gpsString = "GPS : Mat tin hieu";
        }

        if(lockState) {
          lockString = "Chong trom : Bat";
        }
        else {
          lockString = "Chong trom : Tat";
        }

        if(engineState) {
          engineString = "Khoa dong co : Khoa";
        }
        else {
          engineString = "Khoa dong co : Mo khoa";
        }
        Reply("Trang thai he thong\n" + gpsString + "\n" + lockString + "\n" + engineString, callerID);
      }
      else {
        Reply("Sai cu phap", callerID);
      } 
      if(cmd.equals("khoadongco") == false) {
        confirmEngineSms = false;
      } 
      A7670C.println("AT+CMGD=1,4");
    }
  }
}


String getCallerID(String buff) {
  unsigned int index, index2;
  index = buff.indexOf("\"");
  index2 = buff.indexOf("\"", index+1);
  String callerID = buff.substring(index+1, index2);
  callerID.trim();
  Serial.println("Caller ID: "+callerID);
  return callerID;
}

String getMsgContent(String buff) {
  unsigned int index, index2;
  index = buff.lastIndexOf("\"");
  index2 = buff.length();
  String command = buff.substring(index+1, index2);
  command.trim();
  command.toLowerCase();
  Serial.println("Command: "+command);
  return command;
}

String getNumber(String text) {
  String temp = text.substring(4, 17);
  temp.trim();
  return temp;
}

bool isPhoneDigits(String str) {
  for(int i = 1; i < str.length(); i++) {
    if(!isdigit(str.charAt(i))) {
      return false;
    } 
  }
  return true;  
}

void Reply(String text, String caller_id) {
 if(caller_id == mainPhone || caller_id == subPhone) {
  A7670C.print("AT+CMGF=1\r");
  A7670C.print("AT+CMGS=\""+caller_id+"\"\r");
  delay(100);
  A7670C.print(text);
  delay(100);
  A7670C.write(0x1A);
  A7670C.println("AT+CMGD=1,4");
  Serial.println("SMS Sent Successfully.");
 } 
}

void location() {
  while(NEO6M.available() > 0) {
    GPS.encode(NEO6M.read());
    if (GPS.location.isUpdated()) {
      latitude = String(GPS.location.lat(),6);
      longitude = String(GPS.location.lng(),6);
      gpsState = true;
      lastGpsUpdateTime = millis();
    }
  }

  if (millis() - lastGpsUpdateTime >= 5000) {
    gpsState = false;
  }

  unsigned long currentMillisGps = millis();
  if ((currentMillisGps - previousMillisGps >= 60000) && trackingLocation == true) {
    previousMillisGps = currentMillisGps; 
    Reply("Che do theo doi xe\nToa do: " + latitude + " " + longitude + "\n" + "Dan duong den xe may: https://www.google.com/maps/dir/?api=1&origin&destination=" + latitude + "," + longitude, trackingPhone);
  }

  double lat = atof(latitude.c_str());
  double latLock = atof(latitudeLock.c_str());
  double lon = atof(longitude.c_str());
  double lonLock = atof(longitudeLock.c_str());
  latLock = latLock * PI / 180;
  lonLock = lonLock * PI / 180;
  lat = lat * PI / 180;
  lon = lon * PI / 180;
  double dLat = lat - latLock;
  double dLon = lon - lonLock;
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(latLock) * cos(lat) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  distance = RADIUS_OF_EARTH * c;
}

void buzzer(int n, int s) {
  for(int i = 0; i<n; i++) {
    digitalWrite(BUZZER,HIGH);
    delay(s);
    digitalWrite(BUZZER,LOW);
    delay(s);
  }
}

void buzzer() {
  unsigned long currentMillisBuzzer = millis();
  if(currentMillisBuzzer - previousMillisBuzzer >= 100) {
    previousMillisBuzzer = currentMillisBuzzer;
    buzzerState = !buzzerState;
    digitalWrite(BUZZER, buzzerState);
  } 
}

void countVibration() {
  if(vibrationCount < 200) {
    vibrationCount++;
    lastMillisVibration = millis();
  }
}

// This function used to send data to ThingSpeak
void sendToThingspeak() {
  unsigned long currentSendThingspeakMillis = millis();
  if(currentSendThingspeakMillis - previousSendThingspeakMillis >= 20000) {
    previousSendThingspeakMillis = currentSendThingspeakMillis;
    ThingSpeak.setField(1, latitude);
    ThingSpeak.setField(2, longitude);
    ThingSpeak.setField(3, mainPhone.substring(1, 12));
    ThingSpeak.setField(4, subPhone.substring(1, 12));
    ThingSpeak.setField(5, gpsState);
    ThingSpeak.setField(6, lockState);
    ThingSpeak.setField(7, engineState);
    ThingSpeak.setField(8, "1234");
    ThingSpeak.writeFields(2518028, "1CLMYSY6R1Y2AQ1F");
  }
}

void reciveFromThingspeak() {
  unsigned long currentReciveThingspeakMillis = millis();
  if(currentReciveThingspeakMillis - previousReciveThingspeakMillis >= 10000) {
    previousReciveThingspeakMillis = currentReciveThingspeakMillis;
    ThingSpeak.readMultipleFields(2559134, "KZSMBJYZ2NJT2GVJ");
    int LockStateApp = ThingSpeak.getFieldAsInt(1);
    int EngineStateApp = ThingSpeak.getFieldAsInt(2);
    int newflagApp = ThingSpeak.getFieldAsInt(3);
    if(newflagApp != flagApp) {
      flagApp = newflagApp;
      lockState = LockStateApp;
      engineState = EngineStateApp;
      digitalWrite(RELAY, engineState);
      if(lockState == 1) {
        latitudeLock = latitude;
        longitudeLock = longitude;
        distance = 0;
      }
    }
  }
}

// These funcitons used to read, write and delete EEPROM
void saveEEPROM(int fAddr, int lAddr, String data) {
  for (int i = fAddr; i < lAddr; i++) {
    if(fAddr  == 0) {
      EEPROM.write(i, data[i]);
    } 
    else {
      EEPROM.write(i, data[i - 13]);
    } 
  }
  EEPROM.commit();
}

String readEEPROM(int fAddr, int lAddr) {
  String data = "";
  char character;
  for (int i = fAddr; i < lAddr; i++) { 
    character = EEPROM.read(i);
    data += character;
  }
  if(!isPhoneDigits(data)) {
    data = "";
  } 
  return data;
}

void clearEEPROM(int fAddr, int lAddr) {
  for (int i = fAddr; i < lAddr; i++) { 
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
}