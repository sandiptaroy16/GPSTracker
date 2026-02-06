#include <TinyGPS++.h>
#include "BluetoothSerial.h"

HardwareSerial sim800(2);   // UART2 → SIM800L
HardwareSerial gpsSerial(1); // UART1 → GPS
TinyGPSPlus gps;

BluetoothSerial SerialBT;
#define SIM800_PWR_CTRL 26
#define GPS_PWR_CTRL 33
#define SLEEP_TIME_SEC 60  // 5 minutes
#define GPS_TIMEOUT  120000  // 2 min
#define GSM_TIMEOUT 60000
String incomingData = "";
String phoneNumber = "";
String alertNumber = "9874068384";
#define VIB_PIN 27
#define VIBRATION_GPIO GPIO_NUM_27
#define GPS_LED 25
//#define GSM_LED 21
#define ESP32_LED 32

bool vibrationCheck = false;
unsigned long lastSMSTime = 0;
const unsigned long smsCooldown = 20000;

void setup() {
  Serial.begin(9600);  
  delay(500);
  pinMode(GPS_LED, OUTPUT);
  //pinMode(GSM_LED, OUTPUT);
  pinMode(ESP32_LED, OUTPUT); 
  delay(500);
  digitalWrite(ESP32_LED, LOW);
  //digitalWrite(GSM_LED, LOW);
  digitalWrite(GPS_LED, LOW);
  delay(500);

  SerialBT.begin("ESP32_BT_Terminal");
  logMessage("Bluetooth started. Pair ESP32_BT_Terminal");
  blinkLED(ESP32_LED, 2);
  delay(500);

  pinMode(SIM800_PWR_CTRL, OUTPUT);
  pinMode(GPS_PWR_CTRL, OUTPUT);
  digitalWrite(SIM800_PWR_CTRL, LOW);
  digitalWrite(GPS_PWR_CTRL, LOW);
  delay(500);

  digitalWrite(GPS_PWR_CTRL, HIGH);
  delay(3000);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); //Rx, Tx
  delay(500);

  digitalWrite(GPS_LED, HIGH);

  logMessage("Connecting GPS....");
  bool fix = waitForGPSFix();
  if(fix){
    logMessage("GPS service is On");
    blinkLED(GPS_LED, 3);
  }
  else{
    logMessage("GPS not working..");    
  }

  digitalWrite(SIM800_PWR_CTRL, HIGH);
  delay(3000);
  sim800.begin(9600, SERIAL_8N1, 18, 19); //Rx, Tx
  delay(1000);

  digitalWrite(ESP32_LED, HIGH);
  logMessage("Connecting GSM....");

  bool gsmFix = waitForNetwork();
  if(gsmFix){
    initSIM800();    
    blinkLED(ESP32_LED, 3);
  }
  delay(5000);
  
  logMessage("System Ready");

  if(networkRegistered()) {
    checkOfflineSMS();
  }

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_TIME_SEC * 1000000ULL);
  esp_sleep_enable_ext0_wakeup(VIBRATION_GPIO, 1);

  // Check wake-up reason
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  if (reason == ESP_SLEEP_WAKEUP_EXT0) {
    logMessage("Wakeup by vibration");
    //sendSMS(alertNumber, "⚠️ ALERT: Vibration detected!");
    delay(1000);
    makeCall(alertNumber);
    lastSMSTime = millis();
  } else {
    logMessage("Normal wakeup");
  }
}

void logMessage(String message){
  Serial.println(message);
  SerialBT.println(message);
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (SerialBT.available()) {
    String c = SerialBT.readString();
    c.trim();
    sim800.println(c);       // show on USB Serial
    if(c == "gps")
    {
      Serial.println("GPS Command from phone");
      SerialBT.println(GetGPSLocation());
    }
    else if (c == "vibrationoff")
    {
      vibrationCheck = false;
    }
    else if (c == "vibrationon")
    {
      vibrationCheck = true;
    }
    else if(c == "sleep"){
        esp_deep_sleep_start();
    }
    else if (c == "simoff"){
      sim800PowerOff();
    }
  }

  if(vibrationCheck){
      checkVibrationAndCreateAlert();
  }
  
  if(Serial.available()){
    String s = Serial.readString();
    sim800.println(s);
  }

  if (sim800.available()) {
    readFromSim800();
  }
  //sim800PowerOff();
  //gpsPowerOff();
  //esp_deep_sleep_start();
}


void blinkLED(int ledPin, int blinkTimes) {
  pinMode(ledPin, OUTPUT);

  for (int i = 0; i < blinkTimes; i++) {
    digitalWrite(ledPin, HIGH);
    delay(300);
    digitalWrite(ledPin, LOW);
    delay(300);
  }
}


bool waitForGPSFix() {
  unsigned long start = millis();

  while (millis() - start < GPS_TIMEOUT) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    if (gps.location.isValid() &&
        gps.location.age() < 2000 &&
        gps.satellites.value() >= 3) {
      return true; // FIX ACQUIRED
    }
  }
  return false; // TIMEOUT
}

bool waitForNetwork(){
  uint32_t start = millis();
  uint8_t stableCount = 0;
  while (millis() - start < GSM_TIMEOUT)
  {
    sim800.println("AT+CREG?");
    delay(400);
    String resp = "";
    while (sim800.available()) {
      resp += sim800.readStringUntil('\n');
    }
    logMessage(resp);
    // Check registration
    if (resp.indexOf("+CREG:") != -1 && (resp.indexOf(",1") != -1 || resp.indexOf(",5") != -1))
    {
      stableCount++;
      if (stableCount >= 3) {   // 3 consecutive OKs
        logMessage("✅ Network registered (stable)");
        return true;
      }
    }
    else {
      stableCount = 0;
    }
    delay(2000);
  }
  logMessage("❌ Network registration timeout");
  return false;
}

void sim800PowerOff() {
  logMessage("SIM800 POWER OFF");
  sim800.println("AT+CMGD=1,4");
  delay(500);
  // Optional clean shutdown via AT command
  sim800.println("AT+CPOWD=1");
  delay(3000);
  // Cut power completely
  digitalWrite(SIM800_PWR_CTRL, LOW);
  delay(1000);
}

void gpsPowerOff(){
    // Cut power completely
  digitalWrite(GPS_PWR_CTRL, LOW);
  delay(1000);
}
void checkVibrationAndCreateAlert(){
  int vibState = digitalRead(VIB_PIN);

  if (vibState == HIGH) {
    logMessage("Vibration Detected");
    if (millis() - lastSMSTime > smsCooldown) {
      sendSMS(alertNumber, "⚠️ ALERT: Vibration detected!");
      delay(1000);
      makeCall(alertNumber);
      lastSMSTime = millis();
    }
  }
}

void readFromSim800(){
  // Read SIM800 messages
  incomingData = sim800.readString();
  logMessage(incomingData);

  // New SMS indication
  if (incomingData.indexOf("+CMTI") != -1) {
    int index = incomingData.substring(incomingData.lastIndexOf(",") + 1).toInt();
    readSMS(index);
  }
}

void initSIM800() {
  sim800.println("AT");
  delay(500);
  sim800.println("AT+CMGF=1");          // Text mode
  delay(500);
  sim800.println("AT+CSCS=\"GSM\"");
  delay(500);
  sim800.println("AT+CNMI=2,1,0,0,0");  // SMS notify  
  delay(500);
  sim800.println("AT+CPMS=\"SM\",\"SM\",\"SM\"");
  delay(500);
  sim800.println("AT+CHFA=1");   // Loudspeaker
  delay(500);
  sim800.println("AT+CLVL=80");  // Volume
  delay(500);  
}

void readSMS(int index) {
  logMessage("read SMS called");
  sim800.println("AT+CMGR=" + String(index));

  unsigned long t = millis();
  String sms = "";

  while (millis() - t < 3000) {
    while (sim800.available()) {
      sms += char(sim800.read());
    }
  }

  logMessage("📩 RAW SMS:");
  logMessage(sms);
  logMessage(sms);

  // ---- Validate SMS ----
  if (sms.indexOf("+CMGR:") == -1) {
    logMessage("⚠️ Invalid SMS index");
    return;   // do NOT delete
  }

  // ---- Extract phone number ----
  ExtractPhoneNumber(sms);

  // ---- Extract SMS BODY ----
  int headerEnd = sms.indexOf("\n");
  if (headerEnd == -1) return;

  int okPos = sms.lastIndexOf("\nOK");
  if (okPos == -1) okPos = sms.length();

  String body = sms.substring(headerEnd + 1, okPos);
  body.trim();
  body.toUpperCase();

  logMessage("📨 SMS BODY: " + body);

  // ---- Process commands ----
  if (body.indexOf("LOC") != -1) {
    sendLocation(phoneNumber);
  }

  // ---- Delete ONLY after processing ----
  deleteSMS(index);
}

String GetGPSLocation(){
  String message;
  if (gps.location.isValid()) {
    message = "Location:\n";
    message += "https://maps.google.com/?q=";
    message += String(gps.location.lat(), 6);
    message += ",";
    message += String(gps.location.lng(), 6);
  } else {
    message = "GPS not fixed. Try again outside.";
  }     
  return message;
}

void sendLocation(String number) {
  String message = GetGPSLocation();
  sendSMS(number, message);
}

void sendSMS(String number, String text) {
  logMessage("Sending sms :" + text + " to " + number);
  sim800.print("AT+CMGS=\"");
  sim800.print(number);
  sim800.println("\"");
  delay(500);
  sim800.print(text);
  delay(500);
  sim800.write(26); // CTRL+Z
  delay(3000);
}

void deleteSMS(int index) {
  sim800.print("AT+CMGD=");
  sim800.println(index);
  delay(500);
}

void makeCall(String number) {
  logMessage("📞 Calling...");
  sim800.print("ATD");
  sim800.print(number);
  sim800.println(";");   // semicolon is REQUIRED
  delay(20000);          // ring for 20 seconds
  sim800.println("ATH"); // hang up
}

void checkOfflineSMS() {
  logMessage("📩 Reading Offline SMS");
  sim800.println("AT+CMGL=\"REC UNREAD\"");

  unsigned long t = millis();
  String resp = "";

  while (millis() - t < 6000) {
    while (sim800.available()) {
      resp += char(sim800.read());
    }
  }

  if (resp.indexOf("+CMGL:") == -1) {
    logMessage("No offline SMS found");
    return;
  }

  int pos = 0;

  while (true) {
    int hdr = resp.indexOf("+CMGL:", pos);
    if (hdr == -1) break;

    int headerEnd = resp.indexOf("\n", hdr);
    if (headerEnd == -1) break;

    // -------- Extract index --------
    int comma = resp.indexOf(",", hdr);
    int index = resp.substring(hdr + 7, comma).toInt();

    // -------- Extract body --------
    int bodyStart = headerEnd + 1;
    int nextHdr = resp.indexOf("+CMGL:", bodyStart);
    int bodyEnd = (nextHdr == -1)
                    ? resp.indexOf("\nOK", bodyStart)
                    : nextHdr;

    String smsHeader = resp.substring(hdr, headerEnd);
    String body = resp.substring(bodyStart, bodyEnd);
    body.trim();
    body.toUpperCase();

    logMessage("📩 Index: " + String(index));
    logMessage("Body: " + body);

    // -------- Phone number --------
    ExtractPhoneNumber(resp);

    if (body.indexOf("LOC") != -1) {
      sendLocation(phoneNumber);
    }

    deleteSMS(index);
    delay(300);
    
    pos = bodyEnd;
  }
}

void ExtractPhoneNumber(String sms) {
  int pos = sms.indexOf("+CMGR:");
  if (pos == -1) {
    pos = sms.indexOf("+CMGL:");
    if (pos == -1) return;
  }

  int q1 = sms.indexOf('"', pos);
  if (q1 == -1) return;

  int q2 = sms.indexOf('"', q1 + 1);
  if (q2 == -1) return;

  int q3 = sms.indexOf('"', q2 + 1);
  if (q3 == -1) return;

  int q4 = sms.indexOf('"', q3 + 1);
  if (q4 == -1) return;

  phoneNumber = sms.substring(q3 + 1, q4);
  phoneNumber.trim();

  logMessage("📞 PhoneNumber: " + phoneNumber);
}

bool networkRegistered() {
  sim800.println("AT+CREG?");
  unsigned long t = millis();
  String resp = "";

  while (millis() - t < 1500) {
    while (sim800.available()) {
      resp += char(sim800.read());
    }
  }
  logMessage("CREG Resp:");
  logMessage(resp);

  // Look for ,1 or ,5
  if (resp.indexOf(",1") != -1 || resp.indexOf(",5") != -1) {
    logMessage("Network Registered");
    return true;
  }
  return false;
}
