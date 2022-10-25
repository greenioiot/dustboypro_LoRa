/* Project : DustBoy via LoRaWAN
   Send  Temperature, Humidity, Wind Speed, Wind Direction, PM 2.5, PM10, Atmospheric pressure data from SEM1000 Sensor (Micro Weather Station) to LoRawan (RAK4270 module)
   Date : 22/10/2022
*/

#include <ArduinoJson.h>
#include <EEPROM.h>

#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "REG_CONFIG.h"
#include "BluetoothSerial.h"
#include <Arduino.h>
#include <TaskScheduler.h>
#include <ArduinoOTA.h>
//#include <WiFi.h>
#include <Wire.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

void t1CallgetMeter();
void t2CallsendViaLoRa();

//TASK
Task t1(80000, TASK_FOREVER, &t1CallgetMeter);
Task t2(90000, TASK_FOREVER, &t2CallsendViaLoRa);

#define battPIN  34
float Batt = 0.0;

#define trigWDTPin    32
#define ledHeartPIN   0

Scheduler runner;
StaticJsonDocument<400> doc;

#define rxPin                  14
#define txPin                  27
#define buadrate               115200
#define configParam            SERIAL_8N1

//==============Buffer====================
String data_input;
//==============Parameter=================

HardwareSerial modbus(1);
HardwareSerial lora(2);
BluetoothSerial SerialBT;
// instantiate ModbusMaster object
ModbusMaster node;

struct sem1000_7in1
{
  String windsp;
  String winddi;
  String humi;
  String temp;
  String PM2_5;
  String PM10;
  String press;
  String rssi;
  String battery;
};

sem1000_7in1 smartsensor ;

unsigned long previousMillis1, currentMillis1;
const unsigned long interval1 = 90000; // Interval Time

unsigned long ms;

long readModbus(char addr, uint16_t  REG)
{
  uint8_t j, result;
  uint16_t data;
  // communicate with Modbus slave ID 1 over Hardware Serial (port 1)
  node.begin(addr, modbus);
  result = node.readHoldingRegisters(REG, 1);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    data = node.getResponseBuffer(0);
    //Serial.println("Connec modbus Ok.");
    return data;
  } else
  {
    Serial.print("Connec modbus ID: ");
    Serial.print(addr);
    Serial.print(" Sensor fail. REG >>> ");
    Serial.println(REG); // Debug
    SerialBT.print("Connec modbus ID: ");
    SerialBT.print(addr);
    SerialBT.print(" Sensor fail. REG >>> ");
    SerialBT.println(REG); // Debug
    delay(100);
    return 0;
  }
}

void t1CallgetMeter() {     // Update read all data
  readSensor();
  delay(6000);
}

void t2CallsendViaLoRa() {     // Update read all data
  joinNetwork();
  delay(6000);
  sendviaLora();
  delay(6000);

}

void readSensor()
{
  modbus.begin(4800, SERIAL_8N1, 16, 17);
  delay(300);
  smartsensor.windsp = readModbus(ID_WEATHER, Address_7IN1[0]);
  smartsensor.winddi = readModbus(ID_WEATHER, Address_7IN1[1]);
  smartsensor.humi = readModbus(ID_WEATHER, Address_7IN1[2]);
  smartsensor.temp = readModbus(ID_WEATHER, Address_7IN1[3]);
  smartsensor.PM2_5 = readModbus(ID_WEATHER, Address_7IN1[4]);
  smartsensor.PM10 = readModbus(ID_WEATHER, Address_7IN1[5]);
  smartsensor.press = readModbus(ID_WEATHER, Address_7IN1[6]);

  Serial.println("---------------------------------------------------------");
  Serial.print("Temperature : ");  Serial.println((smartsensor.temp.toFloat()) / 10);
  Serial.print("Humidity : ");  Serial.println((smartsensor.humi.toFloat()) / 10);
  Serial.print("PM 2.5 : ");  Serial.println(smartsensor.PM2_5);
  Serial.print("PM 10 : ");  Serial.println(smartsensor.PM10);
  Serial.print("Wind Speed : ");  Serial.println(smartsensor.windsp);
  Serial.print("Wind Direction : ");  Serial.println(smartsensor.winddi);
  Serial.print("Atmospheric : ");  Serial.println(smartsensor.press);
  Serial.println("---------------------------------------------------------");

  SerialBT.print("Temperature : ");  SerialBT.println((smartsensor.temp.toFloat()) / 10);
  SerialBT.print("Humidity : ");  SerialBT.println((smartsensor.humi.toFloat()) / 10);
  SerialBT.print("PM 2.5 : ");  SerialBT.println(smartsensor.PM2_5);
  SerialBT.print("PM 10 : ");  SerialBT.println(smartsensor.PM10);
  SerialBT.print("Wind Speed : ");  SerialBT.println(smartsensor.windsp);
  SerialBT.print("Wind Direction : ");  SerialBT.println(smartsensor.winddi);
  SerialBT.print("Atmospheric : ");  SerialBT.println(smartsensor.press);
  SerialBT.println("---------------------------------------------------------");

  smartsensor.battery = Read_Batt() * 100.00;
  Serial.print("Battery : "); Serial.print(smartsensor.battery); Serial.println(" V");
  SerialBT.print("Battery : "); SerialBT.print(smartsensor.battery); SerialBT.println(" V");
  Serial.print("RSSI : "); Serial.println((smartsensor.rssi.toInt()) * (-1));
  SerialBT.print("RSSI : "); SerialBT.println((smartsensor.rssi.toInt()) * (-1));
  Serial.println("---------------------------------------------------------");
  SerialBT.println("---------------------------------------------------------");
}

void sendviaLora() {
  /* Decode
    case 0x0768:// Humidity (0-99)          2 Byte
    case 0x0267:// Temperature (0-80)       2 Byte
    case 0x0673:// PM2.5 (0-1000)           2 Byte
    case 0x0188:// PM10 (0-1000)            2 Byte
    case 0x0371:// Wind Speed (0-40)        2 Byte
    case 0x0402:// Wind Direction (0-359)   2 Byte
    case 0x0586:// Atmospheric (0-120)      2 Byte
    case 0x0802:// Battery                  2 Byte
    case 0x0803:// RSSI                     2 Byte
    example data
    sensorData = '0768' + sensor.moisture + '0267' + sensor.temperature + '0673' + sensor.EC + '0188' + sensor.PH + '0371' + sensor.Nit + '0402' + sensor.Pho + '0586' + sensor.Pot;
  */
  String sensorData;
  int randomport;
  randomport = random(1, 223);
  sensorData = "0768" + convert2Hex(smartsensor.humi) + "0267" + convert2Hex(smartsensor.temp) + "0673" + convert2Hex(smartsensor.PM2_5) + "0188" + convert2Hex(smartsensor.PM10) + "0371" + convert2Hex(smartsensor.windsp) + "0402" + convert2Hex(smartsensor.winddi) + "0586" + convert2Hex(smartsensor.press) + "0802" + convert2Hex(smartsensor.battery) + "0803" + convert2Hex(smartsensor.rssi);
  sendHexData(randomport, sensorData);
  delay(6000);
}

String convert2Hex(String tempdata) {
  unsigned long datatemp;
  int dataLength;
  String sensorData;
  datatemp = tempdata.toInt();
  sensorData = String(datatemp, HEX);
  dataLength = sensorData.length();
  if (dataLength == 3) {
    sensorData = "0" + sensorData;
  } else if (dataLength == 2) {
    sensorData = "00" + sensorData;
  } else if (dataLength == 1) {
    sensorData = "000" + sensorData;
  } else  if (sensorData == 0) {
    sensorData = "0000";
  }
  return sensorData;
}

float Read_Batt()
{
  unsigned int vRAW = 0;
  float Vout = 0.0;
  float Vin = 0.0;
  float R1 = 15000.0;
  float R2 = 3900.0;
  int bitRes = 4096;
  int16_t adc = 0;
  adc = analogRead(battPIN);
  //  for (int a = 0; a < 20; a++)
  //  {
  //    adc  += analogRead(battPIN);
  //    delay(1);
  //  }
  //  vRAW = adc / 20;
  vRAW = adc;
  //  Serial.println(vRAW);
  //  Vout = (vRAW * 3.3162) / bitRes;
  //  Serial.println(Vout);
  //  Vin = Vout / (R2 / (R1 + R2));
  Vin = adc * 0.004353019;
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }
  return Vin;
}

//***************************************************************************
void sendHexData(int type, String data) {
  String data_response = "";
  String rssi;
  String atComm = (String)
                  "at+send=lora:" + type + ":" + data + (String)
                  "\r\n";
  lora.flush();
  lora.print(atComm + "\r\n");
  Serial.print(atComm + "\r\n");
  delay(300);
  while (1) {
    if (lora.available()) {
      data_response = lora.readStringUntil('\n');
      if (data_response.indexOf(F("at+recv=")) != -1) {
        byte index = data_response.indexOf(F("="));
        if (data_response.substring(index + 4, 13) != 0) {
          smartsensor.rssi = data_response.substring(index + 4, 13);
          break;
        }
        else if (data_response.indexOf(F("OK")) != -1) {
          break;
        }
        Serial.println(data_response);
      }
    }
  }
}
String joinNetwork() {
  data_input = "";
  String atComm = "at+join";
  lora.flush();
  lora.print(atComm + "\r\n");
  Serial.print(atComm + "\r\n");
  delay(300);
  while (1) {
    if (lora.available()) {
      data_input = lora.readStringUntil('\n');
      break;
    }
  }
  data_input.replace(F("\""), "");
  data_input.trim();
  data_input.toUpperCase();
  blankChk(data_input);
  return data_input;
}

void ShowConfig() {
  String data_response = "";
  String dev_eui;
  String atComm = "at+get_config=lora:status";
  lora.flush();
  lora.print(atComm + "\r\n");
  Serial.print(atComm + "\r\n");
  delay(300);
  Serial.println(F("*********** LoRa Config ***********"));
  SerialBT.println(F("*********** LoRa Config ***********"));
  while (lora.available()) {
    data_response = lora.readString();
    Serial.println(data_response);
    SerialBT.println(data_response);
  }
  Serial.println(F("***********************************"));
  SerialBT.println(F("***********************************"));
}

void blankChk(String val) {
  if (val == "") {
    val = "N/A";
  }
}


/**********************************************  WIFI Client 注意编译时要设置此值 *********************************
   wifi client
*/
const char* ssid = "greenio"; //replace "xxxxxx" with your WIFI's ssid
const char* password = "green7650"; //replace "xxxxxx" with your WIFI's password

//WiFi&OTA 参数
String HOSTNAME = "Dustboy-";
#define PASSWORD "green7650" //the password for OTA upgrade, can set it in any char you want

void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);
  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());
  //No authentication by default
  ArduinoOTA.setPassword(password);
  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");
    SerialBT.println("Start Updating....");
    HeartBeat();
    SerialBT.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });
  ArduinoOTA.onEnd([]()
  {
    SerialBT.println("Update Complete!");
    Serial.println("Update Complete!");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    //    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);
    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
    ms = millis();
    if (ms % 10000 == 0)
    {
      HeartBeat();
    }
  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }
    Serial.println(info);
    ESP.restart();
  });

  ArduinoOTA.begin();
}

void setupWIFI()
{
  Serial.println("Connecting...");
  Serial.println(String(ssid));
  //连接WiFi，删除旧的配置，关闭WIFI，准备重新配置
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  //WiFi.onEvent(WiFiEvent);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);    //断开WiFi后自动重新连接,ESP32不可用
  WiFi.setHostname(HOSTNAME.c_str());
  WiFi.begin(ssid, password);
  //等待5000ms，如果没有连接上，就继续往下
  //不然基本功能不可用
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");
}

String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}

//********************************************************************//
//*********************** HeartBeat Function **************************//
//********************************************************************//
void HeartBeat() {
  //   Sink current to drain charge from watchdog circuit
  pinMode(trigWDTPin, OUTPUT);
  digitalWrite(trigWDTPin, LOW);
  // Led monitor for Heartbeat
  digitalWrite(ledHeartPIN, LOW);
  delay(300);
  digitalWrite(ledHeartPIN, HIGH);
  // Return to high-Z
  pinMode(trigWDTPin, INPUT);
  Serial.println("Heartbeat");
  SerialBT.println("Heartbeat");
}

void setup() {
  HeartBeat();
  Serial.begin(115200);
  SerialBT.begin("DustBoy Blutooht");
  lora.begin(buadrate, configParam, rxPin, txPin);
  Serial.println();
  Serial.println(F("Starting... Smart Weather Station"));
  Serial.println(F("***********************************"));
  SerialBT.println(F("Starting... Smart Weather Station"));
  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");
  HeartBeat();
  delay(2000);
  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");
  //  t3.enable();  Serial.println("Enabled t3");
  HeartBeat();
  HOSTNAME.concat(getMacAddress());
  SerialBT.begin(HOSTNAME); //Bluetooth
  SerialBT.begin(HOSTNAME); //Bluetooth device name
  SerialBT.println(HOSTNAME);

  Serial.println();
  Serial.println(F("***********************************"));
  Serial.println("Initialize...");
  HeartBeat();
  setupWIFI();
  HeartBeat();
  setupOTA();
  HeartBeat();

  Serial.println(joinNetwork());
  ShowConfig();
  delay(6000);
  readSensor();
  delay(6000);
  sendviaLora();
  delay(6000);
}

void loop() {
  runner.execute();
  ArduinoOTA.handle();
  ms = millis();
  if (ms % 600000 == 0)
  {
    Serial.println("Attach WiFi for，OTA "); Serial.println(WiFi.RSSI() );
    SerialBT.println("Attach WiFi for OTA"); SerialBT.println(WiFi.RSSI() );
    setupWIFI();
    HeartBeat();
    setupOTA();
  }
  if (ms % 60000 == 0)
  {
    Serial.println("Waiting for，OTA now"); Serial.println(WiFi.RSSI() );
    SerialBT.println("Waiting for, OTA now"); SerialBT.println(WiFi.RSSI() );
  }
  if (ms % 10000 == 0)
  {
    HeartBeat();
  }
}
