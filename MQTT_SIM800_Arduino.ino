#include <NMEAGPS.h>
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <stdint.h>
#include <avr/wdt.h>

#include <Streamers.h>


//uint8_t mcusr_mirror __attribute__((section(".noinit")));
//
//void get_mcusr(void)
//    __attribute__((naked))
//    __attribute__((section(".init3")));
//void get_mcusr(void)
//{
//  mcusr_mirror = MCUSR;
//  MCUSR = 0;
//  wdt_disable();
//}


#define FIRST_P_Pin 8        // на реле первого положения замка зажигания с 8-го пина ардуино
#define BAT_Pin A0           // на батарею, через делитель напряжения 39кОм / 11 кОм
#define RESET_Pin 5          // аппаратная перезагрузка модема

/********************************************************************/
// Data wire is plugged into pin 2 on the Arduino 
#define ONE_WIRE_BUS 2 
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
SoftwareSerial Serial1(7, 6); // для старых плат начиная с версии RX,TX

NMEAGPS gps;
gps_fix fix;

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "internet.mts.ru";
const char user[] = "";
const char pass[] = "";

// MQTT details
const char* broker = "95.79.40.140";
const char* mqttuser = "starex";
const char* mqttpass = "starex1133";
int mqttport = 1883;
  
const char trackingServer[] = "95.79.40.140";
const int trackingPort = 5000;
  
const char* refreshTopic = "refresh/all";
const char* relay1PushTopic= "push/relay1";
const char* timer1PushTopic= "push/timer1";
const char* relay1StateTopic= "state/relay1";
const char* ok = " ok";
const char* fail = " fail";
const char* connectto="Connect to ";

bool hasModem=true;
bool lowVoltage=false;
bool relay1 = false;
int timer1 = 10;
int defaultTimer1 = 10;
char temperature[6];
    
unsigned long time1=0, time2=0;

float Vbat, V_min; // переменная хранящая напряжение бортовой сети
float m = 57.701915071; 

String imei;
String simno;
String lat = "NA";
String lon = "NA";
char  uptime[6];
char vbat[6];
char timer1str[5];
int nacount=0;

TinyGsm modem(Serial1);
TinyGsmClient client(modem, 1);
TinyGsmClient client2(modem, 2);
PubSubClient mqtt(client);

long lastReconnectAttempt = 0;
int reconnectTry=0;

void (*resetFunc)(void) = 0; //declare reset function @ address 0

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  
  Serial.print("Msg ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.write(payload, len);
  Serial.println();
  String _topic(topic);
  if(_topic.equals(relay1PushTopic)){
    pushRelay1();
    }
  if(_topic.equals(timer1PushTopic)){
      char p[len+1];
      p[len]=0;
      strncpy(p, (char*)payload, len);
      String _payload(p); 
      int newtimer = _payload.toInt();
      if (newtimer > 30) newtimer = 30;
      if (relay1 == true)
      {
        timer1 = newtimer;
      }
      else
      {
        defaultTimer1 = newtimer;
        timer1 = newtimer;
      }
      
  }
  mqttPublishAll();
}

boolean mqttConnect() {

  mqtt.setServer(broker, mqttport);

  boolean status = mqtt.connect("STAREX", mqttuser, mqttpass);

  reconnectTry++;
  Serial.print("Connect mqtt");
  if (status == false) {
    Serial.println(fail);
    if(reconnectTry>2){
      resetFunc();
    }
    return false;
  }
  Serial.println(ok);
  mqtt.subscribe(refreshTopic);
  mqtt.subscribe(relay1PushTopic);
  mqtt.subscribe(timer1PushTopic);
  return mqtt.connected();

}


boolean trackingConnect(){

    Serial.print(connectto);
    Serial.print(trackingServer);
    Serial.print(":");
    Serial.print(trackingPort);
    if (!client2.connect(trackingServer, trackingPort)) {
      Serial.println(fail);
      return false;
    } else {
      Serial.println(ok);
      return true;
    }
}

int mincount = 6;

void detection(){
  if (relay1 == true && timer1 < 1) {
      pushRelay1(); // остановка прогрева если закончился отсчет таймера
    } 
    if(mincount<=0){
      mincount=6;
      if(hasModem){
        if (!mqtt.connected()) {
          Serial.println("NO MQTT");
          if (mqttConnect()) {
              mqttPublishAll();
          } else {
            resetFunc();
            }
        } else {
          mqttPublishAll();
        }
        
    } 
      if(relay1 == true) {
            timer1-=1;
      }
      if(!hasModem){
        resetFunc();
        }
    }

    if (fix.valid.date && fix.valid.altitude && fix.valid.location && fix.valid.speed) {
      float speed=fix.speed_kph();
      unsigned long m=millis();
      nacount=0;
      lat = lonlatddmmss(fix.latitudeDMS);
      lon = lonlatddmmss(fix.longitudeDMS);

      if(speed>60) {
        sendPosition();
        
        }
      else if(speed>=30 && m>=time2+30000){
        sendPosition();
        
        }
      else if(speed>=10 && m>=time2+60000){
        sendPosition();
        
        }
      else if(m>=time2+120000){
        sendPosition();        
        }
       } else {
        
        nacount++;
        if(nacount>30){
          lat="NA";
          lon="NA";  
          }      
        
       } 


  mincount--;
  }
  
void mqttPublishAll(){
    Vbat = voltRead(); // замеряем напряжение на батарее
    dtostrf(Vbat, 0, 1, vbat);
    dtostrf(timer1, 0, 0, timer1str);
    sensors.requestTemperatures(); // Send the command to get temperature readings 
    float t = sensors.getTempCByIndex(0);
    dtostrf(t, 0, 1, temperature);
    float utime = (float)millis()/3600000;
    dtostrf(utime, 0, 1, uptime);
    Serial.println(".");
    char latlonchar[32];
    mqtt.publish("state/vbat", vbat);
    mqtt.publish("state/uptime", uptime);
    mqtt.publish("state/relay1", relay1 ? "start" : "stop");
    mqtt.publish("state/timer1", timer1str);
    mqtt.publish("state/temperature", temperature);
//    https://yandex.ru/maps/?pt=30.335429,59.944869&z=18&l=map

    String(lat+String(",")+lon).toCharArray(latlonchar, 32);
    mqtt.publish("state/position", latlonchar);
    
          

  }


void sendPosition(){
//imei:353451044508750,001,0809231929,13554900601,F,055403.000,A,2233.1870,N,11354.3067,E,0.00,30.1,65.43,1,0,10.5%,0.0%,28;
//
//Content specification
//Content after the first comma is: keywords
//Content after the second comma is: year, month, day, hour, minute
//Content after the third comma is: the number of sim card in the tracker
//Content after the fourth comma is: whether the signal of GPS is ok. 
//
//F means gps data is valid, L means no gps signal. In the old version, it is 5 comma when L, the following field is null. It stand for LBS in the new version, LAC is instead of latitude, Cellid is instead of longitude.
//
//Content after the fifth comma is: hour, minute, second of zero time zone
//Content after the sixth comma is: Be corresponding to the content after the fourth comma, A=F，V=L；
//Content after the seventh comma is: Latitude
//Content after the eighth comma is: N is north latitude, S is south latitude
//Content after the ninth comma is: Longitude
//Content after the tenth comma is: E is east longitude, W is west longitude
//Content after the eleventh comma is: speed
//Content after the twelfth comma is: 1 stand for address request, non-1 stand for direction ( Direction will be sure with a decimal point )
//Content after the thirteenth comma is: altitude
//Content after the fourteenth comma is: The current state of the vehicle ACC; 1 is on, 0 is off
//Content after the fifteenth comma is: The current state of the vehicle door; 1 is open the door, 0 is close the door
//Content after the sixteenth comma is: the remaining Oil percentage in the fuel tank 1
//Content after the seventeenth comma is: the remaining oil percentage in the fuel tank 2
//Content after the eighteenth comma is: The current temperature sensor indicated
//End with a semicolon after the nineteenth comma.


// TRACCAR TEST
//imei:868683023212255,tracker,190205084503,,F,064459.000,A,4915.1221,N,01634.5655,E,3.91,83.95;



        char buffer[20];
        int32_t altitude_cm =fix.altitude_cm();
        uint16_t hdg = fix.heading_cd();
        float speed = fix.speed_kph();


        String data="imei:"+imei+",tracker,";
        // add date
        data+=fix.dateTime.year;
        if(fix.dateTime.month<=9) data+="0";
        data+=fix.dateTime.month;
        if(fix.dateTime.date<=9) data+="0";
        data+=fix.dateTime.date;
        if(fix.dateTime.hours<=9) data+="0";
        data+=fix.dateTime.hours;
        if(fix.dateTime.minutes<=9) data+="0";
        data+=fix.dateTime.minutes;
        if(fix.dateTime.seconds<=9) data+="0";
        data+=fix.dateTime.seconds;
        data+=",,F,";

        if(fix.dateTime.hours<=9) data+="0";
        data+=fix.dateTime.hours;
        if(fix.dateTime.minutes<=9) data+="0";
        data+=fix.dateTime.minutes;
        if(fix.dateTime.seconds<=9) data+="0";
        data+=fix.dateTime.seconds;

        data+=".000,A,";
        
        data+=lat+","+fix.latitudeDMS.NS()+","; 
        data+=lon+","+fix.longitudeDMS.EW()+",";
        
        data+=String(speed)+",";
        data+=String(hdg)+",";
        data+=String((float)altitude_cm/100)+",";
        data+="0,0,1%,1%,";
        data+=String(temperature);
        data+=";"; // ignition, door, fuel1, fuel2, temperature
        Serial.println(data);

        if(hasModem){
          if(trackingConnect()){
              Serial.print("Send...");
              client2.print(data);
              // Wait for data to arrive
              uint32_t start = millis();
              while (client2.connected() && !client2.available() &&
                     millis() - start < 10000L) {
                delay(100);
              };
          
              // Read data
              start = millis();
              
              while (client2.connected() && millis() - start < 5000L) {
                while (client2.available()) {
                  Serial.write(client2.read());
                  start = millis();
                }
              }
              Serial.println(ok);
            }
        }
          client2.stop();
          time2 = millis();

  }

String lonlatddmmss(DMS_t latitudeDMS){
    String outs="";
      outs+=latitudeDMS.degrees;
  
    if (latitudeDMS.minutes < 10)
      outs+='0';
    outs+=latitudeDMS.minutes;
    outs+='.';
  
    uint16_t mmmm = latitudeDMS.seconds_whole * 166;  // same as 10000/60, less .66666...
    mmmm += (latitudeDMS.seconds_whole * 2 + latitudeDMS.seconds_frac/2 ) / 3;  // ... plus the remaining .66666
  
    //  print leading zeroes, if necessary
    if (mmmm < 1000)
      outs+='0';
    if (mmmm <  100)
      outs+='0';
    if (mmmm <   10)
      outs+='0';
    outs+=mmmm;
    return outs;
    
}

void setup(){
    //Hardware serial initializacion
    Serial.begin(9600);
    sensors.begin(); 
    pinMode(RESET_Pin, OUTPUT);
    pinMode(FIRST_P_Pin, OUTPUT);
    pinMode(BAT_Pin, INPUT);
    SIM800_reset();
    initGPRSWithMqtt();
    time2=millis();
}

void GPRSLoop(){
  if (mqtt.connected()) {
      mqtt.loop();
  }
}

void GPSloop(){
  
  while (gps.available( Serial )) {
    fix = gps.read();
     
  }
}

void loop(){
  GPSloop();
  if(hasModem){
      GPRSLoop();
    }
  
  
  if (millis() > time1 + 10000)
    time1 = millis(), detection(); // выполняем функцию detection () каждые 10 сек
}


float voltRead() { // замеряем напряжение на батарее и переводим значения в вольты
  float ADCC = analogRead(BAT_Pin);
  float realadcc = ADCC;
  ADCC = ADCC / m;
  if (ADCC < V_min)
    V_min = ADCC;
  return (ADCC);
} // переводим попугаи в вольты


void SIM800_reset(){
  digitalWrite(RESET_Pin, LOW);
  delay(400);
  digitalWrite(RESET_Pin, HIGH); // перезагрузка модема
  delay(200);
}


void pushRelay1() {
  digitalWrite(FIRST_P_Pin, HIGH);
  delay(800);
  digitalWrite(FIRST_P_Pin, LOW);
  relay1=!relay1;
  timer1 = defaultTimer1;
}


void initGPRSWithMqtt(){
  Serial1.begin(9600);
  float volt = voltRead();
  if(volt<5){
      Serial.println("Low voltage.");
      lowVoltage=true;
      hasModem=false;
      digitalWrite(RESET_Pin, LOW);
    } else {
      lowVoltage=false;
      Serial.println("Init modem.");
      modem.restart();
      String modemInfo = modem.getModemInfo();
      if(modemInfo.length()>0){
          Serial.print("Modem: ");
          Serial.println(modemInfo);
          hasModem=true;
        } else {
          Serial.println("No Modem");      
          hasModem=false;
        }
      if(hasModem){
    
          imei = modem.getIMEI();
          
          
          Serial.print("Wait network");
          if (!modem.waitForNetwork()) {
            Serial.println(fail);
              resetFunc();
          }else{
            Serial.println(ok);
          }
          Serial.print(connectto);
          Serial.print(apn);
          if (!modem.gprsConnect(apn, user, pass)) {
            Serial.println(fail);
              resetFunc();
          }
          Serial.println(ok);
        
          // MQTT Broker setup
          mqtt.setServer(broker, 1883);
          mqtt.setCallback(mqttCallback);
          mqttConnect(); 
      }      
      }


    
}
