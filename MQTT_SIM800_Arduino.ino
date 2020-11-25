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

const char* refreshTopic = "refresh/all";
const char* relay1PushTopic= "push/relay1";
const char* timer1PushTopic= "push/timer1";
const char* relay1StateTopic= "state/relay1";

bool relay1 = false;
int timer1 = 10;
int defaultTimer1 = 10;
char temperature[6];
    
unsigned long time1=0, time2=0;

float Vbat, V_min; // переменная хранящая напряжение бортовой сети
float m = 57.701915071; //58.3402489626556;                            // делитель для перевода АЦП в вольты для резистров 39/10kOm

String imei;
String simno;

char  uptime[6];
char vbat[6];
char timer1str[5];

TinyGsm modem(Serial1);
TinyGsmClient client(modem, 1);
TinyGsmClient client2(modem, 2);
PubSubClient mqtt(client);

long lastReconnectAttempt = 0;
int reconnectTry=0;

void (*resetFunc)(void) = 0; //declare reset function @ address 0

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.write(payload, len);
  Serial.println();
  String _topic(topic);
  if(_topic.equals("refresh/all")){
      mqttPublishAll();
    }
  if(_topic.equals(relay1PushTopic)){
    pushRelay1();
    mqttPublishAll();
    }
  if(_topic.equals(timer1PushTopic)){
      char p[len];
      strncpy(p, (char*)payload, len);
      String _payload(p); 
      Serial.print("_payload=");
      Serial.println(_payload);
      if (relay1 == true)
      {
        timer1 = _payload.toInt();
        if (timer1 > 30)
          timer1 = 30;
      }
      else
      {
        defaultTimer1 = _payload.toInt();
        if (defaultTimer1 > 30)
          defaultTimer1 = 30;
        timer1 = defaultTimer1;
      }
      
      mqttPublishAll();
  }
}

boolean mqttConnect() {

  mqtt.setServer(broker, mqttport);

  boolean status = mqtt.connect("STAREX", mqttuser, mqttpass);

  reconnectTry++;
  Serial.print("Conn mqtt");
  if (status == false) {
    Serial.println(" fail");
    if(reconnectTry>2){
      resetFunc();
    }
    return false;
  }
  Serial.println(" ok");
  mqtt.subscribe(refreshTopic);
  mqtt.subscribe(relay1PushTopic);
  mqtt.subscribe(timer1PushTopic);
  return mqtt.connected();

}
void detection(){
  if (relay1 == true && timer1 < 1) {
      pushRelay1(); // остановка прогрева если закончился отсчет таймера
    } else if(relay1 == true) {
      timer1-=1;
    }
  if (!mqtt.connected()) {
    Serial.println("MQTT NOT CONN");
    if (mqttConnect()) {
        mqttPublishAll();
    } else {
      Serial.println("reset");
      resetFunc();
      }
  } else {
    mqttPublishAll();
    }
  

  if (fix.valid.date && fix.valid.altitude && fix.valid.location && fix.valid.speed) {
    float speed=fix.speed_kph();
    unsigned long m=millis();
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
     }
  
  }
  
void mqttPublishAll(){
    float _Vbat = VoltRead(); // замеряем напряжение на батарее
    dtostrf(_Vbat, 0, 1, vbat);
    dtostrf(timer1, 0, 0, timer1str);
    sensors.requestTemperatures(); // Send the command to get temperature readings 
    Serial.print("T: "); 
    float t = sensors.getTempCByIndex(0);
    dtostrf(t, 0, 1, temperature);
    Serial.println(temperature );
    String(millis()/3600000,0).toCharArray(uptime,6);
    

    mqtt.publish("state/vbat", vbat);
    mqtt.publish("state/uptime", uptime);
    mqtt.publish("state/relay1", relay1 ? "start" : "stop");
    mqtt.publish("state/timer1", timer1str);
    mqtt.publish("state/temperature", temperature);
    
          

  }


void sendPosition(){
// imei:353451044508750,001,0809231929,13554900601,F,055403.000,A,2233.1870,N,11354.3067,E,0.00,30.1,65.43,1,0,10.5%,0.0%,28;
// imei:359587010124900,tracker,0809231929,13554900601,F,112909.397,A,2234.4669,N,11354.3287,E,0.11,;  
//For example:
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

    //      imei,command,time,Cell phone number,latitude,S/N,Longitude,E/W,Speed,direction,/request address


//This is likely TK103 and TK110 devices. But it's not for sure.
//
//The other xexun devices that send messages in one of the following formats are identified by IMEI as usual:
//imei:864895030551111,tracker,191003074503,,F,044503.00,A,4700.08753,N,02853.79387,E,4.985,327.55;
//imei:865472036980071,tracker,201125105210,,F,017390.00,A,5627.24410,N,04397.86620,E,0.09,0;
//imei:866771024070798,tracker,1312170400,,F,230030.000,A,2455.3288,N,06705.8537,E,0.00,0,,0,0,0.00%,,;

//imei:865472036980071,tracker,201125104133,,F,018850.00,A,56272.49,N,43978.59,E,0.02,0;
//190926151811,+31711117111,GPRMC,151811.775,A,5215.5816,N,00508.7693,E,28.31,83.20,260919,,,A*6D,F,SHAKE, imei:354778030121111,03,1.3,F:4.14V,1,143,5451,204,08,0D2A,B6E5\n\r
//
//This is likely TK102, TK102-2 and TK103-2, but it's not for sure either.


// TRACCAR TEST
//imei:868683023212255,tracker,190205084503,,F,064459.000,A,4915.1221,N,01634.5655,E,3.91,83.95;
//imei:865472036980071,tracker,201125122553,,F,122553.000,A,5627.2461,N,4397.8662,E,0.27,0,17370.00,0,0,1%,1%,1,10;


//if (fix.valid.date && fix.valid.altitude && fix.valid.location && fix.valid.speed) {
        char buffer[20];
        int32_t altitude_cm =fix.altitude_cm();
        uint16_t hdg = fix.heading_cd();
        float speed = fix.speed_kph();

        String lat = lonlatddmmss(fix.latitudeDMS);
        String lon = lonlatddmmss(fix.longitudeDMS);

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

          
          const char server[] = "128.199.174.173";
          const int port = 5001;
          Serial.print("Conn ");
          Serial.print(server);
          if (!client2.connect(server, port)) {

            Serial.println("fail");
          } else {
            client2.print(data);
            // Wait for data to arrive
            uint32_t start = millis();
            while (client2.connected() && !client2.available() &&
                   millis() - start < 30000L) {
              delay(100);
            };
        
            // Read data
            start = millis();
            Serial.print(". ret=");
            while (client2.connected() && millis() - start < 5000L) {
              while (client2.available()) {
                Serial.write(client2.read());
                start = millis();
              }
            }
            Serial.println("");
            client2.stop();
            Serial.println("ok");
          }
          time2 = millis();

//      }
        
  
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
  SIM800_reset();
  initGPRSWithMqtt();
  time2=millis();
}

void GPRSLoop(){
  if (mqtt.connected()) {
      mqtt.loop();
  }
}

void GPSloop()
{
  
  while (gps.available( Serial )) {
    fix = gps.read();
     
  }
}

void loop(){
  GPSloop();
  GPRSLoop();
  
  if (millis() > time1 + 10000)
    time1 = millis(), detection(); // выполняем функцию detection () каждые 60 сек
}


float VoltRead() { // замеряем напряжение на батарее и переводим значения в вольты
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
  Serial.println("Init modem...");
  modem.restart();
  String modemInfo = modem.getModemInfo();
  Serial.print("Modem: ");
  Serial.println(modemInfo);
  if (!modem.hasSSL()) {
    Serial.println(F("No SSL"));
    Serial.println("reset");
    resetFunc();
  }else{
    Serial.println("Have SSL");    
  }
  imei = modem.getIMEI();
  
  
  Serial.print("Wait network");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
      resetFunc();
  }else{
    Serial.println(" OK");
  }
  Serial.print("Conn to ");
  Serial.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(" fail");
      resetFunc();
  }
  Serial.println(" OK");

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  mqttConnect(); 
  
}
