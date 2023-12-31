#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Arduino_JSON.h>
#include <WiFi.h>
#include <Wire.h>
#include <AWS_IOT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

AWS_IOT esp_sensor;

const char* ssid = ""; //wifi ssid here
const char* password = ""; //wifi password here
char HOST_ADDRESS[] = "**********-ats.iot.ap-northeast-2.amazonaws.com";
char CLIENT_ID[]= "esp_sensor";
char sTOPIC_NAME[]= "$aws/things/esp_sensor/shadow/update"; // subscribe topic name
char pTOPIC_NAME[]= "$aws/things/esp_sensor/shadow/update"; // publish topic name

int msgCount=0,msgReceived = 0;
char payload[512];
char rcvdPayload[512];

#define SEALEVELPRESSURE_HPA (1013.25)

WiFiClientSecure net = WiFiClientSecure();
Adafruit_BME280 bme;
HardwareSerial port(2);

unsigned long pre_time = 0;
unsigned long cur_time = 0;

const int duration = 10000;

const int trigPin = 18;
const int echoPin = 23;
const int ledPin = 16;

const int ledChannel = 0;
const int buzzerPin = 17;
const int resolution = 8;
const int duty = 128;
int sVal;
int nFrq[]={262, 294, 330, 349, 392, 440, 494, 523};

void mySubCallBackHandler (char *topicName, int payloadLen, char *payLoad)
{
  strncpy(rcvdPayload,payLoad,payloadLen);
  rcvdPayload[payloadLen] = 0;
  msgReceived = 1;
}

void connectAWS()
{

  if(esp_sensor.connect(HOST_ADDRESS,CLIENT_ID)== 0) {
    Serial.println("Connected to AWS");
    delay(1000);
    if(0==esp_sensor.subscribe(sTOPIC_NAME, mySubCallBackHandler)) {
      Serial.println("Subscribe Successfull");
    }
    else {
      Serial.println("Subscribe Failed, Check the Thing Name and Certificates");
      while(1);
    }
  }
  else {
    Serial.println("AWS connection failed, Check the HOST Address");
    while(1);
  }
}

void connectWifi()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi ");
  Serial.print(ssid);
  Serial.println("...");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("Connected to Wi-Fi!");
}

void connectBme()
{
  bool status = bme.begin(0x76);
  Serial.println("Connecting to BME280...");
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }
  Serial.println("Connected to BME280");
}

void publishJson(char *topic, bool feedEaten)
{
  StaticJsonDocument<200> doc;

  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");

  reported["temp"] = bme.readTemperature();
  reported["humid"] = bme.readHumidity();
  reported["press"] = bme.readPressure();
  reported["feed_eaten"] = feedEaten;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  while(esp_sensor.publish(pTOPIC_NAME, jsonBuffer) != 0){
    Serial.println("Publish failed");
    delay(10);
  }
  Serial.println("Publish Message");
}

void publishMeasurment()
{
  StaticJsonDocument<200> doc;

  JsonObject state = doc.createNestedObject("state");
  JsonObject reported = state.createNestedObject("reported");

  reported["temp"] = bme.readTemperature();
  reported["humid"] = bme.readHumidity();
  reported["press"] = bme.readPressure() / 100.0F;
  //reported["feed_eaten"] = false; //checkDistance();

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  esp_sensor.publish(pTOPIC_NAME, jsonBuffer);
}

bool checkDistance()
{
  long duration, distance;
  //Triggering by 10us pulse
  digitalWrite(trigPin, LOW); // trig low for 2us
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); // trig high for 10us
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //getting duration for echo pulse
  duration = pulseIn(echoPin, HIGH);
  //sound speed = 340 m/s = 34/1000 cm/us
  //distance = duration * 34/1000 * 1/2
  distance = duration * 17 / 1000;

  // Output the distance to the serial monitor
  Serial.print(distance);
  Serial.print("cm");
  Serial.println();
  if (distance <= 17) {return true;}
  if (distance > 17) {return false;}

  // Check if the distance exceeds 30 cm
  //return (distance < 200);
}

void playNote(int note){
  ledcSetup(ledChannel, nFrq[note], resolution);
  ledcWrite(ledChannel, duty);
  delay(250);
  ledcSetup(ledChannel, 0, resolution);
  ledcWrite(ledChannel, 0);
  delay(10);
}

void setup()
{
  Serial.begin(115200);
  pre_time = millis();
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  ledcAttachPin(buzzerPin, ledChannel);
  connectWifi();
  connectAWS();
  connectBme();
}

void loop()
{
  cur_time = millis();

  if( cur_time - pre_time > duration ) {
  publishMeasurment();
  pre_time = cur_time;
  }
  if(msgReceived == 1)
  {
    msgReceived = 0;
    bool feed_given;
    Serial.print("Received Message:");
    Serial.println(rcvdPayload);
    // Parse JSON
    JSONVar myObj = JSON.parse(rcvdPayload);
    feed_given = myObj["state"]["reported"]["feed_given"];
    Serial.print("FeedGiven is ");
    Serial.println(feed_given);
    if (feed_given == true)
    {
        digitalWrite(ledPin, HIGH);
        playNote(0);
        playNote(1);
        playNote(2);
        playNote(3);
        playNote(4);
        playNote(5);
        playNote(6);
        playNote(7);
        playNote(4);
        playNote(2);
        playNote(0);
        digitalWrite(ledPin, LOW);
        int f = 0;
      for (int i = 0; i < 6; i++)
      {
        digitalWrite(ledPin, HIGH);
        publishMeasurment();
        delay(5000);
        digitalWrite(ledPin, LOW);
        delay(5000);
        
        if (checkDistance() == true) {f = f + 1;}
      }
      if (f > 1) {
        publishJson(pTOPIC_NAME, true);
      }
      else{
        publishJson(pTOPIC_NAME, false);
      }
      pre_time = millis();
    }
    else if (feed_given == false) {
      digitalWrite(ledPin, LOW);
    }
  }

}