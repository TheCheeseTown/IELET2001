
/****************************************
 * Legger til biblioteker
 ****************************************/
#include "UbidotsEsp32Mqtt.h"
#include "Arduino.h"
#include "Adafruit_BME280.h"

Adafruit_BME280 bme;

/****************************************
 * Definerer konstanter
 ****************************************/
const char *UBIDOTS_TOKEN = ""; 
const char *WIFI_SSID = "";   
const char *WIFI_PASS = "";    
const char *SUBSCRIBE_DEVICE_LABEL = "";   

const int PUBLISH_FREQUENCY = 5000; // Update rate in millisecondsx

unsigned long timer;
uint8_t led = 25;
int moisturePin = 32; 

Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
 * Definerer RTC konstanter som kan holde på lagret data under DeepSleep modus
 ****************************************/
RTC_DATA_ATTR double tmpList[3]; 
RTC_DATA_ATTR double moistureList[3];
RTC_DATA_ATTR double AirmoistureList[3];
RTC_DATA_ATTR byte count = 0;
RTC_DATA_ATTR byte minTemp = 0;
RTC_DATA_ATTR byte maxTemp = 0;
RTC_DATA_ATTR byte minHumidity = 0;
RTC_DATA_ATTR byte maxHumidity = 0;
RTC_DATA_ATTR int Deep_Sleep_Time = 1;

// Tar imot verdier fra Ubidots
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String TOPIC = topic;
  Serial.println(TOPIC);
  String message ;
  for (int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  if (TOPIC == "/v2.0/devices/esp32/deepsleeptimer/lv"){
    Deep_Sleep_Time = message.toInt();
  }
  if (TOPIC == "/v2.0/devices/esp32/minmoisture/lv") {
    minHumidity = message.toInt();
  }
  if (TOPIC == "/v2.0/devices/esp32/maxmoisture/lv") {
    maxHumidity = message.toInt();
  }
  if (TOPIC == "/v2.0/devices/esp32/maxtemp/lv") {
    minTemp = message.toInt();
  }
  if (TOPIC == "/v2.0/devices/esp32/mintemp/lv") {
    minTemp = message.toInt();
  }
}

// Sender esp32 inn i deep sleep
void goToDeepSleep(){
  Serial.println("time to sleep");
  esp_sleep_enable_timer_wakeup(Deep_Sleep_Time * 60 * 100000 ); // skal ganges med 1 million 
  esp_deep_sleep_start();
}

// sjekker om avleste verdier er innenfor gitte parametere
void MinMax(int MIN,int MAX, float value) {
  if ((MIN > value) or (value > MAX)) {
    digitalWrite(led,HIGH);
  }
}

// regner ut gjsn verdi av listene med sensordata
double ListToAverage(double array[],int lengde){
    float sum = 0;
    int i;
    for (i=0;i<lengde;i++){
        sum += array[i];
        Serial.println(array[i]);
    }
    //Serial.println(sum/lengde);
    return (sum/lengde);
}


// leser av sesnor flere ganger for å få en god avlesning
float readMoisture() {
  int moistureSum = 0; 
  for (int i=0; i<=2;i++){
    moistureSum += analogRead(moisturePin);
  }
  float moistureGjsn = moistureSum / 3;
  return moistureGjsn;
}


void setup()
{
  Serial.begin(115200);
  Serial.print("God Morgen");
  pinMode(led, OUTPUT);
  pinMode(moisturePin, INPUT);

  // sjekker om BME280 sensor er tilkoblet
  if (!bme.begin(0x76)) 
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  count++; // øker count med +1

  tmpList[count - 1] = bme.readTemperature(); // leser av temp fra BME280 og legger inn i liste
  moistureList[count - 1] = readMoisture(); // leser av verdie fra soilmoisture sensor
  AirmoistureList[count - 1] = bme.readHumidity();
  // for hver n målinge skal esp32 koble seg på wifi og sende/motta verdier
  if (count == 3){ 
    ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
    ubidots.setCallback(callback);
    ubidots.setup();
    ubidots.reconnect();
    // variablene vi ønsker å ta i mot fra Ubidots
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "deepsleeptimer"); 
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "minmoisture");
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "maxmoisture");
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "mintemp");
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "maxtemp");
    count = 0; // tilbakestiller count 
    timer = millis();
  }
  else {
    goToDeepSleep();
  }
}


void loop()
{ 
  // sjekker om vi fremdeles er tilkoblet wifi
  if (!ubidots.connected())
  {
    ubidots.reconnect();
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "deepsleeptimer"); 
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "minmoisture");
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "maxmoisture");
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "mintemp");
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, "maxtemp"); 
  }
  if (int(millis() - timer) > PUBLISH_FREQUENCY ) // begynner å sende verdier etter 5 sekund
  {
    double pubtemp = ListToAverage(tmpList, 3); // regner ut gjsn av temp målinger
    double pubMoisture = ListToAverage(moistureList,3); // regner ut gjsn av soilmoisture målinger
    double pubMoistureProsent = map(pubMoisture,0,4095,0,100); // mapper gjsn av soilmoisture til prosent
    float AIRHUMIDITY = ListToAverage(AirmoistureList,3); // leser av luftfuktighet

    // sjekker om veridene er innenfor de ønskede rammene
    MinMax(minTemp,maxTemp,pubtemp); 
    MinMax(minHumidity,maxHumidity,AIRHUMIDITY);

    // publiserer de ønskede verdiene til Ubidots
    ubidots.add("airmoisture",AIRHUMIDITY);
    ubidots.publish();
    ubidots.add("tmp", pubtemp); // Insert your variable Labels and the value to be sent
    ubidots.publish();
    ubidots.add("moisture",pubMoistureProsent);
    ubidots.publish();
    ubidots.add("airmoisture",AIRHUMIDITY);
    ubidots.publish();
    delay(500);
  }

  ubidots.loop();

  // går tilbake i deep sleep etter 8 sekund
   if (int(millis() - timer) > (2 * PUBLISH_FREQUENCY))
  {
    goToDeepSleep();
  }

}
