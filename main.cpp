#include <Arduino.h>
#include <RTClib.h>
#include "LowPower.h"
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <SPI.h>
#include <EEPROM.h>
#define phSensorPin A0
#define turbidityPin A1
#define gsmSwitch 6
#define SensorSwitch 33
#define CLOCK_INTERRUPT_PIN 2
#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";
#define SerialAT Serial1
#define SerialMon Serial
/* mqtt connection credentials */
#define channelID 1622708
const char clientID[]="CicpCBIeECM8NBMwDDkcGTM";
const char mqttUserName[]="CicpCBIeECM8NBMwDDkcGTM";
const char mqttPass[]="iBCrWUtnKxtuIDx5RRDTW/Sn";
const char* server = "mqtt3.thingspeak.com";
#define mqttPort 1883
int connectionDelay = 1;


int wakeCount=0;
bool flag=1;
void startUpGsm();
float SamplePH(int samples); //function prototype to return an average of a defined number of samples taken from the ph sensor
float SampleTurbidity(int samples);// fuction prototype to  return average of turbidity sensor readings.
void wakeup(); //wake up function prototype
RTC_DS3231 rtc; // instance of rtc library
#include <TinyGsmClient.h>
#include <PubSubClient.h>
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient  mqttClient(client);
void mqttSubscriptionCallback( char* topic, byte* payload, unsigned int length ) {
  // Print the details of the message that was received to the serial monitor.
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
void mqttSubscribe( long subChannelID ){
  String myTopic = "channels/"+String( subChannelID )+"/subscribe";
  mqttClient.subscribe(myTopic.c_str());
}
//publish function to publish data to thingspeak channel
void mqttPublish(long puChannelID,String message)
{
  String topicString="channels/"+String(puChannelID)+"/publish";
  mqttClient.publish(topicString.c_str(),message.c_str());
}
void mqttConnect() {
  // Loop until connected.
  while ( !mqttClient.connected() )
  {
    // Connect to the MQTT broker.
    if ( mqttClient.connect( clientID, mqttUserName, mqttPass ) ) {
      Serial.print( "MQTT to " );
      Serial.print( server );
      Serial.print (" at port ");
      Serial.print( mqttPort );
      Serial.println( " successful." );
    } else {
      Serial.print( "MQTT connection failed, rc = " );
      // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
      Serial.print( mqttClient.state() );
      Serial.println( " Will try again in a few seconds" );
      delay( connectionDelay*1000 );
    }
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //SerialAT.begin(9600);
  pinMode(SensorSwitch,OUTPUT);
  pinMode(gsmSwitch,OUTPUT);
  pinMode(13,1);
  Serial.println(F("setting up"));
  //setup rtc 
  if(!rtc.begin())
  {
    Serial1.println(F("couldn't find RTC!"));
  }
  if(rtc.lostPower())
  {
    rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  }
  rtc.disable32K();
  pinMode(CLOCK_INTERRUPT_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN),wakeup,LOW);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.disableAlarm(2);
  if(!rtc.setAlarm1(
    rtc.now()+TimeSpan(10),DS3231_A1_Second
  )){
    Serial.println(F("error,alarm wasn't set!"));
  }
  else{
    Serial.println(F("sampling frequency set for once every minute"));
  }
  delay(600);
  //digitalWrite(gsmSwitch,HIGH);
}

void loop() {
  attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN),wakeup,LOW);
  Serial.println("going to sleep");delay(100);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
  detachInterrupt(digitalPinToInterrupt(2));
  if(rtc.alarmFired(1))
  {
    rtc.clearAlarm(1);
  } 
  digitalWrite(13,flag);
  Serial.print("*******************wake count:");Serial.print(wakeCount);Serial.println("***********************");
  Serial.println("turning on sensors to sample");
  SampleTurbidity(5);
  SamplePH(5);
  // routine to connect to a network and upload averages to thingspeak
  delay(1000);
  if(wakeCount%5==0)
  {
    startUpGsm();
  }


}
void wakeup()
{
  detachInterrupt(digitalPinToInterrupt(2));
  flag=!flag;
  wakeCount++;
  //Serial.println("woken here");
}
float SamplePH(int samples)
{
  digitalWrite(SensorSwitch,1);//turn sensor on
  float sampleArray[samples];
  float sampleSum=0.00;
  for(int i=0;i<samples;i++)
  {
    sampleArray[i]=analogRead(phSensorPin);
    delay(200); //delay 200ms to allow for stable ADC convertions 
    Serial.print(" ph sample:");Serial.print(i );Serial.print("________");Serial.println(sampleArray[i]);
    sampleSum+=sampleArray[i];//sum up all read samples
    sampleArray[i]=0.00;//clear out the sample array;
  }
  digitalWrite(SensorSwitch,0); //turn sensor off
  return (sampleSum/samples);//return average of samples taken
}
float SampleTurbidity(int samples)
{
  digitalWrite(SensorSwitch,1);// turn sensor on
  float sampleArray[samples];
  float sampleSum=0.00;
  for(int i=0;i<samples;i++)
  {
    sampleArray[i]=analogRead(turbidityPin);
    Serial.print("turbidity sample:");Serial.print(i );Serial.print("________");Serial.println(sampleArray[i]);
    delay(200); //delay 200ms to allow for stable ADC convertions 
    sampleSum+=sampleArray[i];//sum up all read samples
    sampleArray[i]=0.00;//clear out the sample array;
  }
  digitalWrite(SensorSwitch,0);// turn sensor off
  return (sampleSum/samples);//return average of samples taken 
}
void startUpGsm()
{
  digitalWrite(gsmSwitch,HIGH); //turn onthe gm power
  SerialAT.begin(9600);
  modem.init();
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  SerialMon.print("waiting for networkk");
  if(!modem.waitForNetwork())
  {
    SerialMon.println("failed to get network");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }
  #if TINY_GSM_USE_GPRS
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
  if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }
  #endif
  mqttClient.setServer( server, mqttPort ); 
  mqttClient.setCallback( mqttSubscriptionCallback );
  mqttClient.setBufferSize( 2048 );
  if (!mqttClient.connected()) {
     mqttConnect(); 
     mqttSubscribe( channelID );
  }
  mqttClient.loop();
  mqttPublish( channelID, (String("field1=")+String(SamplePH(5))+String("&field2=")+String(SampleTurbidity(5))) );
  delay(2000);
  digitalWrite(gsmSwitch,LOW);
}