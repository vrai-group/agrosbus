// Licensed under the Apache License, Version 2.0  license. See LICENSE file in the project root for full license information.
// Luigi Di Marcantonio AGROSBUS Project UNIVPM

//Main library________________________
#include <CAN.h>
 //Sensors library_____________________
#include <DHT.h>                       // temperature and humidity sensor library

#include <DHT_U.h>                     // temperature and humidity sensor library

#include <HCSR04.h>                    //Ultrasonic distance sensor library
 //_____________________________________
#define DHTPIN A2 //temperature and humidity sensor PINs
#define DHTTYPE DHT11 //temperature and humidity sensor type
#define NODE_ID 0x12

DHT_Unified dht(DHTPIN, DHTTYPE); //Type and PIN 
UltraSonicDistanceSensor distanceSensor(A4, A3); //Ultrasonic distance sensor used PIN
void encodeUINT16(float value, int digits, unsigned short int * valueDST);

//union to manage data encoding

typedef union sensorS {
  unsigned char buf[2];
  unsigned short int value;
}sensorS;

sensorS Humidity;
sensorS Humidity_Ext;
sensorS Temperature;
sensorS Temperature_Ext;
sensorS Radar;

uint32_t delayMS;

//--SIMULATION TYPE--------------------------+
int simulation_type = 0; //                     |
//  0= continuous sending of packages        |
//  1= send packets if sensor values ​​change  |
//  2= random selected delay and sensor      |
//-------------------------------------------+

float temperature_value = 0;
float humidity_value = 0;
float radar_value = 0;

//function to encode data
void encodeUINT16(float value, int digits, unsigned short int * valueDST) {
  * valueDST = (unsigned short int)(value * pow(10, digits));
  return;
}

//setup function or arduino
void setup() {
  Serial.begin(9600);
  while (!Serial);
  //Sensors Setup
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor( & sensor);
  dht.humidity().getSensor( & sensor);
  delayMS = sensor.min_delay / 1000;
  //Simulation type--------------------
  Serial.println("CAN Sender");
  Serial.println("Selected simulation type: ");
  Serial.print(simulation_type);
  if (simulation_type == 0)
    Serial.println(" = Continuous sending of packages.");
  if (simulation_type == 1)
    Serial.println(" = Send packets if sensor values ​​change.");
  if (simulation_type == 2)
    Serial.println("= Radarandom selected delay and sensor.");
  //----------------------------------------
  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

//callback function to manage incoming CAN-bus frame
void receiverCallback(int packetSize) {
  Serial.print("\n\n\n");
  Serial.print("ID: ");
  Serial.print(CAN.packetId());
  Serial.print("  LENGHT: ");
  Serial.print(packetSize);
  Serial.print("  VALUES: ");
  if(CAN.packetId() != NODE_ID) //messege not of interest
    return; 
  //get raw bytes
  for (int i = 0; i < packetSize; i++) {
    int l = CAN.read();
    Serial.print(l, DEC);
    Serial.print(" | ");
    if (l == 0) 
      digitalWrite(LED_BUILTIN, LOW);
    else if (l == 1) 
      digitalWrite(LED_BUILTIN, HIGH);
  }
}

//function to send frame on CAN-bus
void sender(int id, unsigned char * data, unsigned short int len) {
  int i = 0;
  CAN.beginPacket(id);
  while (i < len) {
    CAN.write(data[i]);
    i++;
  }
  CAN.endPacket();
}

//function to send frame on CAN-bus with extended format
void extended_packet_sender(int id, unsigned char * data, unsigned short int len) {
  int i = 0;
  CAN.beginExtendedPacket(id);
  while (i < len) {
    CAN.write(data[i]);
    i++;
  }
  CAN.endPacket();
}

//function to manage the temperature sensor
void temperature_sensor() {
  sensors_event_t event;
  dht.temperature().getEvent( & event);
  Serial.print("Sending temperature data packet... ");
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    Serial.print(event.temperature); //TEST
    encodeUINT16(event.temperature, 1, & Temperature.value);
    Serial.println(" ValueDST: ");
    Serial.print(Temperature.value);
    sender(0x12, Temperature.buf, 2); //send packet with temperature data sensor
    Serial.println(" temp fatto");
  }
}

//function to manage the temperature sensor sending with extended format
void extended_temperature_sensor() {
  sensors_event_t event;
  dht.temperature().getEvent( & event);
  Serial.print("Sending temperature data packet... ");
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  } else {
    Serial.print(event.temperature); //TEST
    encodeUINT16(event.temperature, 1, & Temperature_Ext.value);
    Serial.println(" ValueDST extended packet temperature: ");
    Serial.print(Temperature.value);
    extended_packet_sender(0x12, Temperature.buf, 2); //send extended packet with temperature data sensor
    Serial.println(" Temperature data sent");
  }
}

//function to manage humidity sensor
void humidity_sensor() {
  sensors_event_t event;
  dht.humidity().getEvent( & event);
  Serial.print("Sending humidity data packet ");
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  } else {
    Serial.print(event.relative_humidity); //TEST
    encodeUINT16(event.relative_humidity, 1, & Humidity.value);
    Serial.println(" ValueDST Humidity: ");
    Serial.print(Humidity.value);
    sender(0x13, Humidity.buf, 2);
    Serial.println(" Humidity data packet sent ");
  }
}

//function to manage humidity sensor sending with extended format
void extended_humidity_sensor() {
  sensors_event_t event;
  dht.humidity().getEvent( & event);
  Serial.print("Sending humidity data packet ");
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  } else {
    Serial.print(event.relative_humidity); //TEST
    encodeUINT16(event.relative_humidity, 1, & Humidity_Ext.value);
    Serial.println(" ValueDST humidity: ");
    Serial.print(Temperature.value);
    extended_packet_sender(0x15, Humidity_Ext.buf, 2);
    Serial.println(" Humidity data packet sent");
  }
}

//function to manage distance sensor
void anti_collision_radar() {
  double distance = distanceSensor.measureDistanceCm();
  Serial.println(distance);
  encodeUINT16(distance, 2, & Radar.value);
  sender(0x14, Radar.buf, 2);
}

//function to send data only on changed values
void send_if_new_data_are_changed() {
  sensors_event_t event;
  dht.temperature().getEvent( & event);
  dht.humidity().getEvent( & event);

  if (temperature_value != event.temperature) {
    temperature_sensor();
    temperature_value = event.temperature;
  }

  if (humidity_value != event.relative_humidity) {
    humidity_sensor();
    humidity_value = event.relative_humidity;
  }
  if (radar_value != distanceSensor.measureDistanceCm()) {
    anti_collision_radar();
    radar_value = distanceSensor.measureDistanceCm();
  }
}

//funtion to manage random scheduling for data sending
int random_() {
  int v1 = rand() % 3;
  int v2 = rand() % 6;
  int delay_time[3] = {
    1000,
    500,
    20
  };
  int delay_ = delay_time[v1];
  switch (v2) {
	  case 1: {
		extended_temperature_sensor();
		delay(delay_);
	  }
	  case 2: {
		humidity_sensor();
		delay(delay_);
	  }
	  case 3: {
		anti_collision_radar();
		delay(delay_);
	  }
	  case 4: {
		extended_humidity_sensor();
		delay(delay_);
	  }
	  case 5: {
		delay(delay_);
	  }
	  case 6: {
		delay(delay_);
	  }
  }
}

//loop Arduino function
void loop() {
  if (simulation_type > 2) {
    Serial.println("Selected simulation type is wrong...");
    delay(10000);
  }
  if (simulation_type == 0) {
    temperature_sensor();
    delay(500);
    humidity_sensor();
    delay(500);
    extended_humidity_sensor();
    delay(500);
    anti_collision_radar();
    delay(500);
  }
  if (simulation_type == 1) {
    send_if_new_data_are_changed();
  }
  if (simulation_type == 2)
    random_();
  receiverCallback(int packetSize);
  Serial.println("All done!");
  delay(1000);
}