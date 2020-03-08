#define MY_RADIO_RF24
#define MY_DEBUG

// The temp and humidity sensor
#define DHT_PIN 4
#define DHT_TYPE DHT22
#define HALLSENSOR_PIN 3

// the two "messages" we send
#define CHILD_ID_HUM 10
#define CHILD_ID_TEMP 11
#define CHILD_ID_WATER 7

// the rf comms library
#include <SPI.h>
#include <MySensors.h>

// Adafruit unified sensor library
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

DHT_Unified dht(DHT_PIN, DHT_TYPE);


int seconds_between_messages = 30;
uint32_t delayMS = seconds_between_messages*1000;

volatile int NbTopsFan; //measuring the rising edges of the signal
int calculated_flowrate, prev_calculated_flowrate;           
                    
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgWater(CHILD_ID_WATER, V_FLOW);

//This is the function that the interupt calls 
void rpm () { 
  NbTopsFan++;  //This function measures the rising and falling edge of the hall effect sensors signal
} 


void presentation() {
  sendSketchInfo("waterflow", "1.0");
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_WATER, S_WATER);
}

void before() {
  dht.begin();
  Serial.println("DHTxx Unified Sensor Example");

  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");



  // Set delay between sensor readings based on sensor details.
  Serial.print  ("Init water flow sensor");

  // set up the water flow fall effect sensor
  pinMode(HALLSENSOR_PIN, INPUT); //initializes digital pin 2 as an input
  attachInterrupt(digitalPinToInterrupt(HALLSENSOR_PIN), rpm, RISING); //and the interrupt is attached
  
  Serial.println("------------------------------------");
}

int loop_count = 0;

void loop() {
  loop_count++;
  
  NbTopsFan = 0;  //Set NbTops to 0 ready for calculations
  sei();    //Enables interrupts
  delay (1000); //Wait 1 second
  cli();    //Disable interrupts
  
  //(Pulse frequency x 60) / 7.5Q, = flow rate in L/hour 
  calculated_flowrate = (NbTopsFan * 60 / 7.5); 
  
//  Serial.print (calculated_flowrate, DEC);  Serial.println (" l/hour");

  prev_calculated_flowrate = calculated_flowrate;

  if(prev_calculated_flowrate !=0 || prev_calculated_flowrate != prev_calculated_flowrate || loop_count > 30) {
    // Send water rates
    Serial.print("Flowrate: "); Serial.print(calculated_flowrate); Serial.println(" l/hr");
    send(msgWater.set(calculated_flowrate, 1));
    
  }
 
  if (loop_count > 30) {
    
    loop_count = 0;

    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println("Error reading temperature!");
    } else {
      Serial.print("Temperature: "); Serial.print(event.temperature); Serial.println(" *C");
      send(msgTemp.set(event.temperature, 1));
    }

    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println("Error reading humidity!");
    } else {
      Serial.print("Humidity: "); Serial.print(event.relative_humidity); Serial.println("%");
      send(msgHum.set(event.relative_humidity, 1));
    }
    
  }
  /**/
}
