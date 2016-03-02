/*
  Movement/Presence Detector testcode

  by Hassel
  v0.2
         added ldr
  v0.3
        added dht

*/


/*  Todo
  Parameter nötig
  empfindlichkeit präsenz
  empfindlichkeit bewegung
  zeit/timeout um präsenz zu detektieren

  bw1 lastpinstate
  bw muss die ersten 30-60 sec in ruhe gelassen bzw ignoriert werden zum initialisieren

  maybe optimize memory usage by stuffing together some of the bool and Pinstate Vars

*/

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

    const uint8_t LED1_presence_Pin = 5;
    const uint8_t LED2_movement_Pin = 4;
    
// Pir1 vars
    const uint8_t pir1_trigger_pin = 7;                                  // pin connected to the first pir detector
    uint8_t pir1_pin_State = LOW;
    uint8_t pir1_pin_State_last = LOW;
    bool pir1_pin_State_change = false;
    bool pir1_detected_movement = false;                                 // braucht n KO in knx
    bool pir1_detected_presence = false;                                 // braucht n KO in knx

    unsigned int pir1_sensitivity_movement = 20;                         // braucht als parameter in knx
    unsigned int pir1_sensivity_presence = 100;                          // braucht als parameter in knx
    unsigned int pir1_count_max = 22000;                                 // braucht als parameter in knx
    const unsigned int pir1_count_slowdown = 1000;                       // braucht eventuel als parameter in knx
    unsigned int pir1_count;
    
    unsigned long pir1_pin_State_change_last_millis;
    unsigned long pir1_last_movement_millis;

// LDR1 vars
    const int LDR1_Pin = A0;                                             //Where is the LDR connected
    int LDR1_Reading = -1;
    int LDR1_Reading_last = -1;
    int LDR1_Reading_avg = -1;                                           // braucht n KO in knx

// variablen für "multitasking" bzw ablaufkontrolle x*4byte
    unsigned long this_loop_millis = 0;
    unsigned long last_loop_millis = 0;
    unsigned long pir_interval = 25;                                     // PIR check interval in millis
    unsigned long pir1_last_millis = 0;
    unsigned long serial_interval = 100;                                 // Serial Output interval in millis
    unsigned long serial_last_millis = 0;
    unsigned long dht_interval = 1000;                                   // DHT Read interval in millis
    unsigned long dht_last_millis = 0;

//DHT Sensor
    #define DHTPIN            6                                          // Pin which is connected to the DHT sensor.

    // Uncomment the type of sensor in use:
    #define DHTTYPE           DHT11       // DHT 11 
    //#define DHTTYPE           DHT22     // DHT 22 (AM2302)
    //#define DHTTYPE           DHT21     // DHT 21 (AM2301)

    DHT_Unified dht1 (DHTPIN, DHTTYPE);
    
    float currentTemp = 0;
    float currentHumd = 0;
    float previousTemp = 0;
    float previousHumd = 0;

    //uint32_t delayMS;

// ------------------------------------------------------------------------------------------------------------------------------



void setup ( ) {

    Serial.begin (500000);                                                  // initialize serial comm     921600  500000  115200

    pinMode (LED1_presence_Pin, OUTPUT);
    pinMode (LED2_movement_Pin, OUTPUT);

    pinMode (pir1_trigger_pin, INPUT);                                      // set the BW trigger pin as input:

    dht1.begin ( );
    sensor_t sensor;
    dht1.temperature ( ).getSensor (&sensor);
    dht1.humidity ( ).getSensor (&sensor);
    //delayMS = sensor.min_delay / 1000;

}

void loop ( ) {
    this_loop_millis = millis ( );

    // read pir status depending on the interval:
    if (this_loop_millis - pir1_last_millis >= pir_interval) {
        get_state_pir1 ( );
        pir1_last_millis = this_loop_millis;
    }

    // dht abfragen
    if (this_loop_millis - dht_last_millis >= dht_interval) {
        get_temp_humid ( );
        dht_last_millis = this_loop_millis;                              
    }
    
    // serial output :
    if (this_loop_millis - serial_last_millis >= serial_interval) {
        read_LDR_1 ( );   
        toggle_pir_status_led ( );
        write_Serial_Debug ( );
    }

    last_loop_millis = this_loop_millis;
}


// ------------------------------------------------------------------------------------------------------------------------------


//enable led if presence detected
void toggle_pir_status_led ( ){

    if (pir1_detected_presence) { digitalWrite (LED1_presence_Pin, HIGH); }
    else { digitalWrite (LED1_presence_Pin, LOW); }

    if (pir1_detected_movement) { digitalWrite (LED2_movement_Pin, HIGH); }
        else { digitalWrite (LED2_movement_Pin, LOW); }
}

// Read LDR Value, and do some math
void read_LDR_1 ( ){
    /*
    Todo
    use array
    aplly smoothing
    */

    LDR1_Reading = analogRead (LDR1_Pin);                          //read ldr value
    LDR1_Reading_avg = (LDR1_Reading + LDR1_Reading_last) / 2;
    LDR1_Reading_last = LDR1_Reading;
}

//Read PIR state und do some math
void get_state_pir1 ( ){
    pir1_pin_State = digitalRead (pir1_trigger_pin);

    //has pin state changed?
    if (pir1_pin_State != pir1_pin_State_last) {
        pir1_pin_State_change = true;
        pir1_pin_State_change_last_millis = this_loop_millis;
    }
    else {
        pir1_pin_State_change = false;
    }

    // increment or decrement counter for presence detection
    if (pir1_pin_State == HIGH){

        //movement detection check
        if (!pir1_pin_State_change && !pir1_detected_movement && (pir1_count >= pir1_sensitivity_movement)){
            pir1_detected_movement = true;
        }

        // counter hochzählen wenn bewegung konstant schneller
        if (pir1_count <= pir1_count_max) {
            pir1_count++;
            if ((!pir1_pin_State_change) && (pir1_count > 30)){
                pir1_count = pir1_count + 10;
            }
        }
        pir1_last_movement_millis = this_loop_millis;
    }
    else{   //Pinstate == LOW

        //movement detection check                              && (pir1_count < pir1_sensitivity_movement)
        if (!pir1_pin_State_change && pir1_detected_movement ){
            pir1_detected_movement = false;
        }

        if (pir1_count > 0){
            pir1_count--;
            if ((!pir1_pin_State_change) && (pir1_count > pir1_count_slowdown)){
                pir1_count--;                       // = pir1_count - 1; }
            }
        }
    }

    //set presence variabble if counter is high enough
    if (pir1_count >= pir1_sensivity_presence) { pir1_detected_presence = true; }
    else if (pir1_count < 1){ pir1_detected_presence = false; }

    pir1_pin_State_last = pir1_pin_State;
}

// Jsat plain simple debug stuff
void write_Serial_Debug ( ){
    Serial.println ();
    Serial.println (F("---------------------------------------------"));
    Serial.print (F ("Time since start    millis   : "));
    Serial.println (this_loop_millis);
    Serial.print (F ("PIR1 last movement   millis  : "));
    Serial.println (pir1_last_movement_millis);
    Serial.print (F ("PIR1_pin_change_last millis  : "));
    Serial.println (pir1_pin_State_change_last_millis);
    Serial.println (F ("----------------------------------------"));
    Serial.print (F ("PIR1 pin state               : "));
    Serial.println (pir1_pin_State);
    Serial.print (F ("PIR1 counter                 : "));
    Serial.println (pir1_count);
    Serial.print (F ("PIR1 counter max             : "));
    Serial.println (pir1_count_max);
    Serial.print (F ("PIR1 Movement                : "));
    Serial.println (pir1_detected_movement);
    Serial.print (F ("PIR1 Presence                : "));
    Serial.println (pir1_detected_presence);
    Serial.print (F ("LDR Avg. Value               : "));
    Serial.println (LDR1_Reading_avg);

    //if (currentTemp != 255 || currentHumd != 255)
    //{
        Serial.println (F ("----------------------------------------"));
        Serial.print (F("Temperature                  : "));
        Serial.print (currentTemp);
        Serial.println (" *C");
        Serial.print (F("Humidity                     : "));
        Serial.print (currentHumd);
        Serial.println (" %");
    //}

    serial_last_millis = this_loop_millis;                      // sercomm aktuelle Zeit abspeichern
}


void get_temp_humid ( ){

    sensors_event_t event;
    dht1.temperature ( ).getEvent (&event);

    // Get temperature event
    if (isnan (event.temperature)) {
        //Serial.println ("Error reading temperature!");
        currentTemp = 255;
    }
    else {
        currentTemp = event.temperature;
    }

    // Get humidity event
    dht1.humidity ( ).getEvent (&event);
    if (isnan (event.relative_humidity)) {
        //Serial.println ("Error reading humidity!");
        currentHumd = 255;
    }
    else {
        currentHumd = event.relative_humidity;
    }
}