/*
  Movement/Presence Detector testcode

  by Hassel v0.2

  added ldr
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

//#include "HK_PIR_Sensor.h"

    const uint8_t LEDPin = LED_BUILTIN;
    const int LDR1_Pin = A0;                                             //Where is the LDR connected

// Pir1 vars
    const uint8_t pir1_trigger_pin = 4;                                  // pin connected to the first pir detector
    bool pir1_pin_State = LOW;
    bool pir1_pin_State_last = LOW;
    bool pir1_pin_State_change = false;
    bool pir1_detected_movement = false;                                 // braucht n KO
    bool pir1_detected_presence = false;                                 // braucht n KO


    byte pir1_sensivity_presence = 10;                                   // braucht als parameter in knx
    unsigned int pir1_count_max = 22000;                                 // braucht als parameter in knx
    unsigned int pir1_count;
    const unsigned int pir1_count_slowdown = 1000;

    unsigned long pir1_pin_State_change_last_millis;
    unsigned long pir1_last_movement_millis;

// LDR1 vars
    int LDR1_Reading = -1;
    int LDR1_Reading_last = -1;
    int LDR1_Reading_avg = -1;

// variablen für "multitasking" bzw ablaufkontrolle x*4byte
    unsigned long this_loop_millis = 0;
    unsigned long last_loop_millis = 0;
    unsigned long pir_interval = 25;                                     // Sensor check interval in millis
    unsigned long pir1_last_millis = 0;
    unsigned long serial_interval = 100;                                 // Serial Output interval in millis
    unsigned long serial_last_millis = 0;

// 6+2 x4byte =32byte    3+2+1 x2byte =12byte     4+1 x1byte = 5byte  === 49byte



void setup ( ) {

  Serial.begin (115200);                                                  // initialize serial comm at 115200 bits per second:

  pinMode (LEDPin, OUTPUT);
  pinMode (pir1_trigger_pin, INPUT);                                      // set the BW trigger pin as input:
}

void loop ( ) {
  this_loop_millis = millis ( );

  // read pir status depending on the interval:
  if (this_loop_millis - pir1_last_millis >= pir_interval) {
    get_state_pir1 ( );
    pir1_last_millis = this_loop_millis;                              // bw check aktuelle Zeit abspeichern
  }

  // serial output :
  if (this_loop_millis - serial_last_millis >= serial_interval) {
    read_LDR_1 ( );
    
    write_Serial_Debug ( );
    toggle_pir_status_led ( );
  }

  last_loop_millis = this_loop_millis;
}

//-------------------------------------------------------------------------------------------------------------

//enable led if presence detected
void toggle_pir_status_led ( ){

    if (pir1_detected_presence) { digitalWrite (LEDPin, HIGH); }
    else { digitalWrite (LEDPin, LOW); }
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

//Read PIR state und do all math
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

    // increment or decrement counter for better accuracy presence detection
    if (pir1_pin_State == HIGH){

        if (!pir1_pin_State_change){ pir1_detected_movement = true; }

        if (pir1_count <= pir1_count_max) {
            pir1_count++;
            if (!pir1_pin_State_change){ pir1_count = pir1_count + 15; }
        }
        pir1_last_movement_millis = this_loop_millis;
    }
    else{   //Pinstate == LOW
        if (!pir1_pin_State_change){ pir1_detected_movement = false; }

        if (pir1_count >> 0){
            pir1_count--;
            if ((!pir1_pin_State_change) && (pir1_count >> pir1_count_slowdown)){
                pir1_count++;// = pir1_count - 1; }
            }
        }

        //set presence variabble if counter is high enough
        if (pir1_count >= pir1_sensivity_presence) { pir1_detected_presence = true; }
        else if (pir1_count <= 0){ pir1_detected_presence = false; }
    }

    pir1_pin_State_last = pir1_pin_State;
}

// Jusat plain simple debug stuff
void write_Serial_Debug ( ){
    Serial.println (F("-------------------------------------------------"));
    Serial.println (this_loop_millis);
    Serial.println (F("-------------------------------------------------"));
    Serial.print (F ("BW1 pin state               : "));
    Serial.println (pir1_pin_State);
    Serial.print (F ("BW1 counter                 : "));
    Serial.println (pir1_count);
    Serial.print (F ("BW1 counter max             : "));
    Serial.println (pir1_count_max);
    Serial.print (F ("BW1 Movement                : "));
    Serial.println (pir1_detected_movement);
    Serial.print (F ("BW1 Presence                : "));
    Serial.println (pir1_detected_presence);
    Serial.print (F ("bw1 last movement millis    : "));
    Serial.println (pir1_last_movement_millis);
    Serial.print (F ("bw1_pin_change_last millis  : "));
    Serial.println (pir1_pin_State_change_last_millis);
    Serial.print (F ("LDR Avg. Value  : "));
    Serial.println (LDR1_Reading_avg);

    serial_last_millis = this_loop_millis;                      // sercomm aktuelle Zeit abspeichern
}
