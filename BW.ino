/*
DigitalReadSerial
Reads a digital input on pin 2, prints the result to the serial monitor

This example code is in the public domain.
*/

const uint8_t LEDPin = 13;

// Define on which pin the first detector sits:
const uint8_t bw1_trigger_pin = 3;
// define parameters for bw1
bool bw1_movement_detected = false;
bool bw1_presence_detected = false;
uint8_t bw1_pin_State = LOW;
uint8_t bw1_pin_State_last = LOW;
bool bw1_pin_State_change = false;
int bw1_count = 0;
int bw1_sensivity_presence = 10;
unsigned long bw1_pin_State_change_last_millis = 0;
unsigned long bw1_last_movement_millis = 0;

// variablen für "multitasking"
unsigned long this_loop_millis = 0;
unsigned long last_loop_millis = 0;

unsigned long bw_interval = 25;                                     // interval in millis for Sensor check
unsigned long last_bw_millis = 0;

unsigned long serial_interval = 250;                                // interval in millis for Serial Output
unsigned long last_serial_millis = 0;

// Parameter nötig
//      empfindlichkeit präsenz
//		empfindlichkeit bewegung
//		zeit/timeout um päsenz zu detektieren
//
//		bw1 lastpinstate
// bw muss die ersten 30-60 sec in ruhe gelassen bzw ignoriert werden zum initialisieren


// the setup routine runs once when you press reset:
void setup ( ) {
    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);

    pinMode (13, OUTPUT);

    // set the trigger pin as input:
    pinMode (bw1_trigger_pin, INPUT);
}

// the loop routine runs over and over again forever:
void loop ( ) {
    this_loop_millis = millis ( );
    

    // read the input pin depending on the interval:
    if (this_loop_millis - last_bw_millis >= bw_interval) {

        bw1_pin_State = digitalRead (bw1_trigger_pin);


        //has pin state changed?
        if (bw1_pin_State != bw1_pin_State_last) {
            bw1_pin_State_change = true;
            bw1_pin_State_change_last_millis = this_loop_millis;
        }
        else {
            bw1_pin_State_change = false; }

        // increment or decrement counter for better accuracy presence detection
        if (bw1_pin_State == HIGH){
            if (bw1_count <= 20000) {
                bw1_count++;
                if (bw1_pin_State == bw1_pin_State_last)bw1_count = bw1_count + 5;
            }
            bw1_last_movement_millis = this_loop_millis;
        }
        else{
            if (bw1_count >> 0){
                bw1_count--;
                if (bw1_pin_State == bw1_pin_State_last && bw1_count >> 20)bw1_count = bw1_count - 2;
            }
        }


        //set presence variabble if counter is high enough
        if (bw1_count >= bw1_sensivity_presence) { bw1_presence_detected = true; }
        else if (bw1_count <= 0){ bw1_presence_detected = false; }


        last_bw_millis = this_loop_millis;                              // bw check aktuelle Zeit abspeichern
        bw1_pin_State_last = bw1_pin_State;
    }


    // print out the state of the detector:
    if (this_loop_millis - last_serial_millis >= serial_interval) {
        Serial.println ();
        Serial.println (this_loop_millis);
        Serial.println ("------------------------------");
        
        Serial.print ("BW1 pin state            : ");
        Serial.println (bw1_pin_State);
        Serial.print ("BW1 counter              : ");
        Serial.println (bw1_count);
        Serial.print ("BW1 Presence              : ");
        Serial.println (bw1_presence_detected);
        Serial.print ("bw1 last movement millis  : ");
        Serial.println (bw1_last_movement_millis);
        
        Serial.print ("bw1_pin_State_change_last millis  : ");
        Serial.println (bw1_pin_State_change_last_millis);
        
        //led einschalten wenn presence detected
        if (bw1_presence_detected) { digitalWrite (LEDPin, HIGH); }
        else { digitalWrite (LEDPin, LOW); }


        last_serial_millis = this_loop_millis;   // sercomm aktuelle Zeit abspeichern
    }

    last_loop_millis = this_loop_millis;
    
}



