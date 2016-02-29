/*
DigitalReadSerial
Reads a digital input on pin 2, prints the result to the serial monitor

This example code is in the public domain.
*/

// Define on which pin the first detector sits:
const uint8_t bw1_trigger_pin = 3;
bool bw1_movement_detected = false;
bool bw1_presence_detected = false;
uint8_t bw1_pin_State = LOW;
uint8_t bw1_pin_State_last = LOW;
bool bw1_pin_State_change = false;
int bw1_count = 0;

// variablen für "multitasking"
unsigned long bw1_last_movement_millis = 0;
unsigned long this_loop_millis = 0;
unsigned long last_loop_millis = 0;

unsigned long bw_interval = 50; // interval in millis
unsigned long last_bw_millis = 0;

unsigned long serial_interval = 200; // interval in millis
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

    // set the trigger pin as input:
    pinMode (bw1_trigger_pin, INPUT);
}

// the loop routine runs over and over again forever:
void loop ( ) {
    this_loop_millis = millis ( );
    

    // read the input pin depending on the interval:
    if (this_loop_millis - last_bw_millis >= bw_interval) {

        //Serial.println ("bwckeck");
        //Serial.println ("bwcheck This Loop Millis:  " + this_loop_millis);

        bw1_pin_State = digitalRead (bw1_trigger_pin);

        if (bw1_pin_State == HIGH){
            bw1_count++;
        }
        else{ if (bw1_count >> 0)bw1_count--; }

        if (bw1_pin_State != bw1_pin_State_last)                        //has pin state changed?
        {
            

            bw1_pin_State_change = true;
            //Serial.println ("pin geaendert");
        }
        else
        {
            bw1_pin_State_change = false;
            //Serial.println ("pin nicht geaendert");
        }

        if (bw1_count >= 5) { bw1_presence_detected = true; }
        else if (bw1_count == 0){ bw1_presence_detected = false; }

        last_bw_millis = this_loop_millis;                              // aktuelle Zeit abspeichern
        bw1_pin_State_last = bw1_pin_State;
    }


    // print out the state of the detector:
    if (this_loop_millis - last_serial_millis >= serial_interval) {
        Serial.println ();
        Serial.println ("------------------------------");
        //if (bw1_pin_State_change){
            Serial.print ("BW1 pin state  : ");
            Serial.println (bw1_pin_State);
            Serial.print ("BW1 counter    : ");
            Serial.println (bw1_count);

            Serial.print ("BW1 Presence   : ");
            Serial.println (bw1_presence_detected);
            
        //}
        last_serial_millis = this_loop_millis;   // aktuelle Zeit abspeichern
        
    }
    last_loop_millis = this_loop_millis;
    delay (50);
}



