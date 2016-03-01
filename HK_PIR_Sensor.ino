/*
Movement/Presence Detector testcode

by Hassel v0.1
*/



/*  Todo
 Parameter nötig
      empfindlichkeit präsenz
        empfindlichkeit bewegung
        zeit/timeout um päsenz zu detektieren

        bw1 lastpinstate
      bw muss die ersten 30-60 sec in ruhe gelassen bzw ignoriert werden zum initialisieren
*/


const uint8_t LEDPin = 13;
const int LDR_Pin = A0;                                             //Where is the LDR connected

// define parameters for bw1
const uint8_t bw1_trigger_pin = 4;                                  // Define on which pin the first detector sits:
uint8_t bw1_pin_State = LOW;
uint8_t bw1_pin_State_last = LOW;
bool bw1_movement_detected = false;                                 // braucht n KO
bool bw1_presence_detected = false;                                 // braucht n KO
bool bw1_pin_State_change = false;
unsigned long bw1_pin_State_change_last_millis = 0;
unsigned long bw1_last_movement_millis = 0;
byte bw1_sensivity_presence = 10;                                    // braucht als parameter in knx
unsigned int bw1_count_max = 61000;                                          // braucht als parameter in knx
unsigned int bw1_count = 0;

int LDRReading = -1;

// variablen für "multitasking" bzw ablaufkontrolle
unsigned long this_loop_millis = 0;
unsigned long last_loop_millis = 0;
unsigned long bw_interval = 25;                                     // interval in millis for Sensor check
unsigned long last_bw_millis = 0;
unsigned long serial_interval = 100;                                // interval in millis for Serial Output
unsigned long last_serial_millis = 0;



void setup ( ) {
    
    Serial.begin(115200);                                           // initialize serial comm at 115200 bits per second:

    pinMode (LEDPin, OUTPUT);
    pinMode (bw1_trigger_pin, INPUT);                               // set the BW trigger pin as input:

    bw1_count_max = 61000; 
}

void loop ( ) {
    this_loop_millis = millis ( );
    

    // read the detector depending on the interval:
    if (this_loop_millis - last_bw_millis >= bw_interval) {

        bw1_pin_State = digitalRead (bw1_trigger_pin);


        //has pin state changed?
        if (bw1_pin_State != bw1_pin_State_last) {
            bw1_pin_State_change = true;
            bw1_pin_State_change_last_millis = this_loop_millis;
        }
        else {
            bw1_pin_State_change = false;
        }

        // increment or decrement counter for better accuracy presence detection
        if (bw1_pin_State == HIGH){


            if (bw1_count <= bw1_count_max) {
                bw1_count++;
                if (!bw1_pin_State_change){ bw1_count = bw1_count + 12; }
            }

            if (!bw1_pin_State_change){ bw1_movement_detected; }
            bw1_last_movement_millis = this_loop_millis;
        }
        else{
            if (bw1_count >> 0){
                bw1_count--;    
                if ((!bw1_pin_State_change) && (bw1_count >> 100)){
                    bw1_count = bw1_count - 3; }
            }
        }


        //set presence variabble if counter is high enough
        if (bw1_count >= bw1_sensivity_presence) { bw1_presence_detected = true; }
        else if (bw1_count <= 0){ bw1_presence_detected = false; }


        last_bw_millis = this_loop_millis;                              // bw check aktuelle Zeit abspeichern
        bw1_pin_State_last = bw1_pin_State;
    }


    // serial output state of the detector:
    if (this_loop_millis - last_serial_millis >= serial_interval) {

        
        LDRReading = analogRead (LDR_Pin);                          //read ldr value
        
        write_Serial_Debug ( );

        //enable led if presence detected
        if (bw1_presence_detected) { digitalWrite (LEDPin, HIGH); }
        else { digitalWrite (LEDPin, LOW); }


        last_serial_millis = this_loop_millis;   // sercomm aktuelle Zeit abspeichern
    }


    last_loop_millis = this_loop_millis;
}



void write_Serial_Debug ( )
{
    Serial.println ("--------------------------------------------------------");
    Serial.println (this_loop_millis);
    Serial.println ("------------------------------");
    Serial.print ("BW1 pin state             : ");
    Serial.println (bw1_pin_State);
    Serial.print ("BW1 counter               : ");
    Serial.println (bw1_count);
    Serial.print ("BW1 counter max           : ");
    Serial.println (bw1_count_max);
    Serial.print ("BW1 Movement              : ");
    Serial.println (bw1_movement_detected);
    Serial.print ("BW1 Presence              : ");
    Serial.println (bw1_presence_detected);
    Serial.print ("bw1 last movement millis  : ");
    Serial.println (bw1_last_movement_millis);
    Serial.print ("bw1_pin_State_change_last millis  : ");
    Serial.println (bw1_pin_State_change_last_millis);
    Serial.print ("LDR Value  : ");
    Serial.println (LDRReading);
}
