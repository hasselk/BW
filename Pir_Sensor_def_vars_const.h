
const uint8_t LEDPin = 13;
const int LDR1_Pin = A0;                                             //Where is the LDR connected

// Pir1 vars
const uint8_t pir1_trigger_pin = 4;                                  // pin connected to the first pir detector
uint8_t pir1_pin_State = LOW;
uint8_t pir1_pin_State_last = LOW;

bool bw1_movement_detected = false;                                 // braucht n KO
bool bw1_presence_detected = false;                                 // braucht n KO
bool pir1_pin_State_change = false;

byte pir1_sensivity_presence = 10;                                   // braucht als parameter in knx
unsigned int pir1_count_max = 22000;                                  // braucht als parameter in knx
unsigned int pir1_count ;

unsigned long pir1_pin_State_change_last_millis ;
unsigned long pir1_last_movement_millis ;

// LDR1 vars
int LDRReading1 = -1;
int LDRReading1_last = -1;
int LDRReading1_avg = -1;

// variablen für "multitasking" bzw ablaufkontrolle x*4byte
unsigned long this_loop_millis = 0;
unsigned long last_loop_millis = 0;
unsigned long bw_interval = 25;                                     // Sensor check interval in millis
unsigned long pir1_last_millis = 0;
unsigned long serial_interval = 100;                                // Serial Output interval in millis
unsigned long serial_last_millis = 0;

// 6+2 x4byte =32byte    3+2+1 x2byte =12byte     4+1 x1byte = 5byte  === 49byte