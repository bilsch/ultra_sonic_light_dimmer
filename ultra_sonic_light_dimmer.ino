
/*
 * This project combines a shift register controlling a bunch of LEDs
 * and an ultrasonic distance sensor + button to trigger program mode
 *
 * Basically the idea is to use the distance sensor to adjust the intensity of the pulse. 
 * The further your hand moves from the distance sensor the brighter the light gets
 * The closer your hand moves to the distance sensor the dimmer it gets
 * 
 * simple eh? ;)
 *
 */

/* Note this project is takjing a lot of ideas and code snippets from the 74HC595 arduino 
 * examples and also the DFRobot ultrasonic sensor demo code
 */

/*
 * Example urls to look at for setup:
 * http://www.arduino.cc/en/Tutorial/ShiftOut
 * http://www.dfrobot.com/wiki/index.php/URM37_V3.2_Ultrasonic_Sensor_(SKU:SEN0001)#The_sketch_for_PWM_passive_control_mode
 *
 */

/*
 * latch Pin connected to ST_CP of 74HC595
 * clock Pin connected to SH_CP of 74HC595
 * data Pin connected to DS of 74HC595
 */
int latchPin = 8;
int clockPin = 12;
int dataPin = 11;

// # Connection:
// #       Pin 1 VCC (URM V3.2) -> VCC (Arduino)
// #       Pin 2 GND (URM V3.2) -> GND (Arduino)
// #       Pin 4 PWM (URM V3.2) -> Pin 3 (Arduino)
// #       Pin 6 COMP/TRIG (URM V3.2) -> Pin 5 (Arduino)
// #
int URPWM = 3; // PWM Output 0－25000US，Every 50US represent 1cm
int URTRIG = 5; // PWM trigger pin
unsigned int Distance=0; // centimeter
short initial_brightness=100; // initial brightness of the leds
int program_led = 9; // visual aid acking programming mode
int program_button = 10; // set up a button here ;)

uint8_t EnPwmCmd[4]={
  0x44,0x02,0xbb,0x01
};    // distance measure command

void setup() {
  Serial.begin(115200);
  pinMode(program_led,OUTPUT);
  pinMode(program_button,INPUT);
  setup_ultrasonic();
  setup_lcds();
  set_brightness(initial_brightness);
}

void setup_lcds() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, initial_brightness);  
  digitalWrite(latchPin, HIGH); 
}

int set_brightness(unsigned long level) {
  if ( level > 255 ) {
    level = 255;
  }
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, level);  
  digitalWrite(latchPin, HIGH); 
}

void setup_ultrasonic(){ 
  pinMode(URTRIG,OUTPUT);                     // A low pull on pin COMP/TRIG
  digitalWrite(URTRIG,HIGH);                  // Set to HIGH
  pinMode(URPWM, INPUT);                      // Sending Enable PWM mode command

  for(int i=0;i<4;i++){
    Serial.write(EnPwmCmd[i]);
  } 
}

unsigned long adjust_distance() {                              // a low pull on pin COMP/TRIG  triggering a sensor reading
  digitalWrite(URTRIG, LOW);
  digitalWrite(URTRIG, HIGH);               // reading Pin PWM will output pulses

  unsigned long DistanceMeasured=pulseIn(URPWM,LOW);

  if (DistanceMeasured==50000) {              // the reading is invalid.
    return -1;  
  } else{
    Distance = DistanceMeasured / 50;           // every 50us low level stands for 1cm
    return Distance;
  }
}

void loop() {
  if ( digitalRead(program_button) ) {
    Serial.println("Enter program mode");
    pinMode(program_led, HIGH);
      unsigned long distance = adjust_distance();
      if ( distance > 0 ) { 
        set_brightness(distance);
      } else {
        Serial.print("Not sure what to do with set value: ");
        Serial.println(distance);
      }
    Serial.println("end program mode");
    pinMode(program_led, LOW);
  } else {
    delay(300);
  }
}
