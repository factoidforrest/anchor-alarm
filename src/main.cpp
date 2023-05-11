#include <TinyGPSPlus.h>
#include <ezButton.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_adc_cal.h"
#include <pwmWrite.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define GPS_BAUD 9600
#define MYPORT_TX 12
#define MYPORT_RX 13

#define ROTARY_PIN_A 6
#define ROTARY_PIN_B 7
#define ROTARY_PIN_BTN 10

#define BUZZER_PIN 4
#define BUZZER_PWM_CHANNEL 0
#define BUZZER_PWM_FREQUENCY 1000
#define BUZZER_PWM_RESOLUTION 8

#define MAX_DISTANCE_FT 2000 // maximum allowable distance

Pwm pwm = Pwm();

TinyGPSTime lastGpsLock = TinyGPSTime();

TinyGPSPlus gps;

double anchor_lat, anchor_lng;
bool is_armed = false;
ezButton button(10); 

#define BAT_ADC    2

float Voltage = 0.0;

// this is for the vbat
uint32_t readADC_Cal(int ADC_Raw)
{
    esp_adc_cal_characteristics_t adc_chars;

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}


int alarm_range = 250;

void read_encoder() {
  // cli();
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ROTARY_PIN_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ROTARY_PIN_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    alarm_range += 5;              // Increase counter
    encval = 0;
  }
  else if( encval < -3 ) {  // Four steps backwards
   alarm_range -=5;               // Decrease counter
   encval = 0;
  }
  // sei();
}

void setup() {
    Serial.begin(9600);

    WiFi.mode( WIFI_OFF );
    btStop();

    pinMode(ROTARY_PIN_A, INPUT_PULLUP);
    pinMode(ROTARY_PIN_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A), read_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B), read_encoder, CHANGE);

    pinMode(ROTARY_PIN_BTN, INPUT_PULLUP);
    button.setDebounceTime(50); // set debounce time to 50 milliseconds

    Voltage = (readADC_Cal(analogRead(BAT_ADC))) * 2 ;

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    // Display static text
    display.println("Startup, waiting for GPS...");
    
    display.println(String("VBat: " + String(Voltage) + "v").c_str());
    display.display(); 

}



void displayGPS() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(String("Alarm: " + String(is_armed ? "ARMED" : "Disarmed")).c_str());
  display.println("Locked: " + String(gps.satellites.value()));
  display.println("Accuracy(meters): " + String(gps.hdop.value()));

  display.println(String("VBat: " + String(Voltage) + "v").c_str());

  display.println(String("Range(feet): " + String(alarm_range) + "ft").c_str());

  if (is_armed) {
    double current_distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), anchor_lat, anchor_lng);
    current_distance *= 3.281; // convert to feet
    display.println(String("Distance: " + String(current_distance, 2) + "ft").c_str());
  }

  display.display();
}

void alarm(bool isOn){
  pwm.write(BUZZER_PIN, isOn ? 127 : 0, 2000);//2000, isOn ? 65535 : 0);
}

void checkDistance() {
  if (is_armed) {
    double current_distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), anchor_lat, anchor_lng);
    current_distance *= 3.281; // convert to feet
      if (current_distance > alarm_range) {
      alarm(true);
    } else {
      alarm(false);
    }
  } else {
    alarm(false);
  }
}



void lockLost() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("GPS Lock: No");
  display.println("Locked: " + String(gps.satellites.value()));
  display.println(lastGpsLock.age());

  display.display();
  if (is_armed) {
    alarm(true);
  }
}

void loop() {
  button.loop();
  
   if (lastGpsLock.isValid() && lastGpsLock.age() > 10000 && is_armed){
    lockLost();
   }


  Voltage = (readADC_Cal(analogRead(BAT_ADC))) * 2 ;
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())) {
      if (!gps.location.isValid() && lastGpsLock.age() > 10000) {
          lockLost();
      } else {
        lastGpsLock = gps.time;
        if (button.getCount() > 0) {
          button.resetCount();
          if (is_armed) {
            is_armed = false;
          } else {
            is_armed = true;
            anchor_lat = gps.location.lat();
            anchor_lng = gps.location.lng();
          }
        }
        displayGPS();
        checkDistance();
      }
    } else {
        display.clearDisplay();
        display.println("GPS Error");
    }
  }
}


