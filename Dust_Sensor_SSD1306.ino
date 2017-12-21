/*
 Standalone Sketch to use 
 Sharp Optical Dust Sensor GP2Y1010AU0F with SSD1306 128x32 OLED
 */
#include <Arduino.h>
#include <U8g2lib.h>
#include <avr/sleep.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
U8G2_SSD1306_128X32_UNIVISION_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED

// Dust Sensor 
int measurePin = A1;
int measureDust = 12;   

int samplingTime = 200;
int deltaTime = 70;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

// OLED
int sda = A4;
int scl = A5;

// Soft on
int interruptPin = 2;
int auxPin = 13;

void setup() {
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(auxPin, OUTPUT);
  digitalWrite(auxPin, HIGH);
  pinMode(measureDust, OUTPUT);
  u8g2.begin();
  Serial.begin(9600);
}

void loop() {
  int i;

  // Display dust monitor for 10 seconds
  for (i=0; i < 10; i++) {
    monitorDust();
  }
  // Sleep until touch sensor goes low
  sleepNow();
}

void monitorDust() {
  calcVoltage = 0;
  dustDensity = 0;
  int i;
  String indicator;
  
  for (i = 0; i < 10; i++) {
    digitalWrite(measureDust,LOW); // power on the LED
    delayMicroseconds(samplingTime);
  
    voMeasured = analogRead(measurePin); // read the dust value
    
    //delayMicroseconds(deltaTime);
    digitalWrite(measureDust,HIGH); // turn the LED off
    delayMicroseconds(sleepTime);
  
    // 0 - 2.24V mapped to 0 - 1023 integer values 
    calcVoltage += (voMeasured * (2.28 / 634));     
    delay(100);
  }
  Serial.print("Raw Signal Value: ");
  Serial.print(voMeasured);

  calcVoltage = calcVoltage/(i-1);
  if (calcVoltage < 0.5) calcVoltage = 0.56;
  Serial.print(" - Voltage: ");
  Serial.print(calcVoltage);
  
  dustDensity = ((0.16 * (calcVoltage - 0.55)) * 1000); 
  Serial.print(" - Dust Density [ug/m3]: ");
  Serial.print(dustDensity);
  int dust = dustDensity;
  Serial.println(" " + stateOfDust(dust));  
  display_bar(dust);
  delay(1000);
}

String stateOfDust(int dust) {
  String indicator;
  
  if (dust > 249) {
    indicator = "Hazardous";
  }
  else if (dust > 150) {
    indicator = "Extremely unhealthy";
  }
  else if (dust > 115) {
    indicator = "Unhealthy";
  }
  else if (dust > 75) {
    indicator = "Unhealthy for sensitive groups";
  }
  else if (dust > 35) {
    indicator = "Medium";
  }
  else {
    indicator = "Good";
  }
  return indicator;
}

void display_bar(int r) {
  char val[5];
  int i;
  int bar;
  int offset = 36;

  u8g2.firstPage();
  do {
    // Display a zero padded 3 digit value
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.setDrawColor(1);
    u8g2.setCursor(0, 16);
    sprintf(val, "%03d", r);
    u8g2.print(val);
    
    // Display micro-gram
    u8g2.setFont(u8g2_font_unifont_t_symbols);
    u8g2.drawGlyph(12, 28, 0x00b5);
    
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.setCursor(22, 28);
    u8g2.print("g");

    // Draw a filled box
    u8g2.setDrawColor(1);
    bar = map(r, 0, 300, 0, 127 - offset);
    u8g2.drawBox(offset, 8, bar, 22);
    
    // Draw the guage bars
    for (i = 1; i < 13; i++) {
      u8g2.setDrawColor(0);
      u8g2.drawBox(offset - 3 + (i * 10), 2, 3, 29);
    }
  } while ( u8g2.nextPage() );
}

void sleepNow()
{
    // blank the screen to save power 
    u8g2.setPowerSave(1);
    
    // shutdown auxilary systems
    digitalWrite(auxPin, LOW);
    
    attachInterrupt(digitalPinToInterrupt(interruptPin), pinInterrupt, LOW);
    
    // Choose our preferred sleep mode:
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // Set sleep enable (SE) bit:
    sleep_enable();
 
    // Put the device to sleep:
    sleep_mode(); 
}

void pinInterrupt(void)
{
    // Upon waking up, sketch continues from this point.
    sleep_disable();
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    digitalWrite(auxPin, HIGH);
    u8g2.setPowerSave(0);
}
