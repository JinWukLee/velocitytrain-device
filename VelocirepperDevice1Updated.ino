/*
  @name Velocirepper code
  @author Jin, Michelle, Ben
  @date March 30, 2023
  @version 0.0.1  original code at end of ENAS 118  PM
  @version 0.0.2  cleaned up code at end of ENAS 118  JL
  @version 0.0.3  fixed errors  JL, ML
  @version 0.1.1  measurement filter  JL, ML
  @version 0.2.1  angle of rotation with gyroscope  JL, ML
  @version 0.3.1  angle fusion, new sounds, cleaner code  JL, ML
  @version 0.3.2  no sound delays, measurements in setup w/ help from Larry JL, ML
  @version 0.4.1  touchscreen ML, JL, BP
  @version 0.4.2  new graphics  BP
  @version 0.5.1  removed calibration step  ML
*/

float test_time1;
float test_time2;
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include "Adafruit_VL53L0X.h"
#include "pitches.h"


#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define YALEBLUE 0x08D1
#define SILVER 0xD659
#define GRAY 0x9C10
#define GOLD 0x8FE1
#define BRONZE 0x7547
#define GRAYISH 0xA572

#define LED_PIN 13
int Buzzer = 5;

#include <ILI9341_t3.h>
#include <font_Arial.h>  // from ILI9341_t3
#include <font_Impact.h>
#include <font_AwesomeF000.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

#define CS_PIN 8
#define TFT_DC 9
#define TFT_CS 10
// MOSI=11, MISO=12, SCK=13

#define MINPRESSURE 10
#define MAXPRESSURE 1000

XPT2046_Touchscreen ts(CS_PIN);
#define TIRQ_PIN 2
//XPT2046_Touchscreen ts(CS_PIN);  // Param 2 - NULL - No interrupts
//XPT2046_Touchscreen ts(CS_PIN, 255);  // Param 2 - 255 - No interrupts
//XPT2046_Touchscreen ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling

//touchscreen setup

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

int topbutton[5] = { 0, 50, 310, 120, 0 };  //nbutton[4]= state
//int midbutton[5] = { 0, 110, 300, 140, 0 };
int botbutton[5] = { 0, 130, 310, 200, 0 };
int enterbutton[5] = { 130, 180, 195, 240, 0 };
int homebutton[5] = { 5, 155, 100, 240, 0 };
int muteswitch[5] = { 220, 60, 260, 80, 1 };
int preset[5] = { 220, 100, 260, 120, 0 };

int increasebutton[5] = { 220, 100, 270, 150, 0 };
int decreasebutton[5] = { 50, 100, 100, 150, 0 };
int advdetailsbutton[5] = { 210, 5, 315, 50, 0 };
int nextsetbutton[5] = { 200, 190, 305, 230, 0 };

String state = "home";

int mypx;
int mypy;

Adafruit_MPU6050 mpu;

int counter1 = 1;
int counter2 = 0;
int counter3 = 0;
float V_Threshhold = 0.7;
int Reps = 1;
int ledState = LOW;
int weight = 45;
float avgPower;

//Delay screen
int Delay = 5;
int delay_countdown;

//for gyroscope
float distance_revised;

float theta_accel;
float theta_gyro = 0;
float theta_filt;
float alpha = 0.98;

float accelx, accely, accelz, gyrox, gyroy, gyroz = 0;
float accelx_off = 0.415;
float accely_off = -0.175; 
float accelz_off = -.725; 

float gyrox_off = 0.05;
float gyroy_off = 0; 
float gyroz_off = -0.03;


float accelx_scale = .998;
float accely_scale = 1;
float accelz_scale = 1.02;

//Adafruit_VL53LOX sample code
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
uint16_t period_ms = 37;
uint32_t budget_us = 33000;
VL53L0X_RangingMeasurementData_t measure;

//Setting up global variables
int distance;
float timer;
int prev_sVal;
float prev_timer;
float elapsed_time;
int delta_sVal;
float velocity;
float prev_velocity;

//Calibration and thresholds variables
int delta_d;
int cal_d_min;
int cal_d_max;
float d_high_threshold;
float d_low_threshold;
float vel_threshold_beg = .238;
float vel_threshold_end = .04;
int stop_cal = 0;

//Average velocity variables
int d_bottom;
int d_top;
int rep_time;
long beg_rep_time;
long end_rep_time;
int flag_bottom;
float avg_vel;
int above_threshold_counter;
float percent_above;
int downward;
int upward;
bool beg_rep_time_bool = false;

//Stops updating display every second
int stop_display = 0;

//Stops final sound after set
int stop_fsound = 0;
int reps_done = 0;

//Sound Effects Portion

int32_t frequency = 880;  //frequency (in Hz)
bool success;

unsigned long previousMillis = 0;
const long pauseBetweenNotes = 30;   // interval between notes (ms)
const long pauseBetweenNotes2 = 15;  // interval between notes (ms)
const long noteDuration = 80;        // (ms)
const long noteDuration2 = 45;       // (ms)
boolean outputTone = false;          // Records current state
const int MelodyLength4 = 4;
const int MelodyLength2 = 2;
const int goodNoise[MelodyLength2] = { 932, 1245 };
const int badNoise[MelodyLength2] = { 131, 131 };
const int marioMelody[MelodyLength4] = { 440, 440, 440, 880 };
const int calibfinishedMelody[MelodyLength4] = { 932, 1245, 932, 1245 };

int MelodyIndex = 0;
unsigned long currentMillis;
unsigned long calibTimeoutMillis;
boolean good = false;
boolean bad = false;
boolean mario = false;

//Melody

int melody_cal_start[] = {  //Calibration and count down
  NOTE_D6, NOTE_A5, NOTE_D6, NOTE_A6
};
int noteDurations_cal_start[] = { 12, 12, 12, 12 };

int melody_startup[] = {  //Start up and power down
  NOTE_A4, NOTE_CS5, NOTE_E5,
  NOTE_A5
};
int noteDurations_startup[] = {
  12, 12, 12, 12
};

int melody_endreps[] = {  //Final sound
  NOTE_G5, NOTE_F5, NOTE_E5,
  NOTE_D5, NOTE_C5
};
int noteDurations_endreps[] = {
  12, 12, 12, 12, 12
};

int melody_countdown[] = {  //Mario kart countdown
  NOTE_A4, NOTE_A4, NOTE_A4, NOTE_A5
};
int noteDurations_countdown[] = {
  8, 8, 8, 8
};

// Collecting rep values
float Rep_Array[14];
byte arrayIndex = 0;
float Average_Reps;
float sum;
int a;
int stop_sum = 0;
int test_var;

//teensy testing
int led = 13;

void setup() {

  //accelerometer setup
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

  //Teensy testing
  pinMode(led, OUTPUT);

  Serial.begin(115200);
  delay(400);

  //debug sensor connections
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
  }
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }


  //Power for TOF
  lox.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
  lox.startRangeContinuous(period_ms);
  lox.setMeasurementTimingBudgetMicroSeconds(budget_us);

  cal_d_min = 3000;
  //clean out initial sensor values
  for (int i = 0; i < 5; i++) {
    measurement();
  }

  //touchscreen initialization
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  ts.begin();
  ts.setRotation(1);

  //memory
  if (((int)EEPROM.read(3)) != 255) {
    V_Threshhold = ((int)EEPROM.read(3)) / 100.00;
  } else {
    V_Threshhold = 0.7;
  }

  if (((int)EEPROM.read(1)) != 255) {
    Reps = (int)EEPROM.read(1);
  } else {
    Reps = 1;
  }

  if (((int)EEPROM.read(2)) != 255) {
    Delay = (int)EEPROM.read(2);
  } else {
    Delay = 5;
  }
  if ((((int)EEPROM.read(4)) != 255) && (((int)EEPROM.read(5)) != 255)) {
    d_low_threshold = ((int)EEPROM.read(5) * 100) + ((int)EEPROM.read(4));
    Serial.print("Low threshold 1:    ");
    Serial.println(d_low_threshold);
  }
  if ((((int)EEPROM.read(6)) != 255) && (((int)EEPROM.read(7)) != 255)) {
    d_high_threshold = ((int)EEPROM.read(7) * 100) + ((int)EEPROM.read(6));
    Serial.print("High threshold:    ");
    Serial.println(d_high_threshold);
  }
  if (((int)EEPROM.read(8)) != 255) {
    muteswitch[4] = (int)EEPROM.read(8);
  } else {
    muteswitch[4] = 0;
  }
  if (((int)EEPROM.read(9)) != 255) {
    preset[4] = (int)EEPROM.read(9);
  } else {
    preset[4] = 0;
  }

  MainMenu();

  //start up sound
  for (int thisNote = 0; thisNote < 4; thisNote++) {
    int noteDuration = 1000 / noteDurations_startup[thisNote];
    tone(Buzzer, melody_startup[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(Buzzer);
  }
}

void loop() {
  //teensy testing
  digitalWrite(led, HIGH);
  currentMillis = millis();

  if (state != "StartReps" && state != "StartCalibration") {
    //touchscreen check if pressed
    TS_Point p = ts.getPoint();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      //---------------------------------------------------------------------
      mypx = map(p.x, 340, 3840, 0, 320);
      mypy = map(p.y, 200, 3800, 0, 240);
      screenPressed();
    }
  }
}

void measurement() {
  timer = micros();

  /* Get new sensor events with the readings */
  accelx = 0;
  accely = 0;
  accelz = 0;

  gyrox = 0;
  gyroy = 0;
  gyroz = 0;

  //from ToF
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!


  //from accelerometer
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accelx = accelx + (a.acceleration.x - accelx_off) / accelx_scale;
  accely = accely + (a.acceleration.y - accely_off) / accely_scale;
  accelz = accelz + (a.acceleration.z - accelz_off) / accelz_scale;

  gyrox = gyrox + (g.gyro.x - gyrox_off);
  gyroy = gyroy + (g.gyro.y - gyroy_off);
  gyroz = gyroz + (g.gyro.z - gyroz_off);

  theta_accel = (atan2(accely, accelz));
  theta_gyro = theta_gyro + gyrox * (.0415);
  theta_filt = alpha * (theta_filt + gyrox * (.0415)) + (1 - alpha) * theta_accel;

  //Measuring Instantaneous Velocity
  distance = measure.RangeMilliMeter * abs(cos(theta_filt));
  delta_sVal = distance - prev_sVal;
  elapsed_time = timer - prev_timer;
  if (abs(delta_sVal) <= 1000) {
    delta_sVal = distance - prev_sVal;
    velocity = ((float(delta_sVal)) / ((float(elapsed_time)) / 1000.0));
  } else {
    distance = abs(delta_sVal - distance);
    Serial.println("Uh oh");
  }
  if (abs(velocity) > 3){
    velocity = prev_velocity;
    Serial.println("Previous Velocity Used");
  }
  prev_timer = timer;
  prev_sVal = distance;
  prev_velocity = velocity;


  //testing section
  /*Serial.print(degrees(theta_accel));
    Serial.print("       ");
    Serial.print(degrees(theta_gyro));
    Serial.print("       ");*/
  Serial.print(timer / 1000000.000, 3);
  Serial.print("       ");
  Serial.print(degrees(theta_filt));
  Serial.print("       ");
  Serial.print(measure.RangeMilliMeter);
  Serial.print("       ");
  Serial.print(distance);
  Serial.print("       ");
  Serial.println(velocity);
}

void resetReps() {
  arrayIndex = 0;
  Average_Reps = 0;
  sum = 0;
  cal_d_min = 3000;
  cal_d_max = 0;
  stop_sum = 0;
  stop_display = 0;
  d_bottom = 0;
  d_top = 0;
  delay_countdown = 0;
  stop_fsound = 0;
  reps_done = 0;
  stop_cal = 0;
  above_threshold_counter = 0;

  //Clears the Array for next round of reps
  for (int x = 0; x < 14; x++) {
    Rep_Array[x] = 0;
  }
  delay(10);
}

void screenPressed() {

  //Debugging------------------------------------------------------------
  Serial.print("screen tapped at x: ");
  Serial.print(mypx);
  Serial.print(", y: ");
  Serial.print(mypy);
  Serial.println("");

  //p.x = map(p.x, TS_MINX,TS_MAXX,0,240);
  //p.y = map(p.y,TS_MINY,TS_MAXY,0,320);

  //       p.x = map(p.x, TS_MINX, TS_MAXX, tft.width(), 0);
  //   p.y = map(tft.height()-map(p.y, TS_MINY, TS_MAXY, tft.height(),0));

  //___________________________________________________________________________________________________________________________
  //Controlling Home Menu 1 Selections


  if (state == "home") {


    if (checkbutton(topbutton))  //first argument calls function check button which tells if user touched button.. second states that button state is true
    // The user has pressed inside the red rectangle
    {

      tft.fillRect(0, 50, 310, 120, YELLOW);
      tft.drawRect(0, 50, 310, 120, WHITE);


      V_ThreshMenu();
    }

//    if (checkbutton(midbutton)) {
//
//
//      tft.fillRect(271, 91, 28, 28, YELLOW);
//      tft.drawRect(270, 90, 30, 30, WHITE);
//
//
//      setState("Leaderboard");
//      LeaderboardMenu1();
//    }

   

    if (checkbutton(botbutton)) {

      tft.fillRect(0, 130, 310, 200, YELLOW);
      tft.drawRect(0, 130, 310, 200, WHITE);
      SettingsMenu();
    }
  }
  if (checkbutton(muteswitch)) {
    if (state == "Settings") {
      MuteSwitchHit(muteswitch[4]);
      delay(200);
    }
  }

  if (checkbutton(preset)) {
    if (state == "Settings") {
      presetSwitchHit(preset[4]);
      delay(200);
    }
  }


  //  USING ENTER BUTTON

  if (checkbutton(enterbutton)) {

    if (state == "Vinput") {
      delay(200);
      RepMenu();

    } else if (state == "Rep Menu") {
      delay(200);
      DelayMenu();

    } else if (state == "Delay Menu") {
      delay(200);
      delay_countdown = Delay;
      Countdown();
    } else if (state == "Input Weight") {
      delay(200);
      AdvancedResults();
    }

    //This part was originally included but leaderboard was taken out
//    else if (state == "Leaderboard1") {
//      delay(200);
//      LeaderboardMenu2();
//    } else if (state == "Leaderboard2") {
//      delay(200);
//      LeaderboardMenu1();
//    }
  }
  //Return Home at any point
  if (checkbutton(homebutton)) {
    Serial.print("Homebutton touched");
    resetReps();
    MainMenu();
  }

  if (checkbutton(increasebutton)) {
    if (state == "Vinput") {
      Serial.print("Hit increase button");
      V_Threshhold += 0.05;
      Serial.println(V_Threshhold);
      tft.fillRect(120, 80, 80, 80, BLACK);
      tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
      if (V_Threshhold > 1.5) {
        V_Threshhold = 1.5;
      }
      tft.setCursor(125, 120);
      tft.setFont(Impact_18);
      tft.setTextSize(3);
      tft.print(V_Threshhold);
      delay(200);
    } else if (state == "Rep Menu") {
      Serial.print("Hit increase button");
      Reps += 1;
      Serial.println(Reps);
      tft.fillRect(120, 80, 80, 80, BLACK);
      tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
      if ((Reps > 12)) {
        Reps = 12;
      }
      tft.setCursor(140, 120);
      tft.setTextSize(3);
      tft.setFont(Impact_18);
      tft.print(Reps);
      delay(200);
    } else if (state == "Delay Menu") {
      Serial.print("Hit increase button");
      Delay += 5;
      Serial.println(Delay);
      tft.fillRect(120, 80, 80, 80, BLACK);
      tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
      if (Delay > 25) {
        Delay = 25;
      }
      tft.setCursor(140, 120);
      tft.setTextSize(3);
      tft.setFont(Impact_18);
      tft.print(Delay);
      delay(200);
    } else if (state == "Input Weight") {
      Serial.print("Hit increase button");
      weight += 5;
      Serial.println(weight);
      tft.fillRect(120, 80, 80, 80, BLACK);
      tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
      tft.setCursor(140, 120);
      tft.setFont(Impact_18);
      tft.print(weight);
      delay(200);
    }
  }

  if (checkbutton(decreasebutton)) {
    if (state == "Vinput") {
      Serial.print("Hit decrease button");
      V_Threshhold -= 0.05;
      Serial.println(V_Threshhold);
      tft.fillRect(120, 80, 80, 80, BLACK);
      tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
      if (V_Threshhold <= 0.05) {
        V_Threshhold = 0.05;
      }
      tft.setCursor(125, 120);
      tft.setTextSize(3);
      tft.setFont(Impact_18);
      tft.print(V_Threshhold);
      delay(200);
    } else if (state == "Rep Menu") {
      Serial.print("Hit decrease button");
      Reps -= 1;
      Serial.println(Reps);
      tft.fillRect(120, 80, 80, 80, BLACK);
      tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
      if (Reps <= 1) {
        Reps = 1;
      }
      tft.setCursor(140, 120);
      tft.setTextSize(3);
      tft.setFont(Impact_18);
      tft.print(Reps);
      delay(200);
    } else if (state == "Delay Menu") {
      Serial.print("Hit decrease button");
      Delay -= 5;
      Serial.println(Delay);
      tft.fillRect(120, 80, 80, 80, BLACK);
      tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
      if (Delay <= 0) {
        Delay = 0;
      }
      tft.setCursor(140, 120);
      tft.setTextSize(3);
      tft.setFont(Impact_18);
      tft.print(Delay);
      delay(200);
    } else if (state == "Input Weight") {
      Serial.print("Hit decrease button");
      weight -= 5;
      Serial.println(weight);
      tft.fillRect(120, 80, 80, 80, BLACK);
      tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
      tft.setCursor(140, 120);
      tft.setFont(Impact_18);
      tft.print(weight);
      delay(200);
    }
  }

  if (state == "Results") {
    if (checkbutton(advdetailsbutton)) {
      inputWeight();
      delay(200);
    }
    if (checkbutton(nextsetbutton)) {
      resetReps();
      V_ThreshMenu();
      delay(200);
    }
  }
  if (state == "Advanced Results") {
    if (checkbutton(nextsetbutton)) {
      resetReps();
      V_ThreshMenu();
      delay(200);
    }
  }
}
bool checkbutton(int buttonName[]) {
  if ((buttonName[0] < mypx) && (buttonName[2] > mypx) && (buttonName[1] < mypy) && (buttonName[3] > mypy)) {

    return true;

  } else {
    return false;
  }
}

void drawButton(int r[], uint16_t Color) {
  tft.fillRect(r[0], r[1], r[2] - r[0], r[3] - r[1], Color);
  tft.drawRect(r[0], r[1], r[2] - r[0], r[3] - r[1], WHITE);
}

void MainMenu() {

  setState("home");

  //Main Menu Display
  tft.fillScreen(BLACK);
  drawMuteIcon();
  tft.fillRect(0, 0, 320, 40, YALEBLUE);
  tft.drawRect(0, 0, 320, 40, WHITE);  //Border rectangle
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.setCursor(90, 15);
  tft.setFont(Impact_18);
  tft.println("Velocirepper");


  //Top Selection box
  drawButton(topbutton, YALEBLUE);
  //Middle Selection box
 // drawButton(midbutton, BLACK);
 
  //Bottom Select box
  drawButton(botbutton, YALEBLUE);

  tft.setFont(Arial_28);
  //Exercise text
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.setCursor(80, 75);
  tft.println("Exercise");

//  //Leaderboard text
//  tft.setCursor(80, 115);
//  tft.println("Leaderboard");
  
  //Settings text
  tft.setCursor(82, 150);
  tft.println("Settings");



  //Battery Icon
  drawBattery();
}

void V_ThreshMenu() {

  setState("Vinput");
  tft.fillScreen(BLACK);
  drawMuteIcon();
  tft.fillRect(0, 0, 320, 40, YALEBLUE);
  tft.drawRect(0, 0, 320, 40, WHITE);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 15);
  tft.setFont(Impact_18);
  tft.println("Set Velocity (m/s)");

  drawHome();

  drawEnter();

  drawIncreaseButton();
  drawDecreaseButton();

  //Display box for velocity
  tft.fillRect(120, 80, 80, 80, BLACK);
  tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
  tft.setTextSize(3);
  tft.setCursor(125, 120);
  tft.print(V_Threshhold);

  drawBattery();
}

void RepMenu() {
  setState("Rep Menu");
  Serial.println("Rep Menu was called");
  tft.fillScreen(BLACK);
  drawMuteIcon();
  tft.fillRect(0, 0, 320, 40, YALEBLUE);
  tft.drawRect(0, 0, 320, 40, WHITE);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 15);
  tft.setFont(Impact_18);
  tft.println("# of Reps");

  drawHome();

  drawEnter();

  //Increase Reps button
  drawIncreaseButton();

  //Decrease Reps button

  drawDecreaseButton();

  //Display box for Reps
  tft.fillRect(120, 80, 80, 80, BLACK);
  tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
  tft.setTextSize(3);
  tft.setCursor(140, 120);
  tft.print(Reps);

  //Print text of previous stats

  tft.setTextSize(1);
  tft.setFont(Arial_8);
  tft.setTextColor(WHITE);
  tft.setCursor(10, 45);
  tft.println("Velocity:");
  tft.setCursor(65, 45);
  tft.print(V_Threshhold);
  tft.setCursor(90, 45);
  tft.print("m/s");

  drawBattery();
}

void DelayMenu() {

  setState("Delay Menu");

  tft.fillScreen(BLACK);
  tft.fillRect(0, 0, 320, 40, YALEBLUE);
  tft.drawRect(0, 0, 320, 40, WHITE);
  drawMuteIcon();
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 15);
  tft.setFont(Impact_18);
  tft.println("Delay");

  drawHome();
  drawEnter();

  //Increase Reps button
  drawIncreaseButton();

  //Decrease Reps button

  drawDecreaseButton();

  //Display box for Delay
  tft.fillRect(120, 80, 80, 80, BLACK);
  tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
  tft.setTextSize(3);
  tft.setCursor(140, 120);
  tft.print(Delay);

  //Print text of previous stats
  tft.setTextSize(1);
  tft.setFont(Arial_8);
  tft.setTextColor(WHITE);
  tft.setCursor(10, 45);
  tft.println("Velocity:");
  tft.setCursor(65, 45);
  tft.print(V_Threshhold);
  tft.setCursor(90, 45);
  tft.print("m/s");
  tft.setCursor(10, 55);
  tft.println("Reps:");
  tft.setCursor(65, 55);
  tft.print(Reps);

  drawBattery();
}

void Countdown() {

  setState("Countdown");
  tft.fillScreen(BLACK);

  tft.fillRect(0, 0, 320, 40, YALEBLUE);
  tft.drawRect(0, 0, 320, 40, WHITE);
  drawMuteIcon();
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 15);
  tft.setFont(Impact_18);
  tft.println("Countdown");


  drawHome();

  //Display box for Countdown
  tft.fillRect(110, 50, 100, 100, WHITE);
  tft.drawRect(110, 50, 100, 100, BLUE);  //Border rectangle
  tft.setTextColor(BLUE);
  tft.setTextSize(6);
  tft.setFont(Impact_48);
  tft.setCursor(140, 80);
  tft.print(Delay);

  tft.setFont(Arial_8);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 45);
  tft.println("Velocity:");
  tft.setCursor(65, 45);
  tft.print(V_Threshhold);
  tft.setCursor(90, 45);
  tft.print("m/s");
  tft.setCursor(10, 55);
  tft.println("Reps:");
  tft.setCursor(65, 55);
  tft.print(Reps);
  drawBattery();

  Serial.println(Delay);
  Serial.println(delay_countdown);

  //COUNTDOWN
  if (delay_countdown > 0) {
    for (int k = 0; k < Delay; k++) {
      tone(Buzzer, 400, 100);
      delay(1000);
      delay_countdown = delay_countdown - 1;
      tft.fillRect(110, 50, 100, 100, WHITE);
      tft.drawRect(110, 50, 100, 100, BLUE);  //Border rectangle
      tft.setTextColor(BLUE);
      tft.setFont(Impact_48);
      tft.setCursor(140, 80);
      tft.print(delay_countdown);
    }
  }
  setState("Calibration Bottom");
  Rep_Cycle();
}

void Rep_Cycle() {
  //CALIBRATION REP
  theta_accel = (atan2(accely, accelz));
  theta_gyro = 0;
  theta_filt = alpha * (theta_filt + gyrox * (.0415)) + (1 - alpha) * theta_accel;
  for (int i = 0; i < 5; i++) {  //clean out sensor values
    measurement();
  }
  delay(200);
  mario = true;
  outputTone = false;
  MelodyIndex = 0;
  d_low_threshold = 0;
  d_high_threshold = 0;

  for (int thisNote = 0; thisNote < 4; thisNote++) {  //starting sound
    int noteDuration = 1000 / noteDurations_cal_start[thisNote];
    tone(Buzzer, melody_cal_start[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(Buzzer);
  }
  tft.fillScreen(YALEBLUE);
  tft.setFont(Arial_24);
  tft.setTextColor(WHITE);
  tft.setFont(Impact_24);
  tft.setCursor(25, 15);
  tft.println("Start");
  tft.setCursor(25, 55);
  tft.println("Reps!");
  while ((state == "Calibration Bottom") || (state == "Calibration Top")) {
    measurement();
    currentMillis = millis();
    //distance and angle measurement function
    //Calibration steps
    if (state == "Calibration Bottom") {
      if ((cal_d_min > distance) && (distance != 0)) {  //cal_d_min = 3000
        if (velocity <= -0.2) {
          calibTimeoutMillis = millis();
          if (currentMillis - calibTimeoutMillis < 500) {
            downward++;
          } else {
            downward = 0;
            Serial.println("Downward reset");
          }
        }
        if ((velocity > 0.1) && (downward >= 2)) {  //changed the criteria to just be based on velocity, let's see if it works
          Serial.println("Condition met");
          cal_d_min = distance;
          d_bottom = distance;
          calibTimeoutMillis = millis();
          setState("Calibration Top");
          downward = 0;
        }
      }
    }
    //END OF BEG OF CALIB REP
    if (state == "Calibration Top") {
      if ((cal_d_max < distance) && (distance != 0)) {  //cal_d_max = 0
        if (velocity >= 0.2) {
          upward = 1;
          if (beg_rep_time_bool == false) {
            d_bottom = distance;
            cal_d_min = distance;
            Serial.println("bottom threshold:");
            Serial.println(d_bottom);
            beg_rep_time = millis();
            Serial.println("beg time found");
            beg_rep_time_bool = true;
          }
        }
        if ((velocity <= 0.2) && (distance > (cal_d_min + 100) && (upward == 1))) {
          stop_cal++;
          Serial.println("top threshold");
          cal_d_max = distance;
          Serial.println(cal_d_max);
          d_top = distance;
          if (stop_cal >= 1) {
            end_rep_time = millis();
            rep_time = end_rep_time - beg_rep_time;
            Serial.println("rep_time");
            Serial.println(rep_time);
            avg_vel = (d_top - d_bottom) / (rep_time * 1.0);
            Serial.println("top found");
            upward = 0;
            //Good/Bad rep sound
            if (avg_vel >= V_Threshhold) {  //Good rep
              good = true;
              above_threshold_counter++;
              Serial.println("Good");
            } else if (avg_vel < V_Threshhold) {  //Bad rep
              bad = true;
              Serial.println("Bad");
            }
            //Continue rep
            flag_bottom = 0;
            Rep_Array[arrayIndex] = avg_vel;  //put a value in entry 0
            arrayIndex++;                     //increment the array index
            reps_done++;
            setState("Rep Cycle");
            upward = 0;
            stop_cal = 0;
            beg_rep_time_bool = false;
          }
        }
        delta_d = cal_d_max - cal_d_min;
        d_high_threshold = cal_d_max - (.2 * (delta_d));
        d_low_threshold = cal_d_min + (.2 * (delta_d));
      }
    }  //END OF END OF CALIB REP

    while ((micros() - timer) < 41500) {}
  }

  //NORMAL REPS

  if ((reps_done < Reps) && (state == "Rep Cycle")) {  //Ensures sounds stop after done with set
    lox.rangingTest(&measure, false);                  // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {                    // phase failures have incorrect data
      theta_accel = (atan2(accely, accelz));
      theta_gyro = 0;
      theta_filt = theta_accel;
      outputTone = false;
      MelodyIndex = 0;
      while (state == "Rep Cycle") {  //Ensures no reps until after calibration
        currentMillis = millis();
        measurement();  //distance and angle measurement function
        goodSound();
        badSound();
        if (distance < d_low_threshold && velocity > vel_threshold_beg && flag_bottom == 0) {  //Cross bottom/beginning of rep
          test_var = 1;
          d_bottom = distance;
          beg_rep_time = millis();
          Serial.println("beg time found");
          flag_bottom = 1;
          Serial.println("bottom");
        }

        else if (distance > d_high_threshold && velocity < vel_threshold_end && flag_bottom == 1) {  //At top/end of rep
          test_var = 2;
          d_top = distance;
          end_rep_time = millis();
          rep_time = end_rep_time - beg_rep_time;
          Serial.println("rep_time");
          Serial.println(rep_time);
          avg_vel = (d_top - d_bottom) / (rep_time * 1.0);
          Serial.println("top");


          //Good/Bad rep sound
          if (avg_vel >= V_Threshhold) {  //Good rep
            good = true;
            above_threshold_counter++;
            Serial.println("Good");
          }

          else if (avg_vel < V_Threshhold) {  //Bad rep
            bad = true;
            Serial.println("Bad");
          }

          //Continue rep
          flag_bottom = 0;
          Rep_Array[arrayIndex] = avg_vel;  //put a value in entry 0
          arrayIndex++;                     //increment the array index
          reps_done++;

          //In between reps
        } else {
          test_var = 0;
        }


        if (reps_done >= Reps && stop_fsound == 0) {
          break;
        }
        while ((micros() - timer) < 41500) {}
      }
    }
    if (reps_done >= Reps && stop_fsound == 0) {  //Final sound (done with set)
      outputTone = false;
      while (bad == true || good == true) {
        currentMillis = millis();
        goodSound();
        badSound();
      }
      delay(200);
      for (int thisNote = 0; thisNote < 5; thisNote++) {
        int noteDuration = 1000 / noteDurations_endreps[thisNote];
        tone(Buzzer, melody_endreps[thisNote], noteDuration);
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        noTone(Buzzer);
      }
      stop_fsound = 1;
    }
  }
  ResultsMenu();
}

void drawGoldSilverBronze() {
  //1st 2nd and 3rd will be displayed on screen
  //1st place
  tft.fillCircle(295, 60, 12, BLACK);
  tft.fillCircle(295, 60, 10, GOLD);
  //Place
  tft.setTextColor(BLACK);
  tft.setCursor(10, 50);
  tft.setFont(Impact_16);
  tft.println("1.");
  //Medal Outer Mark
  tft.setFont(Impact_12);
  tft.setCursor(292, 58);
  tft.println("1");
  //Medal Inner Mark
  tft.setTextColor(GOLD);
  tft.setFont(Impact_8);
  tft.setCursor(294, 60);
  tft.println("1");

  //2nd place
  tft.fillCircle(295, 100, 12, BLACK);
  tft.fillCircle(295, 100, 10, SILVER);
  //Place
  tft.setTextColor(BLACK);
  tft.setCursor(10, 90);
  tft.setFont(Impact_16);
  tft.println("2.");
  //Medal Outer Mark
  tft.setFont(Impact_12);
  tft.setCursor(292, 98);
  tft.println("2");
  //Medal Inner Mark
  tft.setTextColor(SILVER);
  tft.setFont(Impact_8);
  tft.setCursor(294, 100);
  tft.println("2");

  //3rd place
  tft.fillCircle(295, 140, 12, BLACK);
  tft.fillCircle(295, 140, 10, BRONZE);
  //Place
  tft.setTextColor(BLACK);
  tft.setCursor(10, 130);
  tft.setFont(Impact_16);
  tft.println("3.");
  //Medal Outer Mark
  tft.setFont(Impact_12);
  tft.setCursor(292, 138);
  tft.println("3");
  //Medal Inner Mark
  tft.setTextColor(BRONZE);
  tft.setFont(Impact_8);
  tft.setCursor(294, 140);
  tft.println("3");
}

//void LeaderboardMenu1() {
//  setState("Leaderboard1");
//  tft.fillScreen(BLACK);
//
//  tft.fillRect(0, 0, 320, 40, MAGENTA);
//  tft.drawRect(0, 0, 320, 40, WHITE);
//  tft.fillRect(0, 40, 320, 200, GRAYISH);
//  drawMuteIcon();
//  tft.setTextColor(WHITE);
//  tft.setFont(Impact_8);
//  tft.setCursor(95, 5);
//  tft.println("Leaderboard");
//  tft.setCursor(95, 15);
//  tft.setFont(Impact_20);
//  tft.println("Peak Power");
//
//  drawGoldSilverBronze();
//  drawEnter();
//  drawHome();
//  drawBattery();
//}

//
//void LeaderboardMenu2() {
//  setState("Leaderboard2");
//  tft.fillScreen(BLACK);
//
//  tft.fillRect(0, 0, 320, 40, RED);
//  tft.drawRect(0, 0, 320, 40, WHITE);
//  tft.fillRect(0, 40, 320, 200, GRAYISH);
//  drawMuteIcon();
//  tft.setTextColor(WHITE);
//  tft.setFont(Impact_8);
//  tft.setCursor(80, 5);
//  tft.println("Leaderboard");
//  tft.setCursor(80, 15);
//  tft.setFont(Impact_20);
//  tft.println("Average Power");
//  drawGoldSilverBronze();
//  drawEnter();
//  drawHome();
//  drawBattery();
//}

void StartReps() {
  setState("StartReps");
  tft.fillScreen(YALEBLUE);
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.setCursor(25, 35);
  tft.println("Start Reps!");
  drawBarbell();
  drawMuteIcon();
  drawBattery();
}


void ResultsMenu() {
  setState("Results");
  tft.fillScreen(BLACK);
  tft.setFont(Impact_24);
  tft.setCursor(10, 15);
  tft.println("Results");
  drawHome();
  drawNextSet();
  drawAdvDetails();
  int spacer = 55;
  if (Reps <= 5) {
    tft.setFont(Arial_14);
    tft.setCursor(10, 55);
    tft.println("Reps");
    tft.setCursor(10, 95);
    tft.println("Velocity");
    tft.setCursor(10, 135);
    tft.println("% above threshold");
    for (int thisRep = 0; thisRep < Reps; thisRep++) {
      spacer = spacer + 40;
      if (spacer < 280) {
        tft.setFont(Arial_12);
        tft.setCursor(spacer, 57);
        tft.print(thisRep + 1);
        tft.setFont(Arial_12);
        tft.setCursor(spacer, 97);
        tft.print(Rep_Array[thisRep]);
      }
    }
  } else if (Reps > 5) {
    tft.setFont(Arial_14);
    tft.setCursor(10, 57);
    tft.println("Reps");
    tft.setCursor(65, 57);
    tft.print(Reps);
    tft.setFont(Arial_12);
    tft.setCursor(10, 97);
    tft.println("Average                                                                                                                                                                                Velocity:");
    //Finding the avg velocity of reps
    if (stop_sum == 0) {
      for (int a = 0; a < Reps; a++) {
        sum += Rep_Array[a];
      }
    }
    stop_sum = 1;  //Stops sum from continuously adding each array part
    Average_Reps = sum / Reps;
    tft.setCursor(65, 97);
    tft.print(Average_Reps);
  }
  tft.setCursor(180, 137);
  percent_above = (float)above_threshold_counter / (float)Reps;
  tft.print(percent_above * 100);
  tft.setCursor(240, 137);
  tft.print("%");

  if (preset[4] == 1) {
    EEPROM.update(3, V_Threshhold * 100);
    EEPROM.update(1, Reps);
    EEPROM.update(2, Delay);
  }
}

void SettingsMenu() {
  setState("Settings");
  tft.fillScreen(BLACK);
  tft.fillRect(0, 0, 320, 50, RED);
  tft.drawRect(0, 0, 320, 50, WHITE);
  drawMuteIcon();
  tft.setTextColor(WHITE);
  tft.setFont(Impact_18);
  tft.setCursor(30, 15);
  tft.println("Settings");
  tft.fillCircle(285, 25, 20, BLACK);
  tft.fillCircle(285, 25, 12, GRAY);
  tft.fillCircle(285, 13, 3, GRAY);
  tft.fillCircle(285, 37, 3, GRAY);
  tft.fillCircle(273, 25, 3, GRAY);      //left middle
  tft.fillCircle(297, 25, 3, GRAY);      //right middle
  tft.fillCircle(276.5, 16.5, 3, GRAY);  //top left
  tft.fillCircle(276.5, 33.5, 3, GRAY);  //bottom left
  tft.fillCircle(293.5, 33.5, 3, GRAY);  //bottom right
  tft.fillCircle(293.5, 16.5, 3, GRAY);  //top right
  tft.fillCircle(285, 25, 4, BLACK);     //middle ring

  drawMuteSwitch(muteswitch[4]);
  drawpresetSwitch(preset[4]);
  drawHome();
  drawBattery();
}

void drawBattery() {
  //Battery Icon
  tft.fillRect(310, 220, 5, 10, WHITE);
  tft.fillRect(280, 215, 30, 20, WHITE);
  tft.fillRect(282, 216, 5, 18, GREEN);
  tft.fillRect(289, 216, 5, 18, GREEN);
  tft.fillRect(296, 216, 5, 18, GREEN);
  tft.fillRect(303, 216, 5, 18, GREEN);
}

void drawHome() {
  //Home Button
  tft.fillRect(10, 180, 60,240, WHITE);
  //Chimney
  tft.fillRect(50, 185, 7.5, 13, MAGENTA);
  tft.fillTriangle(22, 205, 40, 185, 58, 205, RED);
  tft.fillRect(25, 205, 30, 30, BLUE);
  //Left Window
  tft.drawRect(28.3, 210, 7, 7, YALEBLUE);
  tft.drawRect(28.3, 210, 7, 7, WHITE);
  //Left Frame
  tft.fillRect(27.8, 217.5, 8, 1.5, CYAN);
  //Right Window
  tft.drawRect(45.5, 210, 7, 7, YALEBLUE);
  tft.drawRect(45.5, 210, 7, 7, WHITE);
  //White Frame
  tft.fillRect(45, 217.5, 8, 1.5, CYAN);
  //Door
  tft.fillRect(37.5, 222, 7.5, 13.5, RED);
  tft.fillCircle(42, 226, 0.75, YELLOW);
}

void drawEnter() {
  tft.fillRect(130, 180, 65, 60, YALEBLUE);
  tft.fillRect(140, 190, 10, 36, WHITE);
  tft.fillRect(140, 216, 40, 10, WHITE);
  tft.fillTriangle(190, 221, 170, 206, 170, 236, WHITE);
}

void drawNextSet() {
  tft.fillRect(200, 190, 105, 40, RED);
  tft.drawRect(200, 190, 105, 40, WHITE);
  tft.setTextColor(WHITE);
  tft.setFont(Arial_14);
  tft.setCursor(206, 205);
  tft.println("Next Set ->");
}
void drawAdvDetails() {
  tft.fillRect(200, 5, 105, 45, YALEBLUE);
  tft.drawRect(200, 5, 105, 45, WHITE);
  tft.setTextColor(WHITE);
  tft.setFont(Arial_14);
  tft.setCursor(210, 10);
  tft.println("Advanced");
  tft.setCursor(220, 30);
  tft.println("Results");
}


void setState(String s) {
  state = s;
  Serial.println("");

  Serial.print("state: ");
  Serial.println(state);
}
void marioSound() {
  if (mario == true) {
    if (outputTone) {
      if (currentMillis - previousMillis >= noteDuration) {
        previousMillis = currentMillis;
        noTone(Buzzer);
        outputTone = false;
      }
    } else {
      if (currentMillis - previousMillis >= pauseBetweenNotes) {
        previousMillis = currentMillis;
        tone(Buzzer, marioMelody[MelodyIndex]);
        outputTone = true;
        MelodyIndex += 1;
        if (MelodyIndex >= MelodyLength4 + 1) {
          MelodyIndex = 0;
          mario = false;
          noTone(Buzzer);
        }
      }
    }
  }
}
void goodSound() {
  if (good == true) {
    if (outputTone) {
      if (currentMillis - previousMillis >= noteDuration2) {
        previousMillis = currentMillis;
        noTone(Buzzer);
        outputTone = false;
      }
    } else {
      if (currentMillis - previousMillis >= pauseBetweenNotes2) {
        previousMillis = currentMillis;
        tone(Buzzer, goodNoise[MelodyIndex]);
        outputTone = true;
        MelodyIndex = MelodyIndex + 1;
        if (MelodyIndex >= MelodyLength2 + 1) {
          MelodyIndex = 0;
          good = false;
          noTone(Buzzer);
        }
      }
    }
  }
}
void badSound() {
  if (bad == true) {
    if (outputTone) {
      if (currentMillis - previousMillis >= noteDuration) {
        previousMillis = currentMillis;
        noTone(Buzzer);
        outputTone = false;
      }
    } else {
      if (currentMillis - previousMillis >= pauseBetweenNotes2) {
        previousMillis = currentMillis;
        tone(Buzzer, badNoise[MelodyIndex]);
        outputTone = true;
        MelodyIndex = MelodyIndex + 1;
        if (MelodyIndex >= MelodyLength2 + 1) {
          MelodyIndex = 0;
          bad = false;
          noTone(Buzzer);
        }
      }
    }
  }
}
void drawBarbell() {

  //Middle
  tft.fillRect(0, 20, 320, 200, YALEBLUE);
  tft.fillRect(75, 65, 170, 10, SILVER);
  tft.fillRect(80, 50, 15, 40, GRAY);
  tft.fillRect(225, 50, 15, 40, GRAY);
  tft.fillRect(245, 60, 14, 20, WHITE);
}
void drawTrafficLight() {
  //Background
  tft.fillScreen(BLACK);
  tft.fillRect(0, 0, 40, 40, BLACK);
  tft.fillRect(40, 0, 40, 40, WHITE);
  tft.fillRect(80, 0, 40, 40, BLACK);
  tft.fillRect(120, 0, 40, 40, WHITE);
  tft.fillRect(160, 0, 40, 40, BLACK);
  tft.fillRect(200, 0, 40, 40, WHITE);
  tft.fillRect(240, 0, 40, 40, BLACK);
  tft.fillRect(280, 0, 40, 40, WHITE);
  tft.fillRect(0, 40, 40, 40, WHITE);
  tft.fillRect(40, 40, 40, 40, BLACK);
  tft.fillRect(80, 40, 40, 40, WHITE);
  tft.fillRect(120, 40, 40, 40, BLACK);
  tft.fillRect(160, 40, 40, 40, WHITE);
  tft.fillRect(200, 40, 40, 40, BLACK);
  tft.fillRect(240, 40, 40, 40, WHITE);
  tft.fillRect(280, 40, 40, 40, BLACK);
  //Light animation

  tft.fillRect(140, 220, 40, 20, SILVER);  //pole
  tft.fillRect(110, 20, 100, 200, GRAY);
  tft.fillCircle(160, 60, 35, BLACK);   //black r
  tft.fillCircle(160, 120, 35, BLACK);  //black r
  tft.fillCircle(160, 180, 35, BLACK);  //black r

  delay(440);
  tft.fillCircle(160, 60, 30, RED);  //black r
  delay(440);
  tft.fillCircle(160, 120, 30, RED);  //black r
  delay(440);
  tft.fillCircle(160, 180, 30, RED);  //black r
  delay(880);
  tft.fillCircle(160, 60, 30, GREEN);  //black r

  tft.fillCircle(160, 120, 30, GREEN);  //black r

  tft.fillCircle(160, 180, 30, GREEN);  //black r
}
void drawCheck() {
  tft.setFont(AwesomeF000_96);
  tft.setCursor(110, 60);
  tft.print("F");
}
void drawX() {
  tft.fillRect(80, 40, 160, 160, RED);
  tft.drawRect(80, 40, 160, 160, WHITE);
  tft.setTextColor(WHITE);
  tft.setTextSize(20);
  tft.setCursor(110, 50);
  tft.println("X");
}
void drawMuteSwitch(int off) {
  tft.setTextSize(2);
  tft.setCursor(40, 60);
  tft.println("Mute Device");
  //ON State
  tft.fillRect(220, 60, 40, 20, GRAY);
  tft.fillRect(222, 62, 36, 16, BLACK);
  if (!off) {
    tft.fillRect(240.5, 60.5, 19, 18, GRAY);
    tft.fillRect(241, 62, 18, 16, GREEN);
  } else {
    tft.fillRect(220.5, 60.5, 19, 18, GRAY);
    tft.fillRect(221, 62, 18, 16, RED);
  }
}

void drawpresetSwitch(int off) {
  tft.setTextSize(2);
  tft.setCursor(40, 100);
  tft.println("Save Presets");
  //ON State
  tft.fillRect(220, 100, 40, 20, GRAY);
  tft.fillRect(222, 102, 36, 16, BLACK);
  if (off) {
    tft.fillRect(240.5, 100.5, 19, 18, GRAY);
    tft.fillRect(241, 102, 18, 16, GREEN);
  } else {
    tft.fillRect(220.5, 100.5, 19, 18, GRAY);
    tft.fillRect(221, 102, 18, 16, RED);
  }
}


void drawMuteIcon() {
  if (muteswitch[4] == 1) {
    //    tft.drawLine(250, 235, 270, 220, BLACK);
    //    tft.drawLine(250, 234, 270, 219, BLACK);
    //    tft.drawLine(250, 233, 270, 218, BLACK);
    tft.fillRect(250, 220, 5, 10, WHITE);
    tft.fillTriangle(250, 225, 265, 235, 265, 215, WHITE);
  } else if (muteswitch[4] == 0) {
    tft.fillRect(250, 220, 5, 10, WHITE);
    tft.fillTriangle(250, 225, 265, 235, 265, 215, WHITE);
    tft.drawLine(250, 235, 270, 220, RED);
    tft.drawLine(250, 234, 270, 219, RED);
    tft.drawLine(250, 233, 270, 218, RED);
  }
}
void drawIncreaseButton() {
  tft.fillTriangle(270, 125, 220, 100, 220, 150, YALEBLUE);
}
void drawDecreaseButton() {
  tft.fillTriangle(50, 125, 100, 100, 100, 150, YALEBLUE);
}

void MuteSwitchHit(int hit) {
  if (hit == 0) {
    muteswitch[4] = 1;
    hit = 1;
    Buzzer = 5;
    drawMuteIcon();
  } else if (hit == 1) {
    muteswitch[4] = 0;
    hit = 0;
    Buzzer = 6;
    tft.drawLine(250, 235, 270, 220, BLACK);
    tft.drawLine(250, 234, 270, 219, BLACK);
    tft.drawLine(250, 233, 270, 218, BLACK);
    drawMuteIcon();
  }
  drawMuteSwitch(muteswitch[4]);
  EEPROM.update(8, muteswitch[4]);
}

void presetSwitchHit(int hit) {
  if (hit == 0) {
    preset[4] = 1;
    hit = 1;
    Serial.println("turned on");
  } else if (hit == 1) {
    preset[4] = 0;
    hit = 0;
    Serial.println("turned off");
  }
  drawpresetSwitch(preset[4]);
  EEPROM.update(9, preset[4]);
}

void inputWeight() {
  setState("Input Weight");
  tft.fillScreen(BLACK);
  tft.fillRect(0, 0, 320, 40, YALEBLUE);
  tft.drawRect(0, 0, 320, 40, WHITE);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 15);
  tft.setFont(Impact_16);
  tft.println("Weight");
  tft.setCursor(50, 45);
  tft.println("Input Weight used (lbs)");
  tft.fillRect(120, 80, 80, 80, BLACK);
  tft.drawRect(120, 80, 80, 80, WHITE);  //Border rectangle
  tft.setFont(Impact_18);
  tft.setCursor(140, 120);
  tft.print(weight);

  drawIncreaseButton();
  drawDecreaseButton();

  drawEnter();
  drawHome();
  drawMuteIcon();
  drawBattery();
}

void AdvancedResults() {
  setState("Advanced Results");
  tft.fillScreen(YALEBLUE);
  tft.setFont(Impact_24);
  tft.setCursor(10, 15);
  tft.println("Advanced Results");

  int spacer = 55;
  if (Reps <= 5) {
    tft.setFont(Arial_14);
    tft.setCursor(10, 55);
    tft.println("Reps");
    tft.setCursor(10, 95);
    tft.println("Velocity");
    for (int thisRep = 0; thisRep < Reps; thisRep++) {
      spacer = spacer + 40;
      if (spacer < 240) {
        tft.setFont(Arial_12);
        tft.setCursor(spacer, 57);
        tft.print(thisRep + 1);
        tft.setFont(Arial_12);
        tft.setCursor(spacer, 97);
        tft.print(Rep_Array[thisRep]);
      }
    }
  }
  //Finding the avg velocity of reps
  for (int a = 0; a < Reps; a++) {
    sum += Rep_Array[a];
  }
  Average_Reps = sum / Reps;

  tft.setFont(Arial_14);
  tft.setCursor(10, 135);
  avgPower = (float(weight) / 2.2) * 9.81 * Average_Reps;
  tft.print("Average Power");
  tft.setFont(Arial_12);
  tft.setCursor(150, 135);
  tft.print(avgPower);
  tft.setCursor(205, 135);
  tft.print("Watts");

  drawNextSet();
  drawHome();
}
