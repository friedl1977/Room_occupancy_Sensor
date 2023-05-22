// Include VL53L1x ToF libraries // 
#include "vl53l1x_class.h"
#include "SparkFun_VL53L1X.h"

// VL53L1x ToF definitions // 
#define SHUTDOWN_PIN 2          // OPTIONAL Shut Down pin    
#define INTERRUPT_PIN 3         // OPTIONAL Interrupt pin

SFEVL53L1X distanceSensor(Wire);

float calibrated_zone_0;
float calibrated_zone_1;

int sum_zone_0 = 0;
int sum_zone_1 = 0;

int average_zone_0;
int average_zone_1;

int zone_state = 0;
uint16_t distance = 0;

int number_attempts;
int PathTrack[3][2];

int counter = 0;

static int DIST_THRESHOLD_MAX = 0.75;
//static int MIN_DISTANCE[] = {0, 0};

static int center[2] = {0,0};   // center of the two zones 
static int Zone = 0;

int ROI_height = 0;
int ROI_width = 0;

int ROUTE[3] = {0,0,0};
int Entry_state;
int Exit_state;


// Include ST7789 TFT Display libraries //
#include "../lib/Adafruit_GFX_RK/src/Adafruit_GFX.h"
#include "../lib/Adafruit_ST7735_RK/src/Adafruit_ST7789.h"
#include <SPI.h>

// ST7789 TFT  definitions // 
#define TFT_CS        S3
#define TFT_RST       D6        
#define TFT_DC        D5

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);     // Hardware SPI

float p = 3.1415926;

void setup(void) {

  // TFT Setup //
  Serial.begin(9600);
  tft.init(240, 320);                                               // Init ST7789 320x240

  uint16_t time = millis();
  tft.fillScreen(ST77XX_BLACK);
  time = millis() - time;

  // ToF Setup //
  Wire.begin();
  zones_calibration();

  if (distanceSensor.init() == false)
    Particle.publish("Counter is online");
    delay(2000);
    Particle.publish("I am ready", PRIVATE);

    tft.setTextWrap(false);
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(30, 30);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(36);
    tft.print("0");
    delay(50);
}

void zones_calibration() {

  center[0] = 231; //167
  center[1] = 167; //231 
  
  ROI_height = 8;
  ROI_width = 8;
  
  delay(500);
  Zone = 0;
  
  sum_zone_0 = 0;
  sum_zone_1 = 0;
  
  distance = 0;
  
  number_attempts = 20;
  
  for (int i=0; i<number_attempts; i++){
      
      // Zone 0
      distanceSensor.setIntermeasurementPeriod(55);
      distanceSensor.setDistanceModeLong();
      distanceSensor.setROI(ROI_height, ROI_width, center[0]);  // first value: height of the zone, second value: width of the zone
      delay(50);
      distanceSensor.setTimingBudgetInMs(50);
      distanceSensor.startRanging();                            //Write configuration bytes to initiate measurement
      distance = distanceSensor.getDistance();                  //Get the result of the measurement from the sensor
      distanceSensor.stopRanging();      
      sum_zone_0 = sum_zone_0 + distance;
      Zone++;
      Zone = Zone%2;

      // Zone 1
      distanceSensor.setIntermeasurementPeriod(55);
      distanceSensor.setDistanceModeLong();
      distanceSensor.setROI(ROI_height, ROI_width, center[1]);  // first value: height of the zone, second value: width of the zone
      delay(50);
      distanceSensor.setTimingBudgetInMs(50);
      distanceSensor.startRanging();                            //Write configuration bytes to initiate measurement
      distance = distanceSensor.getDistance();                  //Get the result of the measurement from the sensor
      distanceSensor.stopRanging();      
      sum_zone_1 = sum_zone_1 + distance;
      Zone++;
      Zone = Zone%2;
      
  }
  
  // after we have computed the sum for each zone, we can compute the average distance of each zone
  calibrated_zone_0 = sum_zone_0 / number_attempts;
  calibrated_zone_1 = sum_zone_1 / number_attempts;
  
  //DIST_THRESHOLD_MAX = (((calibrated_zone_0 + calibrated_zone_1)/2) * (threshold)); 
  
  Particle.publish("Zones_C: ", String(calibrated_zone_0) + " & " + String(calibrated_zone_1), PRIVATE);

}

void measure_zones() {
 
    distance = 0;
    sum_zone_0 = 0;
    sum_zone_1 = 0;
    number_attempts = 1;
    
    Zone = 0;

  // MONITORING ZONES  //

    for (int i=0; i<number_attempts; i++) {
    
    // increase sum of values in Zone 
      distanceSensor.setIntermeasurementPeriod(35);
      distanceSensor.setDistanceModeLong();
      distanceSensor.setROI(ROI_height, ROI_width, center[0]);  // first value: height of the zone, second value: width of the zone
      delay(25);
      distanceSensor.setTimingBudgetInMs(33);
      distanceSensor.startRanging();                            //Write configuration bytes to initiate measurement
      distance = distanceSensor.getDistance();                  //Get the result of the measurement from the sensor
      distanceSensor.stopRanging();      
      sum_zone_0 = sum_zone_0 + distance;
      Zone++;
      Zone = Zone%2;

      // increase sum of values in Zone 1
      distanceSensor.setIntermeasurementPeriod(35);
      distanceSensor.setDistanceModeLong();
      distanceSensor.setROI(ROI_height, ROI_width, center[1]);  // first value: height of the zone, second value: width of the zone
      delay(25);
      distanceSensor.setTimingBudgetInMs(33);
      distanceSensor.startRanging();                            //Write configuration bytes to initiate measurement
      distance = distanceSensor.getDistance();                  //Get the result of the measurement from the sensor
      distanceSensor.stopRanging();      
      sum_zone_1 = sum_zone_1 + distance;
      Zone++;
      Zone = Zone%2;
    }
    
        
// IF SOMEONE IS DETECTED -- CALL PATHTRACK //
   
    if ((sum_zone_0 <= (calibrated_zone_0 * 0.75)) || (sum_zone_1 <= (calibrated_zone_1 * 0.75))) {
        delay(10);
        Path();
        } else {
               }
}
    
void Path() {
    
    distance = 0;
    number_attempts = 2;
    
    Zone = 0;
    
    //START PATHTRACK //
    
    // measure 3 times //if ((ROUTE[0] == 1) && 
    for (int k=0; k<3; k++) {  
        
            sum_zone_0 = 0;
            sum_zone_1 = 0;
        
        for (int i=0; i<number_attempts; i++) {

    // increase sum of values in Zone 0 //
            distanceSensor.setIntermeasurementPeriod(35);
            distanceSensor.setDistanceModeLong();
            distanceSensor.setROI(ROI_height, ROI_width, center[0]);    // first value: height of the zone, second value: width of the zone
            delay(30);
            distanceSensor.setTimingBudgetInMs(33);
            distanceSensor.startRanging();                              //Write configuration bytes to initiate measurement
            distance = distanceSensor.getDistance();                    //Get the result of the measurement from the sensor
            distanceSensor.stopRanging();      
            sum_zone_0 = sum_zone_0 + distance;
            Zone++;
            Zone = Zone%2;
            
            delay(10);

    // increase sum of values in Zone 1 //
            distanceSensor.setIntermeasurementPeriod(35);
            distanceSensor.setDistanceModeLong();
            distanceSensor.setROI(ROI_height, ROI_width, center[1]);    // first value: height of the zone, second value: width of the zone
            delay(30);
            distanceSensor.setTimingBudgetInMs(33);
            distanceSensor.startRanging();                              //Write configuration bytes to initiate measurement
            distance = distanceSensor.getDistance();                    //Get the result of the measurement from the sensor
            distanceSensor.stopRanging();      
            sum_zone_1 = sum_zone_1 + distance;
            Zone++;
            Zone = Zone%2;
    
            PathTrack[k][0] = sum_zone_0 / number_attempts;
            PathTrack[k][1] = sum_zone_1 / number_attempts;

            if (sum_zone_0 < 20) { 
            counter = 0;
            tft.setTextWrap(false);
            tft.fillScreen(ST77XX_BLACK);
            tft.setCursor(30, 30);
            tft.setTextColor(ST77XX_WHITE);
            tft.setTextSize(36);
            tft.print(counter);
            //delay(1500);
            delay(1500);
            }
        }
    
       // Particle.publish("Zone 1: " + String(PathTrack[k][0]) + " &  Zone 2:" + String(PathTrack[k][1]), PRIVATE);  // DEBUG
        }
     
      people_counter_array();   
}    
  
void people_counter_array() {

    // PROCESS ENTRY // 

    if (PathTrack[0][0] < PathTrack[0][1]) {
        ROUTE[0] = 1;
    } else {
        ROUTE[0] = 2;
    }
    
    if (PathTrack[1][0] > PathTrack[1][1]) {
        ROUTE[1] = 1;
    } else {
        ROUTE[1] = 2;
        }
    
    if ((PathTrack[2][0] > PathTrack[1][0]) &&  (PathTrack[2][1] > PathTrack[1][1])) {
        ROUTE[2] = 1;
    } else {
        ROUTE[2] = 2;
    } 
    
    if ((ROUTE[0] == 1) && (ROUTE[1] == 1) && (ROUTE[2] == 1))  {
        Particle.publish("Entry ", PRIVATE);
        Entry_state = 1;
        Exit_state = 0;

    } else if ((ROUTE[0] == 2) && (ROUTE[1] == 2) && (ROUTE[2] == 1)) {
        Particle.publish("Exit ", PRIVATE);
        Entry_state = 0;
        Exit_state = 1;
    }
    
   // Particle.publish("Route : " + String(ROUTE[0]) + String(ROUTE[1]) + String(ROUTE[2]), PRIVATE);     // DEBUG

    ROUTE[0] = 0;
    ROUTE[1] = 0;
    ROUTE[2] = 0;
    
    count();
  
}

void count() {
    
    if ((Entry_state == 1) && (Exit_state == 0)) { 
        counter = counter + 1;
        Particle.publish("Counter: " + String(counter), PRIVATE);
        TFT();
        Entry_state = 0;
    }
    
    if ((Entry_state == 0) && (Exit_state == 1)) { 
        counter = counter - 1;    
        Particle.publish("Counter: " + String(counter), PRIVATE);
        TFT();
        Exit_state = 0;
    }    

    if (counter <= 0) {
        counter = 0;
    }
    
}


void loop() {

  measure_zones();

}

  void TFT() {
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(30, 30);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(36);
  tft.print(counter);
  //delay(1500);
  delay(1500);
}
