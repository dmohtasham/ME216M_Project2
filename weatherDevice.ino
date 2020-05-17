/*
 * ME216M Project 1
 * Delara Mohtasham 
 * Spring 2020
 * 
 * GET data from an API via HTTP and POST it to a shared feed
 * NOTE: fill out config.h with your wifi credientials and target API info.
 *       
*/

#include "config.h"  // Your wifi credentials go here
#include "icons.h"
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>
#include <TimeLib.h>
#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include <SAMD21PWM.h>

// button and buzzer
#define BUTTON_PIN 2
#define BUZZER_PIN 4 // pin on which connector is placed


// Leave this line unchanged
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define  NumberIcons 13
int XBMP_width[NumberIcons] = {clear_day_width,clear_night_width,few_clouds_day_width,few_clouds_night_width,scattered_clouds_width,broken_clouds_width,shower_width,rain_width,thunderstorm_width,snow_width,mist_width,WiFi_Logo_width,sunset_width};
int XBMP_height[NumberIcons] = {clear_day_height,clear_night_height,few_clouds_day_height,few_clouds_night_height,scattered_clouds_height,broken_clouds_height,shower_height,rain_height,thunderstorm_height,snow_height,mist_height,WiFi_Logo_height,sunset_height};
const uint8_t * XBMP_bits[NumberIcons] = {clear_day_bits,clear_night_bits,few_clouds_day_bits,few_clouds_night_bits,scattered_clouds_bits,broken_clouds_bits,shower_bits,rain_bits,thunderstorm_bits,snow_bits,mist_bits,WiFi_Logo_bits,sunset_bits};
int draw_state = 0;
int nextState = draw_state;
int lastState = draw_state;

// temperature sensor
#define I2C_ADDRESS 0x18 // should not need to change

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// WiFi and HTTP objects
WiFiSSLClient wifi;
HttpClient GetClient = HttpClient(wifi, ad_host, ad_port); 

// Sensor variables 
float currentTemp_F;
float lastTemp_F;
float averageTemp_F = 0;
float minTemp_F = 200;
float maxTemp_F = 0; 
int readingCount = 1;
int beepHighThresh = 80;
int beepLowThresh = 70;

// API variables
String currentInfo[3];
String todayInfo[3];
String nextDayInfo[3];
String dayAfterInfo[3];
char dayDate[5];
char nextDayDate[5];
char dayAfterDate[5];

// API time variables
int dayAfterMonth;
int dayAfterDay;
int nextDayMonth;
int nextDayDay;
int todayMonth;
int todayDay; 

// Timer variables
long startMillis;
long currentMillis;
long buzzerStartMillis;
long buzzerCurrentMillis;


        
void setup() {  
  Serial.begin(9600); // initialize serial
  while (!Serial && millis()<6000); // wait for serial to connect (6s timeout)
    
  // Light LED while trying to connect
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  
  // initialize the button pin as an input:
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT); 
  
  // start OLED
  u8g2.begin();

  // check for temperature sensor:
  if (!tempsensor.begin(I2C_ADDRESS)) {
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    while (1);
  }  
  tempsensor.setResolution(3); // sets the resolution mode of reading, the modes are defined in the table bellow:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true); // don't continue
  }
  // Check for firmware version
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(wifi_ssid);
    
    // Connect to WPA/WPA2 network. SSID and password are set in config.h
    WiFi.begin(wifi_ssid, wifi_password);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("WiFi successfully connected");
  digitalWrite(LED_BUILTIN, LOW); // turn off light once connected 

  buzzerStartMillis = millis();
  buzzerCurrentMillis = millis();
  startMillis = millis(); // initial start time
  getAPIData();
   
}

void loop() {
  getSensorData();

  buzzerCurrentMillis = millis();
  if ((buzzerCurrentMillis - buzzerStartMillis) >= 5000) {
    buzzer(); // check every 8 seconds
    buzzerStartMillis = buzzerCurrentMillis;
  }
  
  currentMillis = millis();
  if ((currentMillis - startMillis) >= 3.6e+6) {
      getAPIData(); //update every hour
      startMillis = currentMillis;
  }

  u8g2.clearBuffer();
  draw_SM();
  u8g2.sendBuffer();
}


void buzzer() {
  if (currentTemp_F > beepHighThresh) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      lastState = draw_state;
      draw_state = 4;
  } 
}

void alertState() {
  u8g2.drawStr(10, 0, "ALERT: HIGH TEMP");
  char tempInfo[20];
  (String(currentTemp_F) + "F").toCharArray(tempInfo,20);
  u8g2.drawStr(45, 35, tempInfo);
}

/* GetSensorData
 *  Collect readings from temperature sensor
 *  @param none
 *  @return none
 */
void getSensorData() {
  readingCount = readingCount + 1;
  tempsensor.wake(); // wake up MCP9808 - power consumption ~200 mikro Ampere
  
  // Read and print out the temperature, also shows the resolution mode used for reading.
  float r = tempsensor.getResolution();
  currentTemp_F = tempsensor.readTempF();
  
  if (currentTemp_F < minTemp_F) {
    minTemp_F = currentTemp_F;
  }
  if (currentTemp_F > maxTemp_F) {
    maxTemp_F = currentTemp_F;
  }
  
  averageTemp_F = (averageTemp_F *(readingCount - 2) + currentTemp_F) / (readingCount - 1);
  
  // delay(2000);
  tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling

} 


/* GetAPIData
 *  Sends a get request to the Weather server and returns advice.
 *  @param none
 *  @return none
 */
void getAPIData() {
  // Make sure we're connected to WiFi
  if (WiFi.status() == WL_CONNECTED) {

    // Create a GET request to the weather path
    GetClient.get(ad_path);
    Serial.println("[HTTP] GET... Weather Requested");
    
    // read the status code and body of the response
    int statusCode = GetClient.responseStatusCode();
    String response = GetClient.responseBody();

    
    // Check for the OK from the Server
    if(statusCode == 200) {
        Serial.println("[HTTP] GET received reply!"); 
        
        // Parse Server response as JSON document
        DynamicJsonDocument doc(GetClient.contentLength());  // doc = JSON object
        deserializeJson(doc, response); // response = JSON object document
        //serializeJsonPretty(doc, Serial); 
         
        // Set output to the weather data from the JSON
        // current time's data
        String currentData_str = doc["current"]["weather"][0]["main"]; 
        String currentIcon_str = doc["current"]["weather"][0]["icon"];
        String currentTemp_str = doc["current"]["temp"];
        currentInfo[0] = currentData_str;
        currentInfo[1] = currentTemp_str;
        currentInfo[2] = currentIcon_str;
        
        // todays's data 
        String todayTime_str = doc["daily"][0]["dt"];
        time_t todayTime = todayTime_str.toInt();
        todayMonth = month(todayTime);
        todayDay = day(todayTime);
        
        String dailyData_str = doc["daily"][0]["weather"][0]["main"]; 
        String dailyIcon_str = doc["daily"][0]["weather"][0]["icon"];
        String dailyTemp_str = doc["daily"][0]["temp"]["day"];
        //String dailyMaxTemp_str = doc["daily"][0]["temp"]["min"];
        //String dailyMinTemp_str = doc["daily"][0]["temp"]["max"];
        todayInfo[0] = dailyData_str;
        todayInfo[1] = dailyTemp_str;
        todayInfo[2] = dailyIcon_str;

        // tomorrow and day after's data 
        String nextDayTime_str = doc["daily"][1]["dt"];
        String dayAfterTime_str = doc["daily"][2]["dt"]; /// NOT WORKING
        time_t nextDayTime = nextDayTime_str.toInt();
        time_t dayAfterTime = dayAfterTime_str.toInt();
        nextDayMonth = month(nextDayTime);
        nextDayDay = day(nextDayTime);
        dayAfterMonth = month(dayAfterTime);
        dayAfterDay = day(dayAfterTime);

        String nextDayIcon_str = doc["daily"][1]["weather"][0]["icon"];
        String nextDayData_str = doc["daily"][1]["weather"][0]["main"]; /// NOT WORKING
        String dayAfterIcon_str = doc["daily"][2]["weather"][0]["icon"];
        String dayAfterData_str = doc["daily"][2]["weather"][0]["main"]; /// NOT WORKING

        nextDayInfo[0] = nextDayData_str;
        nextDayInfo[1] = nextDayIcon_str;
        dayAfterInfo[0] = dayAfterData_str;
        dayAfterInfo[1] = dayAfterIcon_str;

        
    } else if (statusCode > 0) {
        // Server issue
        Serial.print("[HTTP] GET... Received response code: "); 
        Serial.println(statusCode);
    } else {
        // Client issue
        Serial.print("[HTTP] GET... Failed, error code: "); 
        Serial.println(statusCode);
    }
  } else {
    Serial.println("[WIFI] Not connected");
  }

} 

 // OLED CODE
void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void glanceableMode() {
  u8g2.drawStr(0, 0, "Ambient temp");
  //sprintf(buffer, "%d.%d", int(currentTemp_F), int(currentTemp_F*1000)%1000); 
  char tempInfo[20];
  (String(currentTemp_F) + "F").toCharArray(tempInfo,20);
  u8g2.drawStr(45, 35, tempInfo);
 
}

void detailedMode() {
  u8g2.drawStr(0, 0, "Today's ambient temp");

  char tempInfo[20];
  (String(currentTemp_F) + "F").toCharArray(tempInfo,20);
  u8g2.drawStr(45, 15 , tempInfo);

  char tempInfo_avg[30];
  ("Average temp: " + String(averageTemp_F) + "F").toCharArray(tempInfo_avg,30);
  u8g2.drawStr(0, 35, tempInfo_avg);

  char tempInfo_max[30];
  ("Max temp: " + String(maxTemp_F) + "F").toCharArray(tempInfo_max,30);
  u8g2.drawStr(0, 45, tempInfo_max);

  char tempInfo_min[30];
  ("Min temp: " + String(minTemp_F) + "F").toCharArray(tempInfo_min,30);
  u8g2.drawStr(0, 55, tempInfo_min);

}

void localWeatherMode() {
   u8g2.drawStr(0, 10, "Current forecast");
   char currentData[20];
   currentInfo[0].toCharArray(currentData,20);
   u8g2.drawStr(0, 30, currentData);
   char currentTemp[8];
   (currentInfo[1] + "F").toCharArray(currentTemp,8);
   u8g2.drawStr(0, 40, currentTemp);
   int draw_state = currentInfo[2].toInt() ;
   u8g2.drawXBMP(90, 30, XBMP_width[draw_state], XBMP_height[draw_state], XBMP_bits[draw_state]); 

}

void localForecastMode() {
   // TODAY
   String todayDate_str = String(todayMonth) + "/" + String(todayDay);
   char todayDate[8];
   todayDate_str.toCharArray(todayDate,8); 
   u8g2.drawStr(0, 10, todayDate); // date
   char todayData[20];
   todayInfo[0].toCharArray(todayData,20);
   u8g2.drawStr(40, 10, todayData); // textual condition
   draw_state = todayInfo[2].toInt() ;
   u8g2.drawXBMP(90, 0, XBMP_width[draw_state], XBMP_height[draw_state], XBMP_bits[draw_state]); // icon

   // NEXT DAY
   String nextDayDate_str = String(nextDayMonth) + "/" + String(nextDayDay);
   char nextDayDate[8];
   nextDayDate_str.toCharArray(nextDayDate,8); 
   u8g2.drawStr(0, 30, nextDayDate); // date
   char nextDayData[20];
   nextDayInfo[0].toCharArray(nextDayData,20);
   u8g2.drawStr(40, 30, nextDayData); // textual condition
   draw_state = nextDayInfo[1].toInt() ;
   u8g2.drawXBMP(90, 20, XBMP_width[draw_state], XBMP_height[draw_state], XBMP_bits[draw_state]); // icon
   
   // DAY AFTER
   String dayAfterDate_str = String(dayAfterMonth) + "/" + String(dayAfterDay);
   char dayAfterDate[8];
   dayAfterDate_str.toCharArray(dayAfterDate,8); 
   u8g2.drawStr(0, 50, dayAfterDate); // date
   char dayAfterData[20];
   dayAfterInfo[0].toCharArray(dayAfterData,20);
   u8g2.drawStr(40, 50, dayAfterData); // textual condition
   draw_state = dayAfterInfo[1].toInt() ;
   u8g2.drawXBMP(90, 40, XBMP_width[draw_state], XBMP_height[draw_state], XBMP_bits[draw_state]); // icon

}

bool buttonPress() {
    bool eventHappened = false; // Initialize eventHappened flag to false
    int eventParameter = 0;

    // Create variable to hold last button pin reading
    // note: static variables are only initialized once and keep their value between function calls.
    static int lastButtonPinReading = LOW;

    // Read value of button pin
    int thisButtonPinReading = digitalRead(BUTTON_PIN);

    // Check if button was pressed
    if (thisButtonPinReading != lastButtonPinReading) { 
      if (thisButtonPinReading == 1) {
        // Set flag so event will be posted
        eventHappened = true;
      }
    }  
  
   // Update last button reading to current reading
   lastButtonPinReading = thisButtonPinReading;

   return eventHappened;
}

void draw_SM() {
  u8g2_prepare();
  switch(draw_state) {
    case 0: 
        localWeatherMode(); 
        if (buttonPress()) {
          lastState = draw_state;
          nextState = 1;         
        } 
        break;
    case 1: 
        localForecastMode(); 
        if (buttonPress()) {
          lastState = draw_state;
          nextState = 2;         
        }
        break;
    case 2: 
        glanceableMode(); 
        if (buttonPress()) {
          lastState = draw_state;
          nextState = 3;         
        }
        break;
    case 3: 
        detailedMode(); 
        if (buttonPress()) {
          lastState = draw_state;
          nextState = 0;         
        }
        break;
    case 4:
        alertState();
        if (buttonPress()) {
          nextState = lastState;
          lastState = draw_state;
        } else {
          nextState = 4;
        }
        break;
    default: 
        Serial.println("STATE: Unknown State");
        break;
  }
  draw_state = nextState;
} 
