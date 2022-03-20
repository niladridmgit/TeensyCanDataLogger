#include <TimeLib.h>
#include <Bounce.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "SdFat.h"
#include "RingBuf.h"
#include "SD.h"
#include "MTP.h"
#include <TinyGPSPlus.h>
#include <TinyMPU6050.h>
#include <FlexCAN_T4.h>   // if defined before SdFat.h and RingBuf.h Teensy will keep restart when initSD_card function called (Niladri) //

#define USE_SD  1         // SDFAT based SDIO and SPI

#if USE_EVENTS==1
  extern "C" int usb_init_events(void);
#else
  int usb_init_events(void) {}
#endif



#if defined(__IMXRT1062__)
  // following only as long usb_mtp is not included in cores
  #if !__has_include("usb_mtp.h")
    #include "usb1_mtp.h"
  #endif
#else
  #ifndef BUILTIN_SDCARD 
    #define BUILTIN_SDCARD 254
  #endif
  void usb_mtp_configure(void) {}
#endif


/****  Start device specific change area  ****/
// SDClasses 
#if USE_SD==1
  // edit SPI to reflect your configuration (following is for T4.1)
  #define SD_MOSI 11
  #define SD_MISO 12
  #define SD_SCK  13

  #define SPI_SPEED SD_SCK_MHZ(33)  // adjust to sd card 

  #if defined (BUILTIN_SDCARD)
    const char *sd_str[]={"logger","sd1"}; // edit to reflect your configuration
    const int cs[] = {BUILTIN_SDCARD,10}; // edit to reflect your configuration
  #else
    const char *sd_str[]={"sd1"}; // edit to reflect your configuration
    const int cs[] = {10}; // edit to reflect your configuration
  #endif
  const int nsd = sizeof(sd_str)/sizeof(const char *);

SDClass sdx[nsd];
#endif


MTPStorage_SD storage;
MTPD    mtpd(&storage);

bool mtp_enable =false;

// initialize i2c oled display //
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// initialize CAN //
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

#define TIME_HEADER  "T"   // Header tag for serial time sync message

#define LOG_FILE_SIZE_SD 10485760 // 10mb

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 40

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
#define LOG_FILE_SIZE 10*25000*600  // 150,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 400*512
#define LOG_FILENAME "CANLog.asc"

#define LOG_BTN  5
#define LED_PIN  13
#define BTN_TIMEOUT 3000

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

// Max RingBuf used bytes. Useful to understand RingBuf overrun.
size_t maxUsed = 0;

elapsedMicros elapselogtime = 0;
int period = 1000;
unsigned long time_now = 0;

uint32_t logFileSize = 0;
uint32_t newFileCount = 0;

// Button Initialization //
Bounce logButton = Bounce(LOG_BTN, 15); // 15 = 15 ms debounce time
bool log_enable = false;
bool triggerCommand = false;
int logButton_state=0;
long btn_duration=0;

TinyGPSPlus gps;

bool rtcSync = false;


// Function prtotype declaration //
void initSD_card();
void stopLogging();
void startLogging();
void digitalClockDisplay();
void printDigits(int digits);
unsigned long processSyncMessage();
void printDisplay();
void displayMsgOled();
void parseGpsNmea();
void sensorDataWrite(uint8_t sensortype);
void storage_configure();
void checkButton();
void displayMsgOled_generic(char *msg);

char disp_msg_1[20] = { "Please Wait" };
char disp_msg_2[20] = { " " };
char disp_msg_3[20] = { " " };
char disp_msg_4[20] = { " " };
char disp_msg_5[20] = { " " };

struct mpu6050
{
  int16_t angleX_pitch;
  int16_t angleY_roll;
  int16_t angleZ_Yaw;
  
}mpu_6050_sensor;

struct GPSLocation
{
  float gps_lat;
  float gps_lng;
}GPS_Location;

MPU6050 mpu (Wire1); //SCL1(16), SDA1(17)//

time_t getTeensy3Time();

unsigned long mpu6050_task = 0;

void setup(void) {

  pinMode(LOG_BTN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  setSyncProvider(getTeensy3Time);

  u8g2.begin();          //SCL0(19), SDA0(18)//
  u8g2.clearBuffer();    // clear the internal memory

 // Initialization
  mpu.Initialize();

  // Calibration
  Serial7.begin(115200); // Debug and Config
  Serial2.begin(9600);  // GPS Module Pin7 Rx, Pin8 Tx as per https://www.pjrc.com/teensy/td_uart.html

  delay(400);
  
  Serial7.println("=====================================");
  Serial7.println("GPS NEO-6M Setting");
  sprintf(disp_msg_1, "GPS conf %s", "InProgress");
  displayMsgOled();
  gpsSetting();
  
  Serial7.println("=====================================");
  Serial7.println("Starting MPU6050 calibration.........");
  sprintf(disp_msg_1, "MPU6050 %s", "calibration");
  displayMsgOled();
  mpu.Calibrate();
  Serial7.println("Calibration complete!");
  Serial7.println("Offsets:");
  Serial7.print("GyroX Offset = ");
  Serial7.println(mpu.GetGyroXOffset());
  Serial7.print("GyroY Offset = ");
  Serial7.println(mpu.GetGyroYOffset());
  Serial7.print("GyroZ Offset = ");
  Serial7.println(mpu.GetGyroZOffset());

  if (timeStatus() != timeSet) {
    Serial7.println("Unable to sync with the RTC");
  }
  else {
    Serial7.println("RTC has set the system time");
  }

  digitalClockDisplay();
  delay(1000);
  
  Serial7.println("=====================================");
  Can0.begin();
  Can0.setBaudRate(500000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();

}

void gpsSetting()
{
  delay(1000);
  byte GGA_off[26] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x01, 0x00, 0x24, 0xb5, 0x62, 0x06, 0x01,0x02, 0x00, 0xf0, 0x00, 0xf9, 0x11 };
  Serial2.write(GGA_off, sizeof(GGA_off));  
  delay(1000);
  byte GLL_off[26] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x01, 0x00, 0x00,0x00, 0x00, 0x00, 0x01, 0x01, 0x2b, 0xb5, 0x62, 0x06, 0x01,0x02, 0x00, 0xf0, 0x01, 0xfa, 0x12 };
  Serial2.write(GLL_off, sizeof(GLL_off));
  delay(1000);
  byte GSA_off[26] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x02, 0x00, 0x00,0x00, 0x00, 0x00, 0x01, 0x02, 0x32, 0xb5, 0x62, 0x06, 0x01,0x02, 0x00, 0xf0, 0x02, 0xfb, 0x13 };
  Serial2.write(GSA_off, sizeof(GSA_off));
  delay(1000);
  byte GSV_off[26] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x03, 0x00, 0x00,0x00, 0x00, 0x00, 0x01, 0x03, 0x39, 0xb5, 0x62, 0x06, 0x01,0x02, 0x00, 0xf0, 0x03, 0xfc, 0x14 };
  Serial2.write(GSV_off, sizeof(GSV_off));
  delay(1000);
  byte VTG_off[26] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x05, 0x00, 0x00,0x00, 0x00, 0x00, 0x01, 0x05, 0x47, 0xb5, 0x62, 0x06, 0x01,0x02, 0x00, 0xf0, 0x05, 0xfe, 0x16 };
  Serial2.write(VTG_off, sizeof(VTG_off));
  delay(1000);
  byte save_config[29] = {0xb5, 0x62, 0x06, 0x09, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00,0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1d,0xab, 0xb5, 0x62, 0x0a, 0x04, 0x00,0x00,0x0e,0x34 };
  Serial2.write(save_config, sizeof(save_config));
  delay(1000);
  Serial2.flush();

}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1646932827; //2022

  if(Serial7.find(TIME_HEADER)) {
     pctime = Serial7.parseInt();
     return pctime;
     if( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
       pctime = 0L; // return 0 to indicate that the time is not valid
     }
  }
  return pctime;
}

void printDisplay()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(2, 12, (const char*)disp_msg_1);
  u8g2.drawStr(2, 24, (const char*)disp_msg_2);
  u8g2.drawStr(2, 36, (const char*)disp_msg_3);
  u8g2.drawStr(2, 48, (const char*)disp_msg_4);
  u8g2.drawStr(2, 60, (const char*)disp_msg_5);
  u8g2.sendBuffer();          // transfer internal memory to the display
}

void displayMsgOled_generic(char *msg)
{
  memset(disp_msg_1,'\0',20);
  memset(disp_msg_2,'\0',20);
  memset(disp_msg_3,'\0',20);
  memset(disp_msg_4,'\0',20);
  memset(disp_msg_5,'\0',20);
  sprintf(disp_msg_2, "%s", msg);
  printDisplay();
}

void displayMsgOled()
{
  Serial7.println(disp_msg_1);
  printDisplay();
}

void digitalClockDisplay()
{
  sprintf(disp_msg_1, "%02d:%02d:%02d %02d/%02d/%04d", hour(), minute(), second(), day(), month(), year());
  //Serial7.println(disp_msg_1);
  printDisplay();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial7.print(":");
  if (digits < 10)
    Serial7.print('0');
  Serial7.print(digits);
}

void initSD_card()
{
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }
  // Open or create file - truncate existing file.
  char file_name[50] = { "" };
  sprintf(file_name, "log_%02d%02d%02d_%02d%02d%04d", hour(), minute(), second(), day(), month(), year());
  Serial7.print("FileName: ");
  Serial7.println(file_name);
  strncpy(disp_msg_3, file_name, 19);
  disp_msg_4[0] = '\0';
  if (!file.open((const char*)file_name, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial7.println("open failed\n");
    return;
  }
  // File must be pre-allocated to avoid huge delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) {
    Serial7.println("preAllocate failed\n");
    file.close();
    return;
  }
  // initialize the RingBuf.
  rb.begin(&file);
}

void canSniff(const CAN_message_t& msg) {
  double timeStamp = double((double)elapselogtime / 1000000);  //micros()
  if(!log_enable)
  {
    Serial7.print(" ");
    Serial7.print(timeStamp, 6);
    Serial7.print("  ");
    Serial7.print("1");
    Serial7.print("      ");
    Serial7.print(msg.id, HEX);
    Serial7.print("  Rx d ");
    Serial7.print(msg.len);
    Serial7.print(" ");
    for (uint8_t i = 0; i < msg.len; i++) {
      Serial7.print(msg.buf[i], HEX); Serial7.print(" ");
      Serial7.print(" ");
    }
    Serial7.println();
  }
  

  size_t n = rb.bytesUsed();
  logFileSize = (uint32_t)n + (uint32_t)file.curPosition();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
    Serial7.println("File full - quiting.");
    return;
  }
  if (n > maxUsed) {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy()) {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512)) {
      Serial7.println("writeOut failed");
      return;
    }
  }

  if (!log_enable)
    return;

  rb.print(" ");
  rb.print(timeStamp, 6);
  rb.print("  ");
  rb.print("1");
  rb.print("      ");
  rb.print(msg.id, HEX);
  rb.print("  Rx d ");
  rb.print(msg.len);
  rb.print(" ");
  for (uint8_t i = 0; i < msg.len; i++) {
    rb.print(msg.buf[i], HEX);
    rb.print(" ");
  }
  rb.println();
  if (rb.getWriteError()) {
    // Error caused by too few free bytes in RingBuf.
    Serial7.println("WriteError");
    return;
  }
 
}

void sensorDataWrite(uint8_t sensortype) {

  if (!log_enable)
    return;
    
  double timeStamp = double((double)elapselogtime / 1000000);  //micros()
  size_t n = rb.bytesUsed();
  logFileSize = (uint32_t)n + (uint32_t)file.curPosition();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
    Serial7.println("File full - quiting.");
    return;
  }
  if (n > maxUsed) {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy()) {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512)) {
      Serial7.println("writeOut failed");
      return;
    }
  }

  CAN_message_t sensorData;
  sensorData.len = 8;
  
  if(sensortype==0)
  {
    sensorData.id = 0x701;
    memcpy(sensorData.buf,(uint8_t*)(&mpu_6050_sensor),sizeof(mpu_6050_sensor));
  }
  else if(sensortype==1)
  {
    sensorData.id = 0x702;
    memcpy(sensorData.buf,(uint8_t*)(&GPS_Location),sizeof(GPS_Location));
  }
 
  rb.print(" ");
  rb.print(timeStamp, 6);
  rb.print("  ");
  rb.print("1");
  rb.print("      ");
  rb.print(sensorData.id, HEX);
  rb.print("  Rx d ");
  rb.print(sensorData.len);
  rb.print(" ");
  for (uint8_t i = 0; i < sensorData.len; i++) {
    rb.print(sensorData.buf[i], HEX);
    rb.print(" ");
  }
  rb.println();
  if (rb.getWriteError()) {
    // Error caused by too few free bytes in RingBuf.
    Serial7.println("WriteError");
    return;
  }

  /*Serial7.print(" ");
  Serial7.print(timeStamp, 6);
  Serial7.print("  ");
  Serial7.print("1");
  Serial7.print("      ");
  Serial7.print(sensorData.id, HEX);
  Serial7.print("  Rx d ");
  Serial7.print(sensorData.len);
  Serial7.print(" ");
  for (uint8_t i = 0; i < sensorData.len; i++) {
    Serial7.print(sensorData.buf[i], HEX); Serial7.print(" ");
    Serial7.print(" ");
  }
  Serial7.println();*/
  
}

void stopLogging()
{
  log_enable = false;
  Serial7.println("Stop Logging");
  strcpy(disp_msg_2, "Stop Logging");
  digitalWrite(LED_PIN, LOW);
  rb.sync();

  file.truncate();
  file.rewind();
  Serial7.print("fileSize: ");
  Serial7.println((uint32_t)file.fileSize());
  Serial7.print("maxBytesUsed: ");
  Serial7.println(maxUsed);
  sprintf(disp_msg_4, "FileSize: %ld", (uint32_t)file.fileSize());
  file.close();
}

void startLogging()
{
  log_enable = true;
  Serial7.println("Start Logging");
  strcpy(disp_msg_2, "Start Logging");
  digitalWrite(LED_PIN, HIGH);
  elapselogtime = 0;
  initSD_card();
}

void parseGpsNmea()
{
  if(!log_enable)
    Serial7.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    GPS_Location.gps_lat= (float)gps.location.lat();
    GPS_Location.gps_lng= (float)gps.location.lng();
    if(!log_enable)
    {
      sprintf(disp_msg_3, "lat: %.6f", GPS_Location.gps_lat);
      sprintf(disp_msg_4, "lng: %.6f", GPS_Location.gps_lng);
    
      Serial7.print(GPS_Location.gps_lat,6);
      Serial7.print(F(","));
      Serial7.print(GPS_Location.gps_lng, 6);
    }
    sensorDataWrite(1);
  }
  else
  {
    if(!log_enable)
      Serial7.print(F("INVALID"));
  }

  if(log_enable)
    return;
  
  Serial7.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial7.print(gps.date.month());
    Serial7.print(F("/"));
    Serial7.print(gps.date.day());
    Serial7.print(F("/"));
    Serial7.print(gps.date.year());
  }
  else
  {
    Serial7.print(F("INVALID"));
  }

  Serial7.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial7.print(F("0"));
    Serial7.print(gps.time.hour());
    Serial7.print(F(":"));
    if (gps.time.minute() < 10) Serial7.print(F("0"));
    Serial7.print(gps.time.minute());
    Serial7.print(F(":"));
    if (gps.time.second() < 10) Serial7.print(F("0"));
    Serial7.print(gps.time.second());
    //Serial7.print(F("."));
    //if (gps.time.centisecond() < 10) Serial7.print(F("0"));
    //Serial7.print(gps.time.centisecond());
    
    sprintf(disp_msg_2, "%02d:%02d:%02d %02d/%02d/%04d", gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
    if(!rtcSync)
    {
      setTime((int)gps.time.hour(),(int)gps.time.minute(),(int)gps.time.second(),(int)gps.date.day(), (int)gps.date.month(), (int)gps.date.year());
      time_t t= now()+19800;
      setTime(t);
      Serial7.print(" ...time:");
      Serial7.println(t);
      Teensy3Clock.set(t);
      rtcSync=true;
      
    }
  }
  else
  {
    Serial7.print(F("INVALID"));
  }

  Serial7.println();
  sprintf(disp_msg_5, "X: %d, Y: %d, Z: %d", mpu_6050_sensor.angleX_pitch,mpu_6050_sensor.angleY_roll,mpu_6050_sensor.angleX_pitch);
}

void storage_configure()
{
  Serial7.println("MTP_Start");
  #if !__has_include("usb_mtp.h")
     usb_mtp_configure();
  #endif
      
  #if USE_SD==1
    #if defined SD_SCK
      SPI.setMOSI(SD_MOSI);
      SPI.setMISO(SD_MISO);
      SPI.setSCK(SD_SCK);
    #endif

    for(int ii=0; ii<nsd; ii++)
    { 
      #if defined(BUILTIN_SDCARD)
        if(cs[ii] == BUILTIN_SDCARD)
        {
          if(!sdx[ii].sdfs.begin(SdioConfig(FIFO_SDIO))) 
          { Serial7.printf("SDIO Storage %d %d %s failed or missing",ii,cs[ii],sd_str[ii]);  Serial7.println();
          }
          else
          {
            storage.addFilesystem(sdx[ii], sd_str[ii]);
            uint64_t totalSize = sdx[ii].totalSize();
            uint64_t usedSize  = sdx[ii].usedSize();
            Serial7.printf("SDIO Storage %d %d %s ",ii,cs[ii],sd_str[ii]); 
            Serial7.print(totalSize); Serial7.print(" "); Serial7.println(usedSize);
          }
        }
        else if(cs[ii]<BUILTIN_SDCARD)
      #endif
      {
        pinMode(cs[ii],OUTPUT); digitalWriteFast(cs[ii],HIGH);
        if(!sdx[ii].sdfs.begin(SdSpiConfig(cs[ii], SHARED_SPI, SPI_SPEED))) 
        { Serial7.printf("SD Storage %d %d %s failed or missing",ii,cs[ii],sd_str[ii]);  Serial7.println();
        }
        else
        {
          storage.addFilesystem(sdx[ii], sd_str[ii]);
          uint64_t totalSize = sdx[ii].totalSize();
          uint64_t usedSize  = sdx[ii].usedSize();
          Serial7.printf("SD Storage %d %d %s ",ii,cs[ii],sd_str[ii]); 
          Serial7.print(totalSize); Serial7.print(" "); Serial7.println(usedSize);
        }
      }
    }
    #endif
}

void checkButton(){

  logButton.update();
  if ( logButton.read() != 1)
  {
    btn_duration = logButton.duration();
    if(btn_duration>BTN_TIMEOUT && triggerCommand==false)
    {
      triggerCommand = true;
      Serial7.println("Button pressed for more than 3 sec");
      if (log_enable)
      {
        stopLogging();
      }
      Serial7.println("Start MTP");
      displayMsgOled_generic("MTP Service Started");
      storage_configure();
      mtp_enable=true;
      mtpd.loop();
      return;
    }
  }
  else if(logButton.read() == 1 && btn_duration>0)
  {
    if(btn_duration>15 && btn_duration<BTN_TIMEOUT)
    {
      Serial7.println("Button pressed");
      Serial7.println("Start/stop Logging");
      if (!log_enable)
      {
        startLogging();
      }
      else
      {
        stopLogging();
      }
    }
    btn_duration = 0;
  }
}


void loop() {

  if(mtp_enable)
  {
     mtpd.loop();
     return;
  }
  
  checkButton();
  
  if (!log_enable)
  {
    if (Serial7.available()) {
      time_t t = processSyncMessage();
      if (t != 0) {
        Teensy3Clock.set(t); // set the RTC
        setTime(t);
        Serial7.println("Time Set Success!");
        
      }
    }
  }
  
  Can0.events();
  
  mpu.Execute();
  if((millis()-mpu6050_task)>100){ // print data every 100ms
    //Serial7.print("AngX = ");
    mpu_6050_sensor.angleX_pitch=(int16_t)(mpu.GetAngX());
    //Serial7.print(mpu_6050_sensor.angleX_pitch);
    //Serial7.print("  AngY = ");
    mpu_6050_sensor.angleY_roll=(int16_t)(mpu.GetAngY());
    //Serial7.print(mpu_6050_sensor.angleY_roll);
    //Serial7.print("  AngZ = ");
    mpu_6050_sensor.angleZ_Yaw=(int16_t)(mpu.GetAngZ());
    //Serial7.println(mpu_6050_sensor.angleZ_Yaw);
    mpu6050_task = millis();
    sensorDataWrite(0);  
  }
  
  while (millis() > time_now + period) {
    digitalClockDisplay();
    if(log_enable)
    {
      int fsize = (int)((uint32_t)logFileSize/1024);
      sprintf(disp_msg_4, "FileSize: %d Kb", fsize);
      if (logFileSize >= LOG_FILE_SIZE_SD)
      {
        stopLogging();
        startLogging();
        newFileCount++;
        sprintf(disp_msg_5, "NewFileCount: %ld", newFileCount);
      }
    }
    time_now = millis();
  }

  while (Serial2.available() > 0)
  {
      if (gps.encode(Serial2.read()))
        parseGpsNmea();
  }
}
