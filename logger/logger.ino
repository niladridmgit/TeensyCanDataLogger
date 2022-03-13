#include <TimeLib.h>
#include <Bounce.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "SdFat.h"
#include "RingBuf.h"
#include <TinyMPU6050.h>
#include <FlexCAN_T4.h>  // if defined before SdFat.h and RingBuf.h Teensy will keep restart when initSD_card function called (Niladri) //

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

// Function prtotype declaration //
void initSD_card();
void stopLogging();
void startLogging();
void digitalClockDisplay();
void printDigits(int digits);
unsigned long processSyncMessage();
void printDisplay();
void MPUDisplay();

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

MPU6050 mpu (Wire1);

time_t getTeensy3Time();

unsigned long mpu6050_task = 0;

void setup(void) {

	pinMode(LOG_BTN, INPUT_PULLUP);
	pinMode(LED_PIN, OUTPUT);
	setSyncProvider(getTeensy3Time);

	Serial.begin(115200); delay(400);

	u8g2.begin();
	u8g2.clearBuffer();          // clear the internal memory

 // Initialization
  mpu.Initialize();

  // Calibration
  Serial.begin(115200);
  Serial.println("=====================================");
  Serial.println("Starting MPU6050 calibration.........");
  sprintf(disp_msg_1, "MPU6050 %s", "calibration");
  MPUDisplay();
  mpu.Calibrate();
  Serial.println("Calibration complete!");
  Serial.println("Offsets:");
  Serial.print("GyroX Offset = ");
  Serial.println(mpu.GetGyroXOffset());
  Serial.print("GyroY Offset = ");
  Serial.println(mpu.GetGyroYOffset());
  Serial.print("GyroZ Offset = ");
  Serial.println(mpu.GetGyroZOffset());

	if (timeStatus() != timeSet) {
		Serial.println("Unable to sync with the RTC");
	}
	else {
		Serial.println("RTC has set the system time");
	}

	digitalClockDisplay();
	delay(1000);

	Can0.begin();
	Can0.setBaudRate(500000);
	Can0.setMaxMB(16);
	Can0.enableFIFO();
	Can0.enableFIFOInterrupt();
	Can0.onReceive(canSniff);
	Can0.mailboxStatus();

}

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1646932827; //2022

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
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

void MPUDisplay()
{
  Serial.println(disp_msg_1);
  printDisplay();
}

void digitalClockDisplay()
{
	sprintf(disp_msg_1, "%02d:%02d:%02d %02d/%02d/%04d", hour(), minute(), second(), day(), month(), year());
	//Serial.println(disp_msg_1);
	printDisplay();
}

time_t getTeensy3Time()
{
	return Teensy3Clock.get();
}

void printDigits(int digits) {
	// utility function for digital clock display: prints preceding colon and leading 0
	Serial.print(":");
	if (digits < 10)
		Serial.print('0');
	Serial.print(digits);
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
	Serial.print("FileName: ");
	Serial.println(file_name);
	strncpy(disp_msg_3, file_name, 19);
	disp_msg_4[0] = '\0';
	if (!file.open((const char*)file_name, O_RDWR | O_CREAT | O_TRUNC)) {
		Serial.println("open failed\n");
		return;
	}
	// File must be pre-allocated to avoid huge delays searching for free clusters.
	if (!file.preAllocate(LOG_FILE_SIZE)) {
		Serial.println("preAllocate failed\n");
		file.close();
		return;
	}
	// initialize the RingBuf.
	rb.begin(&file);
}

void canSniff(const CAN_message_t& msg) {
	//Serial.print("MB "); Serial.print(msg.mb);
	//Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
	//Serial.print(" EXT: "); Serial.print(msg.flags.extended);
	//Serial.print(" TS: "); Serial.print(msg.timestamp);
	Serial.print(" ");
	double timeStamp = double((double)elapselogtime / 1000000);  //micros()
	Serial.print(timeStamp, 6);
	Serial.print("  ");
	Serial.print("1");
	Serial.print("      ");
	Serial.print(msg.id, HEX);
	Serial.print("  Rx d ");
	Serial.print(msg.len);
	Serial.print(" ");
	for (uint8_t i = 0; i < msg.len; i++) {
		Serial.print(msg.buf[i], HEX); Serial.print(" ");
		Serial.print(" ");
	}
	Serial.println();

	size_t n = rb.bytesUsed();
	logFileSize = (uint32_t)n + (uint32_t)file.curPosition();
	if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
		Serial.println("File full - quiting.");
		return;
	}
	if (n > maxUsed) {
		maxUsed = n;
	}
	if (n >= 512 && !file.isBusy()) {
		// Not busy only allows one sector before possible busy wait.
		// Write one sector from RingBuf to file.
		if (512 != rb.writeOut(512)) {
			Serial.println("writeOut failed");
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
		Serial.println("WriteError");
		return;
	}
}

void sensorDataWrite() {

  if (!log_enable)
    return;
    
  double timeStamp = double((double)elapselogtime / 1000000);  //micros()
  size_t n = rb.bytesUsed();
  logFileSize = (uint32_t)n + (uint32_t)file.curPosition();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
    Serial.println("File full - quiting.");
    return;
  }
  if (n > maxUsed) {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy()) {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512)) {
      Serial.println("writeOut failed");
      return;
    }
  }

  CAN_message_t sensorData;
  sensorData.id = 0x701;
  sensorData.len = 8;
  memcpy(sensorData.buf,(uint8_t*)(&mpu_6050_sensor),sizeof(mpu_6050_sensor));
  Serial.print(" ");
  Serial.print(timeStamp, 6);
  Serial.print("  ");
  Serial.print("1");
  Serial.print("      ");
  Serial.print(sensorData.id, HEX);
  Serial.print("  Rx d ");
  Serial.print(sensorData.len);
  Serial.print(" ");
  for (uint8_t i = 0; i < sensorData.len; i++) {
    Serial.print(sensorData.buf[i], HEX); Serial.print(" ");
    Serial.print(" ");
  }
  Serial.println();
  
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
    Serial.println("WriteError");
    return;
  }
}

void stopLogging()
{
	log_enable = false;
	Serial.println("Stop Logging");
	strcpy(disp_msg_2, "Stop Logging");
	digitalWrite(LED_PIN, LOW);
	rb.sync();

	file.truncate();
	file.rewind();
	Serial.print("fileSize: ");
	Serial.println((uint32_t)file.fileSize());
	Serial.print("maxBytesUsed: ");
	Serial.println(maxUsed);
	sprintf(disp_msg_4, "FileSize: %ld", (uint32_t)file.fileSize());
	file.close();
}

void startLogging()
{
	log_enable = true;
	Serial.println("Start Logging");
	strcpy(disp_msg_2, "Start Logging");
	digitalWrite(LED_PIN, HIGH);
	elapselogtime = 0;
	initSD_card();
}


void loop() {

  if (!log_enable)
  {
    if (Serial.available()) {
      time_t t = processSyncMessage();
      if (t != 0) {
        Teensy3Clock.set(t); // set the RTC
        setTime(t);
        Serial.println("Time Set Success!");
        
      }
    }
  }
  
	Can0.events();
  
  mpu.Execute();
  if((millis()-mpu6050_task)>100){ // print data every 100ms
    //Serial.print("AngX = ");
    mpu_6050_sensor.angleX_pitch=(int16_t)(mpu.GetAngX());
    //Serial.print(mpu_6050_sensor.angleX_pitch);
    //Serial.print("  AngY = ");
    mpu_6050_sensor.angleY_roll=(int16_t)(mpu.GetAngY());
    //Serial.print(mpu_6050_sensor.angleY_roll);
    //Serial.print("  AngZ = ");
    mpu_6050_sensor.angleZ_Yaw=(int16_t)(mpu.GetAngZ());
    //Serial.println(mpu_6050_sensor.angleZ_Yaw);
    mpu6050_task = millis();
    sensorDataWrite();  
  }
  
	logButton.update();
	if (logButton.fallingEdge()) {
		if (!log_enable)
		{
			startLogging();
		}
		else
		{
			stopLogging();
		}

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
				sprintf(disp_msg_5, "NewFileCount: %d", newFileCount);
			}
		}
		time_now = millis();
	}
}
