#include <FlexCAN_T4.h>
#include <TimeLib.h>
#include <Bounce.h>
#include <Arduino.h>
//#include <U8x8lib.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "SdFat.h"
#include "RingBuf.h"

//U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

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
Bounce logButton = Bounce(LOG_BTN, 15); // 15 = 15 ms debounce time
bool log_enable = false;
void initSD_card();
void stopLogging();
void startLogging();
void digitalClockDisplay();
void printDigits(int digits);
//void printDisplay(uint8_t x_pos, uint8_t y_pos, const char* msg);
void printDisplay();

char disp_msg_1[20] = { "Please Wait" };
char disp_msg_2[20] = { " " };
char disp_msg_3[20] = { " " };
char disp_msg_4[20] = { " " };
char disp_msg_5[20] = { " " };

time_t getTeensy3Time();


void setup(void) {

	pinMode(LOG_BTN, INPUT_PULLUP);
	pinMode(LED_PIN, OUTPUT);
	setSyncProvider(getTeensy3Time);

	Serial.begin(115200); delay(400);
	//pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */

	//u8x8.begin();
	//u8x8.setPowerSave(0);

	u8g2.begin();
	u8g2.clearBuffer();          // clear the internal memory

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

void digitalClockDisplay()
{
	sprintf(disp_msg_1, "%02d:%02d:%02d %02d/%02d/%04d", hour(), minute(), second(), day(), month(), year());
	Serial.println(disp_msg_1);
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
	//String fileName = "CANLOG_" + String(hour()) + String(minute()) + String(second()) + String(day()) + String(month()) + String(year());
	sprintf(file_name, "log_%02d%02d%02d_%02d%02d%04d", hour(), minute(), second(), day(), month(), year());
	Serial.print("FileName: ");
	Serial.println(file_name);
	strncpy(disp_msg_3, file_name, 19);
	disp_msg_4[0] = '\0';
	//int str_len = fileName.length() + 1;
	//fileName.toCharArray(file_name, str_len);
	if (!file.open((const char*)file_name, O_RDWR | O_CREAT | O_TRUNC)) {
		Serial.println("open failed\n");
		return;
	}
	// File must be pre-allocated to avoid huge
	// delays searching for free clusters.
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
	Can0.events();
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
			sprintf(disp_msg_4, "FileSize: %ld", logFileSize);
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
