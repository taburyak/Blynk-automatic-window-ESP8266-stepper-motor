/*
 Name:		StepperBlynk.ino
 Created:	6/11/2018 6:46:01 PM
 Author:	Andriy
*/

/* CODE BEGIN Includes */
#include <Stepper.h>
#include <TimeLib.h>
#include <Time.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266httpUpdate.h>
#include <DHT.h>
#include <BlynkSimpleEsp8266.h>
#include <WidgetRTC.h>
#include <WidgetTerminal.h>
#include <Ticker.h>
#include <WiFiManager.h>
/* CODE END Includes */

/* CODE BEGIN Defines */
//27.09.2018 - виправлено алгоритм обробки слайдеру без зависань.
#define FIRMWARE_VERSION "0.0.5"
#define NAME_DEVICE "Stepper-Motor-Wemos"
#define TYPE_DEVICE "ESP8266, NodeMCU, Wemos D1 mini"

#define BUTTON_SYSTEM			0
#define LED_BLUE				2
#define INITIAL_POSITION_SWITCH	16
#define ULN2003_IN1				5
#define ULN2003_IN2				4
#define ULN2003_IN3				14
#define ULN2003_IN4				12

#define DHTTYPE DHT11		// DHT11, DHT 22  (AM2302), AM2321
#define DHTPIN 13			// DHT PIN

// Сенсор DHT
#define TMP_DHT V5
#define HUM_DHT V6

#define MOTOR_MOVE_SLIDER			V20	// Віджет слайдера
#define MOTOR_SPEED_STEP_CONTROL	V21 // Віджет STEP CONTROL
#define MOTOR_MAXIMUM_STEPS			V22 // Максимальна кількість кроків для слайдеру і кнопки
#define MOTOR_DIRECTION_MENU		V60 // Меню направлення руху мотору

#define TERMINAL			V41
#define TIME_INPUT_0		V50
#define TIME_INPUT_1		V51
#define WIFI_SIGNAL			V80

#define INTERVAL_PRESSED_RESET_ESP		3000L
#define INTERVAL_PRESSED_RESET_SETTINGS 5000L
#define INTERVAL_PRESSED_SHORT			50
#define INTERVAL_SEND_DATA				30033L
#define INTERVAL_RECONNECT				60400L
#define INTERVAL_TIME_CHECKER			9990L
#define INTERVAL_REFRESH_DATA			4065L
#define WRITE_SETTING_TIMEOUT			3000L
#define WIFI_MANAGER_TIMEOUT			180
#define FIND_MOTOR_POSITION_TIMEOUT		10000L

#define EEPROM_SETTINGS_SIZE			512
#define EEPROM_START_SETTING_WM			0
#define EEPROM_SALT_WM					42561

#define NUMBER_SHEDULERS				2
#define STEPS_PER_REVOLUTION			64

#define LED_BLUE_TOGGLE()				digitalWrite(LED_BLUE, !digitalRead(LED_BLUE))
#define LED_BLUE_ON()					digitalWrite(LED_BLUE, LOW)
#define LED_BLUE_OFF()					digitalWrite(LED_BLUE, HIGH)
/* CODE END Defines */

/* CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//volatile bool StartPositionState = false;

bool shouldSaveConfigWM = false;				//flag for saving data for WiFi Manager
bool triggerBlynkConnect = false;				// прапорець чи є конект з сервером blynk
bool isFirstConnect = true;						// Keep this flag not to re-sync on every reconnection
volatile bool btnSystemState = false;
volatile unsigned int startPressBtn = 0;		// значення мілісекунд коли натиснули кнопку (має бути в глобальних зміних)
int currentPositionMotor = 0;					// поточне положення мотору
bool windowState = false;						// статус вікна, true/false закрито/відкрито

bool firstSettingChange = true;					// прапорець для створення разового таймеру один раз
unsigned int numTimerWriteSetting;				// номер слоту разового таймеру

//----------------------------------
//Level Wi-Fi signal
//----------------------------------
int wifisignal = NAN;
//----------------------------------

//----------------------------------
//DHT
//----------------------------------
typedef struct {
	float h = NAN;
	float t = NAN;
	bool triggerInit = false;
}DHTTypeDef;

DHTTypeDef dhtStruct;
//----------------------------------

//----------------------------------
//Stepper motor
//----------------------------------
// initialize the stepper library on pins:
Stepper myStepper(STEPS_PER_REVOLUTION, ULN2003_IN1, ULN2003_IN3, ULN2003_IN2, ULN2003_IN4);
//----------------------------------

//-----------------------------------------------------------------------------------------
//Змінні для потреб TimeInput планувальника завдань
//-----------------------------------------------------------------------------------------
//структура для налаштувань планувальника
typedef struct {
	long startSeconds = 0;
	long stopSeconds = 0;
	uint8_t activeDaysOfWeek = 0;			// 0 bit - sheduler enable/disable, 1-7 bit - dayOfWeak
	bool activeToday = false;				// Чи активний поточного дня 
	bool schedulerStatusChanged = false;	// Чи змінився статус планувальника
	bool triggerStart = false;				// Прапорець чи було встановлений старт планувальника
	bool triggerStop = false;				// Прапорець чи було встановлений стоп планувальника
}ShedulerStructTypeDef;

ShedulerStructTypeDef sheduler[NUMBER_SHEDULERS] = {};
//-----------------------------------------------------------------------------------------

//структура для початкових налаштувань. Зараз займає 116 bytes
typedef struct {
	char	host[33] = NAME_DEVICE;					// 33 + '\0' = 34 bytes
	char	blynkToken[33] = "";					// 33 + '\0' = 34 bytes
	char	blynkServer[33] = "blynk-cloud.com";	// 33 + '\0' = 34 bytes
	char	blynkPort[6] = "8442";					// 04 + '\0' = 05 bytes
	int		speedMotor = 400;						// 04		 = 04 bytes // швидкість мотору
	int		maxStepsMotor = 1024;					// 04
	int		directionMotor = 1;						// 04
	int   salt = EEPROM_SALT_WM;					// 04		 = 04 bytes
} WMSettings;										// 115 + 1	 = 116 bytes (115 це рахунок ведеться з 0)
//-----------------------------------------------------------------------------------------
	
// Оголошення початкових налаштувань для WiFi Manager
WMSettings wmSettings;
// Оголошення таймеру
BlynkTimer timer;
// Оголошення тікеру
Ticker ticker;

// Оголошення сенсору DHT
DHT dht(DHTPIN, DHTTYPE);

// Оголошення для OTA WebUpdater
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WidgetTerminal terminal(TERMINAL); // Attach virtual serial terminal to Virtual Pin V41								   
WidgetRTC rtc;
/* CODE END PV */

/* CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void configModeCallback(WiFiManager *myWiFiManager);
static void saveConfigCallback(void);

static void tick(void);
static void untick(void);
static void readSystemKey(void);
static void TimeInputSheduler(BlynkParam param, uint8_t numSheduler);
static void InitStartPosition(void);
static void ResetMotorPin(void);

static void timerRefreshData(void);
static void timerSendServer(void);
static void timerReconnect(void);
static void timeChecker(void);
static void timerWriteSetting(void);

static void getWiFiLevelSignal(void);
static void getDHTData(void);

static String twoDigits(int digits);
/* CODE END PFP */

// the setup function runs once when you press reset or power the board
void setup()
{
	Serial.begin(115200);

	// Налаштування режимів пінів   
	pinMode(BUTTON_SYSTEM, INPUT);
	pinMode(INITIAL_POSITION_SWITCH, INPUT);
	pinMode(LED_BLUE, OUTPUT);
	
	InitStartPosition();

	dht.begin();

	// Зчитуємо данні налаштувань WM з EEPROM до RAM 
	EEPROM.begin(EEPROM_SETTINGS_SIZE);
	EEPROM.get(EEPROM_START_SETTING_WM, wmSettings);
	EEPROM.end();

	if (wmSettings.salt != EEPROM_SALT_WM)
	{
		Serial.println("Invalid wmSettings in EEPROM, trying with defaults");
		WMSettings defaults;
		wmSettings = defaults;
	}

	// Друк старих значень до терміналу
	Serial.println(wmSettings.host);
	Serial.println(wmSettings.blynkToken);
	Serial.println(wmSettings.blynkServer);
	Serial.println(wmSettings.blynkPort);

	ticker.attach(0.5, tick);   // start ticker with 0.5 because we start in AP mode and try to connect	

	WiFiManager wifiManager;
	
	wifiManager.setConfigPortalTimeout(WIFI_MANAGER_TIMEOUT);
	
	WiFiManagerParameter custom_device_name_text("<br/>Enter name of the device<br/>or leave it as it is<br/>");
	wifiManager.addParameter(&custom_device_name_text);

	WiFiManagerParameter custom_device_name("device-name", "device name", wmSettings.host, 33);
	wifiManager.addParameter(&custom_device_name);

	WiFiManagerParameter custom_blynk_text("<br/>Blynk config.<br/>");
	wifiManager.addParameter(&custom_blynk_text);

	WiFiManagerParameter custom_blynk_token("blynk-token", "blynk token", wmSettings.blynkToken, 33);
	wifiManager.addParameter(&custom_blynk_token);

	WiFiManagerParameter custom_blynk_server("blynk-server", "blynk server", wmSettings.blynkServer, 33);
	wifiManager.addParameter(&custom_blynk_server);

	WiFiManagerParameter custom_blynk_port("blynk-port", "port", wmSettings.blynkPort, 6);
	wifiManager.addParameter(&custom_blynk_port);

	//set config save notify callback
	wifiManager.setSaveConfigCallback(saveConfigCallback);

	//set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
	wifiManager.setAPCallback(configModeCallback);

	if (wifiManager.autoConnect(wmSettings.host))
	{
		//if you get here you have connected to the WiFi
		Serial.println("Connected WiFi!");

		untick(); // скасовуємо блимання світлодіодом   
	}
	else
	{
		Serial.println("failed to connect and hit timeout");
	}

	// Копіювання введених значень до структури
	strcpy(wmSettings.host, custom_device_name.getValue());
	strcpy(wmSettings.blynkToken, custom_blynk_token.getValue());
	strcpy(wmSettings.blynkServer, custom_blynk_server.getValue());
	strcpy(wmSettings.blynkPort, custom_blynk_port.getValue());

	// Друк нових значень до терміналу
	Serial.println(wmSettings.host);
	Serial.println(wmSettings.blynkToken);
	Serial.println(wmSettings.blynkServer);
	Serial.println(wmSettings.blynkPort);

	if (shouldSaveConfigWM)
	{
		LED_BLUE_ON();
		//Записуємо введені данні до EEPROM
		EEPROM.begin(EEPROM_SETTINGS_SIZE);
		EEPROM.put(EEPROM_START_SETTING_WM, wmSettings);
		EEPROM.end();
		//---------------------------------
		LED_BLUE_OFF();
	}

	// Запуск OTA WebUpdater
	MDNS.begin(wmSettings.host);
	httpUpdater.setup(&httpServer);
	httpServer.begin();
	MDNS.addService("http", "tcp", 80);

	Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", wmSettings.host);

	// Конфігуруємо підключення до blynk server
	Blynk.config(wmSettings.blynkToken, wmSettings.blynkServer, atoi(wmSettings.blynkPort));

	if (Blynk.connect())
	{
		triggerBlynkConnect = true;	
	}
	else
	{
		triggerBlynkConnect = false;
	}

	timer.setInterval(INTERVAL_REFRESH_DATA, timerRefreshData);
	timer.setInterval(INTERVAL_SEND_DATA, timerSendServer);
	timer.setInterval(INTERVAL_RECONNECT, timerReconnect);
	// check every 10s if any weekday scheduled events
	timer.setInterval(INTERVAL_TIME_CHECKER, timeChecker);	

	rtc.begin();
	// Період коли синхронізується годинник з сервером (для WidgetRTC)
	setSyncInterval(30 * 60); // Sync interval in seconds (30 minutes)
}

// the loop function runs over and over again until power down or reset
void loop()
{
	if (Blynk.connected())
	{
		Blynk.run(); // Initiates Blynk Server  
	}
	else
	{
		if (triggerBlynkConnect)
		{
			triggerBlynkConnect = false;
			ticker.attach(2, tick);
		}
	}

	timer.run(); // Initiates BlynkTimer

	httpServer.handleClient(); // Initiates OTA WebUpdater  

	readSystemKey();	
}

/* BLYNK CODE BEGIN */
BLYNK_CONNECTED()
{
	untick();
	triggerBlynkConnect = true;

	String str1 = "Blynk Connected!";

	Serial.println(str1);
	Serial.println("local ip");
	Serial.println(WiFi.localIP());
	
	terminal.println();
	terminal.println(str1);
	terminal.println("local ip");
	terminal.println(WiFi.localIP().toString());
	terminal.flush();
	
	char str[30];
	sprintf(str, "%s Online!", wmSettings.host);

	Blynk.notify(str);

	if (isFirstConnect)
	{
		Blynk.syncAll();

		isFirstConnect = false;
	}
}

BLYNK_WRITE(MOTOR_MOVE_SLIDER)
{	
	int gotoPosition = param.asInt() - currentPositionMotor;
	myStepper.step(wmSettings.directionMotor * gotoPosition);

	ResetMotorPin();

	currentPositionMotor = currentPositionMotor + gotoPosition;	

	String str2;
	String str0 = "Move slider is App - ";
	String str1 = String(twoDigits(hour())) + ":" + twoDigits(minute()) + ":" + twoDigits(second()) + " - ";
	String str3 = String(twoDigits(day())) + "." + twoDigits(month()) + "." + year() + " ";

	if (currentPositionMotor == 0)
	{
		windowState = true;
		str2 = "window is closed!";
	}
	else
	{
		windowState = false;
		str2 = "window is open!";
	}

	Serial.print(str3);
	Serial.print(str1);
	Serial.print(str0);
	Serial.println(str2);

	terminal.print(str3);
	terminal.print(str1);
	terminal.print(str0);
	terminal.println(str2);
	terminal.flush();
}

BLYNK_WRITE(MOTOR_SPEED_STEP_CONTROL)
{
	if (wmSettings.speedMotor != param.asInt())
	{										
		wmSettings.speedMotor = param.asInt();
		myStepper.setSpeed(wmSettings.speedMotor);
		
		if (firstSettingChange)
		{
			firstSettingChange = false;
			numTimerWriteSetting = timer.setTimeout(WRITE_SETTING_TIMEOUT, timerWriteSetting);
		}
		else
		{
			timer.restartTimer(numTimerWriteSetting);
		}		
	}
}

BLYNK_WRITE(MOTOR_MAXIMUM_STEPS)
{
	Blynk.setProperty(MOTOR_MOVE_SLIDER, "max", param.asInt());
	
	if (wmSettings.maxStepsMotor != param.asInt())
	{
		wmSettings.maxStepsMotor = param.asInt();
		
		if (firstSettingChange)
		{
			firstSettingChange = false;
			numTimerWriteSetting = timer.setTimeout(WRITE_SETTING_TIMEOUT, timerWriteSetting);
		}
		else
		{
			timer.restartTimer(numTimerWriteSetting);
		}		
	}
}

BLYNK_WRITE(MOTOR_DIRECTION_MENU)
{
	int direction;

	switch (param.asInt())
	{
	case 1:
	{
		direction = 1;
	}
	break;
	case 2:
	{
		direction = -1;
	}
	break;
	default:
		break;
	}
	
	if (wmSettings.directionMotor != direction)
	{
		wmSettings.directionMotor = direction;

		LED_BLUE_ON();
		EEPROM.begin(EEPROM_SETTINGS_SIZE);
		EEPROM.put(EEPROM_START_SETTING_WM, wmSettings);
		EEPROM.end();
		LED_BLUE_OFF();
		Serial.println("Save setting direction");
		terminal.println("Save setting direction");
		terminal.flush();
	}
}

BLYNK_WRITE(TERMINAL)
{
	//TODO тут можна вставити код для обробки запитів з терміналу
	
	if (String("version") == param.asString())
	{
		terminal.println(String("Firmvare version: ") + FIRMWARE_VERSION + " for " + TYPE_DEVICE);
	}
	else if (String("name") == param.asString())
	{
		terminal.println(String("Device name is: ") + wmSettings.host);
	}
	else if (String("ip") == param.asString())
	{
		terminal.print("Device ip address: ");
		terminal.println(WiFi.localIP().toString());
	}
	else if (String("mac") == param.asString())
	{
		terminal.print("Device mac address: ");
		terminal.println(WiFi.macAddress());
	}
	else if (String("reboot") == param.asString())
	{
		terminal.print("The device will restart now!");
		delay(2000);
		ESP.reset();
		delay(2000);
	}
	else if (String("reset") == param.asString())
	{
		WMSettings defaultsWM;
		wmSettings = defaultsWM;		

		LED_BLUE_ON();
		// Записуємо данні за замовчуванням до EEPROM
		EEPROM.begin(EEPROM_SETTINGS_SIZE);
		EEPROM.put(EEPROM_START_SETTING_WM, wmSettings);
		EEPROM.end();		
		//------------------------------------------
		LED_BLUE_OFF();

		Serial.println("Setting the default reset");

		terminal.println("Setting the default reset");
		terminal.flush();

		delay(1000);
		WiFi.disconnect();
		delay(1000);
		ESP.restart();
	}
	else if (String("pins") == param.asStr())
	{
		terminal.println("Sensor DHT:");
		terminal.println("TMP_DHT V5");
		terminal.println("HUM_DHT V6");
		terminal.println("Buttons:");
		terminal.println("BUTTON_OPEN V20");
		terminal.println("BUTTON_CLOSE V21");
		terminal.println("Widget:");
		terminal.println("TERMINAL V41");
		terminal.println("TIME_INPUT_0 V50");
		terminal.println("TIME_INPUT_1 V51");		
		terminal.println("WiFi Level signal V80");
	}
	else
	{
		// Send it back
		/*terminal.print("Unknown command");
		terminal.println();*/
	}

	// Ensure everything is sent
	terminal.flush();
}

BLYNK_WRITE(TIME_INPUT_0)
{
	TimeInputSheduler(param, 0);
}

BLYNK_WRITE(TIME_INPUT_1)
{
	TimeInputSheduler(param, 1);
}

BLYNK_READ(WIFI_SIGNAL)
{
	Blynk.virtualWrite(WIFI_SIGNAL, wifisignal);
}
/* BLYNK CODE END */

/* CODE BEGIN 4 */
void configModeCallback(WiFiManager * myWiFiManager)
{
	Serial.println("Entered config mode");
	Serial.println(WiFi.softAPIP());
	//if you used auto generated SSID, print it
	Serial.println(myWiFiManager->getConfigPortalSSID());

	//entered config mode, make led toggle faster
	ticker.attach(0.2, tick);
}

void saveConfigCallback(void)
{
	Serial.println("Should save config");

	shouldSaveConfigWM = true;
}

static void tick(void)
{
	//toggle state  
	LED_BLUE_TOGGLE();     // set pin to the opposite state
}

static void untick()
{
	ticker.detach();

	LED_BLUE_OFF(); //keep LED off		
}

void readSystemKey(void)
{
	if (!digitalRead(BUTTON_SYSTEM) && !btnSystemState)
	{
		btnSystemState = true;
		startPressBtn = millis();
	}
	else if (digitalRead(BUTTON_SYSTEM) && btnSystemState)
	{
		btnSystemState = false;
		int pressTime = millis() - startPressBtn;

		if (pressTime > INTERVAL_PRESSED_RESET_ESP && pressTime < INTERVAL_PRESSED_RESET_SETTINGS)
		{
			if (Blynk.connected())
			{				
				Blynk.notify(String(wmSettings.host) + " reset!");

				Blynk.disconnect();
			}

			ticker.attach(0.1, tick);			

			delay(2000);
			ESP.restart();
			delay(2000);
		}
		else if (pressTime > INTERVAL_PRESSED_RESET_SETTINGS)
		{
			if (Blynk.connected())
			{
				Blynk.notify(String(wmSettings.host) + " setting reset! Connected WiFi AP this device!");		
			}			

			WMSettings defaults;
			wmSettings = defaults;

			LED_BLUE_ON();
			// Записуємо данні за замовчуванням до EEPROM
			EEPROM.begin(EEPROM_SETTINGS_SIZE);
			EEPROM.put(EEPROM_START_SETTING_WM, wmSettings);
			EEPROM.end();
			//------------------------------------------
			LED_BLUE_OFF();

			delay(1000);
			WiFi.disconnect();
			delay(1000);
			ESP.restart();
			delay(1000);
		}
		else if (pressTime < INTERVAL_PRESSED_RESET_ESP && pressTime > INTERVAL_PRESSED_SHORT)
		{
			Serial.print(String(twoDigits(day())) + "." + twoDigits(month()) + "." + year() + " ");
			Serial.print(String(twoDigits(hour())) + ":" + twoDigits(minute()) + ":" + twoDigits(second()) + " - ");			
			Serial.println("Button pressed is Device!");

			if (windowState)
			{				
				myStepper.step(wmSettings.directionMotor * wmSettings.maxStepsMotor);

				currentPositionMotor = wmSettings.maxStepsMotor;

				windowState = false;
			}
			else
			{				
				myStepper.step(wmSettings.directionMotor * (-currentPositionMotor));

				currentPositionMotor = 0;

				windowState = true;
			}			
			
			ResetMotorPin();

			if (Blynk.connected())
			{				
				Blynk.virtualWrite(MOTOR_MOVE_SLIDER, currentPositionMotor);

				terminal.print(String(twoDigits(day())) + "." + twoDigits(month()) + "." + year() + " ");
				terminal.print(String(twoDigits(hour())) + ":" + twoDigits(minute()) + ":" + twoDigits(second()) + " - ");				
				terminal.print("Button pressed is Device - ");

				if (currentPositionMotor == 0)
				{					
					terminal.println("window is closed!");
				}
				else
				{				
					terminal.println("window is open!");
				}

			}			
		}
		else if (pressTime < INTERVAL_PRESSED_SHORT)
		{
			char str[30];
			sprintf(str, "Fixed false triggering %ims", pressTime);
			
			Serial.print(String(twoDigits(day())) + "." + twoDigits(month()) + "." + year() + " ");
			Serial.print(String(twoDigits(hour())) + ":" + twoDigits(minute()) + ":" + twoDigits(second()) + " - ");
			Serial.println(str);
			
			if (Blynk.connected())
			{
				terminal.print(String(twoDigits(day())) + "." + twoDigits(month()) + "." + year() + " ");
				terminal.print(String(twoDigits(hour())) + ":" + twoDigits(minute()) + ":" + twoDigits(second()) + " - ");
				terminal.println(str);
				// Blynk.notify(str);
			}
		}

		if (Blynk.connected())
		{
			terminal.flush();
		}
	}
}

static void TimeInputSheduler(BlynkParam param, uint8_t numSheduler)
{
	TimeInputParam t(param);

	uint8_t countondays = 0; // Кількість активних днів на тиждень коли є планувальник

	sheduler[numSheduler].activeToday = false;
	sheduler[numSheduler].activeDaysOfWeek = 0;

	int dayadjustment = -1;

	if (weekday() == 1)
	{
		dayadjustment = 6; // needed for Sunday, Time library is day 1 and Blynk is day 7
	}

	uint8_t dayNumber = weekday() + dayadjustment;

	Serial.println(String("Sheduler") + (numSheduler + 1) + " setting:");

	if (Blynk.connected())
	{
		terminal.println(String("Sheduler") + (numSheduler + 1) + " setting:");
	}
	
	// Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
	for (int i = 1; i <= 7; i++)
	{
		if (t.isWeekdaySelected(i))
		{
			sheduler[numSheduler].activeDaysOfWeek |= (1 << i); // Встановлюємо біти активний день тижня == номер біту

			countondays++;

			if (i == dayNumber)
			{
				sheduler[numSheduler].activeToday = true;  // set false at start of function
			}

			Serial.println(String("Day ") + i + " is selected");

			if (Blynk.connected())
			{
				terminal.println(String("Day ") + i + " is selected");
			}
		}
	}

	// Встановлюємо чи скидаємо біт активності планувальника на тиждень
	if (countondays == 0)
	{
		sheduler[numSheduler].activeDaysOfWeek &= (~(1 << 0)); // якщо кількість встановлених днів == 0 обнуляємо нульовий біт
	}
	else
	{
		sheduler[numSheduler].activeDaysOfWeek |= (1 << 0); // якщо кількість встановлених днів > 0 встановлюємо нульовий біт
	}

	// Перевіряємо чи змінився статус планувальника. Може колись згодиться.
	if ((sheduler[numSheduler].activeDaysOfWeek & (1 << 0)) != sheduler[numSheduler].schedulerStatusChanged)
	{
		sheduler[numSheduler].schedulerStatusChanged = (sheduler[numSheduler].activeDaysOfWeek & (1 << 0));

		Serial.println(String("Scheduler") + (numSheduler + 1) + " status changed");

		if (Blynk.connected())
		{
			terminal.println(String("Scheduler") + (numSheduler + 1) + " status changed");
		}
	}

	// Перевіряємо біт активності планувальника на тиждень 
	if ((sheduler[numSheduler].activeDaysOfWeek & (1 << 0)) == false)
	{
		Serial.println(String("Scheduler") + (numSheduler + 1) + " currently disabled");

		if (Blynk.connected())
		{
			terminal.println(String("Scheduler") + (numSheduler + 1) + " currently disabled");
		}
	}
	else
	{
		Serial.println(String("Scheduler") + (numSheduler + 1) + " currently enabled");

		if (Blynk.connected())
		{
			terminal.println(String("Scheduler") + (numSheduler + 1) + " currently enabled");
		}
	}

	// Перевіряємо чи цього дня планувальник активний	
	if ((sheduler[numSheduler].activeDaysOfWeek & (1 << 0)) && sheduler[numSheduler].activeToday)
	{
		Serial.println(String("Scheduler") + (numSheduler + 1) + " active today");

		if (Blynk.connected())
		{
			terminal.println(String("Scheduler") + (numSheduler + 1) + " active today");
		}
	}
	else if ((sheduler[numSheduler].activeDaysOfWeek & (1 << 0)) && !sheduler[numSheduler].activeToday)
	{
		Serial.println(String("Scheduler") + (numSheduler + 1) + " not active today");

		if (Blynk.connected())
		{
			terminal.println(String("Scheduler") + (numSheduler + 1) + " not active today");
		}
	}

	// Якщо планувальник на тиждень активовано, заносимо данні в структуру про старт та стоп планувальника.
	if ((sheduler[numSheduler].activeDaysOfWeek & (1 << 0)) == true)
	{
		// Process start time
		if (t.hasStartTime())
		{
			sheduler[numSheduler].triggerStart = true;
			sheduler[numSheduler].startSeconds = (t.getStartHour() * 3600) + (t.getStartMinute() * 60);

			Serial.println(String("Start: ") +
				t.getStartHour() + ":" +
				t.getStartMinute() + ":" +
				t.getStartSecond());

			if (Blynk.connected())
			{
				terminal.println(String("Start: ") +
					t.getStartHour() + ":" +
					t.getStartMinute() + ":" +
					t.getStartSecond());
			}
		}
		else if (t.isStartSunrise())
		{
			sheduler[numSheduler].triggerStart = true;

			Serial.println("Start at sunrise");

			if (Blynk.connected())
			{
				terminal.println("Start at sunrise");
			}
		}
		else if (t.isStartSunset())
		{
			sheduler[numSheduler].triggerStart = true;

			Serial.println("Start at sunset");

			if (Blynk.connected())
			{
				terminal.println("Start at sunset");
			}
		}
		else
		{
			// Do nothing: no start time was set
			sheduler[numSheduler].triggerStart = false;

			Serial.println("Start: Scheduler is not installed");

			if (Blynk.connected())
			{
				terminal.println("Start: Scheduler is not installed");
			}
		}

		// Process stop time
		if (t.hasStopTime())
		{
			sheduler[numSheduler].triggerStop = true;
			sheduler[numSheduler].stopSeconds = (t.getStopHour() * 3600) + (t.getStopMinute() * 60);

			Serial.println(String("Stop: ") +
				t.getStopHour() + ":" +
				t.getStopMinute() + ":" +
				t.getStopSecond());

			if (Blynk.connected())
			{
				terminal.println(String("Stop: ") +
					t.getStopHour() + ":" +
					t.getStopMinute() + ":" +
					t.getStopSecond());
			}
		}
		else if (t.isStopSunrise())
		{
			sheduler[numSheduler].triggerStop = true;

			Serial.println("Stop at sunrise");

			if (Blynk.connected())
			{
				terminal.println("Stop at sunrise");
			}
		}
		else if (t.isStopSunset())
		{
			sheduler[numSheduler].triggerStop = true;

			Serial.println("Stop at sunset");

			if (Blynk.connected())
			{
				terminal.println("Stop at sunset");
			}
		}
		else
		{
			// Do nothing: no stop time was set
			sheduler[numSheduler].triggerStop = false;

			Serial.println("Stop: Scheduler is not installed");

			if (Blynk.connected())
			{
				terminal.println("Stop: Scheduler is not installed");
			}
		}
	}

	// Process timezone
	// Timezone is already added to start/stop time
	Serial.println(String("Time zone: ") + t.getTZ());

	if (Blynk.connected())
	{
		terminal.println(String("Time zone: ") + t.getTZ());
	}

	// Get timezone offset (in seconds)
	Serial.println(String("Time zone offset: ") + t.getTZ_Offset());
	Serial.println();

	if (Blynk.connected())
	{
		terminal.println(String("Time zone offset: ") + t.getTZ_Offset());
		terminal.println();
		terminal.flush();
	}
}

void InitStartPosition(void)
{			
	unsigned int startFindPosition = millis();
	
	myStepper.setSpeed(wmSettings.speedMotor);
	
	while (digitalRead(INITIAL_POSITION_SWITCH) && ((millis() - startFindPosition) < FIND_MOTOR_POSITION_TIMEOUT))
	{
		myStepper.step(wmSettings.directionMotor * 10);
	}

	windowState = true;	
	currentPositionMotor = 0;

	ResetMotorPin();
}

void ResetMotorPin(void)
{
	digitalWrite(ULN2003_IN1, LOW);
	digitalWrite(ULN2003_IN2, LOW);
	digitalWrite(ULN2003_IN3, LOW);
	digitalWrite(ULN2003_IN4, LOW);
}

void timerRefreshData(void)
{
	getWiFiLevelSignal();

	getDHTData();
}

void timerSendServer(void)
{
	if (Blynk.connected())
	{
		Blynk.virtualWrite(WIFI_SIGNAL, wifisignal);

		Blynk.virtualWrite(TMP_DHT, dhtStruct.t);
		Blynk.virtualWrite(HUM_DHT, dhtStruct.h);
	}
}

void timerReconnect(void)
{
	if (WiFi.status() != WL_CONNECTED)
	{
		Serial.println("WiFi not connected");

		WiFi.begin();
	}
	else
	{
		Serial.println("WiFi in connected");
	}

	if (!Blynk.connected())
	{
		if (Blynk.connect())
		{
			Serial.println("Blynk reconnected");
		}
		else
		{
			Serial.println("Blynk not reconnected");
		}
	}
	else
	{
		Serial.println("Blynk in connected");
	}
}

void timerWriteSetting(void)
{
	firstSettingChange = true;
	
	LED_BLUE_ON();
	EEPROM.begin(EEPROM_SETTINGS_SIZE);
	EEPROM.put(EEPROM_START_SETTING_WM, wmSettings);
	EEPROM.end();
	LED_BLUE_OFF();
	Serial.println("Save setting completed");
	terminal.println("Save setting completed");
	terminal.flush();
}

void timeChecker(void)
{
	for (uint8_t i = 0; i < NUMBER_SHEDULERS; i++)
	{
		// якщо є щось заплановане на тиждень то...
		if ((sheduler[i].activeDaysOfWeek & (1 << 0)) == true)
		{
			int dayadjustment = -1;

			if (weekday() == 1)
			{
				dayadjustment = 6; // needed for Sunday, Time library is day 1 and Blynk is day 7
			}
			// отримуємо поточний день тижня у форматі blynk
			uint8_t dayNumber = weekday() + dayadjustment;
			// якщо є щось заплановане на цей день то...
			if ((sheduler[i].activeDaysOfWeek & (1 << dayNumber)) != 0)
			{
				// визначаємо скільки секунд від початку дня до поточного часу
				unsigned long daySeconds = ((hour() * 3600) + (minute() * 60) + second());

				if (sheduler[i].triggerStart)
				{
					// в час старту планувальника щось увімкнемо
					if ((daySeconds >= sheduler[i].startSeconds) && (daySeconds <= sheduler[i].startSeconds + 10))
					{						
						myStepper.step(wmSettings.directionMotor * (wmSettings.maxStepsMotor - currentPositionMotor));
						ResetMotorPin();
						currentPositionMotor = wmSettings.maxStepsMotor;

						windowState = false;												
					}
				}
				if (sheduler[i].triggerStop)
				{
					// в час стоп планувальника щось вимкнемо
					if ((daySeconds >= sheduler[i].stopSeconds) && (daySeconds <= sheduler[i].stopSeconds + 10))
					{
						myStepper.step(wmSettings.directionMotor * (-currentPositionMotor));
						ResetMotorPin();
						currentPositionMotor = 0;

						windowState = true;						
					}
				}
			}
			else if ((sheduler[i].activeDaysOfWeek & (1 << dayNumber)) == 0)
			{
				//TODO: 
			}
		}
	}
}

void getWiFiLevelSignal(void)
{
	wifisignal = map(WiFi.RSSI(), -105, -40, 0, 100);
	
	Serial.print("WiFi signal level: ");
	Serial.print(wifisignal);
	Serial.println("%");
}

void getDHTData(void)
{
	float temp_h_dht = dht.readHumidity();
	float temp_t_dht = dht.readTemperature();

	if (isnan(temp_h_dht) || isnan(temp_t_dht))
	{
		Serial.print(String(twoDigits(day())) + "." + twoDigits(month()) + "." + year() + " ");
		Serial.print(String(twoDigits(hour())) + ":" + twoDigits(minute()) + ":" + twoDigits(second()) + " - ");		
		Serial.println("DHT is read error!");
		if (Blynk.connected())
		{
			terminal.print(String(twoDigits(day())) + "." + twoDigits(month()) + "." + year() + " ");
			terminal.print(String(twoDigits(hour())) + ":" + twoDigits(minute()) + ":" + twoDigits(second()) + " - ");
			terminal.println("DHT is read error!");
			terminal.flush();
		}
	}
	else
	{
		dhtStruct.h = temp_h_dht;
		dhtStruct.t = temp_t_dht;
		//hic = dht.computeHeatIndex(t_dht, h_dht, false);

		Serial.print("Tmp: ");
		Serial.print(dhtStruct.t);
		Serial.println("C");
		Serial.print(" Hum: ");
		Serial.print(dhtStruct.h);
		Serial.println("%");
		// Serial.print(" HIC: ");
		// Serial.println(hic);
	}
}

// utility function for digital clock display: prints leading 0
static String twoDigits(int digits)
{
	if (digits < 10)
	{
		String i = '0' + String(digits);
		return i;
	}
	else
	{
		return String(digits);
	}
}

/* CODE END 4 */