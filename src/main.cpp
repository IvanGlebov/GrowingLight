#define BLYNK_PRINT Serial
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <WidgetRTC.h>

// Общие константы
#define USE_BLYNK true
#define USE_LCD true
#define SHOW_DEBUG true
#define SEND_SENSORS_DATA true
#define PUSH_RELAYS true
#define DISABLE_STEPPER true
#define STORE_TO_EEPROM true
#define RECOVER_FROM_EEPROM true
#define SHOW_SENSORS_DATA true
#define SHOW_WATRING_PING true
#define SHOW_WATERING_PROCESS false

// Константы для управления тестами
#define TEST_GET_BME true
#define TEST_GROUND_HUM false
#define TEST_LIGHT false
#define TEST_GROUND_TEMP true
#define TEST_STEPPER false
#define TEST_CHECK_BORDER_FULL_FALSE true
#define SHOW_RELAYS_PUSHING true

// Датчик освещённости
BH1750 lightSensor(0x23);
// Датчик температуры и влажности воздуха
Adafruit_BME280 bme280;
// Датчик температуры почвы
#define ONE_WIRE_BUS D7
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature groundSensors(&oneWire);

// Дисплей
LiquidCrystal_I2C lcd(0x27, 20, 4);
#define GROUND_HUM_SENSOR_PIN A0

// Blynk pins mapping
#define GROUND_TEMP_PIN V1
#define GROUND_HUM_PIN V2
#define AIR_TEMP_PIN V3
#define AIR_HUM_PIN V4
#define LIGHT_LEVEL_PIN V5
#define AIR_PRESSURE_PIN V20

//////////////////////////////////////////////////////////
// КОНСТАНТЫ И СТРУКТУРЫ ДАННЫХ //
//////////////////////////////////////////////////////////
//-------------------------//
//--БЛОК-АВТОМАТИКИ--------//
//-------------------------//
#define LIGHT_AUTO true
#define SHOW_AUTO_LIGHT_LOG false
#define DISPLAY_LIGHT_TO_BLYNK true
#define WATER_AUTO true

#define NOTIFICATIONS_PIN V0
#define AUTO_LIGHT_PIN V11
#define LIGHT_BORDER_PIN V12
#define RED_HUM_NOTIFICATION_PIN V13
#define YELLOW_HUM_NOTIFICATION_PIN V14
#define GREEN_HUM_NOTIFICATION_PIN V15
#define WATERING_NOTIFICATIONS_MODE_PIN V16
#define RED_LED_PIN V17
#define YELLOW_LED_PIN V18
#define GREEN_LED_PIN V19
#define START_WATERING_PIN V21
#define BASE_WATERING_TIME_PIN V22
#define RED_HUM_BORDER_PIN V23
#define YELLOW_HUM_BORDER_PIN V24
#define GREEN_HUM_BORDER_PIN V25
#define WATERING_TIME_LEFT V26
#define WATERING_AUTO_MODE_PIN V27

struct autoModeVaruables
{
  int autoLightFlag;
  int autoLightFlagEEPROM;
  int lightBorder;
  int lightBorderEEPROM;

  // Полив
  int wateringMode;
  int wateringModeEEPROM;
  int redHumNotificationBorder;
  int redHumNotificationBorderEEPROM;
  int yellowHumNotificationBorder;
  int yellowHumNotificationBorderEEPROM;
  int greenHumNotificationBroder;
  int greenHumNotificationBroderEEPROM;
  int wateringNotificationsMode;
  int wateringNotificationsModeEEPROM;
  int wateringToOnFlag;
  int baseWateringDuration;
  int baseWateringDurationEEPROM;
  int wateringDuration;
  int wateringTimestamp;
  int wateringFlag;
  int wateringAutoMode;
  int wateringAutoModeEEPROM;
  /*
0%           25%         50%  60%    70% 75%        100%
|----------------------------------------------------|
0% -> 50% - Красная зона
При 25% и ниже выскакивает уведомление о попадении в красную зону

50% -> 75% - Жёлтая зона
При 60% выскакивает уведомлении о попадении в жёлтую зону

70% -> 100% - Зелёная зона
При 75% выскакивает уведомление о попадении в зелёную зону
Включение полива заблокировано.
*/
};

autoModeVaruables autoVariables;

void fillAutoVariables(autoModeVaruables *variables)
{
  variables->autoLightFlag = 1; // 1 - автоматика, 2 - ручной
  variables->lightBorder = 200; // 200 lux
  variables->autoLightFlagEEPROM = 2;
  variables->lightBorderEEPROM = 3;
  // Полив
  variables->wateringMode = 1;                 // 1 - красный режим, 2 - жёлтый, 3 - зелёный
  variables->redHumNotificationBorder = 25;    // 25%
  variables->yellowHumNotificationBorder = 60; // 60%
  variables->greenHumNotificationBroder = 75;  // 75%
  variables->wateringNotificationsMode = 1;    // 1 - получать уведомления, 2 - не получать уведомления
  variables->wateringToOnFlag = 0;
  variables->wateringModeEEPROM = 4;
  variables->redHumNotificationBorderEEPROM = 5;
  variables->yellowHumNotificationBorderEEPROM = 6;
  variables->greenHumNotificationBroderEEPROM = 7;
  variables->wateringNotificationsModeEEPROM = 8;
  variables->baseWateringDurationEEPROM = 9;
  variables->wateringAutoModeEEPROM = 10;
}

// ----------------------- //
// Блок РЕЛЕ
// ----------------------- //
// Пин реле освещения
#define LIGHT_RELAY_PIN V9
#define LIGHT_EEPROM_NUM 0
// Пин реле помпы
#define PUMP_RELAY_PIN V10
#define PUMP_EEPROM_NUM 1
struct relayStates
{
  const uint8_t pin;
  const int virtualPin;
  bool state;
  int eeprom_num;
};
// Структура для хранения состояний реле
struct relaysArray
{
  relayStates lightRelay = {D5, LIGHT_RELAY_PIN, false, 0};
  relayStates pumpRelay = {D6, PUMP_RELAY_PIN, false, 1};
};

//------------------------------------------------------//
//---------------ПАМЯТЬ-ПРИ-ПЕРЕЗАПУСКЕ-----------------//
//------------------------------------------------------//
// Количество байт памяти занятых для работы EEPROM
#define EEPROM_CELLS_NUMBER 30
// Функция для чтения данных из EEPROM в заранее прописанные переменные

struct actionStruct
{
  String actionType;
  int value;
  int eepromPin;
};

void saveToEEPROM(actionStruct *action)
{
  EEPROM.write(action->eepromPin, int(action->value));
  EEPROM.commit();
  if (SHOW_DEBUG)
  {
    Serial.println("Value " + String(action->value) + " is stored to EEPROM");
  }
}

void recoverEEPROM(relaysArray *relays, autoModeVaruables *variables)
{
  // +
  relays->lightRelay.state = EEPROM.read(relays->lightRelay.eeprom_num);
  // +
  relays->pumpRelay.state = EEPROM.read(relays->pumpRelay.eeprom_num);

  // +
  variables->autoLightFlag = EEPROM.read(variables->autoLightFlagEEPROM);
  Blynk.virtualWrite(AUTO_LIGHT_PIN, variables->autoLightFlag);
  // +
  variables->lightBorder = 20 * EEPROM.read(variables->lightBorderEEPROM);
  Blynk.virtualWrite(LIGHT_BORDER_PIN, variables->lightBorder);

  variables->wateringMode = EEPROM.read(variables->wateringModeEEPROM);
  // Blynk.virtualWrite(WATERING_MODE_PIN);

  variables->wateringAutoMode = EEPROM.read(variables->wateringAutoModeEEPROM);
  Blynk.virtualWrite(WATERING_AUTO_MODE_PIN, variables->wateringAutoMode);

  // +
  variables->redHumNotificationBorder = EEPROM.read(variables->redHumNotificationBorderEEPROM);
  Blynk.virtualWrite(RED_HUM_BORDER_PIN, variables->redHumNotificationBorder);
  // +
  variables->yellowHumNotificationBorder = EEPROM.read(variables->yellowHumNotificationBorderEEPROM);
  Blynk.virtualWrite(YELLOW_HUM_BORDER_PIN, variables->yellowHumNotificationBorder);
  // +
  variables->greenHumNotificationBroder = EEPROM.read(variables->greenHumNotificationBroderEEPROM);
  Blynk.virtualWrite(GREEN_HUM_BORDER_PIN, variables->greenHumNotificationBroder);
  // +
  variables->wateringNotificationsMode = EEPROM.read(variables->wateringNotificationsModeEEPROM);
  Blynk.virtualWrite(WATERING_NOTIFICATIONS_MODE_PIN, variables->wateringNotificationsMode);

  // +
  variables->baseWateringDuration = EEPROM.read(variables->baseWateringDurationEEPROM);
  Blynk.virtualWrite(BASE_WATERING_TIME_PIN, variables->baseWateringDuration);
  if (SHOW_DEBUG)
  {
    Serial.println("Recovering data...");
    delay(200);
    Serial.println("Light relay state: " + String(EEPROM.read(relays->lightRelay.eeprom_num)));
    Serial.println("Pump relay state: " + String(EEPROM.read(relays->pumpRelay.eeprom_num)));
    Serial.println("Auto light flag: " + String(variables->autoLightFlag));
    Serial.println("Recovery finished");
  }
}

//------------------------------------------------------//
//-------------------ШАГОВЫЙ-ДВИГАТЕЛЬ------------------//
//------------------------------------------------------//
#define CURRENT_STEP_PIN V6
#define APPROVE_STEP_PIN V7
#define CURRENT_STEP_DISPLAY_PIN V8
// Библиотека для направления движения шагового двигателя
enum directions
{
  down = 0,
  up
};
// Структура содержащая все настройки для работы функций с шаговым двигателем
struct stepperSettings
{
  // Пин для указания направления движения
  uint8_t directionPin;
  // Пин для подачи сигнала шага
  uint8_t steppingPin;
  // Длительность задержки шага мотора в микросекундах
  int stepTime;
  // Количество шагов на преодоление всей высоты
  int maxStep;
  // Пин верхнего датчика касания
  uint8_t topSensor;
  // Пин нижнего датчика касания
  uint8_t bottomSensor;
  // Минимальное положение в шагах (обычно 0, но можно двинуть)
  int bottomStep;
};
stepperSettings motor1 = {D7, D6, 500, 2000, D3, D2, 0};
// Переменная для хранения значения пришедшего шага с Blynk
int tempStep;
// Фактическое значение текущего шага
int currentStep;

//------------------------------------------------------//
//-------------------------РЕЛЕ-------------------------//
//------------------------------------------------------//
// Вариант замены структуры на массив для удобства написания функции применения.
// В таком варианте раписывать значения становится довольно неудобно.
relaysArray relays;
// Функция применения значений с логических переменных реле
void useRelays(relaysArray *relays, int negResponseToBlynkValue = 0, int posResponseToBlynkValue = 1)
{
  if (relays->lightRelay.state == true)
  {
    digitalWrite(relays->lightRelay.pin, HIGH);
    // Blynk.virtualWrite(relays->lightRelay.virtualPin, posResponseToBlynkValue);
  }
  else
  {
    digitalWrite(relays->lightRelay.pin, LOW);
    // Blynk.virtualWrite(relays->lightRelay.virtualPin, negResponseToBlynkValue);
  }
  if (relays->pumpRelay.state == true)
  {
    digitalWrite(relays->pumpRelay.pin, HIGH);
    // Blynk.virtualWrite(relays->pumpRelay.virtualPin, posResponseToBlynkValue);
  }
  else
  {
    digitalWrite(relays->pumpRelay.pin, LOW);
    // Blynk.virtualWrite(relays->pumpRelay.virtualPin, negResponseToBlynkValue);
  }
  if (SHOW_DEBUG)
  {
    if (SHOW_RELAYS_PUSHING)
    {
      Serial.println("//----------Pushed relays-------------//");
      Serial.println("Light relay is " + String(relays->lightRelay.state));
      Serial.println("Pump relay is " + String(relays->pumpRelay.state));
    }
  }
}
// Callback функция, которая нужна только для дёргания другой функции по таймеру
void useRelaysCallback()
{
  useRelays(&relays, 0, 1);
}
BlynkTimer pushRelays;
// Реле освещения
BLYNK_WRITE(LIGHT_RELAY_PIN)
{
  int a = param.asInt();
  relays.lightRelay.state = (a == 1) ? true : false;
  if (STORE_TO_EEPROM)
  {
    actionStruct action = {"Light realy store", relays.lightRelay.state, relays.lightRelay.eeprom_num};
    // saveToEEPROM(&action);
    EEPROM.write(relays.lightRelay.eeprom_num, int(relays.lightRelay.state));
    EEPROM.commit();
    if (SHOW_DEBUG)
    {
      Serial.println("Stored value to byte " + String(relays.lightRelay.eeprom_num) + " is " + String(relays.lightRelay.state));
    }
  }
  if (SHOW_DEBUG)
  {
    Serial.println("Light relay changed logic state to " + String(relays.lightRelay.state));
  }
}
// Реле помпы
BLYNK_WRITE(PUMP_RELAY_PIN)
{
  int a = param.asInt();
  relays.pumpRelay.state = (a == 1) ? true : false;
  if (STORE_TO_EEPROM)
  {
    actionStruct action = {"Pump realy store", relays.pumpRelay.state, relays.pumpRelay.eeprom_num};
    // saveToEEPROM(&action);
    EEPROM.write(relays.pumpRelay.eeprom_num, int(relays.pumpRelay.state));
    EEPROM.commit();
    if (SHOW_DEBUG)
    {
      Serial.println("Stored value to byte " + String(relays.pumpRelay.eeprom_num) + " is " + String(relays.pumpRelay.state));
    }
  }
  if (SHOW_DEBUG)
  {
    Serial.println("Pump relay changed logic state to " + String(relays.pumpRelay.state));
  }
}

// Настройки для подключения к Blynk
// Тут надо указать ключ конкретного устройства
char auth[] = "";
// Тут надо указать название WiFi сети
char ssid[] = "";
// Тут надо указать пароля для WiFi сети
char pass[] = "";

BlynkTimer transferData;

// Структура для хранения данных с сенсоров
struct sensorsData
{
  float groundTemp;
  float groundHum;
  float airTemp;
  float airHum;
  float lightLevel;
  float airPressure;
};

//------------------------------------------------------//
//-ФУНКЦИИ-ДЛЯ-ПОЛУЧЕНИЯ-ДАННЫХ-С-ДИТЧИКОВ-И-ТЕСТОВ-----//
//------------------------------------------------------//
// Функция для отображения данных в консоль из структуры
void showSensorsData(sensorsData *data)
{
  Serial.println("Ground humidity: " + String(data->groundHum));
  Serial.println("Ground temperature: " + String(data->groundTemp));
  Serial.println("Air temperature: " + String(data->airTemp));
  Serial.println("Air humidity: " + String(data->airHum));
  Serial.println("Light level: " + String(data->lightLevel));
  Serial.println("----------------------------------------------");
}
// Функция принимает на вход указатель на переменную в которую будет записан ответ и номер пина с которого
// будут читаться данные. Ответ возвращается как число в диапазоне от 0 до 100.
// void getGroundHumidity(float* res, uint8_t pin){
//   *res = map(analogRead(pin), 0, 1024, 0, 100);
// }
void getGroundHumidity(sensorsData *data, uint8_t pin)
{
  // %101 используется для отрезания лишних значений, которые превышают
  data->groundHum = map(analogRead(pin), 9, 1024, 100, 0);
  // data->groundHum = analogRead(A0);
}
// Функция принимает на вход указатель на переменную в которую будет записан ответ, указатель на объект
// датчиков DS18B20 и вариативно индекс датчика. Если после выполнения пришло значение -100, то при
// чтении произошла ошибка.
void getGroundTemperature(sensorsData *data, DallasTemperature *sensors, int sensorIndex = 0)
{
  sensors->requestTemperatures();
  float temp = sensors->getTempCByIndex(sensorIndex);
  if (temp != DEVICE_DISCONNECTED_C)
  {
    data->groundTemp = temp;
  }
  else
  {
    data->groundTemp = -100;
  }
}
// Функция принимает на вход указатель на переменную в которую будет записан ответ и указатель на объект
// BME280, из которого будут читаться данные
void getBME(sensorsData *data, Adafruit_BME280 *sensor)
{
  data->airTemp = sensor->readTemperature();
  data->airHum = sensor->readHumidity();
  data->airPressure = sensor->readPressure();
}
// Функция принимает на вход указатель на переменну в которую будет записан ответ и указатель на объект
// BH1750, из которого будут читаться данные
void getLightLevel(sensorsData *data, BH1750 *sensor)
{
  float lux;
  data->lightLevel = sensor->readLightLevel();
  lux = data->lightLevel;
  // Measurement time correcting
  if (lux < 0)
    Serial.println("Error reading ligt level");
  else
  {
    if (lux > 40000.0)
    {
      // reduce measurement time - when on direct sun light
      if (lightSensor.setMTreg(32))
      {
        // if (SHOW_DEBUG) Serial.println(F("Setting MTReg to low value for high light environment"));
      }
      else
      {
        // if(SHOW_DEBUG) Serial.println(F("Error setting MTReg to low value for high light environment"));
      }
    }
    else
    {
      if (lux > 10.0)
      {
        // regular light enviroment
        if (lightSensor.setMTreg(69))
        {
          // if (SHOW_DEBUG) Serial.println(F("Setting MTReg to default value for normal light environment"));
        }
        else
        {
          // if (SHOW_DEBUG) Serial.println(F("Error setting MTReg to default value for normal light environment"));
        }
      }
      else
      {
        if (lux <= 10.0)
        {
          // very low light enviroment
          if (lightSensor.setMTreg(138))
          {
            // if (SHOW_DEBUG) Serial.println(F("Setting MTReg to high value for low light environment"));
          }
          else
          {
            // if (SHOW_DEBUG) Serial.println(F("Error setting MTReg to high value for low light environment"));
          }
        }
      }
    }
  }
}

//------------------------------------------------------//
//----------------------ШАГОВЫЙ-ПРИВОД------------------//
//------------------------------------------------------//
// Функция для проверки того упёрлись мы в край или нет
bool checkBorder(uint8_t pin)
{
  if (TEST_CHECK_BORDER_FULL_FALSE)
  {
    return false;
  }
  else
  {
    return (digitalRead(pin) == HIGH) ? true : false;
  }
}
// Функция для передвижения каретки в конкретное положение (шаги/высота)
bool moveCarriageTo(int positionInSteps, int *currentPosition, stepperSettings *motor)
{
  bool direction = *(currentPosition) > positionInSteps ? down : up;
  int stepDifference = abs(*(currentPosition)-positionInSteps);
  if (SHOW_DEBUG)
  {
    Serial.println("Direction: " + String(direction == down ? "down" : "up"));
    Serial.println("Step difference: " + String(stepDifference));
  }
  // Установка направления движения
  if (direction == up)
  {
    digitalWrite(motor->directionPin, HIGH);
  }
  else
  {
    digitalWrite(motor->directionPin, LOW);
  }
  // Само движение шагового двигателя
  if (direction == up)
  {
    for (int i = 0; i < stepDifference; i++)
    {
      if (checkBorder(motor->topSensor) != true)
      {
        digitalWrite(motor->steppingPin, HIGH);
        delayMicroseconds(motor->stepTime);
        digitalWrite(motor->steppingPin, LOW);
        delayMicroseconds(motor->stepTime);
        *(currentPosition) = *(currentPosition) + 1;
        if ((*(currentPosition) % 10 == 0) || (i == stepDifference - 1))
        {
          Blynk.virtualWrite(CURRENT_STEP_DISPLAY_PIN, *(currentPosition));
        }
      }
      else
      {
        if (SHOW_DEBUG)
        {
          Serial.println("Top reached");
          Serial.println("Current step is set to " + String(motor->maxStep));
        }
        *(currentPosition) = motor->maxStep;
        return false;
      }
    }
  }
  else if (direction == down)
  {
    for (int i = 0; i < stepDifference; i++)
    {
      if (checkBorder(motor->bottomSensor) != true)
      {
        digitalWrite(motor->steppingPin, HIGH);
        delayMicroseconds(motor->stepTime);
        digitalWrite(motor->steppingPin, LOW);
        delayMicroseconds(motor->stepTime);
        *(currentPosition) = *(currentPosition)-1;
        if ((*(currentPosition) % 10 == 0) || (i == stepDifference - 1))
        {
          Blynk.virtualWrite(CURRENT_STEP_DISPLAY_PIN, *(currentPosition));
        }
      }
      else
      {
        if (SHOW_DEBUG)
        {
          Serial.println("Bottom reached");
          Serial.println("Current step is set to " + String(motor->bottomStep));
        }
        *(currentPosition) = motor->bottomStep;
        return false;
      }
    }
  }
  // Функция отработала корректно и мотор не упёрся в границу
  return true;
}
// Функция калибровки положения каретки. Каретка спускается вниз, поднимается вверх считая шаги и записывает значение в поле максимума
void calibrate(stepperSettings *motor)
{
  // Вниз до упора
  digitalWrite(motor->directionPin, LOW);
  while (checkBorder(motor->bottomSensor) != true)
  {
    digitalWrite(motor->steppingPin, HIGH);
    delayMicroseconds(motor->stepTime);
    digitalWrite(motor->steppingPin, LOW);
    delayMicroseconds(motor->stepTime);
  }
  // Вверх до упора и считаем шаги
  digitalWrite(motor->directionPin, HIGH);
  int steps = 0;
  while (checkBorder(motor->topSensor) != true)
  {
    digitalWrite(motor->steppingPin, HIGH);
    delayMicroseconds(motor->stepTime);
    digitalWrite(motor->steppingPin, LOW);
    delayMicroseconds(motor->stepTime);
    steps++;
  }
  // Вывод информации в консоль
  if (SHOW_DEBUG)
  {
    Serial.println("Full height is " + String(steps) + " steps");
  }
  motor->maxStep = steps;
  // Обратно опускаемся вниз до упора
  digitalWrite(motor->directionPin, LOW);
  while (checkBorder(motor->bottomSensor) != true)
  {
    digitalWrite(motor->steppingPin, HIGH);
    delayMicroseconds(motor->stepTime);
    digitalWrite(motor->steppingPin, LOW);
    delayMicroseconds(motor->stepTime);
  }
}
BLYNK_WRITE(CURRENT_STEP_PIN)
{
  if (!DISABLE_STEPPER)
  {
    int a = param.asInt();
    tempStep = a;
  }
}
BLYNK_WRITE(APPROVE_STEP_PIN)
{
  if (!DISABLE_STEPPER)
  {
    if (param.asInt() == 1)
    {
      if (TEST_STEPPER)
      {
        Serial.println("Current step is " + String(currentStep));
        Serial.println("Moving to " + String(tempStep));
        currentStep = tempStep;
        Serial.println("Test moving finished.");
      }
      else
      {
        moveCarriageTo(tempStep, &currentStep, &motor1);
      }
      Blynk.virtualWrite(CURRENT_STEP_DISPLAY_PIN, currentStep);
    }
  }
}

//------------------------------------------------------//
//-ФУНКЦИИ-ДЛЯ-ОТПРАВКИ-ДАННЫХ-НА-BLYNK-----------------//
//------------------------------------------------------//
sensorsData dataStorage;
// Функция, которая собирает данные с сенсоров и записывает их в структуру data
void collectData(sensorsData *data, uint8_t pin, DallasTemperature *groundTempSensors,
                 Adafruit_BME280 *bme280Sensor, BH1750 *bh1750Sensor)
{
  getGroundHumidity(data, pin);
  getGroundTemperature(data, groundTempSensors);
  getBME(data, bme280Sensor);
  getLightLevel(data, bh1750Sensor);
}
// Функция, которая отправляет данные из структуры data на Blynk согласно входным пинам
void pushDataToBlynk(sensorsData *data, int groundTempPin, int groundHumPin, int airTempPin,
                     int airHumPin, int lightLevelPin, int airPressurePin)
{
  Blynk.virtualWrite(groundHumPin, data->groundHum);
  Blynk.virtualWrite(groundTempPin, data->groundTemp);
  Blynk.virtualWrite(airTempPin, data->airTemp);
  Blynk.virtualWrite(airHumPin, data->airHum);
  Blynk.virtualWrite(lightLevelPin, data->lightLevel);
  Blynk.virtualWrite(airPressurePin, data->airPressure);
}
// Функция, которая собирает и отправляет данные на Blynk
void blynkDataTransfer()
{
  if (SHOW_DEBUG && SHOW_SENSORS_DATA)
  {
    Serial.println("Sending data to Blynk");
  }
  collectData(&dataStorage, GROUND_HUM_SENSOR_PIN, &groundSensors, &bme280, &lightSensor);
  pushDataToBlynk(&dataStorage, GROUND_TEMP_PIN, GROUND_HUM_PIN, AIR_TEMP_PIN, AIR_HUM_PIN, LIGHT_LEVEL_PIN, AIR_PRESSURE_PIN);
  if (SHOW_DEBUG && SHOW_SENSORS_DATA)
  {
    showSensorsData(&dataStorage);
  }
}

//------------------------------------------------------//
//---------------------ГРАНИЦЫ--------------------------//
//------------------------------------------------------//

struct sensorsBorders
{
  int lightBorder;
  int redHumBorder;
  int yellowHumBorder;
  int greenHumBorder;
};

sensorsBorders borders;

//------------------------------------------------------//
//--------------------АВТОМАТИКА------------------------//
//------------------------------------------------------//
WidgetRTC rtc;
BLYNK_CONNECTED()
{
  rtc.begin();
}
BlynkTimer autoLight;
// Автоматическое включение и выключение света
void lightControl(relayStates *lightRelay, int border, int lightLevel, int gesteresis, int workFlag)
{
  if (workFlag == 1)
  {
    if (lightLevel < border - gesteresis)
    {
      lightRelay->state = true;
      if (DISPLAY_LIGHT_TO_BLYNK)
      {
        Blynk.virtualWrite(lightRelay->virtualPin, 1);
      }
      if (SHOW_DEBUG && SHOW_AUTO_LIGHT_LOG)
      {
        Serial.println("AUTO! -> Light ON.");
      }
    }
    if (lightLevel > border + gesteresis)
    {
      lightRelay->state = false;
      if (DISPLAY_LIGHT_TO_BLYNK)
      {
        Blynk.virtualWrite(lightRelay->virtualPin, 0);
      }
      if (SHOW_DEBUG && SHOW_AUTO_LIGHT_LOG)
      {
        Serial.println("AUTO! -> Light OFF.");
      }
    }
  }
}
void lightControlWrapper()
{
  lightControl(&relays.lightRelay, autoVariables.lightBorder, dataStorage.lightLevel,
               30, autoVariables.autoLightFlag);
}
// Получение режима работы освещения
BLYNK_WRITE(AUTO_LIGHT_PIN)
{
  autoVariables.autoLightFlag = param.asInt();

  EEPROM.write(autoVariables.autoLightFlagEEPROM, autoVariables.autoLightFlag);
  EEPROM.commit();

  if (SHOW_DEBUG)
  {
    Serial.println("Light mode CHANGED to: " + String(autoVariables.autoLightFlag));
  }
}
// Получение границы для срабатывания освещения
BLYNK_WRITE(LIGHT_BORDER_PIN)
{
  autoVariables.lightBorder = param.asInt();

  EEPROM.write(autoVariables.lightBorderEEPROM, autoVariables.lightBorder / 20);
  EEPROM.commit();

  if (SHOW_DEBUG)
  {
    Serial.println("Light border CHANGED to: " + String(autoVariables.lightBorder));
  }
}

BlynkTimer autoWatering;
BLYNK_WRITE(RED_HUM_NOTIFICATION_PIN)
{
  autoVariables.redHumNotificationBorder = param.asInt();
  if (STORE_TO_EEPROM)
  {
    EEPROM.write(autoVariables.redHumNotificationBorderEEPROM, autoVariables.redHumNotificationBorder);
    EEPROM.commit();
  }
}
BLYNK_WRITE(YELLOW_HUM_NOTIFICATION_PIN)
{
  autoVariables.yellowHumNotificationBorder = param.asInt();
  if (STORE_TO_EEPROM)
  {
    EEPROM.write(autoVariables.yellowHumNotificationBorderEEPROM, autoVariables.yellowHumNotificationBorder);
    EEPROM.commit();
  }
}
BLYNK_WRITE(GREEN_HUM_NOTIFICATION_PIN)
{
  autoVariables.greenHumNotificationBroder = param.asInt();
  if (STORE_TO_EEPROM)
  {
    EEPROM.write(autoVariables.greenHumNotificationBroderEEPROM, autoVariables.greenHumNotificationBroder);
    EEPROM.commit();
  }
}

BLYNK_WRITE(WATERING_NOTIFICATIONS_MODE_PIN)
{
  autoVariables.wateringNotificationsMode = param.asInt();
  if (STORE_TO_EEPROM)
  {
    EEPROM.write(autoVariables.wateringNotificationsModeEEPROM, autoVariables.wateringNotificationsMode);
    EEPROM.commit();
  }
}
BLYNK_WRITE(START_WATERING_PIN)
{
  int a = param.asInt();
  if (autoVariables.wateringToOnFlag == 0)
  {
    autoVariables.wateringToOnFlag = (a == 1) ? 1 : 0;
  }
  if (SHOW_DEBUG && SHOW_WATRING_PING)
  {
    Serial.println("Включение полива");
  }
}
BLYNK_WRITE(BASE_WATERING_TIME_PIN)
{
  autoVariables.baseWateringDuration = param.asInt();
  if (SHOW_DEBUG)
  {
    Serial.println("New base watering time is: " + String(autoVariables.baseWateringDuration));
  }
  if (STORE_TO_EEPROM)
  {
    EEPROM.write(autoVariables.baseWateringDurationEEPROM, autoVariables.baseWateringDuration);
    EEPROM.commit();
  }
}
BLYNK_WRITE(WATERING_AUTO_MODE_PIN)
{
  autoVariables.wateringAutoMode = param.asInt();
  if (STORE_TO_EEPROM)
  {
    EEPROM.write(autoVariables.wateringAutoModeEEPROM, autoVariables.wateringAutoMode);
    EEPROM.commit();
  }
}
enum wateringModes
{
  red = 1,
  yellow,
  green
};
// Функиця для установки режима в котором сейчас находится
void wateringModeControl(autoModeVaruables *variables, sensorsData *data)
{
  if (variables->wateringFlag != 1)
  {
    if (50 > data->groundHum)
    {
      variables->wateringMode = red;

      int diff = 75 - data->groundHum;
      variables->wateringDuration = ((diff % 20) * variables->baseWateringDuration) / 20;
      Blynk.virtualWrite(RED_LED_PIN, 255);
      Blynk.virtualWrite(YELLOW_LED_PIN, 0);
      Blynk.virtualWrite(GREEN_LED_PIN, 0);
    }
    else if ((50 <= data->groundHum) && (data->groundHum < 75))
    {
      variables->wateringMode = yellow;

      int diff = 75 - data->groundHum;
      variables->wateringDuration = ((diff % 20) * variables->baseWateringDuration) / 20;
      Blynk.virtualWrite(RED_LED_PIN, 0);
      Blynk.virtualWrite(YELLOW_LED_PIN, 255);
      Blynk.virtualWrite(GREEN_LED_PIN, 0);
    }
    else if (75 <= data->groundHum)
    {
      variables->wateringMode = green;

      int diff = 75 - data->groundHum;
      variables->wateringDuration = ((diff % 20) * variables->baseWateringDuration) / 20;
      Blynk.virtualWrite(RED_LED_PIN, 0);
      Blynk.virtualWrite(YELLOW_LED_PIN, 0);
      Blynk.virtualWrite(GREEN_LED_PIN, 255);
    }
  }
}

// Функция для управления помпой при автоматическом режиме работы
void watering(autoModeVaruables *variables, relayStates *pump)
{
  if (variables->wateringMode != green)
  {
    int currentTime = hour() * 3600 + minute() * 60 + second();
    if (variables->wateringToOnFlag == 1)
    {
      int diff = 75 - dataStorage.groundHum;
      // variables->wateringDuration = ((diff % 50) * variables->baseWateringDuration) / 50;
      // variables->wateringDuration = 25;
      variables->wateringTimestamp = currentTime;
      // variables->wateringDuration
      variables->wateringToOnFlag = 0;
      variables->wateringFlag = 1;
      if (SHOW_DEBUG && SHOW_WATERING_PROCESS)
      {
        Serial.println("Difference: " + String(diff));
        Serial.println("Watering duration: " + String(variables->wateringDuration));
        Serial.println("Current time: " + String(variables->wateringTimestamp));
        Serial.println("WateringFlag to 1");
      }
    }
    if (variables->wateringFlag == 1)
    {
      Blynk.virtualWrite(WATERING_TIME_LEFT, variables->wateringTimestamp + variables->wateringDuration - currentTime);
    }
    if (currentTime - variables->wateringTimestamp <= variables->wateringDuration)
    {
      if (SHOW_DEBUG)
      {
        Serial.println("Current time:" + String(currentTime));
        Serial.println("Duration: " + String(variables->wateringDuration));
        Serial.println("Watering timestamp: " + String(variables->wateringTimestamp));
        Serial.println("Time left: " + String(variables->wateringTimestamp + variables->wateringDuration - currentTime));
      }
      if (variables->wateringFlag == 1)
      {
        pump->state = true;
        Blynk.virtualWrite(pump->virtualPin, pump->state);
        if (SHOW_DEBUG && SHOW_WATERING_PROCESS)
        {
          Serial.println("Pump to ON");
        }
      }
    }
    else
    {
      variables->wateringFlag = 0;
      Blynk.virtualWrite(WATERING_TIME_LEFT, 0);
      pump->state = false;
      Blynk.virtualWrite(pump->virtualPin, pump->state);
      if (SHOW_DEBUG && SHOW_WATERING_PROCESS)
      {
        Serial.println("Pump to OFF");
      }
    }
  }
}
void wateringWrapper()
{
  wateringModeControl(&autoVariables, &dataStorage);
  if (autoVariables.wateringAutoMode == 1)
  {
    watering(&autoVariables, &relays.pumpRelay);
  }
}

//------------------------------------------------------//
//-------------------ОТОБРАЖЕНИЕ------------------------//
//------------------------------------------------------//
// Функция инициализации LCD дисплея
void initLCD(LiquidCrystal_I2C *lcd_i2c)
{
  // Символ полностью заполненного квадрата
  byte fiveCols[] = {
      0x1F,
      0x1F,
      0x1F,
      0x1F,
      0x1F,
      0x1F,
      0x1F,
      0x1F};
  byte fourCols[] = {
      0x1E,
      0x1E,
      0x1E,
      0x1E,
      0x1E,
      0x1E,
      0x1E,
      0x1E};
  byte threeCols[] = {
      0x1C,
      0x1C,
      0x1C,
      0x1C,
      0x1C,
      0x1C,
      0x1C,
      0x1C};
  byte twoCols[] = {
      0x18,
      0x18,
      0x18,
      0x18,
      0x18,
      0x18,
      0x18,
      0x18};
  byte oneCol[] = {
      0x10,
      0x10,
      0x10,
      0x10,
      0x10,
      0x10,
      0x10,
      0x10};
  // Инициализируем LCD дисплей
  lcd_i2c->init();
  // Включаем подсветку на дисплее
  lcd_i2c->backlight();
  lcd_i2c->createChar(1, oneCol);
  lcd_i2c->createChar(2, twoCols);
  lcd_i2c->createChar(3, threeCols);
  lcd_i2c->createChar(4, fourCols);
  lcd_i2c->createChar(5, fiveCols);
}
// Функция вывода информации о грядке на дисплей
// int screen = 0;
void showSensorsLCD(sensorsData *data, LiquidCrystal_I2C *lcd_i2c, autoModeVaruables *variables, relaysArray *relays)
{
  int static screen = 0;
  if (SHOW_DEBUG)
  {
    Serial.println("Pushing LCD");
  }
  if (variables->wateringFlag == 1)
  {
    
    int currentTime = hour() * 3600 + minute() * 60 + second();
    int timeLeft = variables->wateringTimestamp + variables->wateringDuration - currentTime;
    // Длина шкалы в процентах. Удобно что она совпадает по длине с количеством пикселей дисплея.
    int length = 100 - int(float(timeLeft)/float(variables->wateringDuration)*100);
    int fullCells = length / 5;
    int lastCell = length % 5;

    lcd_i2c->clear();
    lcd_i2c->setCursor(3, 0);
    lcd_i2c->print("Watering now");
    lcd_i2c->setCursor(0, 1);
    lcd_i2c->print("Duration  : " + String(variables->wateringDuration) + " s");
    lcd_i2c->setCursor(0,2);
    lcd_i2c->print("Time left : " + String(timeLeft) + " s");

    // Serial.println("//-----------------------------------//");
    // Serial.println("Watering");
    // Serial.println();
    // Serial.println("Duration  : " + String(variables->wateringDuration));
    // Serial.println("Full cells: " + String(fullCells));
    // Serial.println("Last cell : " + String(lastCell));
    // Serial.println("Length    : " + String(length));
    // Serial.println("//-----------------------------------//");

    // Вывод прогресс бара
    lcd_i2c->setCursor(0,3);
    for (int i =0; i < fullCells; i++){
      lcd_i2c->write(5);
    }
    if (lastCell != 0){
      lcd_i2c->write(lastCell);
    }
  }
  else
  {
    if (screen == 0)
    {
      // Serial.println("Screen 1");
      lcd_i2c->clear();
      lcd_i2c->setCursor(0, 0);
      lcd_i2c->print("Soil hum  : " + String(data->groundHum) + "%");
      lcd_i2c->setCursor(0, 1);
      lcd_i2c->print("Soil temp : " + String(data->groundTemp) + "C");
      lcd_i2c->setCursor(0, 2);
      lcd_i2c->print("Air hum   : " + String(data->airHum) + "%");
      lcd_i2c->setCursor(0, 3);
      lcd_i2c->print("Air temp  : " + String(data->airTemp) + "C");
      screen++;
      // worked = true;
    }
    else if (screen == 1)
    {
      // Serial.println("Screen 2");
      lcd_i2c->clear();
      lcd_i2c->setCursor(0, 0);
      lcd_i2c->print("Air pressure: ");
      lcd_i2c->setCursor(0, 1);
      lcd_i2c->print("(pa)  : " + String(data->airPressure));
      // lcd_i2c->print(String(data->airPressure) + " Pa");
      lcd_i2c->setCursor(0, 2);
      lcd_i2c->print("Light level: ");
      lcd_i2c->setCursor(0, 3);
      lcd_i2c->print("(lux) : " + String(data->lightLevel));
      // lcd_i2c->print(String(data->lightLevel) + " lux");
      screen++;
    } else if (screen == 2){
      lcd_i2c->clear();
      lcd_i2c->setCursor(0, 0);
      lcd_i2c->print("Lamp");
      lcd_i2c->setCursor(0,1);
      lcd_i2c->print("status: "  + String((relays->lightRelay.state == 0) ? "OFF" : "ON"));
      lcd_i2c->setCursor(0,2);
      lcd_i2c->print("Pump");
      lcd_i2c->setCursor(0,3);
      lcd_i2c->print("status: " + String((relays->pumpRelay.state == 0) ? "OFF" : "ON"));
      screen = 0;
    }
  }
}

// Функция для BlynkTimer. Выводит данные на дисплей
void showSensorsLCDWrapper()
{
  showSensorsLCD(&dataStorage, &lcd, &autoVariables, &relays);
}

BlynkTimer showDataLCD;

void setup()
{
  Serial.begin(115200);
  // Выставляем режимы работы на пины
  pinMode(GROUND_HUM_SENSOR_PIN, INPUT);
  pinMode(motor1.bottomSensor, INPUT);
  pinMode(motor1.topSensor, INPUT);
  pinMode(motor1.directionPin, OUTPUT);
  pinMode(motor1.steppingPin, OUTPUT);
  pinMode(relays.lightRelay.pin, OUTPUT);
  pinMode(relays.pumpRelay.pin, OUTPUT);

  // Инициализируем EEPROM память с количеством выделенных ячеек равным EEPROM_CELLS_NUMBER
  EEPROM.begin(EEPROM_CELLS_NUMBER);
  // Восстанавливаем значения для реле и ...
  // Инициализация I2C шины
  // Первый аргумент - SDA
  // Второй аргумент - SCL
  Wire.begin();
  // Инициализация BME280
  if (!bme280.begin(0x76, &Wire))
  {
    if (SHOW_DEBUG)
    {
      Serial.println("Can't find valid BME280 sensor");
    }
  }
  // Инициализация DS18B20
  groundSensors.begin();
  Serial.print("Found ");
  Serial.print(groundSensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // Инициализация BH1750
  lightSensor.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  // Записываем стартовые значения в autoVariables
  fillAutoVariables(&autoVariables);

  if (USE_LCD)
  {
    initLCD(&lcd);
    // lcd.setCursor(2, 2);
    // lcd.print("test");
    showDataLCD.setInterval(1000L, showSensorsLCDWrapper);
  }
  if (USE_BLYNK)
  {
    Blynk.begin(auth, ssid, pass, IPAddress(192, 168, 1, 106), 8080);
    // Blynk.begin(auth, ssid, pass, "blynk8080.iota02.keenetic.link", 778);
    pushRelays.setInterval(500L, useRelaysCallback);
    transferData.setInterval(500L, blynkDataTransfer);
    autoLight.setInterval(200L, lightControlWrapper);
    autoWatering.setInterval(200L, wateringWrapper);
  }
  if (RECOVER_FROM_EEPROM)
  {
    recoverEEPROM(&relays, &autoVariables);
  }
  setSyncInterval(10 * 60);
}

void loop()
{
  if (USE_BLYNK)
  {
    Blynk.run();
    if (SEND_SENSORS_DATA)
    {
      transferData.run();
    }
    if (PUSH_RELAYS)
    {
      pushRelays.run();
    }
    if (LIGHT_AUTO)
    {
      autoLight.run();
    }
    if (WATER_AUTO)
    {
      autoWatering.run();
    }
  }
  if (USE_LCD)
  {
    showDataLCD.run();
  }

  if (SHOW_DEBUG)
  {
    if (TEST_GET_BME)
    {
      getBME(&dataStorage, &bme280);
      Serial.println("Temperature: " + String(dataStorage.airTemp));
      Serial.println("Humidity: " + String(dataStorage.airHum));
    }
    if (TEST_GROUND_HUM)
    {
      getGroundHumidity(&dataStorage, A0);
      Serial.println("Ground humidity: " + String(dataStorage.groundHum));
    }
    if (TEST_LIGHT)
    {
      getLightLevel(&dataStorage, &lightSensor);
      Serial.println("Light level: " + String(dataStorage.lightLevel));
    }
    if (TEST_GROUND_TEMP)
    {
      getGroundTemperature(&dataStorage, &groundSensors);
      Serial.println("Ground temperature: " + String(dataStorage.groundTemp));
    }

    delay(1000);
  }
}