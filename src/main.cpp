#include <Arduino.h>
#include "TCA9548.h"
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include "DS18B20.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

#include <INA226_WE.h>
#include <DFRobot_ADS1115.h>

DFRobot_ADS1115 ads(&Wire);

#define I2C1_ADDRESS 0x41
#define I2C2_ADDRESS 0x44
#define I2C3_ADDRESS 0x40
#define I2C4_ADDRESS 0x45
INA226_WE ina226_1 = INA226_WE(I2C1_ADDRESS);
INA226_WE ina226_2 = INA226_WE(I2C2_ADDRESS);
INA226_WE ina226_3 = INA226_WE(I2C3_ADDRESS);
INA226_WE ina226_4 = INA226_WE(I2C4_ADDRESS);

#define RELAY_LITHIUM 12
#define RELAY_LIFEPO4 16
#define RELAY_ADAPTER 14
#define RELAY_4 13

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Daibangto";
char pass[] = "987654321";
/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
/* Fill in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL6tRiUPZuB"
#define BLYNK_TEMPLATE_NAME "battery monitor"
#define BLYNK_AUTH_TOKEN "3QprrQSsykyo5KrCPwak-6lwncmgzKjH"

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

#define NUMFLAKES 10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

WidgetTerminal terminal_tx(V1);
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);

#define PIN_ON 1
#define PIN_OFF 0
#define LITHIUM_LOW 20.4 // mV
#define LIFEPO4_LOW 23.2 // mV
#define INA229_TIME 5000
#define OLED_REFRESH_TIME 1000
#define CONTROL_TIME 3000
#define CHARGE_TIME 10800000

static unsigned long ina_time = 0;
static unsigned long oled_update_time = 0;
static unsigned long control_time = 0;
static unsigned long charge_time = 0; // 3h 10800s

enum sys_state
{
  CHANNEL_LITHIUM = 0,
  CHANNEL_LIFEPO4,
  CHANNEL_RELAY_220,
  CHANNEL_4,
  CHANNEL_5,
  CHANNEL_6,
  CHANNEL_7,
  CHANNEL_8
};

typedef struct data_t
{
  float temp;
  float charge_vol;
  float charge_current;
  float lithium_vol;
  float lithium_current;
  float lifepo4_vol;
  float lifepo4_current;

  bool adapter_state;
  bool lithium_state;
  bool lifepo4_state;

  bool lithium_relay;
  bool lifepo4_relay;
  bool adapter_relay;
} data_t;

struct data_t nlmt;
char array[8];
static void memset_array(void)
{
  array[0] = 0x0;
  array[1] = 0x0;
  array[2] = 0x0;
  array[3] = 0x0;
  array[4] = 0x0;
  array[5] = 0x0;
  array[6] = 0x0;
  array[7] = 0x0;
}
static void ds18b20_get(void)
{
  sensor.requestTemperatures();
  nlmt.temp = sensor.getTempC();
  if (nlmt.temp < 0.0)
    nlmt.temp = 0.0;
  Serial.print("\nTemperatures: ");
  Serial.print(nlmt.temp);
  Serial.println("oC");
}

static void lithium_get(void)
{
  nlmt.lithium_vol = ina226_1.getBusVoltage_V() + (float)(ina226_1.getShuntVoltage_mV() / 1000); // mV
  if (nlmt.lithium_vol < 0.0)
    nlmt.lithium_vol = 0.0;
  nlmt.lithium_current = ina226_1.getCurrent_mA() / 1000 + ina226_2.getCurrent_mA() / 1000;
  if (nlmt.lithium_current < 0.0)
    nlmt.lithium_current = 0.0;
  Serial.print("\nLithium: vol: ");
  Serial.print(nlmt.lithium_vol);
  Serial.print("V - Current: ");
  Serial.print(nlmt.lithium_current);
  Serial.println("A");
}

static void lifepo4_get(void)
{
  nlmt.lifepo4_vol = ina226_3.getBusVoltage_V() + (float)(ina226_3.getShuntVoltage_mV() / 1000); // mV
  if (nlmt.lifepo4_vol < 0.0)
    nlmt.lifepo4_vol = 0.0;
  nlmt.lifepo4_current = ina226_3.getCurrent_mA() / 1000 + ina226_4.getCurrent_mA() / 1000;
  if (nlmt.lifepo4_current < 0.0)
    nlmt.lifepo4_current = 0.0;
  Serial.print("\nLifepo4: vol: ");
  Serial.print(nlmt.lifepo4_vol);
  Serial.print("V - Current: ");
  Serial.print(nlmt.lifepo4_current);
  Serial.println("A");
}

static void solar_get_volt(void)
{
  int16_t rawADC = 0;

  rawADC = abs(ads.comparatorVoltage(01));
  nlmt.charge_vol = (float)(((float)rawADC * 40) / 4096);
  if (nlmt.charge_vol < 0.0)
    nlmt.charge_vol = 0.0;

  Serial.print("\nCharge: vol: ");
  Serial.println(nlmt.charge_vol);
  ads.setGain(eGAIN_SIXTEEN);
}

static void solar_get_current(void)
{
  int16_t rawADC = abs(ads.comparatorVoltage(23)); // voltage
  Serial.print("comparatorVoltage: ");
  Serial.println(rawADC);
  nlmt.charge_current = (float)(((float)rawADC * 19) / 1000); // Convert raw ADC value to voltage

  Serial.print("V - Current: ");
  Serial.print(nlmt.charge_current);
  Serial.println("A");
  ads.setGain(eGAIN_ONE);
}

static void oled_update(void)
{
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();

  if (nlmt.adapter_relay == false)
  {
    // draw solar
    // ngang
    u8g2.drawHLine(15, 0, 31);
    u8g2.drawHLine(9, 7, 30);
    u8g2.drawHLine(1, 15, 30);
    // doc
    u8g2.drawLine(1, 15, 15, 1);
    u8g2.drawLine(15, 15, 30, 1);
    u8g2.drawLine(31, 15, 45, 1);
    // ket noi
    u8g2.drawVLine(15, 15, 5);
  }
  else
  {
    // draw Adapter
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawFrame(0, 48, 45, 13);
    u8g2.setCursor(10, 58);
    u8g2.drawVLine(15, 43, 5);
    u8g2.print("220v");
  }

  if (nlmt.lithium_relay == true)
  {
    // draw ra nguon pin nao
    // lithium on
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.drawHLine(123, 32, 5);
    u8g2.drawVLine(123, 16, 16);
    u8g2.drawHLine(110, 16, 13);
    u8g2.setCursor(110, 10);
    u8g2.print("LI");
  }
  else
  {
    // lifepo4 on
    u8g2.drawHLine(123, 32, 5);
    u8g2.drawVLine(123, 32, 16);
    u8g2.drawHLine(110, 48, 13);
    u8g2.setCursor(110, 58);
    u8g2.print("LE");
  }

  // draw dien ap vao
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setCursor(1, 30);
  memset_array();
  sprintf(array, "%.1fV", nlmt.charge_vol);
  u8g2.print(array);
  u8g2.setCursor(1, 40);
  memset_array();
  sprintf(array, "%.1fA", nlmt.charge_current);
  u8g2.print(array);

  // draw Chia nguon
  u8g2.drawHLine(45, 32, 10);
  u8g2.drawVLine(55, 16, 16);
  u8g2.drawVLine(55, 32, 16);
  u8g2.drawHLine(55, 16, 10);
  u8g2.drawHLine(55, 48, 10);

  // draw dien ap pin
  // lithium
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.setCursor(74, 13);
  memset_array();
  sprintf(array, "%.1fV", nlmt.lithium_vol);
  u8g2.print(array);
  u8g2.setCursor(74, 23);
  memset_array();
  sprintf(array, "%.1fA", nlmt.lithium_current);
  u8g2.print(array);
  // lifepo4
  u8g2.setCursor(74, 48);
  memset_array();
  sprintf(array, "%.1fV", nlmt.lifepo4_vol);
  u8g2.print(array);
  u8g2.setCursor(74, 58);
  memset_array();
  sprintf(array, "%.1fA", nlmt.lifepo4_current);
  u8g2.print(array);

  u8g2.sendBuffer();
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    // will pause Zero, Leonardo, etc until serial console opens
    delay(1);
  }
  pinMode(RELAY_LITHIUM, OUTPUT);
  pinMode(RELAY_LIFEPO4, OUTPUT);
  pinMode(RELAY_ADAPTER, OUTPUT);
  // pinMode(RELAY_4, OUTPUT);
  digitalWrite(RELAY_LITHIUM, HIGH);
  digitalWrite(RELAY_LIFEPO4, HIGH);
  digitalWrite(RELAY_ADAPTER, HIGH);
  // digitalWrite(RELAY_4, HIGH);

  Wire.begin();
  Wire.setClock(400000);
  delay(1);
  if (!ina226_1.init())
  {
    Serial.println("Failed to init INA226 1. Check your wiring.");
    while (1)
    {
    }
  }
  ina226_1.waitUntilConversionCompleted(); // if you comment this line the first data might be zero

  if (!ina226_2.init())
  {
    Serial.println("Failed to init INA226 2. Check your wiring.");
    while (1)
    {
    }
  }
  ina226_2.waitUntilConversionCompleted(); // if you comment this line the first data might be zero

  if (!ina226_3.init())
  {
    Serial.println("Failed to init INA226 3. Check your wiring.");
    while (1)
    {
    }
  }
  ina226_3.waitUntilConversionCompleted(); // if you comment this line the first data might be zero

  if (!ina226_4.init())
  {
    Serial.println("Failed to init INA226 4. Check your wiring.");
    while (1)
    {
    }
  }
  ina226_4.waitUntilConversionCompleted(); // if you comment this line the first data might be zero

  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS0); // 0x48
  ads.setGain(eGAIN_ONE);                    // 2x gain
  ads.setMode(eMODE_SINGLE);                 // single-shot mode
  ads.setRate(eRATE_860);                    // 128SPS (default)
  ads.setOSMode(eOSMODE_SINGLE);             // Start a single-conversion (default)
  ads.setCompMode(eCOMPMODE_TRAD);           // Traditional comparator with hysteresis (default)
  ads.setCompPol(eCOMPPOL_LOW);              // Comparator polarity: Active low (default)
  ads.setCompLat(eCOMPLAT_LATCH);            // Latching comparator
  ads.setCompQue(eCOMPQUE_ONE);              // Comparator queue: Assert after one conversion
  // ADC Range: 2x gain (1 bit = 0.0625mV)");
  // Serial.println("Comparator High Threshold: 32000 (2.000V)");
  ads.setHighThreshold(32000);
  ads.init();

  u8g2.begin();

  sensor.begin();
  //  arbitrary number for the demo
  sensor.setOffset(0.25);
  solar_get_volt();
  ds18b20_get();
  lithium_get();
  lifepo4_get();
  solar_get_current();
  oled_update();

  unsigned long current = millis();
  ina_time = current + INA229_TIME;
  control_time = current + CONTROL_TIME;
  oled_update_time = current + OLED_REFRESH_TIME;

  nlmt.lithium_state = true;
  nlmt.lifepo4_state = true;
  nlmt.adapter_state = true;
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

BLYNK_WRITE(V1)
{
  // Called when the datastream V1 value changes

  // Assign incoming value from pin V1 to a variable
  // String pinValue = param.asStr()
  String pinValue = param.asString();

  Serial.print("V1: ");
  Serial.print(pinValue);
  Serial.println("");
  if (pinValue == "charge on")
  {
    Serial.println("Force turn on adapter charge");
  }
  else if (pinValue == "charge off")
  {
    Serial.println("Force turn off adapter charge");
  }
}

void loop()
{
  unsigned long current = millis();
  if (current > ina_time)
  {
    ina_time = current + INA229_TIME;
    solar_get_volt();
    ds18b20_get();
    lithium_get();
    lifepo4_get();
    solar_get_current();
    char buffer[256] = {0x0};
    sprintf(buffer, "Charge:[%.2fV-%.2fA] Lithium:[%.2fV-%.2fA] Lifepo4:[%.2fV-%.2fA] T:%.2foC\n",
            nlmt.charge_vol, nlmt.charge_current,
            nlmt.lithium_vol, nlmt.lithium_current,
            nlmt.lifepo4_vol, nlmt.lifepo4_current,
            nlmt.temp);
    Blynk.virtualWrite(V1, buffer);
  }

  if (current > control_time)
  {
    control_time = current + CONTROL_TIME;
    if ((nlmt.lithium_vol < LITHIUM_LOW) && (nlmt.lithium_state == true))
    {
      nlmt.lithium_state = false;
    }
    else if ((nlmt.lithium_vol > (LITHIUM_LOW + 2.0)) && (nlmt.lithium_state == false))
    {
      nlmt.lithium_state = true;
    }

    if ((nlmt.lifepo4_vol < LIFEPO4_LOW) && (nlmt.lifepo4_state == true))
    {
      nlmt.lifepo4_state = false;
    }
    else if ((nlmt.lifepo4_vol > (LIFEPO4_LOW + 2.0)) && (nlmt.lifepo4_state == false))
    {
      nlmt.lifepo4_state = true;
    }

    if ((nlmt.lithium_state == true) && (nlmt.lifepo4_state == true))
    {
      nlmt.lithium_relay = true;
      nlmt.lifepo4_relay = false;
      nlmt.adapter_relay = false;
      Serial.println("run 1");
    }
    else if ((nlmt.lithium_state == false) && (nlmt.lifepo4_state == true))
    {
      nlmt.lithium_relay = false;
      nlmt.lifepo4_relay = true;
      nlmt.adapter_relay = false;
      Serial.println("run 2");
    }

    else if ((nlmt.lithium_state == true) && (nlmt.lifepo4_state == false))
    {
      nlmt.lithium_relay = true;
      nlmt.lifepo4_relay = false;
      nlmt.adapter_relay = false;
      Serial.println("run 3");
    }
    else if ((nlmt.lithium_state == false) && (nlmt.lifepo4_state == false))
    {
      nlmt.lithium_relay = true;
      nlmt.lifepo4_relay = false;
      nlmt.adapter_relay = true;
      charge_time = current + CHARGE_TIME;
      Serial.println("run 4");
    }

    if (nlmt.lithium_relay == true)
    {
      digitalWrite(RELAY_LITHIUM, LOW);
      Serial.println("RELAY_LITHIUM ON");
    }
    else
    {
      digitalWrite(RELAY_LITHIUM, HIGH);
      Serial.println("RELAY_LITHIUM OFF");
    }

    if (nlmt.lifepo4_relay == true)
    {
      digitalWrite(RELAY_LIFEPO4, LOW);
      Serial.println("RELAY_LIFEPO4 ON");
    }
    else
    {
      digitalWrite(RELAY_LIFEPO4, HIGH);
      Serial.println("RELAY_LIFEPO4 OFF");
    }

    if (nlmt.adapter_relay == true)
    {
      digitalWrite(RELAY_ADAPTER, LOW);
      Serial.println("RELAY_ADAPTER ON");
    }
    else if (current > charge_time)
    {
      nlmt.adapter_relay = false;
      charge_time = 0;
      digitalWrite(RELAY_ADAPTER, HIGH);
      Serial.println("RELAY_ADAPTER OFF");
    }
  }

  if (current > oled_update_time)
  {
    oled_update_time = current + OLED_REFRESH_TIME;
    oled_update();
  }

  Blynk.run();
}