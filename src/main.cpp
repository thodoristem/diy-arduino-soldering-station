#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <BigNumbers_I2C.h>
#include <Iron/Iron.h>
#include <OneButton.h>

#define MIN_HEATER_OUTPUT 0
#define MAX_HEATER_OUTPUT 255
#define MIN_SELECTABLE_TEMP 100
#define MAX_SELECTABLE_TEMP 400

#define HEATER_A_PIN 9
#define TC_A_PIN A0

#define THERMISTOR_PIN A1

#define TEMP_UP_BTN 4
#define TEMP_DOWN_BTN 5
#define FUNCTION_BTN 6
#define BTN_PRESS_MS 200
#define BTN_LONG_PRESS_MS 5000
#define BTN_LONG_PRESS_INTERVAL_MS 200

#define LCD_INTERVAL 80
#define DEFAULT_SET_POINT 350.0f

LiquidCrystal_I2C lcd(0x27, 20, 4);
BigNumbers_I2C bigNum(&lcd);

OneButton tempUpButton;
OneButton tempDownButton;
OneButton functionButton;

TypeE thermocouple;
Iron iron(HEATER_A_PIN,
          TC_A_PIN,
          &thermocouple,
          MIN_HEATER_OUTPUT,
          MAX_HEATER_OUTPUT,
          MIN_SELECTABLE_TEMP,
          MAX_SELECTABLE_TEMP);

char printBuffer[10];

float vRef;
float coldJunctionTemp;

void drawProgressBar(int pct, int x, int y, int width)
{
    pct = constrain(pct, 0, 100);
    int filled = pct * width / 100;
    lcd.setCursor(x, y);
    for (int i = 0; i < width; i++)
    {
        if (i < filled)
        {
            lcd.write(byte(255));
        }
        else
        {
            lcd.print(' ');
        }
    }
}

void printIronInfo()
{
    lcd.setCursor(4, 0);
    switch (iron.getStatus())
    {
    case IDLE:
        lcd.print("    IDLE    ");
        break;
    case HEATING:
        lcd.print(" HEATER ON! ");
        break;
    case FAULT:
        lcd.print("   ERROR!   ");
        break;
    case TUNING:
        lcd.print(" PID TUNING ");
        break;
    }

    bigNum.displayLargeInt(iron.getCurrentTemp(), 5, 1, 3, true); // Display the temperature in large font
    lcd.setCursor(14, 1);
    lcd.print((char)223);
    lcd.setCursor(14, 2);
    lcd.print("C");

    lcd.setCursor(16, 0);
    sprintf(printBuffer, "%3dC", (int)iron.getSelectedTemp());
    lcd.print(printBuffer);

    lcd.setCursor(16, 3);
    sprintf(printBuffer, "%3d%%", (int)iron.getHeaterOutputPct());
    lcd.print(printBuffer);

    drawProgressBar((int)iron.getHeaterOutputPct(), 0, 3, 16);
}

void printCommonInfo()
{
    lcd.setCursor(0, 0);
    sprintf(printBuffer, "%2d%cC", (int)coldJunctionTemp, (char)223);
    lcd.print(printBuffer);

    lcd.setCursor(0, 1);
    lcd.print(vRef, 1);
    lcd.print("V");
}

void tempUp()
{
    iron.increaseSelectedTemp();
}

void tempDown()
{
    iron.decreaseSelectedTemp();
}

void tune()
{
    iron.tune();
}

void toggleIron()
{
    if (iron.getStatus() == HEATING)
    {
        iron.setStatus(IDLE);
    }
    else
    {
        iron.setStatus(HEATING);
    }
}

void setup()
{
    Serial.begin(9600);

    lcd.init();
    lcd.backlight();
    lcd.clear();

    bigNum.begin();

    tempUpButton.setup(TEMP_UP_BTN, INPUT_PULLUP, true);
    tempUpButton.setPressMs(BTN_PRESS_MS);
    tempUpButton.setLongPressIntervalMs(BTN_LONG_PRESS_INTERVAL_MS);
    tempUpButton.attachClick(tempUp);
    tempUpButton.attachDuringLongPress(tempUp);

    tempDownButton.setup(TEMP_DOWN_BTN, INPUT_PULLUP, true);
    tempDownButton.setPressMs(BTN_PRESS_MS);
    tempDownButton.setLongPressIntervalMs(BTN_LONG_PRESS_INTERVAL_MS);
    tempDownButton.attachClick(tempDown);
    tempDownButton.attachDuringLongPress(tempDown);

    functionButton.setup(FUNCTION_BTN, INPUT_PULLUP, true);
    functionButton.setPressMs(BTN_LONG_PRESS_MS);
    functionButton.attachClick(toggleIron);
    functionButton.attachLongPressStart(tune);

    vRef = Iron::readVccVolts();

    iron.init();
    iron.setSelectedTemp(300);
    Iron::setPWMPrescaler(HEATER_A_PIN, 8);
}

unsigned long previousMillis = 0;

void loop()
{
    unsigned long currentMillis = millis();

    coldJunctionTemp = Iron::readColdJunctionTemp(THERMISTOR_PIN, vRef, THERMISTOR_FIXED_RES, THERMISTOR_RES);

    iron.run(coldJunctionTemp, vRef);

    if (iron.getStatus() != TUNING)
    {
        tempUpButton.tick();
        tempDownButton.tick();
        functionButton.tick();
    }

    if (currentMillis - previousMillis >= LCD_INTERVAL)
    {
        previousMillis = currentMillis;
        printIronInfo();
        printCommonInfo();
    }
}