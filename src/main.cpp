#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <BigNumbers_I2C.h>
#include <Iron/Iron.h>
#include <OneButton.h>

#define OPAMP_ZERO_OUTPUT_V 0.058f
#define OPAMP_GAIN 140.0f
#define USER_MEASURED_VREF_MV 4898.0f

#define HEATER_A_PIN 9
#define TC_A_PIN A0
#define MIN_HEATER_OUTPUT 0
#define MAX_HEATER_OUTPUT 250
#define MIN_SELECTABLE_TEMP 100
#define MAX_SELECTABLE_TEMP 400

#define THERMISTOR_PIN A1

#define TEMP_UP_BTN 4
#define TEMP_DOWN_BTN 5
#define FUNCTION_BTN 6

#define BTN_DEBOUNCE_MS 0
#define BTN_CLICK_MS 0
#define BTN_PRESS_MS 300
#define BTN_LONG_PRESS_INTERVAL_MS 200
#define BTN_VERY_LONG_PRESS_MS 3000

#define BUZZER_PIN 2

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
          BUZZER_PIN,
          &thermocouple,
          MIN_HEATER_OUTPUT,
          MAX_HEATER_OUTPUT,
          MIN_SELECTABLE_TEMP,
          MAX_SELECTABLE_TEMP);

char printBuffer[10];

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

void printCentered(int row, const char *text)
{
    int col = (20 - strlen(text)) / 2;
    lcd.setCursor(col, row);
    lcd.print(text);
}

void printIntro()
{
    printCentered(0, "DIY");
    printCentered(1, "SOLDERING");
    printCentered(2, "STATION");
    printCentered(3, "by Thodoris");
    delay(2000);
    lcd.clear();
}

void printIronInfo()
{
    lcd.setCursor(4, 0);
    switch (iron.getStatus())
    {
    case Iron::IDLE:
        lcd.print("    IDLE    ");
        break;
    case Iron::HEATING:
        lcd.print(" HEATER ON! ");
        break;
    case Iron::FAULT:
        lcd.print("   ERROR!   ");
        break;
    case Iron::TUNING:
        lcd.print(" PID TUNING ");
        break;
    case Iron::SLEEP:
        lcd.print("  STAND BY  ");
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

    // lcd.setCursor(0, 1);
    // lcd.print(iron.getVRef(), 3);
    // lcd.print("V");
}

void tempUp()
{
    iron.increaseSelectedTemp();
}

void tempDown()
{
    iron.decreaseSelectedTemp();
}

void tunePID()
{
    iron.tune();
}

void toggleIron()
{
    iron.toggle();
}

void setup()
{
    Serial.begin(115200);

    lcd.init();
    lcd.backlight();
    lcd.clear();
    bigNum.begin();

    printIntro();

    tempUpButton.setup(TEMP_UP_BTN, INPUT, true);
    tempUpButton.setClickMs(BTN_CLICK_MS);
    tempUpButton.setDebounceMs(BTN_DEBOUNCE_MS);
    tempUpButton.setPressMs(BTN_PRESS_MS);
    tempUpButton.setLongPressIntervalMs(BTN_LONG_PRESS_INTERVAL_MS);
    tempUpButton.attachClick(tempUp);
    tempUpButton.attachDuringLongPress(tempUp);

    tempDownButton.setup(TEMP_DOWN_BTN, INPUT, true);
    tempDownButton.setClickMs(BTN_CLICK_MS);
    tempDownButton.setDebounceMs(BTN_DEBOUNCE_MS);
    tempDownButton.setPressMs(BTN_PRESS_MS);
    tempDownButton.setLongPressIntervalMs(BTN_LONG_PRESS_INTERVAL_MS);
    tempDownButton.attachClick(tempDown);
    tempDownButton.attachDuringLongPress(tempDown);

    functionButton.setup(FUNCTION_BTN, INPUT, true);
    functionButton.setDebounceMs(BTN_DEBOUNCE_MS);
    functionButton.setPressMs(BTN_VERY_LONG_PRESS_MS);
    functionButton.setClickMs(BTN_CLICK_MS);
    functionButton.attachLongPressStart(tunePID);
    functionButton.attachClick(toggleIron);

    iron.init(OPAMP_ZERO_OUTPUT_V, OPAMP_GAIN, USER_MEASURED_VREF_MV);
    iron.setSelectedTemp(300);

    Iron::setPWMPrescaler(HEATER_A_PIN, 8);
    Iron::setADCPrescaler(32);
}

unsigned long previousMillis = 0;

void loop()
{
    tempUpButton.tick();
    tempDownButton.tick();
    functionButton.tick();

    unsigned long currentMillis = millis();

    coldJunctionTemp = Iron::readColdJunctionTemp(THERMISTOR_PIN, iron.getVRef(), THERMISTOR_FIXED_RES, THERMISTOR_RES);

    iron.run(coldJunctionTemp);

    if (currentMillis - previousMillis >= LCD_INTERVAL)
    {
        previousMillis = currentMillis;
        printIronInfo();
        printCommonInfo();
    }
}