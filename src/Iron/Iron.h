#ifndef Iron_H
#define Iron_H

#include <Arduino.h>
#include <math.h>
#include <Thermocouple.h>
#include <QuickPID.h>
#include <sTune.h>

#define TEMP_SELECT_STEP 10

#define THERMISTOR_RES 10000.0f
#define THERMISTOR_FIXED_RES THERMISTOR_RES
#define THERMISTOR_K 298.15f
#define THERMISTOR_BETA 3950.0f

#define TEMP_MEASURE_SAMPLES 30
#define TEMP_MEASURE_SAMPLES_TO_SKIP 5
#define TEMP_MEASURE_SAMPLE_TIME_US 5

#define PID_SAMPLE_TIME_US 100000

#define PID_TUNING_OUTPUT_STEP 100
#define PID_TUNING_TEST_TIME_SEC 300
#define PID_TUNING_SETTLE_TIME_SEC 10
#define PID_TUNING_SAMPLES 300

#define HEATER_CHECK_INTERVAL_SEC 5000
#define HEATER_CHECK_MIN_TEMP_RIZE 2.0f

enum Status
{
    IDLE,
    HEATING,
    TUNING,
    FAULT,
    SLEEP
};

struct Settings
{
    float selectedTemp = 200.0f;
    float kp = 2.0f;
    float ki = 0.05f;
    float kd = 10.0f;
};

class Iron
{
private:
    AbstractThermocouple *tcType;
    QuickPID *pid;
    sTune *tuner;

    uint8_t heaterOutputPin;
    uint8_t tempSensorPin;

    float heaterOutput;
    bool outputReversed = true;
    int minOutput;
    int maxOutput;

    float currentTemp;
    float minTemp;
    float maxTemp;

    float selectedPreSleepTemp;

    float opAmpZeroOutputV = 0.0f;
    float opAmpGain = 100.0f;
    float vRef = 5.0f;
    float coldJunctionTemp = 25.0f;
    long bandGapVoltage = 1100L;
    float userMeasuredVRef = 5.0f;

    unsigned long heaterCheckStart = 0;
    float heaterCheckTemp = 0;
    bool heaterCheckActive = false;

    Settings settings;
    Status status = IDLE;

public:
    Iron(
        uint8_t heaterOutputPin,
        uint8_t tempSensorPin,
        AbstractThermocouple *tcType,
        int minOutput,
        int maxOutput,
        int minTemp,
        int maxTemp);

    static long readBandGapADC();
    static float readVccVolts(long bandGapVoltage);
    static float readVoltage(uint8_t pin, int samples, byte skip, unsigned int sampleTimeUs, float vRef);
    static float readColdJunctionTemp(uint8_t pin, float vRef, float fixedRes, float thermRes);
    static void setPWMPrescaler(uint8_t pin, uint16_t prescale);
    static bool setADCPrescaler(uint8_t prescaler);

    void init(float opAmpZeroOutputV, float opAmpGain, float userMeasuredVRef);
    void run(float cjt);
    void runPID();
    void runPIDTuning();
    void tune();

    Status getStatus();
    void setStatus(Status status);
    void heaterCheck();

    float getSelectedTemp();
    void setSelectedTemp(float tempC);
    void increaseSelectedTemp();
    void decreaseSelectedTemp();

    void readTemp();
    float getCurrentTemp();

    float getColdJunctionTemp();
    void setColdJunctionTemp(float cjt);

    void heaterUpdate();
    void heaterOff();
    void heaterSleep();
    void heaterWake();

    void toggle();

    float getHeaterOutput();
    float getHeaterOutputPct();

    float getVRef();
    void setVRef(float vRef);

    void saveSettings();
    void loadSettings();
};
#endif