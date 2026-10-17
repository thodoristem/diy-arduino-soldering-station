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

struct Settings
{
    float selectedTemp = 200.0f;
    float kp = 18.7727f;
    float ki = 0.0089f;
    float kd = 0.9862f;
};

class Iron
{

public:
    enum Status : uint8_t
    {
        IDLE,
        HEATING,
        TUNING,
        FAULT,
        SLEEP
    };

    enum SoundEvent : uint8_t
    {
        SOUND_NONE,
        SOUND_INIT,
        SOUND_TEMP_CHANGE,
        SOUND_TEMP_REACHED,
        SOUND_ERROR,
        SOUND_HEATER_ON,
        SOUND_HEATER_OFF,
        SOUND_TUNING
    };

private:
    AbstractThermocouple *tcType;
    QuickPID *pid;
    sTune *tuner;

    uint8_t heaterOutputPin;
    uint8_t tempSensorPin;
    uint8_t buzzerPin;

    float heaterOutput;
    bool outputReversed = true;
    int minOutput;
    int maxOutput;

    float currentTemp;
    float minTemp;
    float maxTemp;
    bool heaterTempReached = false;
    bool heaterTempReachedRev = true;

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
    Iron::Status status = IDLE;

public:
    Iron(
        uint8_t heaterOutputPin,
        uint8_t tempSensorPin,
        uint8_t buzzerPin,
        AbstractThermocouple *tcType,
        int minOutput,
        int maxOutput,
        int minTemp,
        int maxTemp);

    void init(float opAmpZeroOutputV, float opAmpGain, float userMeasuredVRef);
    void run(float cjt);
    void runPID();
    void runPIDTuning();
    void tune();
    void playTone(SoundEvent event);

    Status getStatus();
    void setStatus(Status status);
    void heaterCheck();
    void toggle();

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

    float getHeaterOutput();
    float getHeaterOutputPct();

    float getVRef();
    void setVRef(float vRef);

    void saveSettings();
    void loadSettings();

    static long readBandGapADC();
    static float readVccVolts(long bandGapVoltage);
    static float readVoltage(uint8_t pin, int samples, byte skip, unsigned int sampleTimeUs, float vRef);
    static float readColdJunctionTemp(uint8_t pin, float vRef, float fixedRes, float thermRes);
    static void setPWMPrescaler(uint8_t pin, uint16_t prescale);
    static bool setADCPrescaler(uint8_t prescaler);
};
#endif