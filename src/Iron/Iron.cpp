#include <Iron/Iron.h>
#include <EEPROM.h>

Iron::Iron(
    uint8_t heaterOutputPin,
    uint8_t tempSensorPin,
    AbstractThermocouple *tcType,
    int minOutput,
    int maxOutput,
    int minTemp,
    int maxTemp)
{
    this->heaterOutputPin = heaterOutputPin;
    this->tempSensorPin = tempSensorPin;
    this->tcType = tcType;
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
    this->minTemp = minTemp;
    this->maxTemp = maxTemp;

    Serial.print("KP:");
    Serial.println(this->settings.kp);
    Serial.print("KI:");
    Serial.println(this->settings.ki);
    Serial.print("KD:");
    Serial.println(this->settings.kd);

    Serial.println("Loaded saved settings");
    this->loadSettings();

    Serial.print("BandgapADC: ");
    Serial.println(Iron::readBandgapADC());
    Serial.print("KP:");
    Serial.println(this->settings.kp);
    Serial.print("KI:");
    Serial.println(this->settings.ki);
    Serial.print("KD:");
    Serial.println(this->settings.kd);

    // QuickPID
    this->pid = new QuickPID(
        &this->currentTemp,
        &this->heaterOutput,
        &this->settings.selectedTemp,
        this->settings.kp,
        this->settings.ki,
        this->settings.kd,
        QuickPID::pMode::pOnError,   /* pOnError, pOnMeas, pOnErrorMeas */
        QuickPID::dMode::dOnMeas,    /* dOnError, dOnMeas */
        QuickPID::iAwMode::iAwClamp, /* iAwCondition, iAwClamp, iAwOff */
        QuickPID::Action::direct);

    // sTune: open-loop step-response autotuner
    this->tuner = new sTune(
        &this->currentTemp,
        &this->heaterOutput,
        sTune::DampedOsc_PID,
        sTune::directIP,
        sTune::printSUMMARY);
}

void Iron::init()
{
    this->heaterOff();

    this->pid->SetMode(QuickPID::Control::automatic);
    this->pid->SetOutputLimits(this->minOutput, this->maxOutput);
    this->pid->SetSampleTimeUs(PID_SAMPLE_TIME_US);

    this->tuner->Configure(
        this->maxTemp,
        this->maxOutput,
        this->minOutput,
        PID_TUNING_OUTPUT_STEP,
        PID_TUNING_TEST_TIME_SEC,
        PID_TUNING_SETTLE_TIME_SEC,
        PID_TUNING_SAMPLES);

    this->tuner->SetEmergencyStop(this->maxTemp);

    delay(1000);
}

void Iron::run(float cjt, float vRef)
{
    this->coldJunctionTemp = cjt;
    this->vRef = vRef;

    if (this->getStatus() == TUNING)
    {
        this->runPIDTuning();
    }
    else
    {
        this->runPID();
    }
}

void Iron::runPID()
{
    this->readTemp();
    this->heaterCheck();

    if (this->status == HEATING || this->status == TUNING)
    {
        if (this->pid->Compute())
        {
            this->heaterOn();
        }
    }
    else
    {
        this->heaterOutput = 0;
        this->heaterOff();
    }
}

void Iron::runPIDTuning()
{
    switch (this->tuner->Run())
    {
    case this->tuner->sample: // active once per sample during test
        this->readTemp();
        this->tuner->plotter(this->getCurrentTemp(), this->getHeaterOutput(), this->getSelectedTemp(), 1, 3);
        break;
    case this->tuner->tunings:       // active just once when sTune is done
        this->tuner->GetAutoTunings( // sketch variables updated by sTune
            &this->settings.kp,
            &this->settings.ki,
            &this->settings.kd);
        this->pid->SetMode(QuickPID::Control::automatic); // the PID is turned on
        this->pid->SetTunings(                            // update PID with the new tunings
            this->settings.kp,
            this->settings.ki,
            this->settings.kd);
        this->setStatus(HEATING);
        this->saveSettings();
        break;
    case this->tuner->runPid: // active once per sample after tunings
        this->runPID();
        this->tuner->plotter(this->getCurrentTemp(), this->getHeaterOutput(), this->getSelectedTemp(), 1, 3);
        break;
    }
}

void Iron::tune()
{
    this->setStatus(TUNING);
}

Status Iron::getStatus()
{
    return this->status;
}

void Iron::setStatus(Status status)
{
    this->status = status;
}

void Iron::heaterCheck()
{
    if (this->getHeaterOutput() < HEATER_CHECK_PWM_THRESHOLD)
    {
        this->heaterCheckActive = false;
        return;
    }

    if (!this->heaterCheckActive)
    {
        this->heaterCheckActive = true;
        this->heaterCheckStart = millis();
        this->heaterCheckTemp = this->getCurrentTemp();
        return;
    }

    if (millis() - this->heaterCheckStart >= HEATER_CHECK_INTERVAL_SEC)
    {
        float tempDiff = this->getCurrentTemp() - this->heaterCheckTemp;

        if (tempDiff < HEATER_CHECK_MIN_TEMP_RIZE)
        {
            this->status = FAULT;
        }

        // Start a new observation window.
        this->heaterCheckStart = millis();
        this->heaterCheckTemp = this->getCurrentTemp();
    }
}

void Iron::heaterOn()
{
    if (this->outputReversed)
    {
        analogWrite(this->heaterOutputPin, (int)map(this->heaterOutput, 0, 255, 255, 0));
    }
    else
    {
        analogWrite(this->heaterOutputPin, (int)this->heaterOutput);
    }
}

void Iron::heaterOff()
{
    if (this->outputReversed)
    {
        analogWrite(this->heaterOutputPin, 255);
    }
    else
    {
        analogWrite(this->heaterOutputPin, 0);
    }
}

void Iron::heaterSleep()
{
    this->setSelectedTemp(this->minTemp);
}

void Iron::setPWMPrescaler(uint8_t pin, uint16_t prescale)
{
    byte mode;
    if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
    {
        switch (prescale)
        {
        case 1:
            mode = 0b001;
            break;
        case 8:
            mode = 0b010;
            break;
        case 64:
            mode = 0b011;
            break;
        case 256:
            mode = 0b100;
            break;
        case 1024:
            mode = 0b101;
            break;
        default:
            return;
        }
    }
    else if (pin == 3 || pin == 11)
    {
        switch (prescale)
        {
        case 1:
            mode = 0b001;
            break;
        case 8:
            mode = 0b010;
            break;
        case 32:
            mode = 0b011;
            break;
        case 64:
            mode = 0b100;
            break;
        case 128:
            mode = 0b101;
            break;
        case 256:
            mode = 0b110;
            break;
        case 1024:
            mode = 0b111;
            break;
        default:
            return;
        }
    }

    if (pin == 5 || pin == 6)
    {
        TCCR0B = (TCCR0B & 0b11111000) | mode;
    }
    else if (pin == 9 || pin == 10)
    {
        TCCR1B = (TCCR1B & 0b11111000) | mode;
    }
    else if (pin == 3 || pin == 11)
    {
        TCCR2B = (TCCR2B & 0b11111000) | mode;
    }
}

long Iron::readBandgapADC()
{
    // Measure internal bandgap against AVCC
    ADMUX = _BV(REFS0) |
            _BV(MUX3) |
            _BV(MUX2) |
            _BV(MUX1);

    delay(5);

    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC))
    {
    }

    // Read ADC result
    uint8_t low = ADCL;
    uint8_t high = ADCH;

    return (high << 8) | low;
}

float Iron::readVccVolts()
{
    // Select internal 1.1 V bandgap, measured against AVCC
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);

    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC))
    {
    }

    uint8_t low = ADCL;
    uint8_t high = ADCH;
    long adc = (high << 8) | low;

    if (adc == 0)
    {
        return 0;
    }

    // Replace 1100 with your calibrated bandgap voltage in mV
    return (1078L * 1023L / adc) / 1000.0f;
}

float Iron::readVoltage(uint8_t pin, int samples, unsigned int sampleTimeUs, float vRef)
{
    analogRead(pin);

    int total = 0;

    for (int i = 0; i < samples; i++)
    {
        delayMicroseconds(sampleTimeUs);
        total += analogRead(pin);
    }

    return total / TEMP_MEASURE_SAMPLES * vRef / 1023.0f;
}

float Iron::readColdJunctionTemp(byte pin, float vRef, float fixedRes, float thermRes)
{
    float volts = Iron::readVoltage(pin, TEMP_MEASURE_SAMPLES, TEMP_MEASURE_SAMPLE_TIME_US, vRef);

    if (volts <= 0.0f || volts >= vRef)
    {
        return NAN;
    }

    float resistance = fixedRes * volts / (vRef - volts);
    float invT = 1.0f / THERMISTOR_K + log(resistance / thermRes) / THERMISTOR_BETA;

    return (1.0f / invT - 273.15f);
}

void Iron::readTemp()
{
    this->heaterOff();
    float volts = Iron::readVoltage(
        this->tempSensorPin,
        TEMP_MEASURE_SAMPLES,
        TEMP_MEASURE_SAMPLE_TIME_US,
        this->vRef);
    this->heaterOn();

    // Convert amplifier output to thermocouple millivolts.
    float tcMilliVolts = (volts - OPAMP_ZERO_OUTPUT_V) * 1000.0f / OPAMP_GAIN;

    // Library applies cold-junction compensation and
    // converts thermocouple voltage to currentTemp.
    this->currentTemp = this->tcType->getTemperature(tcMilliVolts, this->coldJunctionTemp);
}

float Iron::getSelectedTemp()
{
    return this->settings.selectedTemp;
}

void Iron::setSelectedTemp(float tempC)
{
    if (tempC < this->minTemp)
    {
        this->settings.selectedTemp = this->minTemp;
    }
    else if (tempC > this->maxTemp)
    {
        this->settings.selectedTemp = this->maxTemp;
    }
    else
    {
        this->settings.selectedTemp = tempC;
    }
}

void Iron::increaseSelectedTemp()
{
    if (this->settings.selectedTemp < this->maxTemp)
    {
        this->settings.selectedTemp += TEMP_SELECT_STEP;
    }
    else
    {
        this->settings.selectedTemp = this->maxTemp;
    }
}

void Iron::decreaseSelectedTemp()
{
    if (this->settings.selectedTemp > this->minTemp)
    {
        this->settings.selectedTemp -= TEMP_SELECT_STEP;
    }
    else
    {
        this->settings.selectedTemp = this->minTemp;
    }
}

float Iron::getCurrentTemp()
{
    return this->currentTemp;
}

float Iron::getHeaterOutput()
{
    return this->heaterOutput;
}

float Iron::getHeaterOutputPct()
{
    return this->heaterOutput / this->maxOutput * 100;
}

float Iron::getColdJunctionTemp()
{
    return this->coldJunctionTemp;
}

void Iron::setColdJunctionTemp(float cjt)
{
    this->coldJunctionTemp = cjt;
}

float Iron::getVRef()
{
    return this->vRef;
}

void Iron::setVRef(float vRef)
{
    this->vRef = vRef;
}

void Iron::saveSettings()
{
    EEPROM.put(this->heaterOutputPin, this->settings);
}

void Iron::loadSettings()
{
    Settings temp;

    EEPROM.get(this->heaterOutputPin, temp);

    if (temp.valid)
    {
        this->settings = temp;
    }
}