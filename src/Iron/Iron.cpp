#include <Iron/Iron.h>
#include <EEPROM.h>

Iron::Iron(
    uint8_t heaterOutputPin,
    uint8_t tempSensorPin,
    uint8_t buzzerPin,
    AbstractThermocouple *tcType,
    int minOutput,
    int maxOutput,
    int minTemp,
    int maxTemp)
{
    this->heaterOutputPin = heaterOutputPin;
    this->tempSensorPin = tempSensorPin;
    this->buzzerPin = buzzerPin;
    this->tcType = tcType;
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
    this->minTemp = minTemp;
    this->maxTemp = maxTemp;
}

void Iron::init(float opAmpZeroOutputV, float opAmpGain, float userMeasuredVRefMv)
{
    Serial.println("###########################################");
    Serial.println("########## DIY Soldering Station ##########");
    Serial.println("###########################################");
    Serial.println();

    delay(1000);

    long bandGapADC = Iron::readBandGapADC();

    this->opAmpZeroOutputV = opAmpZeroOutputV;
    this->opAmpGain = opAmpGain;
    this->bandGapVoltage = (long)(userMeasuredVRefMv * bandGapADC / 1023);
    this->vRef = Iron::readVccVolts(this->bandGapVoltage);

    this->loadSettings();

    Serial.print("OpAmpGain:        ");
    Serial.println(this->opAmpGain);
    Serial.print("OpAmpZeroOutputV: ");
    Serial.println(this->opAmpZeroOutputV, 3);
    Serial.print("BandGapADC:       ");
    Serial.println(Iron::readBandGapADC());
    Serial.print("BandGapVoltage:   ");
    Serial.println(this->bandGapVoltage);
    Serial.print("VREF:             ");
    Serial.println(this->getVRef(), 3);
    Serial.print("KP:               ");
    Serial.println(this->settings.kp, 4);
    Serial.print("KI:               ");
    Serial.println(this->settings.ki, 4);
    Serial.print("KD:               ");
    Serial.println(this->settings.kd, 4);
    Serial.print("Min Temp:         ");
    Serial.println(this->minTemp);
    Serial.print("Max Temp:         ");
    Serial.println(this->maxTemp);

    this->heaterOff();
    readTemp();

    // QuickPID
    this->pid = new QuickPID(
        &this->currentTemp,
        &this->heaterOutput,
        &this->settings.selectedTemp,
        this->settings.kp,
        this->settings.ki,
        this->settings.kd,
        QuickPID::pMode::pOnError,
        QuickPID::dMode::dOnMeas,
        QuickPID::iAwMode::iAwClamp,
        QuickPID::Action::direct);

    this->pid->SetMode(QuickPID::Control::automatic);
    this->pid->SetOutputLimits(this->minOutput, this->maxOutput);
    this->pid->SetSampleTimeUs(PID_SAMPLE_TIME_US);

    this->tuner = new sTune(
        &this->currentTemp,
        &this->heaterOutput,
        sTune::NoOvershoot_PID,
        sTune::directIP,
        sTune::printALL);

    this->tuner->Configure(
        this->maxTemp - this->coldJunctionTemp,
        this->maxOutput,
        this->minOutput,
        this->maxOutput / 2,
        PID_TUNING_TEST_TIME_SEC,
        PID_TUNING_SETTLE_TIME_SEC,
        PID_TUNING_SAMPLES);

    this->tuner->SetEmergencyStop(this->maxTemp);

    this->playTone(SoundEvent::SOUND_INIT);
}

void Iron::run(float cjt)
{
    this->coldJunctionTemp = cjt;

    this->readTemp();
    this->heaterCheck();

    if (this->getStatus() == TUNING)
    {
        this->runPIDTuning();
    }
    else
    {
        if (!this->heaterTempReached && this->getCurrentTemp() >= this->getSelectedTemp())
        {
            this->playTone(SoundEvent::SOUND_TEMP_REACHED);
            this->heaterTempReached = true;
        }
        else if (!this->heaterTempReachedRev && this->getCurrentTemp() <= this->getSelectedTemp())
        {
            this->playTone(SoundEvent::SOUND_TEMP_REACHED);
            this->heaterTempReachedRev = true;
        }

        if (this->getCurrentTemp() < this->getSelectedTemp() - 10)
        {
            this->heaterTempReached = false;
        }
        else if (this->getCurrentTemp() > this->getSelectedTemp() + 10)
        {
            this->heaterTempReachedRev = false;
        }

        this->runPID();
    }
}

void Iron::runPID()
{
    if (this->status == HEATING || this->status == TUNING || this->status == SLEEP)
    {
        if (this->pid->Compute())
        {
            this->heaterUpdate();
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
        this->tuner->plotter(
            this->getCurrentTemp(),
            this->getHeaterOutput(),
            this->getSelectedTemp(), 1, 3);
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
        this->pid->Initialize();
        this->saveSettings();
        this->setStatus(HEATING);
        break;
    case this->tuner->runPid: // active once per sample after tunings
        this->runPID();
        this->tuner->plotter(
            this->getCurrentTemp(),
            this->getHeaterOutput(),
            this->getSelectedTemp(), 1, 3);
        break;
    }
}

void Iron::tune()
{
    this->setStatus(TUNING);
    this->pid->SetMode(QuickPID::Control::manual);
    Serial.println("\nPID Tuning started\n");
    this->playTone(SoundEvent::SOUND_TUNING);
}

void Iron::playTone(SoundEvent event)
{
    switch (event)
    {
    case SOUND_INIT:
        tone(this->buzzerPin, 800, 100);
        delay(150);
        tone(this->buzzerPin, 1200, 100);
        delay(150);
        tone(this->buzzerPin, 1600, 150);
        break;
    case SOUND_TEMP_CHANGE:
        tone(this->buzzerPin, 2200, 70);
        break;
    case SOUND_TEMP_REACHED:
        tone(this->buzzerPin, 1200, 100);
        delay(150);
        tone(this->buzzerPin, 1800, 180);
        break;
    case SOUND_ERROR:
        for (uint8_t i = 0; i < 3; i++)
        {
            tone(this->buzzerPin, 500, 180);
            delay(260);
        }
        break;
    case SOUND_HEATER_ON:
        tone(this->buzzerPin, 1600, 70);
        break;
    case SOUND_HEATER_OFF:
        tone(this->buzzerPin, 1000, 100);
        break;
    case SOUND_TUNING:
        tone(this->buzzerPin, 900, 80);
        delay(110);
        tone(this->buzzerPin, 1300, 80);
        delay(110);
        tone(this->buzzerPin, 1800, 120);
        break;
    case SOUND_NONE:
    default:
        noTone(this->buzzerPin);
        break;
    }
}

Iron::Status Iron::getStatus()
{
    return this->status;
}

void Iron::setStatus(Status status)
{
    this->status = status;
}

void Iron::toggle()
{
    switch (this->getStatus())
    {
    case HEATING:
        this->setStatus(IDLE);
        this->playTone(SoundEvent::SOUND_HEATER_OFF);
        break;
    case IDLE:
        this->setStatus(HEATING);
        this->playTone(SoundEvent::SOUND_HEATER_ON);
        break;
    case TUNING:
        this->setStatus(IDLE);
        this->pid->SetMode(QuickPID::Control::automatic);
        this->pid->Initialize();
        this->playTone(SoundEvent::SOUND_HEATER_OFF);
        break;
    default:
        this->setStatus(HEATING);
        this->playTone(SoundEvent::SOUND_HEATER_ON);
        break;
    }
}

void Iron::heaterCheck()
{
    if (this->getHeaterOutput() < this->maxOutput)
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
            this->playTone(SoundEvent::SOUND_ERROR);
        }

        // Start a new observation window.
        this->heaterCheckStart = millis();
        this->heaterCheckTemp = this->getCurrentTemp();
    }
}

void Iron::heaterUpdate()
{
    if (this->outputReversed)
    {
        analogWrite(this->heaterOutputPin, 255 - this->heaterOutput);
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
    this->setStatus(SLEEP);
    this->selectedPreSleepTemp = this->getSelectedTemp();
    this->setSelectedTemp(this->minTemp);
}

void Iron::heaterWake()
{
    this->setStatus(HEATING);
    this->setSelectedTemp(this->selectedPreSleepTemp);
}

float Iron::readVoltage(uint8_t pin, int samples, byte skip, unsigned int sampleTimeUs, float vRef)
{
    int total = 0;

    // Discard first few measurements
    for (int i = 0; i < skip; i++)
    {
        analogRead(pin);
    }

    for (int i = 0; i < samples; i++)
    {
        delayMicroseconds(sampleTimeUs);
        total += analogRead(pin);
    }

    return (float)total / samples * vRef / 1023.0f;
}

float Iron::readColdJunctionTemp(byte pin, float vRef, float fixedRes, float thermRes)
{
    float volts = Iron::readVoltage(
        pin,
        TEMP_MEASURE_SAMPLES,
        TEMP_MEASURE_SAMPLES_TO_SKIP,
        TEMP_MEASURE_SAMPLE_TIME_US,
        vRef);

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
        TEMP_MEASURE_SAMPLES_TO_SKIP,
        TEMP_MEASURE_SAMPLE_TIME_US,
        this->vRef);
    this->heaterUpdate();

    // Convert amplifier output to thermocouple millivolts.
    float tcMilliVolts = (volts - this->opAmpZeroOutputV) * 1000.0f / this->opAmpGain;

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
    this->playTone(SoundEvent::SOUND_TEMP_CHANGE);
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
    this->playTone(SoundEvent::SOUND_TEMP_CHANGE);
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
    Settings tmp;

    EEPROM.get(this->heaterOutputPin, tmp);

    if (!isnan(tmp.selectedTemp) && !isnan(tmp.kp) && !isnan(tmp.ki) && !isnan(tmp.kd))
    {
        Serial.println("Loading settings from eeprom\n");
        this->settings = tmp;
    }
}

/**
 * @brief Sets the ADC clock prescaler on an ATmega328P Arduino Uno.
 *
 * Configures the ADC clock divider relative to the CPU clock.
 * A lower prescaler increases ADC conversion speed but may reduce
 * measurement accuracy and effective resolution.
 *
 * @param prescaler ADC clock divider. Supported values:
 *                  2, 4, 8, 16, 32, 64, 128.
 *
 * @return true  if the prescaler was accepted and configured.
 * @return false if the prescaler value is unsupported.
 *
 * @note The ADC clock is derived from the CPU clock.
 *       For a 16 MHz Arduino Uno:
 *
 *       Prescaler | ADC clock | Approx. conversion time
 *       ----------|-----------|------------------------
 *             2   |   8 MHz   |       1.6 us
 *             4   |   4 MHz   |       3.3 us
 *             8   |   2 MHz   |       6.5 us
 *            16   |   1 MHz   |        13 us
 *            32   | 500 kHz   |        26 us
 *            64   | 250 kHz   |        52 us
 *           128   | 125 kHz   |       104 us
 *
 * @note The prescaler is global to the ADC, not specific to an analog pin.
 *       All analog inputs use the same ADC clock.
 *
 * @note The function modifies only the ADC prescaler bits in ADCSRA.
 *
 * @warning The ADC is specified for full 10-bit performance at a clock
 *          of approximately 50-200 kHz. Faster settings may reduce
 *          accuracy and effective resolution.
 *
 * @warning This function is specific to the ATmega328P and should not
 *          be assumed to work on other Arduino boards.
 *
 * @example
 * setADCPrescaler(128);  // Default: 125 kHz ADC clock
 * setADCPrescaler(64);   // Faster: 250 kHz ADC clock
 * setADCPrescaler(32);   // Faster still: 500 kHz ADC clock
 */
bool Iron::setADCPrescaler(uint8_t prescaler)
{
    uint8_t bits;

    switch (prescaler)
    {
    case 2:
        bits = 1;
        break;
    case 4:
        bits = 2;
        break;
    case 8:
        bits = 3;
        break;
    case 16:
        bits = 4;
        break;
    case 32:
        bits = 5;
        break;
    case 64:
        bits = 6;
        break;
    case 128:
        bits = 7;
        break;
    default:
        return false;
    }

    // Update only the ADC prescaler bits
    ADCSRA = (ADCSRA & 0b11111000) | bits;

    return true;
}

/**
 * @brief Sets the hardware timer prescaler for an Arduino Uno PWM pin.
 *
 * Changes the timer clock divider to adjust the PWM frequency.
 * A lower prescaler increases the PWM frequency; a higher prescaler
 * decreases it.
 *
 * @param pin      Arduino Uno PWM pin: 3, 5, 6, 9, 10, or 11.
 * @param prescale Requested timer prescaler.
 *
 * Supported prescalers and approximate PWM frequencies:
 *
 * Timer0 - Pins 5 and 6 (fast PWM, default Arduino mode):
 *   Prescaler  1:  62500 Hz (62.5 kHz)
 *   Prescaler  8:   7812 Hz (7.81 kHz)
 *   Prescaler 64:    977 Hz
 *   Prescaler 256:   244 Hz
 *   Prescaler 1024:   61 Hz
 *
 * Timer1 - Pins 9 and 10 (standard Arduino 8-bit phase-correct PWM):
 *   Prescaler  1:  31373 Hz (31.37 kHz)
 *   Prescaler  8:   3922 Hz (3.92 kHz)
 *   Prescaler 64:    490 Hz
 *   Prescaler 256:   122 Hz
 *   Prescaler 1024:   31 Hz
 *
 * Timer2 - Pins 3 and 11 (fast PWM, default Arduino mode):
 *   Prescaler  1:  62500 Hz (62.5 kHz)
 *   Prescaler  8:   7812 Hz (7.81 kHz)
 *   Prescaler 32:   1953 Hz (1.95 kHz)
 *   Prescaler 64:    977 Hz
 *   Prescaler 128:   488 Hz
 *   Prescaler 256:   244 Hz
 *   Prescaler 1024:   61 Hz
 *
 * @note Frequencies assume a 16 MHz CPU clock and the standard Arduino
 *       Uno timer waveform-generation modes. Actual frequency depends
 *       on the timer mode and TOP value.
 *
 * @note Prescalers are shared by both PWM pins using the same timer:
 *       Timer0: pins 5 and 6
 *       Timer1: pins 9 and 10
 *       Timer2: pins 3 and 11
 *
 * @warning Changing Timer0 affects millis(), micros(), and delay().
 * @warning Changing Timer1 may interfere with libraries such as Servo.
 * @warning Changing a timer prescaler affects other PWM outputs on that
 *          timer, not just the pin passed to this function.
 *
 * @note Unsupported pins or prescaler values are ignored.
 *
 * @example
 * setPWMPrescaler(9, 1);    // Pin 9: approximately 31.37 kHz
 * setPWMPrescaler(9, 64);   // Pin 9: approximately 490 Hz
 * setPWMPrescaler(11, 32);  // Pin 11: approximately 1.95 kHz
 */
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

/**
 * @brief Reads the internal 1.1 V bandgap reference using the ADC.
 *
 * Configures the ADC to use AVCC as its reference voltage and selects
 * the internal bandgap voltage as the ADC input.
 *
 * The returned ADC count represents the internal bandgap voltage
 * measured relative to AVCC.
 *
 * @return Raw 10-bit ADC reading (0-1023).
 *
 * @note The ADC reading can be used to calculate the actual AVCC supply
 *       voltage when the bandgap voltage is known or calibrated.
 *
 * @note The nominal internal bandgap voltage is approximately 1.1 V,
 *       but its actual value varies between microcontrollers.
 *
 * @note For accurate VCC measurements, calibrate the bandgap voltage
 *       against a multimeter-measured AVCC voltage and store the
 *       calibrated value.
 *
 * @warning This function changes the ADC reference and input selection.
 *          Other ADC operations may be affected until reconfigured.
 *
 * @warning This implementation assumes an ATmega328P-based Arduino Uno.
 *
 * @example
 * long bandgapADC = Iron::readBandGapADC();
 */
long Iron::readBandGapADC()
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
/**
 * @brief Measures the AVCC supply voltage using the internal bandgap.
 *
 * Selects the internal nominal 1.1 V bandgap as the ADC input and
 * measures it relative to AVCC. The measured ADC count is then used
 * to estimate the AVCC supply voltage.
 *
 * @param bandGapVoltage Calibrated internal bandgap voltage in millivolts.
 *                       For example, 1098 means 1.098 V.
 *
 * @return Estimated AVCC supply voltage in volts.
 *         Returns 0.0 if the ADC reading is zero.
 *
 * @note The calculation uses:
 *
 *       VCC (mV) = bandGapVoltage (mV) * 1023 / ADC reading
 *
 * @note The bandgap voltage should be calibrated for the individual
 *       microcontroller for best accuracy. Using 1100 mV assumes the
 *       bandgap is exactly 1.100 V and can cause measurement error.
 *
 * @note This function changes the ADC reference and input selection.
 *       Other ADC operations may be affected until reconfigured.
 *
 * @warning This implementation assumes an ATmega328P-based Arduino Uno.
 *
 * @example
 * float vcc = Iron::readVccVolts(1098);
 * // Example result: approximately 4.95 V
 */
float Iron::readVccVolts(long bandGapVoltage)
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

    return bandGapVoltage * 1023L / adc / 1000.0f;
}