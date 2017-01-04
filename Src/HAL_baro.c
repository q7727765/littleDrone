/*
 * HAL_baro.c
 *
 *  Created on: 2017Äê1ÔÂ3ÈÕ
 *      Author: 50430
 */

#include "HAL.h"
#include "maths.h"

baro_t baro;                        // barometer access functions
uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value
int32_t baroPressure = 0;
int32_t baroTemperature = 0;
int32_t BaroAlt = 0;

static int32_t baroGroundAltitude = 0;
static int32_t baroGroundPressure = 0;
static uint32_t baroPressureSum = 0;

#define BARO_SAMPLE_COUNT_MAX   48

typedef enum {
    BAROMETER_NEEDS_SAMPLES = 0,
    BAROMETER_NEEDS_CALCULATION
} barometerState_e;

static bool baroReady = false;

#define PRESSURE_SAMPLES_MEDIAN 3


static int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;

    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;

    if (medianFilterReady)
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}

uint32_t recalculateBarometerTotal(uint8_t baroSampleCount, uint32_t pressureTotal, int32_t newPressureReading)
{
    static int32_t barometerSamples[BARO_SAMPLE_COUNT_MAX];
    static int currentSampleIndex = 0;
    int nextSampleIndex;

    // store current pressure in barometerSamples
    nextSampleIndex = (currentSampleIndex + 1);
    if (nextSampleIndex == baroSampleCount) {
        nextSampleIndex = 0;
        baroReady = true;
    }
    barometerSamples[currentSampleIndex] = applyBarometerMedianFilter(newPressureReading);

    // recalculate pressure total
    // Note, the pressure total is made up of baroSampleCount - 1 samples - See PRESSURE_SAMPLE_COUNT
    pressureTotal += barometerSamples[currentSampleIndex];
    pressureTotal -= barometerSamples[nextSampleIndex];

    currentSampleIndex = nextSampleIndex;

    return pressureTotal;
}

//bool isBaroReady(void) {
//	return baroReady;
//}
//
//uint32_t baroUpdate(void)
//{
//    static barometerState_e state = BAROMETER_NEEDS_SAMPLES;
//
//    switch (state) {
//        default:
//        case BAROMETER_NEEDS_SAMPLES:
//            baro.get_ut();
//            baro.start_up();
//            state = BAROMETER_NEEDS_CALCULATION;
//            return baro.up_delay;
//        break;
//
//        case BAROMETER_NEEDS_CALCULATION:
//            baro.get_up();
//            baro.start_ut();
//            baro.calculate(&baroPressure, &baroTemperature);
//            baroPressureSum = recalculateBarometerTotal(BARO_SAMPLE_COUNT_MAX, baroPressureSum, baroPressure);
//            state = BAROMETER_NEEDS_SAMPLES;
//            return baro.ut_delay;
//        break;
//    }
//}


//int32_t baroCalculateAltitude(void)
//{
//    int32_t BaroAlt_tmp;
//
//    // calculates height from ground via baro readings
//    // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
//    BaroAlt_tmp = lrintf((1.0f - powf((float)(baroPressureSum / PRESSURE_SAMPLE_COUNT) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
//    BaroAlt_tmp -= baroGroundAltitude;
//    BaroAlt = lrintf((float)BaroAlt * barometerConfig()->baro_noise_lpf + (float)BaroAlt_tmp * (1.0f - barometerConfig()->baro_noise_lpf)); // additional LPF to reduce baro noise
//
//    return BaroAlt;
//}

//void performBaroCalibrationCycle(void)
//{
//    baroGroundPressure -= baroGroundPressure / 8;
//    baroGroundPressure += baroPressureSum / PRESSURE_SAMPLE_COUNT;
//    baroGroundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;
//
//    calibratingB--;
//}
