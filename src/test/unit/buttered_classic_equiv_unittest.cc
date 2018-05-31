#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/filter.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "flight/pid.h"
    #include "flight/imu.h"
    #include "drivers/accgyro/accgyro.h"
    #include "drivers/accgyro/gyro_sync.h"
    #include "sensors/gyro.h"

    gyro_t gyro;
    pidProfile_t pidProfileTest;
    pidProfile_t *pidProfile = &pidProfileTest;

    void resetPidProfile(pidProfile_t *pidProfile);
    float butteredPids(const pidProfile_t *pidProfile, int axis, float errorRate, float dynCi, float iDT, float currentPidSetpoint);
    float classicPids(const pidProfile_t *pidProfile, int axis, float errorRate, float dynCi, float iDT, float currentPidSetpoint);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

gyroDev_t gyroDev;

bool computeAndCompareDerivatives(uint16_t notchCutoff, uint16_t lpfCutoff, uint8_t filterStyle, uint8_t setpointWeight)
{
    // Calculate derivatives on separate axes so filter and derivative states are isolated
    const int axisButtered = FD_ROLL;
    const int axisClassic = FD_PITCH;
    const float gyroRate = 125.0f; // 8k
    const float errorRate = 1.0f;
    const float dynCi = 1.0f;
    const float dT = gyroRate * 0.000001f;
    const float iDT = 1.0f / dT;
    const float currentPidSetpoint = 1.0f;
    float deltaButtered = 0;
    float deltaClassic = 0;
    float randVal = 0;
    bool deltaMatch = false;

    gyro.targetLooptime = gyroRate;
    pidConfigMutable()->pid_process_denom = 1;
    resetPidProfile(pidProfile);

    srand(1);  // idempotence :)

    pidProfile->dtermSetpointWeight = setpointWeight;
    pidProfile->dterm_filter_style = filterStyle;
    pidProfile->dterm_notch_hz = notchCutoff;
    pidProfile->dterm_lpf_hz = lpfCutoff;
    pidInit(pidProfile);

    for (unsigned int i=0; i<1000000; ++i) {
        randVal = ((float)rand() / ((float)RAND_MAX+1) * 2000);
        gyro.gyroADCf[axisButtered] = randVal;
        gyro.gyroADCf[axisClassic] = randVal;
        deltaButtered = butteredPids(pidProfile, axisButtered, errorRate, dynCi, iDT, currentPidSetpoint);
        deltaClassic = classicPids(pidProfile, axisClassic, errorRate, dynCi, iDT, currentPidSetpoint);
        deltaMatch = (deltaButtered == deltaClassic);
        if (!deltaMatch) {
	    // break upon first failure and fast return
            break;
        }
    }

    return deltaMatch;

}

TEST(ButteredPidsHypeTest, NoSetpointWeightAndNotch_SP)
{
    const uint16_t notchCutoff = 0;
    const uint16_t lpfCutoff = 0;
    const uint8_t filterStyle = KD_FILTER_SP;
    const uint8_t setpointWeight = 0;
    EXPECT_TRUE(computeAndCompareDerivatives(notchCutoff, lpfCutoff, filterStyle, setpointWeight));
}

TEST(ButteredPidsHypeTest, NoSetpointWeightAndNotch_NOSP)
{
    const uint16_t notchCutoff = 0;
    const uint16_t lpfCutoff = 0;
    const uint8_t filterStyle = KD_FILTER_NOSP;
    const uint8_t setpointWeight = 0;
    EXPECT_TRUE(computeAndCompareDerivatives(notchCutoff, lpfCutoff, filterStyle, setpointWeight));
}

TEST(ButteredPidsHypeTest, NoSetpointWeightAndNotch_SP_LPF)
{
    const uint16_t notchCutoff = 0;
    const uint16_t lpfCutoff = 160;
    const uint8_t filterStyle = KD_FILTER_SP;
    const uint8_t setpointWeight = 0;
    EXPECT_TRUE(computeAndCompareDerivatives(notchCutoff, lpfCutoff, filterStyle, setpointWeight));
}

TEST(ButteredPidsHypeTest, NoSetpointWeightAndNotch_NOSP_LPF)
{
    const uint16_t notchCutoff = 0;
    const uint16_t lpfCutoff = 160;
    const uint8_t filterStyle = KD_FILTER_NOSP;
    const uint8_t setpointWeight = 0;
    EXPECT_TRUE(computeAndCompareDerivatives(notchCutoff, lpfCutoff, filterStyle, setpointWeight));
}

TEST(ButteredPidsHypeTest, FailOnSetpointWeight_SP)
{
    const uint16_t notchCutoff = 0;
    const uint16_t lpfCutoff = 0;
    const uint8_t filterStyle = KD_FILTER_SP;
    const uint8_t setpointWeight = 1;
    EXPECT_FALSE(computeAndCompareDerivatives(notchCutoff, lpfCutoff, filterStyle, setpointWeight));
}

TEST(ButteredPidsHypeTest, FailOnSetpointWeight_NOSP)
{
    const uint16_t notchCutoff = 0;
    const uint16_t lpfCutoff = 0;
    const uint8_t filterStyle = KD_FILTER_NOSP;
    const uint8_t setpointWeight = 1;
    EXPECT_FALSE(computeAndCompareDerivatives(notchCutoff, lpfCutoff, filterStyle, setpointWeight));
}

TEST(ButteredPidsHypeTest, FailOnNotch_SP)
{
    const uint16_t notchCutoff = 120;
    const uint16_t lpfCutoff = 0;
    const uint8_t filterStyle = KD_FILTER_SP;
    const uint8_t setpointWeight = 0;
    EXPECT_FALSE(computeAndCompareDerivatives(notchCutoff, lpfCutoff, filterStyle, setpointWeight));
}

TEST(ButteredPidsHypeTest, FailOnNotch_NOSP)
{
    const uint16_t notchCutoff = 120;
    const uint16_t lpfCutoff = 0;
    const uint8_t filterStyle = KD_FILTER_NOSP;
    const uint8_t setpointWeight = 0;
    EXPECT_FALSE(computeAndCompareDerivatives(notchCutoff, lpfCutoff, filterStyle, setpointWeight));
}

extern "C" { //stubs
    attitudeEulerAngles_t attitude;
    uint16_t flightModeFlags = 0;
    uint16_t armingFlags = 0;
    int16_t GPS_angle[2] = { 0, 0 };
    float getThrottlePIDAttenuation(void) { return 0; }
    bool mixerIsOutputSaturated(int, float) { return false; }
    float getRcDeflectionAbs(int) { return 10.0f; }
    float getRcDeflection(int) { return 0; }
    float getSetpointRate(int) { return 0; }
    float getMotorMixRange(int) { return 0; }
    void systemBeep(bool) { return; }
    bool sensors(uint32_t) { return false; }
    bool gyroOverflowDetected(void) { return false; }
}
