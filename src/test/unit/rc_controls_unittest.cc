/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>

#include <limits.h>

//#define DEBUG_RC_CONTROLS

extern "C" {
    #include "platform.h"

    #include "common/maths.h"
    #include "common/axis.h"
    #include "common/bitarray.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "blackbox/blackbox.h"
    #include "blackbox/blackbox_fielddefs.h"

    #include "drivers/sensor.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "io/beeper.h"

    #include "rx/rx.h"

    #include "flight/pid.h"

    #include "fc/config.h"
    #include "fc/controlrate_profile.h"
    #include "fc/rc_modes.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"
    #include "fc/fc_core.h"

    #include "scheduler/scheduler.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

void unsetArmingDisabled(armingDisableFlags_e flag) {
  UNUSED(flag);
}

class RcControlsModesTest : public ::testing::Test {
protected:
    virtual void SetUp() {
    }
};

TEST_F(RcControlsModesTest, updateActivatedModesWithAllInputsAtMidde)
{
    // given
    boxBitmask_t mask;
    memset(&mask, 0, sizeof(mask));
    rcModeUpdate(&mask);

    // and
    memset(&rxRuntimeConfig, 0, sizeof(rxRuntimeConfig_t));
    rxRuntimeConfig.channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // when
    updateActivatedModes();

    // then
    for (int index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
#ifdef DEBUG_RC_CONTROLS
        printf("iteration: %d\n", index);
#endif
        EXPECT_EQ(false, IS_RC_MODE_ACTIVE((boxId_e)index));
    }
}

TEST_F(RcControlsModesTest, updateActivatedModesUsingValidAuxConfigurationAndRXValues)
{
    // given
    modeActivationConditionsMutable(0)->modeId = (boxId_e)0;
    modeActivationConditionsMutable(0)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(1)->modeId = (boxId_e)1;
    modeActivationConditionsMutable(1)->auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(1700);

    modeActivationConditionsMutable(2)->modeId = (boxId_e)2;
    modeActivationConditionsMutable(2)->auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(1200);

    modeActivationConditionsMutable(3)->modeId = (boxId_e)3;
    modeActivationConditionsMutable(3)->auxChannelIndex = AUX4 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(3)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(3)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(4)->modeId = (boxId_e)4;
    modeActivationConditionsMutable(4)->auxChannelIndex = AUX5 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(4)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(4)->range.endStep = CHANNEL_VALUE_TO_STEP(925);

    EXPECT_EQ(0, modeActivationConditions(4)->range.startStep);
    EXPECT_EQ(1, modeActivationConditions(4)->range.endStep);

    modeActivationConditionsMutable(5)->modeId = (boxId_e)5;
    modeActivationConditionsMutable(5)->auxChannelIndex = AUX6 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(5)->range.startStep = CHANNEL_VALUE_TO_STEP(2075);
    modeActivationConditionsMutable(5)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    EXPECT_EQ(47, modeActivationConditions(5)->range.startStep);
    EXPECT_EQ(48, modeActivationConditions(5)->range.endStep);

    modeActivationConditionsMutable(6)->modeId = (boxId_e)6;
    modeActivationConditionsMutable(6)->auxChannelIndex = AUX7 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(6)->range.startStep = CHANNEL_VALUE_TO_STEP(925);
    modeActivationConditionsMutable(6)->range.endStep = CHANNEL_VALUE_TO_STEP(950);

    EXPECT_EQ(1, modeActivationConditions(6)->range.startStep);
    EXPECT_EQ(2, modeActivationConditions(6)->range.endStep);

    // and
    boxBitmask_t mask;
    memset(&mask, 0, sizeof(mask));
    rcModeUpdate(&mask);

    // and
    memset(&rxRuntimeConfig, 0, sizeof(rxRuntimeConfig_t));
    rxRuntimeConfig.channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    rcData[AUX1] = PWM_RANGE_MAX;
    rcData[AUX2] = PWM_RANGE_MIDDLE;
    rcData[AUX3] = PWM_RANGE_MIN;
    rcData[AUX4] = PWM_RANGE_MAX;
    rcData[AUX5] = 899; // value lower that range minimum should be treated the same as the lowest range value
    rcData[AUX6] = 2101; // value higher than the range maximum should be treated the same as the highest range value
    rcData[AUX7] = 950; // value equal to range step upper boundary should not activate the mode

    // and
    boxBitmask_t activeBoxIds;
    memset(&activeBoxIds, 0, sizeof(boxBitmask_t));
    bitArraySet(&activeBoxIds, 0);
    bitArraySet(&activeBoxIds, 1);
    bitArraySet(&activeBoxIds, 2);
    bitArraySet(&activeBoxIds, 3);
    bitArraySet(&activeBoxIds, 4);
    bitArraySet(&activeBoxIds, 5);

    // when
    updateActivatedModes();

    // then
    for (int index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
#ifdef DEBUG_RC_CONTROLS
        printf("iteration: %d, %d\n", index, (bool)(bitArrayGet(&activeBoxIds, index)));
#endif
        EXPECT_EQ((bool)(bitArrayGet(&activeBoxIds, index)), IS_RC_MODE_ACTIVE((boxId_e)index));
    }
}

enum {
    COUNTER_QUEUE_CONFIRMATION_BEEP,
    COUNTER_CHANGE_CONTROL_RATE_PROFILE
};
#define CALL_COUNT_ITEM_COUNT 2

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

extern "C" {
void beeperConfirmationBeeps(uint8_t) {
    callCounts[COUNTER_QUEUE_CONFIRMATION_BEEP]++;
}

void beeper(beeperMode_e mode) {
    UNUSED(mode);
}

void changeControlRateProfile(uint8_t) {
    callCounts[COUNTER_CHANGE_CONTROL_RATE_PROFILE]++;
}

}

void resetCallCounters(void) {
    memset(&callCounts, 0, sizeof(callCounts));
}

uint32_t fixedMillis;

extern "C" {
uint32_t millis(void) {
    return fixedMillis;
}

uint32_t micros(void) {
    return fixedMillis * 1000;
}
}

void resetMillis(void) {
    fixedMillis = 0;
}

extern "C" {
void saveConfigAndNotify(void) {}
void initRcProcessing(void) {}
void changePidProfile(uint8_t) {}
void pidInitConfig(const pidProfile_t *) {}
void accSetCalibrationCycles(uint16_t) {}
void gyroStartCalibration(bool isFirstArmingCalibration)
{
    UNUSED(isFirstArmingCalibration);
}
void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
void handleInflightCalibrationStickPosition(void) {}
bool feature(uint32_t) { return false;}
bool sensors(uint32_t) { return false;}
void tryArm(void) {}
void disarm(void) {}
void dashboardDisablePageCycling() {}
void dashboardEnablePageCycling() {}

bool failsafeIsActive() { return false; }
bool rxIsReceivingSignal() { return true; }

uint8_t getCurrentControlRateProfileIndex(void) {
    return 0;
}
void GPS_reset_home_position(void) {}
void baroSetCalibrationCycles(uint16_t) {}

void blackboxLogEvent(FlightLogEvent, flightLogEventData_t *) {}

bool cmsInMenu = false;
uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;
int16_t heading;
uint8_t stateFlags = 0;
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
rxRuntimeConfig_t rxRuntimeConfig;
PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
void resetArmingDisabled(void) {}
timeDelta_t getTaskDeltaTime(cfTaskId_e) { return 20000; }
}
armingDisableFlags_e getArmingDisableFlags(void) {
    return (armingDisableFlags_e) 0;
}
bool isTryingToArm(void) { return false; }
void resetTryingToArm(void) {}
