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
#include <stdbool.h>
#include <limits.h>
#include <cmath>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/vector.h"

    #include "config/feature.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "drivers/accgyro/accgyro.h"
    #include "drivers/compass/compass.h"
    #include "drivers/sensor.h"

    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/imu.h"
    #include "flight/position.h"

    #include "io/gps.h"

    #include "rx/rx.h"

    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"
    #include "sensors/sensors.h"

    void imuComputeRotationMatrix(void);
    void imuUpdateEulerAngles(void);

    extern quaternion q;
    extern float rMat[3][3];
    extern bool attitudeIsEstablished;

    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);

    PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
        .enabledFeatures = 0
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

const float sqrt2over2 = sqrt(2) / 2.0;
const float sqrt3over2 = sqrt(3) / 2.0;
const float identity33[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

// generate quaternion from axis and angle (in rad)
void quaternion_from_axis_angle(quaternion* q, float angle, float x, float y, float z) {
    fpVector3_t a = {{x, y, z}};
    vectorNormalize(&a, &a);
    q->w = cos(angle / 2);
    q->x = a.x * sin(angle / 2);
    q->y = a.y * sin(angle / 2);
    q->z = a.z * sin(angle / 2);
}

TEST(FlightImuTest, TestCalculateRotationMatrix)
{
    #define TOL 1e-6

    // No rotation
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;

    imuComputeRotationMatrix();

    EXPECT_FLOAT_EQ(1.0f, rMat[0][0]);
    EXPECT_FLOAT_EQ(0.0f, rMat[0][1]);
    EXPECT_FLOAT_EQ(0.0f, rMat[0][2]);
    EXPECT_FLOAT_EQ(0.0f, rMat[1][0]);
    EXPECT_FLOAT_EQ(1.0f, rMat[1][1]);
    EXPECT_FLOAT_EQ(0.0f, rMat[1][2]);
    EXPECT_FLOAT_EQ(0.0f, rMat[2][0]);
    EXPECT_FLOAT_EQ(0.0f, rMat[2][1]);
    EXPECT_FLOAT_EQ(1.0f, rMat[2][2]);

    // 90 degrees around Z axis
    quaternion_from_axis_angle(&q, DEGREES_TO_RADIANS(90), 0, 0, 1);

    imuComputeRotationMatrix();

    EXPECT_NEAR(0.0f, rMat[0][0], TOL);
    EXPECT_NEAR(-1.0f, rMat[0][1], TOL);
    EXPECT_NEAR(0.0f, rMat[0][2], TOL);
    EXPECT_NEAR(1.0f, rMat[1][0], TOL);
    EXPECT_NEAR(0.0f, rMat[1][1], TOL);
    EXPECT_NEAR(0.0f, rMat[1][2], TOL);
    EXPECT_NEAR(0.0f, rMat[2][0], TOL);
    EXPECT_NEAR(0.0f, rMat[2][1], TOL);
    EXPECT_NEAR(1.0f, rMat[2][2], TOL);

    // 60 degrees around X axis
    quaternion_from_axis_angle(&q, DEGREES_TO_RADIANS(60), 1, 0, 0);

    imuComputeRotationMatrix();

    EXPECT_NEAR(1.0f, rMat[0][0], TOL);
    EXPECT_NEAR(0.0f, rMat[0][1], TOL);
    EXPECT_NEAR(0.0f, rMat[0][2], TOL);
    EXPECT_NEAR(0.0f, rMat[1][0], TOL);
    EXPECT_NEAR(0.5f, rMat[1][1], TOL);
    EXPECT_NEAR(-sqrt3over2, rMat[1][2], TOL);
    EXPECT_NEAR(0.0f, rMat[2][0], TOL);
    EXPECT_NEAR(sqrt3over2, rMat[2][1], TOL);
    EXPECT_NEAR(0.5f, rMat[2][2], TOL);
}

TEST(FlightImuTest, TestUpdateEulerAngles)
{
    // No rotation
    memcpy(rMat, identity33, sizeof(rMat));

    imuUpdateEulerAngles();

    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
    EXPECT_EQ(0, attitude.values.yaw);

    // 45 degree yaw
    memset(rMat, 0.0, sizeof(rMat));
    rMat[0][0] = sqrt2over2;
    rMat[0][1] = sqrt2over2;
    rMat[1][0] = -sqrt2over2;
    rMat[1][1] = sqrt2over2;

    imuUpdateEulerAngles();

    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
    EXPECT_EQ(450, attitude.values.yaw);
}

TEST(FlightImuTest, TestSmallAngle)
{
    // given
    imuConfigMutable()->small_angle = 25;
    imuConfigure(0, 0);
    attitudeIsEstablished = true;

    // level
    quaternion_from_axis_angle(&q, DEGREES_TO_RADIANS(0), 1, 0, 0);
    imuComputeRotationMatrix();          // identity
    EXPECT_TRUE(isUpright());

    // 26 deg around x
    quaternion_from_axis_angle(&q, DEGREES_TO_RADIANS(26), 1, 0, 0);
    imuComputeRotationMatrix();
    EXPECT_FALSE(isUpright());

    // 24 deg around x
    quaternion_from_axis_angle(&q, DEGREES_TO_RADIANS(24), 1, 0, 0);
    imuComputeRotationMatrix();
    EXPECT_TRUE(isUpright());
}

// STUBS

extern "C" {
    boxBitmask_t rcModeActivationMask;
    float rcCommand[4];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

    gyro_t gyro;
    acc_t acc;
    mag_t mag;

    gpsSolutionData_t gpsSol;

    uint8_t debugMode;
    int16_t debug[DEBUG16_VALUE_COUNT];

    uint8_t stateFlags;
    uint16_t flightModeFlags;
    uint8_t armingFlags;

    pidProfile_t *currentPidProfile;

    uint16_t enableFlightMode(flightModeFlags_e mask) {
        return flightModeFlags |= (mask);
    }

    uint16_t disableFlightMode(flightModeFlags_e mask) {
        return flightModeFlags &= ~(mask);
    }

    bool sensors(uint32_t mask) {
        return mask & SENSOR_ACC;
    };

    uint32_t millis(void) { return 0; }
    uint32_t micros(void) { return 0; }

    bool compassIsHealthy(void) { return true; }
    bool baroIsCalibrated(void) { return true; }
    void performBaroCalibrationCycle(void) {}
    float baroCalculateAltitude(void) { return 0; }
    bool gyroGetAccumulationAverage(float *) { return false; }
    bool accGetAccumulationAverage(float *) { return false; }
    void mixerSetThrottleAngleCorrection(int) {};
    bool gpsRescueIsRunning(void) { return false; }
    bool isFixedWing(void) { return false; }
    void pinioBoxTaskControl(void) {}
    void schedulerIgnoreTaskExecTime(void) {}
    void schedulerIgnoreTaskStateTime(void) {}
    void schedulerSetNextStateTime(timeDelta_t) {}
    bool schedulerGetIgnoreTaskExecTime() { return false; }
    float gyroGetFilteredDownsampled(int) { return 0.0f; }
    float baroUpsampleAltitude()  { return 0.0f; }
    float pt2FilterGain(float, float)  { return 0.0f; }
    float getBaroAltitude(void) { return 3000.0f; }
    float gpsRescueGetImuYawCogGain(void) { return 1.0f; }

    void pt2FilterInit(pt2Filter_t *baroDerivativeLpf, float) {
        UNUSED(baroDerivativeLpf);
    }
    float pt2FilterApply(pt2Filter_t *baroDerivativeLpf, float) {
        UNUSED(baroDerivativeLpf);
        return 0.0f;
    }
}
