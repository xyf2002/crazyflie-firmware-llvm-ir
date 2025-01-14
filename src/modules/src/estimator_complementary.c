/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * estimator_complementary.c - a complementary estimator
 */

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "stabilizer.h"
#include "estimator_complementary.h"
#include "sensfusion6.h"
#include "position_estimator.h"
#include "sensors.h"
#include "stabilizer_types.h"
#include "static_mem.h"

static Axis3f gyro;
static Axis3f acc;
static baro_t baro;
static tofMeasurement_t tof;

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE


//#define  ELAPSED_TIME_MAX_SECTIONS  15
//
//typedef  struct  elapsed_time {
//    uint32_t  start;
//    uint32_t  current;
//    uint32_t  max;
//    uint32_t  min;
//} ELAPSED_TIME;
//
//static  ELAPSED_TIME  elapsed_time_tbl[ELAPSED_TIME_MAX_SECTIONS];
//
//void  elapsed_time_clr (uint32_t  i)
//{
//  ELAPSED_TIME  *p_tbl;
//
//
//  p_tbl          = &elapsed_time_tbl[i];
//  p_tbl->start   = 0;
//  p_tbl->current = 0;
//  p_tbl->min     = 0xFFFFFFFF;
//  p_tbl->max     = 0;
//}
//
//void  elapsed_time_init (void)
//{
//  uint32_t  i;
//
//
//  if (ARM_CM_DWT_CTRL != 0) {                  // See if DWT is available
//    ARM_CM_DEMCR      |= 1 << 24;            // Set bit 24
//    ARM_CM_DWT_CYCCNT  = 0;
//    ARM_CM_DWT_CTRL   |= 1 << 0;             // Set bit 0
//  }
//  for (i = 0; i < ELAPSED_TIME_MAX_SECTIONS; i++) {
//    elapsed_time_clr(i);
//  }
//}
//
//void  elapsed_time_start (uint32_t  i)
//{
//  elapsed_time_tbl[i].start = ARM_CM_DWT_CYCCNT;
//}
//
//void  elapsed_time_stop (uint32_t  i)
//{
//  uint32_t       stop;
//  ELAPSED_TIME  *p_tbl;
//
//  stop           = ARM_CM_DWT_CYCCNT;
//  p_tbl          = &elapsed_time_tbl[i];
//  p_tbl->current = stop - p_tbl->start;
//  if (p_tbl->max < p_tbl->current) {
//    p_tbl->max = p_tbl->current;
//  }
//  if (p_tbl->min > p_tbl->current) {
//    p_tbl->min = p_tbl->current;
//  }
//}




    void
    estimatorComplementaryInit(void)
{
  sensfusion6Init();
}

bool estimatorComplementaryTest(void)
{
  bool pass = true;

  pass &= sensfusion6Test();

  return pass;
}

void estimatorComplementary(state_t *state, const uint32_t tick)
{
  // Pull the latest sensors values of interest; discard the rest
  measurement_t m;
  while (estimatorDequeue(&m)) {
    switch (m.type)
    {
    case MeasurementTypeGyroscope:
      gyro = m.data.gyroscope.gyro;
      break;
    case MeasurementTypeAcceleration:
      acc = m.data.acceleration.acc;
      break;
    case MeasurementTypeBarometer:
      baro = m.data.barometer.baro;
      break;
    case MeasurementTypeTOF:
      tof = m.data.tof;
      break;
    default:
      break;
    }
  }

  // Update filter
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
//    sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z,
//                        acc.x, acc.y, acc.z,
//                        ATTITUDE_UPDATE_DT);

    sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z,
                        acc.x, acc.y, acc.z,
                        ATTITUDE_UPDATE_DT,
                        &state->attitudeQuaternion.x,
                        &state->attitudeQuaternion.y,
                        &state->attitudeQuaternion.z,
                        &state->attitudeQuaternion.w);

    // Save attitude, adjusted for the legacy CF2 body coordinate system
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

    // Save quaternion, hopefully one day this could be used in a better controller.
    // Note that this is not adjusted for the legacy coordinate system
    sensfusion6GetQuaternion(
      &state->attitudeQuaternion.x,
      &state->attitudeQuaternion.y,
      &state->attitudeQuaternion.z,
      &state->attitudeQuaternion.w);

    state->acc.z = sensfusion6GetAccZWithoutGravity(acc.x,
                                                    acc.y,
                                                    acc.z);

    positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);
  }

  if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
    positionEstimate(state, &baro, &tof, POS_UPDATE_DT, tick);
  }
}
