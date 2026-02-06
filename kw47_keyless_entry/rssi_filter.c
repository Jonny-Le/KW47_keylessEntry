/*! *********************************************************************************
* \file rssi_filter.c
*
* RSSI filtering implementation for KW47 keyless entry system
* Implements moving average and Kalman filtering for stable proximity detection
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "rssi_filter.h"
#include "FunctionLib.h"
#include "fsl_component_timer_manager.h"
#include <math.h>

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/

#define KALMAN_PROCESS_NOISE     0.1f    // Process noise variance
#define KALMAN_MEASUREMENT_NOISE 4.0f    // Measurement noise variance (RSSI variance)
#define KALMAN_INITIAL_ERROR     10.0f   // Initial estimation error

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

static void RssiFilter_UpdateMovingAverage(rssiFilter_t *pFilter, int8_t rssi);
static void RssiFilter_UpdateKalman(rssiFilter_t *pFilter, int8_t rssi);
static void RssiFilter_UpdateStateMachine(rssiFilter_t *pFilter);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief     Initialize RSSI filter
********************************************************************************** */
void RssiFilter_Init(rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
        return;

    // Initialize moving average filter
    FLib_MemSet(&pFilter->movingAvg, 0, sizeof(rssiMovingAverage_t));
    pFilter->movingAvg.index = 0;
    pFilter->movingAvg.count = 0;
    pFilter->movingAvg.movingAverage = -100; // Very low initial value

    // Initialize Kalman filter
    pFilter->kalman.x = -70.0f;        // Initial RSSI estimate
    pFilter->kalman.P = KALMAN_INITIAL_ERROR;
    pFilter->kalman.Q = KALMAN_PROCESS_NOISE;
    pFilter->kalman.R = KALMAN_MEASUREMENT_NOISE;
    pFilter->kalman.initialized = false;

    // Initialize state machine
    pFilter->currentState = RssiState_Idle_c;
    pFilter->previousState = RssiState_Idle_c;
    pFilter->lastUpdateTime = 0;
    pFilter->stateChanged = false;
}

/*! *********************************************************************************
* \brief     Add new RSSI measurement and update filter
********************************************************************************** */
void RssiFilter_AddMeasurement(rssiFilter_t *pFilter, int8_t rssi)
{
    if (pFilter == NULL)
        return;

    // Update moving average
    RssiFilter_UpdateMovingAverage(pFilter, rssi);

    // Update Kalman filter (only when we have enough samples)
    if (pFilter->movingAvg.count >= RSSI_FILTER_WINDOW_SIZE / 2)
    {
        RssiFilter_UpdateKalman(pFilter, rssi);
    }

    // Update state machine
    RssiFilter_UpdateStateMachine(pFilter);

    // Update timestamp
    pFilter->lastUpdateTime = TM_GetTimestamp();
}

/*! *********************************************************************************
* \brief     Get current filtered RSSI value
********************************************************************************** */
int8_t RssiFilter_GetFilteredRssi(rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
        return -100;

    // Use Kalman filter if initialized, otherwise use moving average
    if (pFilter->kalman.initialized)
    {
        return (int8_t)roundf(pFilter->kalman.x);
    }
    else
    {
        return pFilter->movingAvg.movingAverage;
    }
}

/*! *********************************************************************************
* \brief     Get current proximity state
********************************************************************************** */
rssiState_t RssiFilter_GetState(rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
        return RssiState_Idle_c;

    return pFilter->currentState;
}

/*! *********************************************************************************
* \brief     Check if state has changed
********************************************************************************** */
bool_t RssiFilter_HasStateChanged(rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
        return false;

    bool_t changed = pFilter->stateChanged;
    pFilter->stateChanged = false; // Clear flag
    return changed;
}

/*! *********************************************************************************
* \brief     Reset filter to initial state
********************************************************************************** */
void RssiFilter_Reset(rssiFilter_t *pFilter)
{
    if (pFilter == NULL)
        return;

    RssiFilter_Init(pFilter);
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief     Update moving average filter
********************************************************************************** */
static void RssiFilter_UpdateMovingAverage(rssiFilter_t *pFilter, int8_t rssi)
{
    rssiMovingAverage_t *pAvg = &pFilter->movingAvg;

    // Add new sample to buffer
    pAvg->values[pAvg->index] = rssi;
    pAvg->index = (pAvg->index + 1) % RSSI_FILTER_WINDOW_SIZE;

    // Update count
    if (pAvg->count < RSSI_FILTER_WINDOW_SIZE)
        pAvg->count++;

    // Calculate moving average
    int32_t sum = 0;
    for (uint8_t i = 0; i < pAvg->count; i++)
    {
        sum += pAvg->values[i];
    }
    pAvg->movingAverage = (int8_t)(sum / pAvg->count);
}

/*! *********************************************************************************
* \brief     Update Kalman filter
********************************************************************************** */
static void RssiFilter_UpdateKalman(rssiFilter_t *pFilter, int8_t rssi)
{
    rssiKalman_t *pK = &pFilter->kalman;

    if (!pK->initialized)
    {
        // Initialize with first measurement
        pK->x = (float)rssi;
        pK->initialized = true;
        return;
    }

    // Prediction step
    // x_pred = x (no process model for RSSI)
    // P_pred = P + Q
    float P_pred = pK->P + pK->Q;

    // Update step
    // K = P_pred / (P_pred + R)
    pK->K = P_pred / (P_pred + pK->R);

    // x = x + K * (measurement - x)
    pK->x = pK->x + pK->K * ((float)rssi - pK->x);

    // P = (1 - K) * P_pred
    pK->P = (1.0f - pK->K) * P_pred;
}

/*! *********************************************************************************
* \brief     Update state machine based on filtered RSSI
********************************************************************************** */
static void RssiFilter_UpdateStateMachine(rssiFilter_t *pFilter)
{
    int8_t filteredRssi = RssiFilter_GetFilteredRssi(pFilter);
    rssiState_t newState = pFilter->currentState;

    switch (pFilter->currentState)
    {
        case RssiState_Idle_c:
            if (filteredRssi > RSSI_APPROACH_THRESHOLD)
            {
                pFilter->stableCount++;
                if (pFilter->stableCount >= RSSI_FILTER_STABLE_COUNT)
                {
                    newState = RssiState_Approach_c;
                    pFilter->stableCount = 0;
                }
            }
            else
            {
                pFilter->stableCount = 0;
            }
            break;

        case RssiState_Approach_c:
            if (filteredRssi > RSSI_PROXIMITY_THRESHOLD)
            {
                pFilter->stableCount++;
                if (pFilter->stableCount >= RSSI_FILTER_STABLE_COUNT)
                {
                    newState = RssiState_Proximity_c;
                    pFilter->stableCount = 0;
                }
            }
            else if (filteredRssi < RSSI_APPROACH_THRESHOLD - 5) // Hysteresis
            {
                pFilter->stableCount++;
                if (pFilter->stableCount >= RSSI_FILTER_STABLE_COUNT)
                {
                    newState = RssiState_Lost_c;
                    pFilter->stableCount = 0;
                }
            }
            else
            {
                pFilter->stableCount = 0;
            }
            break;

        case RssiState_Proximity_c:
            if (filteredRssi < RSSI_PROXIMITY_THRESHOLD - 5) // Hysteresis
            {
                pFilter->stableCount++;
                if (pFilter->stableCount >= RSSI_FILTER_STABLE_COUNT)
                {
                    newState = RssiState_Approach_c;
                    pFilter->stableCount = 0;
                }
            }
            else
            {
                pFilter->stableCount = 0;
            }
            break;

        case RssiState_Lost_c:
            if (filteredRssi > RSSI_APPROACH_THRESHOLD)
            {
                pFilter->stableCount++;
                if (pFilter->stableCount >= RSSI_FILTER_STABLE_COUNT)
                {
                    newState = RssiState_Approach_c;
                    pFilter->stableCount = 0;
                }
            }
            else
            {
                pFilter->stableCount = 0;
            }
            break;

        default:
            newState = RssiState_Idle_c;
            break;
    }

    // Check for state change
    if (newState != pFilter->currentState)
    {
        pFilter->previousState = pFilter->currentState;
        pFilter->currentState = newState;
        pFilter->stateChanged = true;
    }
}
