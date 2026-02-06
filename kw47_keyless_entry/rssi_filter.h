/*! *********************************************************************************
* \file rssi_filter.h
*
* RSSI filtering implementation for KW47 keyless entry system
* Implements moving average and Kalman filtering for stable proximity detection
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef RSSI_FILTER_H
#define RSSI_FILTER_H

#include "EmbeddedTypes.h"
#include "fsl_component_timer_manager.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

#define RSSI_FILTER_WINDOW_SIZE     8    // Moving average window
#define RSSI_APPROACH_THRESHOLD     -85   // dBm - initial detection
#define RSSI_PROXIMITY_THRESHOLD    -65   // dBm - close proximity
#define RSSI_FILTER_STABLE_COUNT    3    // Consecutive readings for state change

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

typedef enum
{
    RssiState_Idle_c,           // No device detected
    RssiState_Approach_c,       // Device detected, coarse ranging
    RssiState_Proximity_c,      // Device close, ready for unlock
    RssiState_Lost_c            // Device lost
} rssiState_t;

typedef struct
{
    int8_t values[RSSI_FILTER_WINDOW_SIZE];  // RSSI history buffer
    uint8_t index;                           // Current buffer index
    uint8_t count;                           // Number of valid samples
    int8_t movingAverage;                    // Current filtered value
    uint8_t stableCount;                     // Consecutive stable readings
} rssiMovingAverage_t;

typedef struct
{
    // Kalman filter state
    float x;              // Estimated RSSI value
    float P;              // Estimation error covariance
    float Q;              // Process noise covariance
    float R;              // Measurement noise covariance
    float K;              // Kalman gain
    
    // Filter state
    bool initialized;     // Filter initialized flag
} rssiKalman_t;

typedef struct
{
    rssiMovingAverage_t movingAvg;
    rssiKalman_t kalman;
    rssiState_t currentState;
    rssiState_t previousState;
    uint32_t lastUpdateTime;
    bool stateChanged;
} rssiFilter_t;

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*! *********************************************************************************
* \brief     Initialize RSSI filter
*
* \param[in] pFilter Pointer to filter structure
********************************************************************************** */
void RssiFilter_Init(rssiFilter_t *pFilter);

/*! *********************************************************************************
* \brief     Add new RSSI measurement and update filter
*
* \param[in] pFilter Pointer to filter structure
* \param[in] rssi    New RSSI measurement in dBm
********************************************************************************** */
void RssiFilter_AddMeasurement(rssiFilter_t *pFilter, int8_t rssi);

/*! *********************************************************************************
* \brief     Get current filtered RSSI value
*
* \param[in] pFilter Pointer to filter structure
*
* \return    Filtered RSSI value in dBm
********************************************************************************** */
int8_t RssiFilter_GetFilteredRssi(rssiFilter_t *pFilter);

/*! *********************************************************************************
* \brief     Get current proximity state
*
* \param[in] pFilter Pointer to filter structure
*
* \return    Current proximity state
********************************************************************************** */
rssiState_t RssiFilter_GetState(rssiFilter_t *pFilter);

/*! *********************************************************************************
* \brief     Check if state has changed
*
* \param[in] pFilter Pointer to filter structure
*
* \return    TRUE if state changed since last check
********************************************************************************** */
bool_t RssiFilter_HasStateChanged(rssiFilter_t *pFilter);

/*! *********************************************************************************
* \brief     Reset filter to initial state
*
* \param[in] pFilter Pointer to filter structure
********************************************************************************** */
void RssiFilter_Reset(rssiFilter_t *pFilter);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_FILTER_H */
