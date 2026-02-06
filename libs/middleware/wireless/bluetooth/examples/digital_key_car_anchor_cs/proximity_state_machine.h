/*! *********************************************************************************
* \file proximity_state_machine.h
*
* Proximity state machine for KW47 keyless entry system
* Manages the tiered handover between RSSI monitoring and Channel Sounding
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef PROXIMITY_STATE_MACHINE_H
#define PROXIMITY_STATE_MACHINE_H

#include "EmbeddedTypes.h"
#include "rssi_filter.h"
#include "fsl_component_timer_manager.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

#define PROXIMITY_UNLOCK_DELAY_MS     2000    // 2 seconds stationary for unlock
#define PROXIMITY_TIMEOUT_MS           10000   // Timeout for proximity state
#define PROXIMITY_MOTION_THRESHOLD     50      // Motion threshold for stationary detection

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

typedef enum
{
    ProximityState_Disconnected_c,    // No device connected
    ProximityState_Monitoring_c,       // RSSI monitoring (low power)
    ProximityState_Approach_c,         // Device approaching (coarse ranging)
    ProximityState_Ranging_c,          // Channel Sounding active
    ProximityState_Proximity_c,        // Device in proximity zone
    ProximityState_Unlock_c,           // Unlock condition met
    ProximityState_Locked_c            // System locked
} proximityState_t;

typedef enum
{
    ProximityEvent_DeviceConnected_c,
    ProximityEvent_DeviceDisconnected_c,
    ProximityEvent_RssiUpdate_c,
    ProximityEvent_MotionDetected_c,
    ProximityEvent_StationaryTimer_c,
    ProximityEvent_Timeout_c,
    ProximityEvent_ManualLock_c,
    ProximityEvent_ManualUnlock_c
} proximityEvent_t;

typedef struct
{
    proximityState_t currentState;
    proximityState_t previousState;
    uint32_t stateEntryTime;
    uint32_t lastRssiUpdateTime;
    bool motionDetected;
    uint32_t stationaryStartTime;
    bool unlockConditionMet;
    rssiFilter_t rssiFilter;
} proximityStateMachine_t;

typedef void (*proximityStateCallback_t)(proximityState_t state, proximityEvent_t event);

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*! *********************************************************************************
* \brief     Initialize proximity state machine
*
* \param[in] pStateMachine Pointer to state machine structure
********************************************************************************** */
void ProximityStateMachine_Init(proximityStateMachine_t *pStateMachine);

/*! *********************************************************************************
* \brief     Register callback for state changes
*
* \param[in] callback Function to call on state changes
********************************************************************************** */
void ProximityStateMachine_RegisterCallback(proximityStateCallback_t callback);

/*! *********************************************************************************
* \brief     Process event in state machine
*
* \param[in] pStateMachine Pointer to state machine structure
* \param[in] event Event to process
* \param[in] pData Optional event data (RSSI value for RSSI events)
********************************************************************************** */
void ProximityStateMachine_ProcessEvent(proximityStateMachine_t *pStateMachine, 
                                       proximityEvent_t event, 
                                       void *pData);

/*! *********************************************************************************
* \brief     Update RSSI measurement
*
* \param[in] pStateMachine Pointer to state machine structure
* \param[in] rssi RSSI measurement in dBm
********************************************************************************** */
void ProximityStateMachine_UpdateRssi(proximityStateMachine_t *pStateMachine, int8_t rssi);

/*! *********************************************************************************
* \brief     Update motion status
*
* \param[in] pStateMachine Pointer to state machine structure
* \param[in] inMotion TRUE if motion detected
********************************************************************************** */
void ProximityStateMachine_UpdateMotion(proximityStateMachine_t *pStateMachine, bool_t inMotion);

/*! *********************************************************************************
* \brief     Get current state
*
* \param[in] pStateMachine Pointer to state machine structure
*
* \return    Current proximity state
********************************************************************************** */
proximityState_t ProximityStateMachine_GetState(proximityStateMachine_t *pStateMachine);

/*! *********************************************************************************
* \brief     Check if unlock condition is met
*
* \param[in] pStateMachine Pointer to state machine structure
*
* \return    TRUE if unlock condition is met
********************************************************************************** */
bool_t ProximityStateMachine_ShouldUnlock(proximityStateMachine_t *pStateMachine);

/*! *********************************************************************************
* \brief     Reset state machine
*
* \param[in] pStateMachine Pointer to state machine structure
********************************************************************************** */
void ProximityStateMachine_Reset(proximityStateMachine_t *pStateMachine);

#ifdef __cplusplus
}
#endif

#endif /* PROXIMITY_STATE_MACHINE_H */
