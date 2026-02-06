/*! *********************************************************************************
* \file proximity_state_machine.c
*
* Proximity state machine for KW47 keyless entry system
* Manages the tiered handover between RSSI monitoring and Channel Sounding
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "proximity_state_machine.h"
#include "FunctionLib.h"
#include "fsl_component_timer_manager.h"
#include "fsl_debug_console.h"

/************************************************************************************
*************************************************************************************
* Private variables
*************************************************************************************
************************************************************************************/

static proximityStateCallback_t gStateCallback = NULL;

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

static void ProximityStateMachine_EnterState(proximityStateMachine_t *pStateMachine, 
                                            proximityState_t newState);
static void ProximityStateMachine_HandleDisconnected(proximityStateMachine_t *pStateMachine, 
                                                     proximityEvent_t event, 
                                                     void *pData);
static void ProximityStateMachine_HandleMonitoring(proximityStateMachine_t *pStateMachine, 
                                                  proximityEvent_t event, 
                                                  void *pData);
static void ProximityStateMachine_HandleApproach(proximityStateMachine_t *pStateMachine, 
                                                 proximityEvent_t event, 
                                                 void *pData);
static void ProximityStateMachine_HandleRanging(proximityStateMachine_t *pStateMachine, 
                                               proximityEvent_t event, 
                                               void *pData);
static void ProximityStateMachine_HandleProximity(proximityStateMachine_t *pStateMachine, 
                                                 proximityEvent_t event, 
                                                 void *pData);
static void ProximityStateMachine_HandleUnlock(proximityStateMachine_t *pStateMachine, 
                                              proximityEvent_t event, 
                                              void *pData);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief     Initialize proximity state machine
********************************************************************************** */
void ProximityStateMachine_Init(proximityStateMachine_t *pStateMachine)
{
    if (pStateMachine == NULL)
        return;

    pStateMachine->currentState = ProximityState_Disconnected_c;
    pStateMachine->previousState = ProximityState_Disconnected_c;
    pStateMachine->stateEntryTime = TM_GetTimestamp();
    pStateMachine->lastRssiUpdateTime = 0;
    pStateMachine->motionDetected = false;
    pStateMachine->stationaryStartTime = 0;
    pStateMachine->unlockConditionMet = false;

    // Initialize RSSI filter
    RssiFilter_Init(&pStateMachine->rssiFilter);
}

/*! *********************************************************************************
* \brief     Register callback for state changes
********************************************************************************** */
void ProximityStateMachine_RegisterCallback(proximityStateCallback_t callback)
{
    gStateCallback = callback;
}

/*! *********************************************************************************
* \brief     Process event in state machine
********************************************************************************** */
void ProximityStateMachine_ProcessEvent(proximityStateMachine_t *pStateMachine, 
                                       proximityEvent_t event, 
                                       void *pData)
{
    if (pStateMachine == NULL)
        return;

    switch (pStateMachine->currentState)
    {
        case ProximityState_Disconnected_c:
            ProximityStateMachine_HandleDisconnected(pStateMachine, event, pData);
            break;

        case ProximityState_Monitoring_c:
            ProximityStateMachine_HandleMonitoring(pStateMachine, event, pData);
            break;

        case ProximityState_Approach_c:
            ProximityStateMachine_HandleApproach(pStateMachine, event, pData);
            break;

        case ProximityState_Ranging_c:
            ProximityStateMachine_HandleRanging(pStateMachine, event, pData);
            break;

        case ProximityState_Proximity_c:
            ProximityStateMachine_HandleProximity(pStateMachine, event, pData);
            break;

        case ProximityState_Unlock_c:
            ProximityStateMachine_HandleUnlock(pStateMachine, event, pData);
            break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief     Update RSSI measurement
********************************************************************************** */
void ProximityStateMachine_UpdateRssi(proximityStateMachine_t *pStateMachine, int8_t rssi)
{
    if (pStateMachine == NULL)
        return;

    // Update RSSI filter
    RssiFilter_AddMeasurement(&pStateMachine->rssiFilter, rssi);
    pStateMachine->lastRssiUpdateTime = TM_GetTimestamp();

    // Process RSSI update event
    ProximityStateMachine_ProcessEvent(pStateMachine, ProximityEvent_RssiUpdate_c, &rssi);
}

/*! *********************************************************************************
* \brief     Update motion status
********************************************************************************** */
void ProximityStateMachine_UpdateMotion(proximityStateMachine_t *pStateMachine, bool_t inMotion)
{
    if (pStateMachine == NULL)
        return;

    pStateMachine->motionDetected = inMotion;

    if (inMotion)
    {
        pStateMachine->stationaryStartTime = 0;
        ProximityStateMachine_ProcessEvent(pStateMachine, ProximityEvent_MotionDetected_c, NULL);
    }
    else
    {
        if (pStateMachine->stationaryStartTime == 0)
        {
            pStateMachine->stationaryStartTime = TM_GetTimestamp();
        }
    }
}

/*! *********************************************************************************
* \brief     Get current state
********************************************************************************** */
proximityState_t ProximityStateMachine_GetState(proximityStateMachine_t *pStateMachine)
{
    if (pStateMachine == NULL)
        return ProximityState_Disconnected_c;

    return pStateMachine->currentState;
}

/*! *********************************************************************************
* \brief     Check if unlock condition is met
********************************************************************************** */
bool_t ProximityStateMachine_ShouldUnlock(proximityStateMachine_t *pStateMachine)
{
    if (pStateMachine == NULL)
        return false;

    return pStateMachine->unlockConditionMet;
}

/*! *********************************************************************************
* \brief     Reset state machine
********************************************************************************** */
void ProximityStateMachine_Reset(proximityStateMachine_t *pStateMachine)
{
    if (pStateMachine == NULL)
        return;

    ProximityStateMachine_Init(pStateMachine);
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief     Enter new state
********************************************************************************** */
static void ProximityStateMachine_EnterState(proximityStateMachine_t *pStateMachine, 
                                            proximityState_t newState)
{
    pStateMachine->previousState = pStateMachine->currentState;
    pStateMachine->currentState = newState;
    pStateMachine->stateEntryTime = TM_GetTimestamp();
    pStateMachine->unlockConditionMet = false;

    // Call callback if registered
    if (gStateCallback != NULL)
    {
        gStateCallback(newState, ProximityEvent_DeviceConnected_c); // Use generic event
    }

    PRINTF("Proximity State: %s -> %s\r\n", 
           (pStateMachine->previousState == ProximityState_Disconnected_c) ? "Disconnected" :
           (pStateMachine->previousState == ProximityState_Monitoring_c) ? "Monitoring" :
           (pStateMachine->previousState == ProximityState_Approach_c) ? "Approach" :
           (pStateMachine->previousState == ProximityState_Ranging_c) ? "Ranging" :
           (pStateMachine->previousState == ProximityState_Proximity_c) ? "Proximity" :
           (pStateMachine->previousState == ProximityState_Unlock_c) ? "Unlock" : "Unknown",
           (newState == ProximityState_Disconnected_c) ? "Disconnected" :
           (newState == ProximityState_Monitoring_c) ? "Monitoring" :
           (newState == ProximityState_Approach_c) ? "Approach" :
           (newState == ProximityState_Ranging_c) ? "Ranging" :
           (newState == ProximityState_Proximity_c) ? "Proximity" :
           (newState == ProximityState_Unlock_c) ? "Unlock" : "Unknown");
}

/*! *********************************************************************************
* \brief     Handle Disconnected state
********************************************************************************** */
static void ProximityStateMachine_HandleDisconnected(proximityStateMachine_t *pStateMachine, 
                                                     proximityEvent_t event, 
                                                     void *pData)
{
    switch (event)
    {
        case ProximityEvent_DeviceConnected_c:
            ProximityStateMachine_EnterState(pStateMachine, ProximityState_Monitoring_c);
            break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief     Handle Monitoring state
********************************************************************************** */
static void ProximityStateMachine_HandleMonitoring(proximityStateMachine_t *pStateMachine, 
                                                  proximityEvent_t event, 
                                                  void *pData)
{
    switch (event)
    {
        case ProximityEvent_RssiUpdate_c:
        {
            rssiState_t rssiState = RssiFilter_GetState(&pStateMachine->rssiFilter);
            if (rssiState == RssiState_Approach_c)
            {
                ProximityStateMachine_EnterState(pStateMachine, ProximityState_Approach_c);
            }
            break;
        }

        case ProximityEvent_DeviceDisconnected_c:
            ProximityStateMachine_EnterState(pStateMachine, ProximityState_Disconnected_c);
            break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief     Handle Approach state
********************************************************************************** */
static void ProximityStateMachine_HandleApproach(proximityStateMachine_t *pStateMachine, 
                                                 proximityEvent_t event, 
                                                 void *pData)
{
    switch (event)
    {
        case ProximityEvent_RssiUpdate_c:
        {
            rssiState_t rssiState = RssiFilter_GetState(&pStateMachine->rssiFilter);
            if (rssiState == RssiState_Unlocked_c)
            {
                // Transition to Proximity state when very close
                ProximityStateMachine_EnterState(pStateMachine, ProximityState_Proximity_c);
            }
            else if (rssiState == RssiState_Locked_c)
            {
                ProximityStateMachine_EnterState(pStateMachine, ProximityState_Monitoring_c);
            }
            break;
        }

        case ProximityEvent_DeviceDisconnected_c:
            ProximityStateMachine_EnterState(pStateMachine, ProximityState_Disconnected_c);
            break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief     Handle Ranging state (future CS implementation)
********************************************************************************** */
static void ProximityStateMachine_HandleRanging(proximityStateMachine_t *pStateMachine, 
                                               proximityEvent_t event, 
                                               void *pData)
{
    switch (event)
    {
        case ProximityEvent_RssiUpdate_c:
            // Handle CS results here in future implementation
            break;

        case ProximityEvent_DeviceDisconnected_c:
            ProximityStateMachine_EnterState(pStateMachine, ProximityState_Disconnected_c);
            break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief     Handle Proximity state
********************************************************************************** */
static void ProximityStateMachine_HandleProximity(proximityStateMachine_t *pStateMachine, 
                                                 proximityEvent_t event, 
                                                 void *pData)
{
    switch (event)
    {
        case ProximityEvent_RssiUpdate_c:
        {
            rssiState_t rssiState = RssiFilter_GetState(&pStateMachine->rssiFilter);
            if (rssiState != RssiState_Unlocked_c)
            {
                ProximityStateMachine_EnterState(pStateMachine, ProximityState_Approach_c);
            }
            break;
        }

        case ProximityEvent_MotionDetected_c:
            // Reset stationary timer when motion detected
            pStateMachine->stationaryStartTime = 0;
            break;

        case ProximityEvent_StationaryTimer_c:
            // Check if device has been stationary for unlock time
            if (!pStateMachine->motionDetected && 
                (TM_GetTimestamp() - pStateMachine->stationaryStartTime) >= PROXIMITY_UNLOCK_DELAY_MS)
            {
                pStateMachine->unlockConditionMet = true;
                ProximityStateMachine_EnterState(pStateMachine, ProximityState_Unlock_c);
            }
            break;

        case ProximityEvent_DeviceDisconnected_c:
            ProximityStateMachine_EnterState(pStateMachine, ProximityState_Disconnected_c);
            break;

        default:
            break;
    }
}

/*! *********************************************************************************
* \brief     Handle Unlock state
********************************************************************************** */
static void ProximityStateMachine_HandleUnlock(proximityStateMachine_t *pStateMachine, 
                                              proximityEvent_t event, 
                                              void *pData)
{
    switch (event)
    {
        case ProximityEvent_ManualLock_c:
        case ProximityEvent_DeviceDisconnected_c:
            ProximityStateMachine_EnterState(pStateMachine, ProximityState_Locked_c);
            break;

        case ProximityEvent_RssiUpdate_c:
        {
            rssiState_t rssiState = RssiFilter_GetState(&pStateMachine->rssiFilter);
            if (rssiState != RssiState_Unlocked_c)
            {
                ProximityStateMachine_EnterState(pStateMachine, ProximityState_Locked_c);
            }
            break;
        }

        default:
            break;
    }
}
