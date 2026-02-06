/*! *********************************************************************************
* \file rssi_integration.c
*
* Integration of RSSI filtering into Digital Key Car Anchor CS example
*
* Copyright 2025
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "EmbeddedTypes.h"
#include "fsl_component_timer_manager.h"
#include "rssi_filter.h"
#include "proximity_state_machine.h"
#include "gap_interface.h"
#include "fsl_format.h"

/* Shell output - defined in shell file */
#if defined(gAppUseShellInApplication_d) && (gAppUseShellInApplication_d == 1)
#include "fsl_shell.h"
extern SHELL_HANDLE_DEFINE(g_shellHandle);
#define RSSI_PRINT(a) (void)SHELL_PrintfSynchronization((shell_handle_t)g_shellHandle, a)
#define RSSI_DBG(a)   (void)0
#else
#define RSSI_PRINT(a) (void)0
#define RSSI_DBG(a)   (void)0
#endif

/************************************************************************************
*************************************************************************************
* Private variables
*************************************************************************************
************************************************************************************/

static proximityStateMachine_t gProximityStateMachine;
static bool_t gRssiIntegrationInitialized = FALSE;
static bool_t gRssiMonitoringActive = FALSE;
static uint8_t gConnectedDeviceId = 0xFF;

/* Timer for continuous RSSI monitoring */
static TIMER_MANAGER_HANDLE_DEFINE(gRssiTimerHandle);
static bool_t gRssiTimerInitialized = FALSE;

#define RSSI_MONITOR_INTERVAL_MS  100U

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/

static void RssiIntegration_StateCallback(proximityState_t state, proximityEvent_t event);
static void RssiIntegration_TimerCallback(void *pParam);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief     Initialize RSSI integration module
********************************************************************************** */
void RssiIntegration_Init(void)
{
    if (gRssiIntegrationInitialized)
        return;

    // Initialize proximity state machine
    ProximityStateMachine_Init(&gProximityStateMachine);
    ProximityStateMachine_RegisterCallback(RssiIntegration_StateCallback);

    gRssiIntegrationInitialized = TRUE;
}

/*! *********************************************************************************
* \brief     Handle device connected event
********************************************************************************** */
void RssiIntegration_DeviceConnected(uint8_t deviceId)
{
    if (!gRssiIntegrationInitialized)
        RssiIntegration_Init();

    gConnectedDeviceId = deviceId;
    
    // Process connection event
    ProximityStateMachine_ProcessEvent(&gProximityStateMachine, 
                                       ProximityEvent_DeviceConnected_c, 
                                       NULL);
}

/*! *********************************************************************************
* \brief     Handle device disconnected event
********************************************************************************** */
void RssiIntegration_DeviceDisconnected(uint8_t deviceId)
{
    (void)deviceId;
    
    // Stop monitoring if active
    if (gRssiMonitoringActive)
    {
        gRssiMonitoringActive = FALSE;
        if (gRssiTimerInitialized)
        {
            (void)TM_Stop((timer_handle_t)gRssiTimerHandle);
        }
    }
    
    gConnectedDeviceId = 0xFF;
    
    // Process disconnection event
    ProximityStateMachine_ProcessEvent(&gProximityStateMachine, 
                                       ProximityEvent_DeviceDisconnected_c, 
                                       NULL);
}

/*! *********************************************************************************
* \brief     Update RSSI value from connected device
********************************************************************************** */
void RssiIntegration_UpdateRssi(uint8_t deviceId, int8_t rssi)
{
    if (!gRssiIntegrationInitialized)
        return;

    (void)deviceId;

    /* Filter out invalid RSSI values (127 or -127 = error) */
    if (rssi == 127 || rssi == -127)
    {
        return;
    }

    // Get state before update
    rssiState_t oldState = RssiFilter_GetState(&gProximityStateMachine.rssiFilter);

    // Update state machine with new RSSI
    ProximityStateMachine_UpdateRssi(&gProximityStateMachine, rssi);

    // Get new state and filtered value
    rssiState_t newState = RssiFilter_GetState(&gProximityStateMachine.rssiFilter);
    int8_t filtered = RssiFilter_GetFilteredRssi(&gProximityStateMachine.rssiFilter);

    /* Get feature extraction diagnostics (StdDev now Q4, divide by 16 for dB) */
    uint16_t stdQ4 = 0U;
    uint8_t pctAbove = 0U;
    int8_t mean = (int8_t)-100;
    RssiFilter_GetFeatures(&gProximityStateMachine.rssiFilter, &stdQ4, &pctAbove, &mean);

    /* Print diagnostics once every ~0.5 seconds (every 5th sample at 100ms) */
    static uint8_t printCounter = 0U;
    printCounter++;
    if (printCounter >= 5U)
    {
        printCounter = 0U;
        RSSI_PRINT("R:");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)(rssi < 0 ? -rssi : rssi)));
        RSSI_PRINT(" F:");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)(filtered < 0 ? -filtered : filtered)));
        RSSI_PRINT(" M:");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)(mean < 0 ? -mean : mean)));
        RSSI_PRINT(" SD:");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)(stdQ4 / 16U)));
        RSSI_PRINT(".");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)((stdQ4 % 16U) * 10U / 16U)));
        RSSI_PRINT(" P:");
        RSSI_PRINT((const char*)FORMAT_Dec2Str((uint32_t)pctAbove));
        RSSI_PRINT("%\r\n");
    }

    /* Always print state changes immediately */
    if (newState != oldState)
    {
        rssiEvent_t ev = RssiFilter_GetLastEvent(&gProximityStateMachine.rssiFilter);
        RSSI_PRINT("*** ");
        switch(ev)
        {
            case RssiEvent_CandidateStarted_c:
                RSSI_PRINT("CANDIDATE (checking stability)");
                break;
            case RssiEvent_UnlockTriggered_c:
                RSSI_PRINT(">>> UNLOCK TRIGGERED <<< (lockout 5s)");
                break;
            case RssiEvent_ExitToFar_c:
                RSSI_PRINT("EXIT -> FAR/LOCKED (confirmed)");
                break;
            case RssiEvent_None_c: /* fall-through */
            default:
                RSSI_PRINT("STATE -> ");
                switch(newState)
                {
                    case RssiState_Idle_c:     RSSI_PRINT("IDLE"); break;
                    case RssiState_Locked_c:   RSSI_PRINT("FAR/LOCKED"); break;
                    case RssiState_Approach_c:  RSSI_PRINT("CANDIDATE"); break;
                    case RssiState_Unlocked_c:  RSSI_PRINT("LOCKOUT"); break;
                    default:                    RSSI_PRINT("UNKNOWN"); break;
                }
                break;
        }
        RSSI_PRINT(" ***\r\n");
    }
}

/*! *********************************************************************************
* \brief     Get current proximity state
********************************************************************************** */
proximityState_t RssiIntegration_GetState(void)
{
    return ProximityStateMachine_GetState(&gProximityStateMachine);
}

/*! *********************************************************************************
* \brief     Check if unlock should occur
********************************************************************************** */
bool_t RssiIntegration_ShouldUnlock(void)
{
    return ProximityStateMachine_ShouldUnlock(&gProximityStateMachine);
}

/*! *********************************************************************************
* \brief     Print current status
********************************************************************************** */
void RssiIntegration_PrintStatus(void)
{
    if (!gRssiIntegrationInitialized)
    {
        RSSI_DBG("Not initialized");
        return;
    }

    int8_t filtered = RssiFilter_GetFilteredRssi(&gProximityStateMachine.rssiFilter);
    rssiState_t rssiState = RssiFilter_GetState(&gProximityStateMachine.rssiFilter);
    proximityState_t proxState = ProximityStateMachine_GetState(&gProximityStateMachine);

    (void)filtered;
    (void)proxState;
    (void)rssiState;
    RSSI_DBG("Status printed");
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

static void RssiIntegration_StateCallback(proximityState_t state, proximityEvent_t event)
{
    (void)state;
    (void)event;
    RSSI_DBG("State callback");
}

/*! *********************************************************************************
* \brief     Start continuous RSSI monitoring
********************************************************************************** */
void RssiIntegration_StartMonitoring(void)
{
    if (gConnectedDeviceId == 0xFF)
    {
        RSSI_PRINT("\r\n[RSSI] No device connected\r\n");
        return;
    }
    
    if (gRssiMonitoringActive)
    {
        RSSI_PRINT("\r\n[RSSI] Already monitoring\r\n");
        return;
    }
    
    /* Initialize timer if not done */
    if (!gRssiTimerInitialized)
    {
        timer_status_t status = TM_Open((timer_handle_t)gRssiTimerHandle);
        if (status != kStatus_TimerSuccess)
        {
            RSSI_PRINT("\r\n[RSSI] Timer init failed\r\n");
            return;
        }
        gRssiTimerInitialized = TRUE;
    }
    
    /* Install callback and start timer */
    (void)TM_InstallCallback((timer_handle_t)gRssiTimerHandle, 
                             RssiIntegration_TimerCallback, NULL);
    (void)TM_Start((timer_handle_t)gRssiTimerHandle, 
                   (uint8_t)kTimerModeIntervalTimer, 
                   RSSI_MONITOR_INTERVAL_MS);
    
    gRssiMonitoringActive = TRUE;
    RSSI_PRINT("\r\n[RSSI] Monitoring STARTED (100ms)\r\n");
    RSSI_PRINT("[RSSI] Pipeline: Hampel->EMA->StdDev->StateMachine\r\n");
    RSSI_PRINT("[RSSI] R=raw F=filtered M=mean SD=stddev P=pctAbove\r\n");
    
    /* Trigger first read immediately */
    (void)Gap_ReadRssi(gConnectedDeviceId);
}

/*! *********************************************************************************
* \brief     Stop continuous RSSI monitoring
********************************************************************************** */
void RssiIntegration_StopMonitoring(void)
{
    if (!gRssiMonitoringActive)
    {
        RSSI_PRINT("\r\n[RSSI] Not monitoring\r\n");
        return;
    }
    
    gRssiMonitoringActive = FALSE;
    
    if (gRssiTimerInitialized)
    {
        (void)TM_Stop((timer_handle_t)gRssiTimerHandle);
    }
    
    RSSI_PRINT("\r\n[RSSI] Monitoring STOPPED\r\n");
}

/*! *********************************************************************************
* \brief     Timer callback - triggers RSSI read
********************************************************************************** */
static void RssiIntegration_TimerCallback(void *pParam)
{
    (void)pParam;
    
    if (gRssiMonitoringActive && gConnectedDeviceId != 0xFF)
    {
        (void)Gap_ReadRssi(gConnectedDeviceId);
    }
}