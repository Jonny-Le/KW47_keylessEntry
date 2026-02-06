#ifndef PROX_RSSI_H
#define PROX_RSSI_H
/*
===============================================================================
 ProxRssi - BLE RSSI proximity filtering + "stable for 2s" decision
 Safety first (automotive-friendly), speed second.

 Target: NXP KW47 (ARM Cortex-M), ThreadX task context recommended.
 Math:   Fixed-point only. No float. Deterministic memory.

 FIXED-POINT FORMATS
 -------------------
 - RSSI Q4:      rssiQ4 = rssi_dBm * 16   (1/16 dB resolution)
 - Alpha Q15:    0..32767  => 0.0..~1.0
 - Percent Q15:  0..32767  => 0.0..~1.0

 SAFETY NOTE
 ----------
 RSSI proximity must NOT be the sole unlock criterion. Always run a secure
 cryptographic handshake before actuating the latch.

 USAGE (ThreadX typical)
 -----------------------
 1) Init once:
    ProxRssi_Init(&ctx, &params, alphaLutQ15, alphaLen);

 2) On each RSSI sample (from a worker thread, not ISR):
    ProxRssi_PushRaw(&ctx, tMs, rssiDbm);
    ProxRssi_MainFunction(&ctx, tMs, &ev, &feat);
    if (ev == PROX_RSSI_EVT_UNLOCK_TRIGGERED) { ... start secure handshake ... }

 CALIBRATION
 -----------
 - enterNearQ4: threshold at ~2 m (phone to anchor)
 - exitNearQ4:  enterNearQ4 - hystQ4 (hysteresis)
 - pctThQ15:    fraction of samples in last wFeatMs above enterNearQ4
 - stdThQ4:     std-dev (Q4 dB) maximum allowed for stability

===============================================================================
*/

#include <stdint.h>

/* ---------------- AUTOSAR-like basic types (portable) ----------------
 * These short names (uint8, sint16, …) are NOT in NXP EmbeddedTypes.h,
 * so we always typedef them here. TRUE/FALSE/boolean are guarded to
 * coexist with EmbeddedTypes.h which defines them differently.
 * -------------------------------------------------------------------- */
typedef uint8_t  uint8;
typedef int8_t   sint8;
typedef uint16_t uint16;
typedef int16_t  sint16;
typedef uint32_t uint32;
typedef int32_t  sint32;
typedef int64_t  sint64;

typedef uint8    boolean;
#ifndef TRUE
#define TRUE       1
#define FALSE      0
#endif

#ifndef E_OK
typedef uint8 Std_ReturnType;
#define E_OK       ((Std_ReturnType)0u)
#define E_NOT_OK   ((Std_ReturnType)1u)
#endif

/* NULL pointer macro (MISRA Rule 11.4 compliant) */
#ifndef NULL_PTR
#define NULL_PTR   ((void*)0)
#endif

/* ---------------- Compile-time configuration ---------------- */
#ifndef PROX_RSSI_RAW_CAP
#define PROX_RSSI_RAW_CAP     (128u)
#endif

#ifndef PROX_RSSI_SMOOTH_CAP
#define PROX_RSSI_SMOOTH_CAP  (128u)
#endif

#ifndef PROX_RSSI_ALPHA_LUT_MAX_MS
#define PROX_RSSI_ALPHA_LUT_MAX_MS (1000u)
#endif

#define PROX_RSSI_Q4_SCALE          ((sint16)16)
#define PROX_RSSI_Q15_ONE           (32767u)

/* ---------------- Public types ---------------- */
typedef enum
{
  PROX_RSSI_ST_FAR       = 0,
  PROX_RSSI_ST_CANDIDATE = 1,
  PROX_RSSI_ST_LOCKOUT   = 2
} ProxRssi_StateType;

typedef enum
{
  PROX_RSSI_EVT_NONE              = 0,
  PROX_RSSI_EVT_CANDIDATE_STARTED = 1,
  PROX_RSSI_EVT_UNLOCK_TRIGGERED  = 2,
  PROX_RSSI_EVT_EXIT_TO_FAR       = 3
} ProxRssi_EventType;

typedef struct
{
  /* Windows (ms) */
  uint32 wRawMs;         /* e.g. 2000 */
  uint32 wSpikeMs;       /* e.g. 800  */
  uint32 wFeatMs;        /* e.g. 2000 */

  /* Hampel spike rejection: threshold = K * 1.5 * MAD */
  uint16 hampelKQ4;      /* e.g. 3.0 => 48 */
  uint16 madEpsQ4;       /* floor MAD, e.g. 0.5 => 8 */

  /* Thresholds (Q4 dB) */
  sint16 enterNearQ4;    /* calibrated */
  sint16 exitNearQ4;     /* if 0, derived */
  uint16 hystQ4;         /* e.g. 5 dB => 80 */

  /* Stability gate */
  uint16 pctThQ15;       /* e.g. 0.80 => 26214 */
  uint16 stdThQ4;        /* e.g. 2.5 dB => 40 */
  uint32 stableMs;       /* e.g. 2000 */
  uint16 minFeatSamples; /* e.g. 6 */

  /* State machine */
  uint32 exitConfirmMs;  /* e.g. 1500 */
  uint32 lockoutMs;      /* e.g. 7000 */

  /* Time anomaly handling */
  uint32 maxReasonableDtMs; /* e.g. 2000; if dt > this => full reset */
} ProxRssi_ParamsType;

typedef struct
{
  uint16 n;
  uint16 pctAboveEnterQ15;
  uint16 stdQ4;
  sint16 lastQ4;
  sint16 minQ4;
  sint16 maxQ4;
} ProxRssi_FeaturesType;

typedef struct
{
  uint32 tMs[PROX_RSSI_RAW_CAP];
  sint8  rssiDbm[PROX_RSSI_RAW_CAP];
  uint16 head;
  uint16 count;
} ProxRssi_RawBufType;

typedef struct
{
  uint32 tMs[PROX_RSSI_SMOOTH_CAP];
  sint16 rssiQ4[PROX_RSSI_SMOOTH_CAP];
  uint16 head;
  uint16 count;
} ProxRssi_SmoothBufType;

typedef struct
{
  ProxRssi_ParamsType p;
  ProxRssi_StateType  st;

  uint32 tCandidateStartMs;
  uint32 tBelowExitStartMs;
  uint32 tLockoutUntilMs;

  boolean emaValid;
  sint16 emaQ4;
  uint32 emaPrevMs;

  ProxRssi_RawBufType    raw;
  ProxRssi_SmoothBufType smooth;

  uint16 alphaQ15[PROX_RSSI_ALPHA_LUT_MAX_MS + 1u];

  /* Temp arrays (no malloc) */
  sint16 tmpA[PROX_RSSI_RAW_CAP];
  sint16 tmpB[PROX_RSSI_RAW_CAP];
  sint16 tmpS[PROX_RSSI_SMOOTH_CAP];
} ProxRssi_CtxType;

/* Helpers */
sint16 ProxRssi_DbmToQ4(sint8 dbm);
sint16 ProxRssi_DbToQ4(sint16 db);

/* API */
Std_ReturnType ProxRssi_Init(ProxRssi_CtxType* Ctx,
                            const ProxRssi_ParamsType* Params,
                            const uint16* AlphaQ15Lut,
                            uint32 AlphaLutLen);

Std_ReturnType ProxRssi_PushRaw(ProxRssi_CtxType* Ctx, uint32 tMs, sint8 rssiDbm);

Std_ReturnType ProxRssi_MainFunction(ProxRssi_CtxType* Ctx, uint32 nowMs,
                                     ProxRssi_EventType* Event,
                                     ProxRssi_FeaturesType* Features);

Std_ReturnType ProxRssi_ForceFar(ProxRssi_CtxType* Ctx);

#endif /* PROX_RSSI_H */
