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

#include "EmbeddedTypes.h"

#ifndef E_OK
typedef uint8_t Std_ReturnType;
#define E_OK       ((Std_ReturnType)0u)
#define E_NOT_OK   ((Std_ReturnType)1u)
#endif

/* NULL pointer macro (MISRA Rule 11.4 compliant) */
#ifndef NULL_PTR
#define NULL_PTR   ((void*)0)
#endif

/* ---------------- Compile-time configuration ---------------- */
#ifndef PROX_RSSI_RAW_CAP
#define PROX_RSSI_RAW_CAP     (64u)
#endif

#ifndef PROX_RSSI_SMOOTH_CAP
#define PROX_RSSI_SMOOTH_CAP  (128u)
#endif

#ifndef PROX_RSSI_ALPHA_LUT_MAX_MS
#define PROX_RSSI_ALPHA_LUT_MAX_MS (1000u)
#endif

#ifndef PROX_RSSI_ALPHA_LUT_STEP_MS
#define PROX_RSSI_ALPHA_LUT_STEP_MS (16u)
#endif

#define PROX_RSSI_ALPHA_LUT_SIZE ((PROX_RSSI_ALPHA_LUT_MAX_MS / PROX_RSSI_ALPHA_LUT_STEP_MS) + 1u)

#define PROX_RSSI_Q4_SCALE          ((int16_t)16)
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
  uint32_t wRawMs;         /* e.g. 2000 */
  uint32_t wSpikeMs;       /* e.g. 800  */
  uint32_t wFeatMs;        /* e.g. 2000 */

  /* Hampel spike rejection: threshold = K * 1.5 * MAD */
  uint16_t hampelKQ4;      /* e.g. 3.0 => 48 */
  uint16_t madEpsQ4;       /* floor MAD, e.g. 0.5 => 8 */

  /* Thresholds (Q4 dB) */
  int16_t enterNearQ4;    /* calibrated */
  int16_t exitNearQ4;     /* if 0, derived */
  uint16_t hystQ4;         /* e.g. 5 dB => 80 */

  /* Stability gate */
  uint16_t pctThQ15;       /* e.g. 0.80 => 26214 */
  uint16_t stdThQ4;        /* e.g. 2.5 dB => 40 */
  uint32_t stableMs;       /* e.g. 2000 */
  uint16_t minFeatSamples; /* e.g. 6 */

  /* State machine */
  uint32_t exitConfirmMs;  /* e.g. 1500 */
  uint32_t lockoutMs;      /* e.g. 7000 */

  /* Time anomaly handling */
  uint32_t maxReasonableDtMs; /* e.g. 2000; if dt > this => full reset */
} ProxRssi_ParamsType;

typedef struct
{
  uint16_t n;
  uint16_t pctAboveEnterQ15;
  uint16_t stdQ4;
  int16_t lastQ4;
  int16_t minQ4;
  int16_t maxQ4;
} ProxRssi_FeaturesType;

typedef struct
{
  uint32_t tMs[PROX_RSSI_RAW_CAP];
  int8_t  rssiDbm[PROX_RSSI_RAW_CAP];
  uint16_t head;
  uint16_t count;
} ProxRssi_RawBufType;

typedef struct
{
  uint32_t tMs[PROX_RSSI_SMOOTH_CAP];
  int16_t rssiQ4[PROX_RSSI_SMOOTH_CAP];
  uint16_t head;
  uint16_t count;
} ProxRssi_SmoothBufType;

typedef struct
{
  ProxRssi_ParamsType p;
  ProxRssi_StateType  st;

  uint32_t tCandidateStartMs;
  uint32_t tBelowExitStartMs;
  uint32_t tLockoutUntilMs;

  bool_t emaValid;
  int16_t emaQ4;
  uint32_t emaPrevMs;

  ProxRssi_RawBufType    raw;
  ProxRssi_SmoothBufType smooth;

  uint16_t alphaQ15[PROX_RSSI_ALPHA_LUT_SIZE];

  /* Temp arrays (no malloc) */
  int16_t tmpA[PROX_RSSI_RAW_CAP];
  int16_t tmpB[PROX_RSSI_RAW_CAP];
  int16_t tmpS[PROX_RSSI_SMOOTH_CAP];
} ProxRssi_CtxType;

/* Helpers */
int16_t ProxRssi_DbmToQ4(int8_t dbm);
int16_t ProxRssi_DbToQ4(int16_t db);

/* API */
Std_ReturnType ProxRssi_Init(ProxRssi_CtxType* Ctx,
                            const ProxRssi_ParamsType* Params,
                            const uint16_t* AlphaQ15Lut,
                            uint32_t AlphaLutLen);

Std_ReturnType ProxRssi_PushRaw(ProxRssi_CtxType* Ctx, uint32_t tMs, int8_t rssiDbm);

Std_ReturnType ProxRssi_MainFunction(ProxRssi_CtxType* Ctx, uint32_t nowMs,
                                     ProxRssi_EventType* Event,
                                     ProxRssi_FeaturesType* Features);

Std_ReturnType ProxRssi_ForceFar(ProxRssi_CtxType* Ctx);

#endif /* PROX_RSSI_H */
