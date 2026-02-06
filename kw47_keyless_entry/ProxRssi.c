#include "ProxRssi.h"

/* ============================================================
 * Safety-first utilities
 * ============================================================ */

/* Wrap-safe time difference: returns (a - b) in uint32, OK across wrap */
static uint32 ProxRssi_TimeDiff(uint32 a, uint32 b)
{
  return (uint32)(a - b);
}

/* Deterministic ring helper (safe modulo). Speed second. */
static uint16 ProxRssi_RingNext(uint16 idx, uint16 cap)
{
  idx++;
  if (idx >= cap) { idx = 0u; }
  return idx;
}

static uint16 ProxRssi_RingTail(uint16 head, uint16 count, uint16 cap)
{
  /* tail = (head - count) mod cap */
  uint32 h = (uint32)head;
  uint32 c = (uint32)count;
  uint32 t = (h + (uint32)cap - (c % (uint32)cap)) % (uint32)cap;
  return (uint16)t;
}

/* Q15(alpha) * Q4(delta) -> Q4 */
static sint16 ProxRssi_MulAlphaQ15_DeltaQ4(uint16 alphaQ15, sint16 deltaQ4)
{
  const sint32 prod = ((sint32)alphaQ15) * ((sint32)deltaQ4);
  return (sint16)(prod >> 15);
}

/* Insertion sort: deterministic, simple to audit; N is bounded (<=128). */
static void ProxRssi_InsertionSortS16(sint16* a, uint16 n)
{
  uint16 i;
  for (i = 1u; i < n; i++)
  {
    const sint16 key = a[i];
    uint16 j = i;
    while ((j > 0u) && (a[(uint16)(j - 1u)] > key))
    {
      a[j] = a[(uint16)(j - 1u)];
      j--;
    }
    a[j] = key;
  }
}

static sint16 ProxRssi_MedianSortedS16(const sint16* aSorted, uint16 n)
{
  return aSorted[(uint16)(n >> 1)];
}

/* Integer sqrt (deterministic) */
static uint16 ProxRssi_IsqrtU32(uint32 x)
{
  uint32 op = x;
  uint32 res = 0u;
  uint32 one = 1uL << 30;

  while (one > op) { one >>= 2; }
  while (one != 0u)
  {
    if (op >= (res + one))
    {
      op -= (res + one);
      res = res + 2u * one;
    }
    res >>= 1;
    one >>= 2;
  }
  return (uint16)res;
}

/* ============================================================
 * Public conversions
 * ============================================================ */
sint16 ProxRssi_DbToQ4(sint16 db)
{
  return (sint16)(db * (sint16)PROX_RSSI_Q4_SCALE);
}

sint16 ProxRssi_DbmToQ4(sint8 dbm)
{
  return (sint16)((sint16)dbm * (sint16)PROX_RSSI_Q4_SCALE);
}

/* ============================================================
 * Internal buffer push/prune
 * ============================================================ */
static void ProxRssi_RawPush(ProxRssi_CtxType* Ctx, uint32 tMs, sint8 rssiDbm)
{
  Ctx->raw.tMs[Ctx->raw.head] = tMs;
  Ctx->raw.rssiDbm[Ctx->raw.head] = rssiDbm;

  Ctx->raw.head = ProxRssi_RingNext(Ctx->raw.head, (uint16)PROX_RSSI_RAW_CAP);
  if (Ctx->raw.count < (uint16)PROX_RSSI_RAW_CAP) { Ctx->raw.count++; }
}

static void ProxRssi_SmoothPush(ProxRssi_CtxType* Ctx, uint32 tMs, sint16 rssiQ4)
{
  Ctx->smooth.tMs[Ctx->smooth.head] = tMs;
  Ctx->smooth.rssiQ4[Ctx->smooth.head] = rssiQ4;

  Ctx->smooth.head = ProxRssi_RingNext(Ctx->smooth.head, (uint16)PROX_RSSI_SMOOTH_CAP);
  if (Ctx->smooth.count < (uint16)PROX_RSSI_SMOOTH_CAP) { Ctx->smooth.count++; }
}

static void ProxRssi_RawPrune(ProxRssi_CtxType* Ctx, uint32 nowMs, uint32 winMs)
{
  if (Ctx->raw.count == 0u) { return; }

  uint16 tail = ProxRssi_RingTail(Ctx->raw.head, Ctx->raw.count, (uint16)PROX_RSSI_RAW_CAP);
  uint16 remaining = Ctx->raw.count;

  /* Keep samples with t >= now - win */
  const uint32 minT = (ProxRssi_TimeDiff(nowMs, 0u) >= winMs) ? (nowMs - winMs) : 0u;

  while (remaining > 0u)
  {
    if (Ctx->raw.tMs[tail] >= minT) { break; }
    tail = ProxRssi_RingNext(tail, (uint16)PROX_RSSI_RAW_CAP);
    remaining--;
  }
  Ctx->raw.count = remaining;
}

static void ProxRssi_SmoothPrune(ProxRssi_CtxType* Ctx, uint32 nowMs, uint32 winMs)
{
  if (Ctx->smooth.count == 0u) { return; }

  uint16 tail = ProxRssi_RingTail(Ctx->smooth.head, Ctx->smooth.count, (uint16)PROX_RSSI_SMOOTH_CAP);
  uint16 remaining = Ctx->smooth.count;

  const uint32 minT = (ProxRssi_TimeDiff(nowMs, 0u) >= winMs) ? (nowMs - winMs) : 0u;

  while (remaining > 0u)
  {
    if (Ctx->smooth.tMs[tail] >= minT) { break; }
    tail = ProxRssi_RingNext(tail, (uint16)PROX_RSSI_SMOOTH_CAP);
    remaining--;
  }
  Ctx->smooth.count = remaining;
}

/* Copy windows into temp arrays for median/MAD/features */
static Std_ReturnType ProxRssi_CopyRawWindowQ4(const ProxRssi_CtxType* Ctx,
                                               uint32 nowMs, uint32 winMs,
                                               sint16* outQ4, uint16 cap,
                                               uint16* outN)
{
  uint16 n = 0u;
  if (Ctx->raw.count == 0u) { *outN = 0u; return E_NOT_OK; }

  const uint32 minT = (ProxRssi_TimeDiff(nowMs, 0u) >= winMs) ? (nowMs - winMs) : 0u;

  uint16 idx = ProxRssi_RingTail(Ctx->raw.head, Ctx->raw.count, (uint16)PROX_RSSI_RAW_CAP);
  uint16 i;

  for (i = 0u; i < Ctx->raw.count; i++)
  {
    if (Ctx->raw.tMs[idx] >= minT)
    {
      if (n < cap)
      {
        outQ4[n] = ProxRssi_DbmToQ4(Ctx->raw.rssiDbm[idx]);
        n++;
      }
    }
    idx = ProxRssi_RingNext(idx, (uint16)PROX_RSSI_RAW_CAP);
  }

  *outN = n;
  return (n >= 3u) ? E_OK : E_NOT_OK;
}

static Std_ReturnType ProxRssi_CopySmoothWindowQ4(const ProxRssi_CtxType* Ctx,
                                                  uint32 nowMs, uint32 winMs,
                                                  sint16* outQ4, uint16 cap,
                                                  uint16* outN, sint16* outLastQ4)
{
  uint16 n = 0u;
  sint16 last = (sint16)0;

  if (Ctx->smooth.count == 0u) { *outN = 0u; *outLastQ4 = (sint16)0; return E_NOT_OK; }

  const uint32 minT = (ProxRssi_TimeDiff(nowMs, 0u) >= winMs) ? (nowMs - winMs) : 0u;

  uint16 idx = ProxRssi_RingTail(Ctx->smooth.head, Ctx->smooth.count, (uint16)PROX_RSSI_SMOOTH_CAP);
  uint16 i;

  for (i = 0u; i < Ctx->smooth.count; i++)
  {
    if (Ctx->smooth.tMs[idx] >= minT)
    {
      if (n < cap)
      {
        outQ4[n] = Ctx->smooth.rssiQ4[idx];
        last = Ctx->smooth.rssiQ4[idx];
        n++;
      }
    }
    idx = ProxRssi_RingNext(idx, (uint16)PROX_RSSI_SMOOTH_CAP);
  }

  *outN = n;
  *outLastQ4 = last;
  return (n >= Ctx->p.minFeatSamples) ? E_OK : E_NOT_OK;
}

/* Hampel spike reject (safety-first): sort + median + MAD (no QuickSelect). */
static Std_ReturnType ProxRssi_HampelSpikeReject(ProxRssi_CtxType* Ctx, uint32 nowMs, sint16* outQ4)
{
  uint16 n;
  Std_ReturnType r = ProxRssi_CopyRawWindowQ4(Ctx, nowMs, Ctx->p.wSpikeMs,
                                              Ctx->tmpA, (uint16)PROX_RSSI_RAW_CAP, &n);
  if (r != E_OK) { return E_NOT_OK; }

  ProxRssi_InsertionSortS16(Ctx->tmpA, n);
  const sint16 medQ4 = ProxRssi_MedianSortedS16(Ctx->tmpA, n);

  uint16 i;
  for (i = 0u; i < n; i++)
  {
    const sint16 d = (sint16)(Ctx->tmpA[i] - medQ4);
    Ctx->tmpB[i] = (d < 0) ? (sint16)(-d) : d;
  }
  ProxRssi_InsertionSortS16(Ctx->tmpB, n);
  sint16 madQ4 = ProxRssi_MedianSortedS16(Ctx->tmpB, n);
  if (madQ4 < (sint16)Ctx->p.madEpsQ4) { madQ4 = (sint16)Ctx->p.madEpsQ4; }

  /* threshold = K * 1.5 * MAD; K is Q4, MAD is Q4 */
  const sint32 prodQ8 = ((sint32)(sint16)Ctx->p.hampelKQ4) * (sint32)madQ4; /* Q8 */
  const sint32 thrQ8  = (prodQ8 * 3) / 2;                                  /* *1.5 */
  const sint16 thrQ4  = (sint16)(thrQ8 / (sint32)PROX_RSSI_Q4_SCALE);

  const uint16 lastIdx = (Ctx->raw.head == 0u) ? ((uint16)PROX_RSSI_RAW_CAP - 1u) : (uint16)(Ctx->raw.head - 1u);
  const sint16 xLatestQ4 = ProxRssi_DbmToQ4(Ctx->raw.rssiDbm[lastIdx]);

  const sint16 diff = (sint16)(xLatestQ4 - medQ4);
  const sint16 absDiff = (diff < 0) ? (sint16)(-diff) : diff;

  *outQ4 = (absDiff > thrQ4) ? medQ4 : xLatestQ4;
  return E_OK;
}

/* EMA update (fixed-point, LUT alpha) */
static uint16 ProxRssi_AlphaQ15FromDt(const ProxRssi_CtxType* Ctx, uint32 dtMs)
{
  uint32 idx = dtMs;
  if (idx > (uint32)PROX_RSSI_ALPHA_LUT_MAX_MS) { idx = (uint32)PROX_RSSI_ALPHA_LUT_MAX_MS; }
  return Ctx->alphaQ15[idx];
}

static void ProxRssi_EmaUpdate(ProxRssi_CtxType* Ctx, uint32 nowMs, sint16 xQ4, sint16* outEmaQ4)
{
  if (Ctx->emaValid == FALSE)
  {
    Ctx->emaValid = TRUE;
    Ctx->emaQ4 = xQ4;
    Ctx->emaPrevMs = nowMs;
    *outEmaQ4 = xQ4;
    return;
  }

  const uint32 dtMs = ProxRssi_TimeDiff(nowMs, Ctx->emaPrevMs);

  /* Time anomaly => full reset (safety-first) */
  if ((dtMs == 0u) || (dtMs > Ctx->p.maxReasonableDtMs))
  {
    Ctx->emaQ4 = xQ4;
    Ctx->emaPrevMs = nowMs;
    *outEmaQ4 = xQ4;
    return;
  }

  const uint16 aQ15 = ProxRssi_AlphaQ15FromDt(Ctx, dtMs);

  const sint16 eQ4 = Ctx->emaQ4;
  const sint16 deltaQ4 = (sint16)(xQ4 - eQ4);
  const sint16 stepQ4  = ProxRssi_MulAlphaQ15_DeltaQ4(aQ15, deltaQ4);

  Ctx->emaQ4 = (sint16)(eQ4 + stepQ4);
  Ctx->emaPrevMs = nowMs;
  *outEmaQ4 = Ctx->emaQ4;
}

/* Features: pctAbove + std, deterministic. */
static Std_ReturnType ProxRssi_ComputeFeatures(ProxRssi_CtxType* Ctx, uint32 nowMs, ProxRssi_FeaturesType* outF)
{
  uint16 n;
  sint16 lastQ4;

  Std_ReturnType r = ProxRssi_CopySmoothWindowQ4(Ctx, nowMs, Ctx->p.wFeatMs,
                                                 Ctx->tmpS, (uint16)PROX_RSSI_SMOOTH_CAP,
                                                 &n, &lastQ4);
  if (r != E_OK) { return E_NOT_OK; }

  /* sum/sumsq in 64-bit for safety */
  sint64 sumQ4 = (sint64)0;
  sint64 sumSqQ8 = (sint64)0;

  sint16 mn = Ctx->tmpS[0];
  sint16 mx = Ctx->tmpS[0];
  uint32 cntAbove = 0u;

  const sint16 enterQ4 = Ctx->p.enterNearQ4;

  uint16 i;
  for (i = 0u; i < n; i++)
  {
    const sint16 xQ4 = Ctx->tmpS[i];
    sumQ4  = sumQ4 + (sint64)xQ4;
    sumSqQ8 = sumSqQ8 + ((sint64)xQ4 * (sint64)xQ4);

    if (xQ4 >= enterQ4) { cntAbove++; }
    if (xQ4 < mn) { mn = xQ4; }
    if (xQ4 > mx) { mx = xQ4; }
  }

  /* variance in Q8: (sumSq - sum^2/n) / (n-1) */
  uint32 stdQ4 = 0u;
  if (n > 1u)
  {
    const sint64 meanSqTerm = (sumQ4 * sumQ4) / (sint64)n;
    sint64 diff = sumSqQ8 - meanSqTerm;
    if (diff < (sint64)0) { diff = (sint64)0; }
    const uint32 varQ8 = (uint32)(diff / (sint64)(n - 1u));
    stdQ4 = ProxRssi_IsqrtU32(varQ8);
  }

  outF->n = n;
  outF->pctAboveEnterQ15 = (uint16)((cntAbove * (uint32)PROX_RSSI_Q15_ONE) / (uint32)n);
  outF->stdQ4 = (uint16)stdQ4;
  outF->lastQ4 = lastQ4;
  outF->minQ4 = mn;
  outF->maxQ4 = mx;
  return E_OK;
}

static boolean ProxRssi_IsStable(const ProxRssi_CtxType* Ctx, const ProxRssi_FeaturesType* f)
{
  boolean result = FALSE;
  if ((f->pctAboveEnterQ15 >= Ctx->p.pctThQ15) && (f->stdQ4 <= Ctx->p.stdThQ4))
  {
    result = TRUE;
  }
  return result;
}

/* State machine */
static ProxRssi_EventType ProxRssi_StateStep(ProxRssi_CtxType* Ctx, uint32 nowMs, const ProxRssi_FeaturesType* f)
{
  const sint16 lastQ4  = f->lastQ4;
  const sint16 enterQ4 = Ctx->p.enterNearQ4;
  const sint16 exitQ4  = Ctx->p.exitNearQ4;

  if (Ctx->st == PROX_RSSI_ST_LOCKOUT)
  {
    if (nowMs < Ctx->tLockoutUntilMs) { return PROX_RSSI_EVT_NONE; }

    if (lastQ4 < exitQ4)
    {
      if (Ctx->tBelowExitStartMs == 0u) { Ctx->tBelowExitStartMs = nowMs; }
      if (ProxRssi_TimeDiff(nowMs, Ctx->tBelowExitStartMs) >= Ctx->p.exitConfirmMs)
      {
        Ctx->st = PROX_RSSI_ST_FAR;
        Ctx->tBelowExitStartMs = 0u;
        return PROX_RSSI_EVT_EXIT_TO_FAR;
      }
    }
    else
    {
      Ctx->tBelowExitStartMs = 0u;
    }
    return PROX_RSSI_EVT_NONE;
  }

  if (Ctx->st == PROX_RSSI_ST_FAR)
  {
    if (lastQ4 >= enterQ4)
    {
      Ctx->st = PROX_RSSI_ST_CANDIDATE;
      Ctx->tCandidateStartMs = nowMs;
      Ctx->tBelowExitStartMs = 0u;
      return PROX_RSSI_EVT_CANDIDATE_STARTED;
    }
    return PROX_RSSI_EVT_NONE;
  }

  /* CANDIDATE */
  if (lastQ4 < exitQ4)
  {
    if (Ctx->tBelowExitStartMs == 0u) { Ctx->tBelowExitStartMs = nowMs; }
    if (ProxRssi_TimeDiff(nowMs, Ctx->tBelowExitStartMs) >= Ctx->p.exitConfirmMs)
    {
      Ctx->st = PROX_RSSI_ST_FAR;
      Ctx->tBelowExitStartMs = 0u;
      Ctx->tCandidateStartMs = 0u;
      return PROX_RSSI_EVT_EXIT_TO_FAR;
    }
  }
  else
  {
    Ctx->tBelowExitStartMs = 0u;
  }

  if (ProxRssi_IsStable(Ctx, f) == TRUE)
  {
    if (ProxRssi_TimeDiff(nowMs, Ctx->tCandidateStartMs) >= Ctx->p.stableMs)
    {
      Ctx->st = PROX_RSSI_ST_LOCKOUT;
      Ctx->tLockoutUntilMs = nowMs + Ctx->p.lockoutMs;
      Ctx->tBelowExitStartMs = 0u;
      return PROX_RSSI_EVT_UNLOCK_TRIGGERED;
    }
  }
  else
  {
    Ctx->tCandidateStartMs = nowMs; /* reset hold timer */
  }

  return PROX_RSSI_EVT_NONE;
}

/* ============================================================
 * Public API
 * ============================================================ */
Std_ReturnType ProxRssi_Init(ProxRssi_CtxType* Ctx,
                            const ProxRssi_ParamsType* Params,
                            const uint16* AlphaQ15Lut,
                            uint32 AlphaLutLen)
{
  uint32 i;

  if ((Ctx == NULL_PTR) ||
      (Params == NULL_PTR) ||
      (AlphaQ15Lut == NULL_PTR) ||
      (AlphaLutLen == 0u))
  {
    return E_NOT_OK;
  }

  Ctx->p = *Params;

  /* Defensive defaults */
  if (Ctx->p.wRawMs == 0u)   { Ctx->p.wRawMs = 2000u; }
  if (Ctx->p.wSpikeMs == 0u) { Ctx->p.wSpikeMs = 800u; }
  if (Ctx->p.wFeatMs == 0u)  { Ctx->p.wFeatMs = 2000u; }

  if (Ctx->p.hystQ4 == 0u)   { Ctx->p.hystQ4 = (uint16)ProxRssi_DbToQ4(5); }
  if (Ctx->p.exitNearQ4 == 0) { Ctx->p.exitNearQ4 = (sint16)(Ctx->p.enterNearQ4 - (sint16)Ctx->p.hystQ4); }

  if (Ctx->p.stableMs == 0u)      { Ctx->p.stableMs = 2000u; }
  if (Ctx->p.exitConfirmMs == 0u) { Ctx->p.exitConfirmMs = 1500u; }
  if (Ctx->p.lockoutMs == 0u)     { Ctx->p.lockoutMs = 7000u; }
  if (Ctx->p.minFeatSamples == 0u){ Ctx->p.minFeatSamples = 6u; }

  if (Ctx->p.maxReasonableDtMs == 0u) { Ctx->p.maxReasonableDtMs = 2000u; }

  /* Copy alpha LUT and clamp */
  if (AlphaLutLen > ((uint32)PROX_RSSI_ALPHA_LUT_MAX_MS + 1u))
  {
    AlphaLutLen = ((uint32)PROX_RSSI_ALPHA_LUT_MAX_MS + 1u);
  }

  for (i = 0u; i < AlphaLutLen; i++) { Ctx->alphaQ15[i] = AlphaQ15Lut[i]; }
  for (i = AlphaLutLen; i <= (uint32)PROX_RSSI_ALPHA_LUT_MAX_MS; i++) { Ctx->alphaQ15[i] = Ctx->alphaQ15[AlphaLutLen - 1u]; }

  /* Reset state */
  Ctx->st = PROX_RSSI_ST_FAR;
  Ctx->tCandidateStartMs = 0u;
  Ctx->tBelowExitStartMs = 0u;
  Ctx->tLockoutUntilMs = 0u;

  Ctx->emaValid = FALSE;
  Ctx->emaQ4 = (sint16)0;
  Ctx->emaPrevMs = 0u;

  Ctx->raw.head = 0u;
  Ctx->raw.count = 0u;
  Ctx->smooth.head = 0u;
  Ctx->smooth.count = 0u;

  return E_OK;
}

Std_ReturnType ProxRssi_PushRaw(ProxRssi_CtxType* Ctx, uint32 tMs, sint8 rssiDbm)
{
  if (Ctx == NULL_PTR) { return E_NOT_OK; }

  /* BLE Core Spec: 127 (0x7F) = "not available"; reject non-negative too */
  if ((rssiDbm == (sint8)127) || (rssiDbm >= (sint8)0))
  {
    return E_NOT_OK;
  }

  /* clamp to valid BLE range */
  if (rssiDbm < (sint8)-127) { rssiDbm = (sint8)-127; }

  ProxRssi_RawPush(Ctx, tMs, rssiDbm);
  return E_OK;
}

Std_ReturnType ProxRssi_MainFunction(ProxRssi_CtxType* Ctx, uint32 nowMs,
                                     ProxRssi_EventType* Event,
                                     ProxRssi_FeaturesType* Features)
{
  ProxRssi_FeaturesType f;
  ProxRssi_EventType ev = PROX_RSSI_EVT_NONE;

  if ((Ctx == NULL_PTR) || (Event == NULL_PTR))
  {
    return E_NOT_OK;
  }

  /* prune */
  ProxRssi_RawPrune(Ctx, nowMs, Ctx->p.wRawMs);
  ProxRssi_SmoothPrune(Ctx, nowMs, Ctx->p.wFeatMs);

  /* default features */
  f.n = 0u; f.pctAboveEnterQ15 = 0u; f.stdQ4 = 0u; f.lastQ4 = (sint16)0; f.minQ4 = (sint16)0; f.maxQ4 = (sint16)0;

  if (Ctx->raw.count == 0u)
  {
    *Event = PROX_RSSI_EVT_NONE;
    if (Features != NULL_PTR) { *Features = f; }
    return E_OK;
  }

  /* Hampel */
  sint16 xQ4;
  if (ProxRssi_HampelSpikeReject(Ctx, nowMs, &xQ4) != E_OK)
  {
    *Event = PROX_RSSI_EVT_NONE;
    if (Features != NULL_PTR) { *Features = f; }
    return E_OK;
  }

  /* EMA */
  sint16 emaQ4;
  ProxRssi_EmaUpdate(Ctx, nowMs, xQ4, &emaQ4);

  /* Smooth push */
  ProxRssi_SmoothPush(Ctx, nowMs, emaQ4);
  ProxRssi_SmoothPrune(Ctx, nowMs, Ctx->p.wFeatMs);

  /* Features + state */
  if (ProxRssi_ComputeFeatures(Ctx, nowMs, &f) == E_OK)
  {
    ev = ProxRssi_StateStep(Ctx, nowMs, &f);
  }

  *Event = ev;
  if (Features != NULL_PTR) { *Features = f; }
  return E_OK;
}

Std_ReturnType ProxRssi_ForceFar(ProxRssi_CtxType* Ctx)
{
  if (Ctx == NULL_PTR) { return E_NOT_OK; }

  Ctx->st = PROX_RSSI_ST_FAR;
  Ctx->tCandidateStartMs = 0u;
  Ctx->tBelowExitStartMs = 0u;
  Ctx->tLockoutUntilMs = 0u;

  Ctx->emaValid = FALSE;
  Ctx->emaQ4 = (sint16)0;
  Ctx->emaPrevMs = 0u;

  Ctx->raw.head = 0u;
  Ctx->raw.count = 0u;
  Ctx->smooth.head = 0u;
  Ctx->smooth.count = 0u;

  return E_OK;
}
