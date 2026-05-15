/*
 * Smart Parking Garage Gate System - Tiva-C TM4C123GH6PM + FreeRTOS
 *
 * Features:
 *  - [SWITCH] Driver Open (PF4) : hardware toggle ON => open, OFF => stop midway
 *  - [BUTTONS] In AUTO mode (ENABLE_AUTO_MODE 1):
 *        short tap => one-touch auto, long press => manual stop on release.
 *      In PURE MANUAL mode (ENABLE_AUTO_MODE 0):
 *        Driver Close, Security Open/Close act as toggle switches:
 *           1st tap = hold (gate moves), 2nd tap = release (gate stops midway).
 *    A second command while moving in the same direction always stops midway.
 *  - Open/Closed limit: single press works in any mode => idle (LEDs off)
 *  - Obstacle: stop, reverse opposite direction 0.5s, stop midway
 *  - Security panel priority (persistent during security-initiated motion)
 *  - Same-panel conflict => safe stop
 *  - Cross-panel conflict (driver + security simultaneously active):
 *      Security overrides immediately; any driver-initiated motion is stopped
 *      and security takes full control.
 *  - RGB LED (auto mode):
 *      Green  = opening / reversing
 *      Red    = closing
 *      Off    = stopped / idle
 *  - RGB LED (manual mode):
 *      Green  = opening / reversing
 *      Red    = closing
 *      Blue   = stopped midway
 *      Off    = idle (fully open or fully closed)
 *
 * Hardware mapping:
 * *  Buttons:
 *    PF4 = Driver OPEN       active-low, onboard SW1 pull-up
 *    PE0 = Driver CLOSE      active-high, external pull-down
 *    PE1 = Security OPEN     active-high, external pull-down
 *    PB0 = Security CLOSE    active-high, external pull-down
 *    PB1 = Open Limit        active-high, external pull-down
 *    PD0 = Closed Limit      active-high, external pull-down
 *    PD1 = Obstacle          active-high, external pull-down
 *
 * FIX (cross-panel conflict):
 *  Previously, when driver inputs and security inputs were simultaneously active,
 *  the code relied on if/else branching to give security priority for new commands,
 *  but it did NOT stop any ongoing driver-initiated motion. Security now immediately
 *  sends CMD_STOP (SOURCE_SECURITY) the first tick it detects a cross-panel conflict,
 *  halting driver motion before processing security commands. This applies to both
 *  AUTO and MANUAL mode.
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ---------- Compile-time mode selection ---------- */
#define ENABLE_AUTO_MODE  1    /* 1 = auto + manual, 0 = pure manual (toggle buttons) */

/* ===================== GPIO DEFINITIONS ===================== */

#define LED_RED      (1U << 1)   /* PF1 */
#define LED_GREEN    (1U << 3)   /* PF3 */
#define LED_BLUE     (1U << 2)   /* PF2 */
#define LED_MASK     (LED_RED | LED_GREEN | LED_BLUE)

#define BTN_DRIVER_OPEN      (1U << 4)   /* PF4, active-low SWITCH */
#define BTN_DRIVER_CLOSE     (1U << 0)   /* PE0 */
#define BTN_SECURITY_OPEN    (1U << 1)   /* PE1 */
#define BTN_SECURITY_CLOSE   (1U << 0)   /* PB0 */
#define BTN_OPEN_LIMIT       (1U << 1)   /* PB1 */
#define BTN_CLOSED_LIMIT     (1U << 0)   /* PD0 */
#define BTN_OBSTACLE         (1U << 1)   /* PD1 */

#define RCGCGPIO_B   (1U << 1)
#define RCGCGPIO_D   (1U << 3)
#define RCGCGPIO_E   (1U << 4)
#define RCGCGPIO_F   (1U << 5)
#define RCGCGPIO_ALL (RCGCGPIO_B | RCGCGPIO_D | RCGCGPIO_E | RCGCGPIO_F)

/* ===================== RTOS SETTINGS ===================== */

#define INPUT_TASK_PRIORITY     (3U)
#define GATE_TASK_PRIORITY      (2U)
#define LED_TASK_PRIORITY       (2U)
#define SAFETY_TASK_PRIORITY    (4U)

#define INPUT_PERIOD_MS         (20U)
#define HOLD_THRESHOLD_MS       (350U)
#define REVERSE_TIME_MS         (500U)

#define INPUT_PERIOD_TICKS      pdMS_TO_TICKS(INPUT_PERIOD_MS)
#define HOLD_THRESHOLD_TICKS    pdMS_TO_TICKS(HOLD_THRESHOLD_MS)
#define REVERSE_TIME_TICKS      pdMS_TO_TICKS(REVERSE_TIME_MS)

/* ===================== TYPES ===================== */

typedef enum {
    GATE_IDLE_OPEN = 0,
    GATE_IDLE_CLOSED,
    GATE_OPENING,
    GATE_CLOSING,
    GATE_STOPPED_MIDWAY,
    GATE_REVERSING
} GateState_t;

typedef enum {
    MOTION_STOP = 0,
    MOTION_OPEN,
    MOTION_CLOSE
} Motion_t;

typedef enum {
    CMD_NONE = 0,
    CMD_OPEN,
    CMD_CLOSE,
    CMD_STOP,
    CMD_OPEN_LIMIT,
    CMD_CLOSED_LIMIT,
    CMD_OBSTACLE
} CommandType_t;

typedef enum {
    SOURCE_DRIVER = 0,
    SOURCE_SECURITY,
    SOURCE_LIMIT,
    SOURCE_SAFETY
} CommandSource_t;

typedef struct {
    CommandType_t   type;
    CommandSource_t source;
    bool            isAuto;
} GateCommand_t;

typedef struct {
    bool driverOpen;
    bool driverClose;
    bool securityOpen;
    bool securityClose;
    bool openLimit;
    bool closedLimit;
    bool obstacle;
} ButtonSnapshot_t;

/* ===================== GLOBAL RTOS OBJECTS ===================== */

static QueueHandle_t      xGateQueue;
static SemaphoreHandle_t  xOpenLimitSem;
static SemaphoreHandle_t  xClosedLimitSem;
static SemaphoreHandle_t  xGateStateMutex;

static volatile Motion_t  g_motion      = MOTION_STOP;
static GateState_t        g_gateState   = GATE_IDLE_CLOSED;
static bool               g_autoClosing = false;
static Motion_t           g_lastMotionBeforeObstacle = MOTION_STOP;
static volatile bool      g_securityMotionActive     = false;

/* ===================== GPIO HELPERS ===================== */

static void GPIO_Init(void)
{
    SYSCTL_RCGCGPIO_R |= RCGCGPIO_ALL;
    while ((SYSCTL_PRGPIO_R & RCGCGPIO_ALL) != RCGCGPIO_ALL) {}

    /* Port F: PF1 red, PF2 blue, PF3 green outputs; PF4 driver open switch input */
    GPIO_PORTF_AMSEL_R &= ~(BTN_DRIVER_OPEN | LED_MASK);
    GPIO_PORTF_PCTL_R  &= ~0x000F0FF0U;
    GPIO_PORTF_AFSEL_R &= ~(BTN_DRIVER_OPEN | LED_MASK);
    GPIO_PORTF_DIR_R   |=  LED_MASK;
    GPIO_PORTF_DIR_R   &= ~BTN_DRIVER_OPEN;
    GPIO_PORTF_PUR_R   |=  BTN_DRIVER_OPEN;
    GPIO_PORTF_DEN_R   |=  BTN_DRIVER_OPEN | LED_MASK;
    GPIO_PORTF_DATA_R  &= ~LED_MASK;

    /* Port E: PE0 driver close, PE1 security open */
    GPIO_PORTE_AMSEL_R &= ~(BTN_DRIVER_CLOSE | BTN_SECURITY_OPEN);
    GPIO_PORTE_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTE_AFSEL_R &= ~(BTN_DRIVER_CLOSE | BTN_SECURITY_OPEN);
    GPIO_PORTE_DIR_R   &= ~(BTN_DRIVER_CLOSE | BTN_SECURITY_OPEN);
    GPIO_PORTE_PDR_R   |=  (BTN_DRIVER_CLOSE | BTN_SECURITY_OPEN);
    GPIO_PORTE_DEN_R   |=  (BTN_DRIVER_CLOSE | BTN_SECURITY_OPEN);

    /* Port B: PB0 security close, PB1 open limit */
    GPIO_PORTB_AMSEL_R &= ~(BTN_SECURITY_CLOSE | BTN_OPEN_LIMIT);
    GPIO_PORTB_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTB_AFSEL_R &= ~(BTN_SECURITY_CLOSE | BTN_OPEN_LIMIT);
    GPIO_PORTB_DIR_R   &= ~(BTN_SECURITY_CLOSE | BTN_OPEN_LIMIT);
    GPIO_PORTB_PDR_R   |=  (BTN_SECURITY_CLOSE | BTN_OPEN_LIMIT);
    GPIO_PORTB_DEN_R   |=  (BTN_SECURITY_CLOSE | BTN_OPEN_LIMIT);

    /* Port D: PD0 closed limit, PD1 obstacle */
    GPIO_PORTD_AMSEL_R &= ~(BTN_CLOSED_LIMIT | BTN_OBSTACLE);
    GPIO_PORTD_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTD_AFSEL_R &= ~(BTN_CLOSED_LIMIT | BTN_OBSTACLE);
    GPIO_PORTD_DIR_R   &= ~(BTN_CLOSED_LIMIT | BTN_OBSTACLE);
    GPIO_PORTD_PDR_R   |=  (BTN_CLOSED_LIMIT | BTN_OBSTACLE);
    GPIO_PORTD_DEN_R   |=  (BTN_CLOSED_LIMIT | BTN_OBSTACLE);
}

static inline bool ReadDriverOpen(void)    { return (GPIO_PORTF_DATA_R & BTN_DRIVER_OPEN)    == 0U; }
static inline bool ReadDriverClose(void)   { return (GPIO_PORTE_DATA_R & BTN_DRIVER_CLOSE)   != 0U; }
static inline bool ReadSecurityOpen(void)  { return (GPIO_PORTE_DATA_R & BTN_SECURITY_OPEN)  != 0U; }
static inline bool ReadSecurityClose(void) { return (GPIO_PORTB_DATA_R & BTN_SECURITY_CLOSE) != 0U; }
static inline bool ReadOpenLimit(void)     { return (GPIO_PORTB_DATA_R & BTN_OPEN_LIMIT)     != 0U; }
static inline bool ReadClosedLimit(void)   { return (GPIO_PORTD_DATA_R & BTN_CLOSED_LIMIT)   != 0U; }
static inline bool ReadObstacle(void)      { return (GPIO_PORTD_DATA_R & BTN_OBSTACLE)       != 0U; }

static ButtonSnapshot_t ReadButtons(void)
{
    ButtonSnapshot_t b;
    b.driverOpen    = ReadDriverOpen();
    b.driverClose   = ReadDriverClose();
    b.securityOpen  = ReadSecurityOpen();
    b.securityClose = ReadSecurityClose();
    b.openLimit     = ReadOpenLimit();
    b.closedLimit   = ReadClosedLimit();
    b.obstacle      = ReadObstacle();
    return b;
}

static void LED_Set(uint32_t mask)
{
    GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~LED_MASK) | (mask & LED_MASK);
}

/* ===================== STATE HELPERS ===================== */

static void SetGateState(GateState_t state)
{
    if (xSemaphoreTake(xGateStateMutex, portMAX_DELAY) == pdTRUE) {
        g_gateState = state;
        xSemaphoreGive(xGateStateMutex);
    }
}

static GateState_t GetGateState(void)
{
    GateState_t state;
    if (xSemaphoreTake(xGateStateMutex, portMAX_DELAY) == pdTRUE) {
        state = g_gateState;
        xSemaphoreGive(xGateStateMutex);
    } else {
        state = GATE_STOPPED_MIDWAY;
    }
    return state;
}

static void StopGate(void)
{
    g_motion      = MOTION_STOP;
    g_autoClosing = false;
}

static void OpenGate(bool isReverse)
{
    g_motion      = MOTION_OPEN;
    g_autoClosing = false;
    SetGateState(isReverse ? GATE_REVERSING : GATE_OPENING);
}

static void CloseGate(bool autoMode)
{
    g_motion      = MOTION_CLOSE;
    g_autoClosing = autoMode;
    SetGateState(GATE_CLOSING);
}

/* ===================== TASKS ===================== */

static void vInputTask(void *pvParameters)
{
    (void)pvParameters;

    ButtonSnapshot_t prev = {0};

#if ENABLE_AUTO_MODE == 1
    /* --- Auto mode variables --- */
    TickType_t driverOpenPressTick    = 0;
    TickType_t driverClosePressTick   = 0;
    TickType_t securityOpenPressTick  = 0;
    TickType_t securityClosePressTick = 0;
    CommandSource_t activeSource = SOURCE_DRIVER;

    /*
     * FIX (auto mode): track whether we already sent a cross-panel override stop
     * this tick so we don't spam the queue. Reset each time the cross-panel
     * conflict clears.
     */
    bool crossPanelStopSent = false;

#else
    /* --- Manual mode variables --- */
    TickType_t drvClosePressTime  = 0;   /* unused in manual, silences warnings */
    TickType_t secOpenPressTime   = 0;
    TickType_t secClosePressTime  = 0;
    bool drvCloseLatched  = false;
    bool secOpenLatched   = false;
    bool secCloseLatched  = false;

    /*
     * FIX (manual mode): same one-shot flag for cross-panel override stop.
     */
    bool crossPanelStopSent = false;
#endif

    for (;;)
    {
        ButtonSnapshot_t cur = ReadButtons();
        TickType_t       now = xTaskGetTickCount();
        GateCommand_t    cmd;

#if ENABLE_AUTO_MODE == 1
        /* ===== AUTO MODE INPUT TASK ===== */

        bool driverPressed   = cur.driverOpen   || cur.driverClose;
        bool securityPressed = cur.securityOpen || cur.securityClose;

        /* -- 1. Limit buttons (always processed, highest natural priority) --- */
        if (cur.openLimit && !prev.openLimit)
        {
            xSemaphoreGive(xOpenLimitSem);
            cmd.type   = CMD_OPEN_LIMIT;
            cmd.source = SOURCE_LIMIT;
            cmd.isAuto = false;
            xQueueSend(xGateQueue, &cmd, 0);
        }
        if (cur.closedLimit && !prev.closedLimit)
        {
            xSemaphoreGive(xClosedLimitSem);
            cmd.type   = CMD_CLOSED_LIMIT;
            cmd.source = SOURCE_LIMIT;
            cmd.isAuto = false;
            xQueueSend(xGateQueue, &cmd, 0);
        }

        /* -- 2. Cross-panel conflict: driver AND security simultaneously active.
         *
         *    Security overrides. We must first stop any ongoing driver-initiated
         *    motion before letting security process its command. We send one
         *    CMD_STOP (SOURCE_SECURITY) the moment the conflict is first detected,
         *    then let security handle the rest normally this tick (and every tick
         *    while the conflict persists). The stop is NOT re-sent on subsequent
         *    ticks while the conflict persists to avoid flooding the queue.
         *
         *    Once the conflict clears (driver releases), crossPanelStopSent is
         *    reset so a future conflict is caught again.
         */
        if (driverPressed && securityPressed)
        {
            if (!crossPanelStopSent)
            {
                cmd.type   = CMD_STOP;
                cmd.source = SOURCE_SECURITY;   /* security-sourced stop */
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
                crossPanelStopSent = true;
            }

            /* Security takes/keeps ownership */
            activeSource = SOURCE_SECURITY;

            /* Now let security process its edges normally this same tick.
             * Driver edges are intentionally NOT processed. */
            if (cur.securityOpen && !prev.securityOpen)
            {
                securityOpenPressTick = now;
                cmd.type   = CMD_OPEN;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (!cur.securityOpen && prev.securityOpen)
            {
                bool wasShort = (now - securityOpenPressTick) < HOLD_THRESHOLD_TICKS;
                cmd.type   = wasShort ? CMD_OPEN : CMD_STOP;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = wasShort;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (cur.securityClose && !prev.securityClose)
            {
                securityClosePressTick = now;
                cmd.type   = CMD_CLOSE;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (!cur.securityClose && prev.securityClose)
            {
                bool wasShort = (now - securityClosePressTick) < HOLD_THRESHOLD_TICKS;
                cmd.type   = wasShort ? CMD_CLOSE : CMD_STOP;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = wasShort;
                xQueueSend(xGateQueue, &cmd, 0);
            }

            prev = cur;
            vTaskDelay(INPUT_PERIOD_TICKS);
            continue;
        }

        /* Conflict cleared: reset the one-shot flag */
        crossPanelStopSent = false;

        /* -- 3. Same-panel conflict => stop --- */
        if ((cur.securityOpen && cur.securityClose) ||
            (cur.driverOpen   && cur.driverClose))
        {
            cmd.type   = CMD_STOP;
            cmd.source = (cur.securityOpen && cur.securityClose)
                         ? SOURCE_SECURITY : SOURCE_DRIVER;
            cmd.isAuto = false;
            xQueueSend(xGateQueue, &cmd, 0);
            prev = cur;
            vTaskDelay(INPUT_PERIOD_TICKS);
            continue;
        }

        /* -- 4. Security-only pressed => security takes/keeps control --- */
        if (securityPressed)
        {
            activeSource = SOURCE_SECURITY;

            if (cur.securityOpen && !prev.securityOpen)
            {
                securityOpenPressTick = now;
                cmd.type   = CMD_OPEN;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (!cur.securityOpen && prev.securityOpen)
            {
                bool wasShort = (now - securityOpenPressTick) < HOLD_THRESHOLD_TICKS;
                cmd.type   = wasShort ? CMD_OPEN : CMD_STOP;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = wasShort;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (cur.securityClose && !prev.securityClose)
            {
                securityClosePressTick = now;
                cmd.type   = CMD_CLOSE;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (!cur.securityClose && prev.securityClose)
            {
                bool wasShort = (now - securityClosePressTick) < HOLD_THRESHOLD_TICKS;
                cmd.type   = wasShort ? CMD_CLOSE : CMD_STOP;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = wasShort;
                xQueueSend(xGateQueue, &cmd, 0);
            }
        }

        /* -- 5. Driver-only pressed => driver handles (when security not active) --- */
        else if (driverPressed)
        {
            /*
             * Block driver while a security-initiated motion is still active
             * (security panel may have released but gate is still moving under
             *  security authority).
             */
            if (g_securityMotionActive)
            {
                prev = cur;
                vTaskDelay(INPUT_PERIOD_TICKS);
                continue;
            }

            activeSource = SOURCE_DRIVER;

            if (cur.driverOpen && !prev.driverOpen)
            {
                driverOpenPressTick = now;
                cmd.type   = CMD_OPEN;
                cmd.source = SOURCE_DRIVER;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (!cur.driverOpen && prev.driverOpen)
            {
                bool wasShort = (now - driverOpenPressTick) < HOLD_THRESHOLD_TICKS;
                cmd.type   = wasShort ? CMD_OPEN : CMD_STOP;
                cmd.source = SOURCE_DRIVER;
                cmd.isAuto = wasShort;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (cur.driverClose && !prev.driverClose)
            {
                driverClosePressTick = now;
                cmd.type   = CMD_CLOSE;
                cmd.source = SOURCE_DRIVER;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (!cur.driverClose && prev.driverClose)
            {
                bool wasShort = (now - driverClosePressTick) < HOLD_THRESHOLD_TICKS;
                cmd.type   = wasShort ? CMD_CLOSE : CMD_STOP;
                cmd.source = SOURCE_DRIVER;
                cmd.isAuto = wasShort;
                xQueueSend(xGateQueue, &cmd, 0);
            }
        }

        /* -- 6. Nothing pressed => reset ownership --- */
        else
        {
            activeSource = SOURCE_DRIVER;
        }

#else
        /* ===== MANUAL MODE INPUT TASK ===== */

        /* -- 1. Limit switches (always processed) --- */
        if (cur.openLimit && !prev.openLimit) {
            xSemaphoreGive(xOpenLimitSem);
            cmd.type   = CMD_OPEN_LIMIT;
            cmd.source = SOURCE_LIMIT;
            cmd.isAuto = false;
            xQueueSend(xGateQueue, &cmd, 0);
        }
        if (cur.closedLimit && !prev.closedLimit) {
            xSemaphoreGive(xClosedLimitSem);
            cmd.type   = CMD_CLOSED_LIMIT;
            cmd.source = SOURCE_LIMIT;
            cmd.isAuto = false;
            xQueueSend(xGateQueue, &cmd, 0);
        }

        /* -- 2. Determine if security panel is active --- */
        bool securityPanelActive = (cur.securityOpen || cur.securityClose);
        bool driverActive        = (cur.driverOpen   || cur.driverClose);
        bool ignoreDriver        = securityPanelActive || g_securityMotionActive;

        /* -- 3. Cross-panel conflict: driver AND security simultaneously active.
         *
         *    Security overrides. Send one CMD_STOP (SOURCE_SECURITY) the moment
         *    the conflict is first detected to immediately halt any driver-initiated
         *    motion. The flag resets once the conflict clears (driver releases),
         *    so future conflicts are caught again.
         */
        if (driverActive && securityPanelActive)
        {
            if (!crossPanelStopSent)
            {
                cmd.type   = CMD_STOP;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
                crossPanelStopSent = true;
            }
            /* Fall through: security panel processing below handles security edges. */
        }
        else
        {
            /* Conflict cleared: reset the one-shot flag */
            crossPanelStopSent = false;
        }

        /* -- 4. Security panel processing (runs whenever security is active,
         *       including during a cross-panel conflict) --- */
        if (securityPanelActive)
        {
            if (cur.securityOpen && !prev.securityOpen) {
                secOpenLatched = !secOpenLatched;
                cmd.type   = secOpenLatched ? CMD_OPEN : CMD_STOP;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            if (cur.securityClose && !prev.securityClose) {
                secCloseLatched = !secCloseLatched;
                cmd.type   = secCloseLatched ? CMD_CLOSE : CMD_STOP;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
            }
            /* Both latched simultaneously => conflict stop */
            if (secOpenLatched && secCloseLatched) {
                cmd.type   = CMD_STOP;
                cmd.source = SOURCE_SECURITY;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
                secOpenLatched  = false;
                secCloseLatched = false;
            }
        }

        /* -- 5. Driver panel (only when not ignored) --- */
        else if (!ignoreDriver)
        {
            /* Driver OPEN switch */
            if (cur.driverOpen && !prev.driverOpen) {
                GateState_t state = GetGateState();
                if (state != GATE_OPENING) {
                    cmd.type   = CMD_OPEN;
                    cmd.source = SOURCE_DRIVER;
                    cmd.isAuto = false;
                    xQueueSend(xGateQueue, &cmd, 0);
                }
            }
            else if (!cur.driverOpen && prev.driverOpen) {
                cmd.type   = CMD_STOP;
                cmd.source = SOURCE_DRIVER;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);
            }

            /* Driver CLOSE toggle button */
            if (cur.driverClose && !prev.driverClose) {
                drvCloseLatched = !drvCloseLatched;
                cmd.type   = drvCloseLatched ? CMD_CLOSE : CMD_STOP;
                cmd.source = SOURCE_DRIVER;
                cmd.isAuto = false;
                xQueueSend(xGateQueue, &cmd, 0);

                /* Conflict: close latched while open switch is still on */
                if (cur.driverOpen && drvCloseLatched) {
                    GateState_t st = GetGateState();
                    if (st == GATE_IDLE_OPEN) {
                        cmd.type   = CMD_CLOSE;
                        cmd.source = SOURCE_DRIVER;
                        cmd.isAuto = false;
                        xQueueSend(xGateQueue, &cmd, 0);
                    } else {
                        cmd.type   = CMD_STOP;
                        cmd.source = SOURCE_DRIVER;
                        cmd.isAuto = false;
                        xQueueSend(xGateQueue, &cmd, 0);
                        drvCloseLatched = false;
                    }
                }
            }
        }

        /* suppress unused-variable warnings in manual mode */
        (void)drvClosePressTime;
        (void)secOpenPressTime;
        (void)secClosePressTime;

#endif  /* ENABLE_AUTO_MODE */

        prev = cur;
        vTaskDelay(INPUT_PERIOD_TICKS);
    }
}

/* ===================== SAFETY TASK (identical in both modes) ===================== */

static void vSafetyTask(void *pvParameters)
{
    (void)pvParameters;
    GateCommand_t cmd;

    for (;;)
    {
        if (ReadObstacle()) {
            GateState_t state = GetGateState();
            if ((state == GATE_CLOSING) || (state == GATE_OPENING)) {
                cmd.type   = CMD_OBSTACLE;
                cmd.source = SOURCE_SAFETY;
                cmd.isAuto = false;
                xQueueSendToFront(xGateQueue, &cmd, 0);
            }
        }
        if (xSemaphoreTake(xOpenLimitSem, 0) == pdPASS) {
            if (GetGateState() == GATE_OPENING) {
                cmd.type   = CMD_OPEN_LIMIT;
                cmd.source = SOURCE_LIMIT;
                cmd.isAuto = false;
                xQueueSendToFront(xGateQueue, &cmd, 0);
            }
        }
        if (xSemaphoreTake(xClosedLimitSem, 0) == pdPASS) {
            if (GetGateState() == GATE_CLOSING) {
                cmd.type   = CMD_CLOSED_LIMIT;
                cmd.source = SOURCE_LIMIT;
                cmd.isAuto = false;
                xQueueSendToFront(xGateQueue, &cmd, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ===================== GATE CONTROL TASK ===================== */

static void vGateControlTask(void *pvParameters)
{
    (void)pvParameters;
    GateCommand_t cmd;

    for (;;)
    {
        if (xQueueReceive(xGateQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            GateState_t state = GetGateState();

            switch (cmd.type)
            {
                case CMD_OPEN:
                    if (state == GATE_IDLE_OPEN)
                    {
                        StopGate();
                    }
                    else if (state == GATE_CLOSING)
                    {
                        /* Opposite command while closing => stop midway */
                        StopGate();
                        SetGateState(GATE_STOPPED_MIDWAY);
                        g_securityMotionActive = false;
                    }
#if ENABLE_AUTO_MODE == 0
                    else if (state == GATE_OPENING)
                    {
                        /* Manual: second CMD_OPEN while opening => stop midway */
                        StopGate();
                        SetGateState(GATE_STOPPED_MIDWAY);
                        g_securityMotionActive = false;
                    }
#endif
                    else
                    {
                        g_securityMotionActive = (cmd.source == SOURCE_SECURITY);
                        OpenGate(false);
                    }
                    break;

                case CMD_CLOSE:
                    if (state == GATE_IDLE_CLOSED)
                    {
                        StopGate();
                    }
                    else if (state == GATE_OPENING)
                    {
                        /* Opposite command while opening => stop midway */
                        StopGate();
                        SetGateState(GATE_STOPPED_MIDWAY);
                        g_securityMotionActive = false;
                    }
#if ENABLE_AUTO_MODE == 0
                    else if (state == GATE_CLOSING)
                    {
                        /* Manual: second CMD_CLOSE while closing => stop midway */
                        StopGate();
                        SetGateState(GATE_STOPPED_MIDWAY);
                        g_securityMotionActive = false;
                    }
#endif
                    else
                    {
                        g_securityMotionActive = (cmd.source == SOURCE_SECURITY);
                        CloseGate(cmd.isAuto);
                    }
                    break;

                case CMD_STOP:
                    StopGate();
                    g_securityMotionActive = false;
                    if ((state == GATE_OPENING) || (state == GATE_CLOSING) ||
                        (state == GATE_REVERSING))
                    {
                        SetGateState(GATE_STOPPED_MIDWAY);
                    }
                    break;

                case CMD_OPEN_LIMIT:
                    if ((state == GATE_OPENING) || (state == GATE_REVERSING))
                    {
                        StopGate();
                        SetGateState(GATE_IDLE_OPEN);
                        g_securityMotionActive = false;
                    }
                    break;

                case CMD_CLOSED_LIMIT:
                    if (state == GATE_CLOSING)
                    {
                        StopGate();
                        SetGateState(GATE_IDLE_CLOSED);
                        g_securityMotionActive = false;
                    }
                    break;

                case CMD_OBSTACLE:
                    if ((state == GATE_CLOSING) || (state == GATE_OPENING))
                    {
                        g_lastMotionBeforeObstacle = g_motion;

                        /* 1. Stop immediately */
                        StopGate();
#if ENABLE_AUTO_MODE == 0
                        /* Manual: brief blue display before reversing */
                        SetGateState(GATE_STOPPED_MIDWAY);
                        vTaskDelay(pdMS_TO_TICKS(150));
#endif
                        /* 2. Reverse opposite direction for 0.5 s */
                        if (g_lastMotionBeforeObstacle == MOTION_CLOSE)
                        {
                            OpenGate(true);          /* reverse => green */
                        }
                        else if (g_lastMotionBeforeObstacle == MOTION_OPEN)
                        {
                            CloseGate(false);
                            SetGateState(GATE_REVERSING);
                        }

                        vTaskDelay(REVERSE_TIME_TICKS);

                        /* 3. Stop midway */
                        StopGate();
                        SetGateState(GATE_STOPPED_MIDWAY);
                        g_securityMotionActive = false;
                    }
                    break;

                default:
                    break;
            }
        }
    }
}

/* ===================== LED TASK ===================== */

static void vLEDTask(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
#if ENABLE_AUTO_MODE == 1
        /* Auto mode LED: green/red/off only */
        switch (g_motion)
        {
            case MOTION_OPEN:
                LED_Set(LED_GREEN);
                break;
            case MOTION_CLOSE:
                LED_Set(LED_RED);
                break;
            case MOTION_STOP:
            default:
                LED_Set(0);
                break;
        }
#else
        /* Manual mode LED: blue when stopped midway */
        GateState_t state = GetGateState();
        if (g_motion == MOTION_OPEN) {
            LED_Set(LED_GREEN);
        } else if (g_motion == MOTION_CLOSE) {
            LED_Set(LED_RED);
        } else {
            LED_Set((state == GATE_STOPPED_MIDWAY) ? LED_BLUE : 0U);
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

/* ===================== MAIN ===================== */

int main(void)
{
    GPIO_Init();

    xGateQueue      = xQueueCreate(12U, sizeof(GateCommand_t));
    xOpenLimitSem   = xSemaphoreCreateBinary();
    xClosedLimitSem = xSemaphoreCreateBinary();
    xGateStateMutex = xSemaphoreCreateMutex();

    if ((xGateQueue      == NULL) ||
        (xOpenLimitSem   == NULL) ||
        (xClosedLimitSem == NULL) ||
        (xGateStateMutex == NULL))
    {
        while (1) {}
    }

    SetGateState(GATE_IDLE_CLOSED);
    StopGate();
    g_securityMotionActive = false;

    xTaskCreate(vInputTask,       "Input",  256U, NULL, INPUT_TASK_PRIORITY,  NULL);
    xTaskCreate(vGateControlTask, "Gate",   256U, NULL, GATE_TASK_PRIORITY,   NULL);
    xTaskCreate(vLEDTask,         "LED",    128U, NULL, LED_TASK_PRIORITY,    NULL);
    xTaskCreate(vSafetyTask,      "Safety", 256U, NULL, SAFETY_TASK_PRIORITY, NULL);

    vTaskStartScheduler();
    while (1) {}
}