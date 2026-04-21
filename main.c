/* ============================================================================
 * DIGITAL CLOCK - MODULARIZED VERSION
 * ============================================================================
 */

#include <SN32F400.h>
#include <SN32F400_Def.h>
#include <stdbool.h>
#include "GPIO.h"
#include "WDT.h"
#include "CT16B1.h"
#include "CT16B0.h"
#include "Utility.h"
#include "Segment.h"
#include "KeyScan.h"
#include "Buzzer.h"
#include "I2C.h"
#include "EEPROM.h"

/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */
#define TIME_SPEED_MULTIPLIER     1   // Speed up time for testing (1 = normal)
#define AUTO_TIMEOUT_MS           30000   // Auto exit adjust mode after 30s
#define ALARM_DURATION_MS         5000    // Alarm rings for 5 seconds
#define BEEP_DURATION_MS          300     // Key beep duration
#define BLINK_INTERVAL_MS         500     // Blink interval for adjust mode

#define EEPROM_I2C_ADDR           0xA0    // EEPROM I2C address
#define EEPROM_ALARM_ADDR         0x02    // Alarm data address in EEPROM

#define LED_D6_ON                 SN_GPIO3->BCLR = (1 << 8)   // Turn on LED D6
#define LED_D6_OFF                SN_GPIO3->BSET = (1 << 8)   // Turn off LED D6

/* ============================================================================
 * EXTERNAL DECLARATIONS
 * ============================================================================ */
extern void PFPA_Init(void);
void NotPinOut_GPIO_init(void);
extern uint8_t segment_buff[4];

/* ============================================================================
 * ENUMERATIONS
 * ============================================================================ */
typedef enum {
    MODE_NORMAL = 0,            // Normal time display mode
    MODE_ADJUST_HOUR,           // Adjust hour mode
    MODE_ADJUST_MINUTE,         // Adjust minute mode
    MODE_ADJUST_ALARM_HOUR,     // Adjust alarm hour mode
    MODE_ADJUST_ALARM_MINUTE    // Adjust alarm minute mode
} clock_mode_t;

/* ============================================================================
 * STRUCTURES
 * ============================================================================ */
typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} time_t;

typedef struct {
    uint8_t hour;
    uint8_t minute;
} alarm_t;

typedef struct {
    clock_mode_t mode;          // Current operation mode
    bool blink_state;           // Current blink state (on/off)
    bool alarm_active;          // Alarm is currently ringing
    bool eeprom_dirty;          // Need to save alarm to EEPROM
    uint32_t last_activity_ms;  // Time since last key press (for timeout)
} system_state_t;

/* ============================================================================
 * STATIC VARIABLES
 * ============================================================================ */
static time_t current_time = {0, 0, 0};
static alarm_t alarm_time = {0, 0};
static system_state_t sys_state = {
    .mode = MODE_NORMAL,
    .blink_state = false,
    .alarm_active = false,
    .eeprom_dirty = false,
    .last_activity_ms = 0
};

static uint16_t beep_timer = 0;         // Countdown for key beep
static uint32_t alarm_timer_ms = 0;     // Countup for alarm duration
static uint32_t blink_timer_ms = 0;     // Countup for blink interval
static uint16_t fast_time_counter = 0;  // Counter for time speed multiplier

static const uint8_t segment_map[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

#define MS_PER_SECOND (1000 / TIME_SPEED_MULTIPLIER)

/* ============================================================================
 * FORWARD DECLARATIONS
 * ============================================================================ */
static void system_init(void);
static void handle_keys(uint16_t raw_key);
static void update_time(void);
static void update_display(void);
static void handle_alarm(void);
static void handle_buzzer(void);
static void eeprom_save_alarm(void);
static void eeprom_load_alarm(void);

// Modularized helper functions
static void handle_timeout(void);
static void handle_blink(void);
static void handle_led_indicator(void);

/* ============================================================================
 * 7-SEGMENT HELPERS
 * ============================================================================ */
static inline void set_segment_digit(uint8_t pos, uint8_t digit) {
    segment_buff[pos] = (digit <= 9) ? segment_map[digit] : 0;
}

static inline void set_colon(bool on) {
    if (on)
        segment_buff[1] |= 0x80;    // Turn on colon (bit 7)
    else
        segment_buff[1] &= ~0x80;   // Turn off colon
}

/* ============================================================================
 * TIME MANIPULATION
 * ============================================================================ */
static void increment_hour(void) {
    current_time.hour = (current_time.hour >= 23) ? 0 : current_time.hour + 1;
}

static void decrement_hour(void) {
    current_time.hour = (current_time.hour == 0) ? 23 : current_time.hour - 1;
}

static void increment_minute(void) {
    current_time.minute = (current_time.minute >= 59) ? 0 : current_time.minute + 1;
}

static void decrement_minute(void) {
    current_time.minute = (current_time.minute == 0) ? 59 : current_time.minute - 1;
}

static void increment_alarm_hour(void) {
    alarm_time.hour = (alarm_time.hour >= 23) ? 0 : alarm_time.hour + 1;
    sys_state.eeprom_dirty = true;   // Mark that alarm needs to be saved
}

static void decrement_alarm_hour(void) {
    alarm_time.hour = (alarm_time.hour == 0) ? 23 : alarm_time.hour - 1;
    sys_state.eeprom_dirty = true;
}

static void increment_alarm_minute(void) {
    alarm_time.minute = (alarm_time.minute >= 59) ? 0 : alarm_time.minute + 1;
    sys_state.eeprom_dirty = true;
}

static void decrement_alarm_minute(void) {
    alarm_time.minute = (alarm_time.minute == 0) ? 59 : alarm_time.minute - 1;
    sys_state.eeprom_dirty = true;
}

/* ============================================================================
 * BUZZER CONTROL
 * ============================================================================ */
static void buzzer_beep(uint16_t duration_ms) {
    set_buzzer_pitch(0);        // Turn buzzer ON
    beep_timer = duration_ms;   // Set timer for auto-off
}

static void buzzer_off(void) {
    if (beep_timer == 0 && !sys_state.alarm_active) {
        set_buzzer_pitch(99);   // Turn buzzer OFF
    }
}

/* ============================================================================
 * EEPROM FUNCTIONS
 * ============================================================================ */
static void eeprom_save_alarm(void) {
    uint8_t data[2] = {alarm_time.hour, alarm_time.minute};
    eeprom_write(EEPROM_I2C_ADDR, EEPROM_ALARM_ADDR, data, 2);
    UT_DelayNms(10);            // Wait for EEPROM write complete
}

static void eeprom_load_alarm(void) {
    uint8_t data[2];
    eeprom_read(EEPROM_I2C_ADDR, EEPROM_ALARM_ADDR, data, 2);
    
    // Validate data before using
    if (data[0] < 24 && data[1] < 60) {
        alarm_time.hour = data[0];
        alarm_time.minute = data[1];
    } else {
        // Invalid data, use default value 07:00
        alarm_time.hour = 7;
        alarm_time.minute = 0;
        sys_state.eeprom_dirty = true;
    }
}

/* ============================================================================
 * KEY HANDLING
 * ============================================================================ */
static void on_setup_pressed(void) {
    switch (sys_state.mode) {
        case MODE_NORMAL:
            sys_state.mode = MODE_ADJUST_HOUR;
            break;
        case MODE_ADJUST_HOUR:
            sys_state.mode = MODE_ADJUST_MINUTE;
            break;
        default:
            sys_state.mode = MODE_NORMAL;
            break;
    }
    sys_state.last_activity_ms = 0;     // Reset timeout counter
}

static void on_alarm_set_pressed(void) {
    switch (sys_state.mode) {
        case MODE_NORMAL:
            eeprom_load_alarm();        // Load current alarm from EEPROM
            sys_state.mode = MODE_ADJUST_ALARM_HOUR;
            break;
        case MODE_ADJUST_ALARM_HOUR:
            sys_state.mode = MODE_ADJUST_ALARM_MINUTE;
            break;
        default:
            sys_state.mode = MODE_NORMAL;
            if (sys_state.eeprom_dirty) {
                eeprom_save_alarm();    // Save alarm when exiting
                sys_state.eeprom_dirty = false;
            }
            break;
    }
    sys_state.last_activity_ms = 0;
}

static void handle_keys(uint16_t raw_key) {
    if (raw_key == 0) return;
    
    uint8_t key = (uint8_t)raw_key;
    
    // Any key press stops the alarm
    if (sys_state.alarm_active) {
        sys_state.alarm_active = false;
        buzzer_off();
        return;
    }
    
    // Normal key handling
    buzzer_beep(BEEP_DURATION_MS);
    sys_state.last_activity_ms = 0;
    sys_state.blink_state = false;
    blink_timer_ms = 0;
    
    switch (key) {
        case KEY_1:                 // SETUP button
            on_setup_pressed();
            break;
        case KEY_13:                // ALARM SET button
            on_alarm_set_pressed();
            break;
        case KEY_4:                 // UP button
            switch (sys_state.mode) {
                case MODE_ADJUST_HOUR: increment_hour(); break;
                case MODE_ADJUST_MINUTE: increment_minute(); break;
                case MODE_ADJUST_ALARM_HOUR: increment_alarm_hour(); break;
                case MODE_ADJUST_ALARM_MINUTE: increment_alarm_minute(); break;
                default: break;
            }
            break;
        case KEY_8:                 // DOWN button
            switch (sys_state.mode) {
                case MODE_ADJUST_HOUR: decrement_hour(); break;
                case MODE_ADJUST_MINUTE: decrement_minute(); break;
                case MODE_ADJUST_ALARM_HOUR: decrement_alarm_hour(); break;
                case MODE_ADJUST_ALARM_MINUTE: decrement_alarm_minute(); break;
                default: break;
            }
            break;
        default:
            break;
    }
}

/* ============================================================================
 * TIME UPDATE
 * ============================================================================ */
static void update_time(void) {
    if (sys_state.mode != MODE_NORMAL) return;  // Only update in normal mode
    
    fast_time_counter++;
    if (fast_time_counter >= MS_PER_SECOND) {
        fast_time_counter = 0;
        
        current_time.second++;
        if (current_time.second >= 60) {
            current_time.second = 0;
            current_time.minute++;
            if (current_time.minute >= 60) {
                current_time.minute = 0;
                current_time.hour++;
                if (current_time.hour >= 24) {
                    current_time.hour = 0;
                }
            }
        }
    }
}

/* ============================================================================
 * ALARM HANDLING
 * ============================================================================ */
static void handle_alarm(void) {
    // Check if alarm should trigger
    if (!sys_state.alarm_active && sys_state.mode == MODE_NORMAL) {
        if (current_time.hour == alarm_time.hour && 
            current_time.minute == alarm_time.minute && 
            current_time.second == 0) {
            sys_state.alarm_active = true;
            alarm_timer_ms = 0;
        }
    }
    
    // Handle active alarm
    if (sys_state.alarm_active) {
        alarm_timer_ms++;
        
        // Toggle buzzer every 500ms
        if ((alarm_timer_ms % 1000) < 500) {
            set_buzzer_pitch(0);    // ON
        } else {
            set_buzzer_pitch(99);   // OFF
        }
        
        // Auto stop after duration
        if (alarm_timer_ms >= ALARM_DURATION_MS) {
            sys_state.alarm_active = false;
            set_buzzer_pitch(99);
        }
    }
}

/* ============================================================================
 * BUZZER HANDLING
 * ============================================================================ */
static void handle_buzzer(void) {
    if (beep_timer > 0) {
        beep_timer--;
        if (beep_timer == 0 && !sys_state.alarm_active) {
            set_buzzer_pitch(99);   // Turn off key beep
        }
    }
}

/* ============================================================================
 * DISPLAY UPDATE
 * ============================================================================ */
static void update_display(void) {
    uint8_t disp_hour, disp_minute;
    bool show_colon;
    
    // Choose what to display based on mode
    if (sys_state.mode >= MODE_ADJUST_ALARM_HOUR) {
        disp_hour = alarm_time.hour;
        disp_minute = alarm_time.minute;
        show_colon = false;         // No colon when setting alarm
    } else {
        disp_hour = current_time.hour;
        disp_minute = current_time.minute;
        show_colon = true;          // Show colon in normal mode
    }
    
    // Update segment buffer
    set_segment_digit(0, disp_hour / 10);
    set_segment_digit(1, disp_hour % 10);
    set_segment_digit(2, disp_minute / 10);
    set_segment_digit(3, disp_minute % 10);
    set_colon(show_colon);
    
    // Handle blinking in adjust mode
    if (sys_state.mode != MODE_NORMAL && sys_state.blink_state) {
        bool blink_hour = (sys_state.mode == MODE_ADJUST_HOUR || 
                          sys_state.mode == MODE_ADJUST_ALARM_HOUR);
        bool blink_min = (sys_state.mode == MODE_ADJUST_MINUTE || 
                         sys_state.mode == MODE_ADJUST_ALARM_MINUTE);
        
        if (blink_hour) {
            segment_buff[0] = 0;            // Turn off hour tens
            segment_buff[1] &= 0x80;        // Keep colon only
        }
        if (blink_min) {
            segment_buff[2] = 0;            // Turn off minute tens
            segment_buff[3] = 0;            // Turn off minute units
        }
    }
}

/* ============================================================================
 * MODULARIZED HELPER FUNCTIONS
 * ============================================================================ */
static void handle_timeout(void) {
    // Auto exit adjust mode after 30 seconds of inactivity
    if (sys_state.mode != MODE_NORMAL) {
        sys_state.last_activity_ms++;
        if (sys_state.last_activity_ms >= AUTO_TIMEOUT_MS) {
            sys_state.mode = MODE_NORMAL;
            sys_state.last_activity_ms = 0;
            buzzer_beep(BEEP_DURATION_MS);  // Beep to notify exit
        }
    }
}

static void handle_blink(void) {
    // Update blink state every 500ms
    blink_timer_ms++;
    if (blink_timer_ms >= BLINK_INTERVAL_MS) {
        blink_timer_ms = 0;
        sys_state.blink_state = !sys_state.blink_state;
    }
}

static void handle_led_indicator(void) {
    // LED D6 indicates alarm setting mode (blinks when in alarm adjust mode)
    if (sys_state.mode >= MODE_ADJUST_ALARM_HOUR) {
        if (sys_state.blink_state) 
            LED_D6_ON;
        else 
            LED_D6_OFF;
    } else {
        LED_D6_OFF;
    }
}

/* ============================================================================
 * SYSTEM INITIALIZATION
 * ============================================================================ */
static void system_init(void) {
    // Basic system setup
    SystemInit();
    SystemCoreClockUpdate();
    PFPA_Init();
    
    // Watchdog configuration
    SN_SYS0->EXRSTCTRL_b.RESETDIS = 0;
    GPIO_Init();
    
    // GPIO mode configuration
    SN_GPIO0->MODE = 0xFF;
    SN_GPIO1->MODE |= (0x0F << 9);
    
    // Initialize peripherals
    WDT_Init();
    CT16B0_Init();
    SN_PFPA->CT16B0_b.PWM0 = 1;
    I2C0_Init();
    SN_PFPA->I2C0 = 0x0000000A;
    
    // Load saved alarm from EEPROM
    eeprom_load_alarm();
    
    // Start 1ms timer (must be last)
    CT16B1_Init();
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */
int main(void) {
    system_init();
    
    while (1) {
        __WDT_FEED_VALUE;                   // Feed watchdog
        
        if (timer_1ms_flag) {               // 1ms timer flag
            timer_1ms_flag = 0;
            
            // Input scanning
            uint16_t raw_key = KeyScan();
            Digital_Scan();
            
            // Core functions
            handle_keys(raw_key);
            update_time();
            handle_alarm();
            handle_buzzer();
            update_display();
            
            // Modularized background tasks
            handle_timeout();               // Auto exit after 30s
            handle_blink();                 // 500ms blink timer
            handle_led_indicator();         // LED D6 for alarm mode
        }
    }
}

/* ============================================================================
 * HARDWARE INITIALIZATION
 * ============================================================================ */
void NotPinOut_GPIO_init(void) {
    #if (PKG == SN32F400)
        SN_GPIO0->CFG = 0x00A008AA;
        SN_GPIO1->CFG = 0x000000AA;
        SN_GPIO3->CFG = 0x0002AAAA;
    #endif
}

void HardFault_Handler(void) {
    NVIC_SystemReset();                     // Reset on hard fault
}