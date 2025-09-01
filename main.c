#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#include "uart.h"
#include "mfrc522.h"

// ================== Traffic Light Config ==================
#define GREEN_TIME    5000    // 5 seconds (ms)
#define YELLOW_TIME   2000    // 2 seconds (ms)
#define EXTRA_GREEN   5000    // +5 seconds if density > 5

// Lane 1 LEDs (PORTA → Pins 22-24)
#define L1R PA0
#define L1Y PA1
#define L1G PA2
// Lane 2 LEDs (PORTA → Pins 25-27)
#define L2R PA3
#define L2Y PA4
#define L2G PA5
// Lane 3 LEDs (PORTC → Pins 35-33)
#define L3R PC2
#define L3Y PC3
#define L3G PC4
// Lane 4 LEDs (PORTC → Pins 32-30)
#define L4R PC5
#define L4Y PC6
#define L4G PC7

// ================== Timer / Traffic Globals ==================
volatile uint16_t ms_counter = 0;
volatile uint8_t  current_lane = 1;      
volatile uint16_t lane_start_time = 0;   
volatile uint8_t  green_phase = 1;       
volatile uint16_t green_time_current = GREEN_TIME;
volatile uint8_t  extended_this_cycle = 0;

// ================== RFID Config / Globals ==================
#define UID_LEN 4
static uint16_t reader_count[NUM_READERS] = {0};
static uint8_t  previous_uid[NUM_READERS][UID_LEN];
static uint8_t  has_prev[NUM_READERS] = {0};

// ---- Helper: compare/copy UIDs ----
static uint8_t compare_uid(uint8_t *a, uint8_t *b) { return memcmp(a, b, UID_LEN) == 0; }
static void    copy_uid(uint8_t *d, uint8_t *s)    { memcpy(d, s, UID_LEN); }

// ================== UART helpers ==================
static void uart_print_counts(void) {
    uart_print("\r\n[COUNT UPDATE]\r\n");
    for (uint8_t i = 0; i < NUM_READERS; i++) {
        char cbuf[40];
        sprintf(cbuf, "Reader %u Count: %u\r\n", (unsigned)(i+1), (unsigned)reader_count[i]);
        uart_print(cbuf);
    }
}

static void uart_print_extension(uint8_t lane) {
    char cbuf[64];
    sprintf(cbuf, "[EXTEND] Lane %u +%u ms (density > 5)\r\n", (unsigned)lane, (unsigned)EXTRA_GREEN);
    uart_print(cbuf);
}

// ================== IO Init ==================
static void init_pins(void) {
    DDRA |= 0x3F;    // PA0..PA5 outputs
    DDRC |= 0xFC;    // PC2..PC7 outputs
    PORTA = 0x00;
    PORTC &= ~0xFC;
}

static void clear_all(void) {
    PORTA &= ~0x3F;
    PORTC &= ~0xFC;
}

// ================== Timer0: 1 ms tick ==================
static void timer0_init(void) {
    TCCR0A = 0x00;
    TCCR0B = (1<<CS01)|(1<<CS00); // prescaler 64 -> 250 kHz
    TCNT0  = 6;                   // 250 counts to overflow -> ~1.000 ms
    TIMSK0 = (1<<TOIE0);          // overflow interrupt
    sei();
}

ISR(TIMER0_OVF_vect) {
    TCNT0 += 6;       // keep ~1 ms period
    ms_counter++;
}

// ================== Lane Enter Helper ==================
static void enter_lane(uint8_t lane) {
    current_lane = lane;                  
    lane_start_time = ms_counter;         
    green_phase = 1;
    green_time_current = GREEN_TIME;
    extended_this_cycle = 0;

    // Apply queued extension (density) at the start of this lane if needed
    uint8_t idx = lane - 1;               
    if (reader_count[idx] > 5) {
        green_time_current = GREEN_TIME + EXTRA_GREEN;
        reader_count[idx] = 0;            // reset only when extension is applied
        extended_this_cycle = 1;
        uart_print_extension(lane);
    }
}

// ================== Traffic State Update ==================
static void update_lights(void) {
    uint16_t now = ms_counter;
    uint16_t elapsed = (uint16_t)(now - lane_start_time);

    clear_all();

    // If currently GREEN and no extension used yet, allow live extension
    if (green_phase && !extended_this_cycle) {
        uint8_t idx = current_lane - 1;
        if (reader_count[idx] > 5) {
            green_time_current = (uint16_t)(green_time_current + EXTRA_GREEN);
            reader_count[idx] = 0;
            extended_this_cycle = 1;
            uart_print_extension(current_lane);
        }
    }

    // phase timing
    if (elapsed < green_time_current) {
        green_phase = 1; // GREEN
    } else if (elapsed < (uint16_t)(green_time_current + YELLOW_TIME)) {
        green_phase = 0; // YELLOW
    } else {
        // ==== HERE: Reset count after cycle ends ====
        uint8_t idx = current_lane - 1;
        reader_count[idx] = 0;             // reset after cycle
        has_prev[idx] = 0;                 // clear last UID too

        // advance to next lane
        uint8_t next = (current_lane % 4) + 1;
        enter_lane(next);
        return;
    }

    // Drive LEDs according to current lane & phase
    switch (current_lane) {
        case 1:
            if (green_phase) PORTA |= (1<<L1G); else PORTA |= (1<<L1Y);
            PORTA |= (1<<L2R);
            PORTC |= (1<<L3R) | (1<<L4R);
            break;
        case 2:
            if (green_phase) PORTA |= (1<<L2G); else PORTA |= (1<<L2Y);
            PORTA |= (1<<L1R);
            PORTC |= (1<<L3R) | (1<<L4R);
            break;
        case 3:
            if (green_phase) PORTC |= (1<<L3G); else PORTC |= (1<<L3Y);
            PORTA |= (1<<L1R) | (1<<L2R);
            PORTC |= (1<<L4R);
            break;
        case 4:
            if (green_phase) PORTC |= (1<<L4G); else PORTC |= (1<<L4Y);
            PORTA |= (1<<L1R) | (1<<L2R);
            PORTC |= (1<<L3R);
            break;
        default:
            break;
    }
}

// ================== RFID Polling ==================
static void check_readers(void) {
    for (uint8_t reader = 0; reader < NUM_READERS; reader++) {
        uint8_t status;
        uint8_t buf[MAX_LEN];

        status = MFRC522_Request(reader, PICC_REQIDL, buf);
        if (status == MI_OK) {
            status = MFRC522_Anticoll(reader, buf);
            if (status == MI_OK) {
                if (!has_prev[reader] || !compare_uid(buf, previous_uid[reader])) {
                    reader_count[reader]++;
                    copy_uid(previous_uid[reader], buf);
                    has_prev[reader] = 1;

                    uart_print_counts();

                    if ((reader + 1) == current_lane && green_phase && !extended_this_cycle) {
                        if (reader_count[reader] > 5) {
                            green_time_current = (uint16_t)(green_time_current + EXTRA_GREEN);
                            reader_count[reader] = 0;
                            extended_this_cycle = 1;
                            uart_print_extension(current_lane);
                        }
                    }
                }
            }
            MFRC522_Halt(reader);
        }
        _delay_ms(20);
    }
}

// ================== MAIN ==================
int main(void) {
    init_pins();
    timer0_init();

    uart_init();
    uart_print("\r\n[BOOT] ATmega328P Traffic + RFID\r\n");

    SPI_init();
    MFRC522_AllInit();
    uart_print("[INFO] Ready with 4 readers on R1/R2/R3/R4\r\n");

    enter_lane(1);

    while (1) {
        update_lights();
        check_readers();
    }
}
