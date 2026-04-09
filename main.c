/************************************************************
 * Capstone Project: GaN Motor Drive UART Protocol 
 *
 * Target MCU: XMC 4200
 *
 * Authors:
 *   Owen Serjak
 *   Julien Bolduc
 *   Nolan Wrigley
 *
 ************************************************************/


#include "cybsp.h"
#include "cycfg_pins.h"
#include "cycfg_peripherals.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>


/* ============================================================
 *                  Current Sensor Constants
 *   Board uses TLI4971 (24 mV/A, mid-supply offset)
 * ============================================================ */
#define VREF_MV             (3300UL)
#define ADC_MAX_COUNTS      (4095UL)
#define CURR_SENS_MV_PER_A  (24)    
#define OFFSET_SAMPLES      (64)     
/* ------------------------------------------------------------ */


/* ============================================================
 *                  Temperature Fault Thresholds
 * ============================================================ */
#define TEMP_WARNING_C       (31)    // Warning threshold
#define TEMP_HARD_FAULT_C    (33)    // Hard fault threshold
/* ------------------------------------------------------------ */


/* ============================================================
 *                  Fault & Run State Definitions
 * ============================================================ */
typedef enum {
    FAULT_NONE = 0,
    FAULT_WARNING,
    FAULT_HARD
} fault_state_t;

typedef enum {
    RUN_STOPPED = 0,
    RUN_RUNNING,
    RUN_FAULT
} run_state_t;
/* ------------------------------------------------------------ */



/* ============================================================
 *				UART RX Setup & Locks 
 * 		Temporary storage Buffer for UART messages
 * ============================================================ */

// Telemetry enable - Constant Polling of temperature/current values
static volatile uint8_t polling_enabled = 1;

// TX Lock - Prevents telemetry durring command responces
static volatile uint8_t uart_tx_locked = 0;

// System state variables
static volatile fault_state_t fault_status = FAULT_NONE;
static volatile run_state_t run_status = RUN_STOPPED;

// RX line Buffer - MAX 63 chars + NULL (RX sends single characters, this buffer is used to string them together)
static char rx_buf[64];
static uint8_t rx_idx = 0;

// RX kick flag to trigger interrupt
static volatile uint8_t rx_kick = 0;

// RX ring buffer
#define RX_RING_SZ 256
static volatile uint8_t  rx_ring[RX_RING_SZ];
static volatile uint16_t rx_w = 0;
static volatile uint16_t rx_r = 0;

static inline void rx_ring_put(uint8_t c)
{
    uint16_t n = (uint16_t)((rx_w + 1u) % RX_RING_SZ);
    if (n != rx_r) {            /* drop byte if full */
        rx_ring[rx_w] = c;
        rx_w = n;
    }
}

static inline int rx_ring_get(uint8_t *out)
{
    __disable_irq();
    if (rx_r == rx_w) {
        __enable_irq();
        return 0;
    }
    *out = rx_ring[rx_r];
    rx_r = (uint16_t)((rx_r + 1u) % RX_RING_SZ);
    __enable_irq();
    return 1;
}


static inline uint8_t rx_ring_has_data(void)
{
    return (rx_r != rx_w);
}

static inline int XMC_UART_GetReceivedData(uint8_t *out)
{
    return rx_ring_get(out);
}

/* Forward declare so helpers can call it */
static void uart_rx_task(void);



/* ============================================================
 * 						TX Helpers
 *				(Lock and wait functions)
 * ============================================================ */
static inline void uart_tx_wait_idle(void)
{
    /* IMPORTANT: do NOT call uart_rx_task() here (avoids nested prints) */
    while (XMC_USIC_CH_GetTransmitBufferStatus(usic_0_ch_1_HW) ==
           XMC_USIC_CH_TBUF_STATUS_BUSY)
    {
        /* just wait */
    }
}

static inline void uart_tx_lock_acquire(void)
{
    __disable_irq();
    uart_tx_locked = 1;
    __enable_irq();
}

static inline void uart_tx_lock_release(void)
{
    __disable_irq();
    uart_tx_locked = 0;
    __enable_irq();
}

static inline uint8_t uart_tx_is_locked(void)
{
    return uart_tx_locked;
}


/* ------------------------------------------------------------
 *                    Safe State Entry
 * Puts system into safe state during fault conditions
 * - Stops motor operation (if PWM were active)
 * - Sets run status to FAULT
 * ------------------------------------------------------------ */
static void enter_safe_state(void)
{
    run_status = RUN_FAULT;
    
    /* Future: Disable PWM outputs here when motor control is implemented */
    /* Example: XMC_CCU8_SLICE_StopTimer(...); */
    /* Example: Set gate driver disable pins high */
}

/* ------------------------------------------------------------
 *                    Safe State Exit
 * Exits safe state when faults are cleared
 * ------------------------------------------------------------ */
static void exit_safe_state(void)
{
    if (fault_status == FAULT_NONE) {
        run_status = RUN_STOPPED;
    }
}


/* ------------------------------------------------------------
 *                    Command Handler
 * Reads pre-defined commands in the output terminal
 * Executes specific orders based on input 

 * @param in  Parsed command from UART line Parser
 * @return    printf message 
 * ------------------------------------------------------------ */
static void handle_cmd(const char *line)
{
    if (strcmp(line, "PING") == 0)
    {
        uart_tx_lock_acquire();
        uart_tx_wait_idle();
        printf("OK PING\r\n");
        uart_tx_wait_idle();
        uart_tx_lock_release();
    }
    else if (strcmp(line, "START_MOTOR") == 0)
    {
        if (fault_status == FAULT_HARD) {
            uart_tx_lock_acquire();
            uart_tx_wait_idle();
            printf("ERR HARD_FAULT_ACTIVE\r\n");
            uart_tx_wait_idle();
            uart_tx_lock_release();
        } else {
            run_status = RUN_RUNNING;
            uart_tx_lock_acquire();
            uart_tx_wait_idle();
            printf("OK START_MOTOR\r\n");
            uart_tx_wait_idle();
            uart_tx_lock_release();
            
            /* Future: Enable PWM outputs here when motor control is implemented */
        }
    }
    else if (strcmp(line, "STOP_MOTOR") == 0)
    {
        run_status = RUN_STOPPED;
        uart_tx_lock_acquire();
        uart_tx_wait_idle();
        printf("OK STOP_MOTOR\r\n");
        uart_tx_wait_idle();
        uart_tx_lock_release();
        
        /* Future: Disable PWM outputs here when motor control is implemented */
    }
    else if (strcmp(line, "CLEAR_FAULTS") == 0)
    {
        fault_status = FAULT_NONE;
        exit_safe_state();
        
        uart_tx_lock_acquire();
        uart_tx_wait_idle();
        printf("OK CLEAR_FAULTS\r\n");
        uart_tx_wait_idle();
        uart_tx_lock_release();
    }
    else if (strcmp(line, "GET_STATUS") == 0)
    {
        const char* fault_str;
        const char* run_str;
        
        switch(fault_status) {
            case FAULT_NONE:    fault_str = "NONE"; break;
            case FAULT_WARNING: fault_str = "WARNING"; break;
            case FAULT_HARD:    fault_str = "HARD"; break;
            default:            fault_str = "UNKNOWN"; break;
        }
        
        switch(run_status) {
            case RUN_STOPPED:  run_str = "STOPPED"; break;
            case RUN_RUNNING:  run_str = "RUNNING"; break;
            case RUN_FAULT:    run_str = "FAULT"; break;
            default:           run_str = "UNKNOWN"; break;
        }
        
        uart_tx_lock_acquire();
        uart_tx_wait_idle();
        printf("OK STATUS FAULT=%s RUN=%s\r\n", fault_str, run_str);
        uart_tx_wait_idle();
        uart_tx_lock_release();
    }
    else if (strcmp(line, "TEST_START") == 0)
    {
        /* Stop telemetry immediately */
        polling_enabled = 0;

        uart_tx_lock_acquire();
        uart_tx_wait_idle();
        printf("OK TEST_START\r\n");
        uart_tx_wait_idle();
        uart_tx_lock_release();
    }
    else if (strcmp(line, "TEST_END") == 0)
    {
        uart_tx_lock_acquire();
        uart_tx_wait_idle();
        printf("OK TEST_END\r\n");
        uart_tx_wait_idle();
        uart_tx_lock_release();

        polling_enabled = 1;
    }
    else
    {
        uart_tx_lock_acquire();
        uart_tx_wait_idle();
        printf("ERR BAD_CMD\r\n");
        uart_tx_wait_idle();
        uart_tx_lock_release();
    }
}


/* ------------------------------------------------------------
 *                  UART RX Line Parser
 *   Accumulates characters until '\n' stores in rx_buf
 *
 * @param out rx_buf        Buffer storing completed UART line
 * @return    None
 * ------------------------------------------------------------ */
static void uart_rx_task(void)
{
    uint8_t c;

    while (XMC_UART_GetReceivedData(&c))
    {
        if (c == '\r')
            continue;

        if (c == '\n')
        {
            rx_buf[rx_idx] = '\0';
            rx_idx = 0;

            if (rx_buf[0] != '\0')
                handle_cmd(rx_buf);
        }
        else
        {
            if (rx_idx < (sizeof(rx_buf) - 1))
                rx_buf[rx_idx++] = (char)c;
            else
                rx_idx = 0;   /* overflow reset */
        }
    }
}
/* ============================================================ */



/* ============================================================
 *                UART TX / printf Redirection
 * Redirects standard output (printf) using c library function _write()
 * 			Each use of printf() calls _write internally 
 *
 *  @param file  Output stream identifier (unused)
 *  @param ptr   Pointer to data buffer to transmit
 *  @param len   Number of bytes to transmit
 *
 *  @return      Number of bytes transmitted
 * ============================================================ */
int _write(int file, char *ptr, int len)
{
    (void)file;

    /* Block background prints when telemetry is disabled */
    while (!polling_enabled) {
    }
    for (int i = 0; i < len; i++)
    {
        XMC_UART_CH_Transmit(
            usic_0_ch_1_HW,
            (uint16_t)ptr[i]
        );

        while (XMC_USIC_CH_GetTransmitBufferStatus(
                   usic_0_ch_1_HW) ==
               XMC_USIC_CH_TBUF_STATUS_BUSY)
        {
        }
    }
    return len;
}
/* ============================================================ */



/* ============================================================
 *  					 ADC Helpers
 *
 * Three functions used by telemetry and board measurments
 *  1. Reads raw 12-bit ADC results
 *  2. Convert ADC counts to millivolts
 *  3. Convert millivolts difference to currents
 * ============================================================ */

/* 1. Read raw 12-bit ADC result */
static inline uint16_t read_adc12(uint8_t result_reg)
{
    uint32_t raw =
        XMC_VADC_GROUP_GetResult(
            vadc_0_group_0_HW,
            result_reg
        );

    return (uint16_t)(raw & 0x0FFF);
}

/* 2. Convert ADC counts → millivolts */
static inline uint32_t adc_to_mv(uint16_t adc12)
{
    return (uint32_t)((adc12 * VREF_MV) / ADC_MAX_COUNTS);
}


/* 3. Convert mV difference → centi-amps */
static inline int32_t mv_to_i_cA(int32_t mv_diff)
{
    /* I(cA) = 100 * mv / 24 */
    return (int32_t)((100 * mv_diff) / CURR_SENS_MV_PER_A);
}
/* ============================================================ */


/* ============================================================
 *              Temperature Fault Checking
 * 
 * Monitors temperature and updates fault status
 * Automatically enters safe state on hard fault
 * 
 * @param temp_c Temperature in degrees Celsius (x10 format)
 * ============================================================ */
static void check_temperature_faults(int32_t temp_x10)
{
    int32_t temp_c = temp_x10 / 10;
    
    fault_state_t old_fault = fault_status;
    
    if (temp_c >= TEMP_HARD_FAULT_C) {
        fault_status = FAULT_HARD;
        
        /* Hard fault detected - enter safe state */
        if (old_fault != FAULT_HARD) {
            enter_safe_state();
            
            uart_tx_lock_acquire();
            uart_tx_wait_idle();
            printf("\r\n*** HARD FAULT: Temperature %.1f C exceeds limit! ***\r\n",
                   (float)temp_x10 / 10.0f);
            uart_tx_wait_idle();
            uart_tx_lock_release();
        }
    }
    else if (temp_c >= TEMP_WARNING_C) {
        if (fault_status != FAULT_HARD) {  // Don't downgrade from hard fault
            fault_status = FAULT_WARNING;
            
            /* Warning transition */
            if (old_fault != FAULT_WARNING) {
                uart_tx_lock_acquire();
                uart_tx_wait_idle();
                printf("\r\n*** WARNING: Temperature %.1f C exceeds warning threshold! ***\r\n",
                       (float)temp_x10 / 10.0f);
                uart_tx_wait_idle();
                uart_tx_lock_release();
            }
        }
    }
    else {
        /* Temperature is normal */
        if (old_fault == FAULT_WARNING) {
            fault_status = FAULT_NONE;
            
            uart_tx_lock_acquire();
            uart_tx_wait_idle();
            printf("\r\n*** Temperature returned to normal ***\r\n");
            uart_tx_wait_idle();
            uart_tx_lock_release();
        }
        /* Note: Hard faults require manual CLEAR_FAULTS command */
    }
}
/* ============================================================ */


/* ============================================================
 * 						IRQ handler
 *				(Interrupt Request Handler)
 * ============================================================ */
void USIC0_1_IRQHandler(void)
{
    /* Drain RX FIFO into ring buffer */
    while (XMC_USIC_CH_RXFIFO_GetLevel(usic_0_ch_1_HW) > 0U)
    {
        uint8_t c = (uint8_t)XMC_UART_CH_GetReceivedData(usic_0_ch_1_HW);
        rx_ring_put(c);
        rx_kick = 1;
    }

    /* Also handle RBUF (if FIFO isn't active / empty) */
    XMC_USIC_CH_RBUF_STATUS_t st = XMC_USIC_CH_GetReceiveBufferStatus(usic_0_ch_1_HW);
    if (st == XMC_USIC_CH_RBUF_STATUS_DATA_VALID0 || st == XMC_USIC_CH_RBUF_STATUS_DATA_VALID1)
    {
        uint8_t c = (uint8_t)XMC_UART_CH_GetReceivedData(usic_0_ch_1_HW);
        rx_ring_put(c);
        rx_kick = 1;
    }
}

/* Enable NVIC for the mapped USIC interrupt */
static void uart_rx_interrupt_init(void)
{
    /* Route RX events to SR0 and enable the interrupt node */
    XMC_USIC_CH_SetInterruptNodePointer(usic_0_ch_1_HW,
                                        XMC_USIC_CH_INTERRUPT_NODE_POINTER_RECEIVE,
                                        1); // SR0

    XMC_USIC_CH_EnableEvent(usic_0_ch_1_HW, XMC_USIC_CH_EVENT_STANDARD_RECEIVE);
    XMC_USIC_CH_EnableEvent(usic_0_ch_1_HW, XMC_USIC_CH_EVENT_RECEIVE_START);
    XMC_USIC_CH_EnableEvent(usic_0_ch_1_HW, XMC_USIC_CH_EVENT_ALTERNATIVE_RECEIVE);

    __enable_irq();
    NVIC_SetPriority(USIC0_1_IRQn, 3);
    NVIC_EnableIRQ(USIC0_1_IRQn);
}
/* ============================================================ */


/* ============================================================
 *                          main
 * The main loop does 4 things:
 * 1. System Initialization
 * 2. Current Sensor Offset Calibration
 * 3. Main Telemetry Loop
 * 4. Polling Rate Control
 *
 * ============================================================ */

int main(void)
{
/* --------------------------------------------------------
 * 1. System Initialization
 * -------------------------------------------------------- */
    cybsp_init();

    init_cycfg_pins();
    init_cycfg_peripherals();
    uart_rx_interrupt_init();

    /* Allow software-triggered queue conversions */
    XMC_VADC_GROUP_QueueSetGatingMode(
        vadc_0_group_0_HW,
        XMC_VADC_GATEMODE_IGNORE
    );

/* --------------------------------------------------------
    2. Offset Calibration 
    (Determines zero-current voltage offsets for each phase)
 * -------------------------------------------------------- */
 uint32_t off3_mv = 0, off4_mv = 0, off5_mv = 0;

    printf("Calibrating current offsets... keep motor off\r\n");

    for (uint32_t n = 0; n < OFFSET_SAMPLES; n++)
    {
        uart_rx_task();

        XMC_VADC_GROUP_QueueTriggerConversion(vadc_0_group_0_HW);
        for (volatile uint32_t i = 0; i < 8000; i++) {}

        off3_mv += adc_to_mv(read_adc12(3));
        off4_mv += adc_to_mv(read_adc12(4));
        off5_mv += adc_to_mv(read_adc12(5));
    }

    off3_mv /= OFFSET_SAMPLES;
    off4_mv /= OFFSET_SAMPLES;
    off5_mv /= OFFSET_SAMPLES;

    printf("Offsets: CH3=%lu mV | CH4=%lu mV | CH5=%lu mV\r\n",
           (unsigned long)off3_mv, (unsigned long)off4_mv, (unsigned long)off5_mv);


/* --------------------------------------------------------
 * 3. Main Loop Polling + Interrupt
 * -------------------------------------------------------- */
    while (1)
    {
        // Highest priority
		// If there is RX activity, process it and skip telemetry this cycle
        if (rx_kick || rx_ring_has_data())
        {
            rx_kick = 0;
            uart_rx_task();
            continue;
        }

		// Lowest priority
		// If there are no locks proceed with polling for telementry
        if (polling_enabled && !uart_tx_is_locked())
        {
            XMC_VADC_GROUP_QueueTriggerConversion(vadc_0_group_0_HW);

			// ADC conversion settling delay
            for (volatile uint32_t i = 0; i < 8000; i++)
            {
                /* Break out early if RX arrives */
                if (rx_kick || rx_ring_has_data())
                    break;
            }

            /* If RX arrived during ADC wait, handle it immediately */
            if (rx_kick || rx_ring_has_data())
            {
                rx_kick = 0;
                uart_rx_task();
                continue;
            }


        		/* ---- Temperature ---- */
        uint16_t adc_temp = read_adc12(6);
        uint32_t mv_temp = adc_to_mv(adc_temp);
        int32_t temp_x10 = (int32_t)mv_temp - 500;
        
        /* Check temperature faults */
        check_temperature_faults(temp_x10);

        		/* ---- VMES ---- */
        uint16_t adc_vmes = read_adc12(7);
        uint32_t mv_vmes = adc_to_mv(adc_vmes);

        //Phase Currents
        uint32_t mv3 = adc_to_mv(read_adc12(3));
        uint32_t mv4 = adc_to_mv(read_adc12(4));
        uint32_t mv5 = adc_to_mv(read_adc12(5));

        int32_t i3_cA = mv_to_i_cA((int32_t)mv3 - (int32_t)off3_mv);
        int32_t i4_cA = mv_to_i_cA((int32_t)mv4 - (int32_t)off4_mv);
        int32_t i5_cA = mv_to_i_cA((int32_t)mv5 - (int32_t)off5_mv);

        		/* ---- Status strings ---- */
        const char* fault_str;
        const char* run_str;
        
        switch(fault_status) {
            case FAULT_NONE:    fault_str = "OK"; break;
            case FAULT_WARNING: fault_str = "WARN"; break;
            case FAULT_HARD:    fault_str = "FAULT"; break;
            default:            fault_str = "UNK"; break;
        }
        
        switch(run_status) {
            case RUN_STOPPED:  run_str = "STOP"; break;
            case RUN_RUNNING:  run_str = "RUN"; break;
            case RUN_FAULT:    run_str = "FAULT"; break;
            default:           run_str = "UNK"; break;
        }

        		/* ---- UART Polling Output (One Line) ---- */
		
        printf(
		    "TEL STATUS=%s RUN=%s TEMP=%ld.%01ldC "
		    "I3=%ld.%02ldA I4=%ld.%02ldA I5=%ld.%02ldA "
		    "VMES=%lu\r\n",
		    
		    fault_str,
		    run_str,
		
		    (long)(temp_x10 / 10),
		    (long)labs(temp_x10 % 10),
		
		    (long)(i3_cA / 100),
		    (long)labs(i3_cA % 100),
		    (long)(i4_cA / 100),
		    (long)labs(i4_cA % 100),
		    (long)(i5_cA / 100),
		    (long)labs(i5_cA % 100),
		
		    (unsigned long)mv_vmes);
	}

/* --------------------------------------------------------
 * 4. Polling Rate Control
 * -------------------------------------------------------- */
		// Software delay 
		// Increasing i slows telemetry update rate
		// Decreasing i increases telemetry update rate

        for (volatile uint32_t i = 0; i < 50000; i++)
        {
	
		// Polls UART receive buffer for incoming commands
		// Smaller modulus value = Faster command responsiveness but more CPU usage 
		// Larger modulus value = Slower command responsiveness but less CPU usage 
            if ((i % 2000) == 0)
                uart_rx_task();
        }
    }
}
/* ------------------------------------------------------------ */