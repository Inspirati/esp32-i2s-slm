/*
 * Display A-weighted sound level measured by I2S Microphone
 *
 * (c)2019 Ivan Kostoski
 * (c)2025 Robert Dickenson (Inspirati)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Sketch samples audio data from I2S microphone, processes the data
 * with digital IIR filters and calculates A or C weighted Equivalent
 * Continuous Sound Level (Leq)
 *
 * Displays line on the small OLED screen with 'short' LAeq(125ms)
 * response and numeric LAeq(1sec) dB value from the signal RMS.
 */
#include <Arduino.h>
#include <driver/i2s.h>
#include "sos-iir-filter.h"
#include "display.h"

//
// Configuration
//
#define LEQ_PERIOD        1           // second(s)
#define WEIGHTING         A_weighting // Also avaliable: 'C_weighting' or 'None' (Z_weighting)

// NOTE: Some microphones require at least DC-Blocker filter
#define MIC_EQUALIZER     ICS43434    // See below for defined IIR filters or set to 'None' to disable
//#define MIC_OFFSET_DB     3.0103      // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration
#define MIC_OFFSET_DB     0.0         // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

// Customize these values from microphone datasheet
#define MIC_SENSITIVITY   -26         // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB        94.0        // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB   116.0       // dB - Acoustic overload point
#define MIC_NOISE_DB      29          // dB - Noise floor
#define MIC_BITS          24          // valid number of bits in I2S data
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT  0           // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

// Calculate reference amplitude value at compile time
constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

//
// I2S pins - Can be routed to almost any (unused) ESP32 pin.
//            SD can be any pin, inlcuding input only pins (36-39).
//            SCK (i.e. BCLK) and WS (i.e. L/R CLK) must be output capable pins
//
// Below ones are just example for my board layout, put here the pins you will use
//
#define I2S_WS            16 
#define I2S_SCK           17 
#define I2S_SD            15

// I2S peripheral to use (0 or 1)
#define I2S_PORT          I2S_NUM_0

//
// Sampling
//
#define SAMPLE_RATE       48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS       32    // bits
#define SAMPLE_T          int32_t 
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

// Data we push to 'queueHandle'
struct sum_queue_t {
  // Sum of squares of mic samples, after Equalizer filter
  float sum_sqr_SPL;
  // Sum of squares of weighted mic samples
  float sum_sqr_weighted;
  // Debug only, FreeRTOS ticks we spent processing the I2S data
  uint32_t proc_ticks;
};

TaskHandle_t taskHandle;
QueueHandle_t queueHandle;

// Static buffer for block of samples
static float samples[SAMPLES_SHORT] __attribute__((aligned(4)));

//
// I2S Microphone sampling setup 
//
void mic_i2s_init() {
  // Setup I2S to sample mono channel for SAMPLE_RATE * SAMPLE_BITS
  // NOTE: Recent update to Arduino_esp32 (1.0.2 -> 1.0.3)
  //       seems to have swapped ONLY_LEFT and ONLY_RIGHT channels
  const i2s_config_t i2s_config = {
    mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate: SAMPLE_RATE,
    bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
//
// TODO: beware: this depends on how you wire your microphone and
//               on which version of Arduino libs being used
//  You may need to experiment with this selection
    channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
//    channel_format: I2S_CHANNEL_FMT_ONLY_RIGHT,
//
    communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
    dma_buf_count: DMA_BANKS,
    dma_buf_len: DMA_BANK_SIZE,
    use_apll: true,
    tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
  // I2S pin mapping
  const i2s_pin_config_t pin_config = {
    bck_io_num:   I2S_SCK,  
    ws_io_num:    I2S_WS,    
    data_out_num: -1, // not used
    data_in_num:  I2S_SD   
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  #if (MIC_TIMING_SHIFT > 0) 
    // Undocumented (?!) manipulation of I2S peripheral registers
    // to fix MSB timing issues with some I2S microphones
    REG_SET_BIT(I2S_TIMING_REG(I2S_PORT), BIT(9));   
    REG_SET_BIT(I2S_CONF_REG(I2S_PORT), I2S_RX_MSB_SHIFT);  
  #endif
  
  i2s_set_pin(I2S_PORT, &pin_config);

  //FIXME: There is a known issue with esp-idf and sampling rates, see:
  //       https://github.com/espressif/esp-idf/issues/2634
  //       In the meantime, the below line seems to set sampling rate at ~47999.992Hz
  //       fifs_req=24576000, sdm0=149, sdm1=212, sdm2=5, odir=2 -> fifs_reached=24575996  
  //NOTE:  This seems to be fixed in ESP32 Arduino 1.0.4, esp-idf 3.2
  //       Should be safe to remove...
  //#include <soc/rtc.h>
  //rtc_clk_apll_enable(1, 149, 212, 5, 2);
}

//
// I2S Reader Task
//
// Rationale for separate task reading I2S is that IIR filter
// processing can be scheduled to different core on the ESP32
// while main task can do something else, like update the 
// display in the example
//
// As this is intended to run as separate high-priority task, 
// we only do the minimum required work with the I2S data
// until it is 'compressed' into sum of squares 
//
// FreeRTOS priority and stack size (in 32-bit words) 
#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048

extern bool oneshot; // for diagnostic purposes only

void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();

  // Discard first block, microphone may have startup time (i.e. INMP441 up to 83ms)
  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    // Block and wait for microphone values from I2S
    //
    // Data is moved from DMA buffers to our 'samples' buffer by the driver ISR
    // and when there is requested amount of data, task is unblocked
    //
    // Note: i2s_read does not care it is writing in float[] buffer, it will write
    //       integer values to the given address, as received from the hardware peripheral. 
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();
    
    // Convert (including shifting) integer microphone values to floats, 
    // using the same buffer (assumed sample size is same as size of float), 
    // to save a bit of memory
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for (int i=0; i<SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    if (oneshot) { // dump ~100 samples to the serial monitor
      oneshot = false;
      for (int i=0; i<SAMPLES_SHORT; i+=60) {
        Serial.printf("sample[%u]:\t%.1f\n", i, samples[i]);
      }
    }

    sum_queue_t q;
    // Apply equalization and calculate Z-weighted sum of squares, 
    // writes filtered samples back to the same buffer.
    q.sum_sqr_SPL = MIC_EQUALIZER.iir_filter(samples, samples, SAMPLES_SHORT);

    // Apply weighting and calucate weigthed sum of squares
    q.sum_sqr_weighted = WEIGHTING.iir_filter(samples, samples, SAMPLES_SHORT);

    // Debug only. Ticks we spent filtering and summing block of I2S data
    q.proc_ticks = xTaskGetTickCount() - start_tick;

    // Send the sums to FreeRTOS queue where main task will pick them up
    // and further calcualte decibel values (division, logarithms, etc...)
    xQueueSend(queueHandle, &q, portMAX_DELAY);
  }
}

//
// Setup and main loop
//
// Note: Use doubles, not floats, here unless you want to pin
//       the task to whichever core it happens to run on at the moment
//

void slm_init() {
  display_init();

  // Create FreeRTOS queue
  queueHandle = xQueueCreate(8, sizeof(sum_queue_t));

  // Create the I2S reader FreeRTOS task
  // NOTE: Current version of ESP-IDF will pin the task
  //       automatically to the first core it happens to run on
  //       (due to using the hardware FPU instructions).
  //       For manual control see: xTaskCreatePinnedToCore
  xTaskCreate(mic_i2s_reader_task, "Mic I2S Reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, &taskHandle);
}

void slm_suspend() {
  if (taskHandle) vTaskSuspend(taskHandle);
}

void slm_resume() {
  if (taskHandle) vTaskResume(taskHandle);
}

bool slm_loop(double& leq) {
  static uint32_t Leq_samples = 0;
  static double Leq_sum_sqr = 0;
  sum_queue_t q;

  // Read sum of samples, calculated by 'i2s_reader_task'
  if (xQueueReceive(queueHandle, &q, portMAX_DELAY)) {

    // Calculate dB values relative to MIC_REF_AMPL and adjust for microphone reference
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    // In case of acoustic overload or below noise floor measurement, report infinty Leq value
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    // Accumulate Leq sum
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    // When we gather enough samples, calculate new Leq value
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      double Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;
      
      // Serial output, customize (or remove) as needed
      //Serial.printf("%.1f\n", Leq_dB);

      // Debug only
      //Serial.printf("%u processing ticks\n", q.proc_ticks);

#ifdef USE_OLED_DISPLAY
      int indicator = 0;
      // It is important to somehow notify when the device is out of its range
      // as the calculated values are very likely with big error
      if (Leq_dB > MIC_OVERLOAD_DB) {
        indicator = 2;  // Display 'Overload' if dB value is over the AOP
      } else if (isnan(Leq_dB) || (Leq_dB < MIC_NOISE_DB)) {
        indicator = 1;  // Display 'Low' if dB value is below noise floor
      }
      // The 'short' Leq line
      double short_Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(double(q.sum_sqr_weighted) / SAMPLES_SHORT) / MIC_REF_AMPL);
      double level = ((short_Leq_dB - MIC_NOISE_DB) / MIC_OVERLOAD_DB);
      display_update(Leq_dB, level, indicator);
#endif // USE_OLED_DISPLAY

      leq = Leq_dB;
      return true;
    }
  }
  return false;
}

