/*
 * Implementation of Infinite Impulse Response (IRR) filter in
 * support of the ESP32 i2s Sound Level Meter (SLM) application
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
#include <Arduino.h>
#include "sos-iir-filter.h"

//
// TODO: investigate why the ASM implementation is returning results
//       approximately 2dB higher than the other two filter options
//
#define FILTER_ESP32_ASM     1  // optimal, but ESP32 only (ie. none of the risc core family of devices)
#define FILTER_ESP_ANSI_C    2  // maximum future portability - should always work
#define FILTER_ESP_DSP_LIB   3  // flexible device support, but at the whim of the vendor (see below)
//
// Select to use one of the filters implementations from above:
//
#define USE_FILTER_IMPLEMENTATION    FILTER_ESP_DSP_LIB
//#define USE_FILTER_IMPLEMENTATION    FILTER_ESP_ANSI_C
//#define USE_FILTER_IMPLEMENTATION    FILTER_ESP32_ASM


//
// IIR Filters
//

// DC-Blocker filter - removes DC component from I2S data
// See: https://www.dsprelated.com/freebooks/filters/DC_Blocker.html
// a1 = -0.9992 should heavily attenuate frequencies below 10Hz
SOS_IIR_Filter DC_BLOCKER = SOS_IIR_Filter(
  1.0,
  (const SOS_Coefficients[]) {
    {-1.0, 0.0, +0.9992, 0}
  }
);

//
// Equalizer IIR filters to flatten microphone frequency response
// See respective .m file for filter design. Fs = 48Khz.
//
// Filters are represented as Second-Order Sections cascade with assumption
// that b0 and a0 are equal to 1.0 and 'gain' is applied at the last step
// B and A coefficients were transformed with GNU Octave:
// [sos, gain] = tf2sos(B, A)
// See: https://www.dsprelated.com/freebooks/filters/Series_Second_Order_Sections.html
// NOTE: SOS matrix 'a1' and 'a2' coefficients are negatives of tf2sos output
//

// TDK/InvenSense ICS-43434
// Datasheet: https://www.invensense.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.1.pdf
// Datasheet: https://invensense.tdk.com/wp-content/uploads/2016/02/DS-000069-ICS-43434-v1.2.pdf
// B = [0.477326418836803, -0.486486982406126, -0.336455844522277, 0.234624646917202, 0.111023257388606];
// A = [1.0, -1.93073383849136326, 0.86519456089576796, 0.06442838283825100, 0.00111249298800616];
SOS_IIR_Filter ICS43434 = SOS_IIR_Filter(
  0.477326418836803,
  (const SOS_Coefficients[]) { // Second-Order Sections {b1, b2, -a1, -a2}
    {+0.96986791463971267, 0.23515976355743193, -0.06681948004769928, -0.00111521990688128},
    {-1.98905931743624453, 0.98908924206960169, +1.99755331853906037, -0.99755481510122113}
  }
);

// TDK/InvenSense ICS-43432
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/ICS-43432-data-sheet-v1.3.pdf
// B = [-0.45733702338341309   1.12228667105574775  -0.77818278904413563, 0.00968926337978037, 0.10345668405223755]
// A = [1.0, -3.3420781082912949, 4.4033694320978771, -3.0167072679918010, 1.2265536567647031, -0.2962229189311990, 0.0251085747458112]
SOS_IIR_Filter ICS43432 = SOS_IIR_Filter(
  -0.457337023383413,
  (const SOS_Coefficients[]) { // Second-Order Sections {b1, b2, -a1, -a2}
    {-0.544047931916859, -0.248361759321800, +0.403298891662298, -0.207346186351843},
    {-1.909911869441421, +0.910830292683527, +1.790285722826743, -0.804085812369134},
    {+0.000000000000000, +0.000000000000000, +1.148493493802252, -0.150599527756651}
  }
);

// TDK/InvenSense INMP441
// Datasheet: https://www.invensense.com/wp-content/uploads/2015/02/INMP441.pdf
// B ~= [1.00198, -1.99085, 0.98892]
// A ~= [1.0, -1.99518, 0.99518]
SOS_IIR_Filter INMP441 = SOS_IIR_Filter(
  1.00197834654696,
  (const SOS_Coefficients[]) { // Second-Order Sections {b1, b2, -a1, -a2}
    {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
  }
);

// Infineon IM69D130 Shield2Go
// Datasheet: https://www.infineon.com/dgdl/Infineon-IM69D130-DS-v01_00-EN.pdf?fileId=5546d462602a9dc801607a0e46511a2e
// B ~= [1.001240684967527, -1.996936108836337, 0.995703101823006]
// A ~= [1.0, -1.997675693595542, 0.997677044195563]
// With additional DC blocking component
SOS_IIR_Filter IM69D130 = SOS_IIR_Filter(
  1.00124068496753,
  (const SOS_Coefficients[]) { // Second-Order Sections {b1, b2, -a1, -a2}
    {-1.0, 0.0, +0.9992, 0}, // DC blocker, a1 = -0.9992
    {-1.994461610298131, 0.994469278738208, +1.997675693595542, -0.997677044195563}
  }
);

// Knowles SPH0645LM4H-B, rev. B
// https://cdn-shop.adafruit.com/product-files/3421/i2S+Datasheet.PDF
// B ~= [1.001234, -1.991352, 0.990149]
// A ~= [1.0, -1.993853, 0.993863]
// With additional DC blocking component
SOS_IIR_Filter SPH0645LM4H_B_RB = SOS_IIR_Filter(
  1.00123377961525,
  (const SOS_Coefficients[]) { // Second-Order Sections {b1, b2, -a1, -a2}
    {-1.0, 0.0, +0.9992, 0}, // DC blocker, a1 = -0.9992
    {-1.988897663539382, +0.988928479008099, +1.993853376183491, -0.993862821429572}
  }
);

//
// Weighting filters
//

//
// A-weighting IIR Filter, Fs = 48KHz
// (By Dr. Matt L., Source: https://dsp.stackexchange.com/a/36122)
// B = [0.169994948147430, 0.280415310498794,  -1.120574766348363,   0.131562559965936,    0.974153561246036,  -0.282740857326553,  -0.152810756202003]
// A = [1.0,              -2.12979364760736134, 0.42996125885751674, 1.62132698199721426, -0.96669962900852902, 0.00121015844426781, 0.04400300696788968]
SOS_IIR_Filter A_weighting = SOS_IIR_Filter(
  0.169994948147430,
  (const SOS_Coefficients[]) { // Second-Order Sections {b1, b2, -a1, -a2}
    {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
    {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
    {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
  }
);

//
// C-weighting IIR Filter, Fs = 48KHz
// Designed by invfreqz curve-fitting, see respective .m file
// B = [-0.49164716933714026, 0.14844753846498662, 0.74117815661529129, -0.03281878334039314, -0.29709276192593875, -0.06442545322197900, -0.00364152725482682]
// A = [ 1.0,                -1.0325358998928318, -0.9524000181023488,   0.8936404694728326,   0.2256286147169398,  -0.1499917107550188,   0.0156718181681081]
SOS_IIR_Filter C_weighting = SOS_IIR_Filter(
  -0.491647169337140,
  (const SOS_Coefficients[]) {
    {+1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
    {+0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
    {-2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}
  }
);

No_IIR_Filter None;


#if (USE_FILTER_IMPLEMENTATION == FILTER_ESP_ANSI_C)

int sos_filter_f32(float *input, float *output, int len, const SOS_Coefficients& coeffs, SOS_Delay_State& w) {
  for (int i = 0; i < len; i++) {
    float d0 = input[i] + (coeffs.a1 * w.w0) + (coeffs.a2 * w.w1);
    output[i] =      d0 + (coeffs.b1 * w.w0) + (coeffs.b2 * w.w1);
    w.w1 = w.w0;
    w.w0 = d0;
  }
  return 0;
}

float sos_filter_sum_sqr_f32(float *input, float *output, int len, const SOS_Coefficients& coeffs, SOS_Delay_State& w, float gain) {
  float sum_sqr = 0.0f;
  for (int i = 0; i < len; i++) {
    float x = input[i];
    float w_new = x + coeffs.a1 * w.w0 + coeffs.a2 * w.w1;
    float y = w_new + coeffs.b1 * w.w0 + coeffs.b2 * w.w1;
    y *= gain;
    output[i] = y;
    sum_sqr += y * y;
    w.w1 = w.w0;
    w.w0 = w_new;
  }
  return sum_sqr;
}

#endif // FILTER_ESP_ANSI_C

#if (USE_FILTER_IMPLEMENTATION == FILTER_ESP32_ASM)

extern "C" {
  int sos_filter_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w);
}
__asm__ (
  //
  // ESP32 implementation of IIR Second-Order Section filter
  // Assumes a0 and b0 coefficients are one (1.0)
  //
  // float* a2 = input;
  // float* a3 = output;
  // int    a4 = len;
  // float* a5 = coeffs;
  // float* a6 = w;
  // float  a7 = gain;
  //
  ".text                    \n"
  ".align  4                \n"
  ".global sos_filter_f32   \n"
  ".type   sos_filter_f32,@function\n"
  "sos_filter_f32:          \n"
  "  entry   a1, 16         \n"
  "  lsi     f0, a5, 0      \n" // float f0 = coeffs.b1;
  "  lsi     f1, a5, 4      \n" // float f1 = coeffs.b2;
  "  lsi     f2, a5, 8      \n" // float f2 = coeffs.a1;
  "  lsi     f3, a5, 12     \n" // float f3 = coeffs.a2;
  "  lsi     f4, a6, 0      \n" // float f4 = w[0];
  "  lsi     f5, a6, 4      \n" // float f5 = w[1];
  "  loopnez a4, 1f         \n" // for (; len>0; len--) {
  "    lsip    f6, a2, 4    \n" //   float f6 = *input++;
  "    madd.s  f6, f2, f4   \n" //   f6 += f2 * f4; // coeffs.a1 * w0
  "    madd.s  f6, f3, f5   \n" //   f6 += f3 * f5; // coeffs.a2 * w1
  "    mov.s   f7, f6       \n" //   f7 = f6; // b0 assumed 1.0
  "    madd.s  f7, f0, f4   \n" //   f7 += f0 * f4; // coeffs.b1 * w0
  "    madd.s  f7, f1, f5   \n" //   f7 += f1 * f5; // coeffs.b2 * w1 -> result
  "    ssip    f7, a3, 4    \n" //   *output++ = f7;
  "    mov.s   f5, f4       \n" //   f5 = f4; // w1 = w0
  "    mov.s   f4, f6       \n" //   f4 = f6; // w0 = f6
  "  1:                     \n" // }
  "  ssi     f4, a6, 0      \n" // w[0] = f4;
  "  ssi     f5, a6, 4      \n" // w[1] = f5;
  "  movi.n   a2, 0         \n" // return 0;
  "  retw.n                 \n"
);

extern "C" {
  float sos_filter_sum_sqr_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w, float gain);
}
__asm__ (
  //
  // ESP32 implementation of IIR Second-Order section filter with applied gain.
  // Assumes a0 and b0 coefficients are one (1.0)
  // Returns sum of squares of filtered samples
  //
  // float* a2 = input;
  // float* a3 = output;
  // int    a4 = len;
  // float* a5 = coeffs;
  // float* a6 = w;
  // float  a7 = gain;
  //
  ".text                    \n"
  ".align  4                \n"
  ".global sos_filter_sum_sqr_f32 \n"
  ".type   sos_filter_sum_sqr_f32,@function \n"
  "sos_filter_sum_sqr_f32:  \n"
  "  entry   a1, 16         \n"
  "  lsi     f0, a5, 0      \n"  // float f0 = coeffs.b1;
  "  lsi     f1, a5, 4      \n"  // float f1 = coeffs.b2;
  "  lsi     f2, a5, 8      \n"  // float f2 = coeffs.a1;
  "  lsi     f3, a5, 12     \n"  // float f3 = coeffs.a2;
  "  lsi     f4, a6, 0      \n"  // float f4 = w[0];
  "  lsi     f5, a6, 4      \n"  // float f5 = w[1];
  "  wfr     f6, a7         \n"  // float f6 = gain;
  "  const.s f10, 0         \n"  // float sum_sqr = 0;
  "  loopnez a4, 1f         \n"  // for (; len>0; len--) {
  "    lsip    f7, a2, 4    \n"  //   float f7 = *input++;
  "    madd.s  f7, f2, f4   \n"  //   f7 += f2 * f4; // coeffs.a1 * w0
  "    madd.s  f7, f3, f5   \n"  //   f7 += f3 * f5; // coeffs.a2 * w1;
  "    mov.s   f8, f7       \n"  //   f8 = f7; // b0 assumed 1.0
  "    madd.s  f8, f0, f4   \n"  //   f8 += f0 * f4; // coeffs.b1 * w0;
  "    madd.s  f8, f1, f5   \n"  //   f8 += f1 * f5; // coeffs.b2 * w1;
  "    mul.s   f9, f8, f6   \n"  //   f9 = f8 * f6;  // f8 * gain -> result
  "    ssip    f9, a3, 4    \n"  //   *output++ = f9;
  "    mov.s   f5, f4       \n"  //   f5 = f4; // w1 = w0
  "    mov.s   f4, f7       \n"  //   f4 = f7; // w0 = f7;
  "    madd.s  f10, f9, f9  \n"  //   f10 += f9 * f9; // sum_sqr += f9 * f9;
  "  1:                     \n"  // }
  "  ssi     f4, a6, 0      \n"  // w[0] = f4;
  "  ssi     f5, a6, 4      \n"  // w[1] = f5;
  "  rfr     a2, f10        \n"  // return sum_sqr;
  "  retw.n                 \n"  //
);

#endif // FILTER_ESP32_ASM

#if (USE_FILTER_IMPLEMENTATION == FILTER_ESP_DSP_LIB)

#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 5)) // Bug was introduced in esp_dsp lib as included with 4.4.5 onwards
#include "esp_dsp.h"
#include "dsps_biquad.h"
#else
//
// Avoid a bug in later esp-idf dsp_lib versions by using local fixed version featured here
// A fix was committed to esp-idf dsp_lib on 19 May 2025 but as of the time of writing
//  a new release candidate with checkable version identifier - is unknown
// At some future point the version test above should exclude only the affected idf releases
//
// https://github.com/espressif/esp-dsp/pull/109#issuecomment-2891685542
//
extern "C" {
    esp_err_t _dsps_biquad_f32_ae32(const float* input, float* output, int len, float* coef, float* w);
  }
  __asm__ (
    ".text                  \n"
    ".align  4              \n"
    ".global _dsps_biquad_f32_ae32 \n"
    ".type   _dsps_biquad_f32_ae32,@function \n"
    "_dsps_biquad_f32_ae32: \n"
    "	entry	a1, 16          \n"
    "	lsi   f0, a5, 0       \n"
    "	lsi   f1, a5, 4       \n"
    "	lsi   f2, a5, 8       \n"
    "	lsi   f3, a5, 12      \n"
    "	lsi   f4, a5, 16      \n"
    "	neg.s  f5, f3         \n" // -a[1]
    "	neg.s  f6, f4         \n" // -a[2]
    "	lsi   f7, a6, 0       \n" // w[0]
    "	lsi   f8, a6, 4       \n" // w[1]
    "	lsip     f9, a2, 4    \n" // f9 = x[i]
    "	loopnez a4, .loop_bq_end_m_ae32 \n"
    "		madd.s  f9, f7, f5  \n" // f9 += -a1*w0
    "		mul.s   f10, f1, f7 \n" // f10 = b1*w0
    "		madd.s  f9, f8, f6  \n" // f9 += -a2*w1
    "		madd.s  f10, f9, f0 \n" // f10 += b0*d0
  //  "		addi    a2, a2, 4   \n" // in++;  (BUG; THIS INSTRUCTION MUST NOT BE HERE)
    "		madd.s  f10, f2, f8 \n" // f10+= b2*w1, f10 - result
    "		mov.s   f8, f7      \n" // w1 = w0
    "		mov.s   f7, f9      \n" // w0 = d0
    "		lsip    f9, a2,  4  \n" // f9 = x[i]
    "		ssip	f10, a3, 4    \n" // y[i] = result
    ".loop_bq_end_m_ae32:   \n"
    "		ssi     f7, a6, 0   \n"
    "		ssi     f8, a6, 4   \n"
    "	movi.n	a2, 0         \n" // return status ESP_OK
    "	retw.n                \n"
  );
#endif

int sos_filter_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w) {
  float coeffs_arr[5] = {1.0f, coeffs.b1, coeffs.b2, -coeffs.a1, -coeffs.a2};  // ESP-DSP expects 5 coefficients: b0, b1, b2, a1, a2
#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 5))
  dsps_biquad_f32(input, output, len, coeffs_arr, (float*)&w);
#else
  _dsps_biquad_f32_ae32(input, output, len, coeffs_arr, (float*)&w);
#endif
  return 0;
}

float sos_filter_sum_sqr_f32(float *input, float *output, int len, const SOS_Coefficients &coeffs, SOS_Delay_State &w, float gain) {
  float coeffs_arr[5] = {1.0f, coeffs.b1, coeffs.b2, -coeffs.a1, -coeffs.a2};
#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 5))
  dsps_biquad_f32(input, output, len, coeffs_arr, (float*)&w);
#else
  _dsps_biquad_f32_ae32(input, output, len, coeffs_arr, (float*)&w);
#endif
  float sum_sqr = 0.0f;
  for (int i = 0; i < len; i++) {
    output[i] *= gain;
    sum_sqr += output[i] * output[i];
  }
  return sum_sqr;
}

#endif // FILTER_ESP_DSP_LIB

