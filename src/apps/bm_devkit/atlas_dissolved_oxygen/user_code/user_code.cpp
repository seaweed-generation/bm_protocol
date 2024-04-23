#include "user_code.h"
#include "LineParser.h"
#include "OrderedSeparatorLineParser.h"
#include "array_utils.h"
#include "avgSampler.h"
#include "bm_network.h"
#include "bm_printf.h"
#include "bm_pubsub.h"
#include "bristlefin.h"
#include "bsp.h"
#include "debug.h"
#include "lwip/inet.h"
#include "payload_uart.h"
#include "sensors.h"
#include "stm32_rtc.h"
#include "task_priorities.h"
#include "uptime.h"
#include "usart.h"
#include "util.h"

#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_LINE_TERM 13 // CR / '\r', 0x0D

/// For Turning Numbers Into Data
#define SAMPLE_PERIOD_MS 10000
// How often to compute and return statistics
#define AGG_PERIOD_MIN 5
// 10 min => 600,000 ms
#define AGG_PERIOD_MS (AGG_PERIOD_MIN * 60 * 1000)
/* We have enough RAM that we can keep it simple for shorter durations - use 64 bit doubles, buffer all readings.
   We could be much more RAM and precision efficient by using numerical methods like Kahan summation and Welford's algorithm.*/
#define MAX_SAMPLES ((AGG_PERIOD_MS / SAMPLE_PERIOD_MS) + 10)
typedef struct {
  uint16_t sample_count;
  double min;
  double max;
  double mean;
  double stdev;
} __attribute__((__packed__)) stats_message_t;
#define MESSAGE_SIZE sizeof(stats_message_t)

// Create an instance of the averaging sampler for our data
static AveragingSampler avg_data;

char payload_buffer[2048];

/// For Turning Text Into Numbers
/*
 * Setup a LineParser to turn the ASCII serial data from the EZO DO Circuit into numbers
 * Lines from the EZO DO Circuit look like:
 *    1.23(, 4.5)
 *    - comma-separated values
 *    - a floating point number representing the dissolved oxygen concentration in mg/L
 *    if enabled (- a floating point number representing the dissolved oxygen concentration in % saturation)
 *    - a 256 character buffer should be more than enough */

const ValueType valueTypes[] = {TYPE_DOUBLE};
// Declare the parser here with separator, buffer length, value types array, and number of values per line.
//   We'll initialize the parser later in setup to allocate all the memory we'll need.
OrderedSeparatorLineParser parser(",", 256, valueTypes, 1);


void setup(void) {
  /* USER ONE-TIME SETUP CODE GOES HERE */
  avg_data.initBuffer(MAX_SAMPLES);

  parser.init();
  // Setup the UART – the on-board serial driver that talks to the RS232 transceiver.
  PLUART::init(USER_TASK_PRIORITY);
  // Baud set per expected baud rate of the sensor.
  PLUART::setBaud(DEFAULT_BAUD_RATE);
  // Enable passing raw bytes to user app.
  PLUART::setUseByteStreamBuffer(true);
  // Enable parsing lines and passing to user app.
  /// Warning: PLUART only stores a single line at a time. If your attached payload sends lines
  /// faster than the app reads them, they will be overwritten and data will be lost.
  PLUART::setUseLineBuffer(true);
  // Set a line termination character per protocol of the sensor.
  PLUART::setTerminationCharacter((char)DEFAULT_LINE_TERM);
  // Turn on the UART.
  PLUART::enable();
}

void loop(void) {
  /* USER LOOP CODE GOES HERE */

  // This aggregates readings into stats, and sends them along to Spotter
  static u_int32_t agg_start_time = uptimeGetMs();
  if ((u_int32_t)uptimeGetMs() - agg_start_time >= AGG_PERIOD_MS) {
    agg_start_time = uptimeGetMs();
    double mean = 0, stdev = 0, min = 0, max = 0;
    uint16_t n_samples = 0;
    if (avg_data.getNumSamples()) {
      mean = avg_data.getMean();
      stdev = avg_data.getStd(mean);
      min = avg_data.getMin();
      max = avg_data.getMax();
      n_samples = avg_data.getNumSamples();
      avg_data.clear();
    }

    // Get the RTC if available
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    char rtcTimeBuffer[32];
    rtcPrint(rtcTimeBuffer, &time_and_date);

    bm_fprintf(0, "payload_data_agg.log",
               "tick: %llu, rtc: %s, n: %u, min: %.4f, max: %.4f, mean: %.4f, "
               "std: %.4f\n",
               uptimeGetMs(), rtcTimeBuffer, n_samples, min, max, mean, stdev);
    bm_printf(0,
              "[atlas-do-agg] | tick: %llu, rtc: %s, n: %u, min: %.4f, max: %.4f, "
              "mean: %.4f, std: %.4f",
              uptimeGetMs(), rtcTimeBuffer, n_samples, min, max, mean, stdev);
    printf("[atlas-do-agg] | tick: %llu, rtc: %s, n: %u, min: %.4f, max: %.4f, "
           "mean: %.4f, std: %.4f\n",
           uptimeGetMs(), rtcTimeBuffer, n_samples, min, max, mean, stdev);
    uint8_t tx_data[MESSAGE_SIZE] = {};
    stats_message_t tx_message = {
        .sample_count = n_samples, .min = min, .max = max, .mean = mean, .stdev = stdev};
    memcpy(tx_data, (uint8_t *)(&tx_message), MESSAGE_SIZE);
    if (spotter_tx_data(tx_data, MESSAGE_SIZE, BM_NETWORK_TYPE_CELLULAR_IRI_FALLBACK)) {
      printf("%llut - %s | Sucessfully sent Spotter transmit data request\n", uptimeGetMs(),
             rtcTimeBuffer);
    } else {
      printf("%llut - %s | Failed to send Spotter transmit data request\n", uptimeGetMs(),
             rtcTimeBuffer);
    }
  }

  // Read a line if it is available
  // Note - PLUART::setUseLineBuffer must be set true in setup to enable lines.
  if (PLUART::lineAvailable()) {
    // Get the RTC if available
    RTCTimeAndDate_t time_and_date = {};
    rtcGet(&time_and_date);
    char rtcTimeBuffer[32] = {};
    rtcPrint(rtcTimeBuffer, NULL);

    uint16_t read_len =
        PLUART::readLine(payload_buffer, sizeof(payload_buffer));

    bm_fprintf(0, "atlas_do_raw.log", "tick: %" PRIu64 ", rtc: %s, line: %.*s\n", uptimeGetMs(),
               rtcTimeBuffer, read_len, payload_buffer);
    bm_printf(0, "[atlas_do] | tick: %" PRIu64 ", rtc: %s, line: %.*s", uptimeGetMs(), rtcTimeBuffer,
              read_len, payload_buffer);
    printf("[atlas_do] | tick: %" PRIu64 ", rtc: %s, line: %.*s\n", uptimeGetMs(), rtcTimeBuffer,
           read_len, payload_buffer);


    // Now when we get a line of text data, our LineParser turns it into numeric values.
    if (parser.parseLine(payload_buffer, read_len)) {
      printf("parsed values: %f \n", parser.getValue(0).data);
    } else {
      printf("Error parsing line!\n");
      return; // FIXME: this is a little confusing
    }

    // Now let's aggregate those values into statistics
    if (avg_data.getNumSamples() >= MAX_SAMPLES) {
      printf("ERR - No more room in read buffer, already have %d "
             "readings!\n",
             MAX_SAMPLES);
      return; // FIXME: this is a little confusing
    }

    double reading = parser.getValue(0).data.double_val;
    avg_data.addSample(reading);

    printf("count: %u/%d, min: %f, max: %f\n", avg_data.getNumSamples(),
           MAX_SAMPLES, avg_data.getMin(), avg_data.getMax());

}
}
