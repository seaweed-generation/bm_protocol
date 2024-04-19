#include "user_code.h"
#include "LineParser.h"
#include "OrderedSeparatorLineParser.h"
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

#define LED_ON_TIME_MS 20
#define LED_PERIOD_MS 1000
#define BYTES_CLUSTER_MS 50

#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_LINE_TERM 13 // CR / '\r', 0x0D

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

const ValueType valueTypes[] = {TYPE_DOUBLE, TYPE_DOUBLE};
// Declare the parser here with separator, buffer length, value types array, and number of values per line.
//   We'll initialize the parser later in setup to allocate all the memory we'll need.
OrderedSeparatorLineParser parser(",", 256, valueTypes, 2);


void setup(void) {
  /* USER ONE-TIME SETUP CODE GOES HERE */
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

  bristlefin.enable5V();
}

void loop(void) {
  /* USER LOOP CODE GOES HERE */

  // Read a line if it is available
  // Note - PLUART::setUseLineBuffer must be set true in setup to enable lines.
  if (PLUART::lineAvailable()) {
    uint16_t read_len =
        PLUART::readLine(payload_buffer, sizeof(payload_buffer));
    // Now when we get a line of text data, our LineParser turns it into numeric values.
    if (parser.parseLine(payload_buffer, read_len)) {
      printf("parsed values: %f \n", parser.getValue(0).data);
    } else {
      printf("Error parsing line!\n");
      return; // FIXME: this is a little confusing
    }
}
}
