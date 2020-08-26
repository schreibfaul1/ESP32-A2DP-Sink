
#include <Arduino.h>

#include "a2dp_sink.h"

// Digital I/O used
#define I2S_DOUT      25
#define I2S_LRC       26
#define I2S_BCLK      27

char BT_SINK_NAME[]   = "ESP32-SPEAKER"; // sink devicename



void setup() {
    a2dp_sink_init(BT_SINK_NAME);
    set_i2s_pinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
}


void loop() {
}
