#include "Arduino_APDS9960_mod.h"

#define TRAIN // otherwise enter on-device test mode

#ifdef TRAIN
  //#define BUF_SIZE 192 // gathering training data (normal)
  #define BUF_SIZE 256 // gathering training data (large)
#else
  #include <asl_alphabet_inference.h>
  #define BUF_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
  float float_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
#endif

uint8_t buf[BUF_SIZE];

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor.");
    while (true);
  }
  Serial.println("Start");
}

void loop() {
  bool res = APDS.updateReadings(buf, BUF_SIZE);
#ifdef TRAIN
  if (res) {
    for (int i = 0; i+3 < BUF_SIZE; i+=4) {
      Serial.print(buf[i]);
      Serial.print(",");
      Serial.print(buf[i+1]);
      Serial.print(",");
      Serial.print(buf[i+2]);
      Serial.print(",");
      Serial.println(buf[i+3]);
    }
  }
#else
  if (res) {
    ei_impulse_result_t result = { 0 };
    
    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i++) {
      float_buf[i] = (float)(buf[i]);
    }
    int err = numpy::signal_from_buffer(float_buf, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
    } else {
      // invoke the impulse
      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
      //ei_printf("run_classifier returned: %d\n", res);
  
      if (res != 0) {
        Serial.println("res != 0");
      } else {
        // print the predictions
        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": \n");
        ei_printf("[");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("%.5f", result.classification[ix].value);
            if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
                ei_printf(", ");
            }
        }
        ei_printf("]\n");
    
        // human-readable predictions
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        }
        ei_printf("\n");
      }
    }
    delay(500);
  }
#endif
}
