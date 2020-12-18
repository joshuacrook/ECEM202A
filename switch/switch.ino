#include <mbed.h>
#include <WiFiEspAT.h>
#include <ArduinoBLE.h>
#include <ArduinoLifx.h>
#include "Arduino_APDS9960_mod.h"

#define NN_GESTURE

#ifdef NN_GESTURE
#include <switch_inference.h>
#endif

// tuning parameters
#define MOTION_TIMEOUT  5   // turn off lights after no motion (s) (max 1800 == 30 mins?) 0 for never
#define GESTURE_TIMEOUT 10  // ignore motion/brightness after gesture (s) (max 1800 == 30 mins?) 0 for never
#define LUM_TRANSITION  6   // turn lights on when luminance is below this value
#define PRIORITIZE_LUM  1   // only react to motion if lumance is below LUM_TRANSITION
#define BRIGHTNESS_DIFF 40  // % to change brightness by for left/right gestures

#define FADE_DELAY      500 // Light fade delay (ms)

// Pin definitions
//#define LED1 p13 // already defined
#define LEDR p24
#define LEDG p16
#define LEDB p6
#define MOTION_PIN  2   // D2
#define WIFI_EN     3   // D3 just leave floating

using namespace rtos;
using namespace mbed;
using namespace std::chrono;

DigitalOut led1(LED1);
DigitalOut ledr(LEDR);
DigitalOut ledg(LEDG);
DigitalOut ledb(LEDB);

struct Profile {
  uint16_t hue;
  uint16_t saturation;
  uint16_t brightness;
  uint16_t kelvin;
  uint32_t duration;
};

struct Profile profiles[3];
struct Profile current_profile; // future: ask light

WiFiUDP Udp;
ArduinoLifx lifx(Udp);
BLEService ble_service("73BCC38D-A6B8-4659-A324-BDE272188253");
BLEByteCharacteristic ble_characteristic("73BCC38D-A6B8-4659-A324-BDE272188253", BLERead | BLEWrite);

Timer motion_timer;
Timer gesture_timer;
bool light_state = 0; // future: ask light

#ifdef NN_GESTURE
uint8_t buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
float float_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
#endif

char target[] = "192.168.1.255";

#ifdef NN_GESTURE
int read_gesture_nn() {
  ei_impulse_result_t result = { 0 };
  int max_index = -1;
  float max_confidence;
  
  // Turn the raw buffer in a signal which we can the classify
  signal_t signal;
  for (int i = 0; i < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; i++) {
    float_buf[i] = (float)buf[i];
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
      max_index = 0;
      max_confidence = result.classification[0].value;
      for (size_t ix = 1; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
          float confidence = result.classification[ix].value;
          if (confidence > max_confidence) {
            max_index = ix;
            max_confidence = confidence;
          }
      }
      //ei_printf("Index: %d\n", max_index);
    }
  }
  return max_index;
}
#endif

void set_light(bool state) {
  lifx.setPower(state, FADE_DELAY);
  light_state = state;
}

void set_profile(struct Profile p) {
  memcpy(&current_profile, &p, sizeof(Profile));
  lifx.setColor(p.hue, p.saturation, p.brightness, p.kelvin, p.duration);
  // no res required, does not turn lights on
}

uint16_t calc_brightness(uint16_t raw, int8_t diff) {
  uint32_t val = raw + (diff*65535)/100;
  if (val < 0) {
    return 0;
  } else if (val > 65535) {
    return 65535;
  } else {
    return val;
  }
}

void gesture_response(int gest) {
  switch (gest) { // micro usb facing up
#ifdef NN_GESTURE
    case 0: // down
      Serial.println("Down");
      set_light(0);
      break;
    case 1: // left
      Serial.println("Left");
      current_profile.brightness = calc_brightness(current_profile.brightness, -BRIGHTNESS_DIFF);
      set_profile(current_profile);
      break;
    case 2: // right
      Serial.println("Right");
      current_profile.brightness = calc_brightness(current_profile.brightness, BRIGHTNESS_DIFF);
      set_profile(current_profile);
      break;
    case 3: // up
      Serial.println("Up");
      set_light(1);
      break;
    case 4: // up-1
      Serial.println("Up-1");
      set_profile(profiles[0]);
      set_light(1);
      break;
    case 5: // up-2
      Serial.println("Up-2");
      set_profile(profiles[1]);
      set_light(1);
      break;
    case 6: // up-3
      Serial.println("Up-3");
      set_profile(profiles[2]);
      set_light(1);
      break;
    default:
      Serial.print("Unknown classification ");
      Serial.println(gest);
#else
    case 0: // down
      Serial.println("Down");
      set_light(0);
      break;
    case 1: // up
      Serial.println("Up");
      set_light(1);
      break;
    case 2: // right
      Serial.println("Right");
      current_profile.brightness = calc_brightness(current_profile.brightness, BRIGHTNESS_DIFF);
      set_profile(current_profile);
      break;
    case 3: // left
      Serial.println("Left");
      current_profile.brightness = calc_brightness(current_profile.brightness, -BRIGHTNESS_DIFF);
      set_profile(current_profile);
      break;
#endif
  }
}

int get_gesture() {
#ifdef NN_GESTURE
  bool res = APDS.updateReadings(buf, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  if (res) {
    return read_gesture_nn();
  }
#else
  if (APDS.gestureAvailable()) {
    return APDS.readGesture();
  }
#endif
  return -1;
}

float get_brightness() {
  // Strange behavior: color not available unless proximityAvailable() called first
  if (APDS.proximityAvailable()) {}
  int iter = 2; // sometimes returns 0, run multiple times
  for (size_t i = 0; i < iter; i++) {
    if (APDS.colorAvailable()) {
      int r, g, b;
      APDS.readColor(r, g, b);
      float lum = 0.2126 * r + 0.7152 * g + 0.0722 * b; // ITU-R BT.709
      if (lum > 0) {
        return lum;
      }
    } else {
      return -1;
    }
  }
  return 0;
}

void init_wifi() {
  //pinMode(WIFI_EN, OUTPUT); // just leave floating
  //digitalWrite(WIFI_EN, HIGH);
  
  Serial1.begin(115200);
  Serial.println("Initializing WiFi");
  WiFi.init(Serial1);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println();
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  Serial.println("Waiting for connection to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();
  Serial.println("Success");

  char targetMac[] = "000000000000"; // broadcast
  IPAddress myIp = WiFi.localIP();
  lifx.begin(myIp, target, targetMac);
  lifx.setFlags(0, 0, 0); // broadcast, no ack/res required
}

void init_ble() {
  Serial.println("Initializing BLE");
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("Smart Switch");
  BLE.setAdvertisedService(ble_service);
  ble_service.addCharacteristic(ble_characteristic);
  BLE.addService(ble_service);
  BLE.setEventHandler(BLEConnected, ble_peripheral_connect_handler);
  BLE.setEventHandler(BLEDisconnected, ble_peripheral_disconnect_handler);
  ble_characteristic.setEventHandler(BLEWritten, ble_characteristic_written);
  ble_characteristic.setValue(0); // default value
  BLE.advertise();
  Serial.println("Success");
}

void ble_peripheral_connect_handler(BLEDevice central) {
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void ble_peripheral_disconnect_handler(BLEDevice central) {
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void ble_characteristic_written(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("Characteristic event, ");
  uint8_t event = ble_characteristic.value();
  if (event >= 0 && event < sizeof(profiles)/sizeof(struct Profile)) {
    Serial.print("updating to profile: ");
    Serial.println(event);
    set_profile(profiles[event]);
  } else {
    Serial.print("invalid profile: ");
    Serial.println(event);
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor.");
    while (true);
  }
  
  profiles[0] = {.hue=0, .saturation=0, .brightness=calc_brightness(65535,0), .kelvin=5000, .duration=500}; // soft daylight
  profiles[1] = {.hue=0, .saturation=0, .brightness=calc_brightness(65535,0), .kelvin=3500, .duration=500}; // neutral
  profiles[2] = {.hue=0, .saturation=0, .brightness=calc_brightness(65535,0), .kelvin=2000, .duration=500}; // sunset
  memcpy(&current_profile, &profiles[1], sizeof(Profile));

  init_wifi();
  init_ble();

  led1.write(1);
  ledr.write(1);
  ledg.write(1);
  ledb.write(1);
  motion_timer.start();
  gesture_timer.start();

  pinMode(MOTION_PIN, INPUT);
}

void loop() {
  // handle gesture
  int gest = get_gesture();
  if (gest > -1) {
    gesture_response(gest);
    gesture_timer.stop();
    gesture_timer.reset();
    gesture_timer.start();
    //Serial.println("Reset gesture timer");
    delay(250);
  }

  // update motion timer
  if (digitalRead(MOTION_PIN)) {
    motion_timer.stop();
    motion_timer.reset();
    motion_timer.start();
    //Serial.println("Reset motion timer");
  }

  // handle motion
  if (MOTION_TIMEOUT && (gesture_timer.read() > GESTURE_TIMEOUT)) {
    if (!PRIORITIZE_LUM && !light_state && (motion_timer.read() < MOTION_TIMEOUT)) {
      Serial.println("Turning lights on (detected motion)");
      set_light(1);
    }
    if (light_state && (motion_timer.read() > MOTION_TIMEOUT)) {
      Serial.println("Turning lights off (motion timeout)");
      set_light(0);
    }
  }

  // turn lights on if it's too dark
  float lum = get_brightness();
  if (!light_state && (lum > -1) && (lum < LUM_TRANSITION)              // lights off, valid reading, dark
      && (gesture_timer.read() > GESTURE_TIMEOUT)                       // no gesture timeout
      && (!MOTION_TIMEOUT || (motion_timer.read() < MOTION_TIMEOUT))) { // motion (or no motion timeout)
    Serial.println("Turning lights on (luminance + motion)");
    set_light(1);
  }

  /*if (gesture_timer.read() > GESTURE_TIMEOUT) {
    Serial.println("Gesture timeout");
  }
  if (motion_timer.read() > MOTION_TIMEOUT) {
    Serial.println("Motion timeout");
  }*/

  // poll BLE
  BLE.poll();
}
