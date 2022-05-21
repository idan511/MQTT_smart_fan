#include <BfButtonManager.h>
#include <BfButton.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"
#include "LED.h"
#include "Relay.h"

// WiFi credentials declared in creds.h or here
#define USE_CREDS

#ifdef USE_CREDS
#include "creds.h"
#else
const char *ssid = "";
const char *password = "";
#endif

// MQTT Broker
const char *mqtt_broker = "10.0.0.9";
const char *topic = "thafan/#";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 1883;

// MQTT Topics
const char *pwm_topic = "thafan/led/brightness/state";
const char *pwm_set_topic = "thafan/led/brightness/set";
const char *led_topic = "thafan/led/state";
const char *temp_topic = "thafan/temp";
const char *led_set_topic = "thafan/led/set";
const char *spd_topic = "thafan/spd/state";
const char *spd_set_topic = "thafan/spd/set";
const char *on_topic = "thafan/on/state";
const char *on_set_topic = "thafan/on/set";

// Number of speeds
const unsigned int num_of_speeds = 4;

// Led pins and pwm channels
LED leds[num_of_speeds] = {LED(3, 0), LED(5, 1), LED(7, 2), LED(9, 3)};

// Relay pins
Relay relays[num_of_speeds - 1] = {Relay(16), Relay(18), Relay(33)};

// one-wire pin
const unsigned int one_wire_pin = 11;
#define PARASITE_POWER  false

// Analog button pin
const unsigned int analog_buttons_pin = 1;

// light sensor pins
const unsigned int light_sensor_pin = 13;
const unsigned int light_sensor_enable_pin = 14;

// brightness update intervals
const unsigned int autobrightness_interval = 3; // seconds
const unsigned int pwm_animation_interval = 500; // micro-seconds
const unsigned int temperature_publish_interval = 60; // seconds

// EEPROM addresses
const unsigned int eeprom_size = 10;
const unsigned int speed_addr = 0;
const unsigned int brightness_addr = 1;

static Placeholder<OneWireNg_CurrentPlatform> _ow;

BfButtonManager manager(analog_buttons_pin, num_of_speeds);
BfButton btn1(BfButton::ANALOG_BUTTON_ARRAY, 0);
BfButton btn2(BfButton::ANALOG_BUTTON_ARRAY, 1);
BfButton btn3(BfButton::ANALOG_BUTTON_ARRAY, 2);
BfButton btn4(BfButton::ANALOG_BUTTON_ARRAY, 3);

static uint8_t last_spd = 0;
static uint8_t last_spd2 = 1;
static uint16_t pwm = 255;
static uint16_t true_pwm = 255;

WiFiClient espClient;
PubSubClient client(espClient);

volatile bool lflag = true, pflag = true, tflag = true;
hw_timer_t * timer[3] = {NULL, NULL, NULL}; 
uint16_t last_light = 255;

uint16_t update_pwm_targets() {
  uint16_t cur_light = get_light();
  int32_t target_pwm = (int32_t) pwm + cur_light - last_light;
  if (target_pwm > 1023) {
    target_pwm = 1023;
  } else if (target_pwm < 1) {
    target_pwm = 1;
  }
  last_light = cur_light;
  return target_pwm;
}

void init_light() {
  pinMode(light_sensor_enable_pin, OUTPUT);
  pinMode(light_sensor_pin, INPUT);
}

uint16_t get_light() {
  digitalWrite(light_sensor_enable_pin, HIGH);
  int r = analogRead(light_sensor_pin);
  digitalWrite(light_sensor_enable_pin, LOW);
  return (8191 - r) >> 3;
}

void init_fan_speed(uint8_t spd) {
  if (spd >= num_of_speeds) {
    return;
  }
  leds[spd].on();
  if (spd == 0) {
    last_spd2 = last_spd;
  } else {
    relays[spd - 1].on();
  }
  last_spd = spd;
  char tmp[5];
  EEPROM.put(speed_addr, last_spd);
  EEPROM.commit();
  snprintf(tmp, 5, "%d", last_spd);
  client.publish(spd_topic, tmp);
  client.publish(on_topic, last_spd > 0 ? "t" : "f");
}

void set_fan_speed(uint8_t spd) {
  if (spd >= num_of_speeds) {
    return;
  }
  if (spd != last_spd) {
    leds[last_spd].off();
    leds[spd].on();
    if (last_spd != 0) {
      relays[last_spd - 1].off();
      delay(15);
    }
    if (spd == 0) {
      last_spd2 = last_spd;
    } else {
      relays[spd - 1].on();
    }
    
    last_spd = spd;
  }
  char tmp[5];
  EEPROM.put(speed_addr, last_spd);
  EEPROM.commit();
  snprintf(tmp, 5, "%d", last_spd);
  client.publish(spd_topic, tmp);
  client.publish(on_topic, last_spd > 0 ? "t" : "f");
}

void set_pwm (uint16_t level) {  
  if (pwm == level) return;
  pwm = level;
  EEPROM.put(brightness_addr, pwm);
  EEPROM.commit();
  
  char tmp[15];
  snprintf(tmp, 15, "%u", pwm);
  client.publish(pwm_topic, tmp);
  client.publish(led_topic, pwm > 0 ? "t" : "f");
}

void publish_temp (const DSTherm::Scratchpad& scrpd) {  
  char tmp[15];
  long temp = scrpd.getTemp();
  snprintf(tmp, 15, "%f", (float) temp / 1000);
  client.publish(temp_topic, tmp);
}

void get_temps_and_publish() {
  DSTherm drv(_ow);
    Placeholder<DSTherm::Scratchpad> _scrpd;

    /* convert temperature on all sensors connected... */
    drv.convertTempAll(DSTherm::SCAN_BUS, PARASITE_POWER);

    /* ...and read them one-by-one */
    for (const auto& id: (OneWireNg&)_ow) {
        if (DSTherm::getFamilyName(id) != NULL) {
            if (drv.readScratchpad(id, &_scrpd) == OneWireNg::EC_SUCCESS)
                publish_temp(_scrpd);
            else
                Serial.println("  Invalid CRC!");
        }
    }
}

void setup_onewire() {
  #ifdef PWR_CTRL_PIN
# ifndef CONFIG_PWR_CTRL_ENABLED
#  error "CONFIG_PWR_CTRL_ENABLED needs to be enabled"
# endif
    new (&_ow) OneWireNg_CurrentPlatform(one_wire_pin, PWR_CTRL_PIN, false);
#else
    new (&_ow) OneWireNg_CurrentPlatform(one_wire_pin, false);
#endif
    DSTherm drv(_ow);

    Serial.begin(115200);
    delay(1500);

#if (CONFIG_MAX_SRCH_FILTERS > 0)
    static_assert(CONFIG_MAX_SRCH_FILTERS >= DSTherm::SUPPORTED_SLAVES_NUM,
        "CONFIG_MAX_SRCH_FILTERS too small");

    drv.filterSupportedSlaves();
#endif
}

int natoi(byte *s, unsigned int n) {
    int tmp = 0;
    while(n--) { 
        tmp = tmp * 10 + (*s - '0');      
        s++;
    }
    return tmp;
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.printf("%s [%u]: %.*s\n", topic, length, length, payload);
  if (strcmp(topic, pwm_set_topic) == 0) {
    set_pwm(natoi(payload, length));
  } else if(strcmp(topic, spd_set_topic) == 0) {
    set_fan_speed(natoi(payload, length));
  } else if(strcmp(topic, led_set_topic) == 0) {
    if (*payload == 'f') {
      set_pwm(0);
    }
  } else if(strcmp(topic, on_set_topic) == 0) {
    if (*payload == 'f') {
      set_fan_speed(0);
    } else if (*payload == 't') {
      set_fan_speed(last_spd2);
    }
  }
}

void pressHandler (BfButton *btn, BfButton::press_pattern_t pattern) {
  set_fan_speed(btn->getID());
  Serial.print(btn->getID());
  switch (pattern) {
    case BfButton::SINGLE_PRESS:
      Serial.println(" pressed.");
      break;
    case BfButton::DOUBLE_PRESS:
      Serial.println(" double pressed.");
      break;
    case BfButton::LONG_PRESS:
      Serial.println(" long pressed.");
      break;
  }
}


void toggle_pwm (BfButton *btn, BfButton::press_pattern_t pattern) {
  Serial.println("toggled");
  set_pwm(pwm == 0 ? get_light() : 0);
  last_light = pwm;
}

void setup_con() {
  // connecting to a WiFi network
  
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      leds[0].toggle();
  }

  leds[0].on();
  Serial.printf("MAC: %s\nIP: ", String(WiFi.macAddress()).c_str());
  Serial.println(WiFi.localIP());
  Serial.println("Connected to the WiFi network");
  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
          Serial.println("MQTT broker connected");
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  }

  client.subscribe(topic);
}

void IRAM_ATTR lflag_on() {
   lflag = true;
}

void IRAM_ATTR pflag_on() {
   pflag = true;
}

void IRAM_ATTR tflag_on() {
   tflag = true;
}

void setup() {

  EEPROM.begin(eeprom_size);
    
  //pwm = get_light();
  EEPROM.get(brightness_addr, pwm);
  last_light = pwm;
  true_pwm = pwm;
  for (LED& led : leds) {
    led.set_brightness(pwm);
  }
  leds[0].on();
  
  Serial.begin(115200);
  /*while (!Serial) {
    delay(200);
    leds[0].toggle();
  }
  leds[0].on();*/
  
  Serial.printf("Last pwm state: %u\n", pwm);

  btn1.onPress(pressHandler);
  btn1.onPressFor(toggle_pwm, 1000);
  manager.addButton(&btn1, 5300, 5800);

  btn2.onPress(pressHandler);
  //btn2.onPressFor(pressHandler, 1500);
  manager.addButton(&btn2, 5900, 6500);
  
  btn3.onPress(pressHandler);
  manager.addButton(&btn3, 6700, 7300);
  
  btn4.onPress(pressHandler);
  manager.addButton(&btn4, 7900, 8200);

  
  WiFi.begin(ssid, password);
  leds[1].on();
  setup_con();
  leds[2].on();
  
  pinMode(analog_buttons_pin, INPUT);
  init_light();

  setup_onewire();

  manager.begin();
  leds[3].on();

  timer[0] = timerBegin(0, 80, true);
  timerAttachInterrupt(timer[0], &lflag_on, true);
  timerAlarmWrite(timer[0], 1000000 * autobrightness_interval, true);
  timerAlarmEnable(timer[0]); 

  timer[1] = timerBegin(2, 80, true);
  timerAttachInterrupt(timer[1], &pflag_on, true);
  timerAlarmWrite(timer[1], pwm_animation_interval, true);
  timerAlarmEnable(timer[1]); 

  timer[2] = timerBegin(3, 80, true);
  timerAttachInterrupt(timer[2], &tflag_on, true);
  timerAlarmWrite(timer[2], 1000000 * temperature_publish_interval, true);
  timerAlarmEnable(timer[2]); 

  delay(1000);

  for (LED& led : leds) {
    led.off();
  }

  uint8_t tspd = 0;
  EEPROM.get(speed_addr, tspd);
  Serial.printf("Last speed state: %u\n", tspd);
  init_fan_speed(tspd);
}


void loop() {
  if (WiFi.status() != WL_CONNECTED || !client.connected()) {
    setup_con();
  }
  //Serial.printf("analog 1 = %d\n", analogRead(analog_buttons_pin));
  manager.loop();
  client.loop();
  if (lflag) {
    set_pwm(update_pwm_targets());
    lflag = false;
  }
  
  if (pflag && true_pwm != pwm) {
    true_pwm += pwm > true_pwm ? 1 : -1;
    for (LED& led : leds) {
      led.set_brightness(true_pwm);
    }
    pflag = false;
  }

  if (tflag) {
    get_temps_and_publish();
    tflag = false;
  }
}
