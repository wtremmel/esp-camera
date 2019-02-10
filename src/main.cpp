
#include <Arduino.h>
#include <ArduinoLog.h>

#include <WiFi.h>
#include <SPIFFS.h>

#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
typedef enum {
    APP_GAP_STATE_IDLE = 0,
    APP_GAP_STATE_DEVICE_DISCOVERING,
    APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE,
    APP_GAP_STATE_SERVICE_DISCOVERING,
    APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE,
} app_gap_state_t;

typedef struct {
    bool dev_found;
    uint8_t bdname_len;
    uint8_t eir_len;
    uint8_t rssi;
    uint32_t cod;
    uint8_t eir[ESP_BT_GAP_EIR_DATA_LEN];
    uint8_t bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
    esp_bd_addr_t bda;
    app_gap_state_t state;
} app_gap_cb_t;

static app_gap_cb_t m_dev_info;

#define I2CSDA 21
#define I2CSCL 22

#include <PubSubClient.h>
#include <ArduinoJson.h>


// Sensor Libraries
#include <Wire.h>
#include "Adafruit_Si7021.h"
#include "Adafruit_BME280.h"
#include <Adafruit_NeoPixel.h>
#include <U8x8lib.h>

// Global defines
#define NEOPIXEL 14 //D5
#define NROFLEDS 10

// Defines for camera
#define PWDN_GPIO_NUM 26
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 32
#define SIOD_GPIO_NUM 13
#define SIOC_GPIO_NUM 12

#define Y9_GPIO_NUM 39
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 23
#define Y6_GPIO_NUM 18
#define Y5_GPIO_NUM 15
#define Y4_GPIO_NUM 4
#define Y3_GPIO_NUM 14
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 27
#define HREF_GPIO_NUM 25
#define PCLK_GPIO_NUM 19


// Global Objects
Adafruit_Si7021 si7021;
Adafruit_BME280 bme280;
Adafruit_NeoPixel led = Adafruit_NeoPixel(NROFLEDS, NEOPIXEL, NEO_GRB + NEO_KHZ800);
WiFiClient espClient;
PubSubClient client;
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

httpd_handle_t camera_httpd = NULL;


unsigned transmission_delay = 60; // seconds
uint32_t led_current_color;


// Strings for dynamic config
String Smyname, Spass, Sssid, Smqttserver, Ssite, Sroom, Smqttuser, Smqttpass;
unsigned int Imqttport;
bool Bflipped;


// Flags for sensors found
bool si7021_found = false;
bool bme280_found = false;
bool voltage_found= true;
bool u8x8_found = false;
bool rtc_init_done = false;
bool rtc_alarm_raised = false;
bool light_on = true;
bool camera_found = false;

// Flags for display
#define DISPLAY_OFF 0
#define DISPLAY_TEMPERATURE 1
#define DISPLAY_HUMIDITY 2
#define DISPLAY_AIRPRESSURE 3
#define DISPLAY_LUX 4
#define DISPLAY_STRING 5
#define DISPLAY_DISTANCE 6

unsigned int display_what = DISPLAY_TEMPERATURE;

// Timer variables
unsigned long now;
unsigned long last_transmission = 0;
unsigned long last_display = 0;

// forward declarations
boolean setup_wifi();

// LED routines
void setled(byte r, byte g, byte b) {
  led.setPixelColor(0, r, g, b);
  led_current_color = led.Color(r,g,b);
  if (light_on) {
    led.show();
  }
}

void setled(byte n, byte r, byte g, byte b) {
  led.setPixelColor(n, r, g, b);
  if (n == 0)
    led_current_color = led.Color(r,g,b);
  if (light_on) {
    led.show();
  }
}

void setled(byte n, byte r, byte g, byte b, byte show) {
  led.setPixelColor(n, r, g, b);
  if (light_on && show) {
    led.show();
  }
}

void setled(byte show) {
  if (!show) {
    int i;
    for (i = 0; i < NROFLEDS; i++) {
      setled(i, 0, 0, 0, 0);
    }
  }
  led.show();
}

// Debug functions
void log_config () {

  Log.verbose("Smyname = %s",Smyname.c_str());
  Log.verbose("Ssite = %s",Ssite.c_str());
  Log.verbose("Sroom = %s",Sroom.c_str());
  Log.verbose("Sssid = %s",Sssid.c_str());
  Log.verbose("Spass = %s",Spass.c_str());
  Log.verbose("Smqttuser = %s",Smqttuser.c_str());
  Log.verbose("Smqttpass = %s",Smqttpass.c_str());
  Log.verbose("Imqttport = %d",Imqttport);
  Log.verbose(F("Bflipped = %t"),Bflipped);

}

void write_config () {
  StaticJsonBuffer<512> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["myname"] = Smyname;
  root["flipped"] = Bflipped;
  JsonObject& network = root.createNestedObject("network");
  network["pass"] = Spass;
  network["ssid"] = Sssid;
  JsonObject& mqtt = root.createNestedObject("mqtt");
  mqtt["server"] = Smqttserver;
  mqtt["user"] = Smqttuser;
  mqtt["pass"] = Smqttpass;
  mqtt["port"] = Imqttport;
  JsonObject& location = root.createNestedObject("location");
  location["site"] = Ssite;
  location["room"] = Sroom;

  Log.notice(F("Writing new config file"));
  root.prettyPrintTo(Serial);

  SPIFFS.begin();
  File f = SPIFFS.open("/config.json","w");
  if (!f) {
    Log.error(F("Open of config file for writing failed"));
  } else {
    if (root.printTo(f) == 0) {
      Log.error(F("Writing object into file failed"));
    } else {
      Log.notice(F("Written new config. Now reboot"));
    }
    f.close();
  }
SPIFFS.end();
}

// Logging helper routines
void printTimestamp(Print* _logOutput) {
  char c[12];
  sprintf(c, "%10lu ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput) {
  _logOutput->print('\n');
}

// MQTT main callback routines
void mqtt_callback(char* topic, byte* payload, unsigned int length)  {

  String in[10];
  unsigned int wordcounter = 0;

  for (unsigned int i = 0; i < length; i++) {
    if ((char)payload[i] == ' ' && wordcounter < 9) {
      wordcounter++;
    } else {
      in[wordcounter] += String((char)payload[i]);
    }
  }
  Log.verbose("Message arrived[%s]: %d Words",topic,wordcounter);
  for (unsigned int i=0; i <= wordcounter; i++)
    Log.verbose("Word[%d] = %s",i,in[i].c_str());

  if (in[0] == "reboot") {
    ESP.restart();
  }

  if (in[0] == "led") {
    if (wordcounter == 3) {
      // led r g b
      setled(in[1].toInt(),in[2].toInt(),in[3].toInt());
    } else if (wordcounter == 4) {
      setled(in[1].toInt(),in[2].toInt(),in[3].toInt(),in[4].toInt());
    }
  }

  if (in[0] == F("config")) {
    if (wordcounter == 0) {
      log_config();
    }
    if (wordcounter == 1) {
      if (in[1] == F("write")) {
        log_config();
        write_config();
      }
    }
    if (wordcounter == 2) {
      if (in[1] == F("room")) {
        Sroom = in[2];
      }
      if (in[1] == F("site")) {
        Ssite = in[2];
      }
      if (in[1] == F("myname")) {
        Smyname = in[2];
      }
      if (in[1] == F("mqttuser")) {
        Smqttuser = in[2];
      }
      if (in[1] == F("mqttpass")) {
        Smqttpass = in[2];
      }
    }
  }

  if (in[0] == "display" && wordcounter >= 1) {
    last_display = 0;
    if (in[1] == "humidity") {
      display_what = DISPLAY_HUMIDITY;
    } else if (in[1] == "airpressure") {
      display_what = DISPLAY_AIRPRESSURE;
    } else if (in[1] == F("temperature")) {
      display_what = DISPLAY_TEMPERATURE;
    } else if (in[1] == F("distance")) {
      display_what = DISPLAY_DISTANCE;
    } else if (in[1] == "off") {
      u8x8.clearDisplay();
      display_what = DISPLAY_OFF;
    } else if (in[1] == F("flip")) {
      u8x8.clearDisplay();
      u8x8.setFlipMode(wordcounter > 1);
      Bflipped = (wordcounter > 1);
    } else { // String
      // large or small?
      // small, if a line is longer than 8 chars or if there are more than 3 lines
      boolean large = true;
      int start;
      for (unsigned int i=1; i<=wordcounter;i++) {
        if (in[i].length() > 8)
          large = false;
      }
      if (wordcounter > 3)
        large = false;

      if (wordcounter == 1)
        start = 3;
      else
        start = 0;

      display_what = DISPLAY_STRING;
      u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
      u8x8.clearDisplay();

      for (unsigned int i=1; i <= wordcounter; i++) {
        if (large) {
          u8x8.draw2x2UTF8(0, start, in[i].c_str());
          start += 3;
        } else {
          u8x8.draw1x2UTF8(0, start, in[i].c_str());
          start += 2;
        }
      }
    }
  }
}

boolean mqtt_reconnect() {
  // Loop until we're reconnected
  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/status", Ssite.c_str(), Sroom.c_str());

  if (WiFi.status() != WL_CONNECTED) {
    if (!setup_wifi())
      return false;
  }

  Log.verbose("Attempting MQTT connection...%d...",client.state());

  // Attempt to connect
  if (client.connect(Smyname.c_str(),Smqttuser.c_str(),Smqttpass.c_str(),mytopic,0,0,"stopped")) {
    Log.verbose("MQTT connected");

    client.publish(mytopic, "started");
    delay(10);
    // ... and resubscribe to my name
    client.subscribe(Smyname.c_str());
    delay(10);
  } else {
    Log.error("MQTT connect failed, rc=%d",client.state());
  }
  return client.connected();
}


unsigned long lastReconnectAttempt = 0;
void mqtt_publish(char *topic, char *msg) {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (mqtt_reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();

  Log.verbose("MQTT Publish message [%s]:%s",topic,msg);

  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/%s", Ssite.c_str(), Sroom.c_str(),topic);
  client.publish(mytopic, msg);
}

void mqtt_publish(char *topic, int i) {
  char buf[15];
  snprintf(buf,14,"%d",i);
  mqtt_publish(topic, buf);
}

void mqtt_publish(char *topic, uint32_t i) {
  char buf[32];
  snprintf(buf,31,"%lu",i);
  mqtt_publish(topic,buf);
}

void mqtt_publish(char *topic, float value) {
  char buf[15];
  snprintf(buf,14,"%.3f",value);
  mqtt_publish(topic, buf);
}




// Setup routines
//
// Scan for sensors
//
void setup_i2c() {
  byte error, address;

// 0x29 TSL45315 (Light)
// 0x38 VEML6070 (Light)
// 0x39 TSL2561
// 0x3c Display
// 0x40 SI7021
// 0x48 4*AD converter ADS1115
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680 (also BMP180)


  Log.notice("Scanning i2c bus");
  Wire.begin(I2CSDA, I2CSCL);

  // Try for DISPLAY
#if defined(BOARD_HELTEC)
  pinMode(16,OUTPUT);
  digitalWrite(16,LOW);
  delay(100);
  digitalWrite(16,HIGH);
#endif

  for(address = 1; address < 127; address++ ) {
    Log.verbose(F("Trying 0x%x"),address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Log.trace("I2C device found at address 0x%x",address);



      if (address == 0x3c) {
        u8x8_found = u8x8.begin();
        if (u8x8_found) {
          Log.notice("U8xu found? %T",u8x8_found);
          u8x8.clear();
          u8x8.setFont(u8x8_font_chroma48medium8_r);
          u8x8.setFlipMode(Bflipped);
        }
      }
      if (address == 0x40) {
        // SI7021
        si7021 = Adafruit_Si7021();
        si7021_found = si7021.begin();
        Log.notice("Si7021 found? %T",si7021_found);
      }
      if (address == 0x76 || address == 0x77) {
        // BME280
        bme280_found = bme280.begin(address);
        Log.notice("BME280 found? %T at 0x%x",bme280_found,address);
      }
    }
  }
  Log.notice("End scanning i2c bus");
}

void setup_serial() {
  Serial.begin(115200);
}

void setup_logging() {
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}

// we assume there is always a LED connected
void setup_led() {
  led.begin();
  led.show();
}

// Camera routinges
void setup_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  // config.pixel_format = PIXFORMAT_JPEG;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Log.error(F("Camera init fail: %d"),err);
  } else {
    Log.notice(F("Camera init ok"));
    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s,FRAMESIZE_QVGA);
    s->set_saturation(s,50000);
    camera_found= true;
  }

}

// read the config file and parse its data
void setup_readconfig() {
  SPIFFS.begin();
  File f = SPIFFS.open("/config.json","r");
  if (!f) {
    Log.error("Cannot open config file");
    return;
  }
  StaticJsonBuffer<512> jsonBuffer;

 // Parse the root object
 JsonObject &root = jsonBuffer.parseObject(f);

 if (!root.success())
   Log.error("Failed to read file");

 // Copy values from the JsonObject to the Config
   Smyname = root["myname"].as<String>();
   Bflipped = root["flipped"];
   Spass = root["network"]["pass"].as<String>();
   Sssid = root["network"]["ssid"].as<String>();
   Smqttserver = root["mqtt"]["server"].as<String>();
   Ssite = root["location"]["site"].as<String>();
   Sroom = root["location"]["room"].as<String>();
   Smqttuser = root["mqtt"]["user"].as<String>();
   Smqttpass = root["mqtt"]["pass"].as<String>();
   Imqttport = root["mqtt"]["port"];


  f.close();
  SPIFFS.end();
}

boolean setup_wifi() {
  WiFi.persistent(false);
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(Sssid.c_str(), Spass.c_str());

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    retries++;
    Log.error(F("Wifi.status() = %d"),WiFi.status());
    if (retries > 20) {
      Log.error(F("Cannot connect to %s"),Sssid.c_str());;
      return false;
    }
  }
  String myIP = String(WiFi.localIP().toString());
  String myMask = String(WiFi.subnetMask().toString());
  Log.verbose("Wifi connected as %s/%s",myIP.c_str(),myMask.c_str());
  return true;
}

void setup_mqtt() {
  client.setClient(espClient);
  client.setServer(Smqttserver.c_str(), Imqttport);
  client.setCallback(mqtt_callback);

}


// ESP32 Specific Setup routines

char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

void update_device_info(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    esp_bt_gap_dev_prop_t *p;

    Log.verbose("Device found: %s", bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            Log.verbose("--Class of Device: 0x%x", cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            Log.verbose("--RSSI: %d", rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
        default:
            break;
        }
    }

    /* search for device with MAJOR service class as "rendering" in COD */
    app_gap_cb_t *p_dev = &m_dev_info;
    if (p_dev->dev_found && 0 != memcmp(param->disc_res.bda, p_dev->bda, ESP_BD_ADDR_LEN)) {
        return;
    }

    if (!esp_bt_gap_is_valid_cod(cod) ||
            !(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_PHONE)) {
        return;
    }

    memcpy(p_dev->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
    p_dev->dev_found = true;
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            p_dev->cod = *(uint32_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            p_dev->rssi = *(int8_t *)(p->val);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME: {
            uint8_t len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN :
                          (uint8_t)p->len;
            memcpy(p_dev->bdname, (uint8_t *)(p->val), len);
            p_dev->bdname[len] = '\0';
            p_dev->bdname_len = len;
            break;
        }
        case ESP_BT_GAP_DEV_PROP_EIR: {
            memcpy(p_dev->eir, (uint8_t *)(p->val), p->len);
            p_dev->eir_len = p->len;
            break;
        }
        default:
            break;
        }
    }

    if (p_dev->eir && p_dev->bdname_len == 0) {
        get_name_from_eir(p_dev->eir, p_dev->bdname, &p_dev->bdname_len);
        Log.verbose("Found a target device, address %s, name %s", bda_str, (char *)(p_dev->bdname));
        // p_dev->state = APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE;
        // Log.verbose("Cancel device discovery ...");
        // esp_bt_gap_cancel_discovery();
    }
}


void callback_esp32_gap(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  char bda_str[18];
  char uuid_str[37];

  switch (event) {
    case 1:
      break;
    case ESP_BT_GAP_DISC_RES_EVT:
      sprintf(bda_str,"%02x:%02x:%02x:%02x:%02x:%02x",
        param->disc_res.bda[0],
        param->disc_res.bda[1],
        param->disc_res.bda[2],
        param->disc_res.bda[3],
        param->disc_res.bda[4],
        param->disc_res.bda[5]);
      Log.notice(F("BT Device found %s"),bda_str);
      update_device_info(param);

      break;
    default:
      Log.verbose(F("BT event %d"),event);
      break;
  }
}



void setup_esp32_bluetooth() {
  if (!btStart()) {
    Log.error(F("Failed to initialize bluetooth"));
    return;
  }

  if (esp_bluedroid_init() != ESP_OK) {
    Log.error(F("Failed to initialize bluedroid"));
    return;
  }

  if (esp_bluedroid_enable() != ESP_OK) {
    Log.error(F("Failed to enable bluedroid"));
    return;
  }

  esp_bt_dev_set_device_name(Smyname.c_str());
  esp_bt_gap_register_callback(callback_esp32_gap);
}

void setup_esp32() {
  setup_esp32_bluetooth();
}

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}


static esp_err_t index_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;

    fb = esp_camera_fb_get();
    if (!fb) {
        Log.error(F("Camera capture failed"));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

    size_t fb_len = 0;
    if(fb->format == PIXFORMAT_JPEG){
        fb_len = fb->len;
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
        jpg_chunking_t jchunk = {req, 0};
        res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
        httpd_resp_send_chunk(req, NULL, 0);
        fb_len = jchunk.len;
    }
    esp_camera_fb_return(fb);
    Log.notice(F("JPG: %u B "), (uint32_t)(fb_len));
    return res;
}



void setup_httpd(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
  };

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    Log.notice(F("http server on port %d started"),config.server_port);
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }

}

void setup() {
  setup_led();
  setled(255,0,0);
  delay(5000);
  setup_serial();
  setup_logging();
  setup_readconfig();
  log_config();
  setup_i2c();
  setup_camera();
  // setup_esp32();
  setled(255, 128, 0);
  if (setup_wifi()) {
    setup_mqtt();
    setled(0, 255, 0);
    delay(1000);
    setled(0,0,0);
  } else {
    setled(2,1,0);
  }
  setup_httpd();
}

void loop_publish_voltage(){

}


void loop_publish_bme280() {
  if (bme280_found) {
    mqtt_publish("temperature", bme280.readTemperature());
    mqtt_publish("airpressure", bme280.readPressure() / 100.0F);
    mqtt_publish("humidity", bme280.readHumidity());
  }
}


void lights_on(int dist) {
  bool x = false;

 if ((dist > 0) && (dist < 500))
    x = true;

  if (x == light_on)
    return;

  Log.verbose(F("Lights on? %T %d mm"),x,dist);

  light_on = x;

  if (x) {
    led.setPixelColor(0, led_current_color);
    led.show();
  } else {
    led_current_color = led.getPixelColor(0);
    led.clear();
    led.show();
  }

  if (u8x8_found) {
    if (x) {
      u8x8.setContrast(255);
    } else {
      u8x8.setContrast(0);
    }
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (mqtt_reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();

  // esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 10);

  // read sensors and publish values

  if ((millis() - last_transmission) > (transmission_delay * 1000)) {
  // Voltage
    loop_publish_voltage();
    loop_publish_bme280();

    last_transmission = millis();
  }


  if (u8x8_found && light_on && (((millis() - last_display) > (1000*30)) ||
      (display_what == DISPLAY_DISTANCE))) {
    char s[10];
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_r);
    switch(display_what) {
      case DISPLAY_TEMPERATURE:
        if (bme280_found) {
          snprintf(s, 9, "%.1f C", bme280.readTemperature());
          u8x8.clearDisplay();
          u8x8.draw2x2String(1, 3, s);
        }
        break;
      case DISPLAY_HUMIDITY:
        if (bme280_found) {
          snprintf(s, 9, "%.1f %%", bme280.readHumidity());
          u8x8.clearDisplay();
          u8x8.draw2x2String(1, 3, s);
        }
        break;
      case DISPLAY_AIRPRESSURE:
        if (bme280_found) {
          snprintf(s, 9, "%d hPa", (int)(bme280.readPressure() / 100));
          u8x8.clearDisplay();
          u8x8.draw2x2String(1, 3, s);
        }
        break;
    }

    last_display = millis();
  }
}
