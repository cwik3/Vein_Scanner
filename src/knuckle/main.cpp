#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "esp_camera.h"

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define IMG_W        160
#define IMG_H        120
#define CHUNK        240
#define FRAME_BYTES  (IMG_W * IMG_H / 2)     // 9600 (4 bits per pixel)
#define NUM_CHUNKS   (FRAME_BYTES / CHUNK)   // 10

static uint8_t frameBuf[FRAME_BYTES];


uint8_t receiverAddress[] = {0x7C, 0x4F, 0xAD, 0xB9, 0xB2, 0x1C};
esp_now_peer_info_t peerInfo;

struct __attribute__((packed)) Packet {
  uint8_t frame_id;
  uint8_t chunk_id;
  uint8_t len;
  uint8_t payload[CHUNK];
};

static Packet pkt;
static uint8_t frameCounter = 0;
static uint8_t bitmap[FRAME_BYTES];
volatile bool sendDone = true;

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  sendDone = true;
}

void blinkForever() {
  while (1) {
    digitalWrite(33, LOW);  delay(100);
    digitalWrite(33, HIGH); delay(100);
  }
}

void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 12000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size   = FRAMESIZE_QQVGA;          
  config.jpeg_quality = 12;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.fb_count    = 2;                      
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.fb_count    = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  if (esp_camera_init(&config) != ESP_OK) {
    digitalWrite(33, LOW);
    while (1);
  }

  // Night-vision tuning
  sensor_t *s = esp_camera_sensor_get();
  s->set_gain_ctrl(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_gainceiling(s, GAINCEILING_8X);
  s->set_aec2(s, 1);          
  s->set_lenc(s, 1);          
  s->set_contrast(s, 1);
  s->set_hmirror(s, 1);    
  // s->set_vflip(s, 1);     
}

void setup() {
  pinMode(33, OUTPUT);
  digitalWrite(33, HIGH);
  Serial.begin(115200);

  initCamera();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) blinkForever();
  esp_now_register_send_cb(onSent);

  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_24M);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) blinkForever();

  digitalWrite(33, LOW); delay(1000); digitalWrite(33, HIGH);
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  uint16_t hist[256] = {0};
  const int N = IMG_W * IMG_H;
  for (int i = 0; i < N; i++) hist[fb->buf[i]]++;
  int lo = 0, hi = 255, acc = 0;
  for (int v = 0; v < 256; v++) { acc += hist[v]; if (acc > N / 50) { lo = v; break; } }
  acc = 0;
  for (int v = 255; v >= 0; v--) { acc += hist[v]; if (acc > N / 50) { hi = v; break; } }
  if (hi - lo < 16) hi = lo + 16;
  int range = hi - lo;


  for (int i = 0; i < N; i += 2) {
    int a = (fb->buf[i]     - lo) * 15 / range;
    int b = (fb->buf[i + 1] - lo) * 15 / range;
    a = constrain(a, 0, 15);
    b = constrain(b, 0, 15);
    frameBuf[i >> 1] = (a << 4) | b;
  }
  esp_camera_fb_return(fb);

  frameCounter++;
  for (int c = 0; c < NUM_CHUNKS; c++) {
    pkt.frame_id = frameCounter;
    pkt.chunk_id = c;
    pkt.len = CHUNK;
    memcpy(pkt.payload, &frameBuf[c * CHUNK], CHUNK);

    uint32_t t = micros();
    while (!sendDone && (micros() - t) < 5000) {}
    sendDone = false;
    esp_now_send(receiverAddress, (uint8_t *)&pkt, sizeof(pkt));
  }
}