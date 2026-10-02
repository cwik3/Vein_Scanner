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


uint8_t receiverAddress[] = {0x7C, 0x4F, 0xAD, 0xB9, 0xB2, 0x1C};
esp_now_peer_info_t peerInfo;


struct Packet {
  uint8_t frame_id;
  uint8_t chunk_id;   // 0, 1, 2
  uint8_t len;
  uint8_t payload[180];
};

Packet pkt;
uint8_t frameCounter = 0;
uint8_t binaryBitmap[512]; // 64x64 bits = 512 bytes

void initCamera() {
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE; 
  config.frame_size = FRAMESIZE_QQVGA;     
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {

    digitalWrite(33, LOW); 
    while (1);
  }
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

  if (esp_now_init() != ESP_OK) {
    // ESP-NOW FAILED: Blink rapidly forever
    while(1) {
      digitalWrite(33, LOW); delay(100); 
      digitalWrite(33, HIGH); delay(100);
    }
  }

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
     while(1) {
      digitalWrite(33, LOW); delay(100); 
      digitalWrite(33, HIGH); delay(100);
    }
  }

  // SUCCESS! Blink once slowly, then turn off and start the loop
  digitalWrite(33, LOW); delay(1000); digitalWrite(33, HIGH);
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  const int x_start = (160 - 64) / 2; 
  const int y_start = (120 - 64) / 2; 

  uint32_t sum = 0;
  for (int y = 0; y < 64; y++) {
    int rowOffset = (y_start + y) * 160;
    for (int x = 0; x < 64; x++) {
      sum += fb->buf[rowOffset + (x_start + x)];
    }
  }
  uint8_t threshold = sum / 4096;

  memset(binaryBitmap, 0, sizeof(binaryBitmap));
  int bitIdx = 0;
  for (int y = 0; y < 64; y++) {
    int rowOffset = (y_start + y) * 160;
    for (int x = 0; x < 64; x++) {
      uint8_t val = fb->buf[rowOffset + (x_start + x)];
      if (val > threshold) {
        binaryBitmap[bitIdx / 8] |= (1 << (7 - (bitIdx % 8)));
      }
      bitIdx++;
    }
  }
  esp_camera_fb_return(fb);

  frameCounter++;
  for (int c = 0; c < 3; c++) {
    pkt.frame_id = frameCounter;
    pkt.chunk_id = c;
    int offset = c * 180;
    int remaining = 512 - offset;
    pkt.len = (remaining > 180) ? 180 : remaining;
    memcpy(pkt.payload, &binaryBitmap[offset], pkt.len);

    esp_now_send(receiverAddress, (uint8_t *)&pkt, sizeof(pkt));
    delayMicroseconds(250); 
  }

  delay(30); 
}