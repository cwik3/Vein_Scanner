#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Arduino_GFX_Library.h>

#define IMG_W        160
#define IMG_H        120
#define SCALE        2
#define CHUNK        240
#define FRAME_BYTES  (IMG_W * IMG_H / 2)     // 9600 (4 bits per pixel)
#define NUM_CHUNKS   (FRAME_BYTES / CHUNK)   // 40

Arduino_DataBus *bus = new Arduino_ESP32SPI(9, 10, 12, 11, 13, FSPI);
Arduino_GFX *tft = new Arduino_ILI9342(bus, 8, 0);   // should print 320 x 240
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

struct __attribute__((packed)) Packet {
  uint8_t frame_id;
  uint8_t chunk_id;
  uint8_t len;
  uint8_t payload[CHUNK];
};

uint8_t bitmap[3][FRAME_BYTES];
volatile int  writeBuf   = 0;    // radio writes here
volatile int  readyBuf   = -1;   // newest complete frame (-1 = none)
volatile int  drawingBuf = -1;   // buffer the screen is reading
volatile bool frameReady = false;

uint8_t  custom_mac[] = {0x7C, 0x4F, 0xAD, 0xB9, 0xB2, 0x1C};
uint16_t palette[16];
uint16_t rowBuffer[IMG_W * SCALE * SCALE];   // 2 screen rows of 320 px

static int pickFreeBuf(int a, int b) {
  for (int i = 0; i < 3; i++) if (i != a && i != b) return i;
  return 0;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(Packet)) return;
  const Packet *pkt = (const Packet *)data;
  if (pkt->chunk_id >= NUM_CHUNKS || pkt->len != CHUNK) return;

  static uint8_t  curFrame = 255;
  static uint64_t mask = 0;
  if (pkt->frame_id != curFrame) { curFrame = pkt->frame_id; mask = 0; }

  memcpy(&bitmap[writeBuf][pkt->chunk_id * CHUNK], pkt->payload, CHUNK);
  mask |= (1ULL << pkt->chunk_id);

  // show only COMPLETE frames
  if (mask == ((1ULL << NUM_CHUNKS) - 1)) {
    mask = 0;
    portENTER_CRITICAL(&mux);          // <-- this line was missing
    readyBuf   = writeBuf;
    writeBuf   = pickFreeBuf(readyBuf, drawingBuf);
    frameReady = true;
    portEXIT_CRITICAL(&mux);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!tft->begin(40000000)) Serial.println("Display init failed!");
  tft->fillScreen(BLACK);

  // green palette (green is unaffected by the panel's red/blue swap)
  for (int i = 0; i < 16; i++) palette[i] = tft->color565(0, i * 17, 0);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, custom_mac);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW failed"); return; }
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_24M);
  esp_now_register_recv_cb(OnDataRecv);

  delay(100);
  tft->fillScreen(BLACK);
  tft->drawRect(0, 0, tft->width(), tft->height(), WHITE);
  Serial.printf("%d x %d\n", tft->width(), tft->height());      // clear again after WiFi start-up
  Serial.println("RX ready");
}

void loop() {
  if (!frameReady) return;
  int rb;
  portENTER_CRITICAL(&mux);
  rb = readyBuf;
  drawingBuf = rb;
  frameReady = false;
  portEXIT_CRITICAL(&mux);

  for (int y = 0; y < IMG_H; y++) {
    const uint8_t *src = &bitmap[rb][y * (IMG_W / 2)];
    for (int x = 0; x < IMG_W; x += 2) {
      uint8_t b = src[x >> 1];
      uint16_t c0 = palette[b >> 4];
      uint16_t c1 = palette[b & 15];
      rowBuffer[x * 2]     = c0;  rowBuffer[x * 2 + 1] = c0;
      rowBuffer[x * 2 + 2] = c1;  rowBuffer[x * 2 + 3] = c1;
    }
    memcpy(&rowBuffer[IMG_W * SCALE], rowBuffer, IMG_W * SCALE * sizeof(uint16_t));
    tft->draw16bitRGBBitmap(0, y * SCALE, rowBuffer, IMG_W * SCALE, SCALE);
  }
  drawingBuf = -1;
}