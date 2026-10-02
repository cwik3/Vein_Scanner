#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Arduino_GFX_Library.h>

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  9 /* DC */, 
  10 /* CS */, 
  12 /* SCK */, 
  11 /* MOSI */, 
  13 /* MISO */, 
  FSPI /* spi_num */
);
Arduino_GFX *tft = new Arduino_ILI9341(bus, 8 /* RST */, 1 /* rotacja pozioma */);

struct __attribute__((packed)) Packet {
  uint8_t frame_id;
  uint8_t chunk_id;
  uint8_t len;
  uint8_t payload[180];
};

uint8_t binaryBitmap[512]; 
volatile bool frameReady = false;
uint8_t custom_mac[] = {0x7C, 0x4F, 0xAD, 0xB9, 0xB2, 0x1C};

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(Packet)) return;
  Packet pkt;
  memcpy(&pkt, incomingData, sizeof(pkt));
  
  if (pkt.chunk_id > 2) return;
  
  int offset = pkt.chunk_id * 180;
  if (offset + pkt.len > 512) return;

  memcpy(&binaryBitmap[offset], pkt.payload, pkt.len);

  if (pkt.chunk_id == 2) {
    frameReady = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- START BOOT ---");

  Serial.println("1. Inicjalizacja ekranu GFX...");
  if (!tft->begin()) {
    Serial.println("Blad inicjalizacji ekranu!");
  }

  tft->fillRect(0, 0, 320, 240, BLACK);

  tft->fillRect(100, 140, 40, 40, GREEN);
  Serial.println("Ekran OK. Zielony kwadrat na srodku.");

  Serial.println("2. Konfiguracja WiFi (MAC)...");
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, &custom_mac[0]);
  Serial.println("WiFi OK.");

  Serial.println("3. Start ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("--- ODBIORNIK GOTOWY DO PRACY ---");
}

void loop() {
  if (frameReady) {
    int bitIdx = 0;
    for (int y = 0; y < 64; y++) {
      for (int x = 0; x < 64; x++) {
        uint8_t byteVal = binaryBitmap[bitIdx / 8];
        bool isBright = (byteVal >> (7 - (bitIdx % 8))) & 1;
        
        uint16_t color = isBright ? GREEN : BLACK;
        tft->fillRect((x * 3) + 64, (y * 3) + 24, 3, 3, color);
        
        bitIdx++;
      }
    }
    frameReady = false;
  }
}