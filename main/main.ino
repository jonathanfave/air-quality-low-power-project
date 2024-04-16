#include <SPI.h> // include libraries
#include <LoRa.h>
#include "mbedtls/aes.h" // accelerated AES module
#include "LoRa.TS.par.h"
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_SGP30.h"

Adafruit_SGP30 sgp;
int counter = 0;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
#define TIME_TO_SLEEP 10 // it corresponds to cycle time
RTC_DATA_ATTR float stemp, shumi; // saved to next cycle
RTC_DATA_ATTR float delta = 10.0, cycle = 10000.0, kpack = 100.0;
#define mS_TO_S_FACTOR 1000
uint32_t time_sec = TIME_TO_SLEEP;

#define SCK 5 // GPIO5 -- SX127x's SCK
#define MISO 19 // GPIO19 -- SX127x's MISO
#define MOSI 27 // GPIO27 -- SX127x's MOSI
#define SS 18 // GPIO18 -- SX127x's CS (NSS)
#define RST 14 // GPIO14 -- SX127x's RESET
#define DI0 26 // GPIO26 -- SX127x's IRQ(DIO0)

ts_t ts;
lora_t lora;

typedef union
{
  uint8_t frame[54];
  struct
  {
    uint32_t channel;
    uint8_t flag[2];
    char wkey[16];
    float sens[8];
  } pay;
} pack_t;

pack_t sdp, enc_sdp;
typedef union
{
  uint8_t frame[22];
  struct
  {
    uint32_t channel; // thingspeak channel number - identifier
    char flag[2]; // validation-information flags
    float data[4]; // ACK data or parameters: cycle time,delta,kpacket, etc.
  } pay;
} ack_t;

ack_t ack, enc_ack; // for clear and encrypted ACK packet

void encrypt(char *plainText, char *key, unsigned char *outputBuffer, int nblocks) {
  mbedtls_aes_context aes;
  mbedtls_aes_init( &aes );
  mbedtls_aes_setkey_enc( &aes, (const unsigned char*)key, strlen(key) * 8 );

  for (int i = 0; i < nblocks; i++)
  {
    mbedtls_aes_crypt_ecb( &aes, MBEDTLS_AES_ENCRYPT, (const unsigned char*)(plainText + i * 16),
                           outputBuffer + i * 16);
  }

  mbedtls_aes_free( &aes );
}

void decrypt(unsigned char *chipherText, char *key, unsigned char *outputBuffer, int nblocks)
{
  mbedtls_aes_context aes;
  mbedtls_aes_init( &aes );
  mbedtls_aes_setkey_dec( &aes, (const unsigned char*) key, strlen(key) * 8 );
  for (int i = 0; i < nblocks; i++)
  {
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT,
                          (const unsigned char*)(chipherText + i * 16), outputBuffer + i * 16);
  }

  mbedtls_aes_free(&aes );
}

QueueHandle_t ackqueue; // packet queue

void onReceive(int packetSize)
{
  int i = 0;
  ack_t enc_ack;
  if (packetSize == 0) return; // if there's no packet, return
  i = 0;
  if (packetSize == 22)
  {
    while (LoRa.available()) {
      enc_ack.frame[i] = LoRa.read(); i++;
    }

    Serial.println(LoRa.packetRssi());
    xQueueReset(ackqueue); // to keep only the last element
    xQueueSend(ackqueue, &enc_ack, portMAX_DELAY);
  }
}

float temp, humi;
float tvoc, eco2, rawH2, rawEthanol;

int readSGP30() 
{
  int val_ret = 0;
  Wire.begin(21, 22); delay(100);
  if (! sgp.begin()) {
    Serial.println("Sensor not found :(");
    while (1);
  }

  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    val_ret = 0;
  }
  else {
    Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
    Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.print(" ppm\t");
    val_ret = 1;
  }

  if (! sgp.IAQmeasureRaw()) {
    Serial.println("Raw Measurement failed");
    val_ret = 0;
  }
  else {
    Serial.print("Raw H2 "); Serial.print(sgp.rawH2); Serial.print(" \t");
    Serial.print("Raw Ethanol "); Serial.print(sgp.rawEthanol); Serial.println("");
    val_ret = 1;
  }

  delay(1000);

  counter++;
  if (counter == 30) {
    counter = 0;

    uint16_t TVOC_base, eCO2_base;
    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
      Serial.println("Failed to get baseline readings");
      val_ret = 0;
    }
    else {
      Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
      Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
      val_ret = 1;
    }
  }

  tvoc = sgp.TVOC;
  eco2 = sgp.eCO2;
  rawH2 = sgp.rawH2;
  rawEthanol = sgp.rawEthanol;

  //  temp = sht31.readTemperature();
  //  humi = sht31.readHumidity(); delay(100); Serial.println();
  Wire.end();
  return val_ret;
}

int toSend() {

}

void setup() {
  ack_t ack, enc_ack;
  BaseType_t qret; // from FreeRTOS
  Serial.begin(115200);
  if (readSGP30()) {
    pinMode(DI0, INPUT); // signal interrupt
    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DI0);
    readEEPROM(&ts, &lora);
    Serial.printf("TS channel: %d\n", ts.par.channel);
    Serial.printf("TS write key: %16.16s\n", ts.par.wkey);
    Serial.printf("frequency: %d\n", lora.par.freq);
    Serial.printf("bandwidth: %d\n", lora.par.bw);
    Serial.printf("spreading factor: %d\n", lora.par.sf);
    Serial.printf("coding rate: %d/8\n", lora.par.cr);
    Serial.print("AES key (HEX): ");
    for (int i = 0; i < 16; i++) Serial.print(lora.par.aeskey[i], HEX);
    Serial.println();
    Serial.printf("AES key (ASCII): %16.16s \n", lora.par.aeskey);
    if (!LoRa.begin(lora.par.freq)) {
      Serial.println("Starting LoRa failed!");
      while (1);
    }

    else Serial.println("Starting LoRa OK!");

    LoRa.setSpreadingFactor(lora.par.sf);
    LoRa.setSignalBandwidth (lora.par.bw);
    LoRa.setCodingRate4(lora.par.cr);
    ackqueue = xQueueCreate(4, 22); // sizeof rdp union
    LoRa.onReceive(onReceive); // link to ISR
    char key[17] = "abcdefghijklmnop";
    Serial.println("New Packet") ;
    LoRa.beginPacket();
    strncpy(sdp.pay.wkey, "JH9SEFA2WVTHCH64", 16); // trouve sur TS
    sdp.pay.sens[0] = tvoc;
    sdp.pay.sens[1] = eco2;
    sdp.pay.sens[2] = rawH2;
    sdp.pay.sens[3] = rawEthanol;
    Serial.printf("tvoc=%f, eco2=%f, rawH2=%f, rawEthanol=%f\n", tvoc, eco2, rawH2, rawEthanol);
    Serial.println(key);
    Serial.println("\nClear packet");
    for (int i = 6; i < 54; i++) Serial.print(sdp.frame[i], HEX); Serial.println();
    encrypt((char *)sdp.frame + 6, key, enc_sdp.frame + 6, 3);
    Serial.println("Encrypted packet");
    for (int i = 6; i < 54; i++) Serial.print(enc_sdp.frame[i], HEX); Serial.println();
    // non ecrypted data: channel number and flags
    enc_sdp.pay.channel = 2475620; //numereo de canal que l'on trouve sur TS
    enc_sdp.pay.flag[0] = 0xC0;
    LoRa.write(enc_sdp.frame, 54);
    LoRa.endPacket();
    LoRa.receive();
    qret = xQueueReceive(ackqueue, &enc_ack, 2000); // portMAX_DELAY); - in milliseconds
    if (qret == pdTRUE)
    {
      Serial.println("data received from the queue");
      Serial.println("Encrypted ACK packet");
      if (enc_sdp.pay.channel == enc_ack.pay.channel)
      {
        for (int i = 0; i < 22; i++) Serial.print(enc_ack.frame[i], HEX); Serial.println();
        decrypt(enc_ack.frame + 6, key, ack.frame + 6, 1);
        Serial.println("Decrypted ACK packet");
        for (int i = 0; i < 22; i++) Serial.print(ack.frame[i], HEX); Serial.println();
        Serial.printf("channel: %d, flag[0]: %2.2X, flag[1]: %2.2X\n",
                      enc_ack.pay.channel, enc_ack.pay.flag[0], enc_ack.pay.flag[1]);
        if (!enc_ack.pay.flag[1]) Serial.println("Attention: data not sent to TS");
        else Serial.println("Data sent to TS");
        if (enc_ack.pay.flag[0] & 0x80) {
          cycle = ack.pay.data[0];
          Serial.printf("cycle: %6.2f\n", cycle);
        }
        if (enc_ack.pay.flag[0] & 0x40) {
          delta = ack.pay.data[1];
          Serial.printf("delta: %6.2f\n", delta);
        }
        if (enc_ack.pay.flag[0] & 0x20) {
          kpack = ack.pay.data[2];
          Serial.printf("kpack: %6.2f\n", kpack);
        }
      }
      else Serial.println("not my ACK channel");
    }
  }
  esp_sleep_enable_timer_wakeup((uint32_t)cycle * mS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(cycle) + " milliseconds");
  esp_deep_sleep_start();
}
void loop() // nothing to do
{readSGP30();}
