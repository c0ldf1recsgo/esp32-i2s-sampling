//#include "micro_model_settings.h"
#include <Arduino.h>
#include <driver/i2s.h> 
#include <WiFi.h>
#include <HTTPClient.h>
#include "WiFiCredentials.h"
#include "I2SMEMSSampler.h"

#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PIN_DOUT I2S_PIN_NO_CHANGE

#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000) 
#define I2S_SAMPLE_BITS   (32)

#define BUFFER_SIZE       512

WiFiClient *wifiClientI2S = NULL;
HTTPClient *httpClientI2S = NULL;
I2SSampler *i2sSampler = NULL;


#define I2S_SERVER_URL "http://192.168.0.199:5003/i2s_samples"


// i2s config for reading from both channels of I2S
i2s_config_t i2sMemsConfigBothChannels = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s pins
i2s_pin_config_t i2sPins = {
    .bck_io_num = GPIO_NUM_32,
    .ws_io_num = GPIO_NUM_25,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = GPIO_NUM_33};


// send data to a remote address
void sendData(WiFiClient *wifiClient, HTTPClient *httpClient, const char *url, uint8_t *bytes, size_t count)
{
  // send them off to the server
  digitalWrite(2, HIGH);
  httpClient->begin(*wifiClient, url);
  httpClient->addHeader("content-type", "application/octet-stream");
  httpClient->POST(bytes, count);
  httpClient->end();
  digitalWrite(2, LOW);
}

// Task to write samples to our server
void i2sMemsWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
  while (true)
  {
    // wait for some samples to save
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      sendData(wifiClientI2S, httpClientI2S, I2S_SERVER_URL, (uint8_t *)sampler->getCapturedAudioBuffer(), sampler->getBufferSizeInBytes());
    }
  }
}

void setup()
{
  Serial.begin(115200);
  // launch WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Started up");
  // indicator LED
  pinMode(22, OUTPUT);

  wifiClientI2S = new WiFiClient();
  httpClientI2S = new HTTPClient();

  // Direct i2s input from INMP441 or the SPH0645
  i2sSampler = new I2SMEMSSampler(i2sPins, false);

  // set up the i2s sample writer task
  TaskHandle_t i2sMemsWriterTaskHandle;
  xTaskCreatePinnedToCore(i2sMemsWriterTask, "I2S Writer Task", 4096, i2sSampler, 1, &i2sMemsWriterTaskHandle, 1);

  // start sampling from i2s device
  i2sSampler->start(I2S_NUM_1, i2sMemsConfigBothChannels, 32768, i2sMemsWriterTaskHandle);
}

void loop()
{
  // nothing to do here - everything is taken care of by tasks
}
