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

/*
void CaptureSamples();
QueueHandle_t xQueueAudioWave;
#define QueueAudioWaveSize 32

namespace {
  bool g_is_audio_initialized = false;
  // An internal buffer able to fit 16x our sample size
  constexpr int kAudioCaptureBufferSize = BUFFER_SIZE * 16;
  int16_t g_audio_capture_buffer[kAudioCaptureBufferSize];
  // A buffer that holds our output
  int16_t g_audio_output_buffer[kMaxAudioSampleSize];
  // Mark as volatile so we can check in a while loop to see if
  // any samples have arrived yet.
  volatile int32_t g_latest_audio_timestamp = 0;
  // Our callback buffer for collecting a chunk of data
  volatile int16_t recording_buffer[BUFFER_SIZE];
}

void AudioRecordingTask(void *pvParameters) {
  static uint16_t audio_idx = 0;
  size_t bytes_read;
  int16_t i2s_data;
  int16_t sample;

  while (1) {

    if (audio_idx >= BUFFER_SIZE) {
      xQueueSend(xQueueAudioWave, &sample, 0);
      CaptureSamples();
      audio_idx = 0;
    }

    i2s_read(I2S_PORT, &i2s_data, 2, &bytes_read, portMAX_DELAY );

    if (bytes_read > 0) {
      //sample = (0xfff - (i2s_data & 0xfff)) - 0x800;
      sample = i2s_data;
      recording_buffer[audio_idx] = sample;
      audio_idx++;
      Serial.println(sample);
    }
  }
}

void CaptureSamples() {
  // This is how many bytes of new data we have each time this is called
  const int number_of_samples = BUFFER_SIZE;
  // Calculate what timestamp the last audio sample represents
  const int32_t time_in_ms =
    g_latest_audio_timestamp +
    (number_of_samples / (kAudioSampleFrequency / 1000));
  // Determine the index, in the history of all samples, of the last sample
  const int32_t start_sample_offset =
    g_latest_audio_timestamp * (kAudioSampleFrequency / 1000);
  // Determine the index of this sample in our ring buffer
  const int capture_index = start_sample_offset % kAudioCaptureBufferSize;
  // Read the data to the correct place in our buffer, note 2 bytes per buffer entry
  memcpy(g_audio_capture_buffer + capture_index, (void *)recording_buffer, BUFFER_SIZE * 2);
  // This is how we let the outside world know that new audio data has arrived.
  g_latest_audio_timestamp = time_in_ms;
}
*/

/*
void setup() {
  xQueueAudioWave = xQueueCreate(QueueAudioWaveSize, sizeof(int16_t));
  Serial.begin(115200);
  Serial.println("Setup I2S ...");

  delay(1000);
  i2s_install();
//  i2s_setpin();
  i2s_start(I2S_PORT);
  xTaskCreatePinnedToCore(AudioRecordingTask, "AudioRecordingTask", 2048, NULL, 10, NULL, 0);
  delay(500);
}

void loop() {
  
}

void i2s_install(){
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // default interrupt priority
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_DOUT,
    .data_in_num = I2S_SD
  };
  
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}
*/

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
