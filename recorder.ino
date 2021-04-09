#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_NUM           I2S_NUM_0           // 0 or 1
#define I2S_SAMPLE_RATE   16000

#define I2S_PIN_CLK       32
#define I2S_PIN_WS        25
#define I2S_PIN_DOUT      I2S_PIN_NO_CHANGE
#define I2S_PIN_DIN       33

#define BUFFER_SIZE       512

constexpr int kMaxAudioSampleSize = 512;
constexpr int kAudioSampleFrequency = 16000;


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
}  // namespace

void InitI2S()
{
  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = I2S_SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 4,
    .dma_buf_len          = 256,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num           = I2S_PIN_CLK,
    .ws_io_num            = I2S_PIN_WS,
    .data_out_num         = I2S_PIN_DOUT,
    .data_in_num          = I2S_PIN_DIN,
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_set_clk(I2S_NUM, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
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

    i2s_read(I2S_NUM_0, &i2s_data, 2, &bytes_read, portMAX_DELAY );

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
  const int32_t time_in_ms = g_latest_audio_timestamp + (number_of_samples / (kAudioSampleFrequency / 1000));
  
  // Determine the index, in the history of all samples, of the last sample
  const int32_t start_sample_offset = g_latest_audio_timestamp * (kAudioSampleFrequency / 1000);
  
  // Determine the index of this sample in our ring buffer
  const int capture_index = start_sample_offset % kAudioCaptureBufferSize;
  
  // Read the data to the correct place in our buffer, note 2 bytes per buffer entry
  memcpy(g_audio_capture_buffer + capture_index, (void *)recording_buffer, BUFFER_SIZE * 2);
  
  // This is how we let the outside world know that new audio data has arrived.
  g_latest_audio_timestamp = time_in_ms;
}



// The name of this function is important for Arduino compatibility.
void setup() {
  
  delay(1000);
  Serial.begin(115200);
  InitI2S();
  xQueueAudioWave = xQueueCreate(QueueAudioWaveSize, sizeof(int16_t));
  
  xTaskCreatePinnedToCore(AudioRecordingTask, "AudioRecordingTask", 2048, NULL, 10, NULL, 0);
}


void loop() {
}
