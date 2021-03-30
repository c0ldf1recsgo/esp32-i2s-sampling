#include "micro_model_settings.h"
#include <driver/i2s.h> 

#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PIN_DOUT I2S_PIN_NO_CHANGE

#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000) 
#define I2S_SAMPLE_BITS   (32)

#define BUFFER_SIZE       512

//void CaptureSamples();
//extern QueueHandle_t xQueueAudioWave;

/*
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
*/
/*
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
    }
  }
}
*/
/*
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

  //int peak = (max_audio - min_audio);
  //Serial.printf("peak-to-peak:  %6d\n", peak);
}
*/


void setup() {
  Serial.begin(115200);
  Serial.println("Setup I2S ...");

  delay(1000);
  i2s_install();
//  i2s_setpin();
//  i2s_start(I2S_PORT);
  delay(500);
}

void loop() {
//  static uint16_t audio_idx = 0;
  int16_t sample = 0;
  int16_t i2s_data;
  size_t bytes_read;

//  if (audio_idx >= BUFFER_SIZE) {
//      xQueueSend(xQueueAudioWave, &sample, 0);
//      CaptureSamples();
//      audio_idx = 0;
//  }

  i2s_read(I2S_PORT, &i2s_data, 2, &bytes_read, portMAX_DELAY );
//  int bytes = i2s_pop_sample(I2S_PORT, (char*)&sample, portMAX_DELAY);
//  if(bytes > 0){
//    Serial.println(sample);
//  }
  if (bytes_read > 0) {
      //sample = (0xfff - (i2s_data & 0xfff)) - 0x800;
      sample = i2s_data;
//      recording_buffer[audio_idx] = sample;
//      audio_idx++;
      Serial.println(sample);
  }
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

//void i2s_setpin(){
//  const i2s_pin_config_t pin_config = {
//    .bck_io_num = I2S_SCK,
//    .ws_io_num = I2S_WS,
//    .data_out_num = -1,
//    .data_in_num = I2S_SD
//  };
//
//  i2s_set_pin(I2S_PORT, &pin_config);
//}
