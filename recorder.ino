#include <Arduino.h>
#include <driver/i2s.h>

#include <SPIFFS.h>

#define I2S_NUM           I2S_NUM_0           // 0 or 1
#define I2S_SAMPLE_RATE   16000

#define I2S_PIN_CLK       32
#define I2S_PIN_WS        25
#define I2S_PIN_DOUT      I2S_PIN_NO_CHANGE
#define I2S_PIN_DIN       33

#define BUFFER_SIZE       512

constexpr int kMaxAudioSampleSize = 512;
constexpr int kAudioSampleFrequency = 16000;

File file;
const char filename[] = "/recording.wav";
const int headerSize = 44;

int32_t time_travel = 0;
int C_INDEX = 0;

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

TaskHandle_t Task1;


void AudioRecordingTask(void *pvParameters) {
  static uint16_t audio_idx = 0;
  size_t bytes_read;
  int16_t i2s_data;
  int16_t sample;

  while (1) {

    if(g_latest_audio_timestamp>=20000){
      vTaskDelete(Task1);
    }
    
    if (audio_idx >= BUFFER_SIZE) {
      xQueueSend(xQueueAudioWave, &sample, 0);
      CaptureSamples();
      audio_idx = 0;
      Serial.println(g_latest_audio_timestamp);
    }

    i2s_read(I2S_NUM_0, &i2s_data, 2, &bytes_read, portMAX_DELAY );

    if (bytes_read > 0) {
//      sample = (0xfff - (i2s_data & 0xfff)) - 0x800;
      sample = i2s_data;
      recording_buffer[audio_idx] = i2s_data;
      audio_idx++;
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
  C_INDEX = capture_index;
  // Read the data to the correct place in our buffer, note 2 bytes per buffer entry
  memcpy(g_audio_capture_buffer + capture_index, (void *)recording_buffer, BUFFER_SIZE * 2);
//  Serial.println(capture_index);
  // This is how we let the outside world know that new audio data has arrived.
  g_latest_audio_timestamp = time_in_ms;
}


void SPIFFSInit(){
  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS initialisation failed!");
    while(1) yield();
  }

  SPIFFS.remove(filename);
  file = SPIFFS.open(filename, FILE_WRITE);
  if(!file){
    Serial.println("File is not available!");
  }

  byte header[headerSize];
  wavHeader(header, 8256);

  file.write(header, headerSize);
  listSPIFFS();
}



void wavHeader(byte* header, int wavSize){
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSize = wavSize + headerSize - 8;
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;
  header[21] = 0x00;
  header[22] = 0x01;
  header[23] = 0x00;
  header[24] = 0x80;
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x00;
  header[29] = 0x7D;
  header[30] = 0x00;
  header[31] = 0x00;
  header[32] = 0x02;
  header[33] = 0x00;
  header[34] = 0x10;
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);
  
}


void listSPIFFS(void) {
  Serial.println(F("\r\nListing SPIFFS files:"));
  static const char line[] PROGMEM =  "=================================================";

  Serial.println(FPSTR(line));
  Serial.println(F("  File name                              Size"));
  Serial.println(FPSTR(line));

  fs::File root = SPIFFS.open("/");
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  fs::File file = root.openNextFile();
  while (file) {

    if (file.isDirectory()) {
      Serial.print("DIR : ");
      String fileName = file.name();
      Serial.print(fileName);
    } else {
      String fileName = file.name();
      Serial.print("  " + fileName);
      // File path can be 31 characters maximum in SPIFFS
      int spaces = 33 - fileName.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      String fileSize = (String) file.size();
      spaces = 10 - fileSize.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      Serial.println(fileSize + " bytes");
    }

    file = root.openNextFile();
  }

  Serial.println(FPSTR(line));
  Serial.println();
  delay(1000);
}

// The name of this function is important for Arduino compatibility.

void setup() {
  
  delay(1000);
  Serial.begin(115200);
  SPIFFSInit();
  InitI2S();
  xQueueAudioWave = xQueueCreate(QueueAudioWaveSize, sizeof(int16_t));

  Serial.println("Start recording !!!");
  xTaskCreatePinnedToCore(AudioRecordingTask, "Task1", 2048, NULL, 10, &Task1, 0);
}


void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 2048;
    }
}

void loop() {
  if(g_latest_audio_timestamp>=20000){
    file.write((const byte*) g_audio_capture_buffer, kAudioCaptureBufferSize);
    file.close();
    listSPIFFS();
    Serial.println("Recorded !!!");
    vTaskDelete(NULL);
  }
  else if (C_INDEX == 7680){
    file.write((const byte*) g_audio_capture_buffer, kAudioCaptureBufferSize);
    Serial.println("Written 7680 !!!");
  }
}
