#include <driver/i2s.h> 

#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_PIN_DOUT I2S_PIN_NO_CHANGE

#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000) 
#define I2S_SAMPLE_BITS   (32)

#define BUFFER_SIZE       512

void setup() {
  Serial.begin(115200);
  Serial.println("Setup I2S ...");

  delay(1000);
  i2s_install();
  i2s_start(I2S_PORT);
  delay(500);
}

void loop() {
//  static uint16_t audio_idx = 0;
  int16_t sample = 0;
  int16_t i2s_data;
  size_t bytes_read;

  i2s_read(I2S_PORT, &i2s_data, 2, &bytes_read, portMAX_DELAY );

  if (bytes_read > 0) {
      //sample = (0xfff - (i2s_data & 0xfff)) - 0x800;
      sample = i2s_data;
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

