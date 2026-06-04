#include "settings.h"
#include <SPI.h>
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"


#if WIFI_ENABLED
  #include "wifi_server.h"
#endif

// This is a porting of PICOW2040_TUSS470.ino to the ESP32-S3, using ESP-IDF features for burst generation and ADC handling.

struct __attribute__((packed)) Frame {
  uint8_t  start = 0xAA;
  uint16_t  depth_index;            
  int16_t  temp_scaled;     
  uint16_t vDrv_scaled;     
  uint8_t  samples[NUM_SAMPLES];
  uint8_t  checksum;         
};

static Frame frame;  // Data frame to send over WebSocket

float temperature = 0.0f;
int vDrv = 0;

// -------------------- SPI BUFFERS --------------------
byte misoBuf[2];
byte inByteArr[2];

// -------------------- BURST GENERATION --------------------

// -------- Burst Generator State --------
static rmt_channel_handle_t burst_tx_channel = nullptr;
static rmt_encoder_handle_t burst_encoder = nullptr;
static rmt_symbol_word_t *burst_symbols = nullptr;
static size_t burst_symbol_count = 0;

void initBurstGenerator(uint pin, float frequency, uint cycles)
{
    // --- Time calculation ---
    float period_s = 1.0f / frequency;
    uint32_t half_period_us = (uint32_t)(period_s * 1e6f / 2.0f);

    burst_symbol_count = cycles;

    // --- Allocate symbols (Heap, persistent) ---
    burst_symbols = (rmt_symbol_word_t *)malloc(sizeof(rmt_symbol_word_t) * cycles);
    if (!burst_symbols) {
        Serial.println("ERROR: burst_symbols malloc failed");
        abort();
    }

    for (uint i = 0; i < cycles; i++) {
        burst_symbols[i].level0 = 1;
        burst_symbols[i].duration0 = half_period_us;
        burst_symbols[i].level1 = 0;
        burst_symbols[i].duration1 = half_period_us;
    }

    // --- RMT TX Channel ---
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = (gpio_num_t)pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,      // APB Clock
        .resolution_hz = 1 * 1000 * 1000,    // 1 MHz → 1 µs
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
        .flags = {
            .invert_out = false,
            .with_dma = false,
        }
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &burst_tx_channel));

    // --- Encoder ---
    rmt_copy_encoder_config_t encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &burst_encoder));

    // --- Channel aktivieren ---
    ESP_ERROR_CHECK(rmt_enable(burst_tx_channel));

    Serial.println("Burst generator initialized (RMT)");
}

void generateBurst()
{
    if (!burst_tx_channel || !burst_encoder || !burst_symbols) {
        return; // not yet initialized
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // exactly once
    };

    ESP_ERROR_CHECK(
        rmt_transmit(
            burst_tx_channel,
            burst_encoder,
            burst_symbols,
            burst_symbol_count * sizeof(rmt_symbol_word_t),
            &tx_config
        )
    );

    ESP_ERROR_CHECK(
        rmt_tx_wait_all_done(burst_tx_channel, portMAX_DELAY)
    );
}

// -------------------- ADC HANDLING  --------------------

static adc_continuous_handle_t adc_handle = nullptr;

static uint8_t adc_dma_buffer[1024];
static uint32_t adc_bytes_read = 0;

static volatile bool detectedDepth = false;


static uint8_t oversampleMax = 0;
static int oversampleCount = 0;


// ---- ADC measurement state ----
static volatile bool adcRunning = false;
static volatile bool frameReady = false;

static volatile int sampleIndex = 0;
static volatile int depthDetectSample = -1;

static int finalDepthSample = -1;


void initAdcContinuous()
{

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 1024,
        .conv_frame_size    = 256,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_continuous_config_t cfg = {
        .sample_freq_hz = (OVERSAMPLE_FACTOR == 1) ? 80000 : 
                          (OVERSAMPLE_FACTOR == 2) ? 40000 : 
                          (OVERSAMPLE_FACTOR == 4) ? 20000 : 80000,  // Adaptive: max 80 kHz total
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    static adc_digi_pattern_config_t pattern = {
        .atten     = ADC_ATTEN_SETTING,
        .channel   = ADC_CHANNEL_0,               // GPIO1 (D0 on XIAO ESP32-S3)
        .unit      = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12,
    };

    cfg.pattern_num = 1;
    cfg.adc_pattern = &pattern;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &cfg));

}

void startMeasurement()
{
    frameReady = false;
    adcRunning = true;
    sampleIndex = 0;
    depthDetectSample = -1;
    detectedDepth = false;
    
    oversampleMax = 0;      
    oversampleCount = 0;  


		ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    tuss4470Write(0x1B, 0x01);   // Start TOF

    generateBurst();             // defined burst
}

void processAdc()
{
    if (!adcRunning) return;

    esp_err_t ret = adc_continuous_read(
        adc_handle,
        adc_dma_buffer,
        sizeof(adc_dma_buffer),
        &adc_bytes_read,
        0   // non-blocking
    );

    if (ret != ESP_OK || adc_bytes_read == 0) {
        return;
    }

    for (size_t i = 0;
         i < adc_bytes_read && sampleIndex < NUM_SAMPLES;
         i += sizeof(adc_digi_output_data_t)) {

        adc_digi_output_data_t *p =
            (adc_digi_output_data_t *)&adc_dma_buffer[i];

        if (p->type2.unit != ADC_UNIT_1) continue;


      uint16_t raw = p->type2.data;
      uint8_t value = raw >> 4;

      // Oversampling: MAX-Aggregation
      if (value > oversampleMax) {
          oversampleMax = value;
      }
      oversampleCount++;

      if (oversampleCount >= OVERSAMPLE_FACTOR) {

          frame.samples[sampleIndex] = oversampleMax;
          sampleIndex++;

          oversampleMax = 0;
          oversampleCount = 0;

          if (sampleIndex >= NUM_SAMPLES) {
              ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
              tuss4470Write(0x1B, 0x00);

              adcRunning = false;
              frameReady = true;
              break;
          }
      }
		}

}

// -------------------- INTERRUPT HANDLER --------------------
void IRAM_ATTR handleInterrupt()
{

    if (!adcRunning) {
        return;
    }

    // Phase 1: Blind zone filter
    if (sampleIndex < BLINDZONE_SAMPLE_END) {
        return; // Event is in blind zone → ignore
    }

    if (!detectedDepth) {
        depthDetectSample = sampleIndex;
        detectedDepth = true;
    }

}


// -------------------- SPI UTILS --------------------
void spiTransfer(byte* mosi, byte sizeOfArr) {
  memset(misoBuf, 0x00, sizeof(misoBuf));

  digitalWrite(SPI_CS, LOW);
  for (int i = 0; i < sizeOfArr; i++) {
    misoBuf[i] = SPI.transfer(mosi[i]);
  }
  digitalWrite(SPI_CS, HIGH);
}

unsigned int BitShiftCombine(unsigned char x_high, unsigned char x_low) {
  return (x_high << 8) | x_low;
}

byte parity16(unsigned int val) {
  byte ones = 0;
  for (int i = 0; i < 16; i++) {
    if ((val >> i) & 1) {
      ones++;
    }
  }
  return (ones + 1) % 2;
}

byte tuss4470Parity(byte* spi16Val) {
  return parity16(BitShiftCombine(spi16Val[0], spi16Val[1]));
}

byte tuss4470Read(byte addr) {
  inByteArr[0] = 0x80 + ((addr & 0x3F) << 1);
  inByteArr[1] = 0x00;
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));
  return misoBuf[1];
}

void tuss4470Write(byte addr, byte data) {
  inByteArr[0] = (addr & 0x3F) << 1;
  inByteArr[1] = data;
  inByteArr[0] |= tuss4470Parity(inByteArr);
  spiTransfer(inByteArr, sizeof(inByteArr));
}

void resolveFinalDepth() {
    finalDepthSample = -1;

#if USE_DEPTH_OVERRIDE
    int overrideSample = computeDepthOverride();
    if (overrideSample >= 0) {
        finalDepthSample = overrideSample;
        return;
    }
#endif

    if (detectedDepth && depthDetectSample >= BLINDZONE_SAMPLE_END) {
        finalDepthSample = depthDetectSample;
        return;
    }

    // No valid bottom echo found
    finalDepthSample = -1;
}

int computeDepthOverride() {
#if USE_DEPTH_OVERRIDE
    int bestIndex = -1;
    uint8_t bestValue = 0;

    for (int i = BLINDZONE_SAMPLE_END; i < NUM_SAMPLES; i++) {
        uint8_t v = frame.samples[i];
        if (v > bestValue) {
            bestValue = v;
            bestIndex = i;
        }
    }
    return bestIndex;
#else
    return -1;
#endif
}

void sendData() {
  // Header fields
  frame.depth_index = depthDetectSample;

  frame.temp_scaled = (int16_t)(temperature * 100.0f);
  frame.vDrv_scaled = (uint16_t)(vDrv * 100);

  // Compute checksum (XOR of depth bytes, temp bytes, vDrv bytes, all samples)
  uint8_t cs = 0;
  // depth
  cs ^= (uint8_t)(frame.depth_index & 0xFF);
  cs ^= (uint8_t)(frame.depth_index >> 8);
  // temp
  cs ^= (uint8_t)(frame.temp_scaled & 0xFF);
  cs ^= (uint8_t)(frame.temp_scaled >> 8);
  // vDrv
  cs ^= (uint8_t)(frame.vDrv_scaled & 0xFF);
  cs ^= (uint8_t)(frame.vDrv_scaled >> 8);
  // samples
  for (int i = 0; i < NUM_SAMPLES; i++) {
    cs ^= frame.samples[i];
  }
  frame.checksum = cs;

  // Total length (packed, known)
  const size_t len = 1 + 2 + 2 + 2 + NUM_SAMPLES + 1;

  Serial.write(reinterpret_cast<uint8_t*>(&frame), len);

  #if WIFI_ENABLED && ENABLE_UDP_ECHO
    udpBroadcastBIN(reinterpret_cast<uint8_t*>(&frame), len, UDP_ECHO_PORT);
  #endif
}


void sendNmeaDBT() {
  // Calculate depth in meters
  float time_of_flight = depthDetectSample * 13.2e-6f;
  float depth_m = (time_of_flight * 1450.0f) / 2.0f;

  float depth_ft = depth_m * 3.28084f;
  float depth_fa = depth_m / 1.8288f;

  // Convert floats to strings
  char str_ft[10], str_m[10], str_fa[10];
  dtostrf(depth_ft, 4, 1, str_ft);
  dtostrf(depth_m, 4, 1, str_m);
  dtostrf(depth_fa, 4, 1, str_fa);

  // Trim leading spaces (manually)
  char* ptr_ft = str_ft;
  char* ptr_m = str_m;
  char* ptr_fa = str_fa;
  while (*ptr_ft == ' ') ptr_ft++;
  while (*ptr_m == ' ') ptr_m++;
  while (*ptr_fa == ' ') ptr_fa++;

  // Build the NMEA DBT sentence
  char dbt_sentence[80];
  snprintf(dbt_sentence, sizeof(dbt_sentence),
           "$SDDBT,%s,f,%s,M,%s,F", ptr_ft, ptr_m, ptr_fa);

  // Calculate checksum (XOR of chars between $ and *)
  uint8_t dbt_checksum = 0;
  for (int i = 1; dbt_sentence[i] != '\0'; i++) {
    dbt_checksum ^= dbt_sentence[i];
  }

  // Final output with checksum
  char fullDBTSentence[90];
  snprintf(fullDBTSentence, sizeof(fullDBTSentence), "%s*%02X\r\n", dbt_sentence, dbt_checksum);

  Serial2.print(fullDBTSentence);
  #if WIFI_ENABLED && ENABLE_UDP_NMEA
    udpBroadcastNMEA(fullDBTSentence, strlen(fullDBTSentence), UDP_NMEA_PORT);
  #endif

  // Build the NMEA DPT sentence
  char str_offset[10];
  dtostrf(DEPTH_OFFSET, 4, 1, str_offset);
  char* ptr_offset = str_offset;
  while (*ptr_offset == ' ') ptr_offset++;

  // We are (possibly optimistically) reporting max depth of 100m
  char dpt_sentence[80];
  snprintf(dpt_sentence, sizeof(dpt_sentence),
           "$SDDPT,%s,%s,100", ptr_m, ptr_offset);

  // Calculate checksum (XOR of chars between $ and *)
  uint8_t dpt_checksum = 0;
  for (int i = 1; dpt_sentence[i] != '\0'; i++) {
    dpt_checksum ^= dpt_sentence[i];
  }

  // Final output with checksum
  char fullDPTSentence[90];
  snprintf(fullDPTSentence, sizeof(fullDPTSentence), "%s*%02X\r\n", dpt_sentence, dpt_checksum);

  Serial2.print(fullDPTSentence);
  #if WIFI_ENABLED && ENABLE_UDP_NMEA
    udpBroadcastNMEA(fullDPTSentence, strlen(fullDPTSentence), UDP_NMEA_PORT);
  #endif
}

void setup() {
  Serial.begin(250000);
  Serial2.begin(NMEA_BAUD_RATE, SERIAL_8N1, -1, NMEA_TX_PIN);
  delay(100);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  pinMode(IO1, OUTPUT);
  digitalWrite(IO1, HIGH);
  pinMode(O4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(O4), handleInterrupt, RISING);

  // --- Initialize TUSS4470 ---
  tuss4470Write(0x10, FILTER_FREQUENCY_REGISTER);             // BPF
  tuss4470Write(0x16, 0x0F);             // Enable VDRV
  tuss4470Write(0x1A, 0x0F);             // 16 pulses
  tuss4470Write(0x17, THRESHOLD_VALUE);  // Threshold detect OUT4
  
  // --- Setup RMT for pulse burst ---
  initBurstGenerator(IO2, DRIVE_FREQUENCY, 16);

  // --- ADC init ---
  initAdcContinuous();

  // --- WiFi init ---
  #if WIFI_ENABLED
    wifiSetup(WIFI_SSID, WIFI_PASS);
  #endif
}

// -------------------- LOOP --------------------
void loop()
{
    if (!adcRunning && !frameReady) {
        startMeasurement();
    }

    processAdc();

    if (frameReady) {
        
    		resolveFinalDepth();

    		depthDetectSample = finalDepthSample;  // ← central handoff

    		sendNmeaDBT();
    		sendData();

    		frameReady = false;

    }

    delay(1);  // RTOS-friendly
}
