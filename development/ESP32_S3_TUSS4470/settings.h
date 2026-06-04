#pragma once

#include <Arduino.h>

// ---------------------- DRIVE FREQUENCY SETTINGS ----------------------
// Sets the output frequency of the ultrasonic transducer
#define DRIVE_FREQUENCY 200000

// ---------------------- BANDPASS FILTER SETTINGS ----------------------
// Sets the digital band-pass filter frequency on the TUSS4470 driver chip
// This should roughly match the transducer drive frequency
// For additional register values, see TUSS4470 datasheet, Table 7.1 (pages 17–18)
// #define FILTER_FREQUENCY_REGISTER 0x00 // 40 kHz
// #define FILTER_FREQUENCY_REGISTER 0x09 // 68 kHz
// #define FILTER_FREQUENCY_REGISTER 0x10 // 100 kHz
// #define FILTER_FREQUENCY_REGISTER 0x18 // 151 kHz
#define FILTER_FREQUENCY_REGISTER 0x1E // 200 kHz

// Number of ADC samples to take per measurement cycle
// This value must match the number of samples expected by the Python visualization tool
#define NUM_SAMPLES 10000

// Number of initial samples to ignore after sending the transducer pulse
// These ignored samples represent the "blind zone" where the transducer is still ringing
#define BLINDZONE_SAMPLE_END 150

// Threshold level for detecting the bottom echo
// The first echo stronger than this value (after the blind zone) is considered the bottom
#define THRESHOLD_VALUE 0x19

  
// ---------------------- DEPTH OVERRIDE ----------------------
// If enabled, software will scan the captured analogValues[] after each
// acquisition and choose the max sample after the blind zone to be 
// the bottom echo, instead of the first sample above the threshold.
#define USE_DEPTH_OVERRIDE 1


// ---------------------- OVERSAMPLING CONFIGURATION ----------------------
// Oversampling configuration
#define OVERSAMPLE_FACTOR 1 // 1 = no oversampling
//#define OVERSAMPLE_FACTOR 2	// 2× oversampling
//#define OVERSAMPLE_FACTOR 4 // 4× oversampling (MAX)

#if (OVERSAMPLE_FACTOR != 1) && (OVERSAMPLE_FACTOR != 2) && (OVERSAMPLE_FACTOR != 4)
#error "OVERSAMPLE_FACTOR must be 1, 2 or 4"
#endif


// ---------------------- PIN MAPPING ----------------------
// XIAO ESP32-S3 pin mapping
// SPI SCK	D8	GPIO7
// SPI MISO	D9	GPIO8
// SPI MOSI	D10	GPIO9
#define SPI_CS   3    	// D2	Chip select for TUSS4470
#define IO1      2    	// D1 Enable pin or control (set HIGH)
#define IO2      44    	// D7 Burst output pin (transducer drive)
#define O4       4    	// D3 TUSS4470 OUT4 threshold detect input (Interrupt)
//#define analogIn 1    	// A0 / ADC1_CH0
#define NMEA_TX_PIN 43  // D6
//#define CAN_TX_PIN 5 	// D4 Reserved for NMEA2000 extension
//#define CAN_RX_PIN 6	// D5 Reserved for NMEA2000 extension


// ---------------------- ADC SETTINGS ----------------------
// ADC input attenuation setting
// #define ADC_ATTEN_SETTING ADC_ATTEN_DB_11  // 0-3.9V range
#define ADC_ATTEN_SETTING ADC_ATTEN_DB_6    // 0-2.2V range (matches TUSS4470 Vref of 2.048V, see Register 0x11)


// ---------------------- NMEA SETTINGS ----------------------
// Fast or slow baud rate for NMEA output on SoftwareSerial (pin 4)
#define NMEA_BAUD_RATE 4800
// #define NMEA_BAUD_RATE 38400

// Depth offset in meters to add to NMEA reported depths (can be negative)
#define DEPTH_OFFSET 0.0f

// ---------------------- WIFI SETTINGS ----------------------
//#define WIFI_ENABLED 1

#if WIFI_ENABLED
  static const char WIFI_SSID[] = "Your SSID";
  static const char WIFI_PASS[] = "Your Password";

  // ---------------------- UDP BROADCAST SETTINGS ----------------------
  // Enable/disable UDP broadcast of the binary frame (same payload as Serial / WebSocket)
  // This cannot be sent to all, so a specific broadcast IP must be set
  #define ENABLE_UDP_ECHO 1
  #define UDP_ECHO_PORT 5005
  static const IPAddress UDP_ECHO_IP(10, 17, 20, 117);

  #define ENABLE_UDP_NMEA 1
  #define UDP_NMEA_PORT 5004
#endif