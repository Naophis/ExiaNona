#ifndef ADS7038_HPP
#define ADS7038_HPP

#include "defines.hpp"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <string.h>

union adc_config_t {
  struct {
    unsigned int _unused : 2;

    unsigned int _ECHO : 1;
    unsigned int SPM : 2;
    unsigned int NSCAN : 2;
    unsigned int NAVG : 2;
    unsigned int AVGON : 1;
    unsigned int REFSEL : 1;
    unsigned int CONFIG_SETUP : 5;
  };
  uint16_t data;
};

union adc_mode_t {
  struct {
    unsigned int _unused : 1;
    unsigned int SWCNV : 1;
    unsigned int CHAN_ID : 1;
    unsigned int PM : 2;
    unsigned int RESET : 2;
    unsigned int CHSEL : 4;
    unsigned int SCAN : 4;
    unsigned int CONFIG_SETUP : 1;
  };
  uint16_t data;
};

union custom_scan_t {
  struct {
    unsigned int _unused : 3;
    unsigned int CHSCAN : 8;
    unsigned int CUST_SCAN0 : 5;
  };
  uint16_t data;
};
union range_setup_t {
  struct {
    unsigned int _unused : 3;
    unsigned int CHSCAN : 8;
    unsigned int RANGE_SETUP : 5;
  };
  uint16_t data;
};

union unipolar_t {
  struct {
    unsigned int _unused : 2;
    unsigned int PDIFF_COM : 1;
    unsigned int UCH : 8;
    unsigned int UNI_SETUP : 5;
  };
  uint16_t data;
};

union bypolar_t {
  struct {
    unsigned int _unused : 2;
    unsigned int PDIFF_COM : 1;
    unsigned int UCH : 8;
    unsigned int BIP_SETUP : 5;
  };
  uint16_t data;
};

#define MAX11136_REG_ADC_MODE_CONTROL 0b00000
#define MAX11136_REG_ADC_CONFIG 0b10000
#define MAX11136_REG_UNIPOLAR 0b10001
#define MAX11136_REG_BIPOLAR 0b10010
#define MAX11136_REG_RANGE 0b10011
#define MAX11136_REG_CUSTOM_SCAN0 0b10100
#define MAX11136_REG_CUSTOM_SCAN1 0b10101
#define MAX11136_REG_SAMPLE_SET 0b10110
#define MAX11136_REG_LAST 0b10111

#define MAX11136_NUMBER_OF_CHANNELS 16
#define MAX11136_DATA_INDEX_LAST 15
typedef struct {
  uint16_t data_12bits[MAX11136_NUMBER_OF_CHANNELS];
  uint32_t data[MAX11136_DATA_INDEX_LAST];
  uint8_t status;
} MAX11136_context_t;

class ADS7038 {
public:
  ADS7038();
  virtual ~ADS7038();

  void init(spi_host_device_t spi, std::shared_ptr<spi_bus_config_t> &bus,
            std::shared_ptr<spi_device_interface_config_t> &devcfg);
  uint8_t write1byte(const uint8_t address, const uint8_t data);
  uint8_t write2byte(const uint16_t data);
  void write3byte(const uint8_t data1, const uint8_t data2,
                  const uint8_t data3);

  uint8_t read1byte(const uint8_t address);
  uint16_t read2byte(const uint16_t address);

  void req_read1byte_itr(const uint8_t address);
  uint8_t read_1byte_itr();

  void req_read2byte_itr(const uint8_t address);
  int16_t read_2byte_itr();
  signed short read_2byte_itr2(std::vector<int> &list);
  void setup();
  int read_gyro_z();
  int read_accel_x();
  int read_accel_y();
  void printBinary(uint16_t value);
  void set_gpio_state(gpio_num_t gpio_num, int state);

private:
  spi_device_handle_t spi;
  spi_transaction_t itr_t;
  spi_transaction_t *r_trans;
};

#endif