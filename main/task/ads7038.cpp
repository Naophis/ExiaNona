#include "include/ads7038.hpp"

ADS7038::ADS7038() {}
ADS7038::~ADS7038() {}

// OPCODE COMMAND DESCRIPTION
// 0000 0000b No operation
// 0001 0000b Single register read
// 0000 1000b Single register write
// 0001 1000b Set bit
// 0010 0000b Clear bit

void ADS7038::init(spi_host_device_t spi_dev,
                   std::shared_ptr<spi_bus_config_t> &bus,
                   std::shared_ptr<spi_device_interface_config_t> &devcfg) {
  esp_err_t ret;
  ret = spi_bus_initialize(spi_dev, bus.get(), SPI_DMA_DISABLED);
  ESP_ERROR_CHECK(ret);
  ret = spi_bus_add_device(spi_dev, devcfg.get(), &spi);
  ESP_ERROR_CHECK(ret);
}
uint8_t ADS7038::write1byte(const uint8_t address, const uint8_t data) {
  esp_err_t ret;
  spi_transaction_t *rtrans;
  static spi_transaction_t t;
  static bool is_initialized = false;
  if (!is_initialized) {
    memset(&t, 0, sizeof(t)); // Zero out the transaction once
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 24; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
    is_initialized = true;
  }
  t.tx_data[0] = 0x08;
  t.tx_data[1] = (uint8_t)(0xff & address);
  t.tx_data[2] = (uint8_t)(0xff & data);
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
  return 0;
}
uint8_t IRAM_ATTR ADS7038::write1byte_2(const uint8_t address,
                                        const uint8_t data) {
  DRAM_ATTR static spi_transaction_t t;
  static bool is_initialized = false;
  if (!is_initialized) {
    memset(&t, 0, sizeof(t)); // Zero out the transaction once
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 24;
    is_initialized = true;
  }
  t.tx_data[0] = 0x08;
  t.tx_data[1] = (uint8_t)(0xff & address);
  t.tx_data[2] = (uint8_t)(0xff & data);
  spi_device_polling_transmit(spi, &t); // Transmit!
  return 0;
}

uint8_t ADS7038::write2byte(const uint16_t data) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = 24;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.tx_data[0] = 0x08;
  t.tx_data[1] = (uint8_t)((data & 0xff00) >> 8);
  t.tx_data[2] = (uint8_t)(data & 0x00ff);
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
  return 0;
}

void ADS7038::write3byte(const uint8_t data1, const uint8_t data2,
                         const uint8_t data3) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length = 24;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t.tx_data[0] = (uint8_t)(data1);
  t.tx_data[1] = (uint8_t)(data2);
  t.tx_data[2] = (uint8_t)(data3);
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
  return;
}

uint8_t ADS7038::read1byte(const uint8_t address) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.flags = SPI_TRANS_USE_RXDATA;
  t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  uint16_t tx_data = (address | READ_FLAG) << 8;
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  t.tx_buffer = &tx_data;
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
  uint8_t data =
      SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; // FF + Data
  return data;
}

uint16_t IRAM_ATTR ADS7038::read2byte(const uint16_t address) {
  esp_err_t ret;
  DRAM_ATTR static spi_transaction_t t;
  static bool is_initialized = false;

  if (!is_initialized) {
    memset(&t, 0, sizeof(t)); // Zero out the transaction once
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
    is_initialized = true;
  }

  t.tx_data[0] = (uint8_t)(address);
  t.tx_data[1] = 0;
  t.tx_data[2] = 0; // (uint8_t)(address);
  t.tx_data[3] = 0;

  // int64_t start_time = esp_timer_get_time();
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  // int64_t end_time = esp_timer_get_time();
  // printf("time: %lld usec\n", end_time - start_time);
  const auto data = (unsigned short)(((unsigned short)(t.rx_data[0]) << 8) |
                                     ((unsigned short)(t.rx_data[1])));

  // printf("%d, %d, %d, %d\n", t.rx_data[0], t.rx_data[1], t.rx_data[2],
  // t.rx_data[3]);

  // assert(ret == ESP_OK); // Should have had no issues.

  // const auto data =
  //     (unsigned short)(((unsigned short)(rtrans->rx_data[0]) << 8) |
  //                      ((unsigned short)(rtrans->rx_data[1])));
  return (data & 0xfff0) >> 4;
}

#define MAX11128_MODE_CNTL (uint16_t)0x0000   // 0b0          followed by 0s
#define MAX11128_MODE_MANUAL (uint16_t)0x0800 // 0b00001      followed by 0s
#define MAX11128_CONFIG (uint16_t)0x8000      // 0b1000       followed by 0s
#define SET_MAX11128_AVGON (uint16_t)0x0200   // 0b1000000000
#define CUSTOM_INT (uint16_t)7                // 7
#define SET_MAX11128_CHAN_ID (uint16_t)0x0004 // 0b100 preceded by 0s

// void ADS7038::setup() {

//   write1byte(0x82, 0x02); // 4kHz
//   write1byte(0x88, 0x02); // 4kHz
//   while (1) {

//     printf("%c[2J", ESC);   /* 画面消去 */
//     printf("%c[0;0H", ESC); /* 戦闘戻す*/
//     for (int i = 0; i < 16; i++) {
//       cout << i << ": ";
//       uint16_t ADC_MODE = 0x0804 | (i << 7);
//       read2byte(ADC_MODE);
//       vTaskDelay(1.0 / portTICK_PERIOD_MS);
//     }
//     vTaskDelay(100.0 / portTICK_PERIOD_MS);
//   }
//   // write1byte(0x1D, 0x00); // 4kHz
// }

void ADS7038::setup() {
  write1byte(0x10, 0x00); // manual sequance mode
  write1byte(0x04, 0x00);
}
void IRAM_ATTR ADS7038::set_gpio_state(gpio_num_t gpio_num, int state) {
  const int num = (int)gpio_num;
  if (num < 32) {
    if (state) {
      GPIO.out_w1ts = BIT(num);
    } else {
      GPIO.out_w1tc = BIT(num);
    }
  } else {
    if (state) {
      GPIO.out1_w1ts.val = BIT(num - 32);
    } else {
      GPIO.out1_w1tc.val = BIT(num - 32);
    }
  }
}

void ADS7038::printBinary(uint16_t value) {
  for (int i = 15; i >= 0; i--) {
    printf("%c", (value & (1 << i)) ? '1' : '0');
    if ((i % 4) == 0)
      printf(" ");
  }
  printf("\n");
}
int ADS7038::read_gyro_z() { return read2byte(0x47); }
int ADS7038::read_accel_x() { return read2byte(0x3B); }
int ADS7038::read_accel_y() { return read2byte(0x3D); }
void ADS7038::req_read1byte_itr(const uint8_t address) {
  memset(&itr_t, 0, sizeof(itr_t)); // Zero out the transaction
  itr_t.flags = SPI_TRANS_USE_RXDATA;
  itr_t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  uint16_t tx_data = (address | READ_FLAG) << 8;
  tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
  itr_t.tx_buffer = &tx_data;
  spi_device_queue_trans(spi, &itr_t, 1 / portTICK_RATE_MS); // Transmit!
}
uint8_t ADS7038::read_1byte_itr() {
  spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  return (uint8_t)(((unsigned short)(r_trans->rx_data[1] & 0xff)));
}

void ADS7038::req_read2byte_itr(const uint8_t address) {
  memset(&itr_t, 0, sizeof(itr_t)); // Zero out the transaction
  itr_t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  itr_t.length = 24; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  itr_t.tx_data[0] = (address | READ_FLAG);
  itr_t.tx_data[1] = 0;
  itr_t.tx_data[2] = 0;
  spi_device_queue_trans(spi, &itr_t, 1 / portTICK_RATE_MS); // Transmit!
}
int16_t ADS7038::read_2byte_itr() {
  spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}
signed short ADS7038::read_2byte_itr2(std::vector<int> &list) {
  spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  list[0] = r_trans->rx_data[0];
  list[1] = r_trans->rx_data[1];
  list[2] = r_trans->rx_data[2];
  list[3] = r_trans->rx_data[3];
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}