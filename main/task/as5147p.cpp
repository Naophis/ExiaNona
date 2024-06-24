#include "include/as5147p.hpp"

AS5147P::AS5147P() {}
AS5147P::~AS5147P() {}
void AS5147P::init(spi_host_device_t spi_dev,
                   std::shared_ptr<spi_bus_config_t> &bus,
                   std::shared_ptr<spi_device_interface_config_t> &devcfg) {
  spi_bus_add_device(spi_dev, devcfg.get(), &spi);
}
uint8_t AS5147P::write1byte(const uint8_t address, const uint8_t data) {
  return 0;
}
bool AS5147P::_spiCalcEvenParity(uint16_t value) {
  bool even = 0;
  uint8_t i;
  for (i = 0; i < 16; i++) {
    if (value & (0x0001 << i)) {
      even = !even;
    }
  }
  return even;
}
uint8_t AS5147P::read1byte(const uint8_t address) { return 0; }

int16_t AS5147P::read2byte(const uint16_t address) { return 0; }

uint32_t IRAM_ATTR AS5147P::read2byte(const uint8_t address1,
                                      const uint8_t address2, bool rorl) {
  esp_err_t ret;

  DRAM_ATTR static spi_transaction_t t;
  static bool is_initialized = false;

  if (!is_initialized) {
    memset(&t, 0, sizeof(t)); // Zero out the transaction once
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
    is_initialized = true;
  }

  auto res = _spiCalcEvenParity(address1 | READ_FLAG2);
  if (res) {
    t.tx_data[0] = (address1 | READ_FLAG2 | PARITY_FLAG);
  } else {
    t.tx_data[0] = (address1 | READ_FLAG2);
  }
  t.tx_data[1] = (address2);
  t.tx_data[2] = 0;
  t.tx_data[3] = 0;
  // int64_t start_time = esp_timer_get_time();
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  // int64_t end_time = esp_timer_get_time();
  // printf("time: %lld usec\n", end_time - start_time);
  return (int32_t)((uint16_t)(t.rx_data[0]) << 8) | (uint16_t)(t.rx_data[1]);
}

int16_t AS5147P::read2byte_2(const uint8_t address1, const uint8_t address2) {
  return 0;
}

void AS5147P::setup() {
  // uint8_t whoami = read1byte(0x0F); //
}
int AS5147P::read_gyro_z() { return read2byte(0x47); }
int AS5147P::read_accel_x() { return read2byte(0x3B); }
int AS5147P::read_accel_y() { return read2byte(0x3D); }
void AS5147P::req_read1byte_itr(const uint8_t address) {}
uint8_t AS5147P::read_1byte_itr() {
  // spi_device_get_trans_result(spi_r, &r_trans, 1 / portTICK_RATE_MS);
  return (uint8_t)(((unsigned short)(r_trans->rx_data[1] & 0xff)));
}

void AS5147P::req_read2byte_itr(const uint8_t address) {
  memset(&itr_t, 0, sizeof(itr_t)); // Zero out the transaction
  itr_t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  itr_t.length = 24; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  itr_t.tx_data[0] = (address | READ_FLAG2);
  itr_t.tx_data[1] = 0;
  itr_t.tx_data[2] = 0;
  // spi_device_queue_trans(spi, &itr_t, 1 / portTICK_RATE_MS); // Transmit!
}
int16_t AS5147P::read_2byte_itr() {
  // spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}
signed short AS5147P::read_2byte_itr2(std::vector<int> &list) {
  // spi_device_get_trans_result(spi, &r_trans, 1 / portTICK_RATE_MS);
  list[0] = r_trans->rx_data[0];
  list[1] = r_trans->rx_data[1];
  list[2] = r_trans->rx_data[2];
  list[3] = r_trans->rx_data[3];
  return (signed short)((((unsigned short)(r_trans->rx_data[1] & 0xff)) << 8) |
                        ((unsigned short)(r_trans->rx_data[2] & 0xff)));
}