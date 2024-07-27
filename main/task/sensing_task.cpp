#include "include/sensing_task.hpp"

SensingTask::SensingTask() {}

SensingTask::~SensingTask() {}

void SensingTask::timer_250us_callback(void *arg) {
  SensingTask *instance = static_cast<SensingTask *>(arg);
  instance->timer_250us_callback_main();
}
void SensingTask::timer_200us_callback(void *arg) {
  SensingTask *instance = static_cast<SensingTask *>(arg);
  instance->timer_200us_callback_main();
}
void SensingTask::timer_10us_callback(void *arg) {
  SensingTask *instance = static_cast<SensingTask *>(arg);
  instance->timer_10us_callback_main();
}

void SensingTask::timer_10us_callback_main() {}

// 壁切れ時に必要ないセンシング処理をやめて、本来ほしいデータにまわす

void SensingTask::timer_200us_callback_main() {}

void SensingTask::timer_250us_callback_main() {}

void SensingTask::create_task(const BaseType_t xCoreID) {
  // xTaskCreatePinnedToCore(task_entry_point, "sensing_task", 8192 * 2, this,
  // 2,
  //                         th, xCoreID);
  xTaskCreatePinnedToCore(task_entry_point, "sensing_task", 8192 * 2, this, 2,
                          NULL, xCoreID);
  // const esp_timer_create_args_t timer_200us_args = {
  //     .callback = &SensingTask::timer_200us_callback,
  //     .arg = this,
  //     .name = "timer_100us"};
  // esp_timer_create(&timer_200us_args, &timer_200us);

  // const esp_timer_create_args_t timer_10us_args = {
  //     .callback = &SensingTask::timer_10us_callback,
  //     .arg = this,
  //     .name = "timer_10us"};
  // esp_timer_create(&timer_10us_args, &timer_10us);
}
void SensingTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void SensingTask::task_entry_point(void *task_instance) {
  static_cast<SensingTask *>(task_instance)->task();
}

void SensingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_sensing_result) {
  sensing_result = _sensing_result;
}

void SensingTask::set_planning_task(std::shared_ptr<PlanningTask> &_pt) {
  pt = _pt;
}
void SensingTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}
void SensingTask::encoder_init(const pcnt_unit_t unit, const gpio_num_t pinA,
                               const gpio_num_t pinB) {
  pcnt_config_0.pulse_gpio_num = pinA;
  pcnt_config_0.ctrl_gpio_num = pinB;
  pcnt_config_0.unit = unit;

  pcnt_config_1.pulse_gpio_num = pinB;
  pcnt_config_1.ctrl_gpio_num = pinA;
  pcnt_config_1.unit = unit;

  pcnt_unit_config(&pcnt_config_0);
  pcnt_unit_config(&pcnt_config_1);

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);

  pcnt_counter_resume(unit);
}

void SensingTask::task() {
  // timer_init_grp0_timer0();
  spi_r_bus = std::make_shared<spi_bus_config_t>();
  spi_l_bus = std::make_shared<spi_bus_config_t>();

  gyro_spi_devcfg = std::make_shared<spi_device_interface_config_t>();
  as5145p_left_spi_devcfg = std::make_shared<spi_device_interface_config_t>();
  as5145p_right_spi_devcfg = std::make_shared<spi_device_interface_config_t>();
  ads7038_spi_devcfg = std::make_shared<spi_device_interface_config_t>();

  { // config spi bus
    spi_r_bus->mosi_io_num = SPI_R_MOSI;
    spi_r_bus->miso_io_num = SPI_R_MISO;
    spi_r_bus->sclk_io_num = SPI_R_CLK;
    spi_r_bus->quadwp_io_num = -1;
    spi_r_bus->quadhd_io_num = -1;
    spi_r_bus->max_transfer_sz = 4;
    spi_r_bus->flags = SPICOMMON_BUSFLAG_MASTER;
    spi_r_bus->intr_flags = 0;

    spi_l_bus->mosi_io_num = SPI_L_MOSI;
    spi_l_bus->miso_io_num = SPI_L_MISO;
    spi_l_bus->sclk_io_num = SPI_L_CLK;
    spi_l_bus->quadwp_io_num = -1;
    spi_l_bus->quadhd_io_num = -1;
    spi_l_bus->max_transfer_sz = 4;
    spi_l_bus->flags = SPICOMMON_BUSFLAG_MASTER;
    spi_l_bus->intr_flags = 0;
  }
  {
    // gyro device config
    gyro_spi_devcfg->mode = 3;
    gyro_spi_devcfg->clock_speed_hz = 10 * 1000 * 1000;
    gyro_spi_devcfg->spics_io_num = SPI_R_GYRO_SSL;
    gyro_spi_devcfg->queue_size = 7;
    gyro_spi_devcfg->pre_cb = nullptr;
    gyro_spi_devcfg->flags = SPI_DEVICE_NO_DUMMY;
  }
  { // ma11137ati device config
    ads7038_spi_devcfg->mode = 3;
    ads7038_spi_devcfg->clock_speed_hz = 10 * 1000 * 1000;
    ads7038_spi_devcfg->spics_io_num = SPI_L_ADC_SSL;
    ads7038_spi_devcfg->queue_size = 7;
    ads7038_spi_devcfg->pre_cb = nullptr;
    ads7038_spi_devcfg->flags = SPI_DEVICE_NO_DUMMY;
  }
  {
    as5145p_left_spi_devcfg->mode = 3;
    as5145p_left_spi_devcfg->clock_speed_hz = 10 * 1000 * 1000;
    as5145p_left_spi_devcfg->spics_io_num = SPI_L_ENC_SSL;
    as5145p_left_spi_devcfg->queue_size = 7;
    as5145p_left_spi_devcfg->pre_cb = nullptr;
    as5145p_left_spi_devcfg->flags = SPI_DEVICE_NO_DUMMY;
  }
  {
    as5145p_right_spi_devcfg->mode = 3;
    as5145p_right_spi_devcfg->clock_speed_hz = 10 * 1000 * 1000;
    as5145p_right_spi_devcfg->spics_io_num = SPI_R_ENC_SSL;
    as5145p_right_spi_devcfg->queue_size = 7;
    as5145p_right_spi_devcfg->pre_cb = nullptr;
    as5145p_right_spi_devcfg->flags = SPI_DEVICE_NO_DUMMY;
  }

  set_gpio_state(SPI_L_ENC_SSL, true);
  set_gpio_state(SPI_R_ENC_SSL, true);

  spi_device_handle_t spi_l;
  spi_device_handle_t spi_r;
  bool adc_flag = true;
  bool enc_flag = true;
  if (!GY_MODE) {
    if (adc_flag) {
      gyro_if.init(SPI2_HOST, spi_r_bus, gyro_spi_devcfg);
      gyro_if.setup();
      adc_if.init(SPI3_HOST, spi_l_bus, ads7038_spi_devcfg);
      adc_if.setup();
    }
    if (enc_flag) {
      enc_r_if.init(SPI2_HOST, spi_r_bus, as5145p_right_spi_devcfg);
      enc_l_if.init(SPI3_HOST, spi_l_bus, as5145p_left_spi_devcfg);
    }
  }
  ready = true;
  spi_transaction_t t_r;
  spi_transaction_t t_l;
  constexpr uint16_t READ_FLAG2 = 0b01000000;
  memset(&t_r, 0, sizeof(t_r)); // Zero out the transaction
  memset(&t_l, 0, sizeof(t_l)); // Zero out the transaction

  t_r.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t_r.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t_r.tx_data[0] = (0x3f | READ_FLAG2 | 0b10000000);
  t_r.tx_data[1] = (0xff);
  t_r.tx_data[2] = 0;

  t_l.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  t_l.length = 16; // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
  t_l.tx_data[0] = (0x3f | READ_FLAG2 | 0b10000000);
  t_l.tx_data[1] = (0xff);
  t_l.tx_data[2] = 0;

  const auto se = get_sensing_entity();

  // esp_timer_start_periodic(timer_200us, 200);
  // esp_timer_start_periodic(timer_250us, 250);

  int64_t start = 0;
  int64_t end = 0;
  int64_t start2 = 0;
  int64_t end2 = 0;
  int64_t start_before = 0;
  bool battery_check = true;
  bool skip_sensing = false;

  int64_t last_gyro_time = 0;
  int64_t now_gyro_time = 0;

  int64_t last_enc_r_time = 0;
  int64_t now_enc_r_time = 0;

  int64_t last_enc_l_time = 0;
  int64_t now_enc_l_time = 0;

  // adc2_config_channel_atten(SEN_R90, atten);
  // while (1) {
  //   auto start_adc = esp_timer_get_time();
  //   adc2_get_raw(SEN_R90, width,
  //   &sensing_result->led_sen_before.right90.raw); auto end_adc =
  //   esp_timer_get_time();

  //   printf("time: %lld\n", end_adc - start_adc);

  //   vTaskDelay(100);
  // }

  // while (1) {

  //   gyro_if.read_2byte(0x26);

  //   vTaskDelay(100);
  // }

  while (1) {
    last_gyro_time = now_gyro_time;
    last_enc_r_time = now_enc_r_time;
    last_enc_l_time = now_enc_l_time;

    skip_sensing = !skip_sensing;

    r90 = true;
    l90 = true;
    r45 = true;
    l45 = true;

    start_before = start;
    start = esp_timer_get_time();
    se->calc_time = (int16_t)(start - start_before);
    se->sensing_timestamp = start;
    const float tmp_dt = ((float)se->calc_time) / 1000000.0;

    start2 = esp_timer_get_time();

    auto start_battery = esp_timer_get_time();
    if (skip_sensing) {
      adc_if.write1byte_2(0x11, 0x07); // battery: AIN7
      sensing_result->battery.raw = adc_if.read2byte(0x10);
      sensing_result->battery.raw = adc_if.read2byte(0x10);
    }
    auto end_battery = esp_timer_get_time();
    se->calc_battery_time = (int16_t)(end_battery - start_battery);

    // r90 = l90 = r45 = l45 = false;
    bool led_on = true;
    if (tgt_val->motion_type == MotionType::PIVOT ||
        tgt_val->motion_type == MotionType::SLALOM) {
      led_on = false;
    }
    if (pt->mode_select) {
      led_on = false;
    }
    led_on = true;
    if (led_on) {
      change_led_mode();
    }
    auto start3 = esp_timer_get_time();
    // LED_OFF ADC
    if (r90) {
      auto tmp_start = esp_timer_get_time();
      adc_if.write1byte_2(0x11, 0x05);
      if (skip_sensing) {
        sensing_result->led_sen_before.right90.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_before.right90.raw = adc_if.read2byte(0x10);
      }
      if (led_on) {
        //(A2,A1,A0):S3(0,1,0)
        set_gpio_state(LED_A0, false);
        set_gpio_state(LED_A1, true);
        set_gpio_state(LED_A2, false);
        set_gpio_state(LED_EN, true);
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        sensing_result->led_sen_after.right90.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_after.right90.raw = adc_if.read2byte(0x10);
        set_gpio_state(LED_EN, false);
      }

      auto tmp_end = esp_timer_get_time();
      se->calc_led_sen_before_right90_time = (int16_t)(tmp_end - tmp_start);

    } else {
      sensing_result->led_sen_before.right90.raw = 0;
    }
    if (l90) {
      auto tmp_start = esp_timer_get_time();
      adc_if.write1byte_2(0x11, 0x06);
      if (!skip_sensing) {
        sensing_result->led_sen_before.left90.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_before.left90.raw = adc_if.read2byte(0x10);
      }
      if (led_on) { // L90
        //(A2,A1,A0):S5(1,0,0)
        set_gpio_state(LED_A0, false);
        set_gpio_state(LED_A1, false);
        set_gpio_state(LED_A2, true);
        set_gpio_state(LED_EN, true);
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        sensing_result->led_sen_after.left90.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_after.left90.raw = adc_if.read2byte(0x10);
        set_gpio_state(LED_EN, false);
      }
      auto tmp_end = esp_timer_get_time();
      se->calc_led_sen_before_left90_time = (int16_t)(tmp_end - tmp_start);
    } else {
      sensing_result->led_sen_before.left90.raw = 0;
    }
    if (r45) {
      auto tmp_start = esp_timer_get_time();
      adc_if.write1byte_2(0x11, 0x02);
      if (skip_sensing) {
        sensing_result->led_sen_before.right45.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_before.right45.raw = adc_if.read2byte(0x10);
      }
      if (led_on) { // R45
        //(A2,A1,A0): S7(1,1,0)
        set_gpio_state(LED_A0, false);
        set_gpio_state(LED_A1, true);
        set_gpio_state(LED_A2, true);
        set_gpio_state(LED_EN, true);
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        sensing_result->led_sen_after.right45.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_after.right45.raw = adc_if.read2byte(0x10);
        set_gpio_state(LED_EN, false);
      }
      if (!skip_sensing) {
        adc_if.write1byte_2(0x11, 0x04);
        sensing_result->led_sen_before.right45_2.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_before.right45_2.raw = adc_if.read2byte(0x10);
      }

      if (led_on) {
        set_gpio_state(LED_A0, true);
        set_gpio_state(LED_A1, true);
        set_gpio_state(LED_A2, false);
        set_gpio_state(LED_EN, true);
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        sensing_result->led_sen_after.right45_2.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_after.right45_2.raw = adc_if.read2byte(0x10);
        set_gpio_state(LED_EN, false);
      }
      auto tmp_end = esp_timer_get_time();
      se->calc_led_before_righ45_time = (int16_t)(tmp_end - tmp_start);
    } else {
      sensing_result->led_sen_before.right45.raw = 0;
    }
    if (l45) {
      adc_if.write1byte_2(0x11, 0x03);
      if (skip_sensing) {
        sensing_result->led_sen_before.left45.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_before.left45.raw = adc_if.read2byte(0x10);
      }
      if (led_on) {
        //(A2,A1,A0): S8(1,1,1)
        set_gpio_state(LED_A0, true);
        set_gpio_state(LED_A1, true);
        set_gpio_state(LED_A2, true);
        set_gpio_state(LED_EN, true);
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        sensing_result->led_sen_after.left45.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_after.left45.raw = adc_if.read2byte(0x10);
        set_gpio_state(LED_EN, false);
      }
      adc_if.write1byte_2(0x11, 0x01);
      if (!skip_sensing) {
        sensing_result->led_sen_before.left45_2.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_before.left45_2.raw = adc_if.read2byte(0x10);
      }
      if (led_on) {
        //(A2,A1,A0): S6(1,0,1)
        set_gpio_state(LED_A0, true);
        set_gpio_state(LED_A1, false);
        set_gpio_state(LED_A2, true);
        set_gpio_state(LED_EN, true);
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        sensing_result->led_sen_after.left45_2.raw = adc_if.read2byte(0x10);
        sensing_result->led_sen_after.left45_2.raw = adc_if.read2byte(0x10);
        set_gpio_state(LED_EN, false);
      }
    }
    auto end3 = esp_timer_get_time();
    // LED_OFF ADC

    auto end4 = esp_timer_get_time();
    end2 = esp_timer_get_time();
    set_gpio_state(LED_EN, false);

    se->battery.data = BATTERY_GAIN * 4 * sensing_result->battery.raw / 4096;
    if (led_on) {
      se->led_sen.right90.raw = std::max(
          se->led_sen_after.right90.raw - se->led_sen_before.right90.raw, 0);
      se->led_sen.right45.raw = std::max(
          se->led_sen_after.right45.raw - se->led_sen_before.right45.raw, 0);
      se->led_sen.right45_2.raw = std::max(
          se->led_sen_after.right45_2.raw - se->led_sen_before.right45_2.raw, 0);
      se->led_sen.left45.raw = std::max(
          se->led_sen_after.left45.raw - se->led_sen_before.left45.raw, 0);
      se->led_sen.left45_2.raw = std::max(
          se->led_sen_after.left45_2.raw - se->led_sen_before.left45_2.raw, 0);
      se->led_sen.left90.raw = std::max(
          se->led_sen_after.left90.raw - se->led_sen_before.left90.raw, 0);
      se->led_sen.front.raw =
          (se->led_sen.left90.raw + se->led_sen.right90.raw) / 2;
    } else {
      se->led_sen.right90.raw = 0;
      se->led_sen.right45.raw = 0;
      se->led_sen.right45_2.raw = 0;
      se->led_sen.left45_2.raw = 0;
      se->led_sen.left45.raw = 0;
      se->led_sen.left90.raw = 0;
      se->led_sen.front.raw = 0;
    }
    auto start5 = esp_timer_get_time();
    // gyro_if.req_read2byte_itr(0x26);
    now_gyro_time = esp_timer_get_time();
    const float gyro_dt = ((float)(now_gyro_time - last_gyro_time)) / 1000000.0;

    // gyro_if.req_read2byte_itr(0x26);
    se->gyro_list[4] = gyro_if.read_2byte(0x26);
    se->gyro.raw = se->gyro_list[4];
    se->gyro.data = (float)(se->gyro_list[4]);

    now_enc_r_time = esp_timer_get_time();
    int32_t enc_r = enc_r_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;
    const auto enc_r_dt =
        ((float)(now_enc_r_time - last_enc_r_time)) / 1000000.0;
    se->encoder.right_old = se->encoder.right;
    se->encoder.right = enc_r;

    now_enc_l_time = esp_timer_get_time();
    int32_t enc_l = enc_l_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;
    const auto enc_l_dt =
        ((float)(now_enc_l_time - last_enc_l_time)) / 1000000.0;
    se->encoder.left_old = se->encoder.left;
    se->encoder.left = enc_l;

    calc_vel(gyro_dt, enc_l_dt, enc_r_dt);
    auto end5 = esp_timer_get_time();
    // cout << enc_r << ", " << enc_l << endl;
    end = esp_timer_get_time();
    se->calc_time2 = (int16_t)(end - start);
    // printf("time: %lld\n", end - start);
    vTaskDelay(1.0 / portTICK_PERIOD_MS);
  }
}

float IRAM_ATTR SensingTask::calc_sensor(float data, float a, float b) {
  auto res = a / std::log(data) - b;
  if (res < 5 || res > 180)
    return 180;
  return res;
}

void IRAM_ATTR SensingTask::calc_vel(float gyro_dt, float enc_r_dt, float enc_l_dt) {
  // const float dt = param->dt;
  const float tire = pt->suction_en ? param->tire2 : param->tire;
  const auto enc_delta_l =
      sensing_result->encoder.left - sensing_result->encoder.left_old;
  float enc_ang_l = 0.f;
  if (ABS(enc_delta_l) < MIN(ABS(enc_delta_l - ENC_RESOLUTION),
                             ABS(enc_delta_l + ENC_RESOLUTION))) {
    enc_ang_l = 2 * m_PI * (float)enc_delta_l / (float)ENC_RESOLUTION;
  } else {
    if (ABS(enc_delta_l - ENC_RESOLUTION) < ABS(enc_delta_l + ENC_RESOLUTION)) {
      enc_ang_l = 2 * m_PI * (float)(enc_delta_l - ENC_RESOLUTION) /
                  (float)ENC_RESOLUTION;
    } else {
      enc_ang_l = 2 * m_PI * (float)(enc_delta_l + ENC_RESOLUTION) /
                  (float)ENC_RESOLUTION;
    }
  }

  const auto enc_delta_r =
      sensing_result->encoder.right - sensing_result->encoder.right_old;
  float enc_ang_r = 0.f;
  if (ABS(enc_delta_r) < MIN(ABS(enc_delta_r - ENC_RESOLUTION),
                             ABS(enc_delta_r + ENC_RESOLUTION))) {
    enc_ang_r = 2 * m_PI * (float)enc_delta_r / (float)ENC_RESOLUTION;
  } else {
    if (ABS(enc_delta_r - ENC_RESOLUTION) < ABS(enc_delta_r + ENC_RESOLUTION)) {
      enc_ang_r = 2 * m_PI * (float)(enc_delta_r - ENC_RESOLUTION) /
                  (float)ENC_RESOLUTION;
    } else {
      enc_ang_r = 2 * m_PI * (float)(enc_delta_r + ENC_RESOLUTION) /
                  (float)ENC_RESOLUTION;
    }
  }

  sensing_result->ego.v_l_old = sensing_result->ego.v_l;
  sensing_result->ego.v_r_old = sensing_result->ego.v_r;

  sensing_result->ego.v_l = tire * enc_ang_l / enc_l_dt / 2;
  sensing_result->ego.v_r = -tire * enc_ang_r / enc_r_dt / 2;

  sensing_result->ego.v_c =
      (sensing_result->ego.v_l + sensing_result->ego.v_r) / 2;

  sensing_result->ego.rpm.right =
      30.0 * sensing_result->ego.v_r / (m_PI * tire / 2);
  sensing_result->ego.rpm.left =
      30.0 * sensing_result->ego.v_l / (m_PI * tire / 2);

  if (tgt_val->motion_dir == MotionDirection::LEFT) {
    sensing_result->ego.w_raw =
        param->gyro_param.gyro_w_gain_left *
        (sensing_result->gyro.data - tgt_val->gyro_zero_p_offset);
  } else {
    sensing_result->ego.w_raw =
        param->gyro_param.gyro_w_gain_right *
        (sensing_result->gyro.data - tgt_val->gyro_zero_p_offset);
  }

  const auto dt = (enc_l_dt + enc_r_dt) / 2;

  tgt_val->ego_in.dist += sensing_result->ego.v_c * dt;
  tgt_val->global_pos.dist += sensing_result->ego.v_c * dt;
  tgt_val->ego_in.ang += sensing_result->ego.w_lp * gyro_dt;
  tgt_val->global_pos.ang += sensing_result->ego.w_lp * gyro_dt;
}

void IRAM_ATTR SensingTask::change_led_mode() {
  if (pt->search_mode && tgt_val->motion_type == MotionType::STRAIGHT) {
    // 加速中は正面は発光させない
    if (tgt_val->ego_in.state == 0) {
      r90 = l90 = false;
    }
  }
  if (pt->search_mode && tgt_val->nmr.sct == SensorCtrlType::NONE) {
    // 探索中、壁制御しないときはOFF
    r90 = l90 = false;
    r45 = l45 = false;
  }
  if (tgt_val->nmr.sct == SensorCtrlType::Dia) {
    if (tgt_val->ego_in.state == 0) {
      // 斜め壁制御加速中は横は発光させない
      r45 = l45 = false;
    }
  }
  if (tgt_val->nmr.sct == SensorCtrlType::Straight) {
    r90 = l90 = true;
    r45 = l45 = true;
  }
  if (tgt_val->motion_type == MotionType::READY) {
    // motion check用
    r90 = l90 = true;
    r45 = l45 = false;
  }
  if (tgt_val->motion_type == MotionType::FRONT_CTRL) {
    // 前壁制御中は横は発光させない
    r90 = l90 = true;
    r45 = l45 = false;
  }
  if (tgt_val->motion_type == MotionType::SENSING_DUMP) {
    r90 = l90 = true;
    r45 = l45 = true;
  }
}