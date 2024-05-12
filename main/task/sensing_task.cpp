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
  xTaskCreatePinnedToCore(task_entry_point, "sensing_task", 8192 * 2, this, 2,
                          &handle, xCoreID);
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
    spi_r_bus->max_transfer_sz = 3;
    spi_r_bus->flags = SPICOMMON_BUSFLAG_MASTER;
    spi_r_bus->intr_flags = 0;

    spi_l_bus->mosi_io_num = SPI_L_MOSI;
    spi_l_bus->miso_io_num = SPI_L_MISO;
    spi_l_bus->sclk_io_num = SPI_L_CLK;
    spi_l_bus->quadwp_io_num = -1;
    spi_l_bus->quadhd_io_num = -1;
    spi_l_bus->max_transfer_sz = 3;
    spi_l_bus->flags = SPICOMMON_BUSFLAG_MASTER;
    spi_l_bus->intr_flags = 0;
  }
  {
    // gyro device config
    gyro_spi_devcfg->mode = 3;
    gyro_spi_devcfg->clock_speed_hz = 1 * 1000 * 1000;
    gyro_spi_devcfg->spics_io_num = SPI_R_GYRO_SSL;
    gyro_spi_devcfg->queue_size = 1;
  }
  { // ma11137ati device config
    ads7038_spi_devcfg->mode = 3;
    ads7038_spi_devcfg->clock_speed_hz = 1 * 1000 * 1000;
    ads7038_spi_devcfg->spics_io_num = SPI_L_ADC_SSL;
    ads7038_spi_devcfg->queue_size = 1;
  }
  {
    as5145p_left_spi_devcfg->mode = 3;
    as5145p_left_spi_devcfg->clock_speed_hz = 1 * 1000 * 1000;
    as5145p_left_spi_devcfg->spics_io_num = SPI_L_ENC_SSL;
    as5145p_left_spi_devcfg->queue_size = 1;
  }
  {
    as5145p_right_spi_devcfg->mode = 3;
    as5145p_right_spi_devcfg->clock_speed_hz = 1 * 1000 * 1000;
    as5145p_right_spi_devcfg->spics_io_num = SPI_R_ENC_SSL;
    as5145p_right_spi_devcfg->queue_size = 1;
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
  while (1) {
    // printf("%c[2J", ESC);   /* 画面消去 */
    // printf("%c[0;0H", ESC); /* 戦闘戻す*/

    if (adc_flag) {
      adc_if.write1byte(0x11, 0x07); // battery: AIN7
      sensing_result->battery.raw = adc_if.read2byte(0x10);
      sensing_result->battery.raw = adc_if.read2byte(0x10);
    }

    if (adc_flag) {

      // before
      adc_if.write1byte(0x11, 0x06);
      sensing_result->led_sen_before.left90.raw = adc_if.read2byte(0x10);
      sensing_result->led_sen_before.left90.raw = adc_if.read2byte(0x10);
      adc_if.write1byte(0x11, 0x05);
      sensing_result->led_sen_before.right90.raw = adc_if.read2byte(0x10);
      sensing_result->led_sen_before.right90.raw = adc_if.read2byte(0x10);
      adc_if.write1byte(0x11, 0x04);
      sensing_result->led_sen_before.right45_2.raw = adc_if.read2byte(0x10);
      sensing_result->led_sen_before.right45_2.raw = adc_if.read2byte(0x10);
      adc_if.write1byte(0x11, 0x03);
      sensing_result->led_sen_before.left45.raw = adc_if.read2byte(0x10);
      sensing_result->led_sen_before.left45.raw = adc_if.read2byte(0x10);
      adc_if.write1byte(0x11, 0x02);
      sensing_result->led_sen_before.right45.raw = adc_if.read2byte(0x10);
      sensing_result->led_sen_before.right45.raw = adc_if.read2byte(0x10);
      adc_if.write1byte(0x11, 0x01);
      sensing_result->led_sen_before.left45_2.raw = adc_if.read2byte(0x10);
      sensing_result->led_sen_before.left45_2.raw = adc_if.read2byte(0x10);

      // L90: AIN6
      adc_if.write1byte(0x11, 0x06);
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
    if (adc_flag) {
      // R90: AIN5
      adc_if.write1byte(0x11, 0x05);
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
    if (adc_flag) {
      // R45_2: AIN4
      adc_if.write1byte(0x11, 0x04);
      //(A2,A1,A0)=S4(0,1,1)
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
    if (adc_flag) {
      // L45: AIN3
      adc_if.write1byte(0x11, 0x03);
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
    if (adc_flag) {
      // R45: AIN2
      adc_if.write1byte(0x11, 0x02);
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
    if (adc_flag) {
      // L45_2: AIN1
      adc_if.write1byte(0x11, 0x01);
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

    // gyro_if.req_read2byte_itr(0x26);
    // se->gyro_list[4] = gyro_if.read_2byte_itr();

    // auto enc_r = enc_r_if.read2byte(0x3F, 0xFF, true); // & 0x3FFF;
    // auto enc_l = enc_l_if.read2byte(0x3F, 0xFF, false); // & 0x3FFF;

    // printf("gyro: %d\n", se->gyro_list[4]);
    // printf("battery: %d\n", sensing_result->battery.raw);
    // printf("L90: %d\n", sensing_result->led_sen_before.left90.raw);
    // printf("L45_2: %d\n", sensing_result->led_sen_before.left45_2.raw);
    // printf("L45: %d\n", sensing_result->led_sen_before.left45.raw);

    // printf("R45: %d\n", sensing_result->led_sen_before.right45.raw);
    // printf("R45_2: %d\n", sensing_result->led_sen_before.right45_2.raw);
    // printf("R90: %d\n", sensing_result->led_sen_before.right90.raw);

    // printf("enc_l: %ld\n", enc_l);
    // printf("enc_r: %ld\n", enc_r);

    if (enc_flag) {
      now_enc_r_time = esp_timer_get_time();
      int32_t enc_r = enc_r_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;
      now_enc_l_time = esp_timer_get_time();
      int32_t enc_l = enc_l_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;
      const auto enc_r_dt =
          ((float)(now_enc_r_time - last_enc_r_time)) / 1000000.0;
      se->encoder.right_old = se->encoder.right;
      se->encoder.right = enc_r;
      const auto enc_l_dt =
          ((float)(now_enc_l_time - last_enc_l_time)) / 1000000.0;
      se->encoder.left_old = se->encoder.left;
      se->encoder.left = enc_l;

      // spi_device_polling_transmit(spi_r, &t_r); // Transmit!
      // printf("t_r.rx_data[0]: %d, t_r.rx_data[1]: %d, t_r.rx_data[2]: %d, "
      //        "t_r.rx_data[3]: %d\n",
      //        t_r.rx_data[0], t_r.rx_data[1], t_r.rx_data[2], t_r.rx_data[3]);
      // spi_device_polling_transmit(spi_l, &t_l); // Transmit!
      // printf("t_l.rx_data[0]: %d, t_l.rx_data[1]: %d, t_l.rx_data[2]: %d, "
      //        "t_l.rx_data[3]: %d\n",
      //        t_l.rx_data[0], t_l.rx_data[1], t_l.rx_data[2], t_l.rx_data[3]);
    }

    se->battery.data = BATTERY_GAIN * 4 * sensing_result->battery.raw / 4096;
    se->led_sen.right90.raw = std::max(
        se->led_sen_after.right90.raw - se->led_sen_before.right90.raw, 0);
    se->led_sen.right45.raw = std::max(
        se->led_sen_after.right45.raw - se->led_sen_before.right45.raw, 0);
    se->led_sen.left45.raw = std::max(
        se->led_sen_after.left45.raw - se->led_sen_before.left45.raw, 0);
    se->led_sen.left90.raw = std::max(
        se->led_sen_after.left90.raw - se->led_sen_before.left90.raw, 0);
    se->led_sen.front.raw =
        (se->led_sen.left90.raw + se->led_sen.right90.raw) / 2;

    vTaskDelay(1.0 / portTICK_PERIOD_MS);
    continue;
    last_gyro_time = now_gyro_time;
    last_enc_r_time = now_enc_r_time;
    last_enc_l_time = now_enc_l_time;

    skip_sensing = !skip_sensing;

    bool r90 = true;
    bool l90 = true;
    bool r45 = true;
    bool l45 = true;

    start_before = start;
    start = esp_timer_get_time();
    se->calc_time = (int16_t)(start - start_before);
    se->sensing_timestamp = start;
    const float tmp_dt = ((float)se->calc_time) / 1000000.0;
    now_gyro_time = esp_timer_get_time();
    const float gyro_dt = ((float)(now_gyro_time - last_gyro_time)) / 1000000.0;
    gyro_if.req_read2byte_itr(0x26);
    start2 = esp_timer_get_time();

    if (skip_sensing) {
      adc2_get_raw(BATTERY, width, &sensing_result->battery.raw);
    }

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

    // LED_OFF ADC
    if (skip_sensing) {
      if (r90) {
        adc2_get_raw(SEN_R90, width,
                     &sensing_result->led_sen_before.right90.raw);
      } else {
        sensing_result->led_sen_before.right90.raw = 0;
      }
      adc2_get_raw(BATTERY, width, &sensing_result->battery.raw);
      if (l90) {
        adc2_get_raw(SEN_L90, width,
                     &sensing_result->led_sen_before.left90.raw);
      } else {
        sensing_result->led_sen_before.left90.raw = 0;
      }
    } else {

      if (r45) {
        adc2_get_raw(SEN_R45, width,
                     &sensing_result->led_sen_before.right45.raw);
      } else {
        sensing_result->led_sen_before.right45.raw = 0;
      }
      if (l45) {
        adc2_get_raw(SEN_L45, width,
                     &sensing_result->led_sen_before.left45.raw);
      } else {
        sensing_result->led_sen_before.left45.raw = 0;
      }
    }

    r90 = true;
    l90 = true;
    r45 = true;
    l45 = true;
    // LED_OFF ADC
    // 超信地旋回中は発光をサボる
    bool led_on = true;
    if (tgt_val->motion_type == MotionType::PIVOT ||
        tgt_val->motion_type == MotionType::SLALOM) {
      led_on = false;
      // if (tgt_val->ego_in.sla_param.counter >
      //     (tgt_val->ego_in.sla_param.limit_time_count / 2)) {
      //   led_on = true;
      // }
    };
    if (pt->mode_select) {
      led_on = false;
    }
    if (led_on) {
      if (pt->search_mode && pt->tgt_val->motion_type == MotionType::STRAIGHT) {
        // 加速中は正面は発光させない
        if (pt->tgt_val->ego_in.state == 0) {
          r90 = l90 = false;
        }
      }
      if (pt->search_mode && pt->tgt_val->nmr.sct == SensorCtrlType::NONE) {
        // 探索中、壁制御しないときはOFF
        r90 = l90 = false;
        r45 = l45 = false;
      }
      if (pt->tgt_val->nmr.sct == SensorCtrlType::Dia) {
        if (pt->tgt_val->ego_in.state == 0) {
          // 斜め壁制御加速中は横は発光させない
          r45 = l45 = false;
        }
      }
      if (pt->tgt_val->nmr.sct == SensorCtrlType::Straight) {
        r90 = l90 = true;
        r45 = l45 = true;
      }
      if (pt->tgt_val->motion_type == MotionType::READY) {
        // motion check用
        r90 = l90 = true;
        r45 = l45 = false;
      }
      if (pt->tgt_val->motion_type == MotionType::FRONT_CTRL) {
        // 前壁制御中は横は発光させない
        r90 = l90 = true;
        r45 = l45 = false;
      }
      if (pt->tgt_val->motion_type == MotionType::SENSING_DUMP) {
        r90 = l90 = true;
        r45 = l45 = true;
      }
      if (r90) { // R90
        set_gpio_state(LED_A0, false);
        set_gpio_state(LED_A1, false);
        set_gpio_state(LED_EN, true);
        lec_cnt = 0;
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        adc2_get_raw(SEN_R90, width, &se->led_sen_after.right90.raw);
      }
      if (l90) { // L90
        set_gpio_state(LED_A0, true);
        set_gpio_state(LED_A1, false);
        set_gpio_state(LED_EN, true);
        lec_cnt = 0;
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        adc2_get_raw(SEN_L90, width, &se->led_sen_after.left90.raw);
      }
      if (r45) { // R45
        set_gpio_state(LED_A0, false);
        set_gpio_state(LED_A1, true);
        set_gpio_state(LED_EN, true);
        lec_cnt = 0;
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        adc2_get_raw(SEN_R45, width, &se->led_sen_after.right45.raw);
      }
      if (l45) { // L45
        set_gpio_state(LED_A0, true);
        set_gpio_state(LED_A1, true);
        set_gpio_state(LED_EN, true);
        lec_cnt = 0;
        for (int i = 0; i < param->led_light_delay_cnt; i++) {
          lec_cnt++;
        }
        adc2_get_raw(SEN_L45, width, &se->led_sen_after.left45.raw);
      }
    }

    end2 = esp_timer_get_time();
    set_gpio_state(LED_EN, false);

    // se->battery.data = linearInterpolation(x, y, se->battery.raw);
    // se->battery.data = linearInterpolation(x, y, se->battery.raw);

    se->battery.data = BATTERY_GAIN * 4 * sensing_result->battery.raw / 4096;
    if (led_on) {
      se->led_sen.right90.raw = std::max(
          se->led_sen_after.right90.raw - se->led_sen_before.right90.raw, 0);
      se->led_sen.right45.raw = std::max(
          se->led_sen_after.right45.raw - se->led_sen_before.right45.raw, 0);
      se->led_sen.left45.raw = std::max(
          se->led_sen_after.left45.raw - se->led_sen_before.left45.raw, 0);
      se->led_sen.left90.raw = std::max(
          se->led_sen_after.left90.raw - se->led_sen_before.left90.raw, 0);
      se->led_sen.front.raw =
          (se->led_sen.left90.raw + se->led_sen.right90.raw) / 2;
    } else {
      se->led_sen.right90.raw = 0;
      se->led_sen.right45.raw = 0;
      se->led_sen.left45.raw = 0;
      se->led_sen.left90.raw = 0;
      se->led_sen.front.raw = 0;
    }

    // gyro_if.req_read2byte_itr(0x26);
    se->gyro_list[4] = gyro_if.read_2byte_itr();
    se->gyro.raw = se->gyro_list[4];
    se->gyro.data = (float)(se->gyro_list[4]);
    // int32_t enc_r = (enc_if.read2byte(0x00, 0x00, true) & 0xFFFF) >> 2;
    // now_enc_r_time = esp_timer_get_time();
    // int32_t enc_r = enc_r_if.read2byte(0x3F, 0xFF, true) & 0x3FFF;
    // const auto enc_r_dt =
    //     ((float)(now_enc_r_time - last_enc_r_time)) / 1000000.0;
    // se->encoder.right_old = se->encoder.right;
    // se->encoder.right = enc_r;

    // // int32_t enc_l = (enc_if.read2byte(0x00, 0x00, false) & 0xFFFF) >> 2;
    // now_enc_l_time = esp_timer_get_time();
    // int32_t enc_l = enc_l_if.read2byte(0x3F, 0xFF, false) & 0x3FFF;
    // const auto enc_l_dt =
    //     ((float)(now_enc_l_time - last_enc_l_time)) / 1000000.0;
    // se->encoder.left_old = se->encoder.left;
    // se->encoder.left = enc_l;

    // calc_vel(gyro_dt, enc_l_dt, enc_r_dt);

    // cout << enc_r << ", " << enc_l << endl;
    end = esp_timer_get_time();
    se->calc_time2 = (int16_t)(end - start);
    // printf("sen: %d, %d\n", (int16_t)(end - start), (int16_t)(end2 -
    // start2));
    vTaskDelay(1.0 / portTICK_PERIOD_MS);
  }
}

float SensingTask::calc_sensor(float data, float a, float b) {
  auto res = a / std::log(data) - b;
  if (res < 5 || res > 180)
    return 180;
  return res;
}

void SensingTask::calc_vel(float gyro_dt, float enc_r_dt, float enc_l_dt) {
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