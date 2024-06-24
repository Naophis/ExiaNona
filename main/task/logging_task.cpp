#include "include/logging_task.hpp"

void LoggingTask::create_task(const BaseType_t xCoreID) {
  qh = xQueueCreate(4, sizeof(motion_tgt_val_t *));
  xTaskCreatePinnedToCore(task_entry_point, "logging_task", 8192, this, 1,
                          &handle, xCoreID);
}

void LoggingTask::task_entry_point(void *task_instance) {
  static_cast<LoggingTask *>(task_instance)->task();
}

void LoggingTask::set_sensing_entity(
    std::shared_ptr<sensing_result_entity_t> &_entity) {
  sensing_result = _entity;
}
void LoggingTask::set_input_param_entity(
    std::shared_ptr<input_param_t> &_param) {
  param = _param;
}

void LoggingTask::set_tgt_val(std::shared_ptr<motion_tgt_val_t> &_tgt_val) {
  tgt_val = _tgt_val;
}

void LoggingTask::set_error_entity(
    std::shared_ptr<pid_error_entity_t> &_error_entity) {
  error_entity = _error_entity;
}

void LoggingTask::start_slalom_log() {
  req_logging_active = true;
  idx_slalom_log = 0;
  // std::vector<std::unique_ptr<log_data_t2>>().swap(log_vec);
  log_vec.clear();
  log_vec.shrink_to_fit();
  sysidlog_vec.clear();
  // log_vec.resize(param->log_size);
  // for (int i = 0; i < param->log_size; i++) {
  //   log_vec[i] = std::make_shared<log_data_t2>();
  // }
  xQueueSendToFront(qh, &req_logging_active, 1);
}

void LoggingTask::stop_slalom_log() {
  active_slalom_log = false; //
  logging_active = false;
}

void LoggingTask::change_sysid_mode(float d_l, float d_r, int t) {
  log_mode = false; //
  duty_l = d_l;
  duty_r = d_r;
  time = t;
  sysidlog_vec.clear();
  sysidlog_vec.shrink_to_fit();
  sysidlog_vec.resize(time);
  for (int i = 0; i < time; i++) {
    sysidlog_vec[i] = std::make_shared<sysid_log>();
  }
}

void LoggingTask::exec_log() {}

void LoggingTask::task() {
  const TickType_t xDelay4 = 40.0 / portTICK_PERIOD_MS;
  const TickType_t xDelay1 = 1.0 / portTICK_PERIOD_MS;
  BaseType_t queue_recieved;
  logging_active = false;
  // 1,4MByteぐらいまで行ける
  // for (int i = 0; i < 26000; i++) {
  //   log_data_t2 *structPtr =
  //       (log_data_t2 *)heap_caps_malloc(sizeof(log_data_t2),
  //       MALLOC_CAP_SPIRAM);
  //   auto ld = std::shared_ptr<log_data_t2>(structPtr, heap_caps_free);
  //   log_vec.emplace_back(std::move(ld));
  //   printf("%d %p %p %p\n", i, &ld, structPtr, &log_vec.at(i));
  // }

  while (1) {

    if (!logging_active) {
      queue_recieved =
          xQueueReceive(qh, &receive_logging_active_req, portMAX_DELAY);
      logging_active = receive_logging_active_req;
    }
    if (idx_slalom_log > param->log_size) {
      logging_active = false;
    }
    if (log_mode) {
      if (logging_active) {
        if (idx_slalom_log <= param->log_size) {
          // auto ld = std::make_shared<log_data_t2>();
          set_data();
        }
        if (param->set_param) {
          vTaskDelay(param->logging_time);
        } else {
          vTaskDelay(xDelay4);
        }
      } else {
        vTaskDelay(xDelay4);
      }
    } else {
      if (logging_active) {
        if (active_slalom_log && idx_slalom_log <= time) {
          auto ld = std::make_shared<sysid_log>();

          ld->v_l = floatToHalf(sensing_result->ego.v_l);
          ld->v_c = floatToHalf(sensing_result->ego.v_c);
          ld->v_r = floatToHalf(sensing_result->ego.v_r);
          ld->w_lp = floatToHalf(sensing_result->ego.w_lp);
          ld->volt_l = floatToHalf(duty_l);
          ld->volt_r = floatToHalf(duty_r);

          sysidlog_vec.emplace_back(ld);
          idx_slalom_log++;
        }
      }
      vTaskDelay(xDelay1);
    }
  }
}
float LoggingTask::calc_sensor(float data, float a, float b, char motion_type) {
  if ((motion_type == static_cast<char>(MotionType::NONE) ||
       motion_type == static_cast<char>(MotionType::PIVOT))) {
    return 0;
  }
  if (data <= 1 || data >= 4005) {
    return 0;
  }
  auto res = a / std::log(data) - b;
  if (res < param->sensor_range_min || res > param->sensor_range_max) {
    return 0;
  }
  if (!isfinite(res)) {
    return 0;
  }
  return res;
}

void IRAM_ATTR LoggingTask::save(std::string file_name) {
  return; //
}

void IRAM_ATTR LoggingTask::save_sysid(std::string file_name) {
  printf("usefile: %s\n", slalom_log_file.c_str());
  f_slalom_log = fopen(slalom_log_file.c_str(), "wb");
  if (f_slalom_log == NULL)
    printf("slalom_file_load_failed\n");

  int i = 0;

  if (f_slalom_log != NULL) {
    fclose(f_slalom_log);
    printf("close\n");
  }
}

// void LoggingTask::dump_log(std::string file_name) {
//   mount();
//   const TickType_t xDelay2 = 100.0 / portTICK_PERIOD_MS;
//   FILE *f = fopen(file_name.c_str(), "rb");
//   if (f == NULL)
//     return;
//   char line_buf[LINE_BUF_SIZE];
//   printf("start___\n"); // csvファイル作成トリガー
//   vTaskDelay(xDelay2);
//   printf("index,ideal_v,v_c,v_c2,v_l,v_r,accl,accl_x,ideal_w,w_lp,alpha,"
//          "ideal_dist,"
//          "dist,"
//          "ideal_ang,ang,left90,left45,front,right45,right90,left90_d,left45_d,"
//          "front_d,right45_d,right90_d,left90_far_d,front_far_d,right90_far_d,"
//          "battery,duty_l,"
//          "duty_r,motion_state,duty_sen,dist_mod90,"
//          "sen_dist_l45,sen_dist_r45,timestamp\n");
//   int c = 0;
//   while (fgets(line_buf, sizeof(line_buf), f) != NULL) {
//     printf("%s\n", line_buf);
//     c++;
//     if (c == 50) {
//       c = 0;
//       vTaskDelay(1.0 / portTICK_PERIOD_MS);
//     }
//   }
//   printf("end___\n"); // csvファイル追記終了トリガー

//   fclose(f);
//   printf("memory: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
//   log_vec.clear();
//   umount();
//   // std::vector<std::shared_ptr<log_data_t2>>().swap(log_vec);
// }

void IRAM_ATTR LoggingTask::dump_log(std::string file_name) {
  const TickType_t xDelay2 = 100.0 / portTICK_PERIOD_MS;
  char line_buf[LINE_BUF_SIZE];
  printf("start___\n"); // csvファイル作成トリガー
  vTaskDelay(xDelay2);
  printf("index,ideal_v,v_c,v_c2,v_l,v_r,v_l_enc,v_r_enc,v_l_enc_sin,v_r_enc_"
         "sin,accl,accl_x,"
         "ideal_w,w_lp,alpha,ideal_dist,dist,dist_kf,"
         "ideal_ang,ang,ang_kf,left90,left45,front,right45,right90,"
         "left90_d,left45_d,"
         "front_d,right45_d,right90_d,left90_far_d,front_far_d,right90_far_d,"
         "battery,duty_l,"
         "duty_r,motion_state,duty_sen,dist_mod90,"
         "sen_dist_l45,sen_dist_r45,timestamp,sen_calc_time,sen_calc_time2,pln_"
         "calc_time,pln_"
         "calc_time2,pln_time_diff,m_pid_p,m_pid_i,m_pid_i2,m_pid_d,m_pid_p_v,"
         "m_pid_i_v,m_"
         "pid_i2_v,m_pid_d_v,g_pid_p,g_pid_i,g_pid_i2,g_pid_d,g_pid_p_v,g_pid_"
         "i_v,g_pid_i2_v,g_pid_d_v,s_pid_p,s_pid_i,s_pid_i2,s_pid_d,s_pid_p_v,"
         "s_pid_i_v,s_pid_i2_v,s_pid_d_v,ff_duty_front,ff_duty_roll,ff_duty_"
         "rpm_r,ff_duty_rpm_l\n");
  int c = 0;
  const char *f1 = format1.c_str();
  const char *f2 = format2.c_str();
  const char *f3 = format3.c_str();
  const char *f4 = format4.c_str();
  const char *f5 = format5.c_str();
  const char *f6 = format6.c_str();
  const char *f7 = format7.c_str();
  const float PI = 3.141592653589793238;
  int i = 0;

  for (const auto &ld : log_vec) {
    printf(f1,                                                  //
           i++,                                                 //
           halfToFloat(ld->img_v),                              //
           halfToFloat(ld->v_c),                                //
           halfToFloat(ld->v_c2),                               //
           halfToFloat(ld->v_l),                                //
           halfToFloat(ld->v_r),                                //
           (ld->v_l_enc),                                       //
           (ld->v_r_enc),                                       //
           (std::sin(2.0 * PI * ld->v_l_enc / ENC_RESOLUTION)), //
           (std::sin(2.0 * PI * ld->v_r_enc / ENC_RESOLUTION)), //
           halfToFloat(ld->accl),                               //
           halfToFloat(ld->accl_x));                            // 8
    printf(f2,                                                  //
           halfToFloat(ld->img_w),                              //
           halfToFloat(ld->w_lp),                               //
           halfToFloat(ld->alpha),                              //
           halfToFloat(ld->img_dist),                           //
           halfToFloat(ld->dist),                               //
           halfToFloat(ld->dist_kf),                            //
           halfToFloat(ld->img_ang),                            //
           halfToFloat(ld->ang),                                //
           halfToFloat(ld->ang_kf));                            // 7

    auto l90 = calc_sensor(halfToFloat(ld->left90_lp), param->sensor_gain.l90.a,
                           param->sensor_gain.l90.b, ld->motion_type);
    auto l45 = calc_sensor(halfToFloat(ld->left45_lp), param->sensor_gain.l45.a,
                           param->sensor_gain.l45.b, ld->motion_type);
    auto r45 =
        calc_sensor(halfToFloat(ld->right45_lp), param->sensor_gain.r45.a,
                    param->sensor_gain.r45.b, ld->motion_type);
    auto r90 =
        calc_sensor(halfToFloat(ld->right90_lp), param->sensor_gain.r90.a,
                    param->sensor_gain.r90.b, ld->motion_type);

    auto l90_far =
        calc_sensor(halfToFloat(ld->left90_lp), param->sensor_gain.l90_far.a,
                    param->sensor_gain.l90_far.b, ld->motion_type);
    auto r90_far =
        calc_sensor(halfToFloat(ld->right90_lp), param->sensor_gain.r90_far.a,
                    param->sensor_gain.r90_far.b, ld->motion_type);
    float front = 0;
    if (l90 > 0 && r90 > 0) {
      front = (l90 + r90) / 2;
    } else if (l90 == 0 && r90 > 0) {
      front = r90;
    } else if (l90 > 0 && r90 == 0) {
      front = l90;
    } else {
      front = 0;
    }

    float front_far = 0;

    if (l90_far > 0 && r90_far > 0) {
      front_far = (l90_far + r90_far) / 2;
    } else if (l90_far > 0 && r90_far == 0) {
      front_far = l90_far;
    } else if (l90_far == 0 && r90_far > 0) {
      front_far = r90_far;
    } else {
      front_far = 0;
    }

    auto dist = halfToFloat(ld->img_dist);
    float dist_mod = (int)(dist / param->dist_mod_num);
    float tmp_dist = dist - param->dist_mod_num * dist_mod;

    printf(
        f3,                                                               //
        halfToFloat(ld->left90_lp),                                       //
        halfToFloat(ld->left45_lp),                                       //
        ((halfToFloat(ld->left90_lp) + halfToFloat(ld->right90_lp)) / 2), //
        halfToFloat(ld->right45_lp),                                      //
        halfToFloat(ld->right90_lp),                                      //
        //  halfToFloat(ld->left45_2_lp),                                     //
        //  halfToFloat(ld->right45_2_lp),                                    //
        l90, l45, front, r45, r90,   //
        l90_far, front_far, r90_far, //
        halfToFloat(ld->battery_lp), //
        halfToFloat(ld->duty_l),     //
        halfToFloat(ld->duty_r),     //
        (ld->motion_type));          // 16

    printf(f4,                                //
           halfToFloat(ld->duty_sensor_ctrl), //
           tmp_dist,                          //
           halfToFloat(ld->sen_log_l45),      //
           halfToFloat(ld->sen_log_r45),      //
           ld->motion_timestamp,              //
           ld->sen_calc_time,                 //
           ld->sen_calc_time2,                //
           ld->pln_calc_time,                 //
           ld->pln_calc_time2,                //
           ld->pln_time_diff);                // 4

    printf(f5,                          //
           halfToFloat(ld->m_pid_p),    //
           halfToFloat(ld->m_pid_i),    //
           halfToFloat(ld->m_pid_i2),   //
           halfToFloat(ld->m_pid_d),    //
           halfToFloat(ld->m_pid_p_v),  //
           halfToFloat(ld->m_pid_i_v),  //
           halfToFloat(ld->m_pid_i2_v), //
           halfToFloat(ld->m_pid_d_v),  //
           halfToFloat(ld->g_pid_p),    //
           halfToFloat(ld->g_pid_i),    //
           halfToFloat(ld->g_pid_i2),   //
           halfToFloat(ld->g_pid_d));   //

    printf(f6,                          //
           halfToFloat(ld->g_pid_p_v),  //
           halfToFloat(ld->g_pid_i_v),  //
           halfToFloat(ld->g_pid_i2_v), //
           halfToFloat(ld->g_pid_d_v),  //
           halfToFloat(ld->s_pid_p),    //
           halfToFloat(ld->s_pid_i),    //
           halfToFloat(ld->s_pid_i2),   //
           halfToFloat(ld->s_pid_d),    //
           halfToFloat(ld->s_pid_p_v),  //
           halfToFloat(ld->s_pid_i_v),  //
           halfToFloat(ld->s_pid_i2_v), //
           halfToFloat(ld->s_pid_d_v)); //

    printf(f7,                             //
           halfToFloat(ld->ff_duty_front), //
           halfToFloat(ld->ff_duty_roll),  //
           halfToFloat(ld->ff_duty_rpm_r), //
           halfToFloat(ld->ff_duty_rpm_l)  //
    );

    if (i > 10 && ld->motion_timestamp == 0) {
      break;
    }
    c++;
    if (c == 50) {
      c = 0;
      vTaskDelay(1.0 / portTICK_PERIOD_MS);
    }
  }
  printf("end___\n"); // csvファイル追記終了トリガー

  printf("memory: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  log_vec.clear();
  // umount();
  // std::vector<std::shared_ptr<log_data_t2>>().swap(log_vec);
}

void IRAM_ATTR LoggingTask::dump_log_sysid(std::string file_name) {

  const TickType_t xDelay2 = 100.0 / portTICK_PERIOD_MS;
  FILE *f = fopen(file_name.c_str(), "rb");
  if (f == NULL)
    return;
  char line_buf[LINE_BUF_SIZE];
  printf("start___\n"); // csvファイル作成トリガー
  vTaskDelay(xDelay2);
  printf("index,v_l,v_c,v_r,w,volt_l,volt_r\n");
  while (fgets(line_buf, sizeof(line_buf), f) != NULL)
    printf("%s\n", line_buf);
  printf("end___\n"); // csvファイル追記終了トリガー

  fclose(f);
}

void IRAM_ATTR LoggingTask::set_data() {
  log_data_t2 *structPtr =
      (log_data_t2 *)heap_caps_malloc(sizeof(log_data_t2), MALLOC_CAP_SPIRAM);
  auto ld = std::shared_ptr<log_data_t2>(structPtr, heap_caps_free);

  ld->img_v = floatToHalf(tgt_val->ego_in.v);
  ld->v_l = floatToHalf(sensing_result->ego.v_l);
  // ld->v_l = floatToHalf(sensing_result->ego.v_kf);
  ld->v_c = floatToHalf(sensing_result->ego.v_c);
  ld->v_c2 = floatToHalf(sensing_result->ego.v_kf);
  ld->v_r = floatToHalf(sensing_result->ego.v_r);
  ld->v_r_enc = (sensing_result->encoder.right);
  ld->v_l_enc = (sensing_result->encoder.left);
  ld->accl = floatToHalf(tgt_val->ego_in.accl);
  ld->accl_x = floatToHalf(sensing_result->ego.w_kf);
  ld->dist_kf = floatToHalf(sensing_result->ego.dist_kf);

  // ld->accl_x = floatToHalf((float)tgt_val->ego_in.state);
  // ld->accl = floatToHalf(tgt_val->v_error);
  // ld->accl_x = floatToHalf(tgt_val->w_error);

  ld->img_w = floatToHalf(tgt_val->ego_in.w);
  ld->w_lp = floatToHalf(sensing_result->ego.w_lp);
  ld->alpha = floatToHalf(tgt_val->ego_in.alpha);

  ld->img_dist = floatToHalf(tgt_val->ego_in.img_dist);
  ld->dist = floatToHalf(tgt_val->ego_in.dist);

  ld->img_ang =
      floatToHalf((tgt_val->ego_in.img_ang + sensing_result->ego.duty.sen_ang) *
                  180 / m_PI);
  ld->ang = floatToHalf(tgt_val->ego_in.ang * 180 / m_PI);
  ld->ang_kf = floatToHalf(sensing_result->ego.ang_kf * 180 / m_PI);

  ld->left90_lp = floatToHalf(sensing_result->ego.left90_lp);
  ld->left45_lp = floatToHalf(sensing_result->ego.left45_lp);
  // ld->front_lp = floatToHalf(sensing_result->ego.front_lp);
  ld->right45_lp = floatToHalf(sensing_result->ego.right45_lp);
  ld->right90_lp = floatToHalf(sensing_result->ego.right90_lp);

  // ld->left45_2_lp = floatToHalf(sensing_result->ego.left45_2_lp);
  // ld->right45_2_lp = floatToHalf(sensing_result->ego.right45_2_lp);

  ld->battery_lp = floatToHalf(sensing_result->ego.batt_kf);
  ld->duty_l = floatToHalf(sensing_result->ego.duty.duty_l);
  ld->duty_r = floatToHalf(sensing_result->ego.duty.duty_r);

  ld->ff_duty_front = floatToHalf(sensing_result->ego.duty.ff_duty_front);
  ld->ff_duty_roll = floatToHalf(sensing_result->ego.duty.ff_duty_roll);
  ld->ff_duty_rpm_r = floatToHalf(sensing_result->ego.duty.ff_duty_rpm_r);
  ld->ff_duty_rpm_l = floatToHalf(sensing_result->ego.duty.ff_duty_rpm_l);

  ld->motion_type = static_cast<int>(tgt_val->motion_type);

  ld->duty_sensor_ctrl = floatToHalf(sensing_result->ego.duty.sen);

  ld->sen_log_l45 = floatToHalf(sensing_result->sen.l45.sensor_dist);
  ld->sen_log_r45 = floatToHalf(sensing_result->sen.r45.sensor_dist);

  ld->motion_timestamp = tgt_val->nmr.timstamp;
  ld->sen_calc_time = sensing_result->calc_time;
  ld->sen_calc_time2 = sensing_result->calc_time2;
  ld->pln_calc_time = tgt_val->calc_time;
  ld->pln_calc_time2 = tgt_val->calc_time2;
  ld->pln_time_diff = tgt_val->calc_time_diff;

  ld->m_pid_p = floatToHalf(error_entity->v_val.p);
  ld->m_pid_i = floatToHalf(error_entity->v_val.i);
  ld->m_pid_i2 = floatToHalf(error_entity->v_val.i2);
  ld->m_pid_d = floatToHalf(error_entity->v_val.d);
  ld->m_pid_p_v = floatToHalf(error_entity->v_val.p_val);
  ld->m_pid_i_v = floatToHalf(error_entity->v_val.i_val);
  ld->m_pid_i2_v = floatToHalf(error_entity->v_val.i2_val);
  ld->m_pid_d_v = floatToHalf(error_entity->v_val.d_val);

  ld->g_pid_p = floatToHalf(error_entity->w_val.p);
  ld->g_pid_i = floatToHalf(error_entity->w_val.i);
  ld->g_pid_i2 = floatToHalf(error_entity->w_val.i2);
  ld->g_pid_d = floatToHalf(error_entity->w_val.d);
  ld->g_pid_p_v = floatToHalf(error_entity->w_val.p_val);
  ld->g_pid_i_v = floatToHalf(error_entity->w_val.i_val);
  ld->g_pid_i2_v = floatToHalf(error_entity->w_val.i2_val);
  ld->g_pid_d_v = floatToHalf(error_entity->w_val.d_val);

  ld->s_pid_p = floatToHalf(error_entity->s_val.p);
  ld->s_pid_i = floatToHalf(error_entity->s_val.i);
  ld->s_pid_i2 = floatToHalf(error_entity->s_val.i2);
  ld->s_pid_d = floatToHalf(error_entity->s_val.d);
  ld->s_pid_p_v = floatToHalf(error_entity->s_val.p_val);
  ld->s_pid_i_v = floatToHalf(error_entity->s_val.i_val);
  ld->s_pid_i2_v = floatToHalf(error_entity->s_val.i2_val);
  ld->s_pid_d_v = floatToHalf(error_entity->s_val.d_val);

  if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > 10000) {
    log_vec.emplace_back(std::move(ld));
    idx_slalom_log++;
  }
}