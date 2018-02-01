#include <mruby.h>
#include <mruby/array.h>
#include <mruby/class.h>
#include <mruby/string.h>
#include <mruby/value.h>
#include <mruby/variable.h>

#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define READ_BIT      I2C_MASTER_READ /*!< I2C master read */
#define ACK_CHECK_EN  0x1             /*!< I2C master will check ack from slave*/
#define ACK_VAL       0x0             /*!< I2C ack value */
#define NACK_VAL      0x1             /*!< I2C nack value */
static const char* TAG = "BME280";


static mrb_value
bme280_receive(mrb_state *mrb, mrb_value self) {
  mrb_int command, size, addr;
  mrb_value port, receive_data;
  esp_err_t err;

  mrb_get_args(mrb, "iii", &command, &size, &addr);
  port = mrb_iv_get(mrb, self, mrb_intern_lit(mrb, "@port"));
  if (size == 0) {
    return mrb_nil_value();
  }

  uint8_t *data_rd = mrb_malloc(mrb, size);
  memset(data_rd, 0, size);

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, command, ACK_CHECK_EN);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1 ) | READ_BIT, ACK_CHECK_EN);
  if (size > 1) {
      i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
  
  i2c_master_stop(cmd);
  err = i2c_master_cmd_begin(mrb_fixnum(port), cmd, 100 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (err != ESP_OK) {
    ESP_LOGI(TAG, "I2C receive data error: %d", err);
    mrb_free(mrb, data_rd);
    return mrb_nil_value();
  }

  receive_data = mrb_ary_new_capa(mrb, size);
  for (int8_t i=0; i<size; i++) {
    mrb_ary_push(mrb, receive_data, mrb_fixnum_value(data_rd[i]));
  }
  mrb_free(mrb, data_rd);

  return receive_data;  
}

// mrbgem init
void
mrb_mruby_esp32_i2c_bme280_gem_init(mrb_state* mrb)
{
  struct RClass *sensor = mrb_define_module(mrb, "SENSOR");
  struct RClass *bme280 = mrb_define_class_under(mrb, sensor, "BME280", mrb->object_class);

  mrb_define_method(mrb, bme280, "receive", bme280_receive, MRB_ARGS_REQ(3)); 
}

void
mrb_mruby_esp32_i2c_bme280_gem_final(mrb_state* mrb)
{
}
