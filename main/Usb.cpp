// Copyright 2025 Pavel Suprunov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Created by jadjer on 05.12.2025.
//

#include "Usb.hpp"

#include <esp_log.h>
#include <tinyusb.h>
#include <tinyusb_cdc_acm.h>
#include <tinyusb_default_config.h>

namespace {

auto const TAG = "USB";

} // namespace

Usb::Usb() {
  ESP_LOGI(TAG, "USB initialization");
  tinyusb_config_t const usbConfig = TINYUSB_DEFAULT_CONFIG();

  ESP_ERROR_CHECK(tinyusb_driver_install(&usbConfig));

  tinyusb_config_cdcacm_t acm_cfg = {.cdc_port = TINYUSB_CDC_ACM_0,
                                     .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
                                     .callback_rx_wanted_char = NULL,
                                     .callback_line_state_changed = NULL,
                                     .callback_line_coding_changed = NULL};

  ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
  ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_LINE_STATE_CHANGED, &tinyusb_cdc_line_state_changed_callback));
}

auto Usb::send() -> void {
  typedef struct {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];     // Data buffer
    size_t buf_len;                                     // Number of bytes received
    uint8_t itf;                                        // Index of CDC device interface
  } app_message_t;

  app_message_t msg;

  tinyusb_cdcacm_write_queue(msg.itf, msg.buf, msg.buf_len);
  esp_err_t err = tinyusb_cdcacm_write_flush(msg.itf, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "CDC ACM write flush error: %s", esp_err_to_name(err));
  }
}