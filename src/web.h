#pragma once

bool start_wifi_client();
bool start_wifi_ap();
void web_init();
void web_loop();

bool web_frame_available();
const uint8_t *web_frame_ptr();
size_t web_frame_len();

bool web_inf_available();
const char *web_inf_ptr();
size_t web_inf_len();
