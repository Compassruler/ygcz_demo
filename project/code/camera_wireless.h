#ifndef CAMERA_WIRELESS_H
#define CAMERA_WIRELESS_H

#include "zf_common_headfile.h"

void camera_wireless_init(void);
uint8 camera_wireless_has_frame(void);
void camera_wireless_send_frame(void);
uint32 camera_wireless_get_frame_count(void);

#endif