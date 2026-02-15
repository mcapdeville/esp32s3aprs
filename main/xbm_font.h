#ifndef _XBM_FONT_H_
#define _XBM_FONT_H_

#include <lvgl.h>

lv_font_t * xbm_font_Init(uint8_t * data, size_t size, int width, int height, int32_t start, bool ovl);

#endif
