
#include "xbm_font.h"

#include <lvgl.h>
#include <esp_log.h>

#define TAG "xbm_font"

struct xbm_font_S {
	lv_font_t font;
	lv_font_fmt_txt_dsc_t font_desc;
	lv_font_fmt_txt_cmap_t cmaps;
	lv_font_fmt_txt_glyph_dsc_t glyph_dsc[];
};

static bool xbm_font_get_dsc(const lv_font_t * font, lv_font_glyph_dsc_t * dsc_out, uint32_t unicode_letter,
                                   uint32_t unicode_letter_next) {
	bool ret;

	if (unicode_letter >= 0xE000 && unicode_letter < 0xE180) {
		__asm__("nop");
	}

	ret = lv_font_get_glyph_dsc_fmt_txt(font, dsc_out, unicode_letter, unicode_letter_next);

	if (ret == true) {
		__asm__("nop");
	}

	return ret;
}

static const uint8_t * xbm_font_get_bitmap(const lv_font_t * font, uint32_t unicode_letter) {
	return lv_font_get_bitmap_fmt_txt(font, unicode_letter);
}

lv_font_t * xbm_font_Init(uint8_t * data, size_t size, int width, int height, int32_t start, bool ovl) {
	struct xbm_font_S *font;
	int32_t len = size/ (((width+7)>>3)*height); 

	if (!(font = malloc(sizeof(struct xbm_font_S) + sizeof(lv_font_fmt_txt_glyph_dsc_t) * (1+ len)))) {
		ESP_LOGE(TAG, "Error allocation lv_font_t");
		return NULL;
	}
	bzero(font, sizeof(struct xbm_font_S) + sizeof(lv_font_fmt_txt_glyph_dsc_t) * (1+len));

	font->font.get_glyph_dsc = xbm_font_get_dsc ;
	font->font.get_glyph_bitmap =xbm_font_get_bitmap;
	font->font.line_height = height;
	font->font.dsc = &font->font_desc;

	font->font_desc.glyph_bitmap = data;
	font->font_desc.bitmap_format = LV_FONT_FMT_TXT_PLAIN;
	font->font_desc.cmaps = &font->cmaps;
	font->font_desc.glyph_dsc = font->glyph_dsc;
	font->font_desc.cmap_num = 1;
	font->font_desc.bpp = 1;

	font->cmaps.range_start = start;
	font->cmaps.range_length = len;
	font->cmaps.glyph_id_start = 0;
	font->cmaps.type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY;

	for (int i=1; i<(len+1); i++) {
		font->glyph_dsc[i].bitmap_index = height*((width+7)>>3) * i;
		font->glyph_dsc[i].adv_w = ovl?(width<<2)+(width<<1):(width<<4);
		font->glyph_dsc[i].box_w = width;
		font->glyph_dsc[i].box_h = height;
		font->glyph_dsc[i].ofs_x = 2;
		font->glyph_dsc[i].ofs_y = -2;
	}

	return &font->font;
}
