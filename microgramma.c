/*******************************************************************************
 * Size: 18 px
 * Bpp: 4
 * Opts: --bpp 4 --size 18 --no-compress --font MicrogrammaNormal1.ttf --symbols 0123456789PSI --format lvgl -o microgramma.c
 ******************************************************************************/

#include "lvgl.h"

#ifndef MICROGRAMMA
#define MICROGRAMMA 1
#endif

#if MICROGRAMMA

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0030 "0" */
    0x3, 0x9c, 0xef, 0xff, 0xfe, 0xc9, 0x30, 0x2f,
    0xfe, 0xba, 0xaa, 0xab, 0xef, 0xf2, 0x8f, 0x80,
    0x0, 0x0, 0x0, 0x7, 0xf8, 0xaf, 0x20, 0x0,
    0x0, 0x0, 0x1, 0xfb, 0xcf, 0x0, 0x0, 0x0,
    0x0, 0x0, 0xfc, 0xcf, 0x0, 0x0, 0x0, 0x0,
    0x0, 0xfd, 0xdf, 0x0, 0x0, 0x0, 0x0, 0x0,
    0xfd, 0xcf, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfc,
    0xaf, 0x20, 0x0, 0x0, 0x0, 0x1, 0xfb, 0x8f,
    0x70, 0x0, 0x0, 0x0, 0x7, 0xf8, 0x2f, 0xfd,
    0xba, 0x99, 0xab, 0xdf, 0xf3, 0x3, 0x9c, 0xef,
    0xff, 0xfe, 0xc9, 0x30,

    /* U+0031 "1" */
    0x0, 0x0, 0x9f, 0xfc, 0x0, 0x9, 0xfc, 0xfc,
    0x0, 0xaf, 0xa1, 0xfc, 0xa, 0xfa, 0x1, 0xfc,
    0x1e, 0x90, 0x1, 0xfc, 0x1, 0x0, 0x1, 0xfc,
    0x0, 0x0, 0x1, 0xfc, 0x0, 0x0, 0x1, 0xfc,
    0x0, 0x0, 0x1, 0xfc, 0x0, 0x0, 0x1, 0xfc,
    0x0, 0x0, 0x1, 0xfc, 0x0, 0x0, 0x1, 0xfc,

    /* U+0032 "2" */
    0x4, 0xad, 0xef, 0xff, 0xfe, 0xda, 0x30, 0x3f,
    0xfd, 0xba, 0xaa, 0xab, 0xdf, 0xf3, 0x9f, 0x60,
    0x0, 0x0, 0x0, 0x6, 0xf9, 0xbf, 0x10, 0x0,
    0x0, 0x0, 0x1, 0xfc, 0x7a, 0x0, 0x0, 0x0,
    0x0, 0x2, 0xfb, 0x0, 0x0, 0x0, 0x0, 0x1,
    0x4c, 0xf6, 0x1, 0x8b, 0xef, 0xff, 0xff, 0xff,
    0xa0, 0x2e, 0xfd, 0xba, 0x99, 0x98, 0x62, 0x0,
    0x9f, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0, 0xcf,
    0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0xcf, 0xa9,
    0x99, 0x99, 0x99, 0x99, 0x97, 0xcf, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfc,

    /* U+0033 "3" */
    0x2, 0x9c, 0xef, 0xff, 0xfe, 0xda, 0x30, 0xe,
    0xfe, 0xba, 0xaa, 0xab, 0xdf, 0xf4, 0x5f, 0xa0,
    0x0, 0x0, 0x0, 0x7, 0xfa, 0x7f, 0x50, 0x0,
    0x0, 0x0, 0x2, 0xfb, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x8, 0xf9, 0x0, 0x0, 0x9, 0xff, 0xff,
    0xff, 0xe1, 0x0, 0x0, 0x6, 0x99, 0x99, 0xae,
    0xf5, 0x45, 0x0, 0x0, 0x0, 0x0, 0x3, 0xfb,
    0xaf, 0x30, 0x0, 0x0, 0x0, 0x1, 0xfb, 0x7f,
    0x80, 0x0, 0x0, 0x0, 0x6, 0xf9, 0x2f, 0xfd,
    0xba, 0x99, 0xaa, 0xcf, 0xf3, 0x3, 0x9c, 0xef,
    0xff, 0xfe, 0xda, 0x30,

    /* U+0034 "4" */
    0x0, 0x0, 0x0, 0x0, 0x9f, 0xff, 0x0, 0x0,
    0x0, 0x0, 0x2, 0xcf, 0xcf, 0xf0, 0x0, 0x0,
    0x0, 0x5, 0xff, 0x70, 0xef, 0x0, 0x0, 0x0,
    0x9, 0xfe, 0x40, 0xe, 0xf0, 0x0, 0x0, 0x2c,
    0xfb, 0x10, 0x0, 0xef, 0x0, 0x0, 0x4e, 0xf8,
    0x0, 0x0, 0xe, 0xf0, 0x0, 0x8f, 0xe4, 0x0,
    0x0, 0x0, 0xef, 0x0, 0xd, 0xf1, 0x0, 0x0,
    0x0, 0xe, 0xf0, 0x0, 0xdf, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0x8, 0x99, 0x99, 0x99, 0x99,
    0x9f, 0xf9, 0x90, 0x0, 0x0, 0x0, 0x0, 0x0,
    0xef, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xe,
    0xf0, 0x0,

    /* U+0035 "5" */
    0xaf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf4, 0xaf,
    0xb9, 0x99, 0x99, 0x99, 0x99, 0x92, 0xaf, 0x30,
    0x0, 0x0, 0x0, 0x0, 0x0, 0xaf, 0x45, 0x79,
    0x99, 0x98, 0x63, 0x0, 0xaf, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xb0, 0xaf, 0xb3, 0x10, 0x0, 0x1,
    0x3b, 0xf7, 0x12, 0x0, 0x0, 0x0, 0x0, 0x2,
    0xfb, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xfc,
    0xcf, 0x10, 0x0, 0x0, 0x0, 0x1, 0xfc, 0xaf,
    0x50, 0x0, 0x0, 0x0, 0x7, 0xf9, 0x4f, 0xfd,
    0xba, 0x99, 0xab, 0xdf, 0xf2, 0x5, 0xad, 0xef,
    0xff, 0xfe, 0xc9, 0x20,

    /* U+0036 "6" */
    0x1, 0x8b, 0xef, 0xff, 0xfe, 0xda, 0x40, 0xe,
    0xfe, 0xba, 0xaa, 0xab, 0xcf, 0xf2, 0x4f, 0xa0,
    0x0, 0x0, 0x0, 0x8, 0xf7, 0x7f, 0x40, 0x0,
    0x0, 0x0, 0x2, 0x74, 0x8f, 0x33, 0x78, 0x99,
    0x98, 0x74, 0x0, 0x9f, 0xdf, 0xff, 0xff, 0xff,
    0xff, 0xd0, 0x9f, 0xc3, 0x10, 0x0, 0x1, 0x3d,
    0xf6, 0x8f, 0x40, 0x0, 0x0, 0x0, 0x5, 0xf8,
    0x7f, 0x40, 0x0, 0x0, 0x0, 0x4, 0xf9, 0x5f,
    0x80, 0x0, 0x0, 0x0, 0x9, 0xf7, 0x1f, 0xfd,
    0xba, 0x99, 0xab, 0xdf, 0xf2, 0x3, 0xad, 0xef,
    0xff, 0xfe, 0xda, 0x30,

    /* U+0037 "7" */
    0xaf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf5, 0x69,
    0x99, 0x99, 0x99, 0x99, 0x9c, 0xf5, 0x0, 0x0,
    0x0, 0x0, 0x0, 0xd, 0xf3, 0x0, 0x0, 0x0,
    0x0, 0x0, 0xbf, 0x70, 0x0, 0x0, 0x0, 0x0,
    0x9, 0xf9, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7f,
    0xb0, 0x0, 0x0, 0x0, 0x0, 0x5, 0xfd, 0x10,
    0x0, 0x0, 0x0, 0x0, 0x4f, 0xe2, 0x0, 0x0,
    0x0, 0x0, 0x2, 0xff, 0x30, 0x0, 0x0, 0x0,
    0x0, 0x1e, 0xf5, 0x0, 0x0, 0x0, 0x0, 0x0,
    0xdf, 0x70, 0x0, 0x0, 0x0, 0x0, 0xb, 0xf9,
    0x0, 0x0, 0x0, 0x0,

    /* U+0038 "8" */
    0x3, 0x9c, 0xef, 0xff, 0xfe, 0xca, 0x40, 0x3f,
    0xfd, 0xba, 0xaa, 0xab, 0xcf, 0xf6, 0x8f, 0x80,
    0x0, 0x0, 0x0, 0x4, 0xfb, 0x9f, 0x40, 0x0,
    0x0, 0x0, 0x1, 0xfc, 0x7f, 0xc4, 0x21, 0x11,
    0x12, 0x3a, 0xfa, 0xd, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xe2, 0x8f, 0xd7, 0x54, 0x44, 0x45, 0x7c,
    0xfb, 0xdf, 0x20, 0x0, 0x0, 0x0, 0x0, 0xff,
    0xdf, 0x0, 0x0, 0x0, 0x0, 0x0, 0xdf, 0xcf,
    0x40, 0x0, 0x0, 0x0, 0x2, 0xfe, 0x6f, 0xfc,
    0xba, 0x99, 0xaa, 0xcf, 0xf8, 0x5, 0xad, 0xef,
    0xff, 0xfe, 0xda, 0x60,

    /* U+0039 "9" */
    0x4, 0xad, 0xef, 0xff, 0xfe, 0xda, 0x40, 0x2f,
    0xfd, 0xba, 0xaa, 0xab, 0xdf, 0xf1, 0x7f, 0x80,
    0x0, 0x0, 0x0, 0x8, 0xf5, 0x8f, 0x40, 0x0,
    0x0, 0x0, 0x4, 0xf7, 0x7f, 0x80, 0x0, 0x0,
    0x0, 0x8, 0xf8, 0x3f, 0xfd, 0xba, 0x99, 0xaa,
    0xcf, 0xf9, 0x5, 0xbd, 0xef, 0xff, 0xfd, 0xa7,
    0xf9, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3, 0xf8,
    0x7e, 0x30, 0x0, 0x0, 0x0, 0x5, 0xf7, 0x6f,
    0x80, 0x0, 0x0, 0x0, 0xb, 0xf4, 0x1f, 0xfd,
    0xba, 0x99, 0xab, 0xef, 0xe0, 0x2, 0x9c, 0xef,
    0xff, 0xfe, 0xc8, 0x10,

    /* U+0049 "I" */
    0x8f, 0x58, 0xf5, 0x8f, 0x58, 0xf5, 0x8f, 0x58,
    0xf5, 0x8f, 0x58, 0xf5, 0x8f, 0x58, 0xf5, 0x8f,
    0x58, 0xf5,

    /* U+0050 "P" */
    0x2f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xa1, 0x2,
    0xfd, 0x99, 0x99, 0x99, 0x99, 0xcf, 0xc0, 0x2f,
    0xa0, 0x0, 0x0, 0x0, 0x0, 0xbf, 0x22, 0xfa,
    0x0, 0x0, 0x0, 0x0, 0x8, 0xf4, 0x2f, 0xa0,
    0x0, 0x0, 0x0, 0x0, 0x8f, 0x42, 0xfa, 0x0,
    0x0, 0x0, 0x0, 0x3e, 0xf1, 0x2f, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf7, 0x2, 0xfd, 0x99, 0x99,
    0x99, 0x99, 0x73, 0x0, 0x2f, 0xa0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x2, 0xfa, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x2f, 0xa0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x2, 0xfa, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0,

    /* U+0053 "S" */
    0x0, 0x6b, 0xde, 0xff, 0xff, 0xdb, 0x71, 0x0,
    0x8f, 0xfb, 0xaa, 0xaa, 0xab, 0xef, 0xd0, 0xd,
    0xf3, 0x0, 0x0, 0x0, 0x0, 0xbf, 0x40, 0xdf,
    0x0, 0x0, 0x0, 0x0, 0x4, 0xb4, 0xa, 0xfb,
    0x31, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1c, 0xff,
    0xff, 0xff, 0xfe, 0xdb, 0x60, 0x0, 0x2, 0x57,
    0x89, 0x99, 0xac, 0xff, 0x51, 0x42, 0x0, 0x0,
    0x0, 0x0, 0x5, 0xfa, 0x3f, 0x90, 0x0, 0x0,
    0x0, 0x0, 0x2f, 0xb1, 0xfe, 0x10, 0x0, 0x0,
    0x0, 0x7, 0xf9, 0xb, 0xff, 0xca, 0xa9, 0x9a,
    0xbd, 0xff, 0x30, 0x6, 0xad, 0xef, 0xff, 0xfe,
    0xc9, 0x20
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 267, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 84, .adv_w = 266, .box_w = 8, .box_h = 12, .ofs_x = 3, .ofs_y = 0},
    {.bitmap_index = 132, .adv_w = 267, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 216, .adv_w = 267, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 300, .adv_w = 266, .box_w = 15, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 390, .adv_w = 266, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 474, .adv_w = 266, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 558, .adv_w = 266, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 642, .adv_w = 267, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 726, .adv_w = 266, .box_w = 14, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 810, .adv_w = 86, .box_w = 3, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 828, .adv_w = 250, .box_w = 15, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 918, .adv_w = 262, .box_w = 15, .box_h = 12, .ofs_x = 0, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_1[] = {
    0x0, 0x7, 0xa
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 48, .range_length = 10, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    },
    {
        .range_start = 73, .range_length = 11, .glyph_id_start = 11,
        .unicode_list = unicode_list_1, .glyph_id_ofs_list = NULL, .list_length = 3, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 2,
    .bpp = 4,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};

extern const lv_font_t lv_font_montserrat_18;


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t microgramma = {
#else
lv_font_t microgramma = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 12,          /*The maximum line height required by the font*/
    .base_line = 0,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -1,
    .underline_thickness = 0,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = &lv_font_montserrat_18,
#endif
    .user_data = NULL,
};



#endif /*#if MICROGRAMMA*/

