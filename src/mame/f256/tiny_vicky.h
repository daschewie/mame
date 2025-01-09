#ifndef MAME_F256_TINYVICKYVIDEO_H
#define MAME_F256_TINYVICKYVIDEO_H

#include "emupal.h"

#define CHAR_HEIGHT     8
#define CHAR_WIDTH      8

class tiny_vicky_video_device
{
public:
    // construction/destruction
	tiny_vicky_video_device() = default;

    // screen update
    uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
    void stop()
    {
        running = false;
        videoram_ptr = nullptr;
        iopage0_ptr = nullptr;
        iopage1_ptr = nullptr;
        iopage2_ptr = nullptr;
        iopage3_ptr = nullptr;
    }
    void set_videoram(uint8_t *videoram, uint8_t *iopage0, uint8_t *iopage1, uint8_t *iopage2, uint8_t *iopage3)
    {
        videoram_ptr = videoram;
        iopage0_ptr = iopage0;
        iopage1_ptr = iopage1;
        iopage2_ptr = iopage2;
        iopage3_ptr = iopage3;
    }
    void start()
    {
        running = true;
    }
protected:

private:
    bool running = false;
    uint8_t *videoram_ptr = nullptr; // Pointer to video RAM
    uint8_t *iopage0_ptr = nullptr;  // Pointer to IO Page 0
    uint8_t *iopage1_ptr = nullptr;  // Pointer to IO Page 1
    uint8_t *iopage2_ptr = nullptr;  // Pointer to IO Page 2
    uint8_t *iopage3_ptr = nullptr;  // Pointer to IO Page 3

    // cursor handling routines
    bool cursor_visible = false;
    bool enable_cursor_flash = true;
    uint8_t cursor_counter = 0;
    uint8_t cursor_flash_rate = 60;
    uint8_t cursor_char;

    rgb_t get_text_lut(uint8_t color_index, bool fg, bool gamma);
    void draw_text(bitmap_rgb32 &bitmap, uint8_t mcr, bool enable_gamma, uint8_t brd_x, uint8_t brd_y, uint16_t line, uint16_t x_res, uint16_t y_res);
};

#endif
