#include "emu.h"
#include "tiny_vicky.h"

#include "machine/ram.h"
#include "screen.h"

DEFINE_DEVICE_TYPE(TINY_VICKY, tiny_vicky_video_device, "tiny_vicky", "F256K Tiny Vicky")

tiny_vicky_video_device::tiny_vicky_video_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock):
    device_t(mconfig, type, tag, owner, clock)
    , m_irq_handler(*this)
{

}
tiny_vicky_video_device::tiny_vicky_video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
    tiny_vicky_video_device(mconfig, TINY_VICKY, tag, owner, clock)
{

}
rgb_t tiny_vicky_video_device::get_text_lut(uint8_t color_index, bool fg, bool gamma)
{
    uint8_t red =   iopage0_ptr[(fg ? 0x1800 : 0x1840) + color_index * 4 + 2];
    uint8_t green = iopage0_ptr[(fg ? 0x1800 : 0x1840) + color_index * 4 + 1];
    uint8_t blue =  iopage0_ptr[(fg ? 0x1800 : 0x1840) + color_index * 4 ];
    if (gamma)
    {
        blue = iopage0_ptr[blue];
        green = iopage0_ptr[0x400 + green];
        red = iopage0_ptr[0x800 + red];
    }
    return rgb_t(red, green, blue);
}

uint32_t tiny_vicky_video_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
    if (running && iopage0_ptr)
    {
        uint8_t mcr = iopage0_ptr[0x1000];
        cursor_counter++;
        if (cursor_counter > cursor_flash_rate *2) 
        {
            cursor_counter = 0;
        }
        // if MCR =0 or MCR bit 7 is set, then video is disabled
        if (mcr != 0 && (mcr & 0x80) == 0)
        {
            // TODO: generate start of frame (SOF) interrupt
            m_irq_handler(1);
            uint8_t border_reg = iopage0_ptr[0x1004];
            bool display_border = (border_reg & 0x1) > 0;
            bool enable_gamma = (mcr & 0x40) > 0;
            bool enable_graphics = (mcr & 0x4) > 0;

            uint8_t border_x = 0;
            uint8_t border_y = 0;
            rgb_t border_color = rgb_t();
            if (display_border) {
                border_x = iopage0_ptr[0x1008];
                border_y = iopage0_ptr[0x1009];
            }

            // Prepare to raise SOL intertups


            // Clear the screen (fill with black)
            //bitmap.fill(rgb_t::black(), cliprect);
            for (uint16_t y = 0; y < 480; y++)
            {
                // border color can change during painting the screen
                if (display_border)
                {
                    border_color = rgb_t(iopage0_ptr[0x1007], iopage0_ptr[0x1006], iopage0_ptr[0x1005]);
                }

                if (y < border_y || y >= 480 - border_y)
                {
                    for (int x = 0; x < 640; x++)
                    {
                        bitmap.pix(y, x) = border_color;
                    }
                }
                else
                {
                    rgb_t background_color = rgb_t();
                    if (enable_graphics)
                    {
                        background_color = rgb_t(iopage0_ptr[0x100F], iopage0_ptr[0x100E], iopage0_ptr[0x100D]);
                    }
                    for (int x = 0; x < 640; x++)
                    {
                        bitmap.pix(y, x) = x < border_x || x >= 640 - border_x ? border_color : background_color;
                    }

                    if (enable_graphics)
                    {

                    }
                    // Only display text in these cases
                    if ((mcr & 0x7) == 1 || (mcr & 0x7) == 3 || (mcr & 0x7) == 7)
                    {
                        draw_text(bitmap, mcr, enable_gamma, border_x, border_y, y, (uint16_t)640, (uint16_t)480);
                    }
                }
            }
        }
    }

    return 0;
}


void tiny_vicky_video_device::draw_text(bitmap_rgb32 &bitmap, uint8_t mcr, bool enable_gamma, uint8_t brd_x, uint8_t brd_y, uint16_t line, uint16_t x_res, uint16_t y_res)
{
    bool overlay = (mcr & 0x2) > 0;
    uint8_t mcrh = iopage0_ptr[0x1001] & 0x3F;
    bool double_x = (mcrh & 0x2) > 0;
    bool double_y = (mcrh & 0x4) > 0;
    bool use_font1 = (mcrh & 0x20) > 0;
    bool overlay_font = (mcrh & 0x10) > 0;
    int txt_line = ((double_y ? line / 2 : line) - brd_y) / CHAR_HEIGHT;
    // Each character is defined by 8 bytes
    int font_line = ((double_y ? line / 2 : line) - brd_y) % CHAR_HEIGHT;
    int txt_cols = double_x ? 40 : 80;

    // do cursor stuff
    int cursor_x = iopage0_ptr[0x1014] + 0xFF * iopage0_ptr[0x1015];
    int cursor_y = iopage0_ptr[0x1016] + 0xFF * iopage0_ptr[0x1017];

    // TODO: enabling/disabling cursor and flashing should be handled somewhere else... maybe in the top screen_update function.
    bool enable_cursor = (iopage0_ptr[0x1010] & 0x1) > 0;
    enable_cursor_flash = (iopage0_ptr[0x1010] & 0x8) == 0;
    // flash rate is the count of screen updates
    // 1s   = 60 ==> 00
    // 0.5s = 30 ==> 01
    // 0.25s= 15 ==> 10
    // 0.20s= 12 ==> 11
    cursor_flash_rate = 60;
    switch ((iopage0_ptr[0x1010] & 6) >> 1)
    {
        case 1:
            cursor_flash_rate = 30;
            break;
        case 2:
            cursor_flash_rate = 15;
            break;
        case 3:
            cursor_flash_rate = 12;
            break;
    }
    int screen_x = brd_x;
    // I'm assuming that if enable_cursor_flash is 0, then the cursor is always visible
    cursor_visible = enable_cursor && (enable_cursor_flash && (cursor_counter < cursor_flash_rate));
    for (int col = 0; col < txt_cols; col++)
    {
        int x = col * CHAR_WIDTH;
        if (x + brd_x > x_res - 1 - brd_x)
        {
            continue;
        }
        int offset = 0;
        if (col < (double_x ? 40 : 80))
        {
            // offset is always based on 80 columns
            offset = 80 * txt_line + col;
        }
        // Each character will have foreground and background colors
        uint8_t character = iopage2_ptr[offset];
        uint8_t color = iopage3_ptr[offset];

        // Display the cursor - this replaces the text character
        if (cursor_x == col && cursor_y == txt_line && cursor_visible)
        {
            character = iopage0_ptr[0x1012];
        }

        uint8_t fg_color_index = (color & 0xF0) >> 4;
        uint8_t bg_color_index = (color & 0x0F);

        rgb_t fg_color = get_text_lut(fg_color_index, true, enable_gamma);
        rgb_t bg_color = get_text_lut(bg_color_index, false, enable_gamma);

        uint8_t value = iopage1_ptr[(use_font1 ? 0x800 : 0) + character * 8 + font_line];

        // For each bit in the font, set the foreground color - if the bit is 0 and overlay is set, skip it (keep the background)
        for (int b = 0x80; b > 0; b >>= 1)
        {
            if (double_x)
            {
                if ((value & b) != 0)
                {
                    bitmap.pix(line, screen_x) = fg_color;
                    bitmap.pix(line, screen_x+1) = fg_color;
                }
                else if (!overlay || (overlay_font && (bg_color == 0)))
                {
                    bitmap.pix(line, screen_x) = bg_color;
                    bitmap.pix(line, screen_x+1) = bg_color;
                }
                screen_x += 2;
            }
            else
            {
                if ((value & b) != 0)
                {
                    bitmap.pix(line, screen_x) = fg_color;
                }
                else if (!overlay || (overlay_font && (bg_color != 0)))
                {
                    bitmap.pix(line, screen_x) = bg_color;
                }
                screen_x++;
            }
        }
    }
}
void tiny_vicky_video_device::device_start()
{
}
void tiny_vicky_video_device::device_reset()
{

}
