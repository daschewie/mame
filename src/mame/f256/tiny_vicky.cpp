#include "emu.h"
#include "tiny_vicky.h"

#include "machine/ram.h"
#include "screen.h"

void tiny_vicky_video_device::start()
{

}

uint32_t tiny_vicky_video_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
    // Clear the screen (fill with black)
    bitmap.fill(rgb_t::black(), cliprect);

    // Example: Draw a red rectangle
    for (int y = 50; y < 150; y++) {
        for (int x = 50; x < 150; x++) {
            bitmap.pix(y + 100, x + 100) = rgb_t(0, 255, 0); // Green color
        }
    }
    return 0;
}
