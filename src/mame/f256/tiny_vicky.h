#ifndef MAME_F256_TINYVICKYVIDEO_H
#define MAME_F256_TINYVICKYVIDEO_H

#include "emupal.h"

class tiny_vicky_video_device
{
public:
    // construction/destruction
	tiny_vicky_video_device() = default;

    // screen update
    uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

    void set_videoram(uint8_t *videoram) { m_videoram = videoram; }
    void start();
protected:

private:
    uint8_t *m_videoram = nullptr; // Pointer to video RAM
};

#endif
