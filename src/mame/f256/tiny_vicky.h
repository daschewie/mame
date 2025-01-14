#ifndef MAME_F256_TINYVICKYVIDEO_H
#define MAME_F256_TINYVICKYVIDEO_H

#define CHAR_HEIGHT     8
#define CHAR_WIDTH      8

class tiny_vicky_video_device : public device_t
{
public:
    tiny_vicky_video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
    tiny_vicky_video_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock = 0);
    //tiny_vicky_video_device() = default;

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
    auto irq_handler() { return m_irq_handler.bind(); }
protected:
    // device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
    // virtual void device_add_mconfig(machine_config &config) override;
private:
    bool running = false;
    devcb_write_line m_irq_handler;
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

DECLARE_DEVICE_TYPE(TINY_VICKY, tiny_vicky_video_device)
#endif
