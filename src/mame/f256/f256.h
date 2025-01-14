#ifndef MAME_F256_F256_H
#define MAME_F256_F256_H

#pragma once

#include "machine/ram.h"
#include "screen.h"
#include "machine/bq4847.h"
#include "machine/6522via.h"
#include "sound/sn76496.h"
#include "utf8.h"
#include "tiny_vicky.h"

#define MASTER_CLOCK        (XTAL(25'175'000))
#define MAINCPU_TAG                 "maincpu"
#define RAM_TAG                     "ram"
#define IOPAGE0_TAG                 "iopage0"
#define IOPAGE1_TAG                 "iopage1"
#define IOPAGE2_TAG                 "iopage2"
#define IOPAGE3_TAG                 "iopage3"
#define ROM_TAG                     "rom"
#define FONT_TAG                    "font"
#define FLASH_TAG                   "flash"
#define VICKY_VIDEO_TAG             "vicky"
#define SCREEN_TAG                  "screen"

class f256_state : public driver_device
{
public:
	f256_state(const machine_config &mconfig, device_type type, const char *tag);
	~f256_state();
    void f256k(machine_config &config);

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
    required_device<cpu_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_device<ram_device> m_iopage0, m_iopage1, m_iopage2, m_iopage3;
	required_memory_region m_rom;
	required_memory_region m_font;
	required_device<screen_device> m_screen;
	required_device<bq4802_device> m_rtc;
	required_ioport_array<8> m_keyboard; // the number 16 will require 16 COL
	required_device<via6522_device> m_via6522_0;
	required_device<via6522_device> m_via6522_1;
	required_device<sn76489_device> m_sn;

	required_device<tiny_vicky_video_device> m_video;

    void program_map(address_map &map);
	void data_map(address_map &map);

	uint8_t mmu_lut[32];
	void reset_mmu();
	u8   lut_r(offs_t offset);
	void lut_w(offs_t offset, u8 data);
	u8   mem_r(offs_t offset);
	void mem_w(offs_t offset, u8 data);

	// screen update
	void sof_interrtupt(int state);
	void rtc_interrupt_handler(int state);
	void via0_interrupt(int state);
	void via1_interrupt(int state);
	uint8_t m_interrupt_reg[3] = { 0, 0 ,0};
	uint8_t m_interrupt_masks[3] = { 0xFF, 0xFF, 0xFF};

	// VIA0 - Atari joystick functions
	u8 via0_system_porta_r();
	u8 via0_system_portb_r();
	void via0_system_porta_w(u8 data);
	void via0_system_portb_w(u8 data);
	void via0_ca2_write(u8 data);
	void via0_cb2_write(u8 data);

	// VIA1 - Internal Keyboard
	uint8_t m_via_port_a = 0xFF;
	uint8_t m_via_port_b = 0xFF;

	u8 via1_system_porta_r();
	u8 via1_system_portb_r();
	void via1_system_porta_w(u8 data);
	void via1_system_portb_w(u8 data);
	void via1_ca2_write(u8 data);
	void via1_cb2_write(u8 data);

	// codec
	uint8_t m_codec[16] = {};
	TIMER_CALLBACK_MEMBER(codec_done);
};

#endif // MAME_F256_F256_H
