#ifndef MAME_F256_F256_H
#define MAME_F256_F256_H

#pragma once

#include "machine/ram.h"

#define MASTER_CLOCK        (XTAL(25'175'000))
#define MAINCPU_TAG                 "maincpu"
#define RAM_TAG                     "ram"
#define ROM_TAG                     "rom"
#define FLASH_TAG                   "flash"

class f256_state : public driver_device
{
public:
	f256_state(const machine_config &mconfig, device_type type, const char *tag);
    void f256k(machine_config &config);

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
    required_device<cpu_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_memory_region m_rom;

	//required_device<screen_device> m_screen;
    void program_map(address_map &map);
	uint8_t mmu_lut[32];
	void reset_mmu();
	u8   lut_r(offs_t offset);
	void lut_w(offs_t offset, u8 data);
	u8   mem_r(offs_t offset);
	void mem_w(offs_t offset, u8 data);
};

#endif // MAME_F256_F256_H
