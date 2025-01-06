#ifndef MAME_F256_F256_H
#define MAME_F256_F256_H

#pragma once

#define MAINCPU_TAG                 "maincpu"

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
	//required_device<screen_device> m_screen;
    void program_map(address_map &map);

};

#endif // MAME_F256_F256_H
