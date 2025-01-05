#include "emu.h"
#include "f256.h"

#include "cpu/m6502/w65c02.h"
#include "sound/sn76496.h"

f256_state::f256_state(const machine_config &mconfig, device_type type, const char *tag) :
    driver_device(mconfig, type, tag),
    m_maincpu(*this, MAINCPU_TAG)
    // m_pia_0(*this, "pia0"),
    // m_pia_1(*this, "pia1"),
    // m_dac(*this, "dac"),
    // m_sbs(*this, "sbs"),
    // m_screen(*this, "screen"),
    // m_cococart(*this, "ext"),
    // m_ram(*this, RAM_TAG),
    // m_cassette(*this, "cassette"),
    // m_floating(*this, "floating"),
    // m_rs232(*this, RS232_TAG),
    // m_vhd_0(*this, "vhd0"),
    // m_vhd_1(*this, "vhd1"),
    // m_beckerport(*this, "dwsock"),
    // m_beckerportconfig(*this, BECKERPORT_TAG),
    // m_irqs(*this, "irqs"),
    // m_firqs(*this, "firqs"),
    // m_keyboard(*this, "row%u", 0),
    // m_joystick_type_control(*this, CTRL_SEL_TAG),
    // m_joystick_hires_control(*this, HIRES_INTF_TAG),
    // m_in_floating_bus_read(false)
{

}

void f256_state::f256k(machine_config &config)
{
    m_maincpu->set_addrmap(AS_PROGRAM, &f256_state::program_map);

}

//-------------------------------------------------
//  device_start
//-------------------------------------------------
void f256_state::device_start()
{
	driver_device::device_start();
}

//-------------------------------------------------
//  device_reset
//-------------------------------------------------
void f256_state::device_reset()
{
	driver_device::device_reset();
}

static void construct_ioport_f256k(device_t &owner, ioport_list &portlist, std::ostream &errorbuf)
{
}

// Memory map
void f256_state::program_map(address_map &map) {
    map(0x0000, 0x7fff).rom(); // Example ROM mapping
    map(0x8000, 0x87ff).ram(); // Example RAM mapping
};

ROM_START(f256k)
	ROM_REGION(0x8000,MAINCPU_TAG,0)
	ROM_LOAD("coco3.rom",   0x0000, 0x8000, CRC(b4c88d6c) SHA1(e0d82953fb6fd03768604933df1ce8bc51fc427d))
ROM_END

//    YEAR  NAME   PARENT COMPAT  MACHINE    INPUT    CLASS        INIT        COMPANY              FULLNAME                        LAGS
COMP( 2024, f256k,    0,      0,    f256k,   f256k,   f256_state, empty_init, "Stefany Allaire", "F256K 8-bit Retro System",          0 )
