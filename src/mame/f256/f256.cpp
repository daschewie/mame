#include "emu.h"
#include "f256.h"
//#include "screen.h"

#include "cpu/m6502/w65c02.h"
//#include "sound/sn76496.h"

#define MASTER_CLOCK        (XTAL(25'175'000))

f256_state::f256_state(const machine_config &mconfig, device_type type, const char *tag) :
    driver_device(mconfig, type, tag),
    m_maincpu(*this, MAINCPU_TAG)
    // m_pia_0(*this, "pia0"),
    // m_pia_1(*this, "pia1"),
    // m_dac(*this, "dac"),
    // m_sbs(*this, "sbs"),
    //m_screen(*this, "screen")
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
    W65C02(config, m_maincpu, MASTER_CLOCK/4);
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
    map(0x0000, 0x7fff).ram();
    map(0x8000, 0xffff).rom();
};

ROM_START(f256k)
	ROM_REGION(0x8000,MAINCPU_TAG,0)
    // Offsets are based on the REGION base address
	ROM_LOAD("3b.bin",   0x0000, 0x2000, CRC(00880c2a) SHA1(bc89208e94674b8157ecb8107cccb6136b03896c))
    ROM_LOAD("3c.bin",   0x2000, 0x2000, CRC(9a67f0d7) SHA1(16152cb6045cad4c9fcd773a961898d57d596c3a))
    ROM_LOAD("3d.bin",   0x4000, 0x2000, CRC(069af1da) SHA1(72101736144cd545484f34393ee0d8df06b1eedb))
ROM_END

//    YEAR  NAME   PARENT COMPAT  MACHINE    INPUT    CLASS        INIT        COMPANY              FULLNAME                        FLAGS
COMP( 2024, f256k,    0,      0,    f256k,   f256k,   f256_state, empty_init, "Stefany Allaire", "F256K 8-bit Retro System",      MACHINE_IS_INCOMPLETE  )
