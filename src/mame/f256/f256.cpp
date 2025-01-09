#include "emu.h"
#include "f256.h"

#include "cpu/m6502/w65c02.h"
#include "tiny_vicky.h"
//#include "sound/sn76496.h"

f256_state::f256_state(const machine_config &mconfig, device_type type, const char *tag) :
    driver_device(mconfig, type, tag)
    , m_maincpu(*this, MAINCPU_TAG)
    , m_ram(*this, RAM_TAG)
    , m_iopage0(*this, IOPAGE0_TAG)
    , m_iopage1(*this, IOPAGE1_TAG)
    , m_iopage2(*this, IOPAGE2_TAG)
    , m_iopage3(*this, IOPAGE3_TAG)
    , m_rom(*this, ROM_TAG)
    , m_font(*this, FONT_TAG)
    // m_pia_0(*this, "pia0"),
    // m_pia_1(*this, "pia1"),
    // m_dac(*this, "dac"),
    // m_sbs(*this, "sbs"),
    , m_screen(*this, SCREEN_TAG)
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
    RAM(config, m_ram).set_default_size("512k").set_default_value(0x0);
    RAM(config, m_iopage0).set_default_size("8k").set_default_value(0x0);
    RAM(config, m_iopage1).set_default_size("8k").set_default_value(0x0);
    RAM(config, m_iopage2).set_default_size("8k").set_default_value(0x0);
    RAM(config, m_iopage3).set_default_size("8k").set_default_value(0x0);
    SCREEN(config, m_screen, SCREEN_TYPE_RASTER);

    m_maincpu->set_addrmap(AS_PROGRAM, &f256_state::program_map);
    //m_maincpu->set_addrmap(AS_DATA, &f256_state::data_map);

    m_screen->set_refresh_hz(60); // Refresh rate (e.g., 60Hz)
    m_screen->set_size(800,525);
    m_screen->set_visarea(0, 639, 0, 479);
    m_screen->set_screen_update(m_video, FUNC(tiny_vicky_video_device::screen_update));
}

f256_state::~f256_state()
{
    m_video.stop();
}
/*
    Memory map
    $00:0000 - $07:FFFF	RAM
    $08:0000 - $0F:FFFF	Flash
    $10:0000 - $13:FFFF	Expansion Memory
    $14:0000 - $1F:FFFF	Reserved
*/
void f256_state::program_map(address_map &map)
{
    // the address range 0:F
    // 0: LUT Edit $80 allows writing to 8 to F, LUT Select 0 to 3
    map(0x0000, 0x000F).rw(FUNC(f256_state::lut_r), FUNC(f256_state::lut_w));
    map(0x0010, 0xFFFF).rw(FUNC(f256_state::mem_r), FUNC(f256_state::mem_w));
};

void f256_state::data_map(address_map &map)
{
    map(0x0000, 0x1FFF).ram().share(IOPAGE0_TAG);
    map(0x0000, 0x1FFF).ram().share(IOPAGE1_TAG);
    map(0x0000, 0x1FFF).ram().share(IOPAGE2_TAG);
    map(0x0000, 0x1FFF).ram().share(IOPAGE3_TAG);
}

u8   f256_state::lut_r(offs_t offset)
{
    // addresses 0 to 7 are always read from RAM
    if (offset < 8)
    {
        return m_ram->read(offset);
    }
    else
    {
        uint8_t mmu = m_ram->read(0);
        // if we are not in edit mode, return RAM data
        if ((mmu & 0x80) == 0)
        {
            return m_ram->read(offset);
        }
        else
        {
            mmu = (mmu >> 4) & 0x3;  // use the top nibble
            return mmu_lut[(mmu & 0x3) * 8 + (offset-8)];
        }
    }

}
void f256_state::lut_w(offs_t offset, u8 data)
{
    // addresses 0:7 are always RAM
    if (offset < 8)
    {
        m_ram->write(offset, data);
    }
    else
    {
        u8 mmu = m_ram->read(0);
        if ((mmu & 0x80) == 0)
        {
            m_ram->write(offset, data);
        }
        else
        {
            mmu = (mmu >> 4) & 0x3;  // use the top nibble
            mmu_lut[mmu * 8 + (offset - 8)] = data;
            // TODO: do the bank switching here???
        }
    }
}

// offsets are 0x10 based
u8   f256_state::mem_r(offs_t offset)
{
    // find which slot to read
    uint8_t mmu = m_ram->read(0) & 3;
    uint16_t adj_addr = offset + 0x10;
    uint8_t slot = adj_addr >> 13;
    uint16_t laddr = adj_addr & 0x1FFF;
    uint8_t fslot = mmu_lut[mmu * 8 + slot];
    if (fslot < 0x40)
    {
        // Slot 6 is where I/O devices are located, when IO_DISABLE is 0
        if (slot == 6)
        {
            uint8_t ioreg = m_ram->read(1);
            // if IO_DISABLED is 1, then slot 6 is regular RAM
            if ((ioreg & 0x4) == 0)
            {
                switch (ioreg & 0x3)
                {
                    case 0:
                        // here we have a number of devices to read
                        if (
                            (adj_addr >= 0xC000 && adj_addr < 0xD400) ||  // gamma, mouse graphics, vicky registers, bitmaps, tiles
                            (adj_addr >= 0xD800 && adj_addr < 0xD880) ||  // text colors
                            (adj_addr >= 0xD800 && adj_addr < 0xD880)     // sprite registers
                           )
                        {
                            return m_iopage0->read(adj_addr - 0xC000);
                        }
                        else if (adj_addr >= 0xD400 && adj_addr < 0xD580)
                        {
                            // SID
                        }
                        else if (adj_addr >= 0xD580 && adj_addr < 0xD583)
                        {
                            // OPL3
                        }
                        else if (adj_addr >= 0xD600 && adj_addr < 0xD620)
                        {
                            // PSG
                        }
                        else if (adj_addr >= 0xD650 && adj_addr < 0xD660)
                        {
                            // Timers
                        }
                        else if (adj_addr >= 0xD660 && adj_addr < 0xD670)
                        {
                            // Interrupt Registers
                        }
                        return 0;
                        break;
                    case 1:
                        return m_iopage1->read(adj_addr - 0xC000);
                        break;
                    case 2:
                        return m_iopage2->read(adj_addr - 0xC000);
                        break;
                    case 3:
                        return m_iopage3->read(adj_addr - 0xC000);
                        break;
                }
            }
        }
        offs_t address = (fslot << 13) + laddr;
        return m_ram->read(address);
    }
    else
    {
        offs_t address = (fslot << 13) + laddr;
        return m_rom->as_u8(address );
    }

}
void f256_state::mem_w(offs_t offset, u8 data)
{
    // find which slot to write
    uint8_t mmu = m_ram->read(0) & 3;
    uint16_t adj_addr = offset + 0x10;
    uint8_t slot = adj_addr >> 13;
    uint16_t laddr = adj_addr & 0x1FFF;

    uint8_t fslot = mmu_lut[mmu * 8 + slot];
    if (fslot < 0x40)
    {
        // Slot 6 is where I/O devices are located, when IO_DISABLE is 0
        if (slot == 6)
        {
            uint8_t ioreg = m_ram->read(1);
            // if IO_DISABLED is 1, then slot 6 is regular RAM
            if ((ioreg & 0x4) == 0)
            {
                switch (ioreg & 0x3)
                {
                    case 0:
                        // here we have a number of devices to write
                        if (
                            (adj_addr >= 0xC000 && adj_addr < 0xD400) ||  // gamma, mouse graphics, vicky registers, bitmaps, tiles
                            (adj_addr >= 0xD800 && adj_addr < 0xD880) ||  // text colors
                            (adj_addr >= 0xD800 && adj_addr < 0xD880)     // sprite registers
                           )
                        {
                            m_iopage0->write(adj_addr - 0xC000, data);
                        }
                        else if (adj_addr >= 0xD400 && adj_addr < 0xD580)
                        {
                            // SID
                        }
                        else if (adj_addr >= 0xD580 && adj_addr < 0xD583)
                        {
                            // OPL3
                        }
                        else if (adj_addr >= 0xD600 && adj_addr < 0xD620)
                        {
                            // PSG
                        }
                        else if (adj_addr >= 0xD650 && adj_addr < 0xD660)
                        {
                            // Timers
                        }
                        else if (adj_addr >= 0xD660 && adj_addr < 0xD670)
                        {
                            // Interrupt Registers
                        }
                        break;
                    case 1:
                        m_iopage1->write(adj_addr - 0xC000, data);
                        break;
                    case 2:
                        m_iopage2->write(adj_addr - 0xC000, data);
                        break;
                    case 3:
                        m_iopage3->write(adj_addr - 0xC000, data);
                        break;
                }
            }
        }
        else
        {
            offs_t address = (fslot << 13) + laddr;
            m_ram->write(address, data);
        }
    }
}

void f256_state::reset_mmu()
{
    for (int i =0; i < 32; i++)
    {
        if (i % 8 == 7)
        {
            mmu_lut[i]= 0x7f;
        }
        else
        {
            mmu_lut[i] = i % 8;
        }
    }
}

//-------------------------------------------------
//  device_start
//-------------------------------------------------
void f256_state::device_start()
{
	driver_device::device_start();
    reset_mmu();
    // TODO: Copy the font from file to IO Page 1
    for (int i=0;i<0x800;i++)
    {
        m_iopage1->write(i, m_font->as_u8(i));
    }

    m_video.set_videoram(m_ram->pointer(), m_iopage0->pointer(), m_iopage1->pointer(), m_iopage2->pointer(), m_iopage3->pointer());
    m_video.start();
}

//-------------------------------------------------
//  device_reset
//-------------------------------------------------
void f256_state::device_reset()
{
	driver_device::device_reset();
    reset_mmu();
}

static void construct_ioport_f256k(device_t &owner, ioport_list &portlist, std::ostream &errorbuf)
{

}

ROM_START(f256k)
	ROM_REGION(0x10'0000,ROM_TAG,0)
    // Offsets are based on the REGION base address - offset by 0x10 because of map.
	ROM_LOAD("3b.bin",   0x0F'6000, 0x2000, CRC(00880c2a) SHA1(bc89208e94674b8157ecb8107cccb6136b03896c))
    ROM_LOAD("3c.bin",   0x0F'8000, 0x2000, CRC(9a67f0d7) SHA1(16152cb6045cad4c9fcd773a961898d57d596c3a))
    ROM_LOAD("3d.bin",   0x0F'A000, 0x2000, CRC(069af1da) SHA1(72101736144cd545484f34393ee0d8df06b1eedb))
    ROM_LOAD("3e.bin",   0x0F'C000, 0x2000, CRC(8db5d29b) SHA1(6d2f4b43bd3ae211bc15a36212fb6c22090edbf7))
    ROM_LOAD("3f.bin",   0x0F'E000, 0x2000, CRC(44eb158f) SHA1(b56eb6288e8f05084726d0ab4ee784bbc32d91f8))

    ROM_REGION(0x0800,FONT_TAG,0)
    ROM_LOAD("f256jr_font_micah_jan25th.bin", 0x0000, 0x0800, CRC(6d66da85) SHA1(377dc27ff3a4ae2d80d740b2d16373f8e639eef6))
ROM_END

//    YEAR  NAME   PARENT COMPAT  MACHINE    INPUT    CLASS        INIT        COMPANY              FULLNAME                        FLAGS
COMP( 2024, f256k,    0,      0,    f256k,   f256k,   f256_state, empty_init, "Stefany Allaire", "F256K 8-bit Retro System",      MACHINE_IS_INCOMPLETE | MACHINE_NO_SOUND  )
