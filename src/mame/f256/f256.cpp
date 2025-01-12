#include "emu.h"
#include "f256.h"

#include "cpu/m6502/w65c02.h"
#include "tiny_vicky.h"

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
    , m_rtc(*this, "rtc")
    , m_keyboard(*this, "COL%u", 0)  // this with the 16 array - requires 16 COL of INPUTs
    , m_via6522_0(*this, "via6522_0")
	, m_via6522_1(*this, "via6522_1")
    , m_joy1(*this, "JOY1")
    , m_joy2(*this, "JOY2")
    , m_sn(*this, "sn76489")
    // m_floating(*this, "floating"),
    // m_rs232(*this, RS232_TAG),
    // m_vhd_0(*this, "vhd0"),
    // m_vhd_1(*this, "vhd1"),
    // m_beckerport(*this, "dwsock"),
    // m_beckerportconfig(*this, BECKERPORT_TAG),
    // m_irqs(*this, "irqs"),
    // m_firqs(*this, "firqs"),
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
    BQ4802(config, m_rtc, MASTER_CLOCK / 1000);  // clock in kHz
    //BQ4802(config, m_rtc);

    m_maincpu->set_addrmap(AS_PROGRAM, &f256_state::program_map);

    //m_maincpu->set_addrmap(AS_DATA, &f256_state::data_map);

    m_screen->set_refresh_hz(60); // Refresh rate (e.g., 60Hz)
    m_screen->set_size(800,525);
    m_screen->set_visarea(0, 639, 0, 479);
    m_screen->set_screen_update(m_video, FUNC(tiny_vicky_video_device::screen_update));

    MOS6522(config, m_via6522_0, MASTER_CLOCK / 16);  // Atari Joysticks
	m_via6522_0->readpa_handler().set(FUNC(f256_state::via0_system_porta_r));
	m_via6522_0->readpb_handler().set(FUNC(f256_state::via0_system_portb_r));
	m_via6522_0->writepa_handler().set(FUNC(f256_state::via0_system_porta_w));
	m_via6522_0->writepb_handler().set(FUNC(f256_state::via0_system_portb_w));
    m_via6522_0->ca2_handler().set(FUNC(f256_state::via0_ca2_write));
    m_via6522_0->cb2_handler().set(FUNC(f256_state::via0_cb2_write));

    // to handle interrupts before the CPU, look into "input_merger_device"
	// m_via6522_0->irq_handler().set(m_irqs, FUNC(input_merger_device::in_w<1>));

    MOS6522(config, m_via6522_1, MASTER_CLOCK / 16);  // Keyboard
    // m_via6522_1->readpa_handler().set(FUNC(f256_state::via1_system_porta_r));
	// m_via6522_1->readpb_handler().set(FUNC(f256_state::via1_system_portb_r));

    SN76489(config, m_sn, MASTER_CLOCK / 4);
    //set interrupt handler for the RTC
    m_rtc->irq().set(FUNC(f256_state::rtc_interrupt_handler));

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
                            (adj_addr >= 0xD900 && adj_addr < 0xDB00)     // sprite registers
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
                        else if (adj_addr >= 0xD620 && adj_addr < 0xD630)
                        {
                            // Codec
                        }
                        else if (adj_addr >= 0xD630 && adj_addr < 0xD640)
                        {
                            // UART
                        }
                        else if (adj_addr >= 0xD640 && adj_addr < 0xD64F)
                        {
                            // PS2
                        }
                        else if (adj_addr >= 0xD650 && adj_addr < 0xD660)
                        {
                            // Timers
                        }
                        else if (adj_addr >= 0xD660 && adj_addr < 0xD670)
                        {
                            // Interrupt Registers
                        }
                        else if (adj_addr >= 0xD690 && adj_addr < 0xD6A0)
                        {
                            // RTC
                            return m_rtc->read(adj_addr - 0xDC90);
                        }
                        else if (adj_addr >= 0xD6A0 && adj_addr < 0xD6C0)
                        {
                            // System Control Registers
                            // D6A0 - buzzer and LED controls - including RESET bit
                            // D6A1 -
                            // D6A2 - Set to 0xDE to enable software reset
                            // D6A3 - Set to 0xAD to enable software reset
                            // D6A4 - D6A6 : Random Number generator
                            // D6A7 - Macine ID - For the F256, the machine ID will be 0x02. For the F256k, the machine ID will be 0x12.
                            switch (adj_addr){
                                case 0xD6A7:
                                    return 0x12;
                                case 0XD6A8:
                                    return 'B';
                                case 0XD6A9:
                                    return '0';
                                case 0XD6AA:
                                    return 1;
                                case 0XD6AB:
                                    return 1;
                                case 0XD6AC:
                                    return 0;
                                case 0XD6AD:
                                    return 0x14;
                                case 0XD6AE:
                                    return 0;
                                case 0XD6AF:
                                    return 0;
                            }

                        }
                        else if (adj_addr >= 0xD800 && adj_addr < 0xD8C0)
                        {
                            // NES
                            return 0xFF;
                        }
                        else if (adj_addr >= 0xDB00 && adj_addr < 0xDB10)
                        {
                            // VIA1 - Keyboard for F256K
                            return m_via6522_1->read(adj_addr - 0xDB00);
                        }
                        else if (adj_addr >= 0xDC00 && adj_addr < 0xDC10)
                        {
                            // VIA0 - Atari Joystick
                            return m_via6522_0->read(adj_addr - 0xDC00);
                        }
                        else if (adj_addr >= 0xDD00 && adj_addr < 0xDD20)
                        {
                            // SD Card
                        }
                        else if (adj_addr >= 0xDE00 && adj_addr < 0xDE20)
                        {
                            // Math Coprocessor
                        }
                        else if (adj_addr >= 0xDF00 && adj_addr < 0xE000)
                        {
                            // DMA
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
                            (adj_addr >= 0xD900 && adj_addr < 0xDB00)     // sprite registers
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
                        else if (adj_addr >= 0xD620 && adj_addr < 0xD630)
                        {
                            // Codec
                        }
                        else if (adj_addr >= 0xD630 && adj_addr < 0xD640)
                        {
                            // UART
                        }
                        else if (adj_addr >= 0xD640 && adj_addr < 0xD64F)
                        {
                            // PS2
                        }
                        else if (adj_addr >= 0xD650 && adj_addr < 0xD660)
                        {
                            // Timers
                        }
                        else if (adj_addr >= 0xD660 && adj_addr < 0xD670)
                        {
                            // Interrupt Registers
                        }
                        else if (adj_addr >= 0xD690 && adj_addr < 0xD6A0)
                        {
                            // RTC
                            m_rtc->write(adj_addr - 0xDC90, data);
                        }
                        else if (adj_addr >= 0xD800 && adj_addr < 0xD8C0)
                        {
                            // NES - only address 0xD8800 is writable
                        }
                        else if (adj_addr >= 0xDB00 && adj_addr < 0xDC00)
                        {
                            // VIA1 - Keyboard for F256K
                            m_via6522_1->write(adj_addr - 0xDB00, data);
                        }
                        else if (adj_addr >= 0xDC00 && adj_addr < 0xDD00)
                        {
                            // VIA0 - Atari Joystick
                            m_via6522_0->write(adj_addr - 0xDC00, data);
                        }
                        else if (adj_addr >= 0xDD00 && adj_addr < 0xDD20)
                        {
                            // SD Card
                        }
                        else if (adj_addr >= 0xDE00 && adj_addr < 0xDE20)
                        {
                            // Math Coprocessor
                        }
                        else if (adj_addr >= 0xDF00 && adj_addr < 0xE000)
                        {
                            // DMA
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

    // set the current time on the RTC device
    time_t now;
    time(&now);
    system_time stnow = system_time(now);

    m_rtc->set_current_time(stnow);

    // Initialize the VIA0
    m_via6522_0->write(via6522_device::VIA_DDRB, 0xFF);  // DDRB
    m_via6522_0->write(via6522_device::VIA_DDRA, 0xFF);  // DDRA
    m_via6522_0->write(via6522_device::VIA_PB,   0xFF);  // JOYSTICK 2
    m_via6522_0->write(via6522_device::VIA_PA,   0xFF);  // JOYSTICK 1
    m_via6522_0->write(via6522_device::VIA_DDRB, 0);     // DDRB
    m_via6522_0->write(via6522_device::VIA_DDRA, 0);     // DDRA
}

//-------------------------------------------------
//  device_reset
//-------------------------------------------------
void f256_state::device_reset()
{
	driver_device::device_reset();
    reset_mmu();
}


void f256_state::rtc_interrupt_handler(int state)
{
    m_interrupt_reg[0] |= 0x10;
}

u8 f256_state::via0_system_porta_r()
{
    u8 data = ioport("JOY2")->read();
    logerror("VIA #0 Port A Read ioport JOY2: %02X\n", data);
    return data;
}
void f256_state::via0_system_porta_w(u8 data)
{
    logerror("VIA #0 Port A Write: %02X\n", data);
    ioport("JOY2")->write(data);
}
u8 f256_state::via0_system_portb_r()
{
    u8 data = ioport("JOY1")->read();
    logerror("VIA #0 Port A Read ioport JOY1: %02X\n", data);
    return data;
}
void f256_state::via0_system_portb_w(u8 data)
{
    logerror("VIA #0 Port B Write: %02X\n", data);
    ioport("JOY1")->write(data);
}
void f256_state::via0_ca2_write(u8 value)
{
    logerror("Write to CA2");
}
void f256_state::via0_cb2_write(u8 value)
{
    logerror("Write to CB2");
}

// TODO: check the ports
static INPUT_PORTS_START(f256k_joysticks)
    PORT_START("JOY1") /* Atari Joystick 1 */
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_8WAY PORT_PLAYER(1) PORT_NAME("Atari Joystick P1 Up")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_8WAY PORT_PLAYER(1) PORT_NAME("Atari Joystick P1 Down")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_8WAY PORT_PLAYER(1) PORT_NAME("Atari Joystick P1 Left")
    PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_8WAY PORT_PLAYER(1) PORT_NAME("Atari Joystick P1 Right")
    PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_BUTTON1)                  PORT_PLAYER(1) PORT_NAME("Atari Joystick P1 Button 1")
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_BUTTON2)                  PORT_PLAYER(1) PORT_NAME("Atari Joystick P1 Button 2")
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_BUTTON3)                  PORT_PLAYER(1) PORT_NAME("Atari Joystick P1 Button 3")

	PORT_START("JOY2") /* Atari Joystick 2 */
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_JOYSTICK_UP)    PORT_8WAY PORT_PLAYER(2) PORT_NAME("Atari Joystick P2 Up")       PORT_CODE(KEYCODE_8_PAD)
    PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_JOYSTICK_DOWN)  PORT_8WAY PORT_PLAYER(2) PORT_NAME("Atari Joystick P2 Down")     PORT_CODE(KEYCODE_2_PAD)
    PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_JOYSTICK_LEFT)  PORT_8WAY PORT_PLAYER(2) PORT_NAME("Atari Joystick P2 Left")     PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_JOYSTICK_RIGHT) PORT_8WAY PORT_PLAYER(2) PORT_NAME("Atari Joystick P2 Right")    PORT_CODE(KEYCODE_6_PAD)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_BUTTON1)                  PORT_PLAYER(2) PORT_NAME("Atari Joystick P2 Button 1") PORT_CODE(KEYCODE_0_PAD)
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_BUTTON2)                  PORT_PLAYER(2) PORT_NAME("Atari Joystick P2 Button 2") PORT_CODE(KEYCODE_1_PAD)
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_BUTTON3)                  PORT_PLAYER(2) PORT_NAME("Atari Joystick P2 Button 3") PORT_CODE(KEYCODE_5_PAD)
INPUT_PORTS_END


// u8 f256_state::via1_system_porta_r()
// {
//     return ioport("COL0")->read();
// }
// u8 f256_state::via1_system_portb_r()
// {
//     return ioport("JOY8")->read();
// }

/*  Port                                        Key description                 Emulated key                    Natural key         Shift 1         Shift 2 (Ctrl) */

static INPUT_PORTS_START(f256k)
    PORT_INCLUDE( f256k_joysticks )

	PORT_START("COL0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SHIFT")              PORT_CODE(KEYCODE_RSHIFT)       PORT_CODE(KEYCODE_LSHIFT) PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Q")                  PORT_CODE(KEYCODE_Q)            PORT_CHAR('Q')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F0")                 PORT_CODE(KEYCODE_F1)           PORT_CHAR(UCHAR_MAMEKEY(F1))
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1 !")                PORT_CODE(KEYCODE_1)            PORT_CHAR('1')      PORT_CHAR('!')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CAPSLOCK")           PORT_CODE(KEYCODE_CAPSLOCK)     PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SHIFTLOCK")          PORT_CODE(KEYCODE_LALT)         PORT_CHAR(UCHAR_MAMEKEY(LALT))
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TAB")                PORT_CODE(KEYCODE_TAB)          PORT_CHAR('\t')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ESCAPE")             PORT_CODE(KEYCODE_ESC)          PORT_CHAR(UCHAR_MAMEKEY(ESC))

	PORT_START("COL1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CTRL")               PORT_CODE(KEYCODE_LCONTROL)     PORT_CODE(KEYCODE_RCONTROL) PORT_CHAR(UCHAR_SHIFT_2)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3 #")                PORT_CODE(KEYCODE_3)            PORT_CHAR('3')      PORT_CHAR('#')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("W")                  PORT_CODE(KEYCODE_W)            PORT_CHAR('W')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2 \"")               PORT_CODE(KEYCODE_2)            PORT_CHAR('2')      PORT_CHAR('\"')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A")                  PORT_CODE(KEYCODE_A)            PORT_CHAR('A')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("S")                  PORT_CODE(KEYCODE_S)            PORT_CHAR('S')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Z")                  PORT_CODE(KEYCODE_Z)            PORT_CHAR('Z')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1")                 PORT_CODE(KEYCODE_F2)           PORT_CHAR(UCHAR_MAMEKEY(F2))

	PORT_START("COL2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4 $")                PORT_CODE(KEYCODE_4)            PORT_CHAR('4')      PORT_CHAR('$')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E")                  PORT_CODE(KEYCODE_E)            PORT_CHAR('E')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D")                  PORT_CODE(KEYCODE_D)            PORT_CHAR('D')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("X")                  PORT_CODE(KEYCODE_X)            PORT_CHAR('X')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C")                  PORT_CODE(KEYCODE_C)            PORT_CHAR('C')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SPACE")              PORT_CODE(KEYCODE_SPACE)        PORT_CHAR(' ')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F2")                 PORT_CODE(KEYCODE_F3)           PORT_CHAR(UCHAR_MAMEKEY(F3))

	PORT_START("COL3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5 %")                PORT_CODE(KEYCODE_5)            PORT_CHAR('5')      PORT_CHAR('%')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("T")                  PORT_CODE(KEYCODE_T)            PORT_CHAR('T')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("R")                  PORT_CODE(KEYCODE_R)            PORT_CHAR('R')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F")                  PORT_CODE(KEYCODE_F)            PORT_CHAR('F')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G")                  PORT_CODE(KEYCODE_G)            PORT_CHAR('G')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("V")                  PORT_CODE(KEYCODE_V)            PORT_CHAR('V')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3")                 PORT_CODE(KEYCODE_F4)           PORT_CHAR(UCHAR_MAMEKEY(F4))

	PORT_START("COL4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F4")                 PORT_CODE(KEYCODE_F5)           PORT_CHAR(UCHAR_MAMEKEY(F5))
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7 '")                PORT_CODE(KEYCODE_7)            PORT_CHAR('7')      PORT_CHAR('\'')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6 &")                PORT_CODE(KEYCODE_6)            PORT_CHAR('6')      PORT_CHAR('&')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Y")                  PORT_CODE(KEYCODE_Y)            PORT_CHAR('Y')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("H")                  PORT_CODE(KEYCODE_H)            PORT_CHAR('H')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B")                  PORT_CODE(KEYCODE_B)            PORT_CHAR('B')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F5")                 PORT_CODE(KEYCODE_F6)           PORT_CHAR(UCHAR_MAMEKEY(F6))

	PORT_START("COL5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8 (")                PORT_CODE(KEYCODE_8)            PORT_CHAR('8')      PORT_CHAR('(')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("I")                  PORT_CODE(KEYCODE_I)            PORT_CHAR('I')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("U")                  PORT_CODE(KEYCODE_U)            PORT_CHAR('U')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J")                  PORT_CODE(KEYCODE_J)            PORT_CHAR('J')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("N")                  PORT_CODE(KEYCODE_N)            PORT_CHAR('N')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("M")                  PORT_CODE(KEYCODE_M)            PORT_CHAR('M')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F6")                 PORT_CODE(KEYCODE_F7)           PORT_CHAR(UCHAR_MAMEKEY(F7))

	PORT_START("COL6")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F7")                 PORT_CODE(KEYCODE_F8)           PORT_CHAR(UCHAR_MAMEKEY(F8))
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9 )")                PORT_CODE(KEYCODE_9)            PORT_CHAR('9')      PORT_CHAR(')')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("O")                  PORT_CODE(KEYCODE_O)            PORT_CHAR('O')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("K")                  PORT_CODE(KEYCODE_K)            PORT_CHAR('K')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("L")                  PORT_CODE(KEYCODE_L)            PORT_CHAR('L')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(", <")                PORT_CODE(KEYCODE_COMMA)        PORT_CHAR(',')      PORT_CHAR('<')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F8")                 PORT_CODE(KEYCODE_F9)           PORT_CHAR(UCHAR_MAMEKEY(F9))

	PORT_START("COL7")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("- =")                PORT_CODE(KEYCODE_MINUS)        PORT_CHAR('-')      PORT_CHAR('=')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0")                  PORT_CODE(KEYCODE_0)            PORT_CHAR('0')
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("P")                  PORT_CODE(KEYCODE_P)            PORT_CHAR('P')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("@")                  PORT_CODE(KEYCODE_BACKSLASH)    PORT_CHAR('@')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("; +")                PORT_CODE(KEYCODE_COLON)        PORT_CHAR(';')      PORT_CHAR('+')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(". >")                PORT_CODE(KEYCODE_STOP)         PORT_CHAR('.')      PORT_CHAR('>')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F9")                 PORT_CODE(KEYCODE_F10)          PORT_CHAR(UCHAR_MAMEKEY(F10))

	PORT_START("COL8")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("^ ~")                PORT_CODE(KEYCODE_EQUALS)       PORT_CHAR('^') PORT_CHAR('~')
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(u8"_ £")              PORT_CODE(KEYCODE_TILDE)        PORT_CHAR('_') PORT_CHAR(0xA3)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("[ {")                PORT_CODE(KEYCODE_OPENBRACE)    PORT_CHAR('[') PORT_CHAR('{')
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(": *")                PORT_CODE(KEYCODE_QUOTE)        PORT_CHAR(':') PORT_CHAR('*')
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("] }")                PORT_CODE(KEYCODE_CLOSEBRACE)   PORT_CHAR(']') PORT_CHAR('}')
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("/ ?")                PORT_CODE(KEYCODE_SLASH)        PORT_CHAR('/') PORT_CHAR('?')
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("\\ |")               PORT_CODE(KEYCODE_BACKSLASH2)   PORT_CHAR('\\') PORT_CHAR('|')

	PORT_START("COL9")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_LEFT)            PORT_CODE(KEYCODE_LEFT)         PORT_CHAR(UCHAR_MAMEKEY(LEFT))
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_DOWN)            PORT_CODE(KEYCODE_DOWN)         PORT_CHAR(UCHAR_MAMEKEY(DOWN))
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_UP)              PORT_CODE(KEYCODE_UP)           PORT_CHAR(UCHAR_MAMEKEY(UP))
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("RETURN")             PORT_CODE(KEYCODE_ENTER)        PORT_CHAR(13)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DELETE")             PORT_CODE(KEYCODE_DEL)          PORT_CODE(KEYCODE_BACKSPACE) PORT_CHAR(8)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("COPY")               PORT_CODE(KEYCODE_END)          PORT_CHAR(UCHAR_MAMEKEY(END))
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_RIGHT)           PORT_CODE(KEYCODE_RIGHT)        PORT_CHAR(UCHAR_MAMEKEY(RIGHT))

	PORT_START("BRK")
	//PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("BREAK")              PORT_CODE(KEYCODE_F12)          PORT_CHAR(UCHAR_MAMEKEY(F12)) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(f256k_state::trigger_reset), 0)

	/* Keyboard columns 10 -> 12 are reserved for BBC Master */
	PORT_START("COL10")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("COL11")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("COL12")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)

	/* Keyboard columns 13 -> 14 are reserved for Torch */
	PORT_START("COL13")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("COL14")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)

	/* Keyboard column 15 not known to be used */
	PORT_START("COL15")
	PORT_BIT(0xff, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

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
