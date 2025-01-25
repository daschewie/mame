#include "emu.h"
#include "f256.h"

#include "cpu/m6502/w65c02.h"
#include "tiny_vicky.h"

/**
 *
 * F256K - WDC65C02 Processor running at 6.25MHz
 *    512KB RAM managed with slots of 8KB in MMU located at address $0000.  Slots 0 to $3F
 *    512KB Flash                                                           Slots $40 to $7F
 *    All I/O are mapped to slot 6 ($C000-$DFFF) - there are 4 I/O maps, switched using address $0001.
 *    Sound Chips: OPL3, PSG, SN74689, CODEC
 *    Keyboard: mechanical switch in a matrix - controlled by VIA6522 chip.
 *    Joysticks: 2 Atari-type ports and 2 S/NES ports
 *    Mouse: over PS/2 - which can also be used for PS/2 Keyboard
 *    IEC: Commodore Floppy Drive Controller
 *
 */

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
    , m_keyboard(*this, "ROW%u", 0)  // this, with the 8 array, requires 8 ROW of INPUTs
    , m_via6522_0(*this, "via6522_0")
	, m_via6522_1(*this, "via6522_1")

    // m_floating(*this, "floating"),
    // m_rs232(*this, RS232_TAG),
    // m_vhd_0(*this, "vhd0"),
    // m_vhd_1(*this, "vhd1"),
    // m_beckerport(*this, "dwsock"),
    // m_beckerportconfig(*this, BECKERPORT_TAG),


    //, m_irqs(*this, "irqs")
    , m_sn0(*this, "sn76489_0")
    , m_sn1(*this, "sn76489_1")
    , m_opl3(*this, "ymf262")
    , m_sid0(*this, "sid_0")
    , m_sid1(*this, "sid_1")
    , m_video(*this, "tiny_vicky")


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
    BQ4802(config, m_rtc, MASTER_CLOCK / 1000);  // RTC clock in kHz

    m_maincpu->set_addrmap(AS_PROGRAM, &f256_state::program_map);

    m_screen->set_refresh_hz(60); // Refresh rate (e.g., 60Hz)
    m_screen->set_size(800,525);
    m_screen->set_visarea(0, 639, 0, 479);
    m_screen->set_screen_update(m_video, FUNC(tiny_vicky_video_device::screen_update));
    TINY_VICKY(config, m_video, MASTER_CLOCK);
    m_video->irq_handler().set(FUNC(f256_state::sof_interrtupt));

    MOS6522(config, m_via6522_0, MASTER_CLOCK / 16);  // Atari Joysticks
	m_via6522_0->readpa_handler().set(FUNC(f256_state::via0_system_porta_r));
	m_via6522_0->readpb_handler().set(FUNC(f256_state::via0_system_portb_r));
	m_via6522_0->writepa_handler().set(FUNC(f256_state::via0_system_porta_w));
	m_via6522_0->writepb_handler().set(FUNC(f256_state::via0_system_portb_w));
    m_via6522_0->ca2_handler().set(FUNC(f256_state::via0_ca2_write));
    m_via6522_0->cb2_handler().set(FUNC(f256_state::via0_cb2_write));
    m_via6522_0->irq_handler().set(FUNC(f256_state::via0_interrupt));

    // to handle interrupts before the CPU, look into "input_merger_device"
	//

    MOS6522(config, m_via6522_1, XTAL(14'318'181)/14); //MASTER_CLOCK / 16);  // Keyboard
    m_via6522_1->readpa_handler().set(FUNC(f256_state::via1_system_porta_r));
	m_via6522_1->readpb_handler().set(FUNC(f256_state::via1_system_portb_r));
	m_via6522_1->writepa_handler().set(FUNC(f256_state::via1_system_porta_w));
	m_via6522_1->writepb_handler().set(FUNC(f256_state::via1_system_portb_w));
    m_via6522_1->ca2_handler().set(FUNC(f256_state::via1_ca2_write));
    m_via6522_1->cb2_handler().set(FUNC(f256_state::via1_cb2_write));
    m_via6522_1->irq_handler().set(FUNC(f256_state::via1_interrupt));

    SN76489(config, m_sn0, MASTER_CLOCK / 4);
    SN76489(config, m_sn1, MASTER_CLOCK / 4);
    YMF262(config, m_opl3, MASTER_CLOCK / 4);
    MOS6581(config, m_sid0, XTAL(14'318'181)/14);
    MOS6581(config, m_sid1, XTAL(14'318'181)/14);

    SPEAKER(config, "lspeaker").front_left();
    SPEAKER(config, "rspeaker").front_right();
    m_sn0->add_route(ALL_OUTPUTS, "lspeaker", 1.00);
    m_sn1->add_route(ALL_OUTPUTS, "rspeaker", 1.00);
    m_opl3->add_route(0, "lspeaker", 1.0);
	m_opl3->add_route(1, "rspeaker", 1.0);
	m_opl3->add_route(2, "lspeaker", 1.0);
	m_opl3->add_route(3, "rspeaker", 1.0);
    m_sid0->add_route(ALL_OUTPUTS, "lspeaker", 1.00);
    m_sid1->add_route(ALL_OUTPUTS, "rspeaker", 1.00);


    //set interrupt handler for the RTC
    m_rtc->int_handler().set(FUNC(f256_state::rtc_interrupt_handler));
}

f256_state::~f256_state()
{
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
                        if (adj_addr >= 0xD400 && adj_addr < 0xD580)
                        {
                            // SID
                        }
                        else if (adj_addr >= 0xD580 && adj_addr < 0xD583)
                        {
                            // OPL3
                        }
                        else if (adj_addr >= 0xD600 && adj_addr < 0xD620)
                        {
                            // PSG - left channel D600, right channel D610 - both D608
                        }
                        else if (adj_addr >= 0xD620 && adj_addr < 0xD630)
                        {
                            uint16_t base = adj_addr - 0xD620;
                            return m_codec[base];
                        }
                        else if (adj_addr >= 0xD630 && adj_addr < 0xD640)
                        {
                            // UART
                        }
                        else if (adj_addr >= 0xD640 && adj_addr < 0xD64F)
                        {
                            // PS2
                            switch(adj_addr - 0xD640)
                            {
                                case 0:
                                case 1:
                                    return m_ps2[adj_addr - 0xD640];
                                case 2:
                                {
                                    // Read from the keyboard fifo
                                    if (kbPacketCntr > kbQLen)
                                    {
                                        return 0;
                                    }
                                    uint8_t kbval = kbFifo[kbPacketCntr++];
                                    if (kbPacketCntr == kbQLen)
                                    {
                                        kbPacketCntr = 0;
                                        kbQLen = 0;
                                        memset(kbFifo, 0, 6);
                                    }
                                    return kbval;
                                }
                                case 3:
                                {
                                    // Read from the mouse fifo
                                    if (msPacketCntr> msQLen)
                                    {
                                        return 0;
                                    }
                                    uint8_t msval = msFifo[msPacketCntr++];
                                    if (msPacketCntr == msQLen)
                                    {
                                        msPacketCntr = 0;
                                        msQLen = 0;
                                        memset(msFifo, 0, 3);
                                    }
                                    return msval;
                                }
                                case 4:
                                    K_AK = false;
                                    M_AK = false;
                                    return ((K_AK ? 0x80:0) + (M_AK ? 0x20 : 0) + (msQLen == 0 ? 2 : 0) + (kbQLen == 0? 1 : 0));
                            }

                            return m_ps2[adj_addr - 0xD640];
                        }
                        else if (adj_addr >= 0xD650 && adj_addr < 0xD660)
                        {
                            // Timers
                        }
                        else if (adj_addr >= 0xD660 && adj_addr < 0xD670)
                        {
                            // Interrupt Registers
                            switch (adj_addr)
                            {
                                case 0xD660:
                                    return m_interrupt_reg[0]; // int_pending_0
                                case 0xD661:
                                    return m_interrupt_reg[1]; // int_pending_1
                                case 0xD662:
                                    return m_interrupt_reg[2]; // int_pending_2
                                case 0xD663:
                                    return 0;
                                case 0xD664:
                                    return 0; // int_polarity_0
                                case 0xD665:
                                    return 0; // int_polarity_1
                                case 0xD666:
                                    return 0; // int_polarity_2
                                case 0xD667:
                                    return 0;
                                case 0xD668:
                                    return m_interrupt_edge[0]; // int_edge_0
                                case 0xD669:
                                    return m_interrupt_edge[1]; // int_edge_1
                                case 0xD66A:
                                    return m_interrupt_edge[2]; // int_edge_2
                                case 0xD66B:
                                    return 0;
                                case 0xD66C:
                                    return m_interrupt_masks[0];
                                case 0xD66D:
                                    return m_interrupt_masks[1];
                                case 0xD66E:
                                    return m_interrupt_masks[2];
                                case 0xD66F:
                                    return 0;

                            }
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
                                    return 'A';
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
                        else if (adj_addr >= 0xD880 && adj_addr < 0xD8C0)
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
                            // switch (adj_addr)
                            // {
                            //     case 0xDC00:
                            //         return m_joy[1]->read();
                            //     case 0xDC01:
                            //         return m_joy[0]->read();
                            //     default:
                            //         return m_via6522_0->read(adj_addr - 0xDC00);
                            // }
                            break;
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
                        // Stick everything else in Vicky
                            // (adj_addr >= 0xC000 && adj_addr < 0xD400) ||  // gamma, mouse graphics, vicky registers, bitmaps, tiles
                            // (adj_addr >= 0xD800 && adj_addr < 0xD880) ||  // text colors
                            // (adj_addr >= 0xD900 && adj_addr < 0xDB00)     // sprite registers
                        else
                        {
                            return m_iopage0->read(adj_addr - 0xC000);
                        }
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
    uint8_t old, combo;

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
                        if (adj_addr >= 0xD400 && adj_addr < 0xD580)
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
                            uint16_t base = adj_addr-0xD620;
                            m_codec[base] = data;
                            // the program is telling the codec to start
                            if ((base == 2) && ((data & 1) == 1))
                            {
                                // start a timer that will reset the value to zero
                                this->machine().scheduler().timer_set(attotime::from_msec(100), timer_expired_delegate(FUNC(f256_state::codec_done), this), 1);   // timer_alloc(timer_expired_delegate(FUNC(f256_state::timer), this));
                            }
                        }
                        else if (adj_addr >= 0xD630 && adj_addr < 0xD640)
                        {
                            // UART
                        }
                        else if (adj_addr >= 0xD640 && adj_addr < 0xD64F)
                        {
                            // PS/2 Keyboard
                            uint16_t delta = adj_addr-0xD640;
                            m_ps2[delta] = data;
                            // Only addresses 0 and 1 are writable
                            if (delta == 0)
                            {
                                switch (data)
                                {
                                    case 0:
                                        if (isK_WR)
                                        {
                                            // write out the byte in data[1] to keyboard
                                            isK_WR = false;
                                            K_AK = true;
                                        }
                                        if (isM_WR)
                                        {
                                            // write out the byte in data[1] to mouse
                                            isM_WR = false;
                                            M_AK = true;
                                        }
                                        break;
                                    case 2:
                                        isK_WR = true;
                                        break;
                                    case 8:
                                        isM_WR = true;
                                        break;
                                    case 0x10: // clear keyboard fifo
                                        memset(kbFifo, 0, 6);
                                        break;
                                    case 0x20: // clear mouse fifo
                                        memset(msFifo, 0, 3);
                                        break;
                                }
                            }
                        }
                        else if (adj_addr >= 0xD650 && adj_addr < 0xD660)
                        {
                            // Timers
                        }
                        else if (adj_addr >= 0xD660 && adj_addr < 0xD670)
                        {
                            // Interrupt Registers
                            // Interrupt Registers
                            switch (adj_addr)
                            {
                                case 0xD660:
                                    // int_pending_0
                                    old = m_interrupt_reg[0];
                                    combo = old & data;
                                    if (combo > 0)
                                    {
                                        m_interrupt_reg[0] = old & ~combo;
                                    }

                                    break;
                                case 0xD661:
                                    // int_pending_1
                                    old = m_interrupt_reg[1];
                                    combo = old & data;
                                    if (combo > 0)
                                    {
                                        m_interrupt_reg[1] = old & ~combo;
                                    }
                                    break;
                                case 0xD662:
                                    // int_pending_2
                                    old = m_interrupt_reg[2];
                                    combo = old & data;
                                    if (combo > 0)
                                    {
                                        m_interrupt_reg[2] = old & ~combo;
                                    }
                                    break;
                                case 0xD663:
                                    break;
                                case 0xD664:
                                    // int_polarity_0
                                    break;
                                case 0xD665:
                                    // int_polarity_1
                                    break;
                                case 0xD666:
                                    // int_polarity_2
                                    break;
                                case 0xD667:
                                    break;
                                case 0xD668:
                                    // int_edge_0
                                    m_interrupt_edge[0] = data;
                                    break;
                                case 0xD669:
                                    // int_edge_1
                                    m_interrupt_edge[1] = data;
                                    break;
                                case 0xD66A:
                                    // int_edge_2
                                    m_interrupt_edge[2] = data;
                                    break;
                                case 0xD66B:
                                    break;
                                case 0xD66C:
                                    m_interrupt_masks[0] = data;
                                    break;
                                case 0xD66D:
                                    m_interrupt_masks[1] = data;
                                    break;
                                case 0xD66E:
                                    m_interrupt_masks[2] = data;
                                    break;
                                case 0xD66F:
                                    break;

                            }
                            if (m_interrupt_reg[0] == 0 && m_interrupt_reg[1] == 0 && m_interrupt_reg[2] == 0)
                            {
                                m_maincpu->set_input_line(M6502_IRQ_LINE, CLEAR_LINE);
                            }
                        }
                        else if (adj_addr >= 0xD690 && adj_addr < 0xD6A0)
                        {
                            // RTC
                            m_rtc->write(adj_addr - 0xDC90, data);
                        }
                        else if (adj_addr >= 0xD880 && adj_addr < 0xD8C0)
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
                        // stick everything else in Vicky
                            // (adj_addr >= 0xC000 && adj_addr < 0xD400) ||  // gamma, mouse graphics, vicky registers, bitmaps, tiles
                            // (adj_addr >= 0xD800 && adj_addr < 0xD880) ||  // text colors
                            // (adj_addr >= 0xD900 && adj_addr < 0xDB00)     // sprite registers
                        else
                        {
                            m_iopage0->write(adj_addr - 0xC000, data);
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
void f256_state::codec_done(s32 param)
{
    m_codec[2] = 0;
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
    m_video->set_videoram(m_ram->pointer(), m_iopage0->pointer(), m_iopage1->pointer(), m_iopage2->pointer(), m_iopage3->pointer());
    m_video->start();

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

    // Initialize the VIA1
    m_via6522_1->write(via6522_device::VIA_DDRB, 0);     // DDRB
    m_via6522_1->write(via6522_device::VIA_DDRA, 0);     // DDRA
}

//-------------------------------------------------
//  device_reset
//-------------------------------------------------
void f256_state::device_reset()
{
	driver_device::device_reset();
    reset_mmu();
    m_via6522_0->reset();
	m_via6522_1->reset();
    m_sid0->reset();
    m_sid1->reset();
    m_sn0->reset();
    m_sn1->reset();
    m_opl3->reset();
}

//-------------------------------------------------
//  Interrupts
//-------------------------------------------------
void f256_state::sof_interrtupt(int state)
{
    if (state && ((m_interrupt_masks[1] & 0x01) == 0))
    {
        m_interrupt_reg[0] |= 0x01;
        m_maincpu->set_input_line(M6502_IRQ_LINE, state);
    }
}
void f256_state::rtc_interrupt_handler(int state)
{
    if (state && ((m_interrupt_masks[1] & 0x10) == 0))
    {
        m_interrupt_reg[1] |= 0x10;
        m_maincpu->set_input_line(M6502_IRQ_LINE, state);
    }
}

void f256_state::via0_interrupt(int state)
{
    // if a joystick button is pressed, set the VIA0 interrupt if the mask allows if
    if (state && ((m_interrupt_masks[1] & 0x20) == 0))
    {
        m_interrupt_reg[1] |= 0x20;
        m_maincpu->set_input_line(M6502_IRQ_LINE, state);
    }
}
void f256_state::via1_interrupt(int state)
{
    logerror("VIA1 INTERRUPT: %02X\n", state);
    // if a keyboard button is pressed, set the VIA1 interrupt if the mask allows if
    if (state && ((m_interrupt_masks[1] & 0x40) == 0))
    {
        m_interrupt_reg[1] |= 0x40;
        m_maincpu->set_input_line(M6502_IRQ_LINE, state);
    }
}

//-------------------------------------------------
//  VIA0 - JOYSTICK
//-------------------------------------------------
u8 f256_state::via0_system_porta_r()
{
    u8 data = ioport("JOY2")->read();
    //logerror("VIA #0 Port A Read ioport JOY2: %02X\n", data);
    return data;
}
void f256_state::via0_system_porta_w(u8 data)
{
    //logerror("VIA #0 Port A Write: %02X\n", data);
    // writing should only be done if DDR allows it
}
u8 f256_state::via0_system_portb_r()
{
    u8 data = ioport("JOY1")->read();
    //logerror("VIA #0 Port A Read ioport JOY1: %02X\n", data);
    return data;
}
void f256_state::via0_system_portb_w(u8 data)
{
    //logerror("VIA #0 Port B Write: %02X\n", data);
    // writing should only be done if DDR allows it
}
void f256_state::via0_ca2_write(u8 value)
{
    //logerror("Write to VIA0 - CA2 %02X\n", value);
    m_via6522_0->write_ca2(value);
}
void f256_state::via0_cb2_write(u8 value)
{
    //logerror("Write to VIA0 - CB2 %02X\n", value);
    m_via6522_0->write_cb2(value);
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

//-------------------------------------------------
//  VIA1 - F256K Keyboard
//-------------------------------------------------
u8 f256_state::via1_system_porta_r()
{
    // int r[8] = {};
    // int first_non_zero_row = -1;
    // for (int row = 0; row < m_keyboard.size(); row++)
    // {
    //     r[row] = ~m_keyboard[row]->read() & ((row==0 || row==6) ? 0x1FF : 0xFF);
    //     if (r[row] != 0)
    //     {
    //         first_non_zero_row = row;
    //     }
    // }
    // //m_via_port_b = first_non_zero_row != -1 ? ~(1 << first_non_zero_row) : 0xFF;
    // m_via_port_a = first_non_zero_row != -1 ? ~r[first_non_zero_row] : 0xFF;
    //logerror("VIA1-A READ: R0: %02X, R1: %02X R2: %02X, R3: %02X R4: %02X, R5: %02X R6: %02X, R7: %02X A: %02X, B: %02X\n",
    //  r[0], r[1],
    //  r[2], r[3],
    //  r[4], r[5],
    //  r[6], r[7],
    //  m_via_port_a, m_via_port_b);
    // m_via6522_1->write_pb(m_via_port_b);
    return m_via_port_a;
}
u8 f256_state::via1_system_portb_r()
{
    // for (int row = 0; row < m_keyboard.size(); row++)
    // {
    //     if (BIT(m_via_port_a, row) == 0)
    //     {
    //         uint16_t v = ~m_keyboard[row]->read() & ((row == 0 || row == 6) ? 0x1FF: 0xFF);
    //         if (v != 0)
    //         {
    //             m_via_port_b = m_keyboard[row]->read();
    //             break;
    //         }

    //     }
    // }
    m_via_port_b = 0xff;

    for (int i = 0; i < 8; i++)
    {
        if (BIT(m_via_port_a,i) == 0)
        {
            m_via_port_b &= m_keyboard[i]->read();
        }
    }
    logerror("\t\t\tRead from VIA1 PORT B: %02X\n", m_via_port_b);
    //m_via6522_1->write_pb(m_via_port_b);
    return m_via_port_b;
}
// Read keyboard as rows
void f256_state::via1_system_porta_w(u8 data)
{
    m_via_port_a = data;
    m_via6522_1->write_pa(data);
    logerror("Write to VIA1 PORT A: %02X\n", data);
}
// Read keyboard as columns
void f256_state::via1_system_portb_w(u8 data)
{
    m_via_port_b = data;
    m_via6522_1->write_pb(data);
    logerror("Write to VIA1 PORT B: %02X\n", data);
}
void f256_state::via1_ca2_write(u8 value)
{
    logerror("Write to VIA1 - CA2 %02X\n", value);
    m_via6522_1->write_ca2(value);
}
void f256_state::via1_cb2_write(u8 value)
{
    logerror("Write to VIA1 - CB2 %02X\n",value);
    m_via6522_1->write_cb2(value);
}

/*  Port                                        Key description                 Emulated key                    Natural key         Shift 1         Shift 2 (Ctrl) */

static INPUT_PORTS_START(f256k)
    PORT_INCLUDE( f256k_joysticks )

	PORT_START("ROW0")
    PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("DEL") PORT_CODE(KEYCODE_DEL)
    PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ENTER") PORT_CODE(KEYCODE_ENTER)
    PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_LEFT) PORT_CODE(KEYCODE_LEFT)
    PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F7") PORT_CODE(KEYCODE_F7)
    PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1)
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3)
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F5") PORT_CODE(KEYCODE_F5)
    PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_UP) PORT_CODE(KEYCODE_UP)
    PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_DOWN) PORT_CODE(KEYCODE_DOWN)

    PORT_START("ROW1")
    PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("3") PORT_CODE(KEYCODE_3)
    PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("W") PORT_CODE(KEYCODE_W)
    PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("A") PORT_CODE(KEYCODE_A)
    PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("4") PORT_CODE(KEYCODE_4)
    PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Z") PORT_CODE(KEYCODE_Z)
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("S") PORT_CODE(KEYCODE_S)
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("E") PORT_CODE(KEYCODE_E)
    PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("L SHIFT") PORT_CODE(KEYCODE_LSHIFT)

    PORT_START("ROW2")
    PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("5") PORT_CODE(KEYCODE_5)
    PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("R") PORT_CODE(KEYCODE_R)
    PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("D") PORT_CODE(KEYCODE_D)
    PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("6") PORT_CODE(KEYCODE_6)
    PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("C") PORT_CODE(KEYCODE_C)
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("F") PORT_CODE(KEYCODE_F)
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("T") PORT_CODE(KEYCODE_T)
    PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("X") PORT_CODE(KEYCODE_X)

    PORT_START("ROW3")
    PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("7") PORT_CODE(KEYCODE_7)
    PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Y") PORT_CODE(KEYCODE_Y)
    PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("G") PORT_CODE(KEYCODE_G)
    PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("8") PORT_CODE(KEYCODE_8)
    PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("B") PORT_CODE(KEYCODE_B)
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("H") PORT_CODE(KEYCODE_H)
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("U") PORT_CODE(KEYCODE_U)
    PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("V") PORT_CODE(KEYCODE_V)

    PORT_START("ROW4")
    PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("9") PORT_CODE(KEYCODE_9)
    PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("I") PORT_CODE(KEYCODE_I)
    PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("J") PORT_CODE(KEYCODE_J)
    PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("0") PORT_CODE(KEYCODE_0)
    PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("M") PORT_CODE(KEYCODE_M)
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("K") PORT_CODE(KEYCODE_K)
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("O") PORT_CODE(KEYCODE_O)
    PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("N") PORT_CODE(KEYCODE_N)

    PORT_START("ROW5")
    PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("-") PORT_CODE(KEYCODE_MINUS)
    PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("P") PORT_CODE(KEYCODE_P)
    PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("L") PORT_CODE(KEYCODE_L)
    PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CAPS LOCK") PORT_CODE(KEYCODE_CAPSLOCK)
    PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(".") PORT_CODE(KEYCODE_STOP)
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(";") PORT_CODE(KEYCODE_COLON)
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("[") PORT_CODE(KEYCODE_OPENBRACE)
    PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(",") PORT_CODE(KEYCODE_COMMA)

    PORT_START("ROW6")
    PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("=") PORT_CODE(KEYCODE_EQUALS)
    PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("]") PORT_CODE(KEYCODE_CLOSEBRACE)
    PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("'") PORT_CODE(KEYCODE_QUOTE)
    PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("HOME") PORT_CODE(KEYCODE_HOME)
    PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("R SHIFT") PORT_CODE(KEYCODE_C)
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("ALT") PORT_CODE(KEYCODE_LALT) PORT_CODE(KEYCODE_RALT)
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("TAB") PORT_CODE(KEYCODE_TAB)
    PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("/") PORT_CODE(KEYCODE_SLASH)
    PORT_BIT(0x100, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME(UTF8_RIGHT) PORT_CODE(KEYCODE_RIGHT)

    PORT_START("ROW7")
    PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("1") PORT_CODE(KEYCODE_1)
    PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("BKSP") PORT_CODE(KEYCODE_BACKSPACE)
    PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("CTRL") PORT_CODE(KEYCODE_LCONTROL)
    PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("2") PORT_CODE(KEYCODE_2)
    PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("SPACE") PORT_CODE(KEYCODE_SPACE)
    PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("FOENIX") PORT_CODE(KEYCODE_LWIN)
    PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("Q") PORT_CODE(KEYCODE_Q)
    PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYBOARD) PORT_NAME("RUN/STOP") PORT_CODE(KEYCODE_STOP)
INPUT_PORTS_END

ROM_START(f256k)
	ROM_REGION(0x10'0000,ROM_TAG,0)
    // Offsets are based on the REGION base address - offset by 0x10 because of map.
    ROM_LOAD("xdev.bin",   0x08'2000, 0x2000, CRC(5cee0cb0) SHA1(a5fb10ad914069f506847150bdd387371e73f1de))
    ROM_LOAD("sb01.bin",   0x08'4000, 0x2000, CRC(21f06e73) SHA1(bbeefb52d4b126b61367169c21599180f3358af7))
    ROM_LOAD("sb02.bin",   0x08'6000, 0x2000, CRC(6ed611b9) SHA1(4a03aa286f6274e6974a3cecdedad651a58f5fb1))
    ROM_LOAD("sb03.bin",   0x08'8000, 0x2000, CRC(653f849d) SHA1(65942d98f26b86499e6359170aa2d0c6e16124ff))
    ROM_LOAD("sb04.bin",   0x08'A000, 0x2000, CRC(f4aa6049) SHA1(11f02fee6ec412f0c96b27b0b149f72cf1770d15))
    ROM_LOAD("dos.bin",    0x08'C000, 0x2000, CRC(f3673c4e) SHA1(9c6b70067d7195d4a6bbd7f379b8e5382bf8cc1b))
    ROM_LOAD("pexec.bin",  0x08'E000, 0x2000, CRC(937c1374) SHA1(40566a51d2ef7321a42fe926b03dee3571c78202))
	ROM_LOAD("3b.bin",     0x0F'6000, 0x2000, CRC(00880c2a) SHA1(bc89208e94674b8157ecb8107cccb6136b03896c))
    ROM_LOAD("3c.bin",     0x0F'8000, 0x2000, CRC(9a67f0d7) SHA1(16152cb6045cad4c9fcd773a961898d57d596c3a))
    ROM_LOAD("3d.bin",     0x0F'A000, 0x2000, CRC(069af1da) SHA1(72101736144cd545484f34393ee0d8df06b1eedb))
    ROM_LOAD("3e.bin",     0x0F'C000, 0x2000, CRC(8db5d29b) SHA1(6d2f4b43bd3ae211bc15a36212fb6c22090edbf7))
    ROM_LOAD("3f.bin",     0x0F'E000, 0x2000, CRC(44eb158f) SHA1(b56eb6288e8f05084726d0ab4ee784bbc32d91f8))

    ROM_REGION(0x0800,FONT_TAG,0)
    ROM_LOAD("f256jr_font_micah_jan25th.bin", 0x0000, 0x0800, CRC(6d66da85) SHA1(377dc27ff3a4ae2d80d740b2d16373f8e639eef6))
ROM_END

//    YEAR  NAME   PARENT COMPAT  MACHINE    INPUT    CLASS        INIT        COMPANY              FULLNAME                        FLAGS
COMP( 2024, f256k,    0,      0,    f256k,   f256k,   f256_state, empty_init, "Stefany Allaire", "F256K 8-bit Retro System",      MACHINE_IS_INCOMPLETE  )
