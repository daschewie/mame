#include "emu.h"
#include "tiny_vicky.h"

#include "machine/ram.h"
#include "screen.h"

DEFINE_DEVICE_TYPE(TINY_VICKY, tiny_vicky_video_device, "tiny_vicky", "F256K Tiny Vicky")

tiny_vicky_video_device::tiny_vicky_video_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock):
    device_t(mconfig, type, tag, owner, clock)
    , m_sof_irq_handler(*this)
    , m_sol_irq_handler(*this)
{
}

tiny_vicky_video_device::tiny_vicky_video_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock):
    tiny_vicky_video_device(mconfig, TINY_VICKY, tag, owner, clock)
{
}

rgb_t tiny_vicky_video_device::get_text_lut(uint8_t color_index, bool fg, bool gamma)
{
    uint8_t red =   iopage0_ptr[(fg ? 0x1800 : 0x1840) + color_index * 4 + 2];
    uint8_t green = iopage0_ptr[(fg ? 0x1800 : 0x1840) + color_index * 4 + 1];
    uint8_t blue =  iopage0_ptr[(fg ? 0x1800 : 0x1840) + color_index * 4 ];
    if (gamma)
    {
        blue = iopage0_ptr[blue];
        green = iopage0_ptr[0x400 + green];
        red = iopage0_ptr[0x800 + red];
    }
    return rgb_t(red, green, blue);
}

rgb_t tiny_vicky_video_device::get_lut_value(uint8_t lut_index, uint8_t pix_val, bool gamma)
{
    int lutAddress = 0xD000 - 0xC000 + (lut_index * 256 + pix_val) * 4;
    if (!gamma)
    {
        return rgb_t(iopage1_ptr[lutAddress + 2],iopage1_ptr[lutAddress + 1],iopage1_ptr[lutAddress]);
    }
    else
    {
        return rgb_t(iopage0_ptr[0x800 + iopage1_ptr[lutAddress + 2]],
                    iopage0_ptr[0x400 + iopage1_ptr[lutAddress + 1]],
                    iopage0_ptr[iopage1_ptr[lutAddress]]);
    }
}

uint16_t tiny_vicky_video_device::line()
{
    return m_line;
}
// This is always 0 - too difficult to do this now
uint16_t tiny_vicky_video_device::column()
{
    return m_column;
}

uint32_t tiny_vicky_video_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
    if (running && iopage0_ptr)
    {
        uint8_t mcr = iopage0_ptr[0x1000];
        uint8_t mcr_h = iopage0_ptr[0x1001];
        cursor_counter++;
        if (cursor_counter > cursor_flash_rate *2)
        {
            cursor_counter = 0;
        }
        // if MCR =0 or MCR bit 7 is set, then video is disabled
        if (mcr != 0 && (mcr & 0x80) == 0)
        {
            // TODO: generate start of frame (SOF) interrupt
            m_sof_irq_handler(1);
            uint8_t border_reg = iopage0_ptr[0x1004];
            bool display_border = (border_reg & 0x1) > 0;
            bool enable_gamma = (mcr & 0x40) > 0;
            bool enable_graphics = (mcr & 0x4) > 0;
            uint16_t lines = (mcr_h & 1) == 0 ? 480 : 400;

            uint8_t border_x = 0;
            uint8_t border_y = 0;
            rgb_t border_color = rgb_t();
            if (display_border) {
                border_x = iopage0_ptr[0x1008];
                border_y = iopage0_ptr[0x1009];
            }

            for (uint16_t y = 0; y < lines; y++)
            {
                m_line = y;
                // Check the Sart of Line registers
                uint8_t sol_reg = iopage0_ptr[0x1018];
                // 12-bit line
                uint16_t sol_line = iopage0_ptr[0x1018] + ((iopage0_ptr[0x101A] & 0xF) << 8);
                if ((sol_reg & 1) != 0 && y == sol_line)
                {
                    // raise an interrupt
                    m_sol_irq_handler(1);
                }
                // border color can change during painting the screen
                if (display_border)
                {
                    if (!enable_gamma)
                    {
                        border_color = rgb_t(iopage0_ptr[0x1007], iopage0_ptr[0x1006], iopage0_ptr[0x1005]);
                    }
                    else
                    {
                        border_color = rgb_t(iopage0_ptr[0x800 + iopage0_ptr[0x1007]],
                                             iopage0_ptr[0x400 + iopage0_ptr[0x1006]],
                                             iopage0_ptr[iopage0_ptr[0x1005]]);
                    }
                }

                if (y < border_y || y >= lines - border_y)
                {
                    for (int x = 0; x < 640; x++)
                    {
                        bitmap.pix(y, x) = border_color;
                    }
                }
                else
                {
                    rgb_t background_color = rgb_t();
                    if (enable_graphics)
                    {
                        if (!enable_gamma)
                        {
                            background_color = rgb_t(iopage0_ptr[0x100F], iopage0_ptr[0x100E], iopage0_ptr[0x100D]);
                        }
                        else
                        {
                            background_color = rgb_t(iopage0_ptr[0x800 + iopage0_ptr[0x100F]],
                                                    iopage0_ptr[0x400 + iopage0_ptr[0x100E]],
                                                    iopage0_ptr[iopage0_ptr[0x100D]]);
                        }
                    }
                    // draw the border or background
                    for (int x = 0; x < 640; x++)
                    {
                        bitmap.pix(y, x) = x < border_x || x >= 640 - border_x ? border_color : background_color;
                    }

                    if (enable_graphics)
                    {
                        // Tiny Vicky Layers for Bitmaps, Tilemaps and sprites
                        uint8_t LayerMgr0 = iopage0_ptr[0xD002 - 0xC000] & 0x7;
                        uint8_t LayerMgr1 = iopage0_ptr[0xD002 - 0xC000] >> 4;
                        uint8_t LayerMgr2 = iopage0_ptr[0xD003 - 0xC000] & 0x7;

                        // draw layers starting from the back
                        if ((mcr & 0x20) != 0)
                        {
                            draw_sprites(bitmap, enable_gamma, 3, display_border, border_x, border_y, y, (uint16_t)640, lines / 2);
                        }
                        if ((mcr & 0x8) != 0 && LayerMgr2 < 3)
                        {
                            draw_bitmap(bitmap, enable_gamma, LayerMgr2, display_border, background_color, border_x, border_y, y, (uint16_t)640);

                        }
                        if ((mcr & 0x10) != 0 && (LayerMgr2 > 3 && LayerMgr2 < 7))
                        {
                            draw_tiles(bitmap, enable_gamma, LayerMgr2 & 3, display_border, border_x, y, (uint16_t)640);
                        }
                        if ((mcr & 0x20) != 0)
                        {
                            draw_sprites(bitmap, enable_gamma, 2, display_border, border_x, border_y, y, (uint16_t)640, lines/2);
                        }
                        if ((mcr & 0x8) != 0 && LayerMgr1 < 3)
                        {
                            draw_bitmap(bitmap, enable_gamma, LayerMgr1, display_border, background_color, border_x, border_y, y, (uint16_t)640);
                        }
                        if ((mcr & 0x10) != 0 && (LayerMgr1 > 3 && LayerMgr1 < 7))
                        {
                            draw_tiles(bitmap, enable_gamma, LayerMgr1 & 3, display_border, border_x, y, (uint16_t)640);
                        }
                        if ((mcr & 0x20) != 0)
                        {
                            draw_sprites(bitmap, enable_gamma, 1, display_border, border_x, border_y, y, (uint16_t)640, lines / 2);
                        }
                        if ((mcr & 0x8) != 0 && LayerMgr0 < 3)
                        {
                            draw_bitmap(bitmap, enable_gamma, LayerMgr0, display_border, background_color, border_x, border_y, y, (uint16_t)640);
                        }
                        if ((mcr & 0x10) != 0 && (LayerMgr0 > 3 && LayerMgr0 < 7))
                        {
                            draw_tiles(bitmap, enable_gamma, LayerMgr0 & 3, display_border, border_x, y, (uint16_t)640);
                        }
                        if ((mcr & 0x20) != 0)
                        {
                            draw_sprites(bitmap, enable_gamma, 0, display_border, border_x, border_y, y, (uint16_t)640, lines/ 2);
                        }
                    }
                    // Only display text in these cases
                    if ((mcr & 0x7) == 1 || (mcr & 0x7) == 3 || (mcr & 0x7) == 7)
                    {
                        draw_text(bitmap, mcr, enable_gamma, border_x, border_y, y, (uint16_t)640, (uint16_t)lines);
                    }
                }
            }
        }
    }

    return 0;
}


void tiny_vicky_video_device::draw_text(bitmap_rgb32 &bitmap, uint8_t mcr, bool enable_gamma, uint8_t brd_x, uint8_t brd_y, uint16_t line, uint16_t x_res, uint16_t y_res)
{
    bool overlay = (mcr & 0x2) != 0;
    uint8_t mcrh = iopage0_ptr[0x1001] & 0x3F;
    bool double_x = (mcrh & 0x2) != 0;
    bool double_y = (mcrh & 0x4) != 0;
    bool use_font1 = (mcrh & 0x20) != 0;
    bool overlay_font = (mcrh & 0x10) != 0;
    int txt_line = ((double_y ? line / 2 : line) - brd_y) / CHAR_HEIGHT;
    // Each character is defined by 8 bytes
    int font_line = ((double_y ? line / 2 : line) - brd_y) % CHAR_HEIGHT;
    int txt_cols = double_x ? 40 : 80;

    // do cursor stuff
    int cursor_x = iopage0_ptr[0x1014] + (iopage0_ptr[0x1015] << 8);
    int cursor_y = iopage0_ptr[0x1016] + (iopage0_ptr[0x1017] << 8);

    // TODO: enabling/disabling cursor and flashing should be handled somewhere else... maybe in the top screen_update function.
    bool enable_cursor = (iopage0_ptr[0x1010] & 0x1) > 0;
    enable_cursor_flash = (iopage0_ptr[0x1010] & 0x8) == 0;
    // flash rate is the count of screen updates
    // 1s   = 60 ==> 00
    // 0.5s = 30 ==> 01
    // 0.25s= 15 ==> 10
    // 0.20s= 12 ==> 11
    cursor_flash_rate = 60;
    switch ((iopage0_ptr[0x1010] & 6) >> 1)
    {
        case 1:
            cursor_flash_rate = 30;
            break;
        case 2:
            cursor_flash_rate = 15;
            break;
        case 3:
            cursor_flash_rate = 12;
            break;
    }
    int screen_x = brd_x;
    // I'm assuming that if enable_cursor_flash is 0, then the cursor is always visible
    cursor_visible = enable_cursor && (enable_cursor_flash && (cursor_counter < cursor_flash_rate));
    for (int col = 0; col < txt_cols; col++)
    {
        int x = col * CHAR_WIDTH;
        if (x + brd_x > x_res - 1 - brd_x)
        {
            continue;
        }
        int offset = 0;
        if (col < (double_x ? 40 : 80))
        {
            // offset is always based on 80 columns
            offset = 80 * txt_line + col;
        }
        // Each character will have foreground and background colors
        uint8_t character = iopage2_ptr[offset];
        uint8_t color = iopage3_ptr[offset];

        // Display the cursor - this replaces the text character
        if (cursor_x == col && cursor_y == txt_line && cursor_visible)
        {
            character = iopage0_ptr[0x1012];
        }

        uint8_t fg_color_index = (color & 0xF0) >> 4;
        uint8_t bg_color_index = (color & 0x0F);

        rgb_t fg_color = get_text_lut(fg_color_index, true, enable_gamma);
        rgb_t bg_color = get_text_lut(bg_color_index, false, enable_gamma);

        uint8_t value = iopage1_ptr[(use_font1 ? 0x800 : 0) + character * 8 + font_line];

        // For each bit in the font, set the foreground color - if the bit is 0 and overlay is set, skip it (keep the background)
        for (int b = 0x80; b > 0; b >>= 1)
        {
            if (double_x)
            {
                if ((value & b) != 0)
                {
                    bitmap.pix(line, screen_x) = fg_color;
                    bitmap.pix(line, screen_x+1) = fg_color;
                }
                else if (!overlay || (overlay_font && (bg_color == 0)))
                {
                    bitmap.pix(line, screen_x) = bg_color;
                    bitmap.pix(line, screen_x+1) = bg_color;
                }
                screen_x += 2;
            }
            else
            {
                if ((value & b) != 0)
                {
                    bitmap.pix(line, screen_x) = fg_color;
                }
                else if (!overlay || (overlay_font && (bg_color != 0)))
                {
                    bitmap.pix(line, screen_x) = bg_color;
                }
                screen_x++;
            }
        }
    }
}
void tiny_vicky_video_device::device_start()
{
}
void tiny_vicky_video_device::device_reset()
{

}
void tiny_vicky_video_device::draw_bitmap(bitmap_rgb32 &bitmap, bool enable_gamma, uint8_t layer, bool bkgrnd, rgb_t bgndColor, uint8_t borderXSize, uint8_t borderYSize, uint16_t line, uint16_t width)
{
    uint8_t reg = iopage0_ptr[(0xD100 - 0xC000) + layer * 8];
    // check if the bitmap is enabled
    if ((reg & 0x01) == 0)
    {
        return;
    }
    uint8_t lut_index = (reg >> 1) & 7;  // 8 possible LUTs

    int bitmapAddress = (iopage0_ptr[(0xD101 - 0xC000) + layer * 8] +
                         (iopage0_ptr[(0xD102 - 0xC000) + layer * 8] << 8) +
                         (iopage0_ptr[(0xD103 - 0xC000) + layer * 8] << 16)
                        ) & 0x3F'FFFF;

    rgb_t color_val = 0;
    int offsetAddress = bitmapAddress + (line/2) * width/2;
    uint8_t pix_val = 0;

    for (int col = borderXSize; col < width - borderXSize; col += 2)
    {
        pix_val = videoram_ptr[offsetAddress + col/2 + 1];
        if (pix_val != 0)
        {
            color_val = get_lut_value(lut_index, pix_val, enable_gamma);
            bitmap.pix(line, col) = color_val;
            bitmap.pix(line, col + 1) = color_val;
        }
    }
}


void tiny_vicky_video_device::draw_sprites(bitmap_rgb32 &bitmap, bool enable_gamma, uint8_t layer, bool bkgrnd, uint8_t borderXSize, uint8_t borderYSize, uint16_t line, uint16_t width, uint16_t height)
{
    // There are 64 possible sprites to choose from.
    for (int s = 63; s > -1; s--)
    {
        int addr_sprite = 0xD900 - 0xC000 + s * 8;
        uint8_t reg = iopage0_ptr[addr_sprite];
        // if the set is not enabled, we're done.
        uint8_t sprite_layer = (reg & 0x18) >> 3;
        // if the sprite is enabled and the layer matches, then check the line
        if ((reg & 1) != 0 && layer == sprite_layer)
        {
            uint8_t sprite_size = 32;
            switch ((reg & 0x60) >> 5)
            {
                case 1:
                    sprite_size = 24;
                    break;
                case 2:
                    sprite_size = 16;
                    break;
                case 3:
                    sprite_size = 8;
                    break;
            }
            int posY = iopage0_ptr[addr_sprite + 6] + (iopage0_ptr[addr_sprite + 7] << 8) - 32;
            int actualLine = line / 2;
            if ((actualLine >= posY && actualLine < posY + sprite_size))
            {
                // TODO Fix this when Vicky II fixes the LUT issue
                uint8_t lut_index = ((reg & 6) >> 1);

                //int lut_address = 0xD000 - 0xC000 + lut_index * 0x400;
                //bool striding = (reg & 0x80) == 0x80;

                int sprite_address = (iopage0_ptr[addr_sprite + 1] +
                                     (iopage0_ptr[addr_sprite + 2] << 8) +
                                     (iopage0_ptr[addr_sprite + 3] << 16)) & 0x3F'FFFF;
                int posX = iopage0_ptr[addr_sprite + 4] + (iopage0_ptr[addr_sprite + 5] << 8) - 32;
                posX *= 2;

                if (posX >= width || posY >= height || (posX + sprite_size) < 0 || (posY + sprite_size) < 0)
                {
                    continue;
                }
                int sprite_width = sprite_size;
                int xOffset = 0;
                // Check for sprite bleeding on the left-hand-side
                if (posX < borderXSize)
                {
                    xOffset = borderXSize - posX;
                    posX = borderXSize;
                    sprite_width = sprite_size - xOffset;
                    if (sprite_width == 0)
                    {
                        continue;
                    }
                }
                // Check for sprite bleeding on the right-hand side
                if (posX + sprite_size > width - borderXSize)
                {
                    sprite_width = width - borderXSize - posX;
                    if (sprite_width == 0)
                    {
                        continue;
                    }
                }

                rgb_t clrVal = 0;
                uint8_t pixVal = 0;

                int sline = actualLine - posY;
                int cols = sprite_size;
                if (posX + sprite_size*2 >= width - borderXSize)
                {
                    cols = width - borderXSize - posX;
                    cols /= 2;
                }
                for (int col = xOffset; col < xOffset + cols; col++)
                {
                    // Lookup the pixel in the tileset - if the value is 0, it's transparent
                    pixVal = videoram_ptr[sprite_address + col + sline * sprite_size];
                    if (pixVal != 0)
                    {
                        clrVal = get_lut_value(lut_index, pixVal, enable_gamma);
                        bitmap.pix(line, (col * 2) - xOffset + posX) = clrVal;
                        bitmap.pix(line, (col * 2) + 1 - xOffset + posX) = clrVal;
                    }
                }
            }
        }
    }
}

void tiny_vicky_video_device::draw_tiles(bitmap_rgb32 &bitmap, bool enable_gamma, uint8_t layer, bool bkgrnd, uint8_t borderXSize, uint16_t line, uint16_t width)
{
    // There are four possible tilemaps to choose from
    int addr_tile_addr = 0xD200 - 0xC000 + layer * 12;
    int reg = iopage0_ptr[addr_tile_addr];
    // if the set is not enabled, we're done.
    if ((reg & 0x01) == 00)
    {
        return;
    }
    bool smallTiles = (reg & 0x10) > 0;

    int tileSize = (smallTiles ? 8 : 16);
    int strideLine = tileSize * 16;
    uint8_t scrollMask = smallTiles ? 0xF : 0xF;  // Tiny Vicky bug: this should be 0xE

    int tilemapWidth = (iopage0_ptr[addr_tile_addr + 4] + (iopage0_ptr[addr_tile_addr + 5] << 8)) & 0x3FF;   // 10 bits
    //int tilemapHeight = VICKY.ReadWord(addrTileCtrlReg + 6) & 0x3FF;  // 10 bits
    int tilemapAddress = (iopage0_ptr[addr_tile_addr + 1] + (iopage0_ptr[addr_tile_addr + 2]  << 8) + (iopage0_ptr[addr_tile_addr + 3] << 16)) & 0x3F'FFFF;

    // the tilemapWindowX is 10 bits and the scrollX is the lower 4 bits.  The IDE combines them.
    int tilemapWindowX = iopage0_ptr[addr_tile_addr + 8] + (iopage0_ptr[addr_tile_addr + 9] << 8);
    uint8_t scrollX = (tilemapWindowX & scrollMask) & scrollMask;
    // the tilemapWindowY is 10 bits and the scrollY is the lower 4 bits.  The IDE combines them.
    int tilemapWindowY = iopage0_ptr[addr_tile_addr + 10] + (iopage0_ptr[addr_tile_addr + 11] << 8);
    uint8_t scrollY = (tilemapWindowY & scrollMask) & scrollMask;
    if (smallTiles)
    {
        tilemapWindowX = ((tilemapWindowX & 0x3FF0)  >> 1) + scrollX + 8;
        tilemapWindowY = ((tilemapWindowY & 0x3FF0) >> 1) + scrollY;
    }
    else
    {
        tilemapWindowX = (tilemapWindowX & 0x3FF0) + scrollX;
        tilemapWindowY = (tilemapWindowY & 0x3FF0) + scrollY;
    }
    int tileXOffset = tilemapWindowX % tileSize;

    int tileRow = (line / 2 + tilemapWindowY) / tileSize;
    int tileYOffset = (line / 2 + tilemapWindowY) % tileSize;
    int maxX = width - borderXSize;

    // we always read tiles 0 to width/TILE_SIZE + 1 - this is to ensure we can display partial tiles, with X,Y offsets
    // TODO - variable length array: int tilemapItemCount = width / 2 / tileSize + 1;
    const uint8_t tilemapItemCount = 41;  // this is the maximum number of tiles in a line
    // The + 2 below is to take an FPGA bug in the F256Jr into account
    //int tlmSize = tilemapItemCount * 2 + 2;
    const uint8_t tlmSize = 84;
    uint8_t tiles[tlmSize];
    int tilesetOffsets[tilemapItemCount];

    // The + 2 below is to take an FPGA bug in the F256Jr into account
    memcpy(tiles, videoram_ptr + tilemapAddress + (tilemapWindowX / tileSize) * 2 + (tileRow + 0) * tilemapWidth * 2, tlmSize);

    // cache of tilesetPointers
    int tilesetPointers[8];
    int strides[8];
    for (int i = 0; i < 8; i++)
    {
        tilesetPointers[i] = (iopage0_ptr[0xD280 - 0xC000 + i * 4] + (iopage0_ptr[0xD280 - 0xC000 + i * 4 + 1] << 8) +
                             (iopage0_ptr[0xD280 - 0xC000 + i * 4 + 2] << 16)) & 0x3F'FFFF;
        uint8_t tilesetConfig = iopage0_ptr[0xD280 - 0xC000 + i * 4 + 3];
        strides[i] = (tilesetConfig & 8) != 0 ? strideLine : tileSize;
    }
    for (int i = 0; i < tilemapItemCount; i++)
    {
        uint8_t tile = tiles[i * 2];
        uint8_t tilesetReg = tiles[i * 2 + 1];
        uint8_t tileset = tilesetReg & 7;

        // tileset
        int tilesetPointer = tilesetPointers[tileset];
        int strideX = strides[tileset];
        if (strideX == tileSize)
        {
            tilesetOffsets[i] = tilesetPointer + (tile % 16) * tileSize * tileSize + (tile / 16) * tileSize * tileSize * 16 + tileYOffset * tileSize;
        }
        else
        {
            tilesetOffsets[i] = tilesetPointer + ((tile / 16) * strideX * tileSize + (tile % 16) * tileSize) + tileYOffset * strideX;
        }
    }

    // alternate display style - avoids repeating the loop so often
    int startTileX = (borderXSize + tileXOffset) / tileSize;
    int endTileX = (width/2 - borderXSize + tileXOffset) / tileSize + 1;
    int startOffset = (borderXSize + tilemapWindowX) % tileSize;
    int x = borderXSize;
    //uint8_t tilepix[tileSize];
    rgb_t clr_val = 0;
    for (int t = startTileX; t < endTileX; t++)
    {
        // The (mode==0 ? 1 : 3) below is to take an FPGA but in the F256Jr into account
        uint8_t tilesetReg = tiles[t * 2 + 3];
        uint8_t lut_index = (tilesetReg & 0x38) >> 3;
        //int lutAddress = MemoryMap.GRP_LUT_BASE_ADDR - VICKY.StartAddress + lutIndex * 1024;
        //int tilesetOffsetAddress = tilesetOffsets[t];  // + startOffset
        //memcpy(tilepix, videoram_ptr + tilesetOffsets[t], tileSize);
        do
        {
            uint8_t pixVal = videoram_ptr[tilesetOffsets[t] + startOffset];  // tilepix[startOffset];
            if (pixVal > 0)
            {
                clr_val = get_lut_value(lut_index, pixVal, enable_gamma);
                bitmap.pix(line, x) = clr_val;
                bitmap.pix(line, x + 1) = clr_val;
            }
            startOffset++;
            //tilesetOffsetAddress++;
            x+=2;
        } while (startOffset != tileSize && x < maxX);
        startOffset = 0;
        if (x == maxX)
        {
            break;
        }
    }
}

void tiny_vicky_video_device::draw_mouse(bitmap_rgb32 &bitmap, bool enable_gamma, uint16_t line, uint16_t width, uint16_t height)
{
    uint8_t mouse_reg = iopage0_ptr[0xD6E0 -0xC000];

    bool MousePointerEnabled = (mouse_reg & 3) != 0;

    if (MousePointerEnabled)
    {
        int PosX = iopage0_ptr[0xD6E0 -0xC000 + 2] + (iopage0_ptr[0xD6E0 -0xC000 + 3] << 8);
        int PosY = iopage0_ptr[0xD6E0 -0xC000 + 4] + (iopage0_ptr[0xD6E0 -0xC000 + 5] << 8);
        if (line >= PosY && line < PosY + 16)
        {
            int ptr_addr = 0xCC00 - 0xC000;

            // Mouse pointer is a 16x16 icon
            int colsToDraw = PosX < width - 16 ? 16 : width - PosX;

            int mline = line - PosY;
            for (int col = 0; col < colsToDraw; col++)
            {
                // Values are 0: transparent, 1:black, 255: white (gray scales)
                uint8_t pixel_index = iopage0_ptr[ptr_addr + mline * 16 + col];
                rgb_t value;

                if (pixel_index != 0)
                {
                    if (enable_gamma)
                    {
                        value = rgb_t(pixel_index, pixel_index, pixel_index);
                    }
                    else
                    {
                        value = rgb_t(iopage0_ptr[0x800 + pixel_index],
                                    iopage0_ptr[0x400 + pixel_index],
                                    iopage0_ptr[pixel_index]);
                    }
                    bitmap.pix(col + PosX, mline) = value;
                }
            }
        }

    }
}
