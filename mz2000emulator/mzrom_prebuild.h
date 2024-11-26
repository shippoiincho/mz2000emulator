// Dummy ROM file for Prebuild binary

// ROM Data                 at
// IPL.ROM(2000)     2KiB    0x10070000
// IPL.ROM(80B)      2KiB    0x10074000
// FONT.ROM          2KiB    0x10078000

#define ROMBASE 0x10070000u

uint8_t *mzipl    =(uint8_t *)(ROMBASE);
uint8_t *mzipl0   =(uint8_t *)(ROMBASE+0x4000);
uint8_t *mzfont   =(uint8_t *)(ROMBASE+0x8000);
