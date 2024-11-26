//  SHARP MZ-2000/80B emulator
//
//  GP0: HSYNC
//  GP1: VSYNC
//  GP2: Blue
//  GP3: Red
//  GP4: Green
//  GP6: Audio

//#define PREBUILD_BINARY         // Genetate Prebuild Binary

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"

#include "tusb.h"
#include "bsp/board.h"

#include "vga16_graphics.h"

#include "mzkeymap.h"
#include "Z80.h"
#include "mzmisc.h"

#ifndef PREBUILD_BINARY
#include "mzrom.h"
#else 
#include "mzrom_prebuild.h"
#endif

#include "lfs.h"

//#define USE_KANJI

#ifdef USE_KANJI
#include "mzkanji.h"
#endif

// VGAout configuration

#define DOTCLOCK 25000
#define CLOCKMUL 9
// Pico does not work at CLOCKMUL=7 (175MHz).

#define VGA_PIXELS_X 640
#define VGA_PIXELS_Y 400

#define VGA_CHARS_X 80
#define VGA_CHARS_Y 25

#define VRAM_PAGE_SIZE (VGA_PIXELS_X*VGA_PIXELS_Y/8)

extern unsigned char vga_data_array[];
volatile uint8_t fbcolor,cursor_x,cursor_y,video_mode;

volatile uint32_t video_hsync,video_vsync,scanline,vsync_scanline;
volatile uint32_t redraw_command=0;
volatile uint32_t scroll_flag=0;

struct repeating_timer timer,timer2;

// MZ configuration

//static Z80Context cpu;
static Z80 cpu;
uint32_t cpu_clocks=0;
uint32_t cpu_ei=0;
int exec_reti;

uint32_t cputrace=0;

uint8_t mainram[0x10000];
//uint8_t cvram[0x800];
uint8_t cvram[0x1000];
uint8_t gvram[0xc000];

uint8_t ioport[0x100];

uint32_t rombank=0;
uint32_t vrambank=0;
uint32_t machinetype=0;
uint32_t videotype=0;

#ifdef USE_KANJI
uint32_t kanji_ptr=0;
uint32_t jisho_ptr=0;
#endif

uint8_t i8253[3];
uint8_t i8253_access[3];
uint16_t i8253_preload[3];
uint16_t i8253_counter[3];
uint8_t i8253_pending[3];
uint16_t i8253_latch[3];
uint32_t i8253_latch_flag=0;
volatile uint32_t i8253_enable_irq=0;
uint32_t beep_flag=0;
uint32_t beep_mute=0;

uint8_t sioa[8],siob[8];
uint8_t sioa_access,siob_access;

uint8_t pioa[4],piob[4];
volatile uint32_t pioa_enable_irq=0;
volatile uint32_t piob_enable_irq=1;
uint32_t pioa_next_mask,pioa_next_iocontrol;
uint32_t piob_next_mask,piob_next_iocontrol;
uint32_t pio_irq_processing=0;

volatile uint8_t keypressed=0;  //last pressed usbkeycode

uint32_t lastmodifier=0; 

// Tape

uint32_t tape_ready=0;
uint32_t tape_ptr=0;
uint32_t tape_phase=0;
uint32_t tape_count=0;

#define TAPE_THRESHOLD 200000

uint8_t uart_rx[32];
uint8_t uart_nibble=0;
uint8_t uart_count=0;
volatile uint8_t uart_write_ptr=0;
volatile uint8_t uart_read_ptr=0;
uint32_t uart_cycle;

// MZ-700/1500
//#define TAPE1 245
//#define TAPE0 123

#define TAPE1 186
#define TAPE0 90

//#define TAPE1 147
//#define TAPE0 74

uint32_t lastclocks;

// UI

uint32_t menumode=0;
uint32_t menuitem=0;

// USB

hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released
extern void hid_app_task(void);

uint32_t usbcheck_count=0;
uint32_t kbhit=0;            // 4:Key pressed (timer stop)/3&2:Key depressed (timer running)/1:no key irq triggerd
uint8_t hid_dev_addr=255;
uint8_t hid_instance=255;
uint8_t hid_led;

uint8_t keymatrix[16];
uint8_t keymatrix_bak[16];

#define USB_CHECK_INTERVAL 30 // 31.5us*30=1ms

// QD
uint8_t qd_status=0;
uint8_t qd_filename[16];
lfs_file_t qd_drive;
uint8_t qd_motor=0;
uint8_t qd_data;
uint32_t qd_ptr=0;
uint32_t qd_blocksize=0;
uint32_t qd_sync=0;
uint32_t qd_type=0;
uint32_t qd_stage=0;
uint8_t qd_numblocks=0;
uint32_t qd_count;

const uint8_t qd_header[]={0xa5,0x00,0x00,0x00,0x16};

// Define the flash sizes
// This is setup to read a block of the flash from the end 
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)
// for 1M flash pico
//#define HW_FLASH_STORAGE_BASE   (1024*1024 - HW_FLASH_STORAGE_BYTES) 
//#define HW_FLASH_STORAGE_BYTES  (512 * 1024)
// for 2M flash
// #define HW_FLASH_STORAGE_BYTES  (1024 * 1024)
#define HW_FLASH_STORAGE_BYTES  (1536 * 1024)
#define HW_FLASH_STORAGE_BASE   (PICO_FLASH_SIZE_BYTES - HW_FLASH_STORAGE_BYTES) 
// for 16M flash
//#define HW_FLASH_STORAGE_BYTES  (15872 * 1024)
//#define HW_FLASH_STORAGE_BASE   (1024*1024*16 - HW_FLASH_STORAGE_BYTES) 

lfs_t lfs;
lfs_file_t lfs_file;

#define FILE_THREHSOLD 20000000
#define LFS_LS_FILES 9

volatile uint32_t load_enabled=0;
volatile uint32_t save_enabled=0;
//uint32_t file_cycle=0;

unsigned char filename[16];
unsigned char tape_filename[16];

static void draw_framebuffer(uint16_t);
static inline unsigned char tohex(int);
static inline unsigned char fromhex(int);
static inline void video_print(uint8_t *);

// Virtual H-Sync for emulation
bool __not_in_flash_func(hsync_handler)(struct repeating_timer *t) {

    if(scanline%262==0) {
        video_vsync=1;
    }

    scanline++;

    video_hsync=1;

    // // Tempo 555 32Hz

    // if((scanline%246)==0) {
    //     if(tempo_timer) {
    //         tempo_timer=0;
    //     } else {
    //         tempo_timer=1;
    //     }
    // }

    // // cursor 555 1.5Hz

    // if((scanline%5249)==0) {
    //     if(cursor_timer) {
    //         cursor_timer=0;
    //     } else {
    //         cursor_timer=1;
    //     }
    // }

    // run i8253 
    // channel 0 31.25kHz

    if((i8253_access[0]<2)&&(i8253_preload[0]!=0)) {
        if(i8253_counter[0]>2) {
            i8253_counter[0]-=2;
        } else {
            i8253_counter[0]=i8253_preload[0];

        if((i8253_access[1]<2)&&(i8253_preload[1]!=0)) {
            if(i8253_counter[1]>1) {
                i8253_counter[1]--;
                    } else {
                i8253_counter[1]=i8253_preload[1];
                if((i8253_access[2]<2)&&(i8253_preload[2]!=0)){
                    if(i8253_pending[2]) {
                        i8253_pending[2]=0;
                    } else {
                        if (i8253_counter[2]>1) {
                           i8253_counter[2]--;
                        } else {
                            i8253_counter[2]=i8253_preload[2];
                        }
                    }
                }
            }
        }

        }
    }

    return true;

}

void __not_in_flash_func(uart_handler)(void) {

    uint8_t ch;

    // if((main_cpu.cycles-uart_cycle)>TAPE_THRESHOLD) {
    //     uart_count=0;        
    // }

    // uart_cycle=main_cpu.cycles;

    if(uart_is_readable(uart0)) {
        ch=uart_getc(uart0);
        if(uart_count==0) {
            uart_nibble=fromhex(ch)<<4;
            uart_count++;
        } else {
            ch=fromhex(ch)+uart_nibble;
            uart_count=0;

            if(uart_read_ptr==uart_write_ptr+1) {  // buffer full
                return;
            }
            if((uart_read_ptr==0)&&(uart_write_ptr==31)) {
                return;
            }

            uart_rx[uart_write_ptr]=ch;
            uart_write_ptr++;
            if(uart_write_ptr>31) {
                uart_write_ptr=0;
            }
        }
    }

}

// MZ series tape format
// Leader block
// 0 x 22000
// 1 x 40
// 0 x 40
// 1
//----
// Information block(128B)
// Checksum(2B)
// 1
// 0 x 256
// Informain block(128B)
// Checksum(2B)
// 1
//---
// 0 x 11000
// 1 x 20
// 0 x 20
// 1
// Data(variable)
// Checksum(2B)
// 1
// 0 x 256
// Data(variable)
// Checksum(2B)
// 1

uint8_t tapein() {

    static uint32_t bytecount,bitcount,bitphase;
    static uint32_t tapebit,nextbit;
    static uint8_t tapebyte;
    static uint32_t state,checksum;
    static uint32_t tapeclocks;
    static uint32_t tape_start_ptr;
    static uint32_t blocksize;

    if(load_enabled==0) {
        return 0;
    }

    // check motor on

    if(tape_ready==0) {
        return 0;
    }

    // check initial state

    if((tape_phase==0)&&(tape_count==0)) {
        bytecount=0;
        bitcount=0;
        bitphase=0;
        blocksize=0;
        nextbit=1;
        tape_start_ptr=tape_ptr;
    }

    // need next bit ?

    if(tapebit) {
        if ((cpu_clocks-tapeclocks)>TAPE1*2) {
            nextbit=1;
        }
    } else {
        if ((cpu_clocks-tapeclocks)>TAPE0*2) {
            nextbit=1;
        }
    }

    // get next bit

    if(nextbit) {

        nextbit=0;
        bitphase=0;
        tape_count=1;

        if(tape_phase==0) { // Send 0 x 22000
            bitcount++;
            tapebit=0;
            if(bitcount>22000) {
                tape_phase=1;
                bitcount=0;
            }           
                    
        } 
        if(tape_phase==1) { // Send 1 x 40
            bitcount++;
            tapebit=1;
            if(bitcount>40) {
                tape_phase=2;
                bitcount=0;
            }  

        }
        if(tape_phase==2) { // Send 0 x 40
            bitcount++;
            tapebit=0;
            if(bitcount>40) {
                tape_phase=3;
                bitcount=0;
            }  
        }
        if(tape_phase==3) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=4;
                bitcount=8;
                bytecount=0;
                checksum=0;
            }  
        }
        if(tape_phase==4) { // Send Information Block
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>128) {
                    tape_phase=5;
                    bitcount=8;
                    bytecount=0;
                } else {
                
                    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
                    tape_ptr++;

                    if(bytecount==19) {
                        blocksize=tapebyte;
                    }
                    if(bytecount==20) {
                        blocksize+=tapebyte*256;
                    }


                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    checksum++;
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==5) { // Send Checksum of  Information Block
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>2) {
                    tape_phase=6;
//                    tape_phase=11;          // SKIP SECOND BLOCK
                    bitcount=0;
                    bytecount=0;
                } else {
                    if(bytecount==1) {
                        tapebyte=(checksum&0xff00)>>8;
                    } else {
                        tapebyte=checksum&0xff;
                    }

                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==6) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=7;
                bitcount=0;
                bytecount=0;
                checksum=0;
            }  
        }    
        if(tape_phase==7) { // Send 0 x 256
            bitcount++;
            tapebit=0;
            if(bitcount>256) {
                tape_phase=8;
                bitcount=8;
                bytecount=0;
                checksum=0;
                tape_ptr=tape_start_ptr;
                lfs_file_seek(&lfs,&lfs_file,tape_ptr,LFS_SEEK_SET);
            }  
        }
        if(tape_phase==8) { // Send Information Block (2nd)
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>128) {
                    tape_phase=9;
                    bitcount=8;
                    bytecount=0;
                } else {
                
                    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
//                    tapebyte=mztest[tape_ptr];
                    tape_ptr++;

//                    checksum+=tapebyte;
                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    checksum++;
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==9) { // Send Checksum of  Information Block
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>2) {
                    tape_phase=10;
                    bitcount=0;
                    bytecount=0;
                } else {
                    if(bytecount==1) {
                        tapebyte=(checksum&0xff00)>>8;
                    } else {
                        tapebyte=checksum&0xff;
                    }
                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }    
        if(tape_phase==10) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=11;
                bitcount=0;
            }  
        }
        if(tape_phase==11) { // Send 0 x 11000
            bitcount++;
            tapebit=0;
            if(bitcount>11000) {
                tape_phase=12;
                bitcount=0;
            }           
                    
        } 
        if(tape_phase==12) { // Send 1 x 20
            bitcount++;
            tapebit=1;
            if(bitcount>20) {
                tape_phase=13;
                bitcount=0;
            }  

        }
        if(tape_phase==13) { // Send 0 x 20
            bitcount++;
            tapebit=0;
            if(bitcount>20) {
                tape_phase=14;
                bitcount=0;
            }  
        }
        if(tape_phase==14) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=15;
                bitcount=8;
                bytecount=0;
                checksum=0;
                tape_ptr=tape_start_ptr+128;
                lfs_file_seek(&lfs,&lfs_file,tape_ptr,LFS_SEEK_SET);
            }  
        }
        if(tape_phase==15) { // Send Data
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>blocksize) {
                    tape_phase=16;
                    bitcount=8;
                    bytecount=0;
                } else {
                
                    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
//                    tapebyte=mztest[tape_ptr];

                    tape_ptr++;

                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    checksum++;
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==16) { // Send Checksum of DATA
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>2) {  
                    tape_phase=17;
                    bitcount=0;
                    bytecount=0;
                } else {
                    if(bytecount==1) {
                        tapebyte=(checksum&0xff00)>>8;
                    } else {
                        tapebyte=checksum&0xff;
                    }

                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==17) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=18; // May be finish here
                bitcount=0;
                bytecount=0;
                checksum=0;
            }  
        }    
        if(tape_phase==18) { // Send 0 x 256
            bitcount++;
            tapebit=0;
            if(bitcount>256) {
                tape_phase=19;
                bitcount=8;
                bytecount=0;
                checksum=0;
                tape_ptr=tape_start_ptr+128;
                lfs_file_seek(&lfs,&lfs_file,tape_ptr,LFS_SEEK_SET);
            }  
        }
        if(tape_phase==19) { // Send DATA (2nd)
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>blocksize) {
                    tape_phase=20;
                    bitcount=8;
                    bytecount=0;
                } else {
                
                    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
//                    tapebyte=mztest[tape_ptr];
                    tape_ptr++;

                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    checksum++;
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==20) { // Send Checksum of DATA
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>2) {
                    tape_phase=21;
                    bitcount=0;
                    bytecount=0;
                } else {
                    if(bytecount==1) {
                        tapebyte=(checksum&0xff00)>>8;
                    } else {
                        tapebyte=checksum&0xff;
                    }
                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }    
        if(tape_phase==21) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=22;
                bitcount=0;
            }  
        }

                
    }

    // 
    if(bitphase==0) { // send High First
        tapeclocks=cpu_clocks;
        bitphase=1;
        return 1;
    } else if(tapebit) {
        if ((cpu_clocks-tapeclocks)<TAPE1) {
            return 1;
        } else {
            return 0;
        }    
    } else {
        if ((cpu_clocks-tapeclocks)<TAPE0) {
            return 1;
        } else {
            return 0;
        }  
    }
    
    

    
    return 0;

}

void tapeout(uint8_t data) {

    static uint32_t bytecount,bitcount,bitphase;
    static uint32_t tapebit,blocksize;
    static uint32_t tapeclocks;
    static uint8_t tapebyte;

    if(save_enabled==0) {
        return;
    }

    save_enabled=2;

    // check motor on

    if(tape_ready==0) {
        return;
    }

    if(data) {  // bit High
        tapeclocks=cpu_clocks;
        return;
    } else { 

        if((cpu_clocks-tapeclocks)>130) {
            if((cpu_clocks-tapeclocks)>1000) { // leader
                tapebit=0;
            } else {
                tapebit=1;
            }
        } else {
            tapebit=0;
        }
    }

    switch(tape_phase) {

        case 0:     //  0 x 22000
            if(tapebit) {
                tape_phase=1;
                bitcount=1;
            }
            return;

        case 1:   // 1 x 40 + 0 x 40 + 1
            bitcount++;
            if(bitcount>81) {
                tape_phase=2;
                bitcount=1;
                bytecount=0;
                tape_count=0;
            }
            return;

        case 2:  // Header(1st)
            bitcount++;
            tape_count++;
            if(bitcount>1) {
                tapebyte<<=1;
                if(tapebit) {
                    tapebyte|=1;
                }
            }
            if(bitcount==9) {
                
                lfs_file_write(&lfs,&lfs_file,&tapebyte,1);

                bitcount=0;
                bytecount++;
                if(bytecount==0x13) {
                    blocksize=tapebyte;
                }
                if(bytecount==0x14) {
                    blocksize+=tapebyte*256;
                }

                if(bytecount==128) {
                    tape_phase=3;
                    bitcount=0;
                }
            }
            return;

        case 3: // checksum(18bit) + 1 + 0 x 256 + Header(2nd 2304bit) + checksum(18bit) + 1
            bitcount++;
            if(bitcount>2598) {
                tape_phase=4;
                bitcount=1;
                bytecount=0;
            }
            return;

        case 4:     //  1 x 22000
            if(tapebit) {
                tape_phase=5;
                bitcount=1;
            }
            return;

       case 5: // 1 x 20 + 0 x 20 + 1
            bitcount++;
            if(bitcount>41) {
                tape_phase=6;
                bitcount=1;
                bytecount=0;
            }
            return;

        case 6:  // data(1st)
            bitcount++;
            tape_count++;
            if(bitcount>1) {
                tapebyte<<=1;
                if(tapebit) {
                    tapebyte|=1;
                }
            }
            if(bitcount==9) {

                lfs_file_write(&lfs,&lfs_file,&tapebyte,1);

                bitcount=0;
                bytecount++;

                if(bytecount>=blocksize) {
                    tape_phase=7;
                    bitcount=0;

                    // can be finish here

                    lfs_file_close(&lfs,&lfs_file);
//                    save_enabled=0;

                }
            }
            return;

        case 7:  // checksum(18bit) + 1 + 0 x 256

            bitcount++;
            if(bitcount>275) {
                tape_phase=8;
                bitcount=1;
                bytecount=0;
            }
            return;

        case 8:  // data(2nd)
            bitcount++;
            tape_count++;
            if(bitcount>1) {
                tapebyte<<=1;
                if(tapebit) {
                    tapebyte|=1;
                }
            }
            if(bitcount==9) {

                bitcount=0;
                bytecount++;

                if(bytecount>=blocksize) {
                    tape_phase=9;
                    bitcount=0;
                }
            }
            return;

       case 9: // checksum(18) + 1
            bitcount++;
            if(bitcount>19) {
                save_enabled=0;
                // tape_phase=6;
                // bitcount=1;
                // bytecount=0;
            }
            return;


    }

}

void mztload() { // Direct load mzt to ram

    uint32_t blocksize;
    uint8_t data;

    lfs_file_seek(&lfs,&qd_drive,0,LFS_SEEK_SET);

    for(int i=0;i<128;i++) {

        lfs_file_read(&lfs,&qd_drive,&data,1);

        if(i==0x12) blocksize=data;
        if(i==0x13) blocksize+=data*256;

    }

//    printf("[MZT:%04x]",blocksize);

    for(int i=0;i<blocksize;i++) {
        lfs_file_read(&lfs,&qd_drive,&data,1);
        mainram[i]=data;
        
    }

}

#if 0
void qd_check() {

    uint8_t qdftmp[16];

    lfs_file_seek(&lfs,&qd_drive,0,LFS_SEEK_SET);
    lfs_file_read(&lfs,&qd_drive,qdftmp,16);

    if(memcmp(qdftmp,"-QD format-",11)==0) {
        qd_type=0;
    } else {
        qd_ptr=0;
        qd_numblocks=0;
        while(qd_ptr<lfs_file_size(&lfs,&qd_drive)) {

            lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET);
            lfs_file_read(&lfs,&qd_drive,qdftmp,16);
            lfs_file_read(&lfs,&qd_drive,qdftmp,16);
            qd_blocksize=qdftmp[3]*256+qdftmp[2];

            qd_ptr+=128;
            qd_ptr+=qd_blocksize;
            qd_numblocks+=2;
        }

        qd_type=1;
        qd_stage=0;
        qd_count=0;
    }

    lfs_file_seek(&lfs,&qd_drive,0,LFS_SEEK_SET);    

}


uint8_t qd_read() {

    uint8_t qd_data,qd_mark,size_hi,size_lo;
    static uint32_t qd_blockcount;

    if(qd_status==0) {
        return 0xff;
    }

    if(qd_type==0) { // QDF format

    if(qd_motor) {

        // find sync

        if(qd_sync) {

            qd_data=0;
            while(qd_data!=0x16) { // Find Sync
                lfs_file_read(&lfs,&qd_drive,&qd_data,1);
                qd_ptr++;
            }

            while(qd_data==0x16) { // Find Sync
                lfs_file_read(&lfs,&qd_drive,&qd_data,1);
                qd_ptr++;
            }

            qd_sync=0;
            qd_ptr--;

//            lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET);  // Not needed ?

            lfs_file_read(&lfs,&qd_drive,&qd_mark,1);

            if(qd_stage==0) {  // first record
                qd_blocksize=5;
                qd_stage=1;
            } else {
                lfs_file_read(&lfs,&qd_drive,&size_lo,1);
                lfs_file_read(&lfs,&qd_drive,&size_hi,1);
                qd_blocksize=size_lo+size_hi*256+7;
            }

            // if((qd_mark==4)||(qd_mark==2)||(qd_mark==0xff)) {
            //     qd_blocksize=5;
            // } else {
            //     lfs_file_read(&lfs,&qd_drive,&size_lo,1);
            //     lfs_file_read(&lfs,&qd_drive,&size_hi,1);
            //     qd_blocksize=size_lo+size_hi*256+7;
            // }

            lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET); 

//            printf("\n\r[QDM:%02x/%05x/%04x]",qd_data,qd_ptr,qd_blocksize);

        }

        if(qd_ptr<=lfs_file_size(&lfs,&qd_drive)) {

            qd_blocksize--;
            if(qd_blocksize==0) {
//                qd_ptr+=8;  // ?
                  qd_ptr+=16;  // ?
                lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET);

                qd_sync=1;
                return 0x16;
            }            

            lfs_file_read(&lfs,&qd_drive,&qd_data,1);
            qd_ptr++;
            return qd_data;

        } else {

            qd_motor=0;

            return 0;
        }

    } 
    } else { // Q20/MZT format

        if(qd_motor) {

            switch(qd_stage) {

                case 0:   // Preample
                    qd_data=qd_header[qd_count];

                    qd_count++;

                    if(qd_count==2) {
                        qd_data=qd_numblocks;

                    }
                    if(qd_count>=5) {
                        qd_stage=1;
                        qd_blockcount=0;
                    }

                    return qd_data;

                case 1: // 

                    if(qd_blockcount>=qd_numblocks) {
                        return 0x16;
                    }

                    qd_stage=2;
                    qd_count=0;

                    return 0xa5;

                case 2:  // Header

                    qd_count++;

                    if(qd_count==1) {
                        return 0;
                    }
                    if(qd_count==2) {
                        return 0x40;
                    }
                    if(qd_count==3) {
                        return 0;
                    }

                    if(qd_count==0x16) {
                        return 0;
                    }
                    if(qd_count==0x17) {
                        return 0;
                    }


                    lfs_file_read(&lfs,&qd_drive,&qd_data,1);
                    qd_ptr++;

                    if(qd_count==0x18) {
                        qd_blocksize=qd_data;
                    }
                    if(qd_count==0x19) {                    
                        qd_blocksize+=qd_data*256;
                    }

                    if(qd_count>=67) {
                        qd_stage=3;
                        qd_ptr+=66;
                        lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET);
                    }

                    return qd_data;

                case 3:  // Checksum
                    qd_stage=4;
                    return 0xff;

                case 4:  // Checksum
                    qd_stage=5;
                    return 0xff;

                case 5:  // Sync
                    qd_stage=6;
                    return 0x16;

                case 6:   // Preample
                    qd_stage=7;
                    qd_count=0;
                    return 0xa5;

                case 7:
                    qd_count++;
                    if(qd_count==1) return 5;
                    if(qd_count==2) return qd_blocksize&0xff;
                    if(qd_count==3) {
                        qd_stage=8;
                        qd_count=0;
                        return (qd_blocksize&0xff00)>>8;
                    }

                case 8:  // Data
                    lfs_file_read(&lfs,&qd_drive,&qd_data,1);
                    qd_ptr++;
                    qd_count++;
                    if(qd_count>=qd_blocksize) {
                        qd_stage=9;
                    }

                    return qd_data;

                case 9:  // Checksum
                    qd_stage=10;
                    return 0xff;

                case 10:  // Checksum
                    qd_stage=11;
                    return 0xff;

                case 11:  // Sync
                    qd_stage=1;
                    qd_blockcount+=2;
                    return 0x16;
            }

        }

    }

    return 0;

}
#endif

static inline void video_cls() {
    memset(vga_data_array, 0x0, (640*400/2));
}

static inline void video_scroll() {

    memmove(vga_data_array, vga_data_array + VGA_PIXELS_X/2*16, (640*384/2));
    memset(vga_data_array + (640*384/2), 0, VGA_PIXELS_X/2*16);

}

static inline void video_print(uint8_t *string) {

    int len;
    uint8_t fdata;

    len = strlen(string);

    for (int i = 0; i < len; i++) {

        for(int slice=0;slice<16;slice++) {

            uint8_t ch=string[i];

            fdata=mzfont[ch*8+(slice>>1)];

            uint32_t vramindex=cursor_x*4+VGA_PIXELS_X*(cursor_y*16+slice)/2;

            for(int slice_x=0;slice_x<4;slice_x++){

                if(fdata&0x40) {
                    vga_data_array[vramindex+slice_x]=(fbcolor&7)<<4;
                } else {
                    vga_data_array[vramindex+slice_x]=fbcolor&0x70;  
                }

                  if(fdata&0x80) {
                    vga_data_array[vramindex+slice_x]+=fbcolor&7;
                } else {
                    vga_data_array[vramindex+slice_x]+=(fbcolor&0x70)>>4;  
                }              

                fdata<<=2;

            }

        }

        cursor_x++;
        if (cursor_x >= VGA_CHARS_X) {
            cursor_x = 0;
            cursor_y++;
            if (cursor_y >= VGA_CHARS_Y) {
                video_scroll();
                cursor_y = VGA_CHARS_Y - 1;
            }
        }
    }

}

void draw_menu(void) {

    cursor_x=2;
    cursor_y=5;
    fbcolor=7;
        video_print("+--------------------------------------+");
    for(int i=6;i<19;i++) {
        cursor_x=2;
        cursor_y=i;
        video_print("|                                      |");
    }

    cursor_x=2;
    cursor_y=19;
    fbcolor=7;

    video_print("+--------------------------------------+");
}

int draw_files(int num_selected,int page) {

    lfs_dir_t lfs_dirs;
    struct lfs_info lfs_dir_info;
    uint32_t num_entry=0;
    unsigned char str[16];

    int err= lfs_dir_open(&lfs,&lfs_dirs,"/");

    if(err) return -1;

    for(int i=0;i<LFS_LS_FILES;i++) {
        cursor_x=24;
        cursor_y=i+6;
        fbcolor=7;
        video_print("             ");
    }

    while(1) {

        int res= lfs_dir_read(&lfs,&lfs_dirs,&lfs_dir_info);
        if(res<=0) {
            break;
        }

        cursor_x=30;
        cursor_y=18;
        fbcolor=7;
        sprintf(str,"Page %02d",page+1);

        video_print(str);

        switch(lfs_dir_info.type) {

            case LFS_TYPE_DIR:
                break;
            
            case LFS_TYPE_REG:

                if((num_entry>=LFS_LS_FILES*page)&&(num_entry<LFS_LS_FILES*(page+1))) {

                    cursor_x=24;
                    cursor_y=num_entry%LFS_LS_FILES+6;

                    if(num_entry==num_selected) {
                        fbcolor=0x70;
                        memcpy(filename,lfs_dir_info.name,16);
                    } else {
                        fbcolor=7;
                    }

                    video_print(lfs_dir_info.name);

                }

                num_entry++;

                break;

            default:
                break; 

        }

    }

    lfs_dir_close(&lfs,&lfs_dirs);

    return num_entry;

}

int file_selector(void) {

    uint32_t num_selected=0;
    uint32_t num_files=0;
    uint32_t num_pages=0;

    num_files=draw_files(-1,0);

    if(num_files==0) {
         return -1;
    }

    while(1) {

        while(video_vsync==0) ;
        video_vsync=0;

        draw_files(num_selected,num_selected/LFS_LS_FILES);

        tuh_task();

        if(keypressed==0x52) { // up
            keypressed=0;
            if(num_selected>0) {
                num_selected--;
            }
        }

        if(keypressed==0x51) { // down
            keypressed=0;
            if(num_selected<num_files-1) {
                num_selected++;
            }
        }

        if(keypressed==0x4b) { // Pageup
            keypressed=0;
            if(num_selected>=LFS_LS_FILES) {
                num_selected-=LFS_LS_FILES;
            }
        }

        if(keypressed==0x4e) { // Pagedown
            keypressed=0;
            if(num_selected<num_files-LFS_LS_FILES) {
                num_selected+=LFS_LS_FILES;
            }
        }

        if(keypressed==0x28) { // Ret
            keypressed=0;

            return 0;
        }

        if(keypressed==0x29 ) {  // ESC

            return -1;

        }

    }
}

int enter_filename() {

    unsigned char new_filename[16];
    unsigned char str[32];
    uint8_t keycode;
    uint32_t pos=0;

    memset(new_filename,0,16);

    while(1) {

        sprintf(str,"Filename:%s  ",new_filename);
        cursor_x=3;
        cursor_y=18;
        video_print(str);

        while(video_vsync==0) ;
        video_vsync=0;

        tuh_task();

        if(keypressed!=0) {

            if(keypressed==0x28) { // enter
                keypressed=0;
                if(pos!=0) {
                    memcpy(filename,new_filename,16);
                    return 0;
                } else {
                    return -1;
                }
            }

            if(keypressed==0x29) { // escape
                keypressed=0;
                return -1;
            }

            if(keypressed==0x2a) { // backspace
                keypressed=0;

                cursor_x=3;
                cursor_y=18;
                video_print("Filename:          ");

                new_filename[pos]=0;

                if(pos>0) {
                    pos--;
                }
            }

            if(keypressed<0x4f) {
                keycode=usbhidcode[keypressed*2];
                keypressed=0;

                if(pos<7) {

                    if((keycode>0x20)&&(keycode<0x5f)&&(keycode!=0x2f)) {

                        new_filename[pos]=keycode;
                        pos++;

                    }

                }
            }


        }
    }

}

// MZ-80B mode
// charactor redraw
static void draw_framebuffer80(uint16_t addr) {

    uint32_t slice_x,slice_y;
    uint16_t offset,gvaddr;
    uint32_t vramindex;
    uint32_t ch,color,bgcolor,bitdata;
    uint32_t bitdataw,bitmaskw;
    
    uint32_t graphw;

    addr&=0x7ff;
    ch=cvram[addr];
    color=ioport[0xf5]&7;
    bgcolor=ioport[0xf4]&7;

    uint32_t *vramptr=(uint32_t *)vga_data_array;

    if(ioport[0xe8]&0x20) { // 80 chars

        if(addr>2000) return;

        slice_x=addr%80;
        slice_y=addr/80;

        for(int slice_yy=0;slice_yy<8;slice_yy++) {

            gvaddr=slice_x/2+(slice_y*8+slice_yy)*40;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand80[bitdata*2];
            bitmaskw=bitexpand80[bitdata*2+1];

            graphw=0;

            if(slice_x%2) {
                if(ioport[0xf4]&2) {
                    graphw|=bitexpandg2[gvram[gvaddr]*4+1];
                } 
                if(ioport[0xf4]&4) {
                    graphw|=bitexpandg2[gvram[gvaddr+0x2000]*4+1];
                } 
            } else {
                if(ioport[0xf4]&2) {
                    graphw|=bitexpandg2[gvram[gvaddr]*4];
                } 
                if(ioport[0xf4]&4) {
                    graphw|=bitexpandg2[gvram[gvaddr+0x2000]*4];
                } 
            }

            vramindex=slice_x+(slice_y*16+slice_yy*2)*80;

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   
            
            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

        }

    } else {  // 40 chars

        if(addr>1000) return;

        slice_x=addr%40;
        slice_y=addr/40;

        for(int slice_yy=0;slice_yy<8;slice_yy++) {

            gvaddr=slice_x+(slice_y*8+slice_yy)*40;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand40[bitdata*4+1];
            bitmaskw=bitexpand40[bitdata*4+3];

            vramindex=slice_x*2+(slice_y*16+slice_yy*2)*80;

            graphw=0;

            if(ioport[0xf4]&2) {
                graphw|=bitexpandg2[gvram[gvaddr]*4];
            } 
            if(ioport[0xf4]&4) {
                graphw|=bitexpandg2[gvram[gvaddr+0x2000]*4];
            }

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   

            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

            bitdataw=bitexpand40[bitdata*4];
            bitmaskw=bitexpand40[bitdata*4+2];

            graphw=0;

            if(ioport[0xf4]&2) {
                graphw|=bitexpandg2[gvram[gvaddr]*4+1];
            } 
            if(ioport[0xf4]&4) {
                graphw|=bitexpandg2[gvram[gvaddr+0x2000]*4+1];
            }

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   

            *(vramptr+vramindex+1) = bitdataw;
            *(vramptr+vramindex+1+VGA_PIXELS_X/8) =  bitdataw;

        }
    }
}

// MZ-2000 mode
// charactor redraw
static void draw_framebuffer20(uint16_t addr) {

    uint32_t slice_x,slice_y;
    uint16_t offset,gvaddr;
    uint32_t vramindex;
    uint32_t ch,color,bgcolor,bitdata;
    uint32_t bitdataw,bitmaskw;
    
    uint32_t graphw;

    addr&=0x7ff;
    ch=cvram[addr];
    color=ioport[0xf5]&7;
    bgcolor=ioport[0xf4]&7;

    uint32_t *vramptr=(uint32_t *)vga_data_array;

    if(ioport[0xe8]&0x20) { // 80 chars

        if(addr>2000) return;

        slice_x=addr%80;
        slice_y=addr/80;

        for(int slice_yy=0;slice_yy<8;slice_yy++) {

            gvaddr=slice_x+(slice_y*8+slice_yy)*80;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand80[bitdata*2];
            bitmaskw=bitexpand80[bitdata*2+1];

            graphw=0;

            if(ioport[0xf6]&1) {
                graphw|=bitexpandg[gvram[gvaddr]*2];
            } 
            if(ioport[0xf6]&2) {
                graphw|=bitexpandg[gvram[gvaddr+0x4000]*2];
            } 
            if(ioport[0xf6]&4) {
                graphw|=bitexpandg[gvram[gvaddr+0x8000]*2];               
            } 
            vramindex=slice_x+(slice_y*16+slice_yy*2)*80;

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   
            
            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

        }

    } else {  // 40 chars

        if(addr>1000) return;

        slice_x=addr%40;
        slice_y=addr/40;

        for(int slice_yy=0;slice_yy<8;slice_yy++) {

            gvaddr=slice_x*2+(slice_y*8+slice_yy)*80;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand40[bitdata*4+1];
            bitmaskw=bitexpand40[bitdata*4+3];

            vramindex=slice_x*2+(slice_y*16+slice_yy*2)*80;

            graphw=0;

            if(ioport[0xf6]&1) {
                graphw|=bitexpandg[gvram[gvaddr]*2];
            }
            if(ioport[0xf6]&2) {
                graphw|=bitexpandg[gvram[gvaddr+0x4000]*2];
            }
            if(ioport[0xf6]&4) {
                graphw|=bitexpandg[gvram[gvaddr+0x8000]*2];
            }

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   

            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

            bitdataw=bitexpand40[bitdata*4];
            bitmaskw=bitexpand40[bitdata*4+2];

            graphw=0;

            if(ioport[0xf6]&1) {
                graphw|=bitexpandg[gvram[gvaddr+1]*2];
            }
            if(ioport[0xf6]&2) {
                graphw|=bitexpandg[gvram[gvaddr+0x4000+1]*2];
            }
            if(ioport[0xf6]&4) {
                graphw|=bitexpandg[gvram[gvaddr+0x8000+1]*2];
            }

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   

            *(vramptr+vramindex+1) = bitdataw;
            *(vramptr+vramindex+1+VGA_PIXELS_X/8) =  bitdataw;

        }

    }

}

// MZ-2200 mode
// charactor redraw
static void draw_framebuffer22(uint16_t addr) {

    uint32_t slice_x,slice_y;
    uint16_t offset,gvaddr;
    uint32_t vramindex;
    uint32_t ch,color,bgcolor,bitdata;
    uint32_t bitdataw,bitmaskw;
    
    uint32_t graphbw,graphrw,graphgw,graphmaskw,graphw,mixmaskw;

    addr&=0x7ff;
    ch=cvram[addr];
    color=ioport[0xf5]&7;
    bgcolor=ioport[0xf4]&7;

    if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
        return;
    }    
//    printf("[%04x:%02x]",addr,ch);

    uint32_t *vramptr=(uint32_t *)vga_data_array;

    if(ioport[0xe8]&0x20) { // 80 chars

        if(addr>2000) return;

        slice_x=addr%80;
        slice_y=addr/80;

        for(int slice_yy=0;slice_yy<8;slice_yy++) {

            gvaddr=slice_x+(slice_y*8+slice_yy)*80;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand80[bitdata*2];
            bitmaskw=bitexpand80[bitdata*2+1];

            graphmaskw=0x11111111;

            if(ioport[0xf6]&1) {
                graphbw=bitexpandg[gvram[gvaddr]*2];
                graphmaskw=bitexpandg[gvram[gvaddr]*2+1];
            } else {
                graphbw=0;
                graphmaskw=0x11111111;
            }
            if(ioport[0xf6]&2) {
                graphrw=bitexpandg[gvram[gvaddr+0x4000]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x4000]*2+1];
            } else {
                graphrw=0;
            }
            if(ioport[0xf6]&4) {
                graphgw=bitexpandg[gvram[gvaddr+0x8000]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x8000]*2+1];
            } else {
                graphgw=0;
            }

            vramindex=slice_x+(slice_y*16+slice_yy*2)*80;

            graphw = graphbw + graphrw * 2 + graphgw * 4;
            mixmaskw = bitmaskw & graphmaskw;

            if(ioport[0xf5]&8) {

                graphmaskw*=15;
                bitdataw = ((bitdataw * color) & graphmaskw) +graphw  + mixmaskw*bgcolor;
            
            } else {

                bitmaskw*=15;
                bitdataw = bitdataw * color + (graphw & bitmaskw) + mixmaskw*bgcolor;
            
            }

            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

        }

    } else {  // 40 chars

        if(addr>1000) return;

        slice_x=addr%40;
        slice_y=addr/40;

        for(int slice_yy=0;slice_yy<8;slice_yy++) {

            gvaddr=slice_x*2+(slice_y*8+slice_yy)*80;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand40[bitdata*4+1];
            bitmaskw=bitexpand40[bitdata*4+3];

            vramindex=slice_x*2+(slice_y*16+slice_yy*2)*80;

            graphmaskw=0x11111111;

            if(ioport[0xf6]&1) {
                graphbw=bitexpandg[gvram[gvaddr]*2];
                graphmaskw=bitexpandg[gvram[gvaddr]*2+1];
            } else {
                graphbw=0;
                graphmaskw=0x11111111;
            }
            if(ioport[0xf6]&2) {
                graphrw=bitexpandg[gvram[gvaddr+0x4000]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x4000]*2+1];
            } else {
                graphrw=0;
            }
            if(ioport[0xf6]&4) {
                graphgw=bitexpandg[gvram[gvaddr+0x8000]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x8000]*2+1];
            } else {
                graphgw=0;
            }

            graphw = graphbw + graphrw * 2 + graphgw * 4;
            mixmaskw = bitmaskw & graphmaskw;

            if(ioport[0xf5]&8) {

                graphmaskw*=15;
                bitdataw = ((bitdataw * color) & graphmaskw) +graphw  + mixmaskw*bgcolor;
            
            } else {

                bitmaskw*=15;
                bitdataw = bitdataw * color + (graphw & bitmaskw) + mixmaskw*bgcolor;
            
            }

            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

            bitdataw=bitexpand40[bitdata*4];
            bitmaskw=bitexpand40[bitdata*4+2];

            if(ioport[0xf6]&1) {
                graphbw=bitexpandg[gvram[gvaddr+1]*2];
                graphmaskw=bitexpandg[gvram[gvaddr+1]*2+1];
            } else {
                graphbw=0;
                graphmaskw=0x11111111;
            }
            if(ioport[0xf6]&2) {
                graphrw=bitexpandg[gvram[gvaddr+0x4000+1]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x4000+1]*2+1];
            } else {
                graphrw=0;
            }
            if(ioport[0xf6]&4) {
                graphgw=bitexpandg[gvram[gvaddr+0x8000+1]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x8000+1]*2+1];
            } else {
                graphgw=0;
            }

            graphw = graphbw + graphrw * 2 + graphgw * 4;
            mixmaskw = bitmaskw & graphmaskw;

            if(ioport[0xf5]&8) {

                graphmaskw*=15;
                bitdataw = ((bitdataw * color) & graphmaskw) +graphw  + mixmaskw*bgcolor;
            
            } else {

                bitmaskw*=15;
                bitdataw = bitdataw * color + (graphw & bitmaskw) + mixmaskw*bgcolor;
            
            }

            *(vramptr+vramindex+1) = bitdataw;
            *(vramptr+vramindex+1+VGA_PIXELS_X/8) =  bitdataw;

        }

    }

}

// MZ-80B mode
// graphic redraw
static void draw_framebuffer_gv80(uint16_t addr) {

    uint32_t slice_x,slice_y;
    uint16_t offset,gvaddr;
    uint32_t vramindex;
    uint32_t ch,color,bgcolor,bitdata;
    uint32_t bitdataw,bitmaskw;
    
    uint32_t graphw;

    gvaddr=addr&0x1fff;

    color=ioport[0xf5]&7;
    bgcolor=ioport[0xf4]&7;

    uint32_t *vramptr=(uint32_t *)vga_data_array;

    if(gvaddr>0x1f40) {
        return;
    }

    if(ioport[0xe8]&0x20) { // 80 chars

        slice_x=gvaddr%40;
        slice_y=gvaddr/40;

        uint32_t slice_yy=slice_y%8;
        slice_y=slice_y/8;

//        for(int slice_yy=0;slice_yy<8;slice_yy++) {

//            gvaddr=slice_x/2+(slice_y*8+slice_yy)*40;

            ch=cvram[slice_x*2+slice_y*80];

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand80[bitdata*2];
            bitmaskw=bitexpand80[bitdata*2+1];

            graphw=0;

            if(ioport[0xf4]&2) {
                graphw|=bitexpandg2[gvram[gvaddr]*4];
            } 
            if(ioport[0xf4]&4) {
                graphw|=bitexpandg2[gvram[gvaddr+0x2000]*4];
            } 
            
            vramindex=slice_x*2+(slice_y*16+slice_yy*2)*80;

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   
            
            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

            ch=cvram[slice_x*2+slice_y*80+1];

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand80[bitdata*2];
            bitmaskw=bitexpand80[bitdata*2+1];

            graphw=0;

            if(ioport[0xf4]&2) {
                graphw|=bitexpandg2[gvram[gvaddr]*4+1];
            } 
            if(ioport[0xf4]&4) {
                graphw|=bitexpandg2[gvram[gvaddr+0x2000]*4+1];
            } 

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   
            
            *(vramptr+vramindex+1) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8+1) =  bitdataw;
//        }

    } else {  // 40 chars

        slice_x=gvaddr%40;
        slice_y=gvaddr/40;

        uint32_t slice_yy=slice_y%8;
        slice_y=slice_y/8;

        
        ch=cvram[slice_x+slice_y*40];

 //       for(int slice_yy=0;slice_yy<8;slice_yy++) {

//            gvaddr=slice_x+(slice_y*8+slice_yy)*40;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand40[bitdata*4+1];
            bitmaskw=bitexpand40[bitdata*4+3];

            vramindex=slice_x*2+(slice_y*16+slice_yy*2)*80;

            graphw=0;

            if(ioport[0xf4]&2) {
                graphw|=bitexpandg2[gvram[gvaddr]*4];
            } 
            if(ioport[0xf4]&4) {
                graphw|=bitexpandg2[gvram[gvaddr+0x2000]*4];
            }

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   

            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

            bitdataw=bitexpand40[bitdata*4];
            bitmaskw=bitexpand40[bitdata*4+2];

            graphw=0;

            if(ioport[0xf4]&2) {
                graphw|=bitexpandg2[gvram[gvaddr]*4+1];
            } 
            if(ioport[0xf4]&4) {
                graphw|=bitexpandg2[gvram[gvaddr+0x2000]*4+1];
            }

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   

            *(vramptr+vramindex+1) = bitdataw;
            *(vramptr+vramindex+1+VGA_PIXELS_X/8) =  bitdataw;

//        }
    }
}

// MZ-2000 
// graphic draw
static void draw_framebuffer_gv20(uint16_t addr) {

    uint32_t slice_x,slice_y;
    uint16_t offset,gvaddr,cvaddr;
    uint32_t vramindex;
    uint32_t ch,color,bgcolor,bitdata;
    uint32_t bitdataw,bitmaskw;
    
    uint32_t graphbw,graphrw,graphgw,graphmaskw,graphw,mixmaskw;

    gvaddr=addr&0x3fff;

    color=ioport[0xf5]&7;
    bgcolor=ioport[0xf4]&7;

    // if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
    //     return;
    // }    

    if(gvaddr>=0x3e80) {  // out of screen
           return;
    }

    uint32_t *vramptr=(uint32_t *)vga_data_array;

    if(ioport[0xe8]&0x20) { // 80 chars

        slice_x=gvaddr%80;
        slice_y=gvaddr/80;

        uint32_t slice_yy=slice_y%8;
        slice_y=slice_y/8;

        ch=cvram[slice_x+slice_y*80];

        // for(int slice_yy=0;slice_yy<8;slice_yy++) {
        //     gvaddr=slice_x+(slice_y*8+slice_yy)*80;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand80[bitdata*2];
            bitmaskw=bitexpand80[bitdata*2+1];

            graphw=0;

            if(ioport[0xf6]&1) {
                graphw|=bitexpandg[gvram[gvaddr]*2];
            } 
            if(ioport[0xf6]&2) {
                graphw|=bitexpandg[gvram[gvaddr+0x4000]*2];
            } 
            if(ioport[0xf6]&4) {
                graphw|=bitexpandg[gvram[gvaddr+0x8000]*2];               
            } 
            vramindex=slice_x+(slice_y*16+slice_yy*2)*80;

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   

            vramindex=slice_x+(slice_y*16+slice_yy*2)*80;

            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

        // }

    } else {  // 40 chars

        gvaddr&=0x3ffe;

        slice_x=gvaddr%80;
        slice_y=gvaddr/80;

        uint32_t slice_yy=slice_y%8;
        slice_y=slice_y/8;
        slice_x=slice_x/2;

        ch=cvram[slice_x+slice_y*40];

    // printf("[%04x,%d,%d]",gvaddr,slice_x,slice_y);

        // for(int slice_yy=0;slice_yy<8;slice_yy++) {

        //     gvaddr=slice_x*2+(slice_y*8+slice_yy)*80;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand40[bitdata*4+1];
            bitmaskw=bitexpand40[bitdata*4+3];

            vramindex=slice_x*2+(slice_y*16+slice_yy*2)*80;


            graphw=0;

            if(ioport[0xf6]&1) {
                graphw|=bitexpandg[gvram[gvaddr]*2];
            }
            if(ioport[0xf6]&2) {
                graphw|=bitexpandg[gvram[gvaddr+0x4000]*2];
            }
            if(ioport[0xf6]&4) {
                graphw|=bitexpandg[gvram[gvaddr+0x8000]*2];
            }

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }   

            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

            bitdataw=bitexpand40[bitdata*4];
            bitmaskw=bitexpand40[bitdata*4+2];

            graphw=0;

            if(ioport[0xf6]&1) {
                graphw|=bitexpandg[gvram[gvaddr+1]*2];
            }
            if(ioport[0xf6]&2) {
                graphw|=bitexpandg[gvram[gvaddr+0x4000+1]*2];
            }
            if(ioport[0xf6]&4) {
                graphw|=bitexpandg[gvram[gvaddr+0x8000+1]*2];
            }

            bitdataw|=graphw;

            if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
                bitdataw*=15;
                bitdataw=~bitdataw;
                bitdataw&=0x44444444;
            } else {
                bitdataw*=4;
            }  

            *(vramptr+vramindex+1) = bitdataw;
            *(vramptr+vramindex+1+VGA_PIXELS_X/8) =  bitdataw;

        // }

    }

}

// MZ-2200 
// graphic draw
static void draw_framebuffer_gv22(uint16_t addr) {

    uint32_t slice_x,slice_y;
    uint16_t offset,gvaddr,cvaddr;
    uint32_t vramindex;
    uint32_t ch,color,bgcolor,bitdata;
    uint32_t bitdataw,bitmaskw;
    
    uint32_t graphbw,graphrw,graphgw,graphmaskw,graphw,mixmaskw;

    gvaddr=addr&0x3fff;

    color=ioport[0xf5]&7;
    bgcolor=ioport[0xf4]&7;

    if((ioport[0xe0]&0x10)==0) {  // BW Reverse mode 
        return;
    }    

    if(gvaddr>=0x3e80) {  // out of screen
           return;
    }

    uint32_t *vramptr=(uint32_t *)vga_data_array;

    if(ioport[0xe8]&0x20) { // 80 chars

        slice_x=gvaddr%80;
        slice_y=gvaddr/80;

        uint32_t slice_yy=slice_y%8;
        slice_y=slice_y/8;

        ch=cvram[slice_x+slice_y*80];

        // for(int slice_yy=0;slice_yy<8;slice_yy++) {
        //     gvaddr=slice_x+(slice_y*8+slice_yy)*80;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand80[bitdata*2];
            bitmaskw=bitexpand80[bitdata*2+1];

            graphmaskw=0x11111111;

            if(ioport[0xf6]&1) {
                graphbw=bitexpandg[gvram[gvaddr]*2];
                graphmaskw=bitexpandg[gvram[gvaddr]*2+1];
            } else {
                graphbw=0;
                graphmaskw=0x11111111;
            }
            if(ioport[0xf6]&2) {
                graphrw=bitexpandg[gvram[gvaddr+0x4000]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x4000]*2+1];
            } else {
                graphrw=0;
            }
            if(ioport[0xf6]&4) {
                graphgw=bitexpandg[gvram[gvaddr+0x8000]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x8000]*2+1];
            } else {
                graphgw=0;
            }

            vramindex=slice_x+(slice_y*16+slice_yy*2)*80;

            graphw = graphbw + graphrw * 2 + graphgw * 4;
            mixmaskw = bitmaskw & graphmaskw;

            if(ioport[0xf5]&8) {

                graphmaskw*=15;
                bitdataw = ((bitdataw * color) & graphmaskw) +graphw  + mixmaskw*bgcolor;
            
            } else {

                bitmaskw*=15;
                bitdataw = bitdataw * color + (graphw & bitmaskw) + mixmaskw*bgcolor;
            
            }

            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

        // }

    } else {  // 40 chars

        gvaddr&=0x3ffe;

        slice_x=gvaddr%80;
        slice_y=gvaddr/80;

        uint32_t slice_yy=slice_y%8;
        slice_y=slice_y/8;
        slice_x=slice_x/2;

        ch=cvram[slice_x+slice_y*40];

    // printf("[%04x,%d,%d]",gvaddr,slice_x,slice_y);

        // for(int slice_yy=0;slice_yy<8;slice_yy++) {

        //     gvaddr=slice_x*2+(slice_y*8+slice_yy)*80;

            bitdata=mzfont[ch*8+slice_yy];
            bitdataw=bitexpand40[bitdata*4+1];
            bitmaskw=bitexpand40[bitdata*4+3];

            vramindex=slice_x*2+(slice_y*16+slice_yy*2)*80;

            graphmaskw=0x11111111;

            if(ioport[0xf6]&1) {
                graphbw=bitexpandg[gvram[gvaddr]*2];
                graphmaskw=bitexpandg[gvram[gvaddr]*2+1];
            } else {
                graphbw=0;
                graphmaskw=0x11111111;
            }
            if(ioport[0xf6]&2) {
                graphrw=bitexpandg[gvram[gvaddr+0x4000]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x4000]*2+1];
            } else {
                graphrw=0;
            }
            if(ioport[0xf6]&4) {
                graphgw=bitexpandg[gvram[gvaddr+0x8000]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x8000]*2+1];
            } else {
                graphgw=0;
            }

            graphw = graphbw + graphrw * 2 + graphgw * 4;
            mixmaskw = bitmaskw & graphmaskw;

            if(ioport[0xf5]&8) {

                graphmaskw*=15;
                bitdataw = ((bitdataw * color) & graphmaskw) +graphw  + mixmaskw*bgcolor;
            
            } else {

                bitmaskw*=15;
                bitdataw = bitdataw * color + (graphw & bitmaskw) + mixmaskw*bgcolor;
            
            }

            *(vramptr+vramindex) = bitdataw;
            *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw;

            bitdataw=bitexpand40[bitdata*4];
            bitmaskw=bitexpand40[bitdata*4+2];

            if(ioport[0xf6]&1) {
                graphbw=bitexpandg[gvram[gvaddr+1]*2];
                graphmaskw=bitexpandg[gvram[gvaddr+1]*2+1];
            } else {
                graphbw=0;
                graphmaskw=0x11111111;
            }
            if(ioport[0xf6]&2) {
                graphrw=bitexpandg[gvram[gvaddr+0x4000+1]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x4000+1]*2+1];
            } else {
                graphrw=0;
            }
            if(ioport[0xf6]&4) {
                graphgw=bitexpandg[gvram[gvaddr+0x8000+1]*2];
                graphmaskw&=bitexpandg[gvram[gvaddr+0x8000+1]*2+1];
            } else {
                graphgw=0;
            }

            graphw = graphbw + graphrw * 2 + graphgw * 4;
            mixmaskw = bitmaskw & graphmaskw;

            if(ioport[0xf5]&8) {

                graphmaskw*=15;
                bitdataw = ((bitdataw * color) & graphmaskw) +graphw  + mixmaskw*bgcolor;
            
            } else {

                bitmaskw*=15;
                bitdataw = bitdataw * color + (graphw & bitmaskw) + mixmaskw*bgcolor;
            
            }

            *(vramptr+vramindex+1) = bitdataw;
            *(vramptr+vramindex+1+VGA_PIXELS_X/8) =  bitdataw;

        // }

    }

}

static void draw_framebuffer(uint16_t addr) {

    if(machinetype==0) {
        if(videotype==0) {
            draw_framebuffer22(addr);
        } else {
            draw_framebuffer20(addr);
        }
    } else {
        draw_framebuffer80(addr);
    }

}

static void draw_framebuffer_gv(uint16_t addr) {

    if(machinetype==0) {
        if(videotype==0) {
            draw_framebuffer_gv22(addr);
        } else {
            draw_framebuffer_gv20(addr);
        }
    } else {
        draw_framebuffer_gv80(addr);
    }

}

static inline void redraw(){

    if((machinetype==0)&&(videotype==0)&&((ioport[0xe0]&0x10)==0)) {
        video_cls();
        return;
    }

    if(ioport[0xe8]&0x20) {
        for(int i=0xd000;i<0xd7d0;i++) {
            draw_framebuffer(i);
        }
    } else {
        for(int i=0xd000;i<0xd3e8;i++) {
            draw_framebuffer(i);
        }        
    }

}

//----------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------

static inline unsigned char tohex(int b) {

    if(b==0) {
        return '0';
    } 
    if(b<10) {
        return b+'1'-1;
    }
    if(b<16) {
        return b+'a'-10;
    }

    return -1;

}

static inline unsigned char fromhex(int b) {

    if(b=='0') {
        return 0;
    } 
    if((b>='1')&&(b<='9')) {
        return b-'1'+1;
    }
    if((b>='a')&&(b<='f')) {
        return b-'a'+10;
    }

    return -1;

}

// LittleFS

int pico_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t fs_start = XIP_BASE + HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] READ: %p, %d\n", addr, size);
    
    memcpy(buffer, (unsigned char *)addr, size);
    return 0;
}

int pico_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] WRITE: %p, %d\n", addr, size);
        
    uint32_t ints = save_and_disable_interrupts();
    multicore_lockout_start_blocking();     // pause another core
    flash_range_program(addr, (const uint8_t *)buffer, size);
    multicore_lockout_end_blocking();
    restore_interrupts(ints);
    
    return 0;
}

int pico_erase(const struct lfs_config *c, lfs_block_t block)
{           
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t offset = fs_start + (block * c->block_size);
    
//    printf("[FS] ERASE: %p, %d\n", offset, block);
        
    uint32_t ints = save_and_disable_interrupts();   
    multicore_lockout_start_blocking();     // pause another core
    flash_range_erase(offset, c->block_size);  
    multicore_lockout_end_blocking();
    restore_interrupts(ints);

    return 0;
}

int pico_sync(const struct lfs_config *c)
{
    return 0;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config PICO_FLASH_CFG = {
    // block device operations
    .read  = &pico_read,
    .prog  = &pico_prog,
    .erase = &pico_erase,
    .sync  = &pico_sync,

    // block device configuration
    .read_size = FLASH_PAGE_SIZE, // 256
    .prog_size = FLASH_PAGE_SIZE, // 256
    
    .block_size = BLOCK_SIZE_BYTES, // 4096
    .block_count = HW_FLASH_STORAGE_BYTES / BLOCK_SIZE_BYTES, // 352
    .block_cycles = 16, // ?
    
    .cache_size = FLASH_PAGE_SIZE, // 256
    .lookahead_size = FLASH_PAGE_SIZE,   // 256    
};



//  Keyboard

static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}

void process_kbd_report(hid_keyboard_report_t const *report) {

    int usbkey;
    uint32_t keycode;
    uint8_t row,col;

    if(menumode==0) { // Emulator mode

        for(int i=0;i<16;i++) {
            keymatrix_bak[i]=keymatrix[i];
            keymatrix[i]=0xff;
        }

        if((report->modifier)&0x22) { // Shift
            keymatrix[0xb]&= ~0x04;
        }

        if((report->modifier)&0x44) { // alt = Graph
            keymatrix[0xb]&= ~0x01;
        }

        // if((report->modifier)&0x11) { // ctrl
        //     keymatrix[0]&= ~0x40;
        // }

        for(int i=0;i<6;i++) {
            keycode=report->keycode[i];
            row=mzusbcode[keycode*2+1];
            if(mzusbcode[keycode*2]) {
                keymatrix[row]&= ~mzusbcode[keycode*2];
            }

            // Enter Menu
            if(report->keycode[i]==0x45) {
                prev_report=*report;
                menumode=1;
                keypressed=0;
            }  
        }

    // keyboard Interrupt
    // piob[3]: mask

//    if((piob[2]&0x80)&&((piob[3]&0x10)==0)) {
//        piob_enable_irq=1;
//    }


    prev_report=*report;

} else {  // menu mode

    for(uint8_t i=0; i<6; i++)
    {
        if ( report->keycode[i] )
        {
        if ( find_key_in_report(&prev_report, report->keycode[i]) )
        {
            // exist in previous report means the current key is holding
        }else
        {
            keypressed=report->keycode[i];
        }
        }
    } 
    prev_report = *report;
    }

}

//static byte mem_read(size_t param, ushort address)
static uint8_t mem_read(void *context,uint16_t address)
{

    uint8_t b;
    static uint32_t lastclocks;

    if(address<0x0800) {
        if(rombank) {
            return mainram[address];
        } else {
            if(machinetype) {
                return mzipl0[address];
            } else {
                return mzipl[address];
            }
        }
    }
    if(address<0x8000) {

        // MZ-80B mode
        if(machinetype==1) {
            if(ioport[0xe8]&0xc0) {

                if((address>=0x5000)&&(address<0x5800)) {
                    return cvram[address-0x5000];
                } else if(address>=0x6000) {
                    if(ioport[0xf4]&1) {
                        return gvram[address-0x6000+0x2000];
                    } else {
                        return gvram[address-0x6000];
                    }
                }


            }
        }

        if(rombank==0) {   
                return 0xff;
            } else {
                return mainram[address];
        }
    }

        if(ioport[0xe8]&0x80) {
            if((machinetype)&&((ioport[0xe8]&0x40)==0)) {  // 80B
                if((address>=0xd000)&&(address<0xd800)) {
                    return cvram[address-0xd000];
                }
                if(address>=0xe000) {
                    if(ioport[0xf4]&1) {
                        return gvram[address-0xe000+0x2000];
                    } else {
                        return gvram[address-0xe000];
                    }
                }

            }
            if(ioport[0xe8]&0x40) {
//                if((address>=0xd000)&&(address<0xd800)) {
                if((address>=0xd000)&&(address<0xd800)) {
                    return cvram[address-0xd000];
                }
            } else {
                if(address>=0xc000) {
                    switch (ioport[0xf7]&3) {
                        case 1:
                            return gvram[address-0xc000];
                        case 2:
                            return gvram[address-0xc000+0x4000];
                        case 3:
                            return gvram[address-0xc000+0x8000];
                        defaut:
                            return 0xff;
                    } 
                }

            }
        }

        if(rombank) {
            return mainram[address];
        } else {
            return mainram[address-0x8000];
        }

//        printf("[!%04x]",address);
        return 0xff;

}


//static void mem_write(size_t param, ushort address, byte data)
static void mem_write(void *context,uint16_t address, uint8_t data)
{

    uint8_t b;
    uint16_t addr;
    static uint32_t lastclocks;

    if((address<0x0800)&&(rombank)) {
        mainram[address]=data;
        return;
    }
    if(address<0x8000) {
        if(rombank) {
            if(machinetype==1) { // 80B
                if((ioport[0xe8]&0x40)) {
                    if((address>=0x5000)&&(address<0x5800)) {
                        cvram[address-0x5000]=data;
                        draw_framebuffer(address);
                        return;
                    }
                    if(address>=0x6000) {
                        if(ioport[0xf4]&1) {
                            gvram[address-0x6000+0x2000]=data;
                            if(ioport[0xf4]&4) draw_framebuffer_gv(address);
                            return;
                        } else {
                            gvram[address-0x6000]=data;
                            if(ioport[0xf4]&2) draw_framebuffer_gv(address);
                            return;
                        }
                    }
                }
            }
            mainram[address]=data;
            return;
        }
    }
    if(ioport[0xe8]&0x80) {
        if(machinetype==0) {  // 2000
            if(ioport[0xe8]&0x40) {
//                if((address>=0xd000)&&(address<0xd800)) {
                if((address>=0xd000)&&(address<0xe000)) {
                    cvram[address-0xd000]=data;
                    draw_framebuffer(address);
                    return;
                }

            } else {

                if(address>=0xc000) {
                    switch(ioport[0xf7]&3) {
                        case 1:
                            gvram[address-0xc000]=data;
                            if(ioport[0xf6]&1) draw_framebuffer_gv(address);
                            return;
                        case 2:
                            gvram[address-0xc000+0x4000]=data;
                            if(ioport[0xf6]&2) draw_framebuffer_gv(address);
                            return;
                        case 3:
                            gvram[address-0xc000+0x8000]=data;
                            if(ioport[0xf6]%4) draw_framebuffer_gv(address);
                            return;
                        default:
                            return;
                    }
                }
            }
        } else { // 80B
            if((ioport[0xe8]&0x40)==0) {
                if((address>=0xd000)&&(address<0xd800)) {
                    cvram[address-0xd000]=data;
                    draw_framebuffer(address);
                    return;
                }
                if(address>=0xe000) {
                    if(ioport[0xf4]&1) {
                        gvram[address-0xe000+0x2000]=data;
                        if(ioport[0xf4]&4) draw_framebuffer_gv(address);
                        return;
                    } else {
                        gvram[address-0xe000]=data;
                        if(ioport[0xf4]&2) draw_framebuffer_gv(address);
                        return;
                    }
                }
            }
        }
    }

    if(rombank) {
        mainram[address]=data;
        return;
    } else {
        mainram[address-0x8000]=data;
        return;
    }

//    printf("[$%04d]",address);
//    mainram[address] = data;

}

//static byte io_read(size_t param, ushort address)
static uint8_t io_read(void *context, uint16_t address)
{
    uint8_t data = ioport[address&0xff];
    uint8_t b;
    uint32_t addr;
    uint32_t kanji_addr;

    switch(address&0xff) {

        case 0xe1:

            b=0xf9;

            if(tape_ready&&(keymatrix[3]&0x80)==0) {
                    b&=0x7f;
            }

            if((tape_ready)&&(load_enabled)) {
                // if tape is loading
                if(tapein()) {
                    b|=0x40;
                } else {
                    b&=0xbf;
                }

            } else {
                b&=0xbf;
            }

            if(load_enabled||save_enabled) {  // tape set flag
                b&=0xdf;
            }

            if(tape_ready) {  // tape running flag
                b&=0xf7;
            }

            if(save_enabled) {  // Write enable flag
                b&=0xef;
            }

            if(video_hsync) {
                b&=0xfe;
            }

            return b;

        case 0xe4:
        case 0xe5:
        case 0xe6:

            addr=(address&0xff);

            if(i8253_latch_flag) {

                if(i8253_access[addr-0xe4]) {
                    i8253_access[addr-0xe4]=0;
                    i8253_latch_flag=0;
                    return (i8253_latch[addr-0xe4]&0xff00)>>8;
                } else {
                    i8253_access[addr-0xe4]=1;
                    return i8253_latch[addr-0xe4]&0xff;
                }

            } else {

                if(i8253_access[addr-0xe4]) {
                    i8253_access[addr-0xe4]=0;
                    return (i8253_counter[addr-0xe4]&0xff00)>>8;
                } else {
                    i8253_access[addr-0xe4]=1;
                    return i8253_counter[addr-0xe4]&0xff;
                }

            }

        case 0xea: // PIOB (keymatrix)

            if(ioport[0xe8]&0x10) {

                return keymatrix[ioport[0xe8]&0xf];

            } else {

                b=0xff;
                for(int i=0;i<16;i++) {
                    b&=keymatrix[i];
                }

                return b;
//                return 0;
            }

#ifdef USE_KANJI
        case 0xb9:

        if(ioport[0xb8]&0x80) { // KANJI ROM
            b=mzkanji[kanji_ptr++];
        } else {  // JISHO ROM
            kanji_addr=(jisho_ptr++)|(ioport[0xb8]&3)<<16;
            b=mzjisho[kanji_addr];
        }

        if(ioport[0xb8]&0x40) { // Change Endian

            uint8_t rb=0;

            for(int i=0;i<8;i++) {
                if(b&(1<<i)){
                    rb|=(1<<(7-i));
                }
            }   
            b=rb;
        }

        return b;

#endif

         case 0xd8: // FDC is absent.
         case 0xd9:

             return 0xff;



        default:

            break;

    }



  return data;

}

//static void io_write(size_t param, ushort address, byte data)
static void io_write(void *context, uint16_t address, uint8_t data)
{

  uint8_t addr,b;
  uint32_t kanji_addr;

  addr=address&0xff;

    //  if(addr!=0xe7) {
    //      printf("[IOW:%04x,%02x,%02x]",cpu.PC,addr,data);
    //  }

  switch(addr) {

        case 0xe0:

            if(data&0x8) {
                if((ioport[0xe0]&0x8)==0) {
                        tape_ready=0;

                        printf("[Motor:OFF]");

                }                
            }

            if(data&0x4) {
                if((ioport[0xe0]&0x4)==0) {
                        tape_ready=1;

                        printf("[Motor:ON]");

                }
            }

            ioport[0xe0]=data;

            if(data&0x10) {
                redraw();
            } else {
                if((machinetype==0)&&(videotype==0)) {
                    video_cls();
                } else {
                    redraw();
                }
            }

            return;

        case 0xe2:

            if((data&0x8)==0) {
                rombank=0;
                if(ioport[0xe2]&8) {
                    printf("[IPL]");
//                    Z80RESET(&cpu);
                    z80_instant_reset(&cpu);
                }

            }

            if(data&0x2) {
                rombank=1;
                if((ioport[0xe2]&2)==0) {
                    printf("[RAM]");
//                    Z80RESET(&cpu);  
                    z80_instant_reset(&cpu);
                }
      
            }

            if(data&0x4) {
                gpio_put(6,1);
            } else {
                gpio_put(6,0);
            }

            ioport[0xe2]=data;

            break;

        case 0xe3:

            if((data&0x80)==0) { // Bit operation

                b=(data&0x0e)>>1;

                if(b==7) {  // Tape Out
                    tapeout(data&1);
                                // printf("[%d/%d]\n\r",(data&1),cpu_clocks-lastclocks);
                                // lastclocks=cpu_clocks;
                }

                if(b==3) {
                    if((data&1)==0) {
                        rombank=0; 
                    }

                    if(ioport[0xe2]&8) {
                        printf("[IPL]");

//                        Z80RESET(&cpu);   
                        z80_instant_reset(&cpu);
                    }
                }

                if(b==1) {
                    if(data&1) {
                        rombank=1; 
                    }

                    if((ioport[0xe2]&2)==0) {
                        printf("[RAM]");
//                        Z80RESET(&cpu);  
                        z80_instant_reset(&cpu);
                    }
                }

                if(b==2) {
                    if(data&1) {
                        gpio_put(6,1);
                    } else {
                        gpio_put(6,0);
                    }
                }



                if(data&1) {
                    ioport[0xe2]|= 1<<b;
                } else {
                    ioport[0xe2]&= ~(1<<b);
                }

            }

            return;

        case 0xe4:
        case 0xe5:
        case 0xe6:

            if((i8253[addr-0xe4]&0x30)==0x30) {

                if(i8253_access[addr-0xe4]) {

                    i8253_preload[addr-0xe4]&=0xff;
                    i8253_preload[addr-0xe4]|=data<<8;
                    i8253_access[addr-0xe4]=0;
                    i8253_counter[addr-0xe4]=i8253_preload[addr-0xe4];

                } else {

                    i8253_preload[addr-0xe4]&=0xff00;
                    i8253_preload[addr-0xe4]|=data;
                    i8253_access[addr-0xe4]=2;
                }
            }

            if((i8253[addr-0xe4]&0x30)==0x20) {
                i8253_preload[addr-0xe4]&=0xff;
                i8253_preload[addr-0xe4]|=data<<8;
                i8253_counter[addr-0xe4]=i8253_preload[addr-0xe4];
            }

            if((i8253[addr-0xe4]&0x30)==0x10) {
                i8253_preload[addr-0xe4]&=0xff00;
                i8253_preload[addr-0xe4]|=data;
                i8253_counter[addr-0xe4]=i8253_preload[addr-4];

            }

//                        i8253_counter[addr-4]=i8253_preload[addr-4];

            i8253_pending[addr-0xe4]=1;

            break;

        case 0xe7:  // i8253 control

            b=(data&0xc0)>>6;

            if(b!=3) {
                if((data&0x30)==0) {
                    i8253_latch[b]=i8253_counter[b];
                    i8253_latch_flag=1;
                }
                i8253[b]=data;
            }

            break;

#ifdef USE_KANJI
    case 0xb9:  // Set Kanji ptr

    kanji_addr=(address&0xff00)+data;

//    printf("[KANJI:%x,%x,%x]",kanji_addr,address,data);

    if(ioport[0xb8]&0x80) { // KANJI ROM
        kanji_ptr=kanji_addr<<5;
    } else {
        jisho_ptr=kanji_addr<<5;
    }
#endif

        case 0xe9: // PIOA

            if(pioa_next_mask) {
                pioa[3]=data;
                pioa_next_mask=0;
                return;
            }
            if(pioa_next_iocontrol) {
                pioa_next_iocontrol=0;
                return;
            }

            switch(data&0xf) {

                case 0:
                case 2:
                case 4:
                case 8:
                case 0xa:
                case 0xc:
                case 0xe:

                    pioa[0]=data;
                    return;
                
                case 3: // Intrrupt disable

                    if(data&0x80) {
                        pioa[2]|=0x80;
                    } else {
                        pioa[2]&=0x7f;
                    }

                    return;


                case 7: // Interrupt control

                    pioa[2]=data;
                    if(data&0x10) {
                        pioa_next_mask=1;
                    }
                    return;

                case 0xf: // Mode control

                    if((data&0xc0)==0xc0) { // Mode 3
                        pioa_next_iocontrol=1;
                    }


                default:                

                    return;
            }

        case 0xeb: // PIOB

            if(piob_next_mask) {
                piob[3]=data;
                piob_next_mask=0;
                return;
            }
            if(piob_next_iocontrol) {
                piob_next_iocontrol=0;
                return;
            }


            switch(data&0xf) {

                case 0:
                case 2:
                case 4:
                case 8:
                case 0xa:
                case 0xc:
                case 0xe:

                    piob[0]=data;
                    return;
                
                case 3: // Intrrupt disable

                    if(data&0x80) {
                        piob[2]|=0x80;
                    } else {
                        piob[2]&=0x7f;
                    }

                    return;

                case 7: // Interrupt control

                    piob[2]=data;
                    if(data&0x10) {
                        piob_next_mask=1;
                    }
                    return;

                case 0xf: // Mode control

                    if((data&0xc0)==0xc0) { // Mode 3
                        piob_next_iocontrol=1;
                    }

                default:                

                    return;
            }


    case 0xf4:
    case 0xf5:
    case 0xf6:
        ioport[addr]=data;
        redraw();
        break;

    default:

        ioport[addr]=data;
    return;

  }



}


void main_core1(void) {

    uint32_t redraw_start,redraw_length;

    multicore_lockout_victim_init();

    scanline=0;

    // set virtual Hsync timer

    add_repeating_timer_us(63,hsync_handler,NULL  ,&timer);

    while(1) { // Wait framebuffer redraw command


    }
}

int main() {

    uint32_t menuprint=0;
    uint32_t filelist=0;
    uint32_t subcpu_wait;

    static uint32_t hsync_wait,vsync_wait;

    set_sys_clock_khz(DOTCLOCK * CLOCKMUL ,true);

    stdio_init_all();

    uart_init(uart0, 115200);

    initVGA();

    gpio_set_function(12, GPIO_FUNC_UART);
    gpio_set_function(13, GPIO_FUNC_UART);

    // gpio_set_slew_rate(0,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(1,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(2,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(3,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(4,GPIO_SLEW_RATE_FAST);

    gpio_set_drive_strength(2,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(3,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(4,GPIO_DRIVE_STRENGTH_2MA);

    // Beep & PSG

    gpio_init(6);
    gpio_set_dir(6,GPIO_OUT);
    gpio_put(6,0);

    // set PSG timer

    tuh_init(BOARD_TUH_RHPORT);


    video_cls();

    video_hsync=0;
    video_vsync=0;

    video_mode=0;
    fbcolor=0x7;

// uart handler

    // irq_set_exclusive_handler(UART0_IRQ,uart_handler);
    // irq_set_enabled(UART0_IRQ,true);
    // uart_set_irq_enables(uart0,true,false);

    multicore_launch_core1(main_core1);

    multicore_lockout_victim_init();

    sleep_ms(1);

// mount littlefs
    if(lfs_mount(&lfs,&PICO_FLASH_CFG)!=0) {
       cursor_x=0;
       cursor_y=0;
       fbcolor=7;
       video_print("Initializing LittleFS...");
       // format
       lfs_format(&lfs,&PICO_FLASH_CFG);
       lfs_mount(&lfs,&PICO_FLASH_CFG);
   }

    menumode=1;  // Pause emulator

//  setup emulator 
// QD Init

    // qd_motor=0;
    // qd_ptr=0;
    // siob[5]=0;

    for(int i=0;i<16;i++) {
        keymatrix[i]=0xff;
    }

    i8253_preload[0]=0;
    tape_ready=0;

    ioport[0xe0]=0xff;

    // cpu.memRead = mem_read;
    // cpu.memWrite = mem_write;
    // cpu.ioRead = io_read;
    // cpu.ioWrite = io_write;

    cpu.read = mem_read;
    cpu.write = mem_write;
    cpu.in = io_read;
    cpu.out = io_write;
	cpu.fetch = mem_read;
    cpu.fetch_opcode = mem_read;

    rombank=0;

//    Z80RESET(&cpu);
    z80_power(&cpu,true);
    z80_instant_reset(&cpu);

    uint32_t cpuwait=0;

    // start emulator

    menumode=0;

    while(1) {

        if(menumode==0) { // Emulator mode

        exec_reti=0;

        // cursor_x=0;
        // cursor_y=24;
        // fbcolor=0x07;

        // unsigned char str[40];
        // sprintf(str,"%04x",cpu.PC);
        // video_print(str);

//        if(cputrace) {
//            printf("[%04x:%02x:%04x]",cpu.PC,cpu.R1.br.A,cpu.R1.wr.HL);
//        }
        // if(cputrace) {
        //     printf("[%04x:%02x:%04x]",Z80_PC(cpu),Z80_A(cpu),Z80_HL(cpu));
        // }

//        Z80Execute(&cpu);
        z80_run(&cpu,1);
        cpu_clocks++;

        // Mode 2 Interrupt (MZ-1500)

        if(exec_reti) { // clear irq on Z80PIO/SIO

            if(pio_irq_processing) {
                pio_irq_processing=0;
                pioa_enable_irq=0;
                piob_enable_irq=0;
            }

        }

        if((piob_enable_irq)&&(pio_irq_processing==0)) {

            // if((cpu.IFF1)&&(cpu.IM==2)) { 
            //         Z80INT(&cpu,piob[0]);
            //         pio_irq_processing=1;
                    
            //     }
    
        }

        // if(qd_read==1) {
        //     qd_read==2;
        //     Z80INT(&cpu,siob[2]&0xfe);
        // }


        // Mode 1 Interrupt (MZ-700)

        // if(i8253_enable_irq) {
        //     Z80INT(&cpu,0x38);
        //     i8253_enable_irq=0; 
        // }

        if(video_hsync==1) {
            hsync_wait++;
            if(hsync_wait>30) {
                video_hsync=0;
                hsync_wait=0;
            }
        }
        
        if((video_vsync)==1) { // Timer
            tuh_task();
            video_vsync=2;
            vsync_scanline=scanline;
        }

        if(video_vsync==2) {
            if(scanline>(vsync_scanline+60)) {
                video_vsync=0;
            }
        }

        } else { // Menu Mode


            unsigned char str[80];

            fbcolor=7;
            
            if(menuprint==0) {

                draw_menu();
                menuprint=1;
                filelist=0;
            }

            cursor_x=3;
            cursor_y=6;
            video_print("MENU");

            uint32_t used_blocks=lfs_fs_size(&lfs);
            sprintf(str,"Free:%d Blocks",(HW_FLASH_STORAGE_BYTES/BLOCK_SIZE_BYTES)-used_blocks);
            cursor_x=3;
            cursor_y=7;
            video_print(str);

            sprintf(str,"TAPE:%x",tape_ptr);
            cursor_x=3;
            cursor_y=8;
            video_print(str);

            cursor_x=3;            
            cursor_y=9;
            if(menuitem==0) { fbcolor=0x70; } else { fbcolor=7; } 
            if(save_enabled==0) {
                video_print("SAVE: empty");
            } else {
                sprintf(str,"SAVE: %8s",tape_filename);
                video_print(str);
            }
            cursor_x=3;
            cursor_y=10;

            if(menuitem==1) { fbcolor=0x70; } else { fbcolor=7; } 
            if(load_enabled==0) {
                video_print("LOAD: empty");
            } else {
                sprintf(str,"LOAD: %8s",tape_filename);
                video_print(str);
            }

            cursor_x=3;
            cursor_y=11;

            if(menuitem==2) { fbcolor=0x70; } else { fbcolor=7; } 
//            if(qd_status==0) {
                video_print("Direct Load");
//            } else {
//                sprintf(str,"QD: %8s",qd_filename);
//                video_print(str);
//            }

            cursor_x=3;
            cursor_y=13;

            if(menuitem==3) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("DELETE File");

            cursor_x=3;
            cursor_y=14;

            if(menuitem==4) { fbcolor=0x70; } else { fbcolor=7; } 
            if(videotype==0) {
                video_print("Monitor:Color");
            } else {
                video_print("Monitor:Green");
            }

            cursor_x=3;
            cursor_y=15;

            if(menuitem==5) { fbcolor=0x70; } else { fbcolor=7; } 
            if(machinetype==0) {
                video_print("Mode:MZ-2000");
            } else {
                video_print("Mode:MZ-80B ");
            }

            cursor_x=3;
            cursor_y=16;
            if(menuitem==6) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("Reset");


            cursor_x=3;
            cursor_y=17;

            if(menuitem==7) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("IPL Reset");

// TEST

            // cursor_x=3;
            //  cursor_y=17;
            //      sprintf(str,"%04x %ld %x %x %x %x",cpu.PC,cpu_clocks,i8253[0],i8253_counter[0],i8253[2],i8253_counter[2]);
            //      video_print(str);

//             cursor_x=3;
//              cursor_y=18;
// //                 sprintf(str,"%04x %02x",cpu.PC,mainram[cpu.PC]);
//                     sprintf(str,"%04x",Z80_PC(cpu));
//                  video_print(str);

            if(filelist==0) {
                draw_files(-1,0);
                filelist=1;
            }
     
            while(video_vsync==0);

            video_vsync=0;

                tuh_task();

                if(keypressed==0x52) { // Up
                    keypressed=0;
                    if(menuitem>0) menuitem--;
                }

                if(keypressed==0x51) { // Down
                    keypressed=0;
                    if(menuitem<7) menuitem++; 
                }

                if(keypressed==0x28) {  // Enter
                    keypressed=0;

                    if(menuitem==0) {  // SAVE

                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=enter_filename();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDWR|LFS_O_CREAT);
                                save_enabled=1;
                                tape_phase=0;
                                tape_ptr=0;
                                tape_count=0;
                            }

                        } else if (save_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            save_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==1) { // LOAD

                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDONLY);
                                load_enabled=1;
                                tape_phase=0;
                                tape_ptr=0;
                                tape_count=0;
//                                file_cycle=cpu.PC;
                            }
                        } else if(load_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            load_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==2) { 

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(qd_filename,filename,16);
                                lfs_file_open(&lfs,&qd_drive,qd_filename,LFS_O_RDONLY);
                                mztload();
                                lfs_file_close(&lfs,&qd_drive);

                                ioport[0xe2]|=2;
                                rombank=1;
                                vrambank=0;

//                                Z80RESET(&cpu);
                                z80_instant_reset(&cpu);
                            }

                    }

                    if(menuitem==3) { // Delete

                        if((load_enabled==0)&&(save_enabled==0)) {
                            uint32_t res=enter_filename();

                            if(res==0) {
                                lfs_remove(&lfs,filename);
                            }
                        }

                        menuprint=0;

                    }

                    if(menuitem==4) { // color change
                        menuprint=0;
                        if(machinetype==0) {
                            if(videotype==0) {
                                videotype=1;
                            } else {
                                videotype=0;
                            }
                        }

                    }

                    if(menuitem==5) { // Mode change
                        menuprint=0;
                        if(machinetype==0) {
                            machinetype=1;
                        } else {
                            machinetype=0;
                        }
                    }

                    if(menuitem==6) { // Reset
                        menumode=0;
                        menuprint=0;
                        redraw();
                    
//                        Z80RESET(&cpu);
                        z80_instant_reset(&cpu);
                        // if(cputrace) {
                        //     cputrace=0;
                        // } else {
                        //     cputrace=1;
                        // }

                    }

                    if(menuitem==7) { // PowerCycle
                        menumode=0;
                        menuprint=0;

                        // memset(mainram,0,0x10000);
                        // memset(vram,0,0x1000);
                        // memset(ioport,0,0x100);
                        // memset(memioport,0,0x100);
                        
                        vrambank=0;
                        rombank=0;

                        redraw();

//                        Z80RESET(&cpu);
                        z80_instant_reset(&cpu);

                    }



                }

                if(keypressed==0x45) {
                    keypressed=0;
                    menumode=0;
                    menuprint=0;
                    redraw();
                //  break;     // escape from menu
                }

        }


    }

}
