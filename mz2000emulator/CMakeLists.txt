add_executable(mz2000emulator)

# by default the header is generated into the build dir
pico_generate_pio_header(mz2000emulator ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(mz2000emulator ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(mz2000emulator ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(pio_blink ${CMAKE_CURRENT_LIST_DIR}/blink.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

# Make sure TinyUSB can find tusb_config.h
target_include_directories(mz2000emulator PUBLIC ${CMAKE_CURRENT_LIST_DIR})

#target_sources(mz2000emulator PRIVATE mz2000emulator.c z80.c hid_app.c vga16_graphics.c lfs.c lfs_util.c)
target_sources(mz2000emulator PRIVATE mz2000emulator.c Z80.c hid_app.c vga16_graphics.c lfs.c lfs_util.c)

pico_enable_stdio_usb(mz2000emulator 0)
pico_enable_stdio_uart(mz2000emulator 1)

set(Z80_SHARED_LIBS                 NO  CACHE BOOL "")
set(Z80_WITH_Q                      YES CACHE BOOL "")
set(Z80_WITH_ZILOG_NMOS_LD_A_IR_BUG YES CACHE BOOL "")
set(Z80_USE_LOCAL_HEADER            YES CACHE BOOL "")

target_link_libraries(mz2000emulator PRIVATE pico_stdlib hardware_pio hardware_timer hardware_dma hardware_uart hardware_flash pico_multicore hardware_pwm tinyusb_host tinyusb_board)
pico_add_extra_outputs(mz2000emulator)
