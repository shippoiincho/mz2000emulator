/* Zeta API - Z/hardware/ISA/RISC-V.h
 ______  ______________  ___
|__   / |  ___|___  ___|/   \
  /  /__|  __|   |  |  /  -  \
 /______|_____|  |__| /__/ \__\
Copyright (C) 2006-2024 Manuel Sainz de Baranda y Goñi.
Released under the terms of the GNU Lesser General Public License v3. */

#ifndef Z_hardware_ISA_RISC_V_H
#define Z_hardware_ISA_RISC_V_H

/* Unprivileged Floating-Point CSRs */
#define Z_RISC_V_CSR_FFLAGS	    0x001
#define Z_RISC_V_CSR_FRM	    0x002
#define Z_RISC_V_CSR_FCSR	    0x003

/* Unprivileged Counter/Timers */
#define Z_RISC_V_CSR_CYCLE	    0xC00
#define Z_RISC_V_CSR_TIME	    0xC00
#define Z_RISC_V_CSR_INSTRET	    0xC02
#define Z_RISC_V_CSR_HPMCOUNTER3    0xC03
#define Z_RISC_V_CSR_HPMCOUNTER4    0xC04
#define Z_RISC_V_CSR_HPMCOUNTER5    0xC05
#define Z_RISC_V_CSR_HPMCOUNTER6    0xC06
#define Z_RISC_V_CSR_HPMCOUNTER7    0xC07
#define Z_RISC_V_CSR_HPMCOUNTER8    0xC08
#define Z_RISC_V_CSR_HPMCOUNTER9    0xC09
#define Z_RISC_V_CSR_HPMCOUNTER10   0xC0A
#define Z_RISC_V_CSR_HPMCOUNTER11   0xC0B
#define Z_RISC_V_CSR_HPMCOUNTER12   0xC0C
#define Z_RISC_V_CSR_HPMCOUNTER13   0xC0D
#define Z_RISC_V_CSR_HPMCOUNTER14   0xC0E
#define Z_RISC_V_CSR_HPMCOUNTER15   0xC0F
#define Z_RISC_V_CSR_HPMCOUNTER16   0xC10
#define Z_RISC_V_CSR_HPMCOUNTER17   0xC11
#define Z_RISC_V_CSR_HPMCOUNTER18   0xC12
#define Z_RISC_V_CSR_HPMCOUNTER19   0xC13
#define Z_RISC_V_CSR_HPMCOUNTER20   0xC14
#define Z_RISC_V_CSR_HPMCOUNTER21   0xC15
#define Z_RISC_V_CSR_HPMCOUNTER22   0xC16
#define Z_RISC_V_CSR_HPMCOUNTER23   0xC17
#define Z_RISC_V_CSR_HPMCOUNTER24   0xC18
#define Z_RISC_V_CSR_HPMCOUNTER25   0xC19
#define Z_RISC_V_CSR_HPMCOUNTER26   0xC1A
#define Z_RISC_V_CSR_HPMCOUNTER27   0xC1B
#define Z_RISC_V_CSR_HPMCOUNTER28   0xC1C
#define Z_RISC_V_CSR_HPMCOUNTER29   0xC1D
#define Z_RISC_V_CSR_HPMCOUNTER30   0xC1E
#define Z_RISC_V_CSR_HPMCOUNTER31   0xC1F
#define Z_RISC_V_CSR_CYCLEH	    0xC80
#define Z_RISC_V_CSR_TIMEH	    0xC81
#define Z_RISC_V_CSR_INSTRETH	    0xC82
#define Z_RISC_V_CSR_HPMCOUNTER3H   0xC83
#define Z_RISC_V_CSR_HPMCOUNTER4H   0xC84
#define Z_RISC_V_CSR_HPMCOUNTER5H   0xC85
#define Z_RISC_V_CSR_HPMCOUNTER6H   0xC86
#define Z_RISC_V_CSR_HPMCOUNTER7H   0xC87
#define Z_RISC_V_CSR_HPMCOUNTER8H   0xC88
#define Z_RISC_V_CSR_HPMCOUNTER9H   0xC89
#define Z_RISC_V_CSR_HPMCOUNTER10H  0xC8A
#define Z_RISC_V_CSR_HPMCOUNTER11H  0xC8B
#define Z_RISC_V_CSR_HPMCOUNTER12H  0xC8C
#define Z_RISC_V_CSR_HPMCOUNTER13H  0xC8D
#define Z_RISC_V_CSR_HPMCOUNTER14H  0xC8E
#define Z_RISC_V_CSR_HPMCOUNTER15H  0xC8F
#define Z_RISC_V_CSR_HPMCOUNTER16H  0xC90
#define Z_RISC_V_CSR_HPMCOUNTER17H  0xC91
#define Z_RISC_V_CSR_HPMCOUNTER18H  0xC92
#define Z_RISC_V_CSR_HPMCOUNTER19H  0xC93
#define Z_RISC_V_CSR_HPMCOUNTER20H  0xC94
#define Z_RISC_V_CSR_HPMCOUNTER21H  0xC95
#define Z_RISC_V_CSR_HPMCOUNTER22H  0xC96
#define Z_RISC_V_CSR_HPMCOUNTER23H  0xC97
#define Z_RISC_V_CSR_HPMCOUNTER24H  0xC98
#define Z_RISC_V_CSR_HPMCOUNTER25H  0xC99
#define Z_RISC_V_CSR_HPMCOUNTER26H  0xC9A
#define Z_RISC_V_CSR_HPMCOUNTER27H  0xC9B
#define Z_RISC_V_CSR_HPMCOUNTER28H  0xC9C
#define Z_RISC_V_CSR_HPMCOUNTER29H  0xC9D
#define Z_RISC_V_CSR_HPMCOUNTER30H  0xC9E
#define Z_RISC_V_CSR_HPMCOUNTER31H  0xC9F

/* Supervisor Trap Setup */
#define Z_RISC_V_CSR_SSTATUS	    0x100
#define Z_RISC_V_CSR_SIE	    0x104
#define Z_RISC_V_CSR_STVEC	    0x105
#define Z_RISC_V_CSR_SCOUNTEREN	    0x106

/* Supervisor Configuration */
#define Z_RISC_V_CSR_SENVCFG	    0x10A

/* Supervisor Trap Handling */
#define Z_RISC_V_CSR_SSCRATCH	    0x140
#define Z_RISC_V_CSR_SEPC	    0x141
#define Z_RISC_V_CSR_SCAUSE	    0x142
#define Z_RISC_V_CSR_STVAL	    0x143
#define Z_RISC_V_CSR_SIP	    0x144

/* Supervisor Protection and Translation */
#define Z_RISC_V_CSR_SATP	    0x180

/* Debug/Trace Registers */
#define Z_RISC_V_CSR_SCONTEXT	    0x5A8

/* Hypervisor Trap Setup */
#define Z_RISC_V_CSR_HSTATUS	    0x600
#define Z_RISC_V_CSR_HEDELEG	    0x602
#define Z_RISC_V_CSR_HIDELEG	    0x603
#define Z_RISC_V_CSR_HIE	    0x604
#define Z_RISC_V_CSR_HCOUNTEREN	    0x606
#define Z_RISC_V_CSR_HGEIE	    0x607

/* Hypervisor Trap Handling */
#define Z_RISC_V_CSR_HTVAL	    0x643
#define Z_RISC_V_CSR_HIP	    0x644
#define Z_RISC_V_CSR_HVIP	    0x645
#define Z_RISC_V_CSR_HTINST	    0x64A
#define Z_RISC_V_CSR_HGEIP	    0xE12

/* Hypervisor Configuration */
#define Z_RISC_V_CSR_HENVCFG	    0x60A
#define Z_RISC_V_CSR_HENVCFGH	    0x61A

/* Hypervisor Protection and Translation */
#define Z_RISC_V_CSR_HGATP	    0x680

/* Debug/Trace Registers */
#define Z_RISC_V_CSR_hcontext	    0x6A8

/* Hypervisor Counter/Timer Virtualization Registers */
#define Z_RISC_V_CSR_htimedelta	    0x605
#define Z_RISC_V_CSR_htimedeltah    0x615

/* Virtual Supervisor Registers */
#define Z_RISC_V_CSR_VSSTATUS	    0x200
#define Z_RISC_V_CSR_VSIE	    0x204
#define Z_RISC_V_CSR_VSTVEC	    0x205
#define Z_RISC_V_CSR_VSSCRATCH	    0x240
#define Z_RISC_V_CSR_VSEPC	    0x241
#define Z_RISC_V_CSR_VSCAUSE	    0x242
#define Z_RISC_V_CSR_VSTVAL	    0x243
#define Z_RISC_V_CSR_VSIP	    0x244
#define Z_RISC_V_CSR_VSATP	    0x280

/* Machine Information Registers */
#define Z_RISC_V_CSR_MVENDORID	    0xF11
#define Z_RISC_V_CSR_MARCHID	    0xF12
#define Z_RISC_V_CSR_MIMPID	    0xF13
#define Z_RISC_V_CSR_MHARTID	    0xF14
#define Z_RISC_V_CSR_MCONFIGPTR	    0xF15

/* Machine Trap Setup */
#define Z_RISC_V_CSR_MSTATUS	    0x300
#define Z_RISC_V_CSR_MISA	    0x301
#define Z_RISC_V_CSR_MEDELEG	    0x302
#define Z_RISC_V_CSR_MIDELEG	    0x303
#define Z_RISC_V_CSR_MIE	    0x304
#define Z_RISC_V_CSR_MTVEC	    0x305
#define Z_RISC_V_CSR_MCOUNTEREN	    0x306
#define Z_RISC_V_CSR_MSTATUSH	    0x310

/* Machine Trap Handling */
#define Z_RISC_V_CSR_MSCRATCH	    0x340
#define Z_RISC_V_CSR_MEPC	    0x341
#define Z_RISC_V_CSR_MCAUSE	    0x342
#define Z_RISC_V_CSR_MTVAL	    0x343
#define Z_RISC_V_CSR_MIP	    0x344
#define Z_RISC_V_CSR_MTINST	    0x34A
#define Z_RISC_V_CSR_MTVAL2	    0x34B

/* Machine Configuration */
#define Z_RISC_V_CSR_MENVCFG	    0x30A
#define Z_RISC_V_CSR_MENVCFGH	    0x31A
#define Z_RISC_V_CSR_MSECCFG	    0x747
#define Z_RISC_V_CSR_MSECCFGH	    0x757

/* Machine Memory Protection */
#define Z_RISC_V_CSR_PMPCFG0	    0x3A0
#define Z_RISC_V_CSR_PMPCFG1	    0x3A1
#define Z_RISC_V_CSR_PMPCFG2	    0x3A2
#define Z_RISC_V_CSR_PMPCFG3	    0x3A3
#define Z_RISC_V_CSR_PMPCFG4	    0x3A4
#define Z_RISC_V_CSR_PMPCFG5	    0x3A5
#define Z_RISC_V_CSR_PMPCFG6	    0x3A6
#define Z_RISC_V_CSR_PMPCFG7	    0x3A7
#define Z_RISC_V_CSR_PMPCFG8	    0x3A8
#define Z_RISC_V_CSR_PMPCFG9	    0x3A9
#define Z_RISC_V_CSR_PMPCFG10	    0x3AA
#define Z_RISC_V_CSR_PMPCFG11	    0x3AB
#define Z_RISC_V_CSR_PMPCFG12	    0x3AC
#define Z_RISC_V_CSR_PMPCFG13	    0x3AD
#define Z_RISC_V_CSR_PMPCFG14	    0x3AE
#define Z_RISC_V_CSR_PMPCFG15	    0x3AF
#define Z_RISC_V_CSR_PMPADDR0	    0x3B0
#define Z_RISC_V_CSR_PMPADDR1	    0x3B1
#define Z_RISC_V_CSR_PMPADDR2	    0x3B2
#define Z_RISC_V_CSR_PMPADDR3	    0x3B3
#define Z_RISC_V_CSR_PMPADDR4	    0x3B4
#define Z_RISC_V_CSR_PMPADDR5	    0x3B5
#define Z_RISC_V_CSR_PMPADDR6	    0x3B6
#define Z_RISC_V_CSR_PMPADDR7	    0x3B7
#define Z_RISC_V_CSR_PMPADDR8	    0x3B8
#define Z_RISC_V_CSR_PMPADDR9	    0x3B9
#define Z_RISC_V_CSR_PMPADDR10	    0x3BA
#define Z_RISC_V_CSR_PMPADDR11	    0x3BB
#define Z_RISC_V_CSR_PMPADDR12	    0x3BC
#define Z_RISC_V_CSR_PMPADDR13	    0x3BD
#define Z_RISC_V_CSR_PMPADDR14	    0x3BE
#define Z_RISC_V_CSR_PMPADDR15	    0x3BF
#define Z_RISC_V_CSR_PMPADDR16	    0x3C0
#define Z_RISC_V_CSR_PMPADDR17	    0x3C1
#define Z_RISC_V_CSR_PMPADDR18	    0x3C2
#define Z_RISC_V_CSR_PMPADDR19	    0x3C3
#define Z_RISC_V_CSR_PMPADDR20	    0x3C4
#define Z_RISC_V_CSR_PMPADDR21	    0x3C5
#define Z_RISC_V_CSR_PMPADDR22	    0x3C6
#define Z_RISC_V_CSR_PMPADDR23	    0x3C7
#define Z_RISC_V_CSR_PMPADDR24	    0x3C8
#define Z_RISC_V_CSR_PMPADDR25	    0x3C9
#define Z_RISC_V_CSR_PMPADDR26	    0x3CA
#define Z_RISC_V_CSR_PMPADDR27	    0x3CB
#define Z_RISC_V_CSR_PMPADDR28	    0x3CC
#define Z_RISC_V_CSR_PMPADDR29	    0x3CD
#define Z_RISC_V_CSR_PMPADDR30	    0x3CE
#define Z_RISC_V_CSR_PMPADDR31	    0x3CF
#define Z_RISC_V_CSR_PMPADDR32	    0x3D0
#define Z_RISC_V_CSR_PMPADDR33	    0x3D1
#define Z_RISC_V_CSR_PMPADDR34	    0x3D2
#define Z_RISC_V_CSR_PMPADDR35	    0x3D3
#define Z_RISC_V_CSR_PMPADDR36	    0x3D4
#define Z_RISC_V_CSR_PMPADDR37	    0x3D5
#define Z_RISC_V_CSR_PMPADDR38	    0x3D6
#define Z_RISC_V_CSR_PMPADDR39	    0x3D7
#define Z_RISC_V_CSR_PMPADDR40	    0x3D8
#define Z_RISC_V_CSR_PMPADDR41	    0x3D9
#define Z_RISC_V_CSR_PMPADDR42	    0x3DA
#define Z_RISC_V_CSR_PMPADDR43	    0x3DB
#define Z_RISC_V_CSR_PMPADDR44	    0x3DC
#define Z_RISC_V_CSR_PMPADDR45	    0x3DD
#define Z_RISC_V_CSR_PMPADDR46	    0x3DE
#define Z_RISC_V_CSR_PMPADDR47	    0x3DF
#define Z_RISC_V_CSR_PMPADDR48	    0x3E0
#define Z_RISC_V_CSR_PMPADDR49	    0x3E1
#define Z_RISC_V_CSR_PMPADDR50	    0x3E2
#define Z_RISC_V_CSR_PMPADDR51	    0x3E3
#define Z_RISC_V_CSR_PMPADDR52	    0x3E4
#define Z_RISC_V_CSR_PMPADDR53	    0x3E5
#define Z_RISC_V_CSR_PMPADDR54	    0x3E6
#define Z_RISC_V_CSR_PMPADDR55	    0x3E7
#define Z_RISC_V_CSR_PMPADDR56	    0x3E8
#define Z_RISC_V_CSR_PMPADDR57	    0x3E9
#define Z_RISC_V_CSR_PMPADDR58	    0x3EA
#define Z_RISC_V_CSR_PMPADDR59	    0x3EB
#define Z_RISC_V_CSR_PMPADDR60	    0x3EC
#define Z_RISC_V_CSR_PMPADDR61	    0x3ED
#define Z_RISC_V_CSR_PMPADDR62	    0x3EE
#define Z_RISC_V_CSR_PMPADDR63	    0x3EF

/* Machine Counter/Timers */
#define Z_RISC_V_CSR_MCYCLE	    0xB00
#define Z_RISC_V_CSR_MINSTRET	    0xB02
#define Z_RISC_V_CSR_MHPMCOUNTER3   0xB03
#define Z_RISC_V_CSR_MHPMCOUNTER4   0xB04
#define Z_RISC_V_CSR_MHPMCOUNTER5   0xB05
#define Z_RISC_V_CSR_MHPMCOUNTER6   0xB06
#define Z_RISC_V_CSR_MHPMCOUNTER7   0xB07
#define Z_RISC_V_CSR_MHPMCOUNTER8   0xB08
#define Z_RISC_V_CSR_MHPMCOUNTER9   0xB09
#define Z_RISC_V_CSR_MHPMCOUNTER10  0xB0A
#define Z_RISC_V_CSR_MHPMCOUNTER11  0xB0B
#define Z_RISC_V_CSR_MHPMCOUNTER12  0xB0C
#define Z_RISC_V_CSR_MHPMCOUNTER13  0xB0D
#define Z_RISC_V_CSR_MHPMCOUNTER14  0xB0E
#define Z_RISC_V_CSR_MHPMCOUNTER15  0xB0F
#define Z_RISC_V_CSR_MHPMCOUNTER16  0xB10
#define Z_RISC_V_CSR_MHPMCOUNTER17  0xB11
#define Z_RISC_V_CSR_MHPMCOUNTER18  0xB12
#define Z_RISC_V_CSR_MHPMCOUNTER19  0xB13
#define Z_RISC_V_CSR_MHPMCOUNTER20  0xB14
#define Z_RISC_V_CSR_MHPMCOUNTER21  0xB15
#define Z_RISC_V_CSR_MHPMCOUNTER22  0xB16
#define Z_RISC_V_CSR_MHPMCOUNTER23  0xB17
#define Z_RISC_V_CSR_MHPMCOUNTER24  0xB18
#define Z_RISC_V_CSR_MHPMCOUNTER25  0xB19
#define Z_RISC_V_CSR_MHPMCOUNTER26  0xB1A
#define Z_RISC_V_CSR_MHPMCOUNTER27  0xB1B
#define Z_RISC_V_CSR_MHPMCOUNTER28  0xB1C
#define Z_RISC_V_CSR_MHPMCOUNTER29  0xB1D
#define Z_RISC_V_CSR_MHPMCOUNTER30  0xB1E
#define Z_RISC_V_CSR_MHPMCOUNTER31  0xB1F
#define Z_RISC_V_CSR_MCYCLEH	    0xB80
#define Z_RISC_V_CSR_MINSTRETH	    0xB82
#define Z_RISC_V_CSR_MHPMCOUNTER3H  0xB83
#define Z_RISC_V_CSR_MHPMCOUNTER4H  0xB84
#define Z_RISC_V_CSR_MHPMCOUNTER5H  0xB85
#define Z_RISC_V_CSR_MHPMCOUNTER6H  0xB86
#define Z_RISC_V_CSR_MHPMCOUNTER7H  0xB87
#define Z_RISC_V_CSR_MHPMCOUNTER8H  0xB88
#define Z_RISC_V_CSR_MHPMCOUNTER9H  0xB89
#define Z_RISC_V_CSR_MHPMCOUNTER10H 0xB8A
#define Z_RISC_V_CSR_MHPMCOUNTER11H 0xB8B
#define Z_RISC_V_CSR_MHPMCOUNTER12H 0xB8C
#define Z_RISC_V_CSR_MHPMCOUNTER13H 0xB8D
#define Z_RISC_V_CSR_MHPMCOUNTER14H 0xB8E
#define Z_RISC_V_CSR_MHPMCOUNTER15H 0xB8F
#define Z_RISC_V_CSR_MHPMCOUNTER16H 0xB90
#define Z_RISC_V_CSR_MHPMCOUNTER17H 0xB91
#define Z_RISC_V_CSR_MHPMCOUNTER18H 0xB92
#define Z_RISC_V_CSR_MHPMCOUNTER19H 0xB93
#define Z_RISC_V_CSR_MHPMCOUNTER20H 0xB94
#define Z_RISC_V_CSR_MHPMCOUNTER21H 0xB95
#define Z_RISC_V_CSR_MHPMCOUNTER22H 0xB96
#define Z_RISC_V_CSR_MHPMCOUNTER23H 0xB97
#define Z_RISC_V_CSR_MHPMCOUNTER24H 0xB98
#define Z_RISC_V_CSR_MHPMCOUNTER25H 0xB99
#define Z_RISC_V_CSR_MHPMCOUNTER26H 0xB9A
#define Z_RISC_V_CSR_MHPMCOUNTER27H 0xB9B
#define Z_RISC_V_CSR_MHPMCOUNTER28H 0xB9C
#define Z_RISC_V_CSR_MHPMCOUNTER29H 0xB9D
#define Z_RISC_V_CSR_MHPMCOUNTER30H 0xB9E
#define Z_RISC_V_CSR_MHPMCOUNTER31H 0xB9F

/* Machine Counter Setup */
#define Z_RISC_V_CSR_MCOUNTINHIBIT  0x320
#define Z_RISC_V_CSR_MHPMEVENT3	    0x323
#define Z_RISC_V_CSR_MHPMEVENT4	    0x324
#define Z_RISC_V_CSR_MHPMEVENT5	    0x325
#define Z_RISC_V_CSR_MHPMEVENT6	    0x326
#define Z_RISC_V_CSR_MHPMEVENT7	    0x327
#define Z_RISC_V_CSR_MHPMEVENT8	    0x328
#define Z_RISC_V_CSR_MHPMEVENT9	    0x329
#define Z_RISC_V_CSR_MHPMEVENT10    0x32A
#define Z_RISC_V_CSR_MHPMEVENT11    0x32B
#define Z_RISC_V_CSR_MHPMEVENT12    0x32C
#define Z_RISC_V_CSR_MHPMEVENT13    0x32D
#define Z_RISC_V_CSR_MHPMEVENT14    0x32E
#define Z_RISC_V_CSR_MHPMEVENT15    0x32F
#define Z_RISC_V_CSR_MHPMEVENT16    0x320
#define Z_RISC_V_CSR_MHPMEVENT17    0x321
#define Z_RISC_V_CSR_MHPMEVENT18    0x332
#define Z_RISC_V_CSR_MHPMEVENT19    0x333
#define Z_RISC_V_CSR_MHPMEVENT20    0x334
#define Z_RISC_V_CSR_MHPMEVENT21    0x335
#define Z_RISC_V_CSR_MHPMEVENT22    0x336
#define Z_RISC_V_CSR_MHPMEVENT23    0x337
#define Z_RISC_V_CSR_MHPMEVENT24    0x338
#define Z_RISC_V_CSR_MHPMEVENT25    0x339
#define Z_RISC_V_CSR_MHPMEVENT26    0x33A
#define Z_RISC_V_CSR_MHPMEVENT27    0x33B
#define Z_RISC_V_CSR_MHPMEVENT28    0x33C
#define Z_RISC_V_CSR_MHPMEVENT29    0x33D
#define Z_RISC_V_CSR_MHPMEVENT30    0x33E
#define Z_RISC_V_CSR_MHPMEVENT31    0x33F

/* Debug/Trace Registers (shared with Debug Mode) */
#define Z_RISC_V_CSR_TSELECT	    0x7A0
#define Z_RISC_V_CSR_TDATA1	    0x7A1
#define Z_RISC_V_CSR_TDATA2	    0x7A2
#define Z_RISC_V_CSR_TDATA3	    0x7A3
#define Z_RISC_V_CSR_MCONTEXT	    0x7A8

/* Debug Mode Registers */
#define Z_RISC_V_CSR_DCSR	    0x7B0
#define Z_RISC_V_CSR_DPC	    0x7B1
#define Z_RISC_V_CSR_DSCRATCH0	    0x7B2
#define Z_RISC_V_CSR_DSCRATCH1	    0x7B3

#endif /* Z_hardware_ISA_RISC_V_H */
