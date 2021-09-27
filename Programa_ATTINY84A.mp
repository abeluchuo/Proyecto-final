Project:      ..\ATTINY84A_Prueba\Programa_ATTINY84A
Build Number: 361
Date:         Wed May 20 21:22:00 2020
Compiler:     ICCAVR
Version:      8.30.00
Device:       ATtiny84

NOTE: AVR Code addresses are word addresses
All other addresses (including ones in FLASH) are byte addresses
  ROM 95% full. 7770 out of 8192 bytes used (does not include absolute areas).
  RAM 333 bytes used (does not include stack usage).

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             lit   006A     10 =     16. bytes (rel,con,rom,lit)

       Addr  Global Symbol
      -----  --------------------------------
       006A  __lit_start
       007A  __lit_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                           idata   007A     6A =    106. bytes (rel,con,rom,lit)

       Addr  Global Symbol
      -----  --------------------------------
       007A  __idata_start
       00E4  __idata_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            text   00E4   1DE0 =   7648. bytes (rel,con,rom,code)

       Addr  Global Symbol
      -----  --------------------------------
       0072  __text_start
       0072  __start
       0093  _exit
       0094  _main
       01CE  _MideF
       0226  _CorrigeSensorColor
       058B  _CargaParametros
       08A6  _SPI_SlaveTransfer
       08B7  _EEPROM_read
       08BE  _EEPROM_write
       08C8  _Config
       08E4  _delayus
       08F5  _delay_ms
       092A  _GuardaParametros
       0BD0  _int0_isr
       0BDF  _PinCh0_int_isr
       0BE0  _PinCh1_int_isr
       0BE1  _wdt_ovf_isr
       0BE2  _timer1_capt_isr
       0BE3  _timer0_compa_isr
       0BE4  _timer1_compb_isr
       0BE5  _timer1_ovf_isr
       0BE6  _timer1_compa_isr
       0C04  _timer0_compb_isr
       0C05  _timer0_ovf_isr
       0C21  _ana_comp_isr
       0C22  _adc_isr
       0C23  _eeprom_ready_isr
       0C24  _usi_start_isr
       0C25  _usi_ovf_isr
       0C26  <LIBRARY>
       0C26  _memcpy
       0C34  push_arg4
       0C36  push_arg2
       0C39  mod16s
       0C3D  div16s
       0C56  mod16u
       0C58  div16u
       0C59  xdiv16u
       0C72  pop_xgsetF0FC
       0C7D  push_xgsetF0FC
       0C7F  push_xgset30FC
       0C81  push_xgset00FC
       0C83  push_xgset003C
       0C88  push_xgset300C
       0C8D  pop_xgset300C
       0C92  push_xgsetF000
       0C97  pop_xgsetF000
       0C9C  neg32
       0CA5  long_prolog
       0CB3  long_epilog
       0CBE  tstzero1
       0CC4  tstzero2
       0CCA  lpm32
       0CDE  mpy16s
       0CF2  mpy32u
       0CF2  mpy32s
       0D0F  fpsub
       0D11  fpadd
       0D12  fpadd_alt
       0D70  fpadd1
       0D79  fpadd2
       0D83  fpint
       0DB3  uint2fp
       0DB5  int2fp
       0DBE  ulong2fp
       0DC0  long2fp
       0DD5  fpdiv1
       0DDE  fpdiv2
       0DE8  fpdiv1x
       0DF4  fpdiv2x
       0E69  fpmul
       0EAE  fpmul1
       0EB7  unpacks
       0EBE  tstR16_R19
       0EC6  unpacku
       0EDF  normalize_and_pack
       0F14  pack
       0F24  saveFPRegs
       0F33  restoreFPRegs
       0F42  fpcmp
       0F4D  fpcmp1
       0F57  fpcmp2
       0F62  __text_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                            data   0060     6A =    106. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       0060  __data_start
       0060  _Esclavo_ID
       0061  _ss
       0063  _inicio
       0065  _ovf
       0067  _Texp
       0068  _Haciendo
       0069  _BalanceNegros
       006A  _BalanceBlancos
       006B  _CorregirGama
       006C  _CorregirColor
       006D  _CorregirHSV
       006E  _ObtenerRGBdeHSV
       006F  _umbralGris
       0070  _rslm
       0074  _rslb
       0078  _gslm
       007C  _gslb
       0080  _bslm
       0084  _bslb
       0088  _mcrm
       008C  _mcrb
       0090  _mcgm
       0094  _mcgb
       0098  _mcbm
       009C  _mcbb
       00A0  _mchm
       00A4  _mchb
       00A8  _mcsm
       00AC  _mcsb
       00B0  _mcvm
       00B4  _mcvb
       00B8  _MHSV11
       00BA  _MHSV12
       00BC  _MHSV13
       00BE  _MHSV21
       00C0  _MHSV22
       00C2  _MHSV23
       00C4  _MHSV31
       00C6  _MHSV32
       00C8  _MHSV33
       00CA  __data_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                             bss   00CA     E3 =    227. bytes (rel,con,ram)

       Addr  Global Symbol
      -----  --------------------------------
       00CA  _H
       00CA  __bss_start
       00CC  _V
       00CD  _S
       00CE  _B
       00CF  _G
       00D0  _R
       00D1  _Hh
       00D2  _Hl
       00D3  _cg
       00D4  _cr
       00D5  _sizefloat
       00D7  _storageint
       00E9  _storage
       0131  _buffint
       0143  _buff
       018B  _cbn
       0191  _Va
       0195  _Sa
       0199  _Ha
       019D  _caracter
       019E  _banConteo
       019F  _rb
       01A3  _rg
       01A7  _rr
       01AB  _conteo
       01AC  _DatoSPI
       01AD  __bss_end

Area                               Addr   Size   Decimal Bytes (Attributes)
--------------------------------   ----   ----   ------- ----- ------------
                          vector   0002     22 =     34. bytes (abs,ovr,rom,code)

Files Linked      [ module(s) ]

C:\iccv8avr\lib\crtAVR.o	[ crtavr.s ]
..\ATTINY84A_Prueba\.objs\main.o	[ main.c ]
<library>	[ memcpy.s, asave.s, div16s.s, div16u.s, gpop5.s, gpush.s, gpushx.s, gpushy.s, lneg.s, longutil.s, lpm32.s ]
[ mpy16s.s, mul32.s, fpadd.s, fpadd1.s, fpadd2.s, fpint.s, int2fp.s, fpdiv_new.s, fpmulnew.s, fpmul1.s ]
[ fputil.s, fpcmp.s, fpcmp1.s, fpcmp2.s ]

User Global Definitions

ram_end = 0x25f
hwstk_size = 0x14

Area Definitions

-bfunc_lit:0x6a.0x2000
-bdata:0x60.0x25f

Additional Flags
  -e:0x2000
