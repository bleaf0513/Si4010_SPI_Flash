;-------------------------------------------------------------------------------
;                           Silicon Laboratories, Inc.
;                            http://www.silabs.com
;                                Copyright 2010
;-------------------------------------------------------------------------------
;
;     FILE:       si4010_rom_keil.a51
;     TARGET:     Si4010
;     TOOLCHAIN:  Keil
;     DATE:       June 14, 2010, Monday
;     RELEASE:    2.3 (MPech), ROM version 02.00, trim 3
;
;---------------------------------------------------------------------------
;
;     DESCRIPTION:
;       This file is must be compiled and linked into builds which intend
;       to user ROM functions for the ROM release specified in the header
;       above. This file can only be used with Keil toolchain.
;
;---------------------------------------------------------------------------

RomSymbol MACRO SymName SymAddress
          PUBLIC SymName
                 SymName CODE SymAddress
ENDM

RomSymbol  _VAES_CIPHER,                   08003H
RomSymbol  _VAES_INVCIPHER,                08006H
RomSymbol  _VAES_INVGENKEY,                08009H
RomSymbol   WBSR_GETCURRENTBUTTON,         0801BH
RomSymbol   BBSR_GETPTSITEMCNT,            08015H
RomSymbol   BBSR_GETTIMESTAMP,             0801EH
RomSymbol  _VBSR_INITPTS,                  08012H
RomSymbol   WBSR_POP,                      0800CH
RomSymbol   VBSR_SERVICE,                  08018H
RomSymbol  _VBSR_SETUP,                    0800FH
RomSymbol   VDMDTS_CLEARDMD,               0802DH
RomSymbol   VDMDTS_CLEARDMDINTFLAG,        08030H
RomSymbol  _VDMDTS_ENABLE,                 08039H
RomSymbol   IDMDTS_GETDATA,                011EFH
RomSymbol   IDMDTS_GETLATESTDMDSAMPLE,     08027H
RomSymbol   IDMDTS_GETLATESTTEMP,          0802AH
RomSymbol   BDMDTS_GETSAMPLESTAKEN,        08036H
RomSymbol   VDMDTS_ISRCALL,                011ECH
RomSymbol  _VDMDTS_RESETCOUNTS,            0803FH
RomSymbol  _VDMDTS_RUNFORTEMP,             0803CH
RomSymbol  _VDMDTS_SETUP,                  08021H
RomSymbol  _BENC_4B5BENCODE,               08045H
RomSymbol  _BENC_MANCHESTERENCODE,         08048H
RomSymbol  _VENC_SET4B5BLASTBIT,           08042H
RomSymbol  _VFCAST_FSKADJ,                 080FCH
RomSymbol   VFCAST_SETUP,                  080F9H
RomSymbol  _VFCAST_FINETUNE,               0809CH
RomSymbol  _VFCAST_TUNE,                   080F6H
RomSymbol  _VFCAST_XOSETUP,                080FFH
RomSymbol   LFC_GETCOUNT,                  08057H
RomSymbol   VFC_POLLDONE,                  08051H
RomSymbol  _VFC_SETUP,                     0804BH
RomSymbol   VFC_STARTCOUNT,                0804EH
RomSymbol   LFC_STARTPOLLGETCOUNT,         08054H
RomSymbol  _BHVRAM_READ,                   0805AH
RomSymbol  _VHVRAM_WRITE,                  0805DH
RomSymbol  _LMTP_GETDECCOUNT,              08060H
RomSymbol  _VMTP_INCCOUNT,                 08063H
RomSymbol  PBMTP_READ,                     0806CH
RomSymbol  _VMTP_SETDECCOUNT,              08072H
RomSymbol   VMTP_STROBE,                   0806FH
RomSymbol  _BMTP_WRITE,                    08069H
RomSymbol  _IMVDD_MEASURE,                 080F0H
RomSymbol   BNVM_COPYBLOCK,                0807BH
RomSymbol   WNVM_GETADDR,                  08078H
RomSymbol   VNVM_MCDISABLEREAD,            08081H
RomSymbol   VNVM_MCENABLEREAD,             0807EH
RomSymbol  _VNVM_SETADDR,                  08075H
RomSymbol  _VODS_ENABLE,                   08087H
RomSymbol  _VODS_SETUP,                    08084H
RomSymbol  _VODS_WRITEDATA,                0808AH
RomSymbol  _VPA_SETUP,                     011E9H
RomSymbol  _VPA_TUNE,                      08090H
RomSymbol  _VSLEEPTIM_ADDTXTIMETOCOUNTER,  080EDH
RomSymbol   BSLEEPTIM_CHECKDUTYCYCLE,      080EAH
RomSymbol   LSLEEPTIM_GETONEHOURVALUE,     011E0H
RomSymbol   LSLEEPTIM_GETCOUNT,            080E4H
RomSymbol  _VSLEEPTIM_SETCOUNT,            080E7H
RomSymbol  _BSTL_ENCODEBYTE,               080DEH
RomSymbol  _VSTL_ENCODESETUP,              080E1H
RomSymbol   VSTL_POSTLOOP,                 011E3H
RomSymbol   VSTL_PRELOOP,                  080D5H
RomSymbol  _VSTL_SINGLETXLOOP,             080DBH
RomSymbol  _VSYS_16BITDECLOOP,             080C3H
RomSymbol  _VSYS_8BITDECLOOP,              080C6H
RomSymbol  _VSYS_BANDGAPLDO,               080A2H
RomSymbol   VSYS_FIRSTPOWERUP,             080C9H
RomSymbol   VSYS_FORCELC,                  08102H
RomSymbol   BSYS_GETBOOTSTATUS,            080CCH
RomSymbol   WSYS_GETCHIPID,                080A8H
RomSymbol   WSYS_GETKEILVER,               080CFH
RomSymbol   LSYS_GETMASTERTIME,            080B4H
RomSymbol   LSYS_GETPRODID,                011E6H
RomSymbol   BSYS_GETREVID,                 080ABH
RomSymbol   WSYS_GETROMID,                 080A5H
RomSymbol  _VSYS_INCMASTERTIME,            080B7H
RomSymbol  _VSYS_LEDINTENSITY,             080BDH
RomSymbol   VSYS_LPOSCADJ,                 080D2H
RomSymbol  _VSYS_SETCLKSYS,                080AEH
RomSymbol  _VSYS_SETMASTERTIME,            080BAH
RomSymbol  _VSYS_SETUP,                    080B1H
RomSymbol   VSYS_SHUTDOWN,                 080C0H

;
;---------------------------------------------------------------------------
;

RomSymbol ?C?ULSHR,                        08400H
RomSymbol ?C?LSHL,                         08413H
RomSymbol ?C?OFFXADD,                      08426H
RomSymbol ?C?FPSUB,                        08435H
RomSymbol ?C?FPADD,                        08439H
RomSymbol ?C?FPMUL,                        0852AH
RomSymbol ?C?FCASTL,                       08630H
RomSymbol ?C?FCASTC,                       0863AH
RomSymbol ?C?FCASTI,                       08635H
RomSymbol ?C?CASTF,                        0866EH
RomSymbol ?C?FPGETOPN2,                    086A5H
RomSymbol ?C?FPNANRESULT,                  086DAH
RomSymbol ?C?FPOVERFLOW,                   086E4H
RomSymbol ?C?FPRESULT,                     086BCH
RomSymbol ?C?FPRESULT2,                    086D0H
RomSymbol ?C?FPUNDERFLOW,                  086E1H
RomSymbol ?C?LNEG,                         086EFH
RomSymbol ?C?LLDIDATA,                     086FDH
RomSymbol ?C?LLDXDATA,                     08709H
RomSymbol ?C?LLDIDATA0,                    08715H
RomSymbol ?C?LLDXDATA0,                    08722H
RomSymbol ?C?LSTIDATA,                     0872EH
RomSymbol ?C?LSTXDATA,                     0873AH
RomSymbol ?C?LMUL,                         08746H
RomSymbol ?C?ULCMP,                        08795H
RomSymbol ?C?LOR,                          087A6H
RomSymbol ?C?LADD,                         087B3H
RomSymbol ?C?IMUL,                         087C0H
RomSymbol ?C?LSUB,                         087D2H
RomSymbol ?C?LXOR,                         087E0H
RomSymbol ?C?FPDIV,                        087F0H
RomSymbol ?C?ULDIV,                        088C9H
RomSymbol ?C?SIDIV,                        0895BH
RomSymbol ?C?FPCMP3,                       089E6H
RomSymbol ?C?FPCMP,                        089E8H
RomSymbol ?C?UIDIV,                        08991H
RomSymbol ?C?SLDIV,                        08A6FH
RomSymbol _ABS,                            08AB3H
RomSymbol ?C?ICALL,                        08A53H
RomSymbol ?C?ICALL2,                       08A57H
RomSymbol ?C?IILDX,                        08A59H

END

;
;-------------------------------------------------------------------------------
;
