######################################################################## 
# $Id:: makefile 1516 2008-12-17 00:28:46Z pdurgesh                    $
# 
# Project: Debugger loadable example makefile
#
# Notes:
#     This type of image is meant to be loaded and executed through a
#     debugger and will not run standalone and cannot be FLASHed into
#     the board.
#
# Description: 
#  Makefile
# 
######################################################################## 
# Software that is described herein is for illustrative purposes only  
# which provides customers with programming information regarding the  
# products. This software is supplied "AS IS" without any warranties.  
# NXP Semiconductors assumes no responsibility or liability for the 
# use of the software, conveys no license or title under any patent, 
# copyright, or mask work right to the product. NXP Semiconductors 
# reserves the right to make changes in the software without 
# notification. NXP Semiconductors also make no representation or 
# warranty that such application will be suitable for the specified 
# use without further testing or modification. 
########################################################################
PROJ_ROOT			= $(CURDIR)/../../../../../..
EXECNAME    =ea1788_tcpecho_sa
EXDIR		=./
LWIP_ROOT           = $(PROJ_ROOT)/lwip_lpc

########################################################################
#
# Pick up the configuration file in make section
#
########################################################################
include $(PROJ_ROOT)/makesection/makeconfig 
EXDIRINC	=$(PROJ_ROOT)/Examples/$(EXDIR)

########################################################################
#
# Pick up the default build rules 
#
########################################################################

include $(PROJ_ROOT)/makesection/makerule/$(DEVICE)/make.$(DEVICE).$(TOOL)

########################################################################
#
# Pick up the assembler and C source files in the directory
#
########################################################################
include $(PROJ_ROOT)/makesection/makerule/common/make.rules.ftypes

########################################################################
#
# For some Flag defined in header file in Example directory  
#
########################################################################
CFLAGS += -I$(EXDIRINC) $(TMPPATH) 

########################################################################
#
# All other object files + VPATH + additional include paths
#
########################################################################

# Drivers, board and system related code used in this app
VPATH    = $(PROJ_ROOT)/Drivers/source
VPATH   += $(PROJ_ROOT)/Core/Device/$(MANUFACTURE)/$(DEVICE)/Source/Templates
VPATH   += $(PROJ_ROOT)/BoardSupport
ADDOBJS  = lpc177x_8x_clkpwr.o lpc177x_8x_emc.o lpc177x_8x_gpio.o
ADDOBJS += lpc177x_8x_pinsel.o lpc177x_8x_systick.o lpc177x_8x_timer.o
ADDOBJS += lpc177x_8x_uart.o
ADDOBJS += system_$(DEVICE).o  
ADDOBJS += sdram_is42s32800d.o

# Main application
VPATH   += $(LWIP_ROOT)/nxpcommon
VPATH   += $(LWIP_ROOT)/nxpcommon/arch
VPATH   += $(LWIP_ROOT)/nxpcommon/arch/lpc177x_8x
VPATH   += $(LWIP_ROOT)/nxpcommon/arch/lpc177x_8x/gnu
VPATH   += $(LWIP_ROOT)/nxpcommon/board/ea1788
VPATH   += $(LWIP_ROOT)/nxpcommon/examples/ea1788/ea1788_tcpecho_sa/source
IPATH    = -I$(LWIP_ROOT)/nxpcommon
IPATH   += -I$(LWIP_ROOT)/nxpcommon/arch
IPATH   += -I$(LWIP_ROOT)/nxpcommon/arch/lpc177x_8x
IPATH   += -I$(LWIP_ROOT)/nxpcommon/board
IPATH   += -I$(LWIP_ROOT)/nxpcommon/examples/ea1788/ea1788_tcpecho_sa/source
APPOBJS  = lpc17_emac.o lpc17_emac_zcbuffs.o lpc_phy.o
APPOBJS += lpc_debug.o sys_arch.o Serial.o lpc177x_8x_systick_arch.o
APPOBJS += retarget.o ea1788_board.o tcpecho_sa_app.o

# LWIP core
VPATH   += $(LWIP_ROOT)/lwip-1.4.0/src/core
VPATH   += $(LWIP_ROOT)/lwip-1.4.0/src/core/ipv4
VPATH   += $(LWIP_ROOT)/lwip-1.4.0/src/netif
IPATH   += -I$(LWIP_ROOT)/lwip-1.4.0/src/include
IPATH   += -I$(LWIP_ROOT)/lwip-1.4.0/src/include/ipv4
IPATH   += -I$(LWIP_ROOT)/lwip-1.4.0/src/include/netif
APPOBJS += def.o dhcp.o dns.o init.o mem.o memp.o netif.o pbuf.o
APPOBJS += raw.o stats.o sys.o tcp.o tcp_in.o tcp_out.o timers.o
APPOBJS += udp.o autoip.o icmp.o igmp.o inet.o inet_chksum.o ip.o
APPOBJS += ip_addr.o ip_frag.o etharp.o

# LWIP contrib
VPATH   += $(LWIP_ROOT)/contrib-1.4.0/apps/tcpecho_raw
IPATH   += -I$(LWIP_ROOT)/contrib-1.4.0/apps/tcpecho_raw
APPOBJS += echo.o

########################################################################
#
# GNU compiler/linker specific stuff
#
########################################################################
MEXT        =.map
MAPFILE     =$(EXECNAME)
#CFLAGS      +=-gdwarf-2
AFLAGS 		+=-gdwarf-2
ADDOBJSS    += $(PROJ_ROOT)/lwip_lpc/nxpcommon/examples/ea1788/ea1788_common/gnu/startup_$(DEVICE).o

# Define Required Linker script file in each build mode
LDSCRIPTDRAMZC  = linker/ldscript_dram_zc_gnu.ld
LDSCRIPTFLASHCP = linker/ldscript_flash_cp_gnu.ld
LDSCRIPTFLASHZC = linker/ldscript_flash_zc_gnu.ld

#CFLAGS += -DLWIP_DEBUG
LDFLAGS += -Xlinker --gc-sections

########################################################################
#
# Rules to build the executable 
#
########################################################################

default: dram_zc

dram_zc: LDSCRIPT=$(LDSCRIPTDRAMZC)
dram_zc: AFLAGS += --defsym RAM_MODE=0
dram_zc: IPATH += -I $(LWIP_ROOT)/nxpcommon/examples/ea1788/ea1788_tcpecho_sa/source/configs/dram_zc
dram_zc: CFLAGS += $(IPATH)
dram_zc:  $(OBJS) $(ADDOBJS) $(ADDOBJSS) $(APPOBJS)
	$(LD) $(OBJS) $(ADDOBJS) $(ADDOBJSS) $(APPOBJS) $(LDFLAGS) $(LK) $(SCAN) $(MAP) \
	$(MAPFILE)$(MEXT) $(LDESC) $(LDSCRIPT) -o $(EXECNAME)$(EXT)
	$(ELFTOHEX) $(EXECNAME)$(EXT) $(EXECNAME)$(HEX)
	$(ELFTOREC) $(EXECNAME)$(EXT) $(EXECNAME)$(REC)
#	$(ELFTOBIN) $(EXECNAME)$(EXT) $(EFLTBINOPT) $(EXECNAME).bin
	$(MKDIR) gnu\dramzc
	$(MV) $(MAPFILE)$(MEXT) gnu\dramzc\$(MAPFILE)$(MEXT)
	$(MV) $(EXECNAME)$(EXT) gnu\dramzc\$(EXECNAME)$(EXT)
	$(MV) $(EXECNAME)$(HEX) gnu\dramzc\$(EXECNAME)$(HEX)
	$(MV) $(EXECNAME)$(REC) gnu\dramzc\$(EXECNAME)$(REC)
	$(CODESIZE) gnu\dramzc\$(EXECNAME)$(EXT)

flash_cp: LDSCRIPT=$(LDSCRIPTFLASHCP)
flash_cp: AFLAGS += --defsym RAM_MODE=0 
flash_cp: IPATH += -I $(LWIP_ROOT)/nxpcommon/examples/ea1788/ea1788_tcpecho_sa/source/configs/flash_cp
flash_cp: CFLAGS += $(IPATH)
flash_cp:  $(OBJS) $(ADDOBJS) $(ADDOBJSS) $(APPOBJS)
	$(LD) $(OBJS) $(ADDOBJS) $(ADDOBJSS) $(APPOBJS) $(LDFLAGS) $(LK) $(SCAN) $(MAP) \
	$(MAPFILE)$(MEXT) $(LDESC) $(LDSCRIPT) -o $(EXECNAME)$(EXT)
	$(ELFTOHEX) $(EXECNAME)$(EXT) $(EXECNAME)$(HEX)
	$(ELFTOREC) $(EXECNAME)$(EXT) $(EXECNAME)$(REC)
#	$(ELFTOBIN) $(EXECNAME)$(EXT) $(EFLTBINOPT) $(EXECNAME).bin
	$(MKDIR) gnu\Flashcp
	$(MV) $(MAPFILE)$(MEXT) gnu\Flashcp\$(MAPFILE)$(MEXT)
	$(MV) $(EXECNAME)$(EXT) gnu\Flashcp\$(EXECNAME)$(EXT)
	$(MV) $(EXECNAME)$(HEX) gnu\Flashcp\$(EXECNAME)$(HEX)
	$(MV) $(EXECNAME)$(REC) gnu\Flashcp\$(EXECNAME)$(REC)
	$(CODESIZE) gnu\Flashcp\$(EXECNAME)$(EXT)

flash_zc: LDSCRIPT=$(LDSCRIPTFLASHZC)
flash_zc: AFLAGS += --defsym RAM_MODE=0 
flash_zc: IPATH += -I $(LWIP_ROOT)/nxpcommon/examples/ea1788/ea1788_tcpecho_sa/source/configs/flash_zc
flash_zc: CFLAGS += $(IPATH)
flash_zc:  $(OBJS) $(ADDOBJS) $(ADDOBJSS) $(APPOBJS)
	$(LD) $(OBJS) $(ADDOBJS) $(ADDOBJSS) $(APPOBJS) $(LDFLAGS) $(LK) $(SCAN) $(MAP) \
	$(MAPFILE)$(MEXT) $(LDESC) $(LDSCRIPT) -o $(EXECNAME)$(EXT)
	$(ELFTOHEX) $(EXECNAME)$(EXT) $(EXECNAME)$(HEX)
	$(ELFTOREC) $(EXECNAME)$(EXT) $(EXECNAME)$(REC)
#	$(ELFTOBIN) $(EXECNAME)$(EXT) $(EFLTBINOPT) $(EXECNAME).bin
	$(MKDIR) gnu\Flashzc
	$(MV) $(MAPFILE)$(MEXT) gnu\Flashzc\$(MAPFILE)$(MEXT)
	$(MV) $(EXECNAME)$(EXT) gnu\Flashzc\$(EXECNAME)$(EXT)
	$(MV) $(EXECNAME)$(HEX) gnu\Flashzc\$(EXECNAME)$(HEX)
	$(MV) $(EXECNAME)$(REC) gnu\Flashzc\$(EXECNAME)$(REC)
	$(CODESIZE) gnu\Flashzc\$(EXECNAME)$(EXT)

clean_objs: realclean lpc_clean
	@$(RM) $(APPOBJS)
	@$(RM) $(ADDOBJS)
	@$(RM) $(ADDOBJSS)
	
cleanall: clean_objs
	@$(RMDIR) "gnu"	

########################################################################
#
# Pick up the compiler and assembler rules
#
########################################################################

include $(PROJ_ROOT)/makesection/makerule/common/make.rules.build

.PHONY: debug_status 
