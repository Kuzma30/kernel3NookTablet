#
# Copyright (C) Imagination Technologies Ltd. All rights reserved.
# 
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
# 
# This program is distributed in the hope it will be useful but, except 
# as otherwise stated in writing, without any warranty; without even the 
# implied warranty of merchantability or fitness for a particular purpose. 
# See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
# 
# The full GNU General Public License is included in this distribution in
# the file called "COPYING".
#
# Contact Information:
# Imagination Technologies Ltd. <gpl-support@imgtec.com>
# Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
# 
#

OPTIM := -Os

SYS_CFLAGS := \
 -march=armv7-a -fno-short-enums -D__linux__ \
 -I$(ANDROID_ROOT)/bionic/libc/arch-arm/include \
 -I$(ANDROID_ROOT)/bionic/libc/include \
 -I$(ANDROID_ROOT)/bionic/libc/kernel/common \
 -I$(ANDROID_ROOT)/bionic/libc/kernel/arch-arm \
 -I$(ANDROID_ROOT)/bionic/libm/include \
 -I$(ANDROID_ROOT)/bionic/libm/include/arm \
 -I$(ANDROID_ROOT)/bionic/libthread_db/include \
 -I$(ANDROID_ROOT)/frameworks/base/include \
 -isystem $(ANDROID_ROOT)/system/core/include \
 -I$(ANDROID_ROOT)/hardware/libhardware/include \
 -I$(ANDROID_ROOT)/external/openssl/include

SYS_EXE_CRTBEGIN := $(TOOLCHAIN)/lib/crtbegin_dynamic.o
SYS_EXE_CRTEND := $(TOOLCHAIN)/lib/crtend_android.o

SYS_EXE_LDFLAGS := \
 -Bdynamic \
 -nostdlib -Wl,-dynamic-linker,/system/bin/linker \
 -lc -ldl -lcutils
SYS_LIB_LDFLAGS := $(SYS_EXE_LDFLAGS)

# This is required with the change to the GCC4.6 toolchain
# in JB (otherwise commands just segfault)
ifeq ($(is_future_version),0)
SYS_EXE_LDFLAGS += -fPIE -pie
endif

# Handle the removal of the armelf.x and armelf.xsc linker scripts.
ifeq ($(strip $(wildcard $(ANDROID_ROOT)/build/core/armelf.x)),)
# The linker scripts have been removed. We need to use these options
# instead.
SYS_EXE_LDFLAGS += -Wl,-z,relro -Wl,-z,now
SYS_LIB_LDFLAGS += -Wl,-z,relro -Wl,-z,now
else
# The linker scripts are still present in the Android tree, so we need to
# use them.
SYS_EXE_LDFLAGS += -Wl,-T$(ANDROID_ROOT)/build/core/armelf.x
SYS_LIB_LDFLAGS += -Wl,-T$(ANDROID_ROOT)/build/core/armelf.xsc
endif

JNI_CPU_ABI := armeabi

# Android builds are usually GPL
#
LDM_PLATFORM ?= 1
