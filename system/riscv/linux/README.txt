# Copyright (c) 2019 EPFL
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Joshua Klein
#          Yasir Qureshi
#          Marina Zapater
#          David Atienza

This folder contains the config file(s) used to build a gem5-RV64-compatible linux kernel.  The main linux kernel tested is built straight from the linux repository (https://github.com/torvalds/linux) and is version 5.4.0-rc5.  Also included is a kernel config file for linux kernel 4.19.82 for legacy support.

In order to build the linux kernel for use with RV64 systems you must have the linux RISC-V crosscompiler/toolchain installed.  After that, you can use the following commands to build vmlinux with the config file:

> cp <config file> /path/to/your/linux/repo/.config
> make ARCH=riscv CROSS_COMPILE=/path/to/your/riscv64-unknown-linux-gnu- vmlinux

The drivers that were altered can be seen by using a diff on the config file vs. the config file generated with linux defconfig.  Of particular note, drivers adding PIIX support were added, while mouse drivers, PS2 drivers, and some drivers relating to virtio were removed.

