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

This folder contains platform-specific bootloader codes for use with gem5's FS (full system) mode implementation running RISC-V and linux.  The preferred bootloader for this is OpenSBI, found at the following link: https://github.com/riscv/opensbi

Compilation instructions for the OpenSBI bootloader are found on their github page.  The specific files included here are the configuration files used for running gem5 -- they are very lightly modified from the files used for QEMU's virt platform, and should be updated when RISC-V gains more support in gem5.  You can simple copy and paste the files here in place of those included in OpenSBI for the qemu/virt platform, and then you can compile it with the following:

> export CROSS_COMPILE=/path/to/riscv/compiler/here
> export PLATFORM_RISCV_XLEN=64
> make PLATFORM=qemu/virt O=/path/to/your/build/directory
> make PLATFORM=qemu/virt I=/path/to/your/install/directory

The result of these commands, if run successfully, should include a file "fw_jump.elf".  This can be used as a standalone bootloader which launches a vmlinux file during runtime.
