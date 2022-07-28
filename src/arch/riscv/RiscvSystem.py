# -*- mode:python -*-

# Copyright (c) 2016 RISC-V Foundation
# Copyright (c) 2016 The University of Virginia
# Copyright (c) 2019 EPFL
# All rights reserved.
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
# Authors: Alec Roelke
#          Robert Scheffel
#          Joshua Klein
#          Yasir Qureshi
#          Marina Zapater
#          David Atienza

from m5.params import *
from m5.objects.PciHost import *
from m5.options import *
from m5.SimObject import *
from m5.util.fdthelper import *

from m5.objects.System import System
from m5.objects.Platform import Platform

from SimpleBoard import SimpleBoard


class RiscvSystem(System):
    type = 'RiscvSystem'
    cxx_header = 'arch/riscv/system.hh'
    bare_metal = Param.Bool(False, "Using Bare Metal Application?")
    reset_vect = Param.Addr(0x0, 'Reset vector')
    load_addr_mask = 0xffffffffffffffff

class BareMetalRiscvSystem(RiscvSystem):
    type = 'BareMetalRiscvSystem'
    cxx_header = 'arch/riscv/bare_metal/system.hh'
    bare_metal = True
    bootloader = Param.String("", "File with bootloader.")

class LinuxRiscvSystem(RiscvSystem):
    type = 'LinuxRiscvSystem'
    cxx_header = 'arch/riscv/linux/system.hh'
    bare_metal = False
    board = SimpleBoard() # Under src/dev/riscv
    bootloader = Param.String("", "File with bootloader.")
    dtb_filename = Param.String("", "File with Device Tree Blob.")
