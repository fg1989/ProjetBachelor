# Copyright (c) 2018 TU Dresden
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
# Authors: Robert Scheffel
#          Joshua Klein
#          Yasir Qureshi
#          Marina Zapater
#          David Atienza

from m5.objects.Device import BasicPioDevice
from m5.objects.PciHost import *
from m5.objects.Platform import Platform
from m5.objects.Terminal import Terminal
from m5.objects.Uart import Uart, SimpleUart, Uart8250
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class GenericRiscvPciHost(GenericPciHost):
    type = 'GenericRiscvPciHost'
    cxx_header = "dev/riscv/pci_host.hh"
    int_base = Param.Int(0x20,
        "Base number used as interrupt line and PLIC source.")


class Clint(BasicPioDevice):
    type = "Clint"
    cxx_header = "dev/riscv/clint.hh"
    system = Param.System(Parent.any, "System this device is part of.")
    cpus = VectorParam.BaseCPU("CPUs/harts attached to this device.")
    frequency = Param.Int(1000000, "CLINT timer frequency (default 1MHz).")


class Plic(BasicPioDevice):
    type = "Plic"
    cxx_header = "dev/riscv/plic.hh"
    system = Param.System(Parent.any, "System this device is part of.")
    cpus = VectorParam.BaseCPU("CPUs/harts attached to this device.")
    platform = Param.Platform(Parent.any, "Plaform PLIC is attached to.")


class SimpleBoard(Platform):
    type = "SimpleBoard"
    cxx_header = "dev/riscv/simpleboard.hh"
    system = Param.System(Parent.any, "system")
    clint = Clint(pio_addr=0x2000000)
    console_int = Param.Int(0xa,
        "Which interrupt source number is posted to the PLIC for console?")

    # Note: Address range set in FSConfig.py, under self.bridge.ranges
    uart = Uart8250(pio_addr=0x10000000)
    plic = Plic(pio_addr=0x0c000000)
    pci_host = GenericRiscvPciHost(conf_base=0x30000000, conf_size='256MB',
        conf_device_bits=12, pci_pio_base=0x2f000000, pci_mem_base=0x40000000)

    # Call methods after buses defined at system level
    # For attaching things to the membus
    def attachIntIO(self, bus):
        self.clint.pio = bus.master
        self.plic.pio = bus.master

    # For attaching things to the iobus
    def attachSimpleBoardIO(self, bus):
        self.uart.pio = bus.master
