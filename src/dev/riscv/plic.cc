/*
 * Copyright (c) 2019 EPFL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#include "dev/riscv/plic.hh"

#include "arch/riscv/faults.hh"
#include "cpu/base.hh"
#include "cpu/intr_control.hh"
#include "cpu/thread_context.hh"
#include "debug/PLIC.hh"

Plic::Plic(const Params *p)
    : BasicPioDevice(p, 0x4000000), platform(p->platform),
      system(dynamic_cast<RiscvSystem *>(p->system))
{
    SimpleBoard * const sb(dynamic_cast<SimpleBoard *>(p->platform));
    sb->setPlic(this);

    for (auto cpu : p->cpus) {
        cpuTargets.push_back(cpu);
    }
}

Plic::~Plic()
{
}

void
Plic::init()
{
    BasicPioDevice::init();
    system->setPlic(this);
}

Tick
Plic::read(PacketPtr pkt)
{
    Addr addr = pkt->getAddr();
    assert(PLICRegFile.count(addr) > 0);

    if ((addr & 0xffff000f) == (HartThresholdClaimBase | 0x4)) {
        int claim = plicClaim();
        DPRINTF(PLIC, "PLIC claiming interrupt %#x.\n", claim);
        pkt->setLE<uint32_t>(claim);
    } else {
        DPRINTF(PLIC, "PLIC read @ %#x: %#x.\n", addr, PLICRegFile[addr]);
        pkt->setLE<uint32_t>(PLICRegFile[addr]);
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
Plic::write(PacketPtr pkt)
{
    Addr addr = pkt->getAddr();
    assert(PLICRegFile.count(addr) > 0);

    uint64_t data = (pkt->getSize() == 8) ?  pkt->getLE<uint64_t>() :
        (uint64_t)pkt->getLE<uint32_t>();

    if ((addr & 0xffff000f) == (HartThresholdClaimBase | 0x4)) {
        DPRINTF(PLIC, "PLIC completing interrupt %#x.\n", data);
    } else {
        DPRINTF(PLIC, "PLIC write @ %#x: %#x --> %#x.\n", addr, data,
            PLICRegFile[addr]);
    }

    // Write to source priority register
    if ((addr & 0xffffff00) == SourcePriorityBase) {
        PLICRegFile[addr] = data & 0x7;
    }
    // Pending registers are RO?
    else if ((addr == 0x0C001000 || addr == 0x0C001004) && data) {
        panic("PLIC writing %#x to a pending register!", data);
    }
    // Write to IE register
    else if ((addr & 0xfffff000) == HartIntEnableBase) {
        PLICRegFile[addr] = data;
    }
    // Write to threshold register
    else if ((addr & 0xffff000f) == HartThresholdClaimBase) {
        PLICRegFile[addr] = data & 0x7;
    }
    // Write a claim register to signal completion
    else if ((addr & 0xffff000f) == (HartThresholdClaimBase | 0x4)) {
        PLICRegFile[addr] = data;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Plic::sendInt(int line)
{
    DPRINTF(PLIC, "Plic::sendInt on line %#x.\n", line);

    // Check interrupt source meets threshold
    // TODO: Select threshold of appropriate HART
    int priority = PLICRegFile[0x0C000004 + (line * 4)];
    int threshold = PLICRegFile[0x0C200000];
    if (priority < threshold) return;

    setPendingArray(line);

    // This block of code is more representative of what QEMU does in its PLIC
    // implementation: check the HART-specific IE registers in the PLIC before
    // posting an interrupt.
    //
    // TODO: Multi-hart support.
    int hartId = 0;

    // HART 0 M-Mode
    if (getPendingArray() & ((uint64_t)PLICRegFile[0x0C002004] << 32 |
         PLICRegFile[0x0C002000])) {
        cpuTargets[hartId]->postInterrupt(hartId,
            RiscvISA::INT_EXT_MACHINE, 0);
    }
    // HART 0 S-mode
    else if (getPendingArray() & ((uint64_t)PLICRegFile[0x0C002084] << 32 |
         PLICRegFile[0x0C002080])) {
        cpuTargets[hartId]->postInterrupt(hartId,
            RiscvISA::INT_EXT_SUPER, 0);
    }
    return;
}

void
Plic::clearInt(int line)
{
    DPRINTF(PLIC, "Plic::clearInt on line %#x.\n", line);

    unsetPendingArray(line);

    // TODO: Select HART for multicore support
    if (!getPendingArray()) {
        int hartId = 0;
        cpuTargets[hartId]->clearInterrupt(hartId,
            RiscvISA::INT_EXT_MACHINE, 0);
        cpuTargets[hartId]->clearInterrupt(hartId,
            RiscvISA::INT_EXT_SUPER, 0);
    }
}

uint64_t
Plic::getPendingArray()
{
    return ((uint64_t)PLICRegFile[0x0C001004] << 32) | PLICRegFile[0x0C001000];
}

void
Plic::setPendingArray(int sourceNo)
{
    if (sourceNo == 0) {
        return;
    } else if (sourceNo >= 32) {
        PLICRegFile[0x0C001004] |= (0x1 << (sourceNo - 32));
    } else {
        PLICRegFile[0x0C001000] |= (0x1 << sourceNo);
    }
}

void
Plic::unsetPendingArray(int sourceNo)
{
    if (sourceNo == 0) {
        return;
    } else if (sourceNo >= 32) {
        PLICRegFile[0x0C001004] &= ~(0x1 << (sourceNo - 32));
    } else {
        PLICRegFile[0x0C001000] &= ~(0x1 << sourceNo);
    }
}

uint32_t
Plic::plicClaim()
{
    // HART claims interrupt by selecting the pending interrupt with the
    // highest priority (lowest high bit index of the pending array)
    uint64_t pendingArray = getPendingArray();
    if (pendingArray) {
        for (int i = 1; i < 64; i++) {
            if ((pendingArray >> i) & 0x1) {
                clearInt(i);
                return i;
            }
        }
    }

    return 0;
}

Plic *
PlicParams::create()
{
    return new Plic(this);
}
