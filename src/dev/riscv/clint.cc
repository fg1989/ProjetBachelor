/*
 * Copyright (c) 2018 TU Dresden
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
 * Authors: Robert Scheffel
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#include "dev/riscv/clint.hh"

#include "arch/riscv/faults.hh"
#include "cpu/base.hh"
#include "debug/Timer.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

CpuTimer::CpuTimer(BaseCPU * cpu, Clint * parent, uint64_t hartId)
  : timecmp(0xffffffffffffffff),
    timerAlarmEvent([this]{ timerAlarm(); }, name())

{
    this->hartId = hartId;
    this->cpu = cpu;
    this->parent = parent;
}

CpuTimer::~CpuTimer()
{
}

void
CpuTimer::startTimer(uint64_t val)
{
    Tick offset = parent->clockPeriod() * val;

    if (timerAlarmEvent.scheduled()) {
        DPRINTF(Timer, "-- AlarmEvent already scheduled, de-scheduling.\n");
        parent->deschedule(timerAlarmEvent);
    }

    parent->schedule(timerAlarmEvent, curTick() + offset);
    DPRINTF(Timer, "-- Scheduling new event for: %d\n", curTick() + offset);
}

void
CpuTimer::timerAlarm()
{
    parent->updateTime();

    // Note: IE/IP bits automagically handled by postInterrupt method.
    if (parent->time >= timecmp) {
        cpu->postInterrupt(hartId, RiscvISA::INT_TIMER_MACHINE, 0);
    }

    DPRINTF(Timer, "Clint::timerAlarm @ tick %#x: time = %#x, timecmp = %#x."
        "\n", curTick(), parent->time, timecmp);
}

void
CpuTimer::serialize(CheckpointOut & cp) const
{
    SERIALIZE_SCALAR(timecmp);
    SERIALIZE_SCALAR(hartId);
    // How to serialize timerAlarmEvent and cpu?
}

void
CpuTimer::unserialize(CheckpointIn & cp)
{
    UNSERIALIZE_SCALAR(timecmp);
    UNSERIALIZE_SCALAR(hartId);
    // How to unserialize timerAlarmEvent and cpu?
}

Clint::Clint(Params *p)
    : BasicPioDevice(p, 0x10000),
      timeDelta(0),
      period((1.0 / p->frequency) * SimClock::Frequency),
      system(dynamic_cast<RiscvSystem *>(p->system))
{
    int i = 0;
    for (auto cpu : p->cpus) {
        cpuTimers.push_back(new CpuTimer(cpu, this, i++));
    }
}

Clint::~Clint()
{
    for (auto cpuTimer : cpuTimers) {
        delete cpuTimer;
    }
}

void
Clint::init()
{
    BasicPioDevice::init();
    system->setClint(this);
}

Tick
Clint::read(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr addr = pkt->getAddr() - pioAddr;
    Addr idx = (addr & 0x0fff);
    uint32_t val;

    switch (pkt->getSize()) {
        case 4:
            idx /= 4;
            if (addr & 0xf000) {
                // The actual value of MSIP is preserved by the MSIP bit of the
                // MIP CSR, handled in arch/riscv/isa.cc.
                val = (cpuTimers[idx]->cpu->getInterruptController(idx)
                    ->readIP() & 0x8) ? 0x1 : 0x0;
                DPRINTF(Timer, "CLINT read @ %#x: hart[%#x]->msip = %#x.",
                    addr, idx, val);
                pkt->setLE<uint32_t>(val);
            } else {
                panic("Invalid 4B read CLINT @ %#x.", addr);
            }
        break;
        case 8:
            idx /= 8;
            switch (addr & 0xf000) {
                case TimeCmpBase:
                    DPRINTF(Timer, "CLINT read @ %#x: hart[%#x]->timecmp "
                        "= %#x.", addr, idx, cpuTimers[idx]->timecmp);
                    pkt->setLE<uint64_t>(cpuTimers[idx]->timecmp);
                break;
                case Time & 0xf000:
                    if (addr == Time) {
                        updateTime();
                        pkt->setLE<uint64_t>(time);
                    } else {
                        panic("Invalid 8B read CLINT @ %#x.", addr);
                    }
                break;
                default:
                    panic("Invalid 8B read CLINT @ %#x.", addr);
            }
        break;
        default:
            panic("Unsupported packet size for read access on Clint");
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
Clint::write(PacketPtr pkt)
{
    assert(pkt->getAddr() >= pioAddr && pkt->getAddr() < pioAddr + pioSize);

    Addr addr = pkt->getAddr() - pioAddr;
    Addr idx = (addr & 0x0fff);
    Tick nextAlarm = 0;

    switch (pkt->getSize()) {
        case 4:
            idx /= 4;
            switch (addr & 0xf000) {
                // According to the SiFive FU540-C000 Manual v1, section 9.2,
                // entitled "MSIP Registers", mode software interrupts should
                // be generated by a write to the CLINT's MSIP register. The
                // actual value of MSIP is preserved by the MSIP bit of the MIP
                // CSR, handled in arch/riscv/isa.cc.
                case InterruptPendingBase:
                    DPRINTF(Timer, "CLINT write @ %#x: hart[%#x]->msip", addr,
                        idx);
                    if (pkt->getLE<uint32_t>())
                        cpuTimers[idx]->cpu->postInterrupt(idx,
                                    RiscvISA::INT_SOFTWARE_MACHINE, 0);
                break;
                default:
                    panic("Invalid 4B write CLINT @ %#x.", addr);
                break;
            }
        break;
        case 8:
            idx /= 8;
            switch (addr & 0xf000) {
                case TimeCmpBase:
                    DPRINTF(Timer, "CLINT write @ %#x: hart[%#x]->timecmp"
                        " = %#x --> %#x.", addr, idx, cpuTimers[idx]->timecmp,
                        pkt->getLE<uint64_t>());
                    cpuTimers[idx]->timecmp = pkt->getLE<uint64_t>();
                    updateTime();

                    // TODO: Properly schedule multiple harts
                    // Per the privileged specification 1.11 section 3.1.9:
                    // "The MTIP bit is read-only and is cleared by writing to
                    //  the memory-mapped machine-mode timer compare register."
                    cpuTimers[idx]->cpu->clearInterrupt(idx,
                                    RiscvISA::INT_TIMER_MACHINE, 0);

                    if (cpuTimers[idx]->timecmp <= time) {
                        cpuTimers[idx]->cpu->postInterrupt(idx,
                            RiscvISA::INT_TIMER_MACHINE, 0);
                    } else {
                        nextAlarm = curTick() + (cpuTimers[idx]->timecmp -
                            ((curTick() - timeDelta) / period)) * period;
                        if (cpuTimers[0]->timerAlarmEvent.scheduled())
                            deschedule(cpuTimers[0]->timerAlarmEvent);
                        schedule(cpuTimers[0]->timerAlarmEvent, nextAlarm);
                    }
                break;
                case Time:
                    if (addr == Time) {
                        warn("Writing to time register in CLINT!");
                        timeDelta = pkt->getLE<uint64_t>() - time;
                        time = pkt->getLE<uint64_t>();
                    } else {
                       panic("Invalid 4B write CLINT @ %#x.", addr);
                    }
                break;
                default:
                    panic("Invalid 4B write CLINT @ %#x.", addr);
                break;
            }
        break;
        default:
            panic("Invalid packet size for CLINT write.");
        break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
Clint::startTimer(int idx, uint64_t val)
{
    cpuTimers[idx]->startTimer(val);
}

void
Clint::setISA(RiscvISA::ISA *isa)
{
}

void
Clint::setThreadContext(ThreadContext *tc)
{
}

RegVal
Clint::readMiscReg(int misc_reg, RegVal tcId)
{
    uint32_t val;

    switch (misc_reg) {
        case RiscvISA::MISCREG_TIME:
            updateTime();
            return time;
        // The actual value of MSIP is preserved by the MSIP bit of the MIP
        // CSR, handled in arch/riscv/isa.cc.
        case RiscvISA::MISCREG_IP:
            val = (cpuTimers[tcId]->cpu->getInterruptController(tcId)->readIP()
                & 0x8) ? 0x1 : 0x0;
            DPRINTF(Timer, "CLINT timer read @ MISCREG_IP: hart[%#x]->msip = "
                "%#x.", tcId, val);
            return val;
        default:
            panic("readMiscReg to unknown register in CLINT.");
            return 0;
    }
}

void
Clint::setMiscReg(int misc_reg, RegVal val, RegVal tcId)
{
    switch (misc_reg) {
        case RiscvISA::MISCREG_TIME:
            timeDelta = val - time;
            time = val;
            return;
        // According to the SiFive FU540-C000 Manual v1, section 9.2, entitled
        // "MSIP Registers", machine mode software interrupts should be
        // generated by a write to the CLINT's MSIP register. The actual value
        // of MSIP is preserved by the MSIP bit of the MIP CSR, handled in
        // arch/riscv/isa.cc.
        case RiscvISA::MISCREG_IP:
            DPRINTF(Timer, "CLINT timer write @ MISCREG_IP, hart[%#x].", tcId);
            if (val)
                cpuTimers[tcId]->cpu->postInterrupt(tcId,
                                RiscvISA::INT_SOFTWARE_MACHINE, 0);
            return;
        default:
            panic("setMiscReg to unknown register in CLINT.");
    }
}

// Before the xtime CSR is read, it is set to avoid having to increment the
// CSR at regular intervals (thus slowing down simulation).  The CLINT timer
// clockrate is based on the HiFive Unleashed RTCCLK (1MHz).
void
Clint::updateTime()
{
    time = (curTick() / period) + timeDelta;
}

void
Clint::serialize(CheckpointOut & cp) const
{
    SERIALIZE_SCALAR(time);
    SERIALIZE_SCALAR(timeDelta);
    SERIALIZE_SCALAR(period);
    // TODO: How to serialize cpuTimers and system?
}

void
Clint::unserialize(CheckpointIn & cp)
{
    UNSERIALIZE_SCALAR(time);
    UNSERIALIZE_SCALAR(timeDelta);
    UNSERIALIZE_SCALAR(period);
    // TODO: How to unserialize cpuTimers and system?
}

Clint *
ClintParams::create()
{
    return new Clint(this);
}
