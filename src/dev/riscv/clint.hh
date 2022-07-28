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

#ifndef __DEV_RISCV_CLINT_HH__
#define __DEV_RISCV_CLINT_HH__

#include <vector>

#include "arch/riscv/isa.hh"
#include "arch/riscv/isa_device.hh"
#include "arch/riscv/registers.hh"
#include "arch/riscv/system.hh"
#include "dev/io_device.hh"
#include "params/Clint.hh"

class BaseCPU;
class Clint;
class RiscvSystem;

// Class containing per-HART timecmp register, in addition to its own event
// queue management.  The MSIP register is left to be handled by the MIP CSR in
// arch/riscv/isa.*.
class CpuTimer : public Serializable
{
public:
    uint64_t timecmp;
    uint64_t hartId;
    EventFunctionWrapper timerAlarmEvent;
    BaseCPU * cpu;
    Clint * parent;

    CpuTimer(BaseCPU * cpu, Clint * parent, uint64_t hartId);
    ~CpuTimer();
    void startTimer(uint64_t val);
    void timerAlarm();
    void serialize(CheckpointOut & cp) const override;
    void unserialize(CheckpointIn & cp) override;
};

class Clint : public BasicPioDevice, public RiscvISA::BaseISADevice
{
private:
    // Per-HART/CPU timers
    std::vector<CpuTimer *> cpuTimers;

    // mtime register
    uint64_t time;

    // Should there be a write to mtime, this stores the difference given how
    // the mtime value is a function of clock period and current tick.
    int timeDelta;

    // Clock period for CLINT
    Tick period;

    // System CLINT belongs to.
    RiscvSystem * system;

public:
    // Register base addresses according to SiFive FU540-C000 CLINT memory map
    // (Table 36 in manual).  IP registers are 32 bit with an offset of 4,
    // timecmp registers are 64 bit with an offset of 0x8.
    enum {
        InterruptPendingBase  = 0x0000,
        TimeCmpBase           = 0x4000,
        Time                  = 0xbff8
    };
    typedef ClintParams Params;
    const Params *
    params() const
    {
      return dynamic_cast<const Params *>(_params);
    }

    Clint(Params *p);
    ~Clint();
    void init() override;

    // Wrapper for HART-granularity timer start
    // Only if value written to timecmp?
    void startTimer(int idx, uint64_t val);

    // Handles a read/write to the timer device by address.
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    // Required functions from BaseISADevice
    void setISA(RiscvISA::ISA *isa);
    void setThreadContext(ThreadContext *tc);

    // Handles a read/write to the timer device by register file.
    RegVal readMiscReg(int misc_reg, RegVal tcId);
    void setMiscReg(int misc_reg, RegVal val, RegVal tcId);

    // Updates time register
    void updateTime();

    void serialize(CheckpointOut & cp) const override;
    void unserialize(CheckpointIn & cp) override;

    friend class CpuTimer;
};

#endif // __DEV_RISCV_CLINT_HH__
