/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
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
 * Authors: Alec Roelke
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */
#include "arch/riscv/isa.hh"

#include <ctime>
#include <set>
#include <sstream>

#include "arch/riscv/registers.hh"
#include "arch/riscv/tlb.hh"
#include "base/bitfield.hh"
#include "cpu/base.hh"
#include "debug/MiscRegs.hh"
#include "debug/RiscvISAMisc.hh"
#include "debug/RiscvMisc.hh"
#include "params/RiscvISA.hh"
#include "sim/core.hh"
#include "sim/full_system.hh"
#include "sim/pseudo_inst.hh"
#include "sim/system.hh"

namespace RiscvISA
{

    ISA::ISA(Params *p)
        : SimObject(p),
          system(dynamic_cast<RiscvSystem *>(p->system))
    {
        miscRegFile.resize(NumMiscRegs);
        clear();
    }

    const RiscvISAParams *
    ISA::params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    void ISA::clear()
    {
        std::fill(miscRegFile.begin(), miscRegFile.end(), 0);

        miscRegFile[MISCREG_PRV] = PRV_M;
        miscRegFile[MISCREG_MISA] = (2ULL << MXL_OFFSET) | 0x14112D;
        miscRegFile[MISCREG_VENDORID] = 0;
        miscRegFile[MISCREG_ARCHID] = 0;
        miscRegFile[MISCREG_IMPID] = 0;
        miscRegFile[MISCREG_STATUS] = (2ULL << UXL_OFFSET) | (2ULL << SXL_OFFSET) |
                                      (1ULL << FS_OFFSET);
        miscRegFile[MISCREG_COUNTEREN] = 0x7;
    }

    bool
    ISA::hpmCounterEnabled(int misc_reg) const
    {
        int hpmcounter = misc_reg - MISCREG_CYCLE;
        if (hpmcounter < 0 || hpmcounter > 31)
            panic("Illegal HPM counter %d\n", hpmcounter);
        int counteren;
        switch (readMiscRegNoEffect(MISCREG_PRV))
        {
        case PRV_M:
            return true;
        case PRV_S:
        case PRV_U:
            counteren = MISCREG_COUNTEREN;
            break;
        default:
            panic("Unknown privilege level %d\n", miscRegFile[MISCREG_PRV]);
            return false;
        }
        return (miscRegFile[counteren] & (1ULL << (hpmcounter))) > 0;
    }

    RegVal
    ISA::readMiscRegNoEffect(int misc_reg) const
    {
        if (misc_reg > NumMiscRegs || misc_reg < 0)
        {
            // Illegal CSR
            panic("Illegal CSR index %#x\n", misc_reg);
            return -1;
        }

        DPRINTF(MiscRegs, "Reading MiscReg %d: %#llx.\n", misc_reg,
                miscRegFile[misc_reg]);
        return miscRegFile[misc_reg];
    }

    RegVal
    ISA::readMiscReg(int misc_reg, ThreadContext *tc)
    {
        RegVal val;
        switch (misc_reg)
        {
        case MISCREG_STATUS:
            switch (miscRegFile[MISCREG_PRV])
            {
            case PRV_M:
                // Read mstatus
                return readMiscRegNoEffect(misc_reg);
            case PRV_S:
                // Read sstatus
                return miscRegFile[MISCREG_STATUS] & SSTATUS_MASK;
            case PRV_U:
                // Read ustatus
                return miscRegFile[MISCREG_STATUS] & USTATUS_MASK;
            default:
                panic("Write to xstatus CSR without valid mode.");
            }
        case MISCREG_HARTID:
            return tc->contextId();
        case MISCREG_CYCLE:
            if (hpmCounterEnabled(MISCREG_CYCLE))
            {
                DPRINTF(RiscvMisc, "Cycle counter at: %llu.\n",
                        tc->getCpuPtr()->curCycle());
                return tc->getCpuPtr()->curCycle();
            }
            else
            {
                warn("Cycle counter disabled.\n");
                return 0;
            }
        case MISCREG_TIME:
            // In Linux FS mode, must get timer from CLINT
            if (FullSystem && !system->isBareMetal())
            {
                return system->getClint()->readMiscReg(misc_reg, tc->contextId());
            }
            else if (hpmCounterEnabled(MISCREG_TIME))
            {
                DPRINTF(RiscvMisc, "Wall-clock counter at: %llu.\n",
                        std::time(nullptr));
                return std::time(nullptr);
            }
            else
            {
                warn("Wall clock disabled.\n");
                return 0;
            }
        case MISCREG_INSTRET:
            if (hpmCounterEnabled(MISCREG_INSTRET))
            {
                DPRINTF(RiscvMisc, "Instruction counter at: %llu.\n",
                        tc->getCpuPtr()->totalInsts());
                return tc->getCpuPtr()->totalInsts();
            }
            else
            {
                warn("Instruction counter disabled.\n");
                return 0;
            }
        case MISCREG_IP:
            val = tc->getCpuPtr()->getInterruptController(tc->threadId())->readIP();
            return val;
        case MISCREG_IE:
            val = tc->getCpuPtr()->getInterruptController(tc->threadId())->readIE();
            return val;
        default:
            // Try reading HPM counters
            // As a placeholder, all HPM counters are just cycle counters
            if (misc_reg >= MISCREG_HPMCOUNTER03 &&
                misc_reg <= MISCREG_HPMCOUNTER31)
            {
                if (hpmCounterEnabled(misc_reg))
                {
                    DPRINTF(RiscvMisc, "HPM counter %d: %llu.\n",
                            misc_reg - MISCREG_CYCLE, tc->getCpuPtr()->curCycle());
                    return tc->getCpuPtr()->curCycle();
                }
                else
                {
                    warn("HPM counter %d disabled.\n", misc_reg - MISCREG_CYCLE);
                    return 0;
                }
            }
            return readMiscRegNoEffect(misc_reg);
        }
    }

    void
    ISA::setMiscRegNoEffect(int misc_reg, RegVal val)
    {
        if (misc_reg > NumMiscRegs || misc_reg < 0)
        {
            // Illegal CSR
            panic("Illegal CSR index %#x\n", misc_reg);
        }

        DPRINTF(MiscRegs, "Setting MiscReg %d to %#x.\n", misc_reg, val);
        miscRegFile[misc_reg] = val;
    }

    void
    ISA::setMiscReg(int misc_reg, RegVal val, ThreadContext *tc)
    {
        if (misc_reg >= MISCREG_CYCLE && misc_reg <= MISCREG_HPMCOUNTER31)
        {
            // Ignore writes to HPM counters for now
            warn("Ignoring write to %s.\n", CSRData.at(misc_reg).name);
        }
        else
        {
            switch (misc_reg)
            {
            case MISCREG_STATUS:
                uint64_t mask;
                switch (miscRegFile[MISCREG_PRV])
                {
                case PRV_M:
                    // Write mstatus
                    setMiscRegNoEffect(misc_reg, val);
                    return;
                case PRV_S:
                    // Write sstatus
                    mask = SSTATUS_MASK;
                    miscRegFile[MISCREG_STATUS] =
                        (miscRegFile[MISCREG_STATUS] & ~mask) | (val & mask);
                    return;
                case PRV_U:
                    // Write ustatus
                    mask = USTATUS_MASK;
                    miscRegFile[MISCREG_STATUS] =
                        (miscRegFile[MISCREG_STATUS] & ~mask) | (val & mask);
                    return;
                default:
                    panic("Write to xstatus CSR without valid mode.");
                }
            case MISCREG_IP:
                if (FullSystem && !system->isBareMetal() && (val & 0x8))
                    system->getClint()->setMiscReg(misc_reg, 0x1, tc->contextId());
                return tc->getCpuPtr()->getInterruptController(tc->threadId())->setIP(val, miscRegFile[MISCREG_PRV]);
            case MISCREG_IE:
                return tc->getCpuPtr()->getInterruptController(tc->threadId())->setIE(val, miscRegFile[MISCREG_PRV]);
            case MISCREG_TIME:
                if (FullSystem && !system->isBareMetal())
                    system->getClint()->setMiscReg(misc_reg, val, tc->contextId());
                setMiscRegNoEffect(misc_reg, val);
                return;
            case MISCREG_SATP:
                tc->getITBPtr()->flushAll();
                tc->getDTBPtr()->flushAll();
                setMiscRegNoEffect(misc_reg, val);
                return;
            default:
                setMiscRegNoEffect(misc_reg, val);
                return;
            }
        }
    }
}

RiscvISA::ISA *
RiscvISAParams::create()
{
    return new RiscvISA::ISA(this);
}
