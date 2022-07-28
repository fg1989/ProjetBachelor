/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
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
 * Authors: Alec Roelke
 *          Robert Scheffel
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */
#include "arch/riscv/faults.hh"

#include "arch/riscv/isa.hh"
#include "arch/riscv/registers.hh"
#include "arch/riscv/system.hh"
#include "arch/riscv/utility.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/RiscvFault.hh"
#include "sim/debug.hh"
#include "sim/full_system.hh"

namespace RiscvISA
{

void
RiscvFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Fault %s encountered at pc 0x%016llx.", name(), tc->pcState().pc());
}

void
RiscvFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    PCState pcState = tc->pcState();

    if (FullSystem) {
        PrivilegeMode prv = PRV_M;
        STATUS status = tc->readMiscReg(MISCREG_STATUS);
        PrivilegeMode pp = (PrivilegeMode)tc->readMiscReg(MISCREG_PRV);

        // Set fault handler privilege mode
        if (isInterrupt()) {
            if (pp != PRV_M && bits(tc->readMiscReg(MISCREG_MIDELEG), _code))
                prv = PRV_S;
            if (pp == PRV_U && bits(tc->readMiscReg(MISCREG_SIDELEG), _code))
                prv = PRV_U;
        } else {
            if (pp != PRV_M && bits(tc->readMiscReg(MISCREG_MEDELEG), _code))
                prv = PRV_S;
            if (pp == PRV_U && bits(tc->readMiscReg(MISCREG_SEDELEG), _code))
                prv = PRV_U;
        }

        // Set fault registers and status
        MiscRegIndex cause, epc, tvec, tval;
        switch (prv) {
          case PRV_U:
            cause = MISCREG_UCAUSE;
            epc = MISCREG_UEPC;
            tvec = MISCREG_UTVEC;
            tval = MISCREG_UTVAL;

            status.upie = status.uie;
            status.uie = 0;
            break;
          case PRV_S:
            cause = MISCREG_SCAUSE;
            epc = MISCREG_SEPC;
            tvec = MISCREG_STVEC;
            tval = MISCREG_STVAL;

            status.spp = pp;
            status.spie = status.sie;
            status.sie = 0;
            break;
          case PRV_M:
            cause = MISCREG_MCAUSE;
            epc = MISCREG_MEPC;
            tvec = MISCREG_MTVEC;
            tval = MISCREG_MTVAL;

            status.mpp = pp;
            status.mpie = status.mie;
            status.mie = 0;
            break;
          default:
            panic("Unknown privilege mode %d.", prv);
            break;
        }

        // Set fault cause, privilege, and return PC.  Note that the top bit of
        // xcause is 1 for interrupts (bit 64 in RV64).
        tc->setMiscReg(cause, ((uint64_t)isInterrupt() << 63) | _code);
        tc->setMiscReg(epc, tc->instAddr());
        tc->setMiscReg(tval, trap_value());
        tc->setMiscReg(MISCREG_PRV, prv);
        tc->setMiscReg(MISCREG_STATUS, status);

        // Use entirety of xtvec for handler address with mode zeroed.
        Addr addr = (tc->readMiscReg(tvec) & (~0x3));

        if (isInterrupt() && (tc->readMiscReg(tvec) & 0x3) == 1)
            addr += 4 * _code;

        pcState.set(addr);

        // if (isInterrupt())
        //     warn("RV Fault @ %d: _code = %#x, isInt = %#x, addr = %#x, "
        //         "pp = %#x.", curTick(), _code, isInterrupt(), addr, prv);
        DPRINTF(RiscvFault, "RV Fault: _code = %d, isInt = %#x, addr = %#x."
            "\n", _code, isInterrupt(), addr);
    } else {
        invokeSE(tc, inst);
        advancePC(pcState, inst);
    }
    tc->pcState(pcState);
}

void Reset::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    tc->setMiscReg(MISCREG_PRV, PRV_M);
    STATUS status = tc->readMiscReg(MISCREG_STATUS);
    status.mie = 0;
    status.mprv = 0;
    tc->setMiscReg(MISCREG_STATUS, status);
    tc->setMiscReg(MISCREG_MCAUSE, 0);

    // Advance the PC to the implementation-defined reset vector
    PCState pc = static_cast<RiscvSystem *>(tc->getSystemPtr())->resetVect();
    tc->pcState(pc);
}

void
UnknownInstFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Unknown instruction 0x%08x at pc 0x%016llx", inst->machInst,
        tc->pcState().pc());
}

void
IllegalInstFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Illegal instruction 0x%08x at pc 0x%016llx: %s", inst->machInst,
        tc->pcState().pc(), reason.c_str());
}

void
UnimplementedFault::invokeSE(ThreadContext *tc,
        const StaticInstPtr &inst)
{
    panic("Unimplemented instruction %s at pc 0x%016llx", instName,
        tc->pcState().pc());
}

void
IllegalFrmFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Illegal floating-point rounding mode %#x at pc 0x%016llx.",
            frm, tc->pcState().pc());
}

void
BreakpointFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    schedRelBreak(0);
}

void
SyscallFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    Fault *fault = NoFault;
    tc->syscall(tc->readIntReg(SyscallNumReg), fault);
}

} // namespace RiscvISA
