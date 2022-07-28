/*
 * Copyright (c) 2011 Google
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
 * Authors: Gabe Black
 *          Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#ifndef __ARCH_RISCV_INTERRUPT_HH__
#define __ARCH_RISCV_INTERRUPT_HH__

#include <bitset>
#include <memory>

#include "arch/riscv/faults.hh"
#include "arch/riscv/registers.hh"
#include "base/logging.hh"
#include "cpu/thread_context.hh"
#include "debug/Interrupt.hh"
#include "params/RiscvInterrupts.hh"
#include "sim/sim_object.hh"

class BaseCPU;
class ThreadContext;

namespace RiscvISA {

/*
 * This is based on version 1.10 of the RISC-V privileged ISA reference,
 * chapter 3.1.14.
 */
class Interrupts : public SimObject
{
  private:
    BaseCPU * cpu;
    std::bitset<NumInterruptTypes> ip;
    std::bitset<NumInterruptTypes> ie;

    // Important bits for QEMU-like CSR shadowing.
    const uint64_t M_MODE_INT = 0x888;
    const uint64_t S_MODE_INT = 0x222;
    const uint64_t U_MODE_INT = 0x111;

  public:
    typedef RiscvInterruptsParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Interrupts(Params * p) : SimObject(p), cpu(nullptr), ip(0), ie(0) {}

    void setCPU(BaseCPU * _cpu) { cpu = _cpu; }

    std::bitset<NumInterruptTypes>
    globalMask(ThreadContext *tc) const
    {
        // From privileged specification 1.11, section 3.1.6.1:
        // "When a hart is executing in privilege mode x, interrupts are
        // globally enabled when xIE=1 and globally disabled when xIE=0.
        // Interrupts for lower-privilege modes, w<x, are always globally
        // disabled regardless of the setting of the lower-privilege mode’s
        // global wIE bit. Interrupts for higher-privilege modes, y>x, are
        // always globally enabled regardless of the setting of the higher-
        // privilege mode’s global yIE bit. Higher-privilege-level code can
        // use separate per-interrupt enable bits to disable selected higher-
        // privilege-mode interrupts before ceding control to a lower-privilege
        // mode."
        INTERRUPT mask = 0;
        STATUS status = tc->readMiscReg(MISCREG_STATUS);
        uint64_t prv = tc->readMiscReg(MISCREG_PRV);

        switch(prv) {
            case PRV_M:
                mask.mei = mask.mti = mask.msi = (status.mie ? 1 : 0);
                mask.sei = mask.sti = mask.ssi = 0;
                mask.uei = mask.uti = mask.usi = 0;
            break;
            case PRV_S:
                mask.mei = mask.mti = mask.msi = 1;
                mask.sei = mask.sti = mask.ssi = (status.sie ? 1 : 0);
                mask.uei = mask.uti = mask.usi = 0;
            break;
            case PRV_U:
                mask.mei = mask.mti = mask.msi = 1;
                mask.sei = mask.sti = mask.ssi = 1;
                mask.uei = mask.uti = mask.usi = (status.uie ? 1 : 0);
            break;
            default:
                panic("Invalid privilege mode in Interrupts::globalMask!");
        }
        return std::bitset<NumInterruptTypes>(mask);
    }

    bool checkInterrupt(int num) const
    {
        DPRINTF(Interrupt, "Interrupts::checkInterrupt: ip[%#x] = %#x, "
            "ie[%#x] = %#x.\n",
            num, ip[num], num, ie[num]);
        return ip[num] && ie[num];
    }

    bool checkInterrupts(ThreadContext *tc) const
    {
        DPRINTF(Interrupt, "Interrupts::checkInterrupts: ip = %#x, ie = %#x, "
            "mask = %#x, returning %#x.\n",
            ip, ie, globalMask(tc), (ip & ie & globalMask(tc)).any());
        return (ip & ie & globalMask(tc)).any();
    }

    Fault
    getInterrupt(ThreadContext *tc) const
    {
        assert(checkInterrupts(tc));
        std::bitset<NumInterruptTypes> mask = globalMask(tc);
        DPRINTF(Interrupt, "Interrupts::getInterrupt called.\n");

        // According to the RISC-V privileged specification, version 1.11,
        // section 3.1.9, "Machine Interrupt Registers (mip and mie)",
        // interrupt handling priorities are sorted as the following: MEI, MSI,
        // MTI, SEI, SSI, STI, UEI, USI, UTI, followed by synchronous
        // exceptions.
        int priorities[] = {11, 3, 7, 9, 1, 5, 8, 0, 4};
        for (auto c : priorities) {
            if (checkInterrupt(c) && mask[c]) {
                DPRINTF(Interrupt, "Interrupts::getInterrupt returning fault "
                    "with code %x.\n", c);
                return std::make_shared<InterruptFault>(c);
            }
        }
        return NoFault;
    }

    void updateIntrInfo(ThreadContext *tc) {}

    void
    post(int int_num, int index)
    {
        DPRINTF(Interrupt, "Interrupt %d:%d posted\n", int_num, index);
        ip[int_num] = true;
        if (int_num == 7 || int_num == 3)
            ie[int_num] = true;
    }

    void
    clear(int int_num, int index)
    {
        DPRINTF(Interrupt, "Interrupt %d:%d cleared\n", int_num, index);
        ip[int_num] = false;
    }

    void
    clearAll()
    {
        DPRINTF(Interrupt, "All interrupts cleared\n");
        ip = 0;
    }

    uint64_t
    readIP() const
    {
        DPRINTF(Interrupt, "Interrupts::readIP: Read IP %#x.\n",
            (uint64_t)ip.to_ulong());
        return ip.to_ulong();
    }

    uint64_t
    readIE() const
    {
        DPRINTF(Interrupt, "Interrupts::readIE: Read IE %#x.\n",
            (uint64_t)ie.to_ulong());
        return (uint64_t)ie.to_ulong();
    }

    void
    setIP(const uint64_t& val, uint64_t prv)
    {
        DPRINTF(Interrupt, "Interrupts::setIP: Set IP to %#x.\n", val);
        // Implement CSR privilege mode shadowing.
        switch (prv) {
            case PRV_M:
                ip = val;
            break;
            case PRV_S:
                ip = (ip.to_ulong() & ~S_MODE_INT) | (val & S_MODE_INT);
            break;
            case PRV_U:
                ip = (ip.to_ulong() & ~U_MODE_INT) | (val & U_MODE_INT);
            break;
        }

        return;
    }

    void
    setIE(const uint64_t& val, uint64_t prv)
    {
        DPRINTF(Interrupt, "Interrupts::setIE: Set IE to %#x.\n", val);
        // Implement CSR privilege mode shadowing.
        switch (prv) {
            case PRV_M:
                ie = val;
            break;
            case PRV_S:
                ie = (ie.to_ulong() & ~S_MODE_INT) | (val & S_MODE_INT);
            break;
            case PRV_U:
                ie = (ie.to_ulong() & ~U_MODE_INT) | (val & U_MODE_INT);
            break;
        }

        return;
    }

    void
    serialize(CheckpointOut &cp) const
    {
        SERIALIZE_SCALAR(ip.to_ulong());
        SERIALIZE_SCALAR(ie.to_ulong());
    }

    void
    unserialize(CheckpointIn &cp)
    {
        long reg;
        UNSERIALIZE_SCALAR(reg);
        ip = reg;
        UNSERIALIZE_SCALAR(reg);
        ie = reg;
    }
};

} // namespace RiscvISA

#endif // __ARCH_RISCV_INTERRUPT_HH__
