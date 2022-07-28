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

#ifndef __DEV_RISCV_PLIC_HH__
#define __DEV_RISCV_PLIC_HH__

#include <map>
#include <vector>

#include "arch/riscv/system.hh"
#include "dev/io_device.hh"
#include "dev/riscv/simpleboard.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/Plic.hh"

class Platform;
class SimpleBoard;
class BaseCPU;
class RiscvSystem;

// PLIC design based on the model from the RISC-V specification (separated
// document as of privileged specification v1.11, "plic.tex").  Source:
// https://github.com/riscv/riscv-isa-manual/blob/master/src/plic.tex
class Plic : public BasicPioDevice
{
  protected:
    Platform * platform;

    // Addresses of all PLIC registers according to Chapter 10 of the
    // FU540-C000 manual, Table 37, v1.0.
    std::map<Addr, uint32_t> PLICRegFile = {
        // Note: 0x0 for priority means "never interrupt", and thus disables
        // interrupts for that source.
        // Interrupt priority is greater for smaller source number and larger
        // value (maximum == Source 1 with value 0x7).  Consequently, this
        // means only the lower 3 bits of the source value mean anything.
        {0x0C000004, 0x0},      // Source 1 Priority: L2 Cache
        {0x0C000008, 0x0},      // Source 2 Priority: L2 Cache
        {0x0C00000C, 0x0},      // Source 3 Priority: L2 Cache
        {0x0C000010, 0x0},      // Source 4 Priority: UART0
        {0x0C000014, 0x0},      // Source 5 Priority: UART1
        {0x0C000018, 0x0},      // Source 6 Priority: QSPI2
        {0x0C00001C, 0x0},      // Source 7 Priority: GPIO
        {0x0C000020, 0x0},      // Source 8 Priority: GPIO
        {0x0C000024, 0x0},      // Source 9 Priority: GPIO
        {0x0C000028, 0x0},      // Source 10 Priority: GPIO
        {0x0C00002C, 0x0},      // Source 11 Priority: GPIO
        {0x0C000030, 0x0},      // Source 12 Priority: GPIO
        {0x0C000034, 0x0},      // Source 13 Priority: GPIO
        {0x0C000038, 0x0},      // Source 14 Priority: GPIO
        {0x0C00003C, 0x0},      // Source 15 Priority: GPIO
        {0x0C000040, 0x0},      // Source 16 Priority: GPIO
        {0x0C000044, 0x0},      // Source 17 Priority: GPIO
        {0x0C000048, 0x0},      // Source 18 Priority: GPIO
        {0x0C00004C, 0x0},      // Source 19 Priority: GPIO
        {0x0C000050, 0x0},      // Source 20 Priority: GPIO
        {0x0C000054, 0x0},      // Source 21 Priority: GPIO
        {0x0C000058, 0x0},      // Source 22 Priority: GPIO
        {0x0C00005C, 0x0},      // Source 23 Priority: DMA
        {0x0C000060, 0x0},      // Source 24 Priority: DMA
        {0x0C000064, 0x0},      // Source 25 Priority: DMA
        {0x0C000068, 0x0},      // Source 26 Priority: DMA
        {0x0C00006C, 0x0},      // Source 27 Priority: DMA
        {0x0C000070, 0x0},      // Source 28 Priority: DMA
        {0x0C000074, 0x0},      // Source 29 Priority: DMA
        {0x0C000078, 0x0},      // Source 30 Priority: DMA
        {0x0C00007C, 0x0},      // Source 31 Priority: DDR Subsystem
        {0x0C000080, 0x0},      // Source 32 Priority: Chiplink MSI
        {0x0C000084, 0x0},      // Source 33 Priority: Chiplink MSI
        {0x0C000088, 0x0},      // Source 34 Priority: Chiplink MSI
        {0x0C00008C, 0x0},      // Source 35 Priority: Chiplink MSI
        {0x0C000090, 0x0},      // Source 36 Priority: Chiplink MSI
        {0x0C000094, 0x0},      // Source 37 Priority: Chiplink MSI
        {0x0C000098, 0x0},      // Source 38 Priority: Chiplink MSI
        {0x0C00009C, 0x0},      // Source 39 Priority: Chiplink MSI
        {0x0C0000A0, 0x0},      // Source 40 Priority: Chiplink MSI
        {0x0C0000A4, 0x0},      // Source 41 Priority: Chiplink MSI
        {0x0C0000A8, 0x0},      // Source 42 Priority: PWM0
        {0x0C0000AC, 0x0},      // Source 43 Priority: PWM0
        {0x0C0000B0, 0x0},      // Source 44 Priority: PWM0
        {0x0C0000B4, 0x0},      // Source 45 Priority: PWM0
        {0x0C0000B8, 0x0},      // Source 46 Priority: PWM1
        {0x0C0000BC, 0x0},      // Source 47 Priority: PWM1
        {0x0C0000C0, 0x0},      // Source 48 Priority: PWM1
        {0x0C0000C4, 0x0},      // Source 49 Priority: PWM1
        {0x0C0000C8, 0x0},      // Source 50 Priority: I2C
        {0x0C0000CC, 0x0},      // Source 51 Priority: QSPI0
        {0x0C0000D0, 0x0},      // Source 52 Priority: QSPI1
        {0x0C0000D4, 0x0},      // Source 53 Priority: Gigabit Ethernet
        // Possible mistake source in SiFive manual.
        {0x0C0000D8, 0x0},      // Source 54 Priority: ???

        // Interrupt pending registers: The (N % 32)th bit indicates an
        // interrupt from that source number bit.  Given there are >32 sources,
        // the words for the interrupt pending bits are held in two registers
        // per hart and user mode.  The word is determined via (N / 32).
        {0x0C001000, 0x0},      // Start of pending array
        {0x0C001004, 0x0},      // Last word of pending array

        // Global interrupts are enabled according to set bits of following
        // registers.  Similar to the pending registers, the (N % 32)th bit of
        // the (N / 32) word is the enable bit for the corresponding global
        // interrupt.
        {0x0C002000, 0x0},      // Start hart 0 M-Mode interrupt enable
        {0x0C002004, 0x0},      // End hart 0 M-Mode interrupt enable

        {0x0C002080, 0x0},      // Start hart 1 M-Mode interrupt enable
        {0x0C002084, 0x0},      // End hart 1 M-Mode interrupt enable

        {0x0C002100, 0x0},      // Start hart 1 S-Mode interrupt enable
        {0x0C002104, 0x0},      // End hart 1 S-Mode interrupt enable

        {0x0C002180, 0x0},      // Start hart 2 M-Mode interrupt enable
        {0x0C002184, 0x0},      // End hart 2 M-Mode interrupt enable

        {0x0C002200, 0x0},      // Start hart 2 S-Mode interrupt enable
        {0x0C002204, 0x0},      // End hart 2 S-Mode interrupt enable

        {0x0C002280, 0x0},      // Start hart 3 M-Mode interrupt enable
        {0x0C002284, 0x0},      // End hart 3 M-Mode interrupt enable

        {0x0C002300, 0x0},      // Start hart 3 S-Mode interrupt enable
        {0x0C002304, 0x0},      // End hart 3 S-Mode interrupt enable

        {0x0C002380, 0x0},      // Start hart 4 M-Mode interrupt enable
        {0x0C002384, 0x0},      // End hart 4 M-Mode interrupt enable

        {0x0C002400, 0x0},      // Start hart 4 S-Mode interrupt enable
        {0x0C002404, 0x0},      // End hart 4 S-Mode interrupt enable

        // In the range [0x0:0x7], the threshold register masks all interrupts
        // with a lower priority for specified hart and mode.  For example, 0x0
        // means permit all interrupts, 0x7 means mask all interrupts.
        // The claim register then returns the ID of the highest priority
        // pending interrupt.
        {0x0C200000, 0x0},      // Hart 0 M-Mode priority threshold
        {0x0C200004, 0x0},      // Hart 0 M-mode claim/complete

        {0x0C201000, 0x0},      // Hart 1 M-Mode priority threshold
        {0x0C201004, 0x0},      // Hart 1 M-mode claim/complete

        {0x0C202000, 0x0},      // Hart 1 S-Mode priority threshold
        {0x0C202004, 0x0},      // Hart 1 S-mode claim/complete

        {0x0C203000, 0x0},      // Hart 2 M-Mode priority threshold
        {0x0C203004, 0x0},      // Hart 2 M-mode claim/complete

        {0x0C204000, 0x0},      // Hart 2 S-Mode priority threshold
        {0x0C204004, 0x0},      // Hart 2 S-mode claim/complete

        {0x0C205000, 0x0},      // Hart 3 M-Mode priority threshold
        {0x0C205004, 0x0},      // Hart 3 M-mode claim/complete

        {0x0C206000, 0x0},      // Hart 3 S-Mode priority threshold
        {0x0C206004, 0x0},      // Hart 3 S-mode claim/complete

        {0x0C207000, 0x0},      // Hart 4 M-Mode priority threshold
        {0x0C207004, 0x0},      // Hart 4 M-mode claim/complete

        {0x0C208000, 0x0},      // Hart 4 S-Mode priority threshold
        {0x0C208004, 0x0},      // Hart 4 S-mode claim/complete
    };

    RiscvSystem * system;
    std::vector<BaseCPU *> cpuTargets;

public:
    enum {
        SourcePriorityBase      = 0x0c000000,
        PendingArrayBase        = 0x0c001000,
        HartIntEnableBase       = 0x0c002000,
        HartThresholdClaimBase  = 0x0c200000
    };


    typedef PlicParams Params;
    const Params * params() const
    {return dynamic_cast<const Params *>(_params);}

    Plic(const Params *p);
    ~Plic();
    void init() override;
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    // For sending/clearing interrupts by PLIC sources
    void sendInt(int line);
    void clearInt(int line);

private:
    // Helper methods
    uint64_t getPendingArray();
    void setPendingArray(int sourceNo);
    void unsetPendingArray(int sourceNo);
    uint32_t plicClaim();
};

#endif // __DEV_RISCV_PLIC_HH__
