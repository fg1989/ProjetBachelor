/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Ali Saidi
 *          Nathan Binkert
 *          Jaidev Patwardhan
 *          Robert Scheffel
 * 			Joshua Klein
 *          Yasir Qureshi
 *          Marina Zapater
 *          David Atienza
 */

#include "arch/riscv/linux/system.hh"

LinuxRiscvSystem::LinuxRiscvSystem(Params *p)
  : RiscvSystem(p),
    bootloader(createObjectFile(p->bootloader))

{    bootloaderSymtab = new SymbolTable;
    if (bootloader == NULL) {
         fatal("Could not load bootloader.\n", p->bootloader);
    }

    if (!bootloader->loadGlobalSymbols(debugSymbolTable)) {
        panic("Could not load bootloader symbols.\n");
    }

    if (!bootloader->loadLocalSymbols(debugSymbolTable)) {
        panic("Could not load bootloader symbols.\n");
    }

    _resetVect = bootloader->entryPoint();
}

LinuxRiscvSystem::~LinuxRiscvSystem()
{
    delete this->bootloader;
    delete this->bootloaderSymtab;
}

void
LinuxRiscvSystem::initState()
{
    // Call the initialisation of the super class
    RiscvSystem::initState();

    if (!kernel) fatal("No kernel to load.\n");

    if (bootloader) {
        if (!bootloader->loadSections(physProxy))
            warn("Could not load bootloader sections to memory.");

        inform("Using bootloader at address %#x\n", bootloader->entryPoint());

        for (int i = 0; i < threadContexts.size(); i++) {
            threadContexts[i]->setIntReg(3, (kernelEntry & loadAddrMask) +
                            loadAddrOffset);
        }
        inform("Using kernel entry address at %#x\n",
               (kernelEntry & loadAddrMask) + loadAddrOffset);
    }

    // Load kernel symbols before MMU
    kernel->loadGlobalSymbols(kernelSymtab, 0, 0, loadAddrMask);
    kernel->loadGlobalSymbols(debugSymbolTable, 0, 0, loadAddrMask);

    // Check if the kernel image has a symbol that tells us it supports
    // device trees.
    Addr addr = 0;
    Addr dtbAddrOffset = 0x08000000 - 0x00200000;
    bool kernel_has_fdt_support =
        kernelSymtab->findAddress("unflatten_device_tree", addr);
    bool dtb_file_specified = params()->dtb_filename != "";

    if (kernel_has_fdt_support && dtb_file_specified) {
        // Kernel supports flattened device tree and dtb file specified.
        // Using Device Tree Blob to describe system configuration.
        ObjectFile *dtb_file = createObjectFile(params()->dtb_filename, true);
        if (!dtb_file) {
            fatal("couldn't load DTB file: %s\n", params()->dtb_filename);
        }

        DtbObject *_dtb_file = dynamic_cast<DtbObject*>(dtb_file);
        if (_dtb_file) {
            if (!_dtb_file->addBootCmdLine(params()->boot_osflags.c_str(),
                                           params()->boot_osflags.size())) {
                warn("couldn't append bootargs to DTB file: %s\n",
                     params()->dtb_filename);
            }
        } else {
            warn("dtb_file cast failed; couldn't append bootargs to DTB ",
                "file: %s\n", params()->dtb_filename);
        }

        // Note: addr from above leads to panic, overlaps w/ kernel region
        inform("Loading DTB file at address %#x\n", loadAddrOffset
            + dtbAddrOffset);
        dtb_file->setTextBase(loadAddrOffset + dtbAddrOffset);
        dtb_file->loadSections(physProxy);
        delete dtb_file;
    }

    // Set HW TID, M mode, and DTB address arg
    for (int i = 0; i < threadContexts.size(); i++) {
        threadContexts[i]->setMiscReg(RiscvISA::MISCREG_HARTID, i);
        threadContexts[i]->setIntReg(11, loadAddrOffset + dtbAddrOffset);
        threadContexts[i]->setMiscReg(RiscvISA::MISCREG_PRV, 0x3);
    }

    return;
}

Clint *
LinuxRiscvSystem::getClint()
{
    return clint;
}

void
LinuxRiscvSystem::setClint(Clint * c)
{
    clint = c;
}

Plic *
LinuxRiscvSystem::getPlic()
{
    return plic;
}

void
LinuxRiscvSystem::setPlic(Plic * p)
{
    plic = p;
}

LinuxRiscvSystem *
LinuxRiscvSystemParams::create()
{
    return new LinuxRiscvSystem(this);
}

