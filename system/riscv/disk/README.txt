# Copyright (c) 2020 EPFL
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
# Authors: Joshua Klein 
#          Yasir Qureshi
#          Marina Zapater
#          David Atienza

This folder contains any necessary documents/files needed to recreated a working RISC-V disk image, as well as this README to document and annotate the disk image-making process.

For testing we used a buildroot (version 2020.02.1) [1] minimal rootfs with a BusyBox [2] init system.  The first step to recreating this image is to download buildroot and follow the getting started guide in its documentation.  Once buildroot is set up, you can copy the buildroot.config file in this folder to buildroot/.config and call "make" in buildroot/.  buildroot should automagically download the entire toolchain necessary to make the rootfs file.  The rootfs file will then be available in buildroot/output/images as "rootfs.etx2".  Note that even though the build configuration specified making an ext4 rootfs, this only appears as a link (a.k.a., don't use the file "rootfs.ext4").

To configure the buildroot image, run "make nconfig" in the main directory and there are numerous options available to customize the rootfs using the menu, and if you so desire, the linux kernel and bootloader as well.  However in our testing, we only used buildroot to make the rootfs.

By default, the rootfs file will be a 60MB image with only the minimal root file system.  The username and password are set to root/buildroot by default but this can be changed in the nconfig menu for buildroot, or for auto-login, in /etc/inittab of rootfs.ext2  Note that in order to use this image, it must be on your gem5 file path, so it is recommended you move/copy it to gem5-V/full_system_images/disks.

To verify you have a working disk image (after you've verified you have a working vmlinux file, device tree binary, and bootloader), the buildroot image should work with RISC-V QEMU.

Finally, because the rootfs is minimal it will not have utilities such as gcc.  Therefore if you want to run your own binaries, you will need to cross-compile using the RISC-V cross-compiler (included with the RISC-V toolchain [3]) and then copy the binaries into the rootfs.  You can mount/unmount the rootfs using the following:

> sudo mount <path_to_rootfs>/rootfs.ext2 /<path_to_mount_point>
> sudo umount /<path_to_mount_point>

[1] https://buildroot.org/
[2] https://www.busybox.net/
[3] https://github.com/riscv/riscv-gnu-toolchain
