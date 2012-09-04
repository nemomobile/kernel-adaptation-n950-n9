#!/bin/sh

KernelVer=3.5.3
ModuleRoot=mods

# Disable parameter support for now, as copy script assumes mods.
#if [ -n "$1" ]
#then
#ModuleRoot=$1
#fi

if [ ! -d $ModuleRoot/lib ]
then
	echo "No modules install directory available!"
	echo "Please install modules to default \"mods\""
	echo "directory like this from Mer SDK:"
	echo "  mkdir mods"
	echo "  sb2 make modules_install INSTALL_MOD_PATH=./mods"
# 	echo "..or provide the directory as parameter:"
#	echo "$0 <my-INSTALL_MOD_PATH>"
	exit
fi

echo "Removing old build and source symlinks"

rm -rf $ModuleRoot/lib/modules/$KernelVer/build
rm -rf $ModuleRoot/lib/modules/$KernelVer/source
mkdir -p $ModuleRoot/lib/modules/$KernelVer/build
(cd $ModuleRoot/lib/modules/$KernelVer/; mkdir -p $ModuleRoot/lib/modules/$KernelVer/build)

echo Populating lib/modules/$KernelVer/build

cp --parents `find  -type f -name "Makefile*" -o -name "Kconfig*"` $ModuleRoot/lib/modules/$KernelVer/build
cp Module.symvers $ModuleRoot/lib/modules/$KernelVer/build
cp System.map $ModuleRoot/lib/modules/$KernelVer/build
if [ -s Module.markers ]; then
	cp Module.markers $ModuleRoot/lib/modules/$KernelVer/build
fi
rm -rf $ModuleRoot/lib/modules/$KernelVer/build/Documentation
rm -rf $ModuleRoot/lib/modules/$KernelVer/build/scripts
rm -rf $ModuleRoot/lib/modules/$KernelVer/build/include

cp .config $ModuleRoot/lib/modules/$KernelVer/build
cp -a scripts $ModuleRoot/lib/modules/$KernelVer/build
if [ -d arch/arm/scripts ]; then
	cp -a arch/arm/scripts $ModuleRoot/lib/modules/$KernelVer/build/arch/arm || :
fi
if [ -f arch/arm/*lds ]; then
	cp -a arch/arm/*lds $ModuleRoot/lib/modules/$KernelVer/build/arch/arm/ || :
fi

rm -f $ModuleRoot/lib/modules/$KernelVer/build/scripts/*.o
rm -f $ModuleRoot/lib/modules/$KernelVer/build/scripts/*/*.o

echo Adding kernel headers

cp -a --parents arch/arm/include $ModuleRoot/lib/modules/$KernelVer/build

cp -a --parents arch/arm/mach-*/include $ModuleRoot/lib/modules/$KernelVer/build
cp -a --parents arch/arm/plat-*/include $ModuleRoot/lib/modules/$KernelVer/build

mkdir -p $ModuleRoot/lib/modules/$KernelVer/build/include
cd include
# HARMATTAN: no include/generated -> remove
cp -a acpi asm-generic config crypto drm keys linux math-emu media mtd net pcmcia rdma rxrpc scsi sound video trace ../$ModuleRoot/lib/modules/$KernelVer/build/include

# Make sure the Makefile and version.h have a matching timestamp
# so that external modules can be built
touch -r ../$ModuleRoot/lib/modules/$KernelVer/build/Makefile ../$ModuleRoot/lib/modules/$KernelVer/build/include/linux/version.h
touch -r ../$ModuleRoot/lib/modules/$KernelVer/build/.config ../$ModuleRoot/lib/modules/$KernelVer/build/include/linux/autoconf.h
# Copy .config to include/config/auto.conf so "make prepare" is unnecessary.
cp ../$ModuleRoot/lib/modules/$KernelVer/build/.config ../$ModuleRoot/lib/modules/$KernelVer/build/include/config/auto.conf
cd ..
echo All done.
echo "Kernel modules now ready in $ModuleRoot/"
echo
echo "Use copy_kernel.sh to copy modules and"
echo "zImage to your N9 and make it run on next"
echo "boot."





