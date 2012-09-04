#!/bin/sh

old_dir=`pwd`

show_help () {
	echo
	echo "Before use, create modules with prepare_modules.sh"
	echo "and attach N9 to USB and let it mount."
	echo
	echo Usage:
	echo
	echo "sudo $0 <directory-of-mounted-device-root>"
	echo
	echo example:
	echo "sudo $0 /media/6b787b38-56e5-44fd-8fec-1b40f67770e1"
	echo
}

if [ `whoami` != root ]
then
	echo "You must be root to run the script!"
	show_help
	exit
fi

if [ ! -d mods ]
then
	show_help
	exit
fi

if [ -z "$1" ]
then
	show_help
	exit
fi

if [ ! -d "$1" ]
then
	echo "Given directory does not exist!"
	show_help
	exit
fi

echo Copying modules
cd mods
cp -R lib $1
echo "Copying zImage to /boot"
cp ../arch/arm/boot/zImage $1/boot/
cd $1/boot
echo "linking /boot/bzImage to point to zImage instead of vmlinuz*"
ln -f -s zImage bzImage
cd $old_dir
echo "Copy done."
echo
echo "If you want to revert back to old kernel"
echo "change the bzImage symlink to point back to"
echo "original vmlinuz-2.6* file in $1/boot dir"


