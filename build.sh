#!/bin/bash

# My custom bash script for building kernel and modules :)
# Lenovo VIBE Z2 Pro (kingdom_row)
# Electry@xda
# github.com/ElectryDev

TOOLCHAIN_PATH=~/android/toolchains/arm-cortex_a15-linux-gnueabihf-linaro_4.9.4/bin/arm-eabi-
BOOT_PATH=arch/arm/boot
BOOTFILES_PATH=bootfiles
DTCSCRIPTS_PATH=scripts/dtc
TOOLS_PATH=~/bin
OUTPUT_PATH=output
TMP_PATH=../tmp

# Begin

function MakeClean() {
	echo ">> make clean"
	make clean
}

function KingdomRowConfig() {
	echo ">> Writing .config according to kingdom_row_defconfig"
	make kingdom_row_defconfig
}

function MakeKernel() {
	echo ">> Building kernel"
	DATE_START=$(date +"%s")
	make -j4
	DATE_END=$(date +"%s")
	DIFF=$(($DATE_END - $DATE_START))
	echo ">> Build completed in $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds."
}

function CleanOld() {
	echo ">> Cleaning old files"
	rm -r $OUTPUT_PATH/
	mkdir $OUTPUT_PATH/
	mkdir $OUTPUT_PATH/modules
	rm $BOOTFILES_PATH/dt.img
	rm $BOOTFILES_PATH/zImage
}

function MakeBoot() {
	echo ">> Copying zImage > kernel"
	cp $BOOT_PATH/zImage $BOOTFILES_PATH/kernel
	echo ">> Creating dt.img"
	$TOOLS_PATH/dtbToolCM -2 -s 2048 -o $BOOTFILES_PATH/dt.img -p $DTCSCRIPTS_PATH/ $BOOT_PATH/
	echo ">> Creating boot.img"
	$TOOLS_PATH/mkboot $BOOTFILES_PATH $OUTPUT_PATH/boot.img
}

function CopyModules() {
	echo ">> Copying modules"
	mkdir $TMP_PATH/
	find . -name "*.ko" -exec cp {} $TMP_PATH/ \;
	find $TMP_PATH/ -name "*.ko" -exec mv {} $OUTPUT_PATH/modules/ \;
	rmdir $TMP_PATH/

}





# Begin

export USE_CCACHE=1

export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=$TOOLCHAIN_PATH

echo "Do you want to start automatic building?"

read -p "(A = automatic, M = manual):" ANSWER
	case $ANSWER in
		a*|A*)
		KingdomRowConfig
		MakeKernel
		CleanOld
		MakeBoot
		#CopyModules
		;;
			m*|M*)
			echo "Exiting..."
			;;
				*)
				echo "Exiting..."
				;;
	esac

