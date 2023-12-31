# The U-Boot loader with the datachk patchset for dualbooting requires image
# sizes and checksums to be provided in the U-Boot environment.
# The devices come with 2 main partitions - while one is active
# sysupgrade will flash the other. The boot order is changed to boot the
# newly flashed partition. If the new partition can't be booted due to
# upgrade failures the previously used partition is loaded.

platform_post_upgrade_sanity_check()
{
	local part_name=$1
	local inactive_mtd=$2
	local cfg_md5=$3
	local part_offset=$4
	local part_size=$5

	md5_part_disk=$(dd if=/dev/${inactive_mtd} bs=$((64*1024)) skip=$((part_offset / (64*1024))) count=$((part_size / (64*1024))) 2>&- | md5sum | awk '{print $1}')

	if [ "${cfg_md5}" != "${md5_part_disk}" ]; then
	      echo "post-flashing checksum mismatch: ${part_name}" >&2
	      echo "${cfg_md5} != ${md5_part_disk}"
	      return 1
	fi

	return 0
}

platform_do_upgrade_dualboot_datachk() {
	local tar_file="$1"
	local restore_backup
	local primary_kernel_mtd

	local setenv_script="/tmp/fw_env_upgrade"

	local kernel_mtd="$(find_mtd_index $PART_NAME)"
	local kernel_offset="$(cat /sys/class/mtd/mtd${kernel_mtd}/offset)"
	local total_size="$(cat /sys/class/mtd/mtd${kernel_mtd}/size)"

	# detect to which flash region the new image is written to.
	#
	# 1. check what is the mtd index for the first flash region on this
	#    device
	# 2. check if the target partition ("inactive") has the mtd index of
	#    the first flash region
	#
	#    - when it is: the new bootseq will be 1,2 and the first region is
	#      modified
	#    - when it isnt: bootseq will be 2,1 and the second region is
	#      modified
	#
	# The detection has to be done via the hardcoded mtd partition because
	# the current boot might be done with the fallback region. Let us
	# assume that the current bootseq is 1,2. The bootloader detected that
	# the image in flash region 1 is corrupt and thus switches to flash
	# region 2. The bootseq in the u-boot-env is now still the same and
	# the sysupgrade code can now only rely on the actual mtd indexes and
	# not the bootseq variable to detect the currently booted flash
	# region/image.
	#
	# In the above example, an implementation which uses bootseq ("1,2") to
	# detect the currently booted image would assume that region 1 is booted
	# and then overwrite the variables for the wrong flash region (aka the
	# one which isn't modified). This could result in a device which doesn't
	# boot anymore to Linux until it was reflashed with ap51-flash.
	local next_boot_part="1"
	case "$(board_name)" in
	plasmacloud,pax1800-v1|\
	plasmacloud,pax1800-v2)
		primary_kernel_mtd=9
		;;
	*)
		echo "failed to detect primary kernel mtd partition for board"
		return 1
		;;
	esac
	[ "$kernel_mtd" = "$primary_kernel_mtd" ] || next_boot_part="2"

	local board_dir=$(tar tf $tar_file | grep -m 1 '^sysupgrade-.*/$')
	board_dir=${board_dir%/}

	local kernel_length=$(tar xf $tar_file ${board_dir}/kernel -O | wc -c)
	local rootfs_length=$(tar xf $tar_file ${board_dir}/root -O | wc -c)
	# rootfs without EOF marker
	rootfs_length=$((rootfs_length-4))

	local kernel_md5=$(tar xf $tar_file ${board_dir}/kernel -O | md5sum); kernel_md5="${kernel_md5%% *}"
	# md5 checksum of rootfs with EOF marker
	local rootfs_md5=$(tar xf $tar_file ${board_dir}/root -O | dd bs=1 count=$rootfs_length | md5sum); rootfs_md5="${rootfs_md5%% *}"

	#
	# add tar support to get_image() to use default_do_upgrade() instead?
	#

	# take care of restoring a saved config
	[ -n "$UPGRADE_BACKUP" ] && restore_backup="${MTD_CONFIG_ARGS} -j ${UPGRADE_BACKUP}"

	mtd -q erase inactive
	tar xf $tar_file ${board_dir}/root -O | mtd -n -p $kernel_length $restore_backup write - $PART_NAME
	tar xf $tar_file ${board_dir}/kernel -O | mtd -n write - $PART_NAME

	platform_post_upgrade_sanity_check "kernel" "mtd${kernel_mtd}" $kernel_md5 0 $kernel_length || return 1
	platform_post_upgrade_sanity_check "rootfs" "mtd${kernel_mtd}" $rootfs_md5 $kernel_length $rootfs_length || return 1

	# prepare new u-boot env
	if [ "$next_boot_part" = "1" ]; then
		echo "bootseq 1,2" > $setenv_script
	else
		echo "bootseq 2,1" > $setenv_script
	fi

	printf "kernel_size_%i 0x%08x\n" $next_boot_part $kernel_length >> $setenv_script
	printf "vmlinux_start_addr 0x%08x\n" ${kernel_offset} >> $setenv_script
	printf "vmlinux_size 0x%08x\n" ${kernel_length} >> $setenv_script
	printf "vmlinux_checksum %s\n" ${kernel_md5} >> $setenv_script

	printf "rootfs_size_%i 0x%08x\n" $next_boot_part $((total_size-kernel_length)) >> $setenv_script
	printf "rootfs_start_addr 0x%08x\n" $((kernel_offset+kernel_length)) >> $setenv_script
	printf "rootfs_size 0x%08x\n" ${rootfs_length} >> $setenv_script
	printf "rootfs_checksum %s\n" ${rootfs_md5} >> $setenv_script

	# store u-boot env changes
	mkdir -p /var/lock
	fw_setenv -s $setenv_script || {
		echo "failed to update U-Boot environment"
		return 1
	}
}
