/*
    mini ramdump module

    This is lenovo midh mrd driver implement.

    Author:Kerry Xi
    Date: Sep, 2013
    Copy Right: Lenovo 2013
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/export.h>
#include <linux/proc_fs.h>
#include "le_mrd.h"
#include <mach/msm_iomap.h>
#include <mach/msm_smem.h>
#include <linux/uaccess.h>

//imem flag for bootloader that store the MRD_SMEM_TRIGGER_MAGIC_NUMBER flag
#define FIX_IMEM_MRD_DUMP_SIGNATURE_OFFSET 0xC   //reuse the uefi flag

//the mrd shareindex table physical address
#define FIX_IMEM_MRD_DUMP_TABLE_ADDR 0x34             //use a new imem flag section

//share index pair variable that defines both the pa and va of shareindex memory
static le_mrd_shareindex_pair mrd_shareindex_data;

extern void imem_write(int offset,unsigned int val);

void mrd_set_mrdmode(int mode);
static int mrd_init_status=0;

#define MRD_USE_SMEM
static void * mrd_alloc_share_index(int size)
{
	void *ret = NULL;
#ifdef MRD_USE_SMEM
	ret = smem_alloc(SMEM_ID_VENDOR_MRD_INDEX,size);
	if (!ret) {
		pr_info("%s:smem_alloc fail, try smem_alloc2\n",__func__);
		ret = smem_alloc2(SMEM_ID_VENDOR_MRD_INDEX,size);
	}
#else
	ret = kzalloc(size, GFP_KERNEL);
#endif
	return ret;
}

static inline unsigned long get_physical_address(unsigned long va)
{
#ifdef MRD_USE_SMEM
	extern phys_addr_t msm_shared_ram_phys;
	return (va - (unsigned long)MSM_SHARED_RAM_BASE) + msm_shared_ram_phys;
#else
	return virt_to_phys(table);
#endif
}

int boot_rtc_seconds=0;

static int mrd_debug=0;

static int __init mrd_setup_debug(char *str)
{
	unsigned int val = memparse(str, &str);

	if (val)
		mrd_debug = val;

	return 0;
}

early_param("mrd_dbg",mrd_setup_debug);

//register one shareindex item for external module
int mrd_register_shareindex_name(char* name, void* va, int size, unsigned int param1)
{
	mrd_shareindex_item_t  index;

	if (!mrd_init_status) {
		pr_info("%s: the mrd is not initialized\n",__func__);
		return -1;
	}

	//register the dmesg log info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, name, MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = virt_to_phys(va);
	index.size = size;
	index.param1 = param1;
	return mrd_register_shareindex(&index);
}

//register one shareindex
int mrd_register_shareindex(mrd_shareindex_item_t* index)
{
	mrd_shareindex_item_t* avail_index = NULL;
	if ( (mrd_shareindex_data.shareindex_table_ptr)->index_count == (mrd_shareindex_data.shareindex_table_ptr)->total_count) {
		pr_err("%s: the shareindex area is full, can not register new index\n", __func__);
		return -1;
	}

	if (index->name[MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH-1]) {
		pr_err("%s:the mrd shareindex name is too long.\n",__func__);
		return -1;
	}

	avail_index = (mrd_shareindex_item_t* ) ((unsigned char*)mrd_shareindex_data.shareindex_table_ptr
			+ mrd_shareindex_data.shareindex_table_ptr->head_size
			+ sizeof(mrd_shareindex_item_t)*mrd_shareindex_data.shareindex_table_ptr->index_count);

	mrd_shareindex_data.shareindex_table_ptr->index_count++;

	memcpy(avail_index, index, sizeof(mrd_shareindex_item_t));

	avail_index->subsys = MRD_SUBSYS_KRAIT;
	pr_info("%s:register shareindex item %s, size %d\n",__func__,index->name, index->size);
	if (mrd_debug)
		pr_info("%s:register shareindex item: \n"
				"  index va  :0x%p\n"
				"      name  :%s\n"
				"      pa    :0x%08x\n"
				"      size  :0x%x\n"
				"      subsys:%d\n",
				__func__,
				avail_index,
				index->name,
				index->address,
				index->size,
				avail_index->subsys);
	return 0;
};

//default value
static int mrd_mode=MRD_MODE_DLOADMODE;

//when normal reboot or reboot to dloadmode, it will enter into normal dloadmode.
void mrd_unset_miniramdump(void)
{
	mrd_set_mrdmode(MRD_MODE_DLOADMODE);
	imem_write(FIX_IMEM_MRD_DUMP_SIGNATURE_OFFSET,0);
	imem_write(FIX_IMEM_MRD_DUMP_TABLE_ADDR, 0);
	pr_err( "%s: unset the mrdmode\n",__func__);
}

/*set system mrd mode
param:
1: enter into mrd mode, and reboot after collect mrd info
0: enter into mrd mode, and enter into dload mode after collect mrd info
*/
void mrd_set_mrdmode(int mode)
{
	mrd_shareindex_header_t *table;
	table = mrd_shareindex_data.shareindex_table_ptr;
	if (mode == MRD_MODE_MRDMODE)   //mode is mrdmode
		table->mrdmode = MRD_MODE_MRDMODE;
	else if (mode == MRD_MODE_DLOADMODE)
		table->mrdmode = MRD_MODE_DLOADMODE;
	else
		pr_err("%s:Error:invalid mode parameter %d\n",__func__,mode);

	pr_err("%s:set the mrdmode %s,value=%d\n",__func__,mode?"true":"false",table->mrdmode);
}

//the module parameter set interface for mrd_mode
//param: 1: download mode
//       2: mrd mode
static int mrd_mode_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = mrd_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	/* If download_mode is not one or two, ignore. */
	if ( (mrd_mode == 0) || (mrd_mode >2)) {
		mrd_mode = old_val;
		return -EINVAL;
	}

	mrd_set_mrdmode(mrd_mode);

	return 0;
}
module_param_call(mrd_download_mode, mrd_mode_set, param_get_int,
			&mrd_mode, 0644);

//print the shareindex info
static void mrd_dump_shareindex_info(void)
{
	mrd_shareindex_header_t *table;
	mrd_shareindex_item_t* cur_index = NULL;
	int i=0;

	table = mrd_shareindex_data.shareindex_table_ptr;

	//dump the header
	pr_info("%s: dump the shareindex table info:\n", __func__);
	pr_info("  shareindex table va=%p, pa=0x%08x\n",mrd_shareindex_data.shareindex_table_ptr, mrd_shareindex_data.shareindex_table_phys);
	pr_info("        signature  :0x%08x\n",table->signature);
	pr_info("        version    :0x%x\n",table->version);
	pr_info("        total_size :0x%08x\n",table->total_size);
	pr_info("        head_size  :0x%08x\n",table->head_size);
	pr_info("        total_count:0x%08x\n",table->total_count);
	pr_info("        index_count:0x%08x\n",table->index_count);
	pr_info("        lock       :0x%08x\n",table->lock);
	pr_info("        mrdmode    :0x%08x\n",table->mrdmode);
	pr_info("        boot_rtc   :0x%08x\n",table->boot_rtc);

	//dump the items
	for (i=0; i<table->index_count; i++) {
		cur_index = (mrd_shareindex_item_t* ) ((unsigned char *)mrd_shareindex_data.shareindex_table_ptr
				+ mrd_shareindex_data.shareindex_table_ptr->head_size
				+ sizeof(mrd_shareindex_item_t)*i);
		pr_info("  No. [%d] shareindex item:\n",i);
		pr_info("        indexva :0x%p\n",cur_index);
		pr_info("        name    :%s\n",cur_index->name);
		pr_info("        address :0x%08x\n",cur_index->address);
		pr_info("        subsyss :0x%08x\n",cur_index->subsys);
		pr_info("        size    :%d\n",cur_index->size);
		pr_info("        param1  :%d\n",cur_index->param1);
		pr_info("        param2  :%d\n",cur_index->param2);
	}
}

//add the debug interface in the proc entry
static int mrd_debug_read_proc(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
	char *tipstr="please see the dmesglog!\n";
	snprintf(buf, count, tipstr);
	mrd_dump_shareindex_info();
	*eof = 1;
	return strlen(tipstr);
}

//enable the mrd debug
#define MRD_DEBUG_FORCE_TRIGGER "force_trigger"
static int mrd_debug_write_proc(struct file *file, const char __user *buffer,
	unsigned long count, void *data)
{
	char buf[256];
	int ret;
	size_t len = strnlen(MRD_DEBUG_FORCE_TRIGGER , sizeof(MRD_DEBUG_FORCE_TRIGGER ));

	if (count < sizeof(MRD_DEBUG_FORCE_TRIGGER )) {
		pr_info("the command length is not valid\n");
		ret = -EINVAL;
		goto write_proc_failed;
	}

	if (copy_from_user(buf, buffer, len)) {
		ret = -EFAULT;
		goto write_proc_failed;
	}

	if (strncmp(buf, MRD_DEBUG_FORCE_TRIGGER, len)) {
		pr_info("the current command %s is not valid.\n",buf);
		ret = -EINVAL;
		goto write_proc_failed;
	}

	//set the sbl force trigger debug magic
	imem_write(FIX_IMEM_MRD_DUMP_SIGNATURE_OFFSET,MRD_FORCE_TRIGGER_MAGIC);
	return count;

write_proc_failed:
	return ret;
}

void mrd_register_rtc_xtime(unsigned int);
static int __init init_mrd_shareindex(void)
{
	mrd_shareindex_header_t *table;
	mrd_shareindex_item_t  index;
	struct proc_dir_entry *mrd_shareindex_entry;
	extern char __log_buf[];

	//alloc the mrd shareindex space from the smem
	mrd_shareindex_data.shareindex_table_ptr = mrd_alloc_share_index(MRD_SHAREINDEX_SIZE);
	if (!mrd_shareindex_data.shareindex_table_ptr)
	{
		pr_info("%s:alloc shareindex memory fail,size=%d!\n",__func__,MRD_SHAREINDEX_SIZE);
		return -1;
	}
	pr_info("%s:init the mrd shareindex table ptr=%p\n",__func__,mrd_shareindex_data.shareindex_table_ptr);
	table = mrd_shareindex_data.shareindex_table_ptr;
	memset(table,0,MRD_SHAREINDEX_SIZE);

	//init the shareindex header info
	table->signature = MRD_SHAREINDEX_HEAD_MAGIC;
	table->version = MRD_SHAREINDEX_VERSION;
	table->total_size = MRD_SHAREINDEX_SIZE;
	table->head_size = MRD_SHAREINDEX_HEAD_SIZE;
	table->total_count = (MRD_SHAREINDEX_SIZE - MRD_SHAREINDEX_HEAD_SIZE) / sizeof(mrd_shareindex_item_t);
	table->index_count = 0;
	table->lock = 0;
	table->mrdmode = mrd_mode;
	table->boot_rtc = boot_rtc_seconds;    //need to update

	//get the shareindex table physical from smem va to pa
	mrd_shareindex_data.shareindex_table_phys = get_physical_address((unsigned long)mrd_shareindex_data.shareindex_table_ptr);
	pr_info("%s:mrd dump table address=0x%08x\n",__func__,(u32)mrd_shareindex_data.shareindex_table_phys);

	//setup the imem flag
	imem_write(FIX_IMEM_MRD_DUMP_SIGNATURE_OFFSET,MRD_SMEM_TRIGGER_MAGIC);
	imem_write(FIX_IMEM_MRD_DUMP_TABLE_ADDR, mrd_shareindex_data.shareindex_table_phys);
	pr_info( "%s: mrd shareindex table set up\n",__func__);

	//register the linux_banner info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, "banner", MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = virt_to_phys(linux_banner);
	index.size = strlen(linux_banner);
	index.param1 = MRD_SHAREINDEX_SPECIAL_TAG;
	mrd_register_shareindex(&index);

	//register the command_line info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, "cmdline", MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = virt_to_phys(saved_command_line);
	index.size = strlen(saved_command_line);
	index.param1 = MRD_SHAREINDEX_SPECIAL_TAG;
	mrd_register_shareindex(&index);

	//register the dmesg log info
	memset(&index, 0, sizeof(index));
	strncpy(index.name, "dmesg", MRD_SHAREINDEX_ITEM_NAME_MAX_LENGTH);
	index.address = virt_to_phys(__log_buf);
	index.size = 1 << (CONFIG_LOG_BUF_SHIFT);
	mrd_register_shareindex(&index);

	mrd_shareindex_entry = create_proc_entry("mrd_debug", S_IRUGO | S_IWUSR | S_IWGRP, NULL);
	if (mrd_shareindex_entry) {
		mrd_shareindex_entry -> read_proc = &mrd_debug_read_proc;
		mrd_shareindex_entry -> write_proc = &mrd_debug_write_proc;
	}

	if (mrd_debug)
		mrd_dump_shareindex_info();
	mrd_init_status = 1;
	mrd_register_rtc_xtime(MRD_SHAREINDEX_SPECIAL_TAG);
	return 0;
}

early_initcall(init_mrd_shareindex);

