#include <linux/module.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include "mmc_proc.h"
#include "mmc_lock.h"

#define MAX_MMC_PROC_SIZE 64

const static char* sUnkonw = "Unknown\0";

static char mmc_proc_buff_name[MAX_MMC_PROC_SIZE]={0};
static char mmc_proc_buff_pw[MAX_MMC_PROC_SIZE]={0};

int mmc_proc_update(int mmc_locked, char* name){
	memset(mmc_proc_buff_name, 0, sizeof(mmc_proc_buff_name));
	if(mmc_locked){
		if(name == NULL)
			sprintf(mmc_proc_buff_name,"%s", sUnkonw);

		if((strlen(name)+2) > MAX_MMC_PROC_SIZE){
			printk(KERN_ERR"mmc proc module name oversize\n");
			return -1;
		}else{
			sprintf(mmc_proc_buff_name,"%s", name);
		}
	}else{
		sprintf(mmc_proc_buff_name,"%s", sUnkonw);
	}
	return 0;
}

static int mmc_proc_read_name(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len = strlen(mmc_proc_buff_name);

    if (off >= len)
	return 0;

    if (count > len - off)//限定字符数
	count = len - off;

    memcpy(page + off, mmc_proc_buff_name + off, count);//复制内存数据
    return off + count;//返回下一个字符指针
}

static int mmc_proc_write_name(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
    unsigned long count2 = count;

    if (count2 >= sizeof(mmc_proc_buff_name))//限定定入的字符数
	count2 = sizeof(mmc_proc_buff_name) - 1;

    memset(mmc_proc_buff_name, 0, sizeof(mmc_proc_buff_name));

    if (copy_from_user(mmc_proc_buff_name, buffer, count2))
	return -EFAULT;

    mmc_proc_buff_name[count2] = '\0';
    return count;
}

static int mmc_proc_read_pw(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len = strlen(mmc_proc_buff_pw);

    if (off >= len)
	return 0;

    if (count > len - off)//限定字符数
	count = len - off;

    memcpy(page + off, mmc_proc_buff_pw + off, count);//复制内存数据
    return off + count;//返回下一个字符指针
}

static int mmc_proc_write_pw(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
    unsigned long count2 = count;

    if (count2 >= sizeof(mmc_proc_buff_pw))//限定定入的字符数
	count2 = sizeof(mmc_proc_buff_pw) - 1;

    memset(mmc_proc_buff_pw, 0, sizeof(mmc_proc_buff_pw));

    if (copy_from_user(mmc_proc_buff_pw, buffer, count2))
	return -EFAULT;

    mmc_proc_buff_pw[count2++] = '\r';
    mmc_proc_buff_pw[count2++] = '\n';
    mmc_proc_buff_pw[count2++] = '\0';

    mmc_password_set(NULL, mmc_proc_buff_pw);
    return count;
}

int mmc_proc_init(void)
{
	struct proc_dir_entry *pfile_name;
	struct proc_dir_entry *pfile_pw;
	struct proc_dir_entry *proc_mmc_root;

	proc_mmc_root = proc_mkdir("mmc", 0);

	pfile_name = create_proc_entry("name", 0664, proc_mmc_root);//创建文件
	if (!pfile_name) {
		printk(KERN_ERR "Can't create /proc/mmc/name\n");
	}else{
		sprintf(mmc_proc_buff_name, "none\n");

		pfile_name->read_proc = mmc_proc_read_name;//读取文件时执行的程序
		pfile_name->write_proc = mmc_proc_write_name;//写入文件时执行的程序
	}


	pfile_pw = create_proc_entry("pw", 0200, proc_mmc_root);//创建文件
	if (!pfile_pw) {
		printk(KERN_ERR "Can't create /proc/mmc/pw\n");
	}else{
	//	pfile_pw->read_proc = mmc_proc_read_pw;//读取文件时执行的程序
		pfile_pw->write_proc = mmc_proc_write_pw;//写入文件时执行的程序
	}

	return 0;
}

void mmc_proc_exit(void)
{
    remove_proc_entry("/proc/mmc/name", NULL);
    remove_proc_entry("/proc/mmc/pw", NULL);
}

