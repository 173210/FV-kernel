/*
 *  fs/partitions/check.c
 *
 *  Code extracted from drivers/block/genhd.c
 *  Copyright (C) 1991-1998  Linus Torvalds
 *  Re-organised Feb 1998 Russell King
 *
 *  We now have independent partition support from the
 *  block drivers, which allows all the partition code to
 *  be grouped in one location, and it to be mostly self
 *  contained.
 *
 *  Added needed MAJORS for new pairs, {hdi,hdj}, {hdk,hdl}
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kmod.h>
#include <linux/ctype.h>
#include <linux/mount.h> /* for name_to_dev_t() */

#include "check.h"

#include "acorn.h"
#include "amiga.h"
#include "atari.h"
#include "ldm.h"
#include "mac.h"
#include "msdos.h"
#include "osf.h"
#include "sgi.h"
#include "sun.h"
#include "ibm.h"
#include "ultrix.h"
#include "efi.h"
#include "karma.h"

#ifdef CONFIG_BLK_DEV_MD
extern void md_autodetect_dev(dev_t dev);
#endif
#ifdef CONFIG_ARBITRARY_PARTITION
static int arb_partition_check(struct parsed_partitions *state,
			       struct block_device *bdev);
#endif

int warn_no_part = 1; /*This is ugly: should make genhd removable media aware*/

static int (*check_part[])(struct parsed_partitions *, struct block_device *) = {
#ifdef CONFIG_ARBITRARY_PARTITION
	arb_partition_check,
#endif
	/*
	 * Probe partition formats with tables at disk address 0
	 * that also have an ADFS boot block at 0xdc0.
	 */
#ifdef CONFIG_ACORN_PARTITION_ICS
	adfspart_check_ICS,
#endif
#ifdef CONFIG_ACORN_PARTITION_POWERTEC
	adfspart_check_POWERTEC,
#endif
#ifdef CONFIG_ACORN_PARTITION_EESOX
	adfspart_check_EESOX,
#endif

	/*
	 * Now move on to formats that only have partition info at
	 * disk address 0xdc0.  Since these may also have stale
	 * PC/BIOS partition tables, they need to come before
	 * the msdos entry.
	 */
#ifdef CONFIG_ACORN_PARTITION_CUMANA
	adfspart_check_CUMANA,
#endif
#ifdef CONFIG_ACORN_PARTITION_ADFS
	adfspart_check_ADFS,
#endif

#ifdef CONFIG_EFI_PARTITION
	efi_partition,		/* this must come before msdos */
#endif
#ifdef CONFIG_SGI_PARTITION
	sgi_partition,
#endif
#ifdef CONFIG_LDM_PARTITION
	ldm_partition,		/* this must come before msdos */
#endif
#ifdef CONFIG_MSDOS_PARTITION
	msdos_partition,
#endif
#ifdef CONFIG_OSF_PARTITION
	osf_partition,
#endif
#ifdef CONFIG_SUN_PARTITION
	sun_partition,
#endif
#ifdef CONFIG_AMIGA_PARTITION
	amiga_partition,
#endif
#ifdef CONFIG_ATARI_PARTITION
	atari_partition,
#endif
#ifdef CONFIG_MAC_PARTITION
	mac_partition,
#endif
#ifdef CONFIG_ULTRIX_PARTITION
	ultrix_partition,
#endif
#ifdef CONFIG_IBM_PARTITION
	ibm_partition,
#endif
#ifdef CONFIG_KARMA_PARTITION
	karma_partition,
#endif
	NULL
};
 
/*
 * disk_name() is used by partition check code and the genhd driver.
 * It formats the devicename of the indicated disk into
 * the supplied buffer (of size at least 32), and returns
 * a pointer to that same buffer (for convenience).
 */

char *disk_name(struct gendisk *hd, int part, char *buf)
{
	if (!part)
		snprintf(buf, BDEVNAME_SIZE, "%s", hd->disk_name);
	else if (isdigit(hd->disk_name[strlen(hd->disk_name)-1]))
		snprintf(buf, BDEVNAME_SIZE, "%sp%d", hd->disk_name, part);
	else
		snprintf(buf, BDEVNAME_SIZE, "%s%d", hd->disk_name, part);

	return buf;
}

const char *bdevname(struct block_device *bdev, char *buf)
{
	int part = MINOR(bdev->bd_dev) - bdev->bd_disk->first_minor;
	return disk_name(bdev->bd_disk, part, buf);
}

EXPORT_SYMBOL(bdevname);

/*
 * There's very little reason to use this, you should really
 * have a struct block_device just about everywhere and use
 * bdevname() instead.
 */
const char *__bdevname(dev_t dev, char *buffer)
{
	scnprintf(buffer, BDEVNAME_SIZE, "unknown-block(%u,%u)",
				MAJOR(dev), MINOR(dev));
	return buffer;
}

EXPORT_SYMBOL(__bdevname);

static struct parsed_partitions *
check_partition(struct gendisk *hd, struct block_device *bdev)
{
	struct parsed_partitions *state;
	int i, res, err;

	state = kmalloc(sizeof(struct parsed_partitions), GFP_KERNEL);
	if (!state)
		return NULL;

	disk_name(hd, 0, state->name);
	printk(KERN_INFO " %s:", state->name);
	if (isdigit(state->name[strlen(state->name)-1]))
		sprintf(state->name, "p");

	state->limit = hd->minors;
	i = res = err = 0;
	while (!res && check_part[i]) {
		memset(&state->parts, 0, sizeof(state->parts));
		res = check_part[i++](state, bdev);
		if (res < 0) {
			/* We have hit an I/O error which we don't report now.
		 	* But record it, and let the others do their job.
		 	*/
			err = res;
			res = 0;
		}

	}
	if (res > 0)
		return state;
	if (!err)
	/* The partition is unrecognized. So report I/O errors if there were any */
		res = err;
	if (!res)
		printk(" unknown partition table\n");
	else if (warn_no_part)
		printk(" unable to read partition table\n");
	kfree(state);
	return ERR_PTR(res);
}

/*
 * sysfs bindings for partitions
 */

struct part_attribute {
	struct attribute attr;
	ssize_t (*show)(struct hd_struct *,char *);
	ssize_t (*store)(struct hd_struct *,const char *, size_t);
};

static ssize_t 
part_attr_show(struct kobject * kobj, struct attribute * attr, char * page)
{
	struct hd_struct * p = container_of(kobj,struct hd_struct,kobj);
	struct part_attribute * part_attr = container_of(attr,struct part_attribute,attr);
	ssize_t ret = 0;
	if (part_attr->show)
		ret = part_attr->show(p, page);
	return ret;
}
static ssize_t
part_attr_store(struct kobject * kobj, struct attribute * attr,
		const char *page, size_t count)
{
	struct hd_struct * p = container_of(kobj,struct hd_struct,kobj);
	struct part_attribute * part_attr = container_of(attr,struct part_attribute,attr);
	ssize_t ret = 0;

	if (part_attr->store)
		ret = part_attr->store(p, page, count);
	return ret;
}

static struct sysfs_ops part_sysfs_ops = {
	.show	=	part_attr_show,
	.store	=	part_attr_store,
};

static ssize_t part_uevent_store(struct hd_struct * p,
				 const char *page, size_t count)
{
	kobject_uevent(&p->kobj, KOBJ_ADD);
	return count;
}
static ssize_t part_dev_read(struct hd_struct * p, char *page)
{
	struct gendisk *disk = container_of(p->kobj.parent,struct gendisk,kobj);
	dev_t dev = MKDEV(disk->major, disk->first_minor + p->partno); 
	return print_dev_t(page, dev);
}
static ssize_t part_start_read(struct hd_struct * p, char *page)
{
	return sprintf(page, "%llu\n",(unsigned long long)p->start_sect);
}
static ssize_t part_size_read(struct hd_struct * p, char *page)
{
	return sprintf(page, "%llu\n",(unsigned long long)p->nr_sects);
}
static ssize_t part_stat_read(struct hd_struct * p, char *page)
{
	return sprintf(page, "%8u %8llu %8u %8llu\n",
		       p->ios[0], (unsigned long long)p->sectors[0],
		       p->ios[1], (unsigned long long)p->sectors[1]);
}
static struct part_attribute part_attr_uevent = {
	.attr = {.name = "uevent", .mode = S_IWUSR },
	.store	= part_uevent_store
};
static struct part_attribute part_attr_dev = {
	.attr = {.name = "dev", .mode = S_IRUGO },
	.show	= part_dev_read
};
static struct part_attribute part_attr_start = {
	.attr = {.name = "start", .mode = S_IRUGO },
	.show	= part_start_read
};
static struct part_attribute part_attr_size = {
	.attr = {.name = "size", .mode = S_IRUGO },
	.show	= part_size_read
};
static struct part_attribute part_attr_stat = {
	.attr = {.name = "stat", .mode = S_IRUGO },
	.show	= part_stat_read
};

#ifdef CONFIG_FAIL_MAKE_REQUEST

static ssize_t part_fail_store(struct hd_struct * p,
			       const char *buf, size_t count)
{
	int i;

	if (count > 0 && sscanf(buf, "%d", &i) > 0)
		p->make_it_fail = (i == 0) ? 0 : 1;

	return count;
}
static ssize_t part_fail_read(struct hd_struct * p, char *page)
{
	return sprintf(page, "%d\n", p->make_it_fail);
}
static struct part_attribute part_attr_fail = {
	.attr = {.name = "make-it-fail", .mode = S_IRUGO | S_IWUSR },
	.store	= part_fail_store,
	.show	= part_fail_read
};

#endif

static struct attribute * default_attrs[] = {
	&part_attr_uevent.attr,
	&part_attr_dev.attr,
	&part_attr_start.attr,
	&part_attr_size.attr,
	&part_attr_stat.attr,
#ifdef CONFIG_FAIL_MAKE_REQUEST
	&part_attr_fail.attr,
#endif
	NULL,
};

extern struct subsystem block_subsys;

static void part_release(struct kobject *kobj)
{
	struct hd_struct * p = container_of(kobj,struct hd_struct,kobj);
	kfree(p);
}

struct kobj_type ktype_part = {
	.release	= part_release,
	.default_attrs	= default_attrs,
	.sysfs_ops	= &part_sysfs_ops,
};

static inline void partition_sysfs_add_subdir(struct hd_struct *p)
{
	struct kobject *k;

	k = kobject_get(&p->kobj);
	p->holder_dir = kobject_add_dir(k, "holders");
	kobject_put(k);
}

static inline void disk_sysfs_add_subdirs(struct gendisk *disk)
{
	struct kobject *k;

	k = kobject_get(&disk->kobj);
	disk->holder_dir = kobject_add_dir(k, "holders");
	disk->slave_dir = kobject_add_dir(k, "slaves");
	kobject_put(k);
}

void delete_partition(struct gendisk *disk, int part)
{
	struct hd_struct *p = disk->part[part-1];
	if (!p)
		return;
	if (!p->nr_sects)
		return;
	disk->part[part-1] = NULL;
	p->start_sect = 0;
	p->nr_sects = 0;
	p->ios[0] = p->ios[1] = 0;
	p->sectors[0] = p->sectors[1] = 0;
	sysfs_remove_link(&p->kobj, "subsystem");
	if (p->holder_dir)
		kobject_unregister(p->holder_dir);
	kobject_uevent(&p->kobj, KOBJ_REMOVE);
	kobject_del(&p->kobj);
	kobject_put(&p->kobj);
}

void add_partition(struct gendisk *disk, int part, sector_t start, sector_t len)
{
	struct hd_struct *p;

	p = kmalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return;
	
	memset(p, 0, sizeof(*p));
	p->start_sect = start;
	p->nr_sects = len;
	p->partno = part;
	p->policy = disk->policy;

	if (isdigit(disk->kobj.name[strlen(disk->kobj.name)-1]))
		snprintf(p->kobj.name,KOBJ_NAME_LEN,"%sp%d",disk->kobj.name,part);
	else
		snprintf(p->kobj.name,KOBJ_NAME_LEN,"%s%d",disk->kobj.name,part);
	p->kobj.parent = &disk->kobj;
	p->kobj.ktype = &ktype_part;
	kobject_init(&p->kobj);
	kobject_add(&p->kobj);
	if (!disk->part_uevent_suppress)
		kobject_uevent(&p->kobj, KOBJ_ADD);
	sysfs_create_link(&p->kobj, &block_subsys.kset.kobj, "subsystem");
	partition_sysfs_add_subdir(p);
	disk->part[part-1] = p;
}

static char *make_block_name(struct gendisk *disk)
{
	char *name;
	static char *block_str = "block:";
	int size;
	char *s;

	size = strlen(block_str) + strlen(disk->disk_name) + 1;
	name = kmalloc(size, GFP_KERNEL);
	if (!name)
		return NULL;
	strcpy(name, block_str);
	strcat(name, disk->disk_name);
	/* ewww... some of these buggers have / in name... */
	s = strchr(name, '/');
	if (s)
		*s = '!';
	return name;
}

static int disk_sysfs_symlinks(struct gendisk *disk)
{
	struct device *target = get_device(disk->driverfs_dev);
	int err;
	char *disk_name = NULL;

	if (target) {
		disk_name = make_block_name(disk);
		if (!disk_name) {
			err = -ENOMEM;
			goto err_out;
		}

		err = sysfs_create_link(&disk->kobj, &target->kobj, "device");
		if (err)
			goto err_out_disk_name;

		err = sysfs_create_link(&target->kobj, &disk->kobj, disk_name);
		if (err)
			goto err_out_dev_link;
	}

	err = sysfs_create_link(&disk->kobj, &block_subsys.kset.kobj,
				"subsystem");
	if (err)
		goto err_out_disk_name_lnk;

	kfree(disk_name);

	return 0;

err_out_disk_name_lnk:
	if (target) {
		sysfs_remove_link(&target->kobj, disk_name);
err_out_dev_link:
		sysfs_remove_link(&disk->kobj, "device");
err_out_disk_name:
		kfree(disk_name);
err_out:
		put_device(target);
	}
	return err;
}

/* Not exported, helper to add_disk(). */
void register_disk(struct gendisk *disk)
{
	struct block_device *bdev;
	char *s;
	int i;
	struct hd_struct *p;
	int err;

	strlcpy(disk->kobj.name,disk->disk_name,KOBJ_NAME_LEN);
	/* ewww... some of these buggers have / in name... */
	s = strchr(disk->kobj.name, '/');
	if (s)
		*s = '!';
	if ((err = kobject_add(&disk->kobj)))
		return;
	err = disk_sysfs_symlinks(disk);
	if (err) {
		kobject_del(&disk->kobj);
		return;
	}
 	disk_sysfs_add_subdirs(disk);

	/* No minors to use for partitions */
	if (disk->minors == 1)
		goto exit;

	/* No such device (e.g., media were just removed) */
	if (!get_capacity(disk))
		goto exit;

	bdev = bdget_disk(disk, 0);
	if (!bdev)
		goto exit;

	/* scan partition table, but suppress uevents */
	bdev->bd_invalidated = 1;
	disk->part_uevent_suppress = 1;
	err = blkdev_get(bdev, FMODE_READ, 0);
	disk->part_uevent_suppress = 0;
	if (err < 0)
		goto exit;
	blkdev_put(bdev);

exit:
	/* announce disk after possible partitions are already created */
	kobject_uevent(&disk->kobj, KOBJ_ADD);

	/* announce possible partitions */
	for (i = 1; i < disk->minors; i++) {
		p = disk->part[i-1];
		if (!p || !p->nr_sects)
			continue;
		kobject_uevent(&p->kobj, KOBJ_ADD);
	}
}

int rescan_partitions(struct gendisk *disk, struct block_device *bdev)
{
	struct parsed_partitions *state;
	int p, res;

	if (bdev->bd_part_count)
		return -EBUSY;
	res = invalidate_partition(disk, 0);
	if (res)
		return res;
	bdev->bd_invalidated = 0;
	for (p = 1; p < disk->minors; p++)
		delete_partition(disk, p);
	if (disk->fops->revalidate_disk)
		disk->fops->revalidate_disk(disk);
	if (!get_capacity(disk) || !(state = check_partition(disk, bdev)))
		return 0;
	if (IS_ERR(state))	/* I/O error reading the partition table */
		return PTR_ERR(state);
	for (p = 1; p < state->limit; p++) {
		sector_t size = state->parts[p].size;
		sector_t from = state->parts[p].from;
		if (!size)
			continue;
		if (from + size > get_capacity(disk)) {
			printk(" %s: p%d exceeds device capacity\n",
				disk->disk_name, p);
		}
		add_partition(disk, p, from, size);
#ifdef CONFIG_BLK_DEV_MD
		if (state->parts[p].flags)
			md_autodetect_dev(bdev->bd_dev+p);
#endif
	}
	kfree(state);
	return 0;
}

unsigned char *read_dev_sector(struct block_device *bdev, sector_t n, Sector *p)
{
	struct address_space *mapping = bdev->bd_inode->i_mapping;
	struct page *page;

	page = read_mapping_page(mapping, (pgoff_t)(n >> (PAGE_CACHE_SHIFT-9)),
				 NULL);
	if (!IS_ERR(page)) {
		wait_on_page_locked(page);
		if (!PageUptodate(page))
			goto fail;
		if (PageError(page))
			goto fail;
		p->v = page;
		return (unsigned char *)page_address(page) +  ((n & ((1 << (PAGE_CACHE_SHIFT - 9)) - 1)) << 9);
fail:
		page_cache_release(page);
	}
	p->v = NULL;
	return NULL;
}

EXPORT_SYMBOL(read_dev_sector);

void del_gendisk(struct gendisk *disk)
{
	int p;

	/* invalidate stuff */
	for (p = disk->minors - 1; p > 0; p--) {
		invalidate_partition(disk, p);
		delete_partition(disk, p);
	}
	invalidate_partition(disk, 0);
	disk->capacity = 0;
	disk->flags &= ~GENHD_FL_UP;
	unlink_gendisk(disk);
	disk_stat_set_all(disk, 0);
	disk->stamp = 0;

	kobject_uevent(&disk->kobj, KOBJ_REMOVE);
	if (disk->holder_dir)
		kobject_unregister(disk->holder_dir);
	if (disk->slave_dir)
		kobject_unregister(disk->slave_dir);
	if (disk->driverfs_dev) {
		char *disk_name = make_block_name(disk);
		sysfs_remove_link(&disk->kobj, "device");
		if (disk_name) {
			sysfs_remove_link(&disk->driverfs_dev->kobj, disk_name);
			kfree(disk_name);
		}
		put_device(disk->driverfs_dev);
		disk->driverfs_dev = NULL;
	}
	sysfs_remove_link(&disk->kobj, "subsystem");
	kobject_del(&disk->kobj);
}

#ifdef CONFIG_ARBITRARY_PARTITION
/*
 * based on cmdlinepart.c
 *
 * The format for the command line is as follows:
 *
 * arbparts=<arbdef>[;<arbdef]
 * <arbdef>  := <arb-id>:<partdef>[,<partdef>]
 * <partdef> := <size>[@offset]
 * <arb-id> := master block device name
 * <size>    := standard linux memsize OR "-" to denote all remaining space
 */

/* error message prefix */
#define ERRP "arb: "

/* special size referring to all the remaining space in a partition */
#define SIZE_REMAINING UINT_MAX
#define OFFSET_CONTINUOUS UINT_MAX

struct arb_partition {
	u_int32_t size;			/* partition size */
	u_int32_t offset;		/* offset within the master device */
};

struct cmdline_arb_partition {
	struct cmdline_arb_partition *next;
	char *arb_id;
	int num_parts;
	struct arb_partition *parts;
};

/* arbpart_setup() parses into here */
static struct cmdline_arb_partition *partitions;

/* the command line passed to arbpart_setup() */
static char *cmdline;
static int cmdline_parsed;

/*
 * Parse one partition definition for an block device. Since there can
 * be many comma separated partition definitions, this function calls
 * itself recursively until no more partition definitions are
 * found. Nice side effect: the memory to keep the arb_partition
 * structs is allocated upon the last definition being found. At that
 * point the syntax has been verified ok.
 */
static struct arb_partition *newpart(char *s,
				     char **retptr,
				     int *num_parts,
				     int this_part,
				     unsigned char **extra_mem_ptr,
				     int extra_mem_size)
{
	struct arb_partition *parts;
	unsigned long size;
	unsigned long offset = OFFSET_CONTINUOUS;
	unsigned char *extra_mem;

	/* fetch the partition size */
	if (*s == '-') {
		/* assign all remaining space to this partition */
		size = SIZE_REMAINING;
		s++;
	} else {
		size = memparse(s, &s);
		if (size < PAGE_SIZE) {
			printk(KERN_ERR ERRP "partition size too small (%lx)\n", size);
			return NULL;
		}
	}

	/* check for offset */
	if (*s == '@') {
		s++;
		offset = memparse(s, &s);
	}

	/* test if more partitions are following */
	if (*s == ',') {
		if (size == SIZE_REMAINING) {
			printk(KERN_ERR ERRP "no partitions allowed after a fill-up partition\n");
			return NULL;
		}
		/* more partitions follow, parse them */
		parts = newpart(s + 1, &s, num_parts,
				this_part + 1, &extra_mem, extra_mem_size);
		if (!parts)
			return NULL;
	} else {
		/* this is the last partition: allocate space for all */
		int alloc_size;

		*num_parts = this_part + 1;
		alloc_size = *num_parts * sizeof(struct arb_partition) +
			extra_mem_size;
		parts = kzalloc(alloc_size, GFP_KERNEL);
		if (!parts) {
			printk(KERN_ERR ERRP "out of memory\n");
			return NULL;
		}
		extra_mem = (unsigned char *)(parts + *num_parts);
	}
	/* enter this partition (offset will be calculated later if it is zero at this point) */
	parts[this_part].size = size;
	parts[this_part].offset = offset;

	/* return (updated) pointer to extra_mem memory */
	if (extra_mem_ptr)
		*extra_mem_ptr = extra_mem;

	/* return (updated) pointer command line string */
	*retptr = s;

	/* return partition table */
	return parts;
}

/*
 * Parse the command line.
 */
static int arbpart_setup_real(char *s)
{
	cmdline_parsed = 1;

	for ( ; s != NULL; ) {
		struct cmdline_arb_partition *this_arb;
		struct arb_partition *parts;
		int arb_id_len;
		int num_parts;
		char *p, *arb_id;

		arb_id = s;
		/* fetch <arb-id> */
		p = strchr(s, ':');
		if (!p) {
			printk(KERN_ERR ERRP "no arb-id\n");
			return 0;
		}
		arb_id_len = p - arb_id;

		/*
		 * parse one device. have it reserve memory for the
		 * struct cmdline_arb_partition.
		 */
		parts = newpart(p + 1,		/* cmdline */
				&s,		/* out: updated cmdline ptr */
				&num_parts,	/* out: number of parts */
				0,		/* first partition */
				(unsigned char **)&this_arb, /* out: extra mem */
				arb_id_len + 1 + sizeof(*this_arb));
		if (!parts) {
			/*
			 * An error occurred. We're either:
			 * a) out of memory, or
			 * b) in the middle of the partition spec
			 * Either way, this device is hosed and we're
			 * unlikely to succeed in parsing any more
			 */
			 return 0;
		 }

		/* enter results */
		this_arb->parts = parts;
		this_arb->num_parts = num_parts;
		this_arb->arb_id = (char *)(this_arb + 1);
		strlcpy(this_arb->arb_id, arb_id, arb_id_len + 1);

		/* link into chain */
		this_arb->next = partitions;
		partitions = this_arb;

		/* EOS - we're done */
		if (*s == 0)
			break;

		/* does another spec follow? */
		if (*s != ';') {
			printk(KERN_ERR ERRP "bad character after partition (%c)\n", *s);
			return 0;
		}
		s++;
	}
	return 1;
}

/*
 * Main function to be called from check_partition() to
 * obtain the partitioning information. At this point the command line
 * arguments will actually be parsed and turned to struct arb_partition
 * information.
 */
static int arb_partition_check(struct parsed_partitions *state,
			       struct block_device *bdev)
{
	unsigned long offset;
	int i;
	int slot;
	struct cmdline_arb_partition *part;
	char name[BDEVNAME_SIZE];

	if (!cmdline)
		return 0;

	/* parse command line */
	if (!cmdline_parsed)
		arbpart_setup_real(cmdline);

	bdevname(bdev, name);
	for (part = partitions; part; part = part->next) {
		if (strcmp(name, part->arb_id))
			continue;
		for (i = 0, offset = 0; i < part->num_parts; i++) {
			unsigned long capa = get_capacity(bdev->bd_disk) * 512;
			if (part->parts[i].offset == OFFSET_CONTINUOUS)
				part->parts[i].offset = offset;
			else
				offset = part->parts[i].offset;
			if (part->parts[i].size == SIZE_REMAINING)
				part->parts[i].size = capa - offset;
			if (offset + part->parts[i].size > capa) {
				printk(KERN_WARNING ERRP
				       "%s: partitioning exceeds flash size, truncating\n",
				       part->arb_id);
				part->parts[i].size = capa - offset;
				part->num_parts = i;
			}
			offset += part->parts[i].size;
		}
		slot = 1;
		for (i = 0, offset = 0; i < part->num_parts; i++) {
			if (slot == state->limit)
				break;
			put_partition(state, slot,
				      part->parts[i].offset / 512,
				      part->parts[i].size / 512);
			slot++;
		}
		printk("\n");
		return 1;
	}
	return 0;
}

static int arbpart_setup(char *s)
{
	cmdline = s;
	return 1;
}

__setup("arbparts=", arbpart_setup);
#endif /* CONFIG_ARBITRARY_PARTITION */
