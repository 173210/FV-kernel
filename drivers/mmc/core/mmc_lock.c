#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/scatterlist.h>
#include <linux/wait.h>

#include "core.h"
#include "bus.h"


#define MMC_LOCK_PASS_LEN 8

struct mmc_lock_data{
	char lock_mode;
	char password_len;
	char password[MMC_LOCK_PASS_LEN];
};

static struct mmc_lock_data LockInfo;
static struct mmc_host *locked_host = NULL;

static int mmc_lock_func(struct mmc_card *card, char* lock_data,int len)
{
    int err;
    struct mmc_request mrq;
    struct mmc_command cmd;
    struct mmc_data data;
    struct scatterlist sg;

    BUG_ON(!card);

    len += 2;

    memset(&cmd, 0, sizeof(struct mmc_command));
    cmd.opcode = MMC_SET_BLOCKLEN;
    cmd.arg = len;
    cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
    err = mmc_wait_for_cmd(card->host, &cmd, 5);
    if (err)
	return err;

    memset(&mrq, 0, sizeof(struct mmc_request));
    memset(&cmd, 0, sizeof(struct mmc_command));
    memset(&data, 0, sizeof(struct mmc_data));

    mrq.cmd = &cmd;
    mrq.data = &data;

    cmd.opcode = MMC_LOCK_UNLOCK;
    cmd.arg = 0;
    cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

    data.blksz = len;
    data.blocks = 1;
    data.flags = MMC_DATA_WRITE;
    data.sg = &sg;
    data.sg_len = 1;

    sg_init_one(&sg, lock_data, len);
    mmc_set_data_timeout(&data, card);

    mmc_wait_for_req(card->host, &mrq);

    do {
    	memset(&cmd, 0, sizeof(struct mmc_command));
	cmd.opcode = MMC_SEND_STATUS;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 5);
	if (err) {
	    goto error;
	}
    } while (!(cmd.resp[0] & R1_READY_FOR_DATA));

    if (cmd.resp[0] & R1_LOCK_UNLOCK_FAILED) {
	printk("LOCK_UNLOCK operation failed\n");
	err = -1;
	goto error;
    }else{
	if(lock_data[0] == 0)
	    printk("mmc unlock successful\n");
	if(lock_data[0]&0x01)
	    printk("mmc set password successful\n");
	if(lock_data[0]&0x02)
	    printk("mmc clr password successful\n");
	if(lock_data[0]&0x04)
	    printk("mmc lock successful\n");
	if(lock_data[0]&0x08)
	    printk("mmc erase successful\n");
    }

    if (cmd.resp[0] & R1_CARD_IS_LOCKED){
    }

error:
    memset(&cmd, 0, sizeof(struct mmc_command));
    cmd.opcode = MMC_SET_BLOCKLEN;
    cmd.arg = 512;
    cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
    mmc_wait_for_cmd(card->host, &cmd, 5);
    return err;
}

int mmc_lock_unlock(struct mmc_card *card, char lock_mode)
{
	locked_host = card->host;
	LockInfo.lock_mode = lock_mode;
	return mmc_lock_func(card, (char *)&LockInfo, MMC_LOCK_PASS_LEN);
}

int mmc_password_set(struct mmc_card *card, char* password)
{
	int i;

	LockInfo.password_len = MMC_LOCK_PASS_LEN;
	for(i=0; i<MMC_LOCK_PASS_LEN; i++){
		LockInfo.password[i] = password[i];
	}
	if(locked_host != NULL)
		mmc_detect_change(locked_host, 1);
	return 0;
}
