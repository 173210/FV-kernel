#ifndef _MMC_PROC_
#define _MMC_PROC_


int mmc_proc_init(void);
void mmc_proc_exit(void);
int mmc_proc_update(int mmc_locked, char* name);

#endif
