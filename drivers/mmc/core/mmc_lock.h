#ifndef _MMC_LOCK_
#define _MMC_LOCK_

int mmc_password_set(struct mmc_card *card, char* password);
int mmc_lock_unlock(struct mmc_card *card, char lock_mode);

#endif
