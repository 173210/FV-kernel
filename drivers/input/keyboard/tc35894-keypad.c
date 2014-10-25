/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * Author: Jayeeta Banerjee <jayeeta.banerjee@stericsson.com>
 * Author: Sundar Iyer <sundar.iyer@stericsson.com>
 *
 * License Terms: GNU General Public License, version 2
 *
 * TC35893 MFD Keypad Controller driver
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/input/matrix_keypad.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/mfd/tc35894.h>

#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/ctype.h>
static int kb_addon=0;

/* Maximum supported keypad matrix row/columns size */
#define TC35894_MAX_KPROW               8
#define TC35894_MAX_KPCOL               12

/* keypad related Constants */
#define TC35894_MAX_DEBOUNCE_SETTLE     0xFF
#define DEDICATED_KEY_VAL		0xFF

/* Pull up/down masks */
#define TC35894_NO_PULL_MASK		0x0
#define TC35894_PULL_DOWN_MASK		0x1
#define TC35894_PULL_UP_MASK		0x2
#define TC35894_PULLUP_ALL_MASK		0xAA
#define TC35894_IO_PULL_VAL(index, mask)	((mask)<<(((index)%4)*2))

/* Bit masks for IOCFG register */
#define IOCFG_BALLCFG		0x01
#define IOCFG_IG		0x08

#define KP_EVCODE_COL_MASK	0x0F
#define KP_EVCODE_ROW_MASK	0x70
#define KP_RELEASE_EVT_MASK	0x80

#define KP_ROW_SHIFT		4

#define KP_NO_VALID_KEY_MASK	0x7F

/* bit masks for RESTCTRL register */
#define TC35894_KBDRST		0x2
#define TC35894_IRQRST		0x10
#define TC35894_RESET_ALL	0x1B
#define TC35894_GPIRST		0x01
/* KBDMFS register bit mask */
#define TC35894_KBDMFS_EN	0x1

/* CLKEN register bitmask */
#define KPD_CLK_EN		0x1

/* RSTINTCLR register bit mask */
#define IRQ_CLEAR		0x1

/* AUTOSLPENA register bit mask */
#define	AUTOSLP_ENA		0x1

/* bit masks for keyboard interrupts*/
#define TC35894_EVT_LOSS_INT	0x8
#define TC35894_EVT_INT		0x4
#define TC35894_KBD_LOSS_INT	0x2
#define TC35894_KBD_INT		0x1

/* bit masks for keyboard interrupt clear*/
#define TC35894_EVT_INT_CLR	0x2
#define TC35894_KBD_INT_CLR	0x1

#define TC35894_KBD_KEYMAP_SIZE     64
/**these Marco for running program logic only**/
#define NORMAL_LOCK 0
#define NUMB_LOCK 0x02
#define CAPS_LOCK 0x01
#define CAPSCODE 51
#define NUMCODE 32
#define SHIFTCODE 6
#define CNTLCODE 14
#define NONEKEY 0xff
/********************The end*******************/

/**
 * struct tc_keypad - data structure used by keypad driver
 * @input:      pointer to input device object
 * @board:      keypad platform device
 * @krow:	number of rows
 * @kcol:	number of coloumns
 * @keymap:     matrix scan code table for keycodes
 */
struct tc_keypad {
	struct tc35894 *tc35894;
	struct delayed_work work;
	int irq_base;
	struct input_dev *input;
	const struct tc35894_keypad_platform_data *board;
	unsigned int krow;
	unsigned int kcol;
	unsigned short keymap[TC35894_KBD_KEYMAP_SIZE];
	bool keypad_stopped;
	u8 keyboard_mode;
	spinlock_t lock;
};

#define work_to_tc35894(w)	container_of(w, struct tc_keypad, work.work)
struct TC_last_Pkey {
	u8 code;
	u8 row_index;
	u8 col_index;
	u8 up;
};
u8 keypadstate[TC_KPD_ROWS];
//u32 current_keymap[TC_KPD_ROWS*TC_KPD_COLUMNS];
const u32 *current_keymap;

static int check_home_mulkey(struct input_dev *dev, int keycode, int status)
{
#define KEY_PRESSED	1
	if(keycode >=KEY_YI_KUO && keycode <=KEY_JPNONE && status==KEY_PRESSED)	//功能键被按下
	{
        	input_report_key(dev, KEY_HOME, !!KEY_PRESSED);
        	input_sync(dev);
		msleep(1);
        	input_report_key(dev, KEY_HOME, !KEY_PRESSED);
        	input_sync(dev);
	}

	return 1;
}
static int __devinit tc35894_keypad_init_key_hardware(struct tc_keypad *keypad)
{
	int ret;
	struct tc35894 *tc35894 = keypad->tc35894;
//	u8 settle_time = keypad->board->settle_time;
//	u8 dbounce_period = keypad->board->debounce_period;
	u8 rows = keypad->board->krow & 0xf;	/* mask out the nibble */
	u8 column = keypad->board->kcol & 0xf;	/* mask out the nibble */

	/* validate platform configurations */
	if (keypad->board->kcol > TC35894_MAX_KPCOL ||
	    keypad->board->krow > TC35894_MAX_KPROW ||
	    keypad->board->debounce_period > TC35894_MAX_DEBOUNCE_SETTLE ||
	    keypad->board->settle_time > TC35894_MAX_DEBOUNCE_SETTLE)
		return -EINVAL;
	/* Start of initialise keypad GPIOs */
//	ret = tc35894_set_bits(tc35894, TC35894_IOCFG, 0x0, IOCFG_IG);
	ret = tc35894_reg_write(tc35894, TC35894_IOCFG, 0xF8);
	if (ret < 0)
		return ret;
	/* configure TC35894_CLKCFG for setting sysclk to 64KHZ and using internal clock */
	ret = tc35894_reg_write(tc35894, TC35894_CLKCFG ,0x44);
	if (ret < 0)
		return ret;

	/* configure KBDSIZE 4 LSbits for cols and 4 MSbits for rows */
	ret = tc35894_reg_write(tc35894, TC35894_KBDSIZE,
			(rows << KP_ROW_SHIFT) | column);
	if (ret < 0)
		return ret;
	/* configure dedicated key config, no dedicated key selected */
	ret = tc35894_reg_write(tc35894, TC35894_KBCFG_LSB, DEDICATED_KEY_VAL);
	if (ret < 0)
		return ret;

	ret = tc35894_reg_write(tc35894, TC35894_KBCFG_MSB, DEDICATED_KEY_VAL);
	if (ret < 0)
		return ret;

	/* Configure settle time */
//	ret = tc35894_reg_write(tc35894, TC35894_KBDSETTLE_REG, settle_time);
	ret = tc35894_reg_write(tc35894, TC35894_KBDSETTLE_REG, 0xff);
	if (ret < 0)
		return ret;
	/* Configure debounce time */
	//ret = tc35894_reg_write(tc35894, TC35894_KBDBOUNCE, dbounce_period);
	ret = tc35894_reg_write(tc35894, TC35894_KBDBOUNCE, 0xff);
	if (ret < 0)
		return ret;


	/* Configure pull-up resistors for all row GPIOs */
	ret = tc35894_reg_write(tc35894, TC35894_IOPULLCFG0_LSB,
					TC35894_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

	ret = tc35894_reg_write(tc35894, TC35894_IOPULLCFG0_MSB,
					TC35894_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

	/* Configure pull-up resistors for all column GPIOs */
	ret = tc35894_reg_write(tc35894, TC35894_IOPULLCFG1_LSB,
			TC35894_NO_PULL_MASK);// TC35894_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

	ret = tc35894_reg_write(tc35894, TC35894_IOPULLCFG1_MSB,
			TC35894_NO_PULL_MASK);//TC35894_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

	ret = tc35894_reg_write(tc35894, TC35894_IOPULLCFG2_LSB,
			TC35894_NO_PULL_MASK);//TC35894_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

	ret = tc35894_reg_write(tc35894, TC35894_IOPULLCFG2_MSB,
			TC35894_NO_PULL_MASK);//TC35894_PULLUP_ALL_MASK);
	if (ret < 0)
		return ret;

        /* disable GPIO interrupts */
        ret = tc35894_reg_write(tc35894,TC35894_GPIOIE0,0x00);
        if (ret < 0)
                return ret;
        /* enable the keypad autosleep */
        ret = tc35894_reg_write(tc35894,TC35894_GPIOIE1,0x00);
        if (ret < 0)
                return ret;
       /* enable the keypad autosleep */
        ret = tc35894_reg_write(tc35894,TC35894_GPIOIE2,0x00);
        if (ret < 0)
                return ret;

        /* enable the keypad autosleep */
        ret = tc35894_reg_write(tc35894,TC35894_GPIODIR0,0x00);
        if (ret < 0)
                return ret;
        /* enable the keypad autosleep */
        ret = tc35894_reg_write(tc35894,TC35894_GPIODIR1,0xFF);
        if (ret < 0)
                return ret;
        /* enable the keypad autosleep */
        ret = tc35894_reg_write(tc35894,TC35894_GPIODIR2,0xFF);
        if (ret < 0)
                return ret;


	/* enable the keypad autosleep */
	ret = tc35894_reg_write(tc35894,TC35894_GPIOWAKE0,0xFF);
	if (ret < 0)
		return ret;
	/* enable the keypad autosleep */
	ret = tc35894_reg_write(tc35894,TC35894_GPIOWAKE1,0x00);
	if (ret < 0)
		return ret;
	/* enable the keypad autosleep */
	ret = tc35894_reg_write(tc35894,TC35894_GPIOWAKE2,0x00);
	if (ret < 0)
		return ret;
	ret = tc35894_reg_write(tc35894,TC35894_DIRECT0,0x00);
	if (ret < 0)
		return ret;
	ret = tc35894_reg_write(tc35894,TC35894_DIRECT1,0x00);
	if (ret < 0)
		return ret;
	ret = tc35894_reg_write(tc35894,TC35894_DIRECT2,0x00);
	if (ret < 0)
		return ret;
	ret = tc35894_reg_write(tc35894,TC35894_DIRECT3,0x00);
	if (ret < 0)
		return ret;

#if 1 
	/* enable the keypad autosleep */
	ret = tc35894_reg_write(tc35894,TC35894_AUTOSLPENA,0x01);
	if (ret < 0)
		return ret;
	/* enable the keypad autosleep timeout, one address two-byte inside*/
	ret = tc35894_reg_write(tc35894,TC35894_AUTOSLPTIMER,0x01);
	if (ret < 0)
		return ret;
	ret = tc35894_reg_write(tc35894,TC35894_AUTOSLPTIMER+1,0x00);
	if (ret < 0)
		return ret;
//	printk("tc35894 configure finished!\n");
#endif
	return ret;
}

static int keypad_state_gen(u8 row_index, u8 col_index , u8 *state, u8 key_release)
{
//	u8 i;
	//u8 col_line = col_index?(1<<col_index):0;
	u8 col_line = 1<<col_index;
	if(key_release)
		state[row_index] &= ~col_line;
	else
		state[row_index] |= col_line;
/*
	for(i = 0; i<TC_KPD_ROWS ; i++)
	{
		printk("state[%d]= %d\n",i,state[i]);
	}
*/
	return 0;
}


static int is_ghost_key_press(u8 *state , u8 row_num)
{
	u8 check = 0;
	u8 row,i;
	for(row = 0; row < row_num; row++)
	{
		for(i = row+1 ; i<row_num ; i++)
		{
			check = state[row];
			check &= state[i];
			
		//	printk("check = %x\n",num_of_High_bit(check));	
        		if(hweight8(check)>= 1&&(hweight8(state[row])>1||(hweight8(state[i])>1)))
			{
                	//	printk("check = %d\n",hweight8(check));
				return 1;
			}
		}
	}
	return 0;
}
int TC35894_key_event_send(struct input_dev *dev,unsigned int code,int value)
{

//	input_event(dev, EV_MSC, MSC_SCAN, code);
	check_home_mulkey(dev, *(current_keymap+code),value);
        input_report_key(dev, *(current_keymap+code), value);
        input_sync(dev);
	return 0;
}
#define TC35893_DATA_REGS               4
#define TC35893_KEYCODE_FIFO_EMPTY      0x7f
#define TC35893_KEYCODE_FIFO_CLEAR      0xff
#define TC35893_KEYPAD_ROW_SHIFT        0x3

int keypad_release_all(struct tc_keypad *keypad)
{
        u8 row_index, col_index;
        u8 code;
        u8 release = 0;
        for(row_index = 0; row_index<TC_KPD_ROWS; row_index++ )
        {
                if(keypadstate[row_index]==0)
                        continue;
                col_index = 0;
		//printk("keypadstate[0x%x] =0x%x\n ",row_index,keypadstate[row_index]);
                while(keypadstate[row_index]!=0){
                        keypadstate[row_index] = keypadstate[row_index]>>1;
			col_index++;
			if(keypadstate[row_index]&0x01){
        	        	code = MATRIX_SCAN_CODE(row_index, col_index,
                	                                TC35893_KEYPAD_ROW_SHIFT);
               		//	printk("release keymap[code] = %d code:%d\n ",keypad->board->keymap_data->keymap[code],code );
                		TC35894_key_event_send(keypad->input, code, release);				
                	}	
		}
        }
	return 0;
}
int Is_Ctrl_KEY_Press(void)
{
	u8 mControl = CNTLCODE;
	u8 mControlIndex = 0;
	u8 i, ret;
	for(i=0 ; i<TC_KPD_ROWS;i++)
	{
		//printk("keypadstate[%d]=%02x\n",i, keypadstate[i]);
		if((u64)1<<mControl >= (u64)1<<TC_KPD_COLUMNS){
			mControl -= TC_KPD_COLUMNS;	
			mControlIndex++;
		}	
	}
	//printk("mControlIndex = %d mControl = %d \n",  mControlIndex,  mControl);
	if(keypadstate[mControlIndex] & 1<<mControl){
			return 1;
	}
	return 0;
}
int isModeChange(void)
{
	u8 mShift = SHIFTCODE;
	u8 mControl = CNTLCODE;
	u8 mShiftIndex = 0;
	u8 mControlIndex = 0;
	u8 i, ret;
	for(i=0 ; i<TC_KPD_ROWS;i++)
	{
		if((u64)1<<mShift >= (u64)1<<TC_KPD_COLUMNS){
			mShift -= TC_KPD_COLUMNS;
			mShiftIndex++;
		}
		if((u64)1<<mControl >= (u64)1<<TC_KPD_COLUMNS){
			mControl -= TC_KPD_COLUMNS;	
			mControlIndex++;
		}	
	}
	//printk("mShiftIndex = %d mShift = %d mControlIndex = %d mControl = %d \n", mShiftIndex, mShift, mControlIndex,  mControl);
        if((keypadstate[mControlIndex] == 1<<mControl) && (keypadstate[mShiftIndex] == 1<<mShift)){
		for(i = 0, ret = 0; i<TC_KPD_ROWS;i++){
			if(i!=mShiftIndex && i!=mControlIndex)
				ret |=keypadstate[i];
		}
		if(ret==0)
			return 1;
	}
	return 0;
}

int event_buffer_overflow_handling(struct tc_keypad *keypad, struct TC_last_Pkey *last_key_press, u8 behavior)
{

	u8 release = 0;
	keypad_release_all(keypad);
//	kbd_code = tc35894_reg_read(tc35894, TC35894_EVTCODE_FIFO);
	if(behavior!= 0xff){
		TC35894_key_event_send(keypad->input,last_key_press->code, !release);
		keypad_state_gen(last_key_press->row_index,last_key_press->col_index, keypadstate,0x00);
		//printk("last_key_press->code = %d\n",last_key_press->code);
	}
	return 0;
}
int toggleModeState(struct tc_keypad *keypad, u8 modeCode)
{
	input_report_key(keypad->input,*(current_keymap+modeCode),1);
	input_sync(keypad->input);
	input_report_key(keypad->input,*(current_keymap+modeCode),0);
	input_sync(keypad->input);
	return 0;
}
u8 isAllKeyReleaseExcept(u8 code)
{
	u8 i, ret; 
	u8 mCode;
	u8 mIndex;
	if(code == NONEKEY){
		mIndex = NONEKEY;
		goto calculate;
	}
	for(i=0, mCode = code, mIndex =0; i<TC_KPD_ROWS; i++)
	{
		if(1<<mCode >= 1<<TC_KPD_COLUMNS){
			mCode -= TC_KPD_COLUMNS;
			mIndex++;
		}
		else
	  		break;
	}
	if(keypadstate[mIndex] != 1<<mCode){
		return 0;
	}
calculate:
	for(i = 0, ret =0; i<TC_KPD_ROWS; i++)
	{
		if(i != mIndex )
			ret |= keypadstate[i];
	}
	return ret? 0: 1;
}

int handleShiftHoldOnce(struct tc_keypad *keypad, u8 code, u8 release)
{
	static u8 statePTR = 0;
	static u8 lastpresskeycode = 0;
	if(!release)
		lastpresskeycode = code;
	switch(statePTR)
	{
		case 0:
			if(lastpresskeycode == SHIFTCODE && isAllKeyReleaseExcept(SHIFTCODE) && !release)
				statePTR = 1;
			else
				statePTR = 0;
			break;
		case 0x1:
			if(lastpresskeycode == code && code == SHIFTCODE && release)
				statePTR = 2;
			else
				statePTR = 0;
			break;
		case 0x2:
			if(!release){
	                        input_report_key(keypad->input,*(current_keymap+SHIFTCODE),1);
        	                input_sync(keypad->input);
                	        input_report_key(keypad->input,*(current_keymap+code),1);
                       		input_sync(keypad->input);
                        	input_report_key(keypad->input,*(current_keymap+SHIFTCODE),0);
                        	input_sync(keypad->input);
			}		
				statePTR = 0;
			break;
		default: 
			printk("Error: unexpected state appears in TC35894-keypad One time Shift effect\n");	
			statePTR = 0;
			break;
	}	

return 0;
}
static void tc35894_keypad_work(struct work_struct *work)
{
	struct tc_keypad *keypad = work_to_tc35894(work);
	struct tc35894 *tc35894 = keypad->tc35894;
	u8 i, kbd_code;
#if 0
	u8 code, row_index, col_index, up;
#endif
	u8 behavior = 0;
	struct TC_last_Pkey last_Pkey[8]; 
	struct TC_last_Pkey fifo_Pkey[8]; 
	u8 KP_index = 0;
	int key_cnt=0;
	kbd_code=tc35894_reg_read(tc35894, TC35894_KBDMIS);
	//printk("KBDMIS:___%02x\n",kbd_code);
	if(kbd_code & TC35894_EVT_LOSS_INT){
		tc35894_set_bits(tc35894, TC35894_KBDIC, 0x0, TC35894_EVT_INT_CLR);
		return;
	}
	if(kbd_code & TC35894_KBD_INT){
		tc35894_set_bits(tc35894, TC35894_KBDIC, 0x0, TC35894_KBD_INT_CLR);
	}

	disable_irq_nosync(gpio_to_irq(MX53_TC35894_KEY_INT));
	/*sync Keyboard Status with whole system*/	
	switch(keypad->input->led[0])
	{
		case 0:
			keypad->keyboard_mode = NORMAL_LOCK;
			break;		
		case 0x01:
			keypad->keyboard_mode = NUMB_LOCK;
			break;
		case 0x02:
			keypad->keyboard_mode = CAPS_LOCK;
			break;	
		default:
			keypad->keyboard_mode = NORMAL_LOCK;
			printk("Error case 0x%lx : return bask to NORMAL\n",keypad->input->led[0]);
			if((keypad->input->led[0]&(1UL))!=0)
				toggleModeState(keypad, CAPSCODE);//disable CAPS_LOCK
			if((keypad->input->led[0]&(1UL<<1))!=0)
				toggleModeState(keypad, NUMCODE);//disable NUM_LOCK
			break;
	}
#if 1
	//读完fifo后，确定是否存在ghost，再发送按键
	for (i = 0; i < TC35893_DATA_REGS * 2; i++) {
		kbd_code = tc35894_reg_read(tc35894, TC35894_EVTCODE_FIFO);
		if (kbd_code == TC35893_KEYCODE_FIFO_EMPTY ||
				kbd_code == TC35893_KEYCODE_FIFO_CLEAR){
			break;//	continue;
		}
		fifo_Pkey[i].col_index = kbd_code & KP_EVCODE_COL_MASK;
		fifo_Pkey[i].row_index = (kbd_code & KP_EVCODE_ROW_MASK) >> KP_ROW_SHIFT;
		fifo_Pkey[i].code = MATRIX_SCAN_CODE(fifo_Pkey[i].row_index, fifo_Pkey[i].col_index, TC35893_KEYPAD_ROW_SHIFT);
		fifo_Pkey[i].up = kbd_code & KP_RELEASE_EVT_MASK;//keyboard IC release:1 press:0; Linux release:0 press:1;
		//printk("i:%d col:%d row:%d code:%d up:%d\n",i, fifo_Pkey[i].col_index,fifo_Pkey[i].row_index,fifo_Pkey[i].code,fifo_Pkey[i].up);
		keypad_state_gen(fifo_Pkey[i].row_index, fifo_Pkey[i].col_index, keypadstate, fifo_Pkey[i].up);		
	}
	key_cnt = i;
	if(!is_ghost_key_press(keypadstate, TC_KPD_ROWS)){
		for (i = 0; i < key_cnt; i++) {
			if(!fifo_Pkey[i].up){
				/***先按ctrl,再按shift才切换模式，所以发现
				 * shift按下消息才检查是否ctl也被按下**/
				if(fifo_Pkey[i].code == SHIFTCODE && isModeChange()){/*check if Shift and control pressed*/
					if(keypad->keyboard_mode == NORMAL_LOCK){
						toggleModeState(keypad, CAPSCODE);//enable CAPS_LOCK
						printk("CAPSLOCK\n");
					}
					else if(keypad->keyboard_mode == CAPS_LOCK){
						toggleModeState(keypad, NUMCODE);//enable NUM_LOCK
						toggleModeState(keypad, CAPSCODE);//disable CAPS_LOCK
						printk("NUMLOCK\n");
					}
					else{
						toggleModeState(keypad, NUMCODE);//disable NUM_LOCK
						printk("NORMAL\n");
					}
				}
				last_Pkey[KP_index]=fifo_Pkey[i];
				if(KP_index<8)	
					KP_index++;
			} else if(KP_index > 0){
				if(last_Pkey[KP_index-1].code == fifo_Pkey[i].code)
					KP_index--;
			}
			if(kb_addon && !fifo_Pkey[i].up){
				int scan_code = *(current_keymap+fifo_Pkey[i].code);
				//字母键和?键时候处理特殊功能
				if((KEY_Q <= scan_code && scan_code <= KEY_P) || (KEY_A <= scan_code && scan_code <= KEY_L) || (KEY_Z <= scan_code && scan_code <= KEY_M) || scan_code==KEY_JPN_SYM) {
					if(!Is_Ctrl_KEY_Press()){
					kb_addon = 0;
					input_report_key(keypad->input, KEY_ADDON, 1);
					input_sync(keypad->input);
					input_report_key(keypad->input, KEY_ADDON, 0);
					input_sync(keypad->input);
					//mingliao require it to make app have
					//full time to switch windown, but it
					//have possible to lost key because
					//irq was disable so long time
					msleep(260);
					}
					else
						printk("Ctrl key was pressed, don't deal add on function");
				}
			}
			check_home_mulkey(keypad->input, *(current_keymap+fifo_Pkey[i].code),!fifo_Pkey[i].up);
//			if(*(current_keymap+fifo_Pkey[i].code) < KEY_YI_KUO || *(current_keymap+fifo_Pkey[i].code) > KEY_JPNONE) {	//功能键被按下
				//keyboard IC release:1 press:0; Linux release:0 press:1;
				input_report_key(keypad->input,*(current_keymap+fifo_Pkey[i].code),!fifo_Pkey[i].up);
				input_sync(keypad->input);
//			}
		}
	}else{
		printk("Get ghost key\n");
	}
	if(kbd_code== TC35893_KEYCODE_FIFO_CLEAR)
		keypad_release_all(keypad);
	else if(key_cnt==8){
		while((behavior ^0x7f) & 0x7f){
			behavior = tc35894_reg_read(tc35894, TC35894_EVTCODE_FIFO);	
		}
		event_buffer_overflow_handling(keypad,&last_Pkey[KP_index-1], behavior);
	}
#else
	/*read keyboard input event and handle the events*/
	for (i = 0; i < TC35893_DATA_REGS * 2; i++) {
		kbd_code = tc35894_reg_read(tc35894, TC35894_EVTCODE_FIFO);
		if (kbd_code == TC35893_KEYCODE_FIFO_EMPTY ||
				kbd_code == TC35893_KEYCODE_FIFO_CLEAR){
			break;//	continue;
		}
		/* valid key is found */
		col_index = kbd_code & KP_EVCODE_COL_MASK;
		row_index = (kbd_code & KP_EVCODE_ROW_MASK) >> KP_ROW_SHIFT;
		code = MATRIX_SCAN_CODE(row_index, col_index, TC35893_KEYPAD_ROW_SHIFT);
		printk("row_index = %d col_index = %d code = %d kbdcode = %02x\n",row_index,col_index,code,kbd_code);
		up = kbd_code & KP_RELEASE_EVT_MASK;//keyboard IC release:1 press:0; Linux release:0 press:1;
		/*generate key-state-map*/
		keypad_state_gen(row_index, col_index, keypadstate, up);		
		if(!up ){
			if(is_ghost_key_press(keypadstate, TC_KPD_ROWS)){/*if a key-press appears, then check ghost key*/	
				keypad_state_gen(row_index, col_index, keypadstate, !up);
				continue;		
			}
			if(isModeChange()){/*check if Shift and control pressed*/
				if(keypad->keyboard_mode == NORMAL_LOCK){
					toggleModeState(keypad, CAPSCODE);//enable CAPS_LOCK
//					printk("CAPSLOCK\n");
				}
				else if(keypad->keyboard_mode == CAPS_LOCK){
					toggleModeState(keypad, NUMCODE);//enable NUM_LOCK
					toggleModeState(keypad, CAPSCODE);//disable CAPS_LOCK
//					printk("NUMLOCK\n");
				}
				else{
					toggleModeState(keypad, NUMCODE);//disable NUM_LOCK
  //                                      printk("NORMAL\n");

				}
			}
			else{
			/*Save the last pressed key to handle exception that event overflow and have keypress*/
				last_Pkey[KP_index].code = *(current_keymap+code);
				last_Pkey[KP_index].row_index = row_index;
				last_Pkey[KP_index].col_index = col_index;
				if(KP_index<8)	
					KP_index++;
			}
//			 printk("keypad->input->led[0] = 0x%lx\n",keypad->input->led[0]);
		}

		else if(KP_index > 0){
			if(last_Pkey[KP_index-1].code == *(current_keymap+code))
				KP_index--;
		}
		/*check and handle Shift ket effect */
//		if(!handleShiftHoldOnce(keypad, code, up)){
			input_report_key(keypad->input,*(current_keymap+code),!up);//keyboard IC release:1 press:0; Linux release:0 press:1;
			input_sync(keypad->input);
//		}
	}
	
	/*when event buffer among is larger than/ equals to 8, then event buffer overfolws */
	/*for the bug that all keys have been released but some release events are missing*/	
	if(kbd_code== 0xff)
		keypad_release_all(keypad);
	else if(i==8){
		while((behavior ^0x7f) & 0x7f){
			behavior = tc35894_reg_read(tc35894, TC35894_EVTCODE_FIFO);	
//			printk("behavior = 0x%x\n",behavior);
		}
		event_buffer_overflow_handling(keypad,&last_Pkey[KP_index-1], behavior);
	}
#endif
	enable_irq(gpio_to_irq(MX53_TC35894_KEY_INT));
	enable_irq(gpio_to_irq(MX53_TC35894_KEY_INT));
	/* clear IRQ for keyevent and key lost and clear event buffer*/
	/* enable IRQ */
//	tc35894_set_bits(tc35894, TC35894_KBDMSK, 0x0, TC35894_EVT_LOSS_INT | TC35894_EVT_INT);
//	printk("schedule_work_done!\n");
	/* Enable IRQs again */
}


static irqreturn_t tc35894_keypad_irq(int irq, void *dev)
{
	struct tc_keypad *keypad = dev;
	schedule_delayed_work(&keypad->work,msecs_to_jiffies(10));
//	schedule_work(&keypad->work);
	return IRQ_HANDLED;
}
static int tc35894_KB_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
//	printk("________tc35894_KB_event : tyep = %d , code = %d , value = %d, led =0x%lx\n",type, code, value, dev->led[0]);
	return 0;
}

static int tc35894_keypad_enable(struct tc_keypad *keypad)
{
	struct tc35894 *tc35894 = keypad->tc35894;
	int ret;

	/* pull the keypad module out of reset */
/*
	ret = tc35894_reg_write(tc35894, TC35894_RSTCTRL,0x00|TC35894_GPIRST);
	ret = tc35894_reg_write(tc35894, TC35894_RSTCTRL,0x00);
	if (ret < 0)
		return ret;
*/
	ret = tc35894_set_bits(tc35894, TC35894_RSTCTRL, TC35894_KBDRST, 0x0);
	if (ret < 0)
		return ret;

	ret = tc35894_set_bits(tc35894, TC35894_RSTCTRL, TC35894_GPIRST, 0x0);
	if (ret < 0)
		return ret;

	/* configure KBDMFS */
	ret = tc35894_set_bits(tc35894, TC35894_KBDMFS, 0x0, TC35894_KBDMFS_EN);
	if (ret < 0)
		return ret;

	/* enable the keypad clock */
	ret = tc35894_set_bits(tc35894, TC35894_CLKEN, 0x0, KPD_CLK_EN);
	if (ret < 0)
		return ret;

	/* clear pending IRQs */
	ret =  tc35894_set_bits(tc35894, TC35894_RSTINTCLR, 0x0, 0x1);
	if (ret < 0)
		return ret;

	/* enable the IRQs */
	ret = tc35894_set_bits(tc35894, TC35894_KBDMSK, 0x0,
					TC35894_EVT_LOSS_INT | TC35894_EVT_INT);
	if (ret < 0)
		return ret;

	keypad->keypad_stopped = false;

	return ret;
}

static int tc35894_keypad_disable(struct tc_keypad *keypad)
{
	struct tc35894 *tc35894 = keypad->tc35894;
	int ret;

	/* clear IRQ */
	ret = tc35894_set_bits(tc35894, TC35894_KBDIC,
			0x0, TC35894_EVT_INT_CLR | TC35894_KBD_INT_CLR);
	if (ret < 0)
		return ret;

	/* disable all interrupts */
	ret = tc35894_set_bits(tc35894, TC35894_KBDMSK,
			~(TC35894_EVT_LOSS_INT | TC35894_EVT_INT), 0x0);
	if (ret < 0)
		return ret;

	/* disable the keypad module */
	ret = tc35894_set_bits(tc35894, TC35894_CLKEN, 0x1, 0x0);
	if (ret < 0)
		return ret;

	/* put the keypad module into reset */
//	ret = tc35894_set_bits(tc35894, TC35894_RSTCTRL, TC35894_KBDRST, 0x1);

	keypad->keypad_stopped = true;

	return ret;
}

static int tc35894_keypad_open(struct input_dev *input)
{
	int error;
	struct tc_keypad *keypad = input_get_drvdata(input);
//	printk("@@@@@@@@@@tc35894_keypad_open run ...\n");
	/* enable the keypad module */
	error = tc35894_keypad_enable(keypad);
	if (error < 0) {
		dev_err(&input->dev, "failed to enable keypad module\n");
		return error;
	}

	error = tc35894_keypad_init_key_hardware(keypad);
	if (error < 0) {
		dev_err(&input->dev, "failed to configure keypad module\n");
		return error;
	}

	return 0;
}

static void tc35894_keypad_close(struct input_dev *input)
{
	struct tc_keypad *keypad = input_get_drvdata(input);

	/* disable the keypad module */
	tc35894_keypad_disable(keypad);
}

static int kb_addon_read_proc(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	char buff[4] = {0};
	sprintf(buff,"%d",kb_addon);
	len = sprintf(page, buff);
	return len;
}

char addon_buff[10];
static int kb_addon_write_proc(struct file *file, const char __user *buffer,
			 unsigned long count, void *data)
{
    unsigned long value=0;
    int ret=0;
    memset(addon_buff,0,sizeof(addon_buff));
    if(count <= 10) {
	    if (copy_from_user(addon_buff, buffer, count))
		    return -EFAULT;
    } else {
	    if (copy_from_user(addon_buff, buffer, 10))
		    return -EFAULT;
    }
    ret=strict_strtoul(addon_buff, 0, &value);
    if(ret != 0)
		    return -EFAULT;
    kb_addon=!!value;
    return count;
}

struct tc_keypad *tc35894_keypad;
static int kb_enable_write_proc(struct file *file, const char __user *buffer,
			 unsigned long count, void *data)
{
    unsigned long value=0;
    int ret=0;
    memset(addon_buff,0,sizeof(addon_buff));
    if(count <= 10) {
	    if (copy_from_user(addon_buff, buffer, count))
		    return -EFAULT;
    } else {
	    if (copy_from_user(addon_buff, buffer, 10))
		    return -EFAULT;
    }
    ret=strict_strtoul(addon_buff, 0, &value);
    if(ret != 0)
		    return -EFAULT;
    if(value){
	    if(tc35894_keypad)
	    	tc35894_keypad_enable(tc35894_keypad);
	    enable_irq(gpio_to_irq(MX53_TC35894_KEY_INT));
    }else {
	    if(tc35894_keypad)
	    	tc35894_keypad_disable(tc35894_keypad);
	    disable_irq(gpio_to_irq(MX53_TC35894_KEY_INT));
    }

    return count;
}
static int __devinit tc35894_keypad_probe(struct platform_device *pdev)
{
	struct tc35894 *tc35894 = dev_get_drvdata(pdev->dev.parent);
	struct input_dev *input;
	const struct tc35894_keypad_platform_data *plat;
	struct tc_keypad *keypad;
	int error, irq;
	plat = tc35894->pdata->keypad;
	if (!plat) {
		dev_err(&pdev->dev, "invalid keypad platform data\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	keypad = kzalloc(sizeof(struct tc_keypad), GFP_KERNEL);
	input = input_allocate_device();
	if (!keypad || !input) {
		dev_err(&pdev->dev, "failed to allocate keypad memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}
	keypad->board = plat;
	keypad->input = input;
	keypad->tc35894 = tc35894;
	keypad->keyboard_mode = NORMAL_LOCK;
	input->id.bustype = BUS_I2C;
	input->name = pdev->name;
	input->dev.parent = &pdev->dev;
	input->keycode = keypad->keymap;
	input->keycodesize = sizeof(keypad->keymap[0]);
	input->keycodemax = ARRAY_SIZE(keypad->keymap);
	input->open = tc35894_keypad_open;
	input->close = tc35894_keypad_close;
	input->event = tc35894_KB_event;
	input_set_drvdata(input, keypad);
	input_set_capability(input, EV_MSC, MSC_SCAN);
	input_set_capability(input, EV_LED, LED_NUML);
	input_set_capability(input, EV_LED, LED_CAPSL);
	__set_bit(EV_KEY, input->evbit);
	if (!plat->no_autorepeat)
	__set_bit(EV_REP, input->evbit);
	__set_bit(EV_LED, input->evbit);
	
	/*Schedule work initailize*/
	INIT_DELAYED_WORK(&keypad->work, tc35894_keypad_work);
	/*The end of schedule work initialize*/
	spin_lock_init(&keypad->lock);
	
	/*tell linux kernel what keys are supported by this keyboard*/
	matrix_keypad_build_keymap(plat->keymap_data, 0x3,\
			input->keycode, input->keybit);
	__set_bit(KEY_ADDON, input->keybit);
	__set_bit(KEY_HOME, input->keybit);
	current_keymap = keypad->board->keymap_data->keymap;
	
	memset(keypadstate, 0, sizeof(keypadstate));
	error = request_threaded_irq(irq, NULL,
			tc35894_keypad_irq, plat->irqtype,
			"tc35894-keypad", keypad);
	if (error < 0) {
		dev_err(&pdev->dev,
				"Could not allocate irq %d,error %d\n",
				irq, error);
		goto err_free_mem;
	}
	error = input_register_device(input);
	if (error) {
		dev_err(&pdev->dev, "Could not register input device\n");
		goto err_free_irq;
	}

	/* let platform decide if keypad is a wakeup source or not */
	device_init_wakeup(&pdev->dev, plat->enable_wakeup);
	device_set_wakeup_capable(&pdev->dev, plat->enable_wakeup);

	enable_irq_wake(irq);
	platform_set_drvdata(pdev, keypad);
	
	tc35894_keypad = keypad;
	struct proc_dir_entry *res;
	struct proc_dir_entry *proc_fv40_kb_root;
	proc_fv40_kb_root = proc_mkdir("keyboard", 0);
	res=create_proc_entry("addon", 0644, proc_fv40_kb_root);
	if(res) {
		res->read_proc = kb_addon_read_proc;
		res->write_proc = kb_addon_write_proc;
		res->data = NULL;
	}

	res=create_proc_entry("enable", 0644, proc_fv40_kb_root);
	if(res) {
		res->read_proc = NULL;
		res->write_proc = kb_enable_write_proc;
		res->data = NULL;
	}

	return 0;

err_free_irq:
	free_irq(irq, keypad);
err_free_mem:
	input_free_device(input);
	kfree(keypad);
	return error;
}

static int __devexit tc35894_keypad_remove(struct platform_device *pdev)
{
	struct tc_keypad *keypad = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

	if (!keypad->keypad_stopped)
		tc35894_keypad_disable(keypad);

	free_irq(irq, keypad);

	input_unregister_device(keypad->input);

	kfree(keypad);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tc35894_keypad_suspend(struct device *dev)
{

	struct platform_device *pdev = to_platform_device(dev);
	struct tc_keypad *keypad = platform_get_drvdata(pdev);
	int irq = platform_get_irq(pdev, 0);

	/* keypad is already off; we do nothing */
	if (keypad->keypad_stopped)
		return 0;

	/* if device is not a wakeup source, disable it for powersave */
	if (!device_may_wakeup(&pdev->dev));
//		tc35894_keypad_disable(keypad);
	else
		enable_irq_wake(irq);

	return 0;
}

static int tc35894_keypad_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tc_keypad *keypad = platform_get_drvdata(pdev);
//	int irq = platform_get_irq(pdev, 0);


	if (!keypad->keypad_stopped)
		return 0;
/*
	// enable the device to resume normal operations 
	if (!device_may_wakeup(&pdev->dev))
		tc35894_keypad_enable(keypad);
	else
		disable_irq_wake(irq);
*/
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(tc35894_keypad_dev_pm_ops,
			 tc35894_keypad_suspend, tc35894_keypad_resume);

static struct platform_driver tc35894_keypad_driver = {
	.driver	= {
		.name	= "tc35894-keypad",
		.owner	= THIS_MODULE,
		.pm	= &tc35894_keypad_dev_pm_ops,
	},
	.probe	= tc35894_keypad_probe,
	.remove	= __devexit_p(tc35894_keypad_remove),
};

static int __init tc35894_keypad_init(void)
{
	return platform_driver_register(&tc35894_keypad_driver);
}
module_init(tc35894_keypad_init);

static void __exit tc35894_keypad_exit(void)
{
	return platform_driver_unregister(&tc35894_keypad_driver);
}
module_exit(tc35894_keypad_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jayeeta Banerjee/Sundar Iyer");
MODULE_DESCRIPTION("TC35894 Keypad Driver");
MODULE_ALIAS("platform:tc35894-keypad");
