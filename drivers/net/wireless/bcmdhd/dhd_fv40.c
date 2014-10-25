/*
 * ==========================================================================
 *
 *       Filename:  dhd_fv40.c
 *
 *    Description:  
 *
 *        Version:  0.01
 *        Created:  2012年06月12日 17时11分15秒
 *
 *         Author:  xyxu (), 
 *        Company:  
 *
 * ==========================================================================
 */

extern unsigned wlan_reset_pin ;
extern unsigned wlan_pwr_pin ;

#include <linux/gpio.h>
#include <linux/delay.h>
#ifdef CUSTOMER_HW
void bcm_wlan_power_off(int i)
{
	gpio_request(wlan_reset_pin, "wifi-reset");
	gpio_direction_output(wlan_reset_pin, 0);
	/* Enable WiFi/BT Power*/
	//gpio_request(wlan_pwr_pin, "bt-wifi-pwren");
	//gpio_direction_output(wlan_pwr_pin, 0);

}
void bcm_wlan_power_on(int i)
{
	gpio_request(wlan_reset_pin, "wifi-reset");
	gpio_direction_output(wlan_reset_pin, 0);
	/* Enable WiFi/BT Power*/
	gpio_request(wlan_pwr_pin, "bt-wifi-pwren");
	gpio_direction_output(wlan_pwr_pin, 1);
	gpio_free(wlan_pwr_pin);
	udelay(60);
	gpio_set_value(wlan_reset_pin, 1);
}
#endif
