/*
 * AMLOGIC backlight driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the named License,
 * or any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA
 *
 * Author:  Wang Han <han.wang@amlogic.com>
 *  
 * Modify:  Evoke Zhang <evoke.zhang@amlogic.com>
 * compatible dts
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/backlight.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <mach/power_gate.h>
#include <linux/delay.h>
#include <linux/amlogic/aml_lcd_bl.h>
#ifdef CONFIG_AML_BACKLIGHT_EXTERN
#include <linux/amlogic/aml_bl_extern.h>
#endif

//#define MESON_BACKLIGHT_DEBUG
#ifdef MESON_BACKLIGHT_DEBUG
#define DPRINT(...)   printk(KERN_INFO __VA_ARGS__)
#define DTRACE()      DPRINT(KERN_INFO "%s()\n", __func__)
static const char* bl_ctrl_method_table[]={
    "gpio",
    "pwm_negative",
    "pwm_positive",
    "pwm_combo",
    "extern",
    "null"
};
#else
#define DPRINT(...)
#define DTRACE()
#endif /* MESON_BACKLIGHT_DEBUG */
#define BL_PR(...)   printk(KERN_INFO __VA_ARGS__)

static struct aml_bl_s *amlbl;

#ifdef CONFIG_OF
static struct lcd_bl_config_s bl_config = {
    .level_default = 128,
    .level_mid = 128,
    .level_mid_mapping = 128,
    .level_min = 10,
    .level_max = 255,
    .power_on_delay = 100,
    .method = BL_CTL_MAX,
};
#endif

#define FIN_FREQ		(24 * 1000)

void get_bl_ext_level(struct bl_extern_config_t *bl_ext_cfg)
{
    if (amlbl == NULL) {
        BL_PR("no bl data\n");
        return;
    }
    bl_ext_cfg->level_min = amlbl->bconf->level_min;
    bl_ext_cfg->level_max = amlbl->bconf->level_max;
}

static DEFINE_MUTEX(bl_power_mutex);
static void power_on_bl(void)
{
    struct lcd_bl_config_s *bconf;
    struct pinctrl_state *s;
    struct aml_bl_extern_driver_t *bl_extern_driver;
    int ret;

    if (amlbl == NULL) {
        BL_PR("no bl data\n");
        return;
    } else {
        bconf = amlbl->bconf;
    }
    mutex_lock(&bl_power_mutex);

    DPRINT("%s: bl_level=%u, state=0x%x\n", __func__, amlbl->level, amlbl->state);
    if ((amlbl->level == 0) ||
      ((amlbl->state & BL_STATE_BL_ON) == 0) ||
      (amlbl->state & BL_STATE_REAL_ON)) {
        goto exit_power_on_bl;
    }

    switch (bconf->method) {
        case BL_CTL_GPIO:
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
            aml_set_reg32_bits(P_LED_PWM_REG0, 1, 12, 2);
#endif
            mdelay(20);
            bl_gpio_direction_output(bconf->gpio, bconf->gpio_on);
            break;
        case BL_CTL_PWM_NEGATIVE:
        case BL_CTL_PWM_POSITIVE:
            switch (bconf->pwm_port) {
                case BL_PWM_A:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, bconf->pwm_pre_div, 8, 7);  //pwm_a_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 4, 2);  //pwm_a_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 15, 1);  //pwm_a_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 0, 1);  //enable pwm_a
                    break;
                case BL_PWM_B:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, bconf->pwm_pre_div, 16, 7);  //pwm_b_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 6, 2);  //pwm_b_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 23, 1);  //pwm_b_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 1, 1);  //enable pwm_b
                    break;
                case BL_PWM_C:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, bconf->pwm_pre_div, 8, 7);  //pwm_c_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 4, 2);  //pwm_c_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 15, 1);  //pwm_c_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 0, 1);  //enable pwm_c
                    break;
                case BL_PWM_D:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, bconf->pwm_pre_div, 16, 7);  //pwm_d_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 6, 2);  //pwm_d_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 23, 1);  //pwm_d_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 1, 1);  //enable pwm_d
                    break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                case BL_PWM_E:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, bconf->pwm_pre_div, 8, 7);  //pwm_c_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 4, 2);  //pwm_c_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 15, 1);  //pwm_c_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 0, 1);  //enable pwm_c
                    break;
                case BL_PWM_F:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, bconf->pwm_pre_div, 16, 7);  //pwm_d_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 6, 2);  //pwm_d_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 23, 1);  //pwm_d_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 1, 1);  //enable pwm_d
                    break;
#endif
                default:
                    break;
            }

            if (IS_ERR(bconf->p)) {
                printk("set backlight pinmux error.\n");
                goto exit_power_on_bl;
            }
            s = pinctrl_lookup_state(bconf->p, "default"); //select pinctrl
            if (IS_ERR(s)) {
                printk("set backlight pinmux error.\n");
                devm_pinctrl_put(bconf->p);
                goto exit_power_on_bl;
            }

            ret = pinctrl_select_state(bconf->p, s); //set pinmux and lock pins
            if (ret < 0) {
                printk("set backlight pinmux error.\n");
                devm_pinctrl_put(bconf->p);
                goto exit_power_on_bl;
            }
            mdelay(20);
            if (bconf->pwm_gpio_used) {
                if (bconf->gpio)
                    bl_gpio_direction_output(bconf->gpio, bconf->gpio_on);
            }
            break;
        case BL_CTL_PWM_COMBO:
            switch (bconf->combo_high_port) {
                case BL_PWM_A:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, bconf->combo_high_pre_div, 8, 7);  //pwm_a_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 4, 2);  //pwm_a_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 15, 1);  //pwm_a_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 0, 1);  //enable pwm_a
                    break;
                case BL_PWM_B:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, bconf->combo_high_pre_div, 16, 7);  //pwm_b_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 6, 2);  //pwm_b_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 23, 1);  //pwm_b_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 1, 1);  //enable pwm_b
                    break;
                case BL_PWM_C:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, bconf->combo_high_pre_div, 8, 7);  //pwm_c_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 4, 2);  //pwm_c_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 15, 1);  //pwm_c_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 0, 1);  //enable pwm_c
                    break;
                case BL_PWM_D:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, bconf->combo_high_pre_div, 16, 7);  //pwm_d_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 6, 2);  //pwm_d_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 23, 1);  //pwm_d_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 1, 1);  //enable pwm_d
                    break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                case BL_PWM_E:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, bconf->combo_high_pre_div, 8, 7);  //pwm_c_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 4, 2);  //pwm_c_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 15, 1);  //pwm_c_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 0, 1);  //enable pwm_c
                    break;
                case BL_PWM_F:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, bconf->combo_high_pre_div, 16, 7);  //pwm_d_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 6, 2);  //pwm_d_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 23, 1);  //pwm_d_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 1, 1);  //enable pwm_d
                    break;
#endif
                default:
                    break;
            }
            switch (bconf->combo_low_port) {
                case BL_PWM_A:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, bconf->combo_low_pre_div, 8, 7);  //pwm_a_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 4, 2);  //pwm_a_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 15, 1);  //pwm_a_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 0, 1);  //enable pwm_a
                    break;
                case BL_PWM_B:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, bconf->combo_low_pre_div, 16, 7);  //pwm_b_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 6, 2);  //pwm_b_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 23, 1);  //pwm_b_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 1, 1, 1);  //enable pwm_b
                    break;
                case BL_PWM_C:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, bconf->combo_low_pre_div, 8, 7);  //pwm_c_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 4, 2);  //pwm_c_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 15, 1);  //pwm_c_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 0, 1);  //enable pwm_c
                    break;
                case BL_PWM_D:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, bconf->combo_low_pre_div, 16, 7);  //pwm_d_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 6, 2);  //pwm_d_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 23, 1);  //pwm_d_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 1, 1, 1);  //enable pwm_d
                    break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                case BL_PWM_E:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, bconf->combo_low_pre_div, 8, 7);  //pwm_c_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 4, 2);  //pwm_c_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 15, 1);  //pwm_c_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 0, 1);  //enable pwm_c
                    break;
                case BL_PWM_F:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, bconf->combo_low_pre_div, 16, 7);  //pwm_d_clk_div
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 6, 2);  //pwm_d_clk_sel
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 23, 1);  //pwm_d_clk_en
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 1, 1, 1);  //enable pwm_d
                    break;
#endif
                default:
                    break;
            }

            if (IS_ERR(bconf->p)) {
                printk("set backlight pinmux error.\n");
                goto exit_power_on_bl;
            }
            s = pinctrl_lookup_state(bconf->p, "pwm_combo");  //select pinctrl
            if (IS_ERR(s)) {
                printk("set backlight pinmux error.\n");
                devm_pinctrl_put(bconf->p);
                goto exit_power_on_bl;
            }

            ret = pinctrl_select_state(bconf->p, s);  //set pinmux and lock pins
            if (ret < 0) {
                printk("set backlight pinmux error.\n");
                devm_pinctrl_put(bconf->p);
                goto exit_power_on_bl;
            }
            break;
#ifdef CONFIG_AML_BACKLIGHT_EXTERN
        case BL_CTL_EXTERN:
            bl_extern_driver = aml_bl_extern_get_driver();
            if (bl_extern_driver == NULL) {
                printk("no bl_extern driver\n");
            }
            else {
                if (bl_extern_driver->power_on) {
                    ret = bl_extern_driver->power_on();
                    if (ret) {
                        printk("[bl_extern] power on error\n");
                        goto exit_power_on_bl;
                    }
                }
                else {
                    printk("[bl_extern] power on is null\n");
                }
            }
            break;
#endif
        default:
            printk("wrong backlight control method\n");
            goto exit_power_on_bl;
            break;
    }
    amlbl->state |= BL_STATE_REAL_ON; //bl_real_status = 1;
    printk("backlight power on\n");

exit_power_on_bl:
    mutex_unlock(&bl_power_mutex);
}

/* bl_delayed_work for LCD_BL_FLAG control */
static void bl_delayd_on(struct work_struct *work)
{
    if (amlbl->state & BL_STATE_LCD_ON)
        amlbl->state |= BL_STATE_BL_ON;
    power_on_bl();
}

void bl_power_on(int bl_flag)
{
    struct lcd_bl_config_s *bconf;

    if (amlbl == NULL) {
        BL_PR("no bl data\n");
        return;
    } else {
        bconf = amlbl->bconf;
    }
    DPRINT("%s(bl_flag=%s): bl_level=%u, state=0x%x\n", __func__, (bl_flag ? "LCD_BL_FLAG" : "DRV_BL_FLAG"), amlbl->level, amlbl->state);

    if (bconf->method < BL_CTL_MAX) {
        if (bl_flag == LCD_BL_FLAG) {
            amlbl->state |= BL_STATE_LCD_ON; //bl_status = 1;
            if (amlbl->workqueue) {
                queue_delayed_work(amlbl->workqueue, &amlbl->bl_delayed_work, msecs_to_jiffies(bconf->power_on_delay));
            }
            else {
                printk("[Warning]: no bl workqueue\n");
                msleep(bconf->power_on_delay);
                if (amlbl->state & BL_STATE_LCD_ON)
                    amlbl->state |= BL_STATE_BL_ON;
                power_on_bl();
            }
        }
        else {
            power_on_bl();
        }
    }
    else {
        printk("wrong backlight control method\n");
    }

    DPRINT("bl_power_on...\n");
}

void bl_power_off(int bl_flag)
{
    struct lcd_bl_config_s *bconf;
    struct aml_bl_extern_driver_t *bl_extern_driver;
    int ret;

    if (amlbl == NULL) {
        BL_PR("no bl data\n");
        return;
    } else {
        bconf = amlbl->bconf;
    }
    mutex_lock(&bl_power_mutex);

    if (bl_flag == LCD_BL_FLAG)
        amlbl->state &= ~(BL_STATE_LCD_ON | BL_STATE_BL_ON); //bl_status = 0;

    DPRINT("%s(bl_flag=%s): bl_level=%u, state=0x%x\n", __func__, (bl_flag ? "LCD_BL_FLAG" : "DRV_BL_FLAG"), amlbl->level, amlbl->state);
    if ((amlbl->state & BL_STATE_REAL_ON) == 0) {
        mutex_unlock(&bl_power_mutex);
        return;
    }

    switch (bconf->method) {
        case BL_CTL_GPIO:
            bl_gpio_direction_output(bconf->gpio, bconf->gpio_off);
            break;
        case BL_CTL_PWM_NEGATIVE:
        case BL_CTL_PWM_POSITIVE:
            if (bconf->pwm_gpio_used) {
                if (bconf->gpio)
                    bl_gpio_direction_output(bconf->gpio, bconf->gpio_off);
            }
            switch (bconf->pwm_port) {
                case BL_PWM_A:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 0, 1);  //disable pwm_a
                    break;
                case BL_PWM_B:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 1, 1);  //disable pwm_b
                    break;
                case BL_PWM_C:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 0, 1);  //disable pwm_c
                    break;
                case BL_PWM_D:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 1, 1);  //disable pwm_d
                    break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                case BL_PWM_E:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 0, 1);  //disable pwm_c
                    break;
                case BL_PWM_F:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 1, 1);  //disable pwm_d
                    break;
#endif
                default:
                    break;
            }
            break;
        case BL_CTL_PWM_COMBO:
            switch (bconf->combo_high_port) {
                case BL_PWM_A:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 0, 1);  //disable pwm_a
                    break;
                case BL_PWM_B:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 1, 1);  //disable pwm_b
                    break;
                case BL_PWM_C:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 0, 1);  //disable pwm_c
                    break;
                case BL_PWM_D:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 1, 1);  //disable pwm_d
                    break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                case BL_PWM_E:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 0, 1);  //disable pwm_c
                    break;
                case BL_PWM_F:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 1, 1);  //disable pwm_d
                    break;
#endif
                default:
                    break;
            }
            switch (bconf->combo_low_port) {
                case BL_PWM_A:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 0, 1);  //disable pwm_a
                    break;
                case BL_PWM_B:
                    aml_set_reg32_bits(P_PWM_MISC_REG_AB, 0, 1, 1);  //disable pwm_b
                    break;
                case BL_PWM_C:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 0, 1);  //disable pwm_c
                    break;
                case BL_PWM_D:
                    aml_set_reg32_bits(P_PWM_MISC_REG_CD, 0, 1, 1);  //disable pwm_d
                    break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                case BL_PWM_E:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 0, 1);  //disable pwm_c
                    break;
                case BL_PWM_F:
                    aml_set_reg32_bits(P_PWM_MISC_REG_EF, 0, 1, 1);  //disable pwm_d
                    break;
#endif
                default:
                    break;
            }
            break;
#ifdef CONFIG_AML_BACKLIGHT_EXTERN
        case BL_CTL_EXTERN:
            bl_extern_driver = aml_bl_extern_get_driver();
            if (bl_extern_driver == NULL) {
                printk("no bl_extern driver\n");
            }
            else {
                if (bl_extern_driver->power_off) {
                    ret = bl_extern_driver->power_off();
                    if (ret)
                        printk("[bl_extern] power off error\n");
                }
                else {
                    printk("[bl_extern] power off is null\n");
                }
            }
            break;
#endif
        default:
            break;
    }
    amlbl->state &= ~BL_STATE_REAL_ON; //bl_real_status = 0;
    printk("backlight power off\n");
    mutex_unlock(&bl_power_mutex);
}

static DEFINE_MUTEX(bl_level_mutex);
static void set_backlight_level(unsigned level)
{
    struct lcd_bl_config_s *bconf;
    unsigned pwm_hi = 0, pwm_lo = 0;
    struct aml_bl_extern_driver_t *bl_extern_driver;
    int ret;

    if (amlbl == NULL) {
        BL_PR("no bl data\n");
        return;
    } else {
        bconf = amlbl->bconf;
    }
    mutex_lock(&bl_level_mutex);

    DPRINT("set_backlight_level: %u, last level: %u, state: 0x%x\n", level, amlbl->level, amlbl->state);
    level = (level > bconf->level_max ? bconf->level_max : (level < bconf->level_min ? (level < BL_LEVEL_OFF ? 0 : bconf->level_min) : level));
    amlbl->level = level;

    if (amlbl->level == 0) {
        if (amlbl->state & BL_STATE_REAL_ON)//(bl_real_status == 1)
            bl_power_off(DRV_BL_FLAG);
    }
    else {
        //mapping
        if (level > bconf->level_mid)
            level = ((level - bconf->level_mid) * (bconf->level_max - bconf->level_mid_mapping)) / (bconf->level_max - bconf->level_mid) + bconf->level_mid_mapping;
        else
            level = ((level - bconf->level_min) * (bconf->level_mid_mapping - bconf->level_min)) / (bconf->level_mid - bconf->level_min) + bconf->level_min;
        DPRINT("level mapping=%u\n", level);

        switch (bconf->method) {
            case BL_CTL_GPIO:
                level = bconf->dim_min - ((level - bconf->level_min) * (bconf->dim_min - bconf->dim_max)) / (bconf->level_max - bconf->level_min);
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
                aml_set_reg32_bits(P_LED_PWM_REG0, level, 0, 4);
#endif
                break;
            case BL_CTL_PWM_NEGATIVE:
            case BL_CTL_PWM_POSITIVE:
                level = (bconf->pwm_max - bconf->pwm_min) * (level - bconf->level_min) / (bconf->level_max - bconf->level_min) + bconf->pwm_min;
                if (bconf->method == BL_CTL_PWM_NEGATIVE) {
                    pwm_hi = bconf->pwm_cnt - level;
                    pwm_lo = level;
                }
                else {
                    pwm_hi = level;
                    pwm_lo = bconf->pwm_cnt - level;
                }
                switch (bconf->pwm_port) {
                    case BL_PWM_A:
                        aml_write_reg32(P_PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case BL_PWM_B:
                        aml_write_reg32(P_PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case BL_PWM_C:
                        aml_write_reg32(P_PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case BL_PWM_D:
                        aml_write_reg32(P_PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                        break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                    case BL_PWM_E:
                        aml_write_reg32(P_PWM_PWM_E, (pwm_hi << 16) | (pwm_lo));
                        break;
                    case BL_PWM_F:
                        aml_write_reg32(P_PWM_PWM_F, (pwm_hi << 16) | (pwm_lo));
                        break;
#endif
                    default:
                        break;
                }
                break;
            case BL_CTL_PWM_COMBO:
                if (level >= bconf->combo_level_switch) {
                    //pre_set combo_low duty max
                    if (bconf->combo_low_method == BL_CTL_PWM_NEGATIVE) {
                        pwm_hi = bconf->combo_low_cnt - bconf->combo_low_duty_max;
                        pwm_lo = bconf->combo_low_duty_max;
                    }
                    else {
                        pwm_hi = bconf->combo_low_duty_max;
                        pwm_lo = bconf->combo_low_cnt - bconf->combo_low_duty_max;
                    }
                    switch (bconf->combo_low_port) {
                        case BL_PWM_A:
                            aml_write_reg32(P_PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_B:
                            aml_write_reg32(P_PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_C:
                            aml_write_reg32(P_PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_D:
                            aml_write_reg32(P_PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                            break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                        case BL_PWM_E:
                            aml_write_reg32(P_PWM_PWM_E, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_F:
                            aml_write_reg32(P_PWM_PWM_F, (pwm_hi << 16) | (pwm_lo));
                            break;
#endif
                        default:
                            break;
                    }

                    //set combo_high duty
                    level = (bconf->combo_high_duty_max - bconf->combo_high_duty_min) * (level - bconf->combo_level_switch) / (bconf->level_max - bconf->combo_level_switch) + bconf->combo_high_duty_min;
                    if (bconf->combo_high_method == BL_CTL_PWM_NEGATIVE) {
                        pwm_hi = bconf->combo_high_cnt - level;
                        pwm_lo = level;
                    }
                    else {
                        pwm_hi = level;
                        pwm_lo = bconf->combo_high_cnt - level;
                    }
                    switch (bconf->combo_high_port) {
                        case BL_PWM_A:
                            aml_write_reg32(P_PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_B:
                            aml_write_reg32(P_PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_C:
                            aml_write_reg32(P_PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_D:
                            aml_write_reg32(P_PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                            break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                        case BL_PWM_E:
                            aml_write_reg32(P_PWM_PWM_E, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_F:
                            aml_write_reg32(P_PWM_PWM_F, (pwm_hi << 16) | (pwm_lo));
                            break;
#endif
                        default:
                            break;
                    }
                }
                else {
                    //pre_set combo_high duty min
                    if (bconf->combo_high_method == BL_CTL_PWM_NEGATIVE) {
                        pwm_hi = bconf->combo_high_cnt - bconf->combo_high_duty_min;
                        pwm_lo = bconf->combo_high_duty_min;
                    }
                    else {
                        pwm_hi = bconf->combo_high_duty_min;;
                        pwm_lo = bconf->combo_high_cnt - bconf->combo_high_duty_min;
                    }
                    switch (bconf->combo_high_port) {
                        case BL_PWM_A:
                            aml_write_reg32(P_PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_B:
                            aml_write_reg32(P_PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_C:
                            aml_write_reg32(P_PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_D:
                            aml_write_reg32(P_PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                            break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                        case BL_PWM_E:
                            aml_write_reg32(P_PWM_PWM_E, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_F:
                            aml_write_reg32(P_PWM_PWM_F, (pwm_hi << 16) | (pwm_lo));
                            break;
#endif
                        default:
                            break;
                    }

                    //set combo_low duty
                    level = (bconf->combo_low_duty_max - bconf->combo_low_duty_min) * (level - bconf->level_min) / (bconf->combo_level_switch - bconf->level_min) + bconf->combo_low_duty_min;
                    if (bconf->combo_low_method == BL_CTL_PWM_NEGATIVE) {
                        pwm_hi = bconf->combo_low_cnt - level;
                        pwm_lo = level;
                    }
                    else {
                        pwm_hi = level;
                        pwm_lo = bconf->combo_low_cnt - level;
                    }
                    switch (bconf->combo_low_port) {
                        case BL_PWM_A:
                            aml_write_reg32(P_PWM_PWM_A, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_B:
                            aml_write_reg32(P_PWM_PWM_B, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_C:
                            aml_write_reg32(P_PWM_PWM_C, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_D:
                            aml_write_reg32(P_PWM_PWM_D, (pwm_hi << 16) | (pwm_lo));
                            break;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                        case BL_PWM_E:
                            aml_write_reg32(P_PWM_PWM_E, (pwm_hi << 16) | (pwm_lo));
                            break;
                        case BL_PWM_F:
                            aml_write_reg32(P_PWM_PWM_F, (pwm_hi << 16) | (pwm_lo));
                            break;
#endif
                        default:
                            break;
                    }
                }
                break;
#ifdef CONFIG_AML_BACKLIGHT_EXTERN
            case BL_CTL_EXTERN:
                bl_extern_driver = aml_bl_extern_get_driver();
                if (bl_extern_driver == NULL) {
                    printk("no bl_extern driver\n");
                }
                else {
                    if (bl_extern_driver->set_level) {
                        ret = bl_extern_driver->set_level(level);
                        if (ret)
                            printk("[bl_extern] set_level error\n");
                    }
                    else {
                        printk("[bl_extern] set_level is null\n");
                    }
                }
                break;
#endif
            default:
                break;
        }
        if ((amlbl->state & BL_STATE_REAL_ON) == 0)//((bl_status == 3) && (bl_real_status == 0))
            bl_power_on(DRV_BL_FLAG);
    }
    mutex_unlock(&bl_level_mutex);
}

unsigned get_backlight_level(void)
{
    if (amlbl == NULL) {
        BL_PR("no bl data\n");
        return 0;
    }

    BL_PR("aml bl state: 0x%x\n", amlbl->state);
    return amlbl->level;
}

static int aml_bl_update_status(struct backlight_device *bd)
{
    int brightness = bd->props.brightness;

    if (brightness < 0)
        brightness = 0;
    else if (brightness > 255)
        brightness = 255;

    set_backlight_level(brightness);

    return 0;
}

static int aml_bl_get_brightness(struct backlight_device *bd)
{
    return get_backlight_level();
}

static const struct backlight_ops aml_bl_ops = {
    .get_brightness = aml_bl_get_brightness,
    .update_status  = aml_bl_update_status,
};

#ifdef CONFIG_OF
#define AMLOGIC_BL_DRV_DATA ((kernel_ulong_t)&bl_config)

static const struct of_device_id backlight_dt_match[] = {
    {
        .compatible = "amlogic,backlight",
        .data = (void *)AMLOGIC_BL_DRV_DATA
    },
    {},
};
#else
#define backlight_dt_match NULL
#endif

#ifdef CONFIG_OF
static inline struct lcd_bl_config_s *bl_get_driver_data(struct platform_device *pdev)
{
    const struct of_device_id *match;

    if(pdev->dev.of_node) {
        match = of_match_node(backlight_dt_match, pdev->dev.of_node);
        return (struct lcd_bl_config_s *)match->data;
    }
    return NULL;
}

static inline int _get_backlight_config(struct platform_device *pdev)
{
    int ret=0;
    int val;
    const char *str;
    unsigned int bl_para[3];
    unsigned pwm_freq=0, pwm_cnt, pwm_pre_div;
    int i;

    if (pdev->dev.of_node) {
        ret = of_property_read_u32_array(pdev->dev.of_node,"bl_level_default_uboot_kernel", &bl_para[0], 2);
        if(ret){
            printk("faild to get bl_level_default_uboot_kernel\n");
            bl_config.level_default = BL_LEVEL_DEFAULT;
        }
        else {
            bl_config.level_default = bl_para[1];
        }
        DPRINT("bl level default kernel=%u\n", bl_config.level_default);
        ret = of_property_read_u32_array(pdev->dev.of_node, "bl_level_middle_mapping", &bl_para[0], 2);
        if (ret) {
            printk("faild to get bl_level_middle_mapping!\n");
            bl_config.level_mid = BL_LEVEL_MID;
            bl_config.level_mid_mapping = BL_LEVEL_MID_MAPPED;
        }
        else {
            bl_config.level_mid = bl_para[0];
            bl_config.level_mid_mapping = bl_para[1];
        }
        DPRINT("bl level mid=%u, mid_mapping=%u\n", bl_config.level_mid, bl_config.level_mid_mapping);
        ret = of_property_read_u32_array(pdev->dev.of_node,"bl_level_max_min", &bl_para[0],2);
        if(ret){
            printk("faild to get bl_level_max_min\n");
            bl_config.level_min = BL_LEVEL_MIN;
            bl_config.level_max = BL_LEVEL_MAX;
        }
        else {
            bl_config.level_max = bl_para[0];
            bl_config.level_min = bl_para[1];
        }
        DPRINT("bl level max=%u, min=%u\n", bl_config.level_max, bl_config.level_min);

        ret = of_property_read_u32(pdev->dev.of_node, "bl_power_on_delay", &val);
        if (ret) {
            printk("faild to get bl_power_on_delay\n");
            bl_config.power_on_delay = 100;
        }
        else {
            val = val & 0xffff;
            bl_config.power_on_delay = (unsigned short)val;
        }
        DPRINT("bl power_on_delay: %ums\n", bl_config.power_on_delay);
        ret = of_property_read_u32(pdev->dev.of_node, "bl_ctrl_method", &val);
        if (ret) {
            printk("faild to get bl_ctrl_method\n");
            bl_config.method = BL_CTL_MAX;
        }
        else {
            val = (val >= BL_CTL_MAX) ? BL_CTL_MAX : val;
            bl_config.method = (unsigned char)val;
        }
        DPRINT("bl control_method: %s(%u)\n", bl_ctrl_method_table[bl_config.method], bl_config.method);

        if (bl_config.method == BL_CTL_GPIO) {
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_gpio_port_on_off", 0, &str);
            if (ret) {
                printk("faild to get bl_gpio_port_on_off!\n");
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
                str = "GPIOD_1";
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
                str = "GPIODV_28";
#endif
            }
            val = amlogic_gpio_name_map_num(str);
            if (val > 0) {
                ret = bl_gpio_request(val);
                if (ret) {
                    printk("faild to alloc bl gpio (%s)!\n", str);
                }
                bl_config.gpio = val;                
                DPRINT("bl gpio = %s(%d)\n", str, bl_config.gpio);
            }
            else {
                bl_config.gpio = -1;
            }
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_gpio_port_on_off", 1, &str);
            if (ret) {
                printk("faild to get bl_gpio_port_on!\n");
                bl_config.gpio_on = BL_GPIO_OUTPUT_HIGH;
            }
            else {
                if (strcmp(str, "2") == 0)
                    bl_config.gpio_on = BL_GPIO_INPUT;
                else if(strcmp(str, "0") == 0)
                    bl_config.gpio_on = BL_GPIO_OUTPUT_LOW;
                else
                    bl_config.gpio_on = BL_GPIO_OUTPUT_HIGH;
            }
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_gpio_port_on_off", 2, &str);
            if (ret) {
                printk("faild to get bl_gpio_port_off!\n");
                bl_config.gpio_off = BL_GPIO_OUTPUT_LOW;
            }
            else {
                if (strcmp(str, "2") == 0)
                    bl_config.gpio_off = BL_GPIO_INPUT;
                else if(strcmp(str, "1") == 0)
                    bl_config.gpio_off = BL_GPIO_OUTPUT_HIGH;
                else
                    bl_config.gpio_off = BL_GPIO_OUTPUT_LOW;
            }
            DPRINT("bl gpio_on=%u, bl gpio_off=%u\n", bl_config.gpio_on, bl_config.gpio_off);
            ret = of_property_read_u32_array(pdev->dev.of_node,"bl_gpio_dim_max_min",&bl_para[0],2);
            if (ret) {
                printk("faild to get bl_gpio_dim_max_min\n");
                bl_config.dim_max = 0x0;
                bl_config.dim_min = 0xf;
            }
            else {
                bl_config.dim_max = bl_para[0];
                bl_config.dim_min = bl_para[1];
            }
            DPRINT("bl dim max=%u, min=%u\n", bl_config.dim_max, bl_config.dim_min);
        }
        else if ((bl_config.method == BL_CTL_PWM_NEGATIVE) || (bl_config.method == BL_CTL_PWM_POSITIVE)) {
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_port_gpio_used", 1, &str);
            if (ret) {
                printk("faild to get bl_pwm_port_gpio_used!\n");
                bl_config.pwm_gpio_used = 0;
            }
            else {
                if (strncmp(str, "1", 1) == 0)
                    bl_config.pwm_gpio_used = 1;
                else
                    bl_config.pwm_gpio_used = 0;
                DPRINT("bl_pwm gpio_used: %u\n", bl_config.pwm_gpio_used);
            }
            if (bl_config.pwm_gpio_used == 1) {
                ret = of_property_read_string(pdev->dev.of_node, "bl_gpio_port_on_off", &str);
                if (ret) {
                    printk("faild to get bl_gpio_port_on_off!\n");
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
                    str = "GPIOD_1";
#elif ((MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8) || (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B))
                    str = "GPIODV_28";
#endif
                }
                val = amlogic_gpio_name_map_num(str);
                if (val > 0) {
                    ret = bl_gpio_request(val);
                    if (ret) {
                      printk("faild to alloc bl gpio (%s)!\n", str);
                    }
                    bl_config.gpio = val;
                    DPRINT("bl gpio = %s(%d)\n", str, bl_config.gpio);
                }
                else {
                    bl_config.gpio = -1;
                }
                          ret = of_property_read_string_index(pdev->dev.of_node, "bl_gpio_port_on_off", 1, &str);
            if (ret) {
                printk("faild to get bl_gpio_port_on!\n");
                bl_config.gpio_on = BL_GPIO_OUTPUT_HIGH;
            }
            else {
                if (strcmp(str, "2") == 0)
                    bl_config.gpio_on = BL_GPIO_INPUT;
                else if(strcmp(str, "0") == 0)
                    bl_config.gpio_on = BL_GPIO_OUTPUT_LOW;
                else
                    bl_config.gpio_on = BL_GPIO_OUTPUT_HIGH;
            }
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_gpio_port_on_off", 2, &str);
            if (ret) {
                printk("faild to get bl_gpio_port_off!\n");
                bl_config.gpio_off = BL_GPIO_OUTPUT_LOW;
            }
            else {
                if (strcmp(str, "2") == 0)
                    bl_config.gpio_off = BL_GPIO_INPUT;
                else if(strcmp(str, "1") == 0)
                    bl_config.gpio_off = BL_GPIO_OUTPUT_HIGH;
                else
                    bl_config.gpio_off = BL_GPIO_OUTPUT_LOW;
            }
            DPRINT("gpio_on=%u, gpio_off=%u\n", bl_config.gpio_on, bl_config.gpio_off);
          }
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_port_gpio_used", 0, &str);
            if (ret) {
                printk("faild to get bl_pwm_port_gpio_used!\n");
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
                bl_config.pwm_port = BL_PWM_D;
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
                bl_config.pwm_port = BL_PWM_C;
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8B)
                bl_config.pwm_port = BL_PWM_MAX;
#endif
            }
            else {
                if (strcmp(str, "PWM_A") == 0)
                    bl_config.pwm_port = BL_PWM_A;
                else if (strcmp(str, "PWM_B") == 0)
                    bl_config.pwm_port = BL_PWM_B;
                else if (strcmp(str, "PWM_C") == 0)
                    bl_config.pwm_port = BL_PWM_C;
                else if (strcmp(str, "PWM_D") == 0)
                    bl_config.pwm_port = BL_PWM_D;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                else if (strcmp(str, "PWM_E") == 0)
                    bl_config.pwm_port = BL_PWM_E;
                else if (strcmp(str, "PWM_F") == 0)
                    bl_config.pwm_port = BL_PWM_F;
#endif
                else
                    bl_config.pwm_port = BL_PWM_MAX;
                DPRINT("bl pwm_port: %s(%u)\n", str, bl_config.pwm_port);
            }
            ret = of_property_read_u32(pdev->dev.of_node,"bl_pwm_freq",&val);
            if (ret) {
#if (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON6)
                pwm_freq = 1000;
#elif (MESON_CPU_TYPE == MESON_CPU_TYPE_MESON8)
                pwm_freq = 300000;
#endif
                printk("faild to get bl_pwm_freq, default set to %u\n", pwm_freq);
            }
            else {
                pwm_freq = ((val >= (FIN_FREQ * 500)) ? (FIN_FREQ * 500) : val);
            }
            for (i=0; i<0x7f; i++) {
                pwm_pre_div = i;
                pwm_cnt = FIN_FREQ * 1000 / (pwm_freq * (pwm_pre_div + 1)) - 2;
                if (pwm_cnt <= 0xffff)
                    break;
            }
            bl_config.pwm_cnt = pwm_cnt;
            bl_config.pwm_pre_div = pwm_pre_div;
            DPRINT("bl pwm_frequency=%u, cnt=%u, div=%u\n", pwm_freq, bl_config.pwm_cnt, bl_config.pwm_pre_div);
            ret = of_property_read_u32_array(pdev->dev.of_node,"bl_pwm_duty_max_min",&bl_para[0],2);
            if (ret) {
                printk("faild to get bl_pwm_duty_max_min\n");
                bl_para[0] = 100;
                bl_para[1] = 20;
            }
            bl_config.pwm_max = (bl_config.pwm_cnt * bl_para[0] / 100);
            bl_config.pwm_min = (bl_config.pwm_cnt * bl_para[1] / 100);
            DPRINT("bl pwm_duty max=%u%%, min=%u%%\n", bl_para[0], bl_para[1]);
        }
        else if (bl_config.method == BL_CTL_PWM_COMBO) {
            ret = of_property_read_u32(pdev->dev.of_node,"bl_pwm_combo_high_low_level_switch",&val);
            if (ret) {
                printk("faild to get bl_pwm_combo_high_low_level_switch\n");
                val = bl_config.level_mid;
            }
            if (val > bl_config.level_mid)
                val = ((val - bl_config.level_mid) * (bl_config.level_max - bl_config.level_mid_mapping)) / (bl_config.level_max - bl_config.level_mid) + bl_config.level_mid_mapping;
            else
                val = ((val - bl_config.level_min) * (bl_config.level_mid_mapping - bl_config.level_min)) / (bl_config.level_mid - bl_config.level_min) + bl_config.level_min;
            bl_config.combo_level_switch = val;
            DPRINT("bl pwm_combo level switch =%u\n", bl_config.combo_level_switch);
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_combo_high_port_method", 0, &str);
            if (ret) {
                printk("faild to get bl_pwm_combo_high_port_method!\n");
                str = "PWM_C";
                bl_config.combo_high_port = BL_PWM_C;
            }
            else {
                if (strcmp(str, "PWM_A") == 0)
                    bl_config.combo_high_port = BL_PWM_A;
                else if (strcmp(str, "PWM_B") == 0)
                    bl_config.combo_high_port = BL_PWM_B;
                else if (strcmp(str, "PWM_C") == 0)
                    bl_config.combo_high_port = BL_PWM_C;
                else if (strcmp(str, "PWM_D") == 0)
                    bl_config.combo_high_port = BL_PWM_D;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                else if (strcmp(str, "PWM_E") == 0)
                    bl_config.pwm_port = BL_PWM_E;
                else if (strcmp(str, "PWM_F") == 0)
                    bl_config.pwm_port = BL_PWM_F;
#endif
                else
                    bl_config.combo_high_port = BL_PWM_MAX;
            }
            DPRINT("bl pwm_combo high port: %s(%u)\n", str, bl_config.combo_high_port);
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_combo_high_port_method", 1, &str);
            if (ret) {
                printk("faild to get bl_pwm_combo_high_port_method!\n");
                str = "1";
                bl_config.combo_high_method = BL_CTL_PWM_NEGATIVE;
            }
            else {
                if (strncmp(str, "1", 1) == 0)
                    bl_config.combo_high_method = BL_CTL_PWM_NEGATIVE;
                else
                    bl_config.combo_high_method = BL_CTL_PWM_POSITIVE;
            }
            DPRINT("bl pwm_combo high method: %s(%u)\n", bl_ctrl_method_table[bl_config.combo_high_method], bl_config.combo_high_method);
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_combo_low_port_method", 0, &str);
            if (ret) {
                printk("faild to get bl_pwm_combo_low_port_method!\n");
                str = "PWM_D";
                bl_config.combo_low_port = BL_PWM_D;
            }
            else {
                if (strcmp(str, "PWM_A") == 0)
                    bl_config.combo_low_port = BL_PWM_A;
                else if (strcmp(str, "PWM_B") == 0)
                    bl_config.combo_low_port = BL_PWM_B;
                else if (strcmp(str, "PWM_C") == 0)
                    bl_config.combo_low_port = BL_PWM_C;
                else if (strcmp(str, "PWM_D") == 0)
                    bl_config.combo_low_port = BL_PWM_D;
#if (MESON_CPU_TYPE >= MESON_CPU_TYPE_MESON8)
                else if (strcmp(str, "PWM_E") == 0)
                    bl_config.pwm_port = BL_PWM_E;
                else if (strcmp(str, "PWM_F") == 0)
                    bl_config.pwm_port = BL_PWM_F;
#endif
                else
                    bl_config.combo_low_port = BL_PWM_MAX;
            }
            DPRINT("bl pwm_combo high port: %s(%u)\n", str, bl_config.combo_low_port);
            ret = of_property_read_string_index(pdev->dev.of_node, "bl_pwm_combo_low_port_method", 1, &str);
            if (ret) {
                printk("faild to get bl_pwm_combo_low_port_method!\n");
                str = "1";
                bl_config.combo_low_method = BL_CTL_PWM_NEGATIVE;
            }
            else {
                if (strncmp(str, "1", 1) == 0)
                    bl_config.combo_low_method = BL_CTL_PWM_NEGATIVE;
                else
                    bl_config.combo_low_method = BL_CTL_PWM_POSITIVE;
            }
            DPRINT("bl pwm_combo low method: %s(%u)\n", bl_ctrl_method_table[bl_config.combo_low_method], bl_config.combo_low_method);
            ret = of_property_read_u32_array(pdev->dev.of_node,"bl_pwm_combo_high_freq_duty_max_min",&bl_para[0],3);
            if (ret) {
                printk("faild to get bl_pwm_combo_high_freq_duty_max_min\n");
                bl_para[0] = 300000;  //freq=300k
                bl_para[1] = 100;
                bl_para[2] = 50;
            }
            pwm_freq = ((bl_para[0] >= (FIN_FREQ * 500)) ? (FIN_FREQ * 500) : bl_para[0]);
            for (i=0; i<0x7f; i++) {
                pwm_pre_div = i;
                pwm_cnt = FIN_FREQ * 1000 / (pwm_freq * (pwm_pre_div + 1)) - 2;
                if (pwm_cnt <= 0xffff)
                    break;
            }
            bl_config.combo_high_cnt = pwm_cnt;
            bl_config.combo_high_pre_div = pwm_pre_div;
            bl_config.combo_high_duty_max = (bl_config.combo_high_cnt * bl_para[1] / 100);
            bl_config.combo_high_duty_min = (bl_config.combo_high_cnt * bl_para[2] / 100);
            DPRINT("bl pwm_combo high freq=%uHz, duty_max=%u%%, duty_min=%u%%\n", pwm_freq, bl_para[1], bl_para[2]);
            ret = of_property_read_u32_array(pdev->dev.of_node,"bl_pwm_combo_low_freq_duty_max_min",&bl_para[0],3);
            if (ret) {
                printk("faild to get bl_pwm_combo_low_freq_duty_max_min\n");
                bl_para[0] = 1000;    //freq=1k
                bl_para[1] = 100;
                bl_para[2] = 50;
            }
            pwm_freq = ((bl_para[0] >= (FIN_FREQ * 500)) ? (FIN_FREQ * 500) : bl_para[0]);
            for (i=0; i<0x7f; i++) {
                pwm_pre_div = i;
                pwm_cnt = FIN_FREQ * 1000 / (pwm_freq * (pwm_pre_div + 1)) - 2;
                if (pwm_cnt <= 0xffff)
                    break;
            }
            bl_config.combo_low_cnt = pwm_cnt;
            bl_config.combo_low_pre_div = pwm_pre_div;
            bl_config.combo_low_duty_max = (bl_config.combo_low_cnt * bl_para[1] / 100);
            bl_config.combo_low_duty_min = (bl_config.combo_low_cnt * bl_para[2] / 100);
            DPRINT("bl pwm_combo low freq=%uHz, duty_max=%u%%, duty_min=%u%%\n", pwm_freq, bl_para[1], bl_para[2]);
        }

        //pinmux
        bl_config.p = devm_pinctrl_get(&pdev->dev);
        if (IS_ERR(bl_config.p))
            printk("get backlight pinmux error.\n");
    }
    return ret;
}
#endif

static int aml_bl_probe(struct platform_device *pdev)
{
    struct backlight_properties props;
    struct lcd_bl_config_s *pdata;
    struct backlight_device *bldev;
    int retval;

    DTRACE();

    amlbl = kzalloc(sizeof(struct aml_bl_s), GFP_KERNEL);
    if (!amlbl)
    {
        printk(KERN_ERR "%s() kzalloc error\n", __func__);
        return -ENOMEM;
    }

    //amlbl->pdev = pdev;

#ifdef CONFIG_OF
    _get_backlight_config(pdev);
    pdata = bl_get_driver_data(pdev);
#else
    pdata = pdev->dev.platform_data;
#endif
    if (!pdata) {
        printk(KERN_ERR "%s() missing platform data\n", __func__);
        retval = -ENODEV;
        goto err;
    }

    amlbl->bconf = pdata;

    memset(&props, 0, sizeof(struct backlight_properties));
    props.type = BACKLIGHT_RAW;
    props.power = FB_BLANK_UNBLANK; /* full on */
    props.max_brightness = (pdata->level_max > 0 ? pdata->level_max : BL_LEVEL_MAX);
    props.brightness = (pdata->level_default > 0 ? pdata->level_default : BL_LEVEL_DEFAULT);

    bldev = backlight_device_register("aml-bl", &pdev->dev, amlbl, &aml_bl_ops, &props);
    if (IS_ERR(bldev)) {
        printk(KERN_ERR "failed to register backlight\n");
        retval = PTR_ERR(bldev);
        goto err;
    }

    amlbl->bldev = bldev;
    //platform_set_drvdata(pdev, amlbl);

    //init workqueue
    INIT_DELAYED_WORK(&amlbl->bl_delayed_work, bl_delayd_on);
    //amlbl->workqueue = create_singlethread_workqueue("bl_power_on_queue");
    amlbl->workqueue = create_workqueue("bl_power_on_queue");
    if (amlbl->workqueue == NULL) {
        printk("can't create bl work queue\n");
    }

    amlbl->state = (BL_STATE_LCD_ON | BL_STATE_BL_ON | BL_STATE_REAL_ON);
    aml_bl_update_status(amlbl->bldev);

    printk("aml bl probe OK\n");
    return 0;

err:
    kfree(amlbl);
    return retval;
}

static int __exit aml_bl_remove(struct platform_device *pdev)
{
    //struct aml_bl_s *amlbl = platform_get_drvdata(pdev);

    DTRACE();

    if (amlbl->workqueue)
        destroy_workqueue(amlbl->workqueue);

    backlight_device_unregister(amlbl->bldev);
    platform_set_drvdata(pdev, NULL);
    kfree(amlbl);

    return 0;
}

static struct platform_driver aml_bl_driver = {
    .driver = {
        .name = "aml-bl",
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = backlight_dt_match,
#endif
    },
    .probe = aml_bl_probe,
    .remove = __exit_p(aml_bl_remove),
};

static int __init aml_bl_init(void)
{
    DTRACE();
    if (platform_driver_register(&aml_bl_driver)) {
        printk("failed to register bl driver module\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit aml_bl_exit(void)
{
    DTRACE();
    platform_driver_unregister(&aml_bl_driver);
}

module_init(aml_bl_init);
module_exit(aml_bl_exit);

MODULE_DESCRIPTION("Meson Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Amlogic, Inc.");
