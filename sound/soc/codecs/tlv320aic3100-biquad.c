/*
 * linux/sound/soc/codecs/tlv320aic3100-biquad.c
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1 	 Adatptive filter support    		       21-Apr-2010
 *
 *          The Biquad programming support is added to codec DAC3100.
 *
 * Rev 0.2      Updated the file with EQ Mode configuration
 *							       26-Jul-2010
 * Rev 0.3      Added new routines to parse the EQ Values Array
 *              minidsp_D_reg_values during initialization time and also
 *              Update the same at run-time depending on the Speaker connection
 *                                                             30-Jul-2010
 *
 * Rev 0.4      Updated the logic of dac3100_parse_biquad_array() and
 * 		dac3100_update_biquad_array() to utilize the Adaptive Filtering
 *		Mode. Hence there is no need to re-program the EQ Values at run-time.
 *							       02-Aug-2010
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/control.h>

#include "tlv320aic3100.h"
#include "tlv320aic3100-biquad.h"
#include "tlv320aic3100_eq_coeff.h" 		/* Eq Coefficient Header file */

/* enable debug prints in the driver */
//#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define dprintk(x...) 	printk(x)
#else
#define dprintk(x...)
#endif

#ifdef CONFIG_ADAPTIVE_FILTER

/*
*****************************************************************************
* Extern Declarations
*****************************************************************************
*/
extern int dac3100_change_page (struct snd_soc_codec *codec, u8 new_page);

extern unsigned int aic3100_read(struct snd_soc_codec *codec, unsigned int reg);


/*
*****************************************************************************
* Global Variables
*****************************************************************************
*/
int aic3100_eq_select = AIC3100_HP_EQ_SELECT;

/* aic3100_biquad_array
 * There are two Pages related to the biquad Array. Buffer A and B and hence
 * we will maintain an array of the aic3100_parse_data structure
 */
aic3100_parse_data  aic3100_biquad_array[ADAPTIVE_MAX_CONTROLS];


/************************** Adaptive filtering section **********************/

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_info_biquad_filter
 *
 * Purpose  : info routine for adaptive filter control amixer kcontrols
 *----------------------------------------------------------------------------
 */
static int
__new_control_info_biquad_adaptive (struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_get_biquad_adaptive
 *
 * Purpose  : get routine for  adaptive filter control amixer kcontrols,
 *            reads to user if adaptive filtering is enabled or disabled.
 *----------------------------------------------------------------------------
 */
static int
__new_control_get_biquad_adaptive (struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip (kcontrol);
	struct i2c_client *i2c;
	char data[2];
	int ret = 0;

	u8 page = (kcontrol->private_value) & AIC3100_8BITS_MASK;
	u8 reg = (kcontrol->private_value >> 8) & AIC3100_8BITS_MASK;
	u8 rmask = (kcontrol->private_value >> 16) & AIC3100_8BITS_MASK;

	i2c = codec->control_data;

	dprintk ("page %d, reg %d, mask 0x%x\n", page, reg, rmask);

	/* Read the register value */
	aic3100_change_page (codec, page);

	/* write register addr to read */
	data[0] = reg;

	if (i2c_master_send (i2c, data, 1) != 1)
	{
		printk ("Can not write register address\n");
		ret = -1;
		goto revert;
	}
	/* read the codec/minidsp registers */
	if (i2c_master_recv (i2c, data, 1) != 1)
	{
		printk ("Can not read codec registers\n");
		ret = -1;
		goto revert;
	}

	dprintk ("read: 0x%x\n", data[0]);

	/* return the read status to the user */
	if (data[0] & rmask)
	{
		ucontrol->value.integer.value[0] = 1;
	}
	else
	{
		ucontrol->value.integer.value[0] = 0;
	}

revert:
	/* put page back to zero */
	aic3100_change_page (codec, 0);
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : __new_control_put_biquad_adaptive
 *
 * Purpose  : put routine for adaptive filter controls amixer kcontrols.
 * 			  This routine will enable/disable adaptive filtering.
 *----------------------------------------------------------------------------
 */
static int
__new_control_put_biquad_adaptive (struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_kcontrol_chip (kcontrol);
	int user_value = ucontrol->value.integer.value[0];
	struct i2c_client *i2c;
	char data[2];
	int ret = 0;

	u8 page = (kcontrol->private_value) & AIC3100_8BITS_MASK;
	u8 reg = (kcontrol->private_value >> 8) & AIC3100_8BITS_MASK;
	u8 wmask = (kcontrol->private_value >> 24) & AIC3100_8BITS_MASK;

	i2c = codec->control_data;

	dprintk ("page %d, reg %d, mask 0x%x, user_value %d\n",
		 page, reg, wmask, user_value);

	/* Program the register value */
	aic3100_change_page (codec, page);

	/* read register addr to read */
	data[0] = reg;

	if (i2c_master_send (i2c, data, 1) != 1)
	{
		printk ("Can not write register address\n");
		ret = -1;
		goto revert;
	}
	/* read the codec/minidsp registers */
	if (i2c_master_recv (i2c, data, 1) != 1)
	{
		printk ("Can not read codec registers\n");
		ret = -1;
		goto revert;
	}

	dprintk ("read: 0x%x\n", data[0]);

	/* set the bitmask and update the register */
	if (user_value == 0)
	{
		data[1] = (data[0]) & (~wmask);
	}
	else
	{
		data[1] = (data[0]) | wmask;
	}
	data[0] = reg;

	if (i2c_master_send (i2c, data, 2) != 2)
	{
		dprintk ("Can not write register address\n");
		ret = -1;
	}

revert:
	/* put page back to zero */
	aic3100_change_page (codec, 0);
	return ret;
}

#define SOC_ADAPTIVE_CTL_AIC3100(xname, page, reg, read_mask, write_mask) \
	{   .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
			.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,	\
			.info = __new_control_info_biquad_adaptive,	\
			.get = __new_control_get_biquad_adaptive, 	\
			.put = __new_control_put_biquad_adaptive,	\
			.count = 0,					\
			.private_value = (page) | (reg << 8) |		\
			( read_mask << 16) | (write_mask << 24)		\
			}

/* Adaptive filtering control and buffer swap  mixer kcontrols */
static struct snd_kcontrol_new snd_adaptive_controls[] = {
	SOC_ADAPTIVE_CTL_AIC3100 (FILT_CTL_NAME_DAC, BUFFER_PAGE_DAC, 0x1, 0x4,
				  0x4),
	SOC_ADAPTIVE_CTL_AIC3100 (COEFF_CTL_NAME_DAC, BUFFER_PAGE_DAC, 0x1, 0x2,
				  0x1),
};

/*
 *----------------------------------------------------------------------------
 * Function : biquad_adaptive_filter_mixer_controls
 *
 * Purpose  : registers adaptive filter mixer kcontrols
 *----------------------------------------------------------------------------
 */
static int
biquad_adaptive_filter_mixer_controls (struct snd_soc_codec *codec)
{
	int err = 0;

	err = snd_soc_add_controls(codec, snd_adaptive_controls,
			     ARRAY_SIZE(snd_adaptive_controls));
	dprintk("Adding controls\n",);
	return err;
}

/***************************************************************************
 * Function: aic3100_add_biquad_controls
 *
 * Purpose: Adds the bi-quad coefficient controls using the snd_ctl_add()
 *          This routine internally invokes the biquad_adaptive_filter_mixer_controls
 *          API for this purpose.
 *
 *       int - 0 on success and -1 on failure
 */
void
aic3100_add_biquads_controls (struct snd_soc_codec *codec)
{

	if (biquad_adaptive_filter_mixer_controls (codec))
	{
		printk ("Biquad Adaptive filter mixer control registration failed\n");
	}
}



/*************************AIC3100 EQ Selection *******************************/

/***************************************************************************
 * Function: aic3100_get_EQ_Select
 *
 * Purpose:  Add amixer kcontrols for AIC3100 EQ selection controls,
 *
 *       int - 0 on success and -1 on failure
 */
static int
aic3100_get_EQ_Select (struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = aic3100_eq_select;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic3100_put_EQ_Select
 *
 * Purpose  : put routine for amixer kcontrols, write user values to registers
 *            values. Used for for AIC3100 'MUX control' amixer controls.
 *----------------------------------------------------------------------------
 */
static int
aic3100_put_EQ_Select (struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	u8 data[MUX_CTRL_REG_SIZE + 1];
	struct snd_soc_codec *codec = snd_kcontrol_chip (kcontrol);
	static int index = -1;
	int user_value = ucontrol->value.integer.value[0];
	/*struct i2c_client *i2c = codec->control_data;*/
	u8 value[2];
	u8 page;
	int ret_val = -1;

	/* Enable the below line only when reading the I2C Transactions */
	/*u8 read_data[10]; */


	printk ("Change EQ mode to %s\n",(user_value == 0)?"SPK EQ":((user_value == 1)?"HP EQ":"No EQ"));

	/* If the control has not been exercised so far, index remains at -1. Check for this condition */
	if( index == -1) {
		for (index = 0; index < ARRAY_SIZE (MUX_aic3100_controls); index++) {
			if (strstr ("Stereo_Mux_1", MUX_aic3100_control_names[index]))
				break;
		}
		if( index == ARRAY_SIZE (MUX_aic3100_controls)) {
			index = -1;
			return (ret_val);
		}
	}

	page = MUX_aic3100_controls[index].control_page;

	user_value ++;

	if (user_value < AIC3100_SPK_EQ_SELECT || user_value > AIC3100_HP_EQ_SELECT)
		return (ret_val);

	if (index < ARRAY_SIZE (MUX_aic3100_controls))
	{
		dprintk ("Index %d Changing to Page %d\r\n", index,
			 MUX_aic3100_controls[index].control_page);

		aic3100_change_page (codec, MUX_aic3100_controls[index].control_page);

		data[1] = (u8) ((user_value >> 8) & AIC3100_8BITS_MASK);
		data[2] = (u8) ((user_value) & AIC3100_8BITS_MASK);

		/* start register address */
		data[0] = MUX_aic3100_controls[index].control_base;

		dprintk ("Writing %d %d %d \r\n", data[0], data[1], data[2]);

		ret_val = codec->hw_write (codec->control_data, &data[0], MUX_CTRL_REG_SIZE + 1);

		if (ret_val != MUX_CTRL_REG_SIZE + 1) {
			printk ("i2c_master_send transfer failed\n");
		}
		else {
			/* store the current level */
			aic3100_eq_select = ucontrol->value.integer.value[0];
			ret_val = 0;
			/* Enable adaptive filtering for ADC/DAC */

		}

		/* Following block of code for testing the previous I2C WRITE Transaction.
		 * Need not enable them in the final release.
		 */
		/*i2c_master_send (i2c, &data[0], 1);

		  i2c_master_recv(i2c, &read_data[0], 2);

		  printk("I2C Read Values are %d %d\r\n", read_data[0], read_data[1]);    */

		/* Perform a BUFFER SWAP Command. Check if we are currently not in Page 8,
		 * if so, swap to Page 8 first
		 */
		aic3100_change_page (codec, 8);

		value[0] = 1;

		if (codec->hw_write (codec->control_data, &value[0], 1) != 1) {
			dprintk ("Can not write register address\n");
		}
		/* Read the Value of the Page 8 Register 1 which controls the Adaptive Switching Mode */
		if (i2c_master_recv(codec->control_data, &value[0], 1) != 1) {
			printk ("Can not read codec registers\n");
		}

		/* Write the Register bit updates for Adaptive Filtering */
		value[1] = value[0] | 1;
		value[0] = 1;

		if (codec->hw_write (codec->control_data, &value[0], 2) != 2) {
			printk ("Can not write register address\n");
		}

		/* After updating the Adaptive Filtering, write the coefficients once again so that
		 * it gets updated in the other half
		 */
		aic3100_change_page (codec, MUX_aic3100_controls[index].control_page);
		ret_val = codec->hw_write (codec->control_data, &data[0], MUX_CTRL_REG_SIZE + 1);
	}

	aic3100_change_page (codec, 0);
	return (ret_val);
}

/**********************************************************************
 * Global Variables related to the EQ MIXER Controls
 **********************************************************************
 */
static struct snd_kcontrol_new snd_mode_controls;

static const char *aic3100_EQ_Select_Str[] = {"Speaker EQ", "Headphone EQ"};

static const struct soc_enum miniDSP_EQ_Select_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(aic3100_EQ_Select_Str), aic3100_EQ_Select_Str),
};

/***************************************************************************
 * Function: aic3100_EQ_Select_mixer_controls
 *
 * Purpose: Configures the AMIXER Controls for the miniDSP run-time configuration.
 * 	    This routine configures the function pointers for the get put and the info
 *          members of the snd_mode_controls and calls the snd_ctl_add routine
 *          to register this with the ALSA Library.
 *
 *       int - 0 on success and -1 on failure
 */
static int
aic3100_EQ_Select_mixer_controls (struct snd_soc_codec *codec)
{
	int err;

	snd_mode_controls.name = "AIC3100 EQ Selection";
	snd_mode_controls.iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	snd_mode_controls.access = SNDRV_CTL_ELEM_ACCESS_READWRITE;
	snd_mode_controls.info = snd_soc_info_enum_ext;
	snd_mode_controls.get = aic3100_get_EQ_Select;
	snd_mode_controls.put = aic3100_put_EQ_Select;
	snd_mode_controls.private_value = (unsigned long)&miniDSP_EQ_Select_enum[0];
	snd_mode_controls.count = 0;

	err = snd_soc_add_controls(codec, &snd_mode_controls, 1);
	if (err < 0)
	{
		printk ("%s:Invalid control %s\n", __FILE__, snd_mode_controls.name);
	}
	else
		dprintk("Added %s Control\r\n", snd_mode_controls.name);

	return err;
}


/***************************************************************************
 * Function: aic3100_add_EQ_mixer_controls
 *
 * Purpose: Configures the AMIXER Controls for the miniDSP run-time configuration.
 * 	    This routine configures the function pointers for the get put and the info
 *          members of the snd_mode_controls and calls the snd_ctl_add routine
 *          to register this with the ALSA Library.
 *
 *       int - 0 on success and -1 on failure
 */
int
aic3100_add_EQ_mixer_controls (struct snd_soc_codec *codec)
{
	int err;

	err = aic3100_EQ_Select_mixer_controls (codec);

	return (err);
}


/***************************************************************************
 * Function: aic3100_parse_biquad_array
 *
 * Purpose: Configures the AMIXER Controls for the miniDSP run-time configuration.
 * 	    This routine configures the function pointers for the get put and the info
 *          members of the snd_mode_controls and calls the snd_ctl_add routine
 *          to register this with the ALSA Library.
 *
 *       int - 0 on success and -1 on failure
 */
int
aic3100_parse_biquad_array (struct snd_soc_codec *codec)
{
	u16 index = 0;
	u16 count = 0;
	u16 jump;
	u16 array_index;
	char data[2];
	u16 totalsize = ARRAY_SIZE (miniDSP_D_reg_values);

	int ret_val = -1;
	u8 regoffset;
	struct i2c_client *i2c = codec->control_data;

	while (index < ADAPTIVE_MAX_CONTROLS) {
		array_index = 0;
		/* check if first location is page register, and populate page addr */
		if (miniDSP_D_reg_values[count].reg_off == 0) {
			aic3100_biquad_array[index].page_num = miniDSP_D_reg_values[count].reg_val;
			dprintk("Biquad[%d].Page = %d\r\n", index, miniDSP_D_reg_values[count].reg_val);
		}

		/* Now increment the Array Count and store the Register Offset */
		aic3100_biquad_array[index].burst_array[array_index++] = regoffset = miniDSP_D_reg_values[count+1].reg_off;
		aic3100_biquad_array[index].burst_array[array_index++] = miniDSP_D_reg_values[count+1].reg_val;

		count+= 2;
		for (jump = 0; (((regoffset + array_index) < 128) &&
				(count < totalsize)); count++, jump++) {
			if (miniDSP_D_reg_values[count].reg_off != (miniDSP_D_reg_values[count - 1].reg_off + 1))
				break;
			else {
				dprintk("Copying %x Value into biquad[%d].array[%d]\r\n",
					miniDSP_D_reg_values[count].reg_val, index, array_index);

				aic3100_biquad_array[index].burst_array[array_index++] = miniDSP_D_reg_values[count].reg_val;
			}
		}

		/* Now Configure the Burst_Tranfer Length into the Array */
		aic3100_biquad_array[index].burst_size = array_index;
		printk("Page %d has %d Coefficient Values \r\n", aic3100_biquad_array[index].page_num,
		       aic3100_biquad_array[index].burst_size);

		/* Increment index to assign values to next Page */
		index++;
	}

	/* Now perform a one-time configuration of the Page 8 and 12 Registers */
	index = 0;

	while (index < ADAPTIVE_MAX_CONTROLS) {

		/* Update the biquad coefficients */
		aic3100_change_page (codec, aic3100_biquad_array[index].page_num);

		ret_val = i2c_master_send (i2c, &aic3100_biquad_array[index].burst_array[0],
					   aic3100_biquad_array[index].burst_size);

		if (ret_val != aic3100_biquad_array[index].burst_size) {
			printk("Error while Updating Page %d Registers %d\r\n",
			       aic3100_biquad_array[index].page_num,
			       aic3100_biquad_array[index].burst_size);
		}
		else {
			dprintk("Wrote %d Values into Page %d Start RegOffset %d\r\n",
				aic3100_biquad_array[index].burst_size,
				aic3100_biquad_array[index].page_num,
				aic3100_biquad_array[index].burst_array[0]);
		}
		index++;
	}

	aic3100_change_page (codec, BUFFER_PAGE_DAC);

	/* Update the Adaptive Filter Status in Page 8[1] Bit 2 */
	/* read register addr to read */
	data[0] = DAC_RAM_CNTRL;

	if (i2c_master_send (i2c, data, 1) != 1) {
		printk ("Cannot write register address for Page 8[1]\n");
		ret_val = -1;
		goto revert;
	}

	/* read the codec/minidsp registers */
	if (i2c_master_recv (i2c, data, 1) != 1) {
		printk ("Can not read codec registers\n");
		ret_val = -1;
		goto revert;
	}

	dprintk ("Page 8 DAC Coefficient RAM Control Reg Value:  0x%x\n", data[0]);

	/* set the bitmask and update the register */
	data[1] = (data[0]) | ADAPTIVE_CONTROL;
	data[0] = DAC_RAM_CNTRL;

	if (i2c_master_send (i2c, data, 2) != 2) {
		printk ("Cannot write Value into register address for Page 8[1]\n");
		ret_val = -1;
	}
	else
		printk("Updated Page 8 DAC Coefficient Control Reg for Adaptive Mode\r\n");

revert:
	return (ret_val);
}


/*
 * Function: aic3100_update_biquad_array
 *
 * Purpose: Configures the AMIXER Controls for the miniDSP run-time
 * 	    configuration.  This routine configures the function
 * 	    pointers for the get put and the info members of the
 * 	    snd_mode_controls and calls the snd_ctl_add routine to
 * 	    register this with the ALSA Library.
 *
 * Return:  int - 0 on success and -1 on failure
 */
int
aic3100_update_biquad_array (struct snd_soc_codec *codec, int speaker_active,
			     int playback_active)
{
	char data[2];
	struct i2c_client *i2c = codec->control_data;
	u8 dac_ram_value;
	int ret_val = -1;

	dprintk(KERN_INFO "update_biquad Speaker_Active %d Playback Status %d\r\n", speaker_active, playback_active);

	if (speaker_active)
		aic3100_eq_select = AIC3100_SPK_EQ_SELECT;
	else
		aic3100_eq_select = AIC3100_HP_EQ_SELECT;

	/* Check for the playback active flag. */
	if (playback_active == 1) {

		/* Add code to mute the DAC */

		/* Wait for some time */

	}
	/* Update the Adaptive Filter Status in Page 8[1] Bit 2 */
	aic3100_change_page (codec, BUFFER_PAGE_DAC);
	/* read register addr to read */
	data[0] = DAC_RAM_CNTRL;

	if (i2c_master_send (i2c, data, 1) != 1) {
		printk ("Cannot write register address for Page 8[1]\n");
		ret_val = -1;
	}
	else {
		/* read the codec/minidsp registers */
		if (i2c_master_recv (i2c, data, 1) != 1) {
			printk ("Can not read Page 8[1] registers\n");
			ret_val = -1;
		}
		else {
			dac_ram_value = data[0];

			/* The Page 8[1] Bit 1 will suggest which Buffer is in ACTIVE use.
			 * Here Bit 1 reset means Buffer A [Speaker EQ] is in Use
			 * similarly Bit 1 set means Buffer B [Headset EQ] is in Use.
			 *
			 * We will perform a buffer Switch only if the speaker_active Flag
			 * and the Bit 1 status do not match
			 */

			if ( ((dac_ram_value & ADAPTIVE_BUFFER_SWITCH) != 0) &&
			     (aic3100_eq_select == AIC3100_SPK_EQ_SELECT)) {
				/* set the bitmask and update the register */
				data[1] = (data[0]) | ADAPTIVE_BUFFER_SWITCH;
				data[0] = DAC_RAM_CNTRL;
			} else if (((dac_ram_value & ADAPTIVE_BUFFER_SWITCH) == 0) &&
				   (aic3100_eq_select == AIC3100_HP_EQ_SELECT)) {
				/* set the bitmask and update the register */
				data[1] = (data[0]) | ADAPTIVE_BUFFER_SWITCH;
				data[0] = DAC_RAM_CNTRL;
			}
			else {
				/* set the bitmask and update the register */
				data[1] = data[0];
				data[0] = DAC_RAM_CNTRL;
			}
			if (i2c_master_send (i2c, data, 2) != 2) {
				printk ("Cannot write Value into register address for Page 8[1]\n");
				ret_val = -1;
			}
			else {
				dprintk(KERN_INFO "Updated for Buffer Switch Old: %x New: %x\r\n", dac_ram_value, data[1]);
			}
		}
	}
	/* Check for the playback active flag. */
	if (playback_active == 1) {
		/* Add code to unmute the DAC */

		/* Wait for some time */
	}

	/* Revert back to Page 0 */
	aic3100_change_page (codec, 0);

	return (0);
}
#endif
