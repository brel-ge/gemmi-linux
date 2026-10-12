/*

 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */



/*!
 * @file gemmi cam bridge driver
 *
 * @ingroup Camera
 */
#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

//#define  GEMMI_CAM_BRIDGE_MODEL_ID_1     0x03
/*!
 * Basic camera values
 */
#define GEMMI_CAM_BRIDGE_FRAME_RATE 30
#define GEMMI_CAM_BRIDGE_CLK_VGA         24000000 /* Desired clock rate */
#define GEMMI_CAM_BRIDGE_CLK_C_CAMII     24000000 /* Desired clock rate */
#define GEMMI_CAM_BRIDGE_CLK_NTSC        27000000 /* Desired clock rate */
#define GEMMI_CAM_BRIDGE_CLK_SQUARE_400  18000000 /* Desired clock rate */
#define GEMMI_CAM_BRIDGE_CLK_MIN         20000000 /* 12000000  */
#define GEMMI_CAM_BRIDGE_CLK_MAX 2700000
#define GEMMI_CAM_BRIDGE_MAX_WIDTH 640  /* Max width for this camera */
#define GEMMI_CAM_BRIDGE_MAX_HEIGHT 480 /* Max height for this camera */



#define MIN_FPS 30
#define MAX_FPS 30
#define DEFAULT_FPS 30


enum gemmi_cam_bridge_mode {
	gemmi_cam_bridge_mode_MIN = 0,
	gemmi_cam_bridge_mode_VGA = 0,
	gemmi_cam_bridge_mode_NTSC = 1,
	gemmi_cam_bridge_mode_SQUARE_400 = 2,
	gemmi_cam_bridge_mode_C_CAMII = 3,
	gemmi_cam_bridge_mode_FPD_DEV = 4,
	gemmi_cam_bridge_mode_1080P = 5,
	gemmi_cam_bridge_mode_800P = 6,
	gemmi_cam_bridge_mode_MAX = 6,
};

struct gemmi_cam_bridge_mode_info {
	enum gemmi_cam_bridge_mode mode;
	unsigned int width;
	unsigned int height;
	unsigned int framerate;
	unsigned int clock_curr;
	unsigned int nobt_hs_inv;
	unsigned int nobt_vs_inv;
};

static struct gemmi_cam_bridge_mode_info
	gemmi_cam_bridge_mode_info_data[gemmi_cam_bridge_mode_MAX + 1] = {
		{
			.mode = gemmi_cam_bridge_mode_VGA,
			.width = 640,
			.height = 480,
			.framerate = 30,
			.clock_curr = GEMMI_CAM_BRIDGE_CLK_VGA,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
		{
			.mode = gemmi_cam_bridge_mode_NTSC,
			.width = 720,
			.height = 260,
			.framerate = 30,
			.clock_curr = GEMMI_CAM_BRIDGE_CLK_NTSC,
			.nobt_hs_inv = 1,
			.nobt_vs_inv = 0
		},
		{
			.mode = gemmi_cam_bridge_mode_SQUARE_400,
			.width = 400,
			.height = 400,
			.framerate = 30,
			.clock_curr = GEMMI_CAM_BRIDGE_CLK_SQUARE_400,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
		{
			.mode = gemmi_cam_bridge_mode_C_CAMII,
			.width = 1280,
			.height = 968,
			.framerate = 30,
			.clock_curr = GEMMI_CAM_BRIDGE_CLK_C_CAMII,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
		{
			.mode = gemmi_cam_bridge_mode_FPD_DEV,
			.width = 1280,
			.height = 720,
			.framerate = 30,
			.clock_curr = GEMMI_CAM_BRIDGE_CLK_C_CAMII,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
		{
			.mode = gemmi_cam_bridge_mode_1080P,
			.width = 960,
			.height = 1080,
			.framerate = 30,
			.clock_curr = GEMMI_CAM_BRIDGE_CLK_C_CAMII,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
		{
			.mode = gemmi_cam_bridge_mode_800P,
			.width = 1280,
			.height = 800,
			.framerate = 30,
			.clock_curr = GEMMI_CAM_BRIDGE_CLK_C_CAMII,
			.nobt_hs_inv = 0,
			.nobt_vs_inv = 1
		},
};

static int gemmi_cam_bridge_probe(struct i2c_client *client);
static void gemmi_cam_bridge_remove(struct i2c_client *client);

static const struct i2c_device_id gemmi_cam_bridge_id[] = {
    {"gemmi_cam_bridge", 0},
    {},
};



MODULE_DEVICE_TABLE(i2c, gemmi_cam_bridge_id);

static struct i2c_driver gemmi_cam_bridge_i2c_driver = {
    .driver = {
           .owner = THIS_MODULE,
           .name = "gemmi_cam_bridge",
           },
    .probe = gemmi_cam_bridge_probe,
    .remove = gemmi_cam_bridge_remove,
    .id_table = gemmi_cam_bridge_id,
};


/*!
 * Function definitions
 */


/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate the required xclk frequency.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
    struct sensor_data *sensor = s->priv;

    if (s == NULL) {
        pr_err("   ERROR!! no slave device set!\n");
        return -1;
    }

    memset(p, 0, sizeof(*p));
    p->if_type = V4L2_IF_TYPE_BT656;
    p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
    p->u.bt656.clock_min = GEMMI_CAM_BRIDGE_CLK_MIN;
    p->u.bt656.clock_max = GEMMI_CAM_BRIDGE_CLK_MAX;

    /* the himax is configured vsync high and hsync high;
     * perhaps cause on a error at the lvds we have to set here the vsync to high and the hsync to low!!
     */



    p->u.bt656.clock_curr =
	    gemmi_cam_bridge_mode_info_data[sensor->streamcap.capturemode].clock_curr;
    p->u.bt656.nobt_vs_inv =
	    gemmi_cam_bridge_mode_info_data[sensor->streamcap.capturemode].nobt_vs_inv;
    p->u.bt656.nobt_hs_inv =
	    gemmi_cam_bridge_mode_info_data[sensor->streamcap.capturemode].nobt_hs_inv;

    p->u.bt656.bt_sync_correct = 1; // translates to CSI ext vsync

    printk(KERN_INFO "Test Trace BT p->u.bt656.clock_curr: %d \n", p->u.bt656.clock_curr);
    printk(KERN_INFO "Test Trace BT p->u.bt656.nobt_vs_inv: %d \n", p->u.bt656.nobt_vs_inv);
    printk(KERN_INFO "Test Trace BT p->u.bt656.nobt_hs_inv: %d \n", p->u.bt656.nobt_hs_inv);
   // printk(KERN_ERR "Test Trace BT p->u.bt656.bt_sync_correct: %d \n", p->u.bt656.bt_sync_correct);

    return 0;
 }

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
    struct sensor_data *sensor = s->priv;

    //printk(KERN_ERR "In gemmi_cam_bridge:ioctl_s_power\n");

    // ipu_csi_enable_mclk_if(CSI_MCLK_I2C, 0 /* cam->csi */, true, true);

    sensor->on = on;

    return 0;
}
static int gemmi_cam_bridge_change_mode(struct sensor_data *sensor, 
                                        enum gemmi_cam_bridge_mode new_mode)
{
	int retval = 0;

	if (new_mode > gemmi_cam_bridge_mode_MAX ||
	    new_mode < gemmi_cam_bridge_mode_MIN) {
		pr_err("Wrong gemmi_cam_bridge mode detected!\n");
		return -1;
	}

	sensor->pix.width =
		gemmi_cam_bridge_mode_info_data[new_mode].width;
	sensor->pix.height =
		gemmi_cam_bridge_mode_info_data[new_mode].height;

    sensor->streamcap.capturemode = new_mode;

	return retval;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	unsigned int tgt_fps;	/* target frames per secound */
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		ret = gemmi_cam_bridge_change_mode(sensor, a->parm.capture.capturemode);
		if(ret < 0){
			return ret;
		}

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
    int ret = 0;
    struct v4l2_captureparm *cparm = &a->parm.capture;
	struct sensor_data *sensor = s->priv;
    /* s->priv points to gemmi_cam_bridge_data */
    //printk(KERN_ERR "In gemmi_cam_bridge:ioctl_g_parm\n");


    switch (a->type) {
    /* This is the only case currently handled. */
    case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        //printk(KERN_ERR "   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
        memset(a, 0, sizeof(*a));
        a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        cparm->capability = sensor->streamcap.capability;
        cparm->timeperframe = sensor->streamcap.timeperframe;
        cparm->capturemode = sensor->streamcap.capturemode;
        ret = 0;
        break;


    /* These are all the possible cases. */
    case V4L2_BUF_TYPE_VIDEO_OUTPUT:
    case V4L2_BUF_TYPE_VIDEO_OVERLAY:
    case V4L2_BUF_TYPE_VBI_CAPTURE:
    case V4L2_BUF_TYPE_VBI_OUTPUT:
    case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
    case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
        pr_err("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE " \
            "but %d\n", a->type);
        ret = -EINVAL;
        break;

    default:
        pr_err("   type is unknown - %d\n", a->type);
        ret = -EINVAL;
        break;
    }

    return ret;
}


/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
    struct sensor_data *sensor = s->priv;
    /* s->priv points to gemmi_cam_bridge_data */

    printk(KERN_ERR "In gemmi_cam_bridge:ioctl_g_fmt_cap.\n");
    printk(KERN_ERR "   Returning size of %dx%d\n",sensor->pix.width, sensor->pix.height);

    f->fmt.pix = sensor->pix;

    return 0;
}


/* list of image formats supported by sensor */
static const struct v4l2_fmtdesc gemmi_cam_bridge_formats[] = {
    {
        .description = "YUV422 UYVY",
        .pixelformat = V4L2_PIX_FMT_UYVY,
    },
};

#define GEMMI_CAM_BRIDGE_NUM_CAPTURE_FORMATS ARRAY_SIZE(gemmi_cam_bridge_formats)

static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
                   struct v4l2_fmtdesc *fmt)
{
    struct sensor_data *sensor = s->priv;
	if (fmt->index > 0)	/* only 1 pixelformat support so far */
		return -EINVAL;

	fmt->pixelformat = sensor->pix.pixelformat;

	return 0;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
    struct sensor_data *sensor = s->priv;
	if (fsize->index > gemmi_cam_bridge_mode_MAX ){
		return -EINVAL;
	}

	fsize->pixel_format = sensor->pix.pixelformat;
	fsize->discrete.width =
		gemmi_cam_bridge_mode_info_data[fsize->index].width;
	fsize->discrete.height =
		gemmi_cam_bridge_mode_info_data[fsize->index].height;
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
    struct sensor_data *sensor = s->priv;
	if (fival->index > gemmi_cam_bridge_mode_MAX )
		return -EINVAL;

	if (fival->pixel_format != sensor->pix.pixelformat){
		pr_warn("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
        fival->discrete.denominator =
		gemmi_cam_bridge_mode_info_data[fival->index].framerate;

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
static struct v4l2_int_ioctl_desc gemmi_cam_bridge_ioctl_desc[] = {
    //{vidioc_int_dev_init_num, (v4l2_int_ioctl_func*)ioctl_dev_init},

    /*!
     * Delinitialise the dev. at slave detach.
     * The complement of ioctl_dev_init.
     */
    {vidioc_int_s_power_num, (v4l2_int_ioctl_func*) ioctl_s_power},
    {vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*) ioctl_g_ifparm},
   // {vidioc_int_init_num, (v4l2_int_ioctl_func*) ioctl_init},

    /*!
     * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
     */
    {vidioc_int_enum_fmt_cap_num,(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},

    /*!
     * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
     * This ioctl is used to negotiate the image capture size and
     * pixel format without actually making it take effect.
     */
    /*{vidioc_int_try_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_try_fmt_cap},*/
    {vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*) ioctl_g_fmt_cap},

    /*!
     * If the requested format is supported, configures the HW to use that
     * format, returns error code if format not supported or HW can't be
     * correctly configured.
     */
    /*{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap},*/
    {vidioc_int_g_parm_num, (v4l2_int_ioctl_func*) ioctl_g_parm},
    {vidioc_int_s_parm_num, (v4l2_int_ioctl_func*) ioctl_s_parm},
    //{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func*) ioctl_g_ctrl},
    //{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func*) ioctl_s_ctrl},
    {vidioc_int_enum_framesizes_num, (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
    {vidioc_int_enum_frameintervals_num, (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
};
#pragma GCC diagnostic pop

static struct v4l2_int_slave gemmi_cam_bridge_slave[] = {
    {
    .ioctls = gemmi_cam_bridge_ioctl_desc,
    .num_ioctls = ARRAY_SIZE(gemmi_cam_bridge_ioctl_desc),
    .attach_to = "mxc_v4l2_cap0",
    },
    {
    .ioctls = gemmi_cam_bridge_ioctl_desc,
    .num_ioctls = ARRAY_SIZE(gemmi_cam_bridge_ioctl_desc),
    .attach_to = "mxc_v4l2_cap1",
    }
};
/*
static struct v4l2_int_device gemmi_cam_bridge_int_device = {
    .module = THIS_MODULE,
    .name = "gemmi_cam_bridge",
    .type = v4l2_int_type_slave,
};
*/


/*!
 * gemmi_cam_bridge I2C probe function
 * Function set in i2c_driver struct.
 * Called by insmod gemmi_cam_bridge_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static int gemmi_cam_bridge_probe(struct i2c_client *client)
{
    //struct device *dev = &client->dev;
    struct pinctrl *pinctrl;
    struct device *dev = &client->dev;
    int retval;
    struct regmap *gpr;

    struct v4l2_int_device* gemmi_cam_bridge_int_device;
    struct sensor_data*  sensor;
    int capturemode = gemmi_cam_bridge_mode_VGA;

    //printk(KERN_ERR "In gemmi_cam_bridge_probe  device id is %s\n", id->name);
    /* gemmi_cam_bridge pinctrl */
    pinctrl = devm_pinctrl_get_select_default(dev);
    if (IS_ERR(pinctrl)) {
	dev_err(dev, "setup pinctrl failed\n");
	return PTR_ERR(pinctrl);
    }


    sensor = devm_kzalloc(dev,
		 sizeof(struct sensor_data), GFP_KERNEL);
    /* Set initial values for the sensor struct. */
    memset(sensor, 0, sizeof(struct sensor_data));
    sensor->i2c_client = client;
    sensor->streamcap.timeperframe.denominator =
	    gemmi_cam_bridge_mode_info_data[capturemode].framerate;
    sensor->streamcap.timeperframe.numerator = 1;
    sensor->streamcap.capability = 0;  /* No higher resolution or frame*/
    sensor->streamcap.capturemode = capturemode;
    sensor->pix.width =
	    gemmi_cam_bridge_mode_info_data[capturemode].width;
    sensor->pix.height =
	    gemmi_cam_bridge_mode_info_data[capturemode].height;
    sensor->pix.pixelformat = V4L2_PIX_FMT_UYVY;
    sensor->pix.priv = 1;
    sensor->on = true;
    //printk(KERN_ERR "   client name is %s\n", client->name);

    sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
    if (IS_ERR(sensor->sensor_clk)) {
	/* assuming clock enabled by default */
	sensor->sensor_clk = NULL;
	dev_err(dev, "clock-frequency missing or invalid\n");
	return PTR_ERR(sensor->sensor_clk);
    }

    retval = of_property_read_u32(dev->of_node, "mclk",
					(unsigned int *) &(sensor->mclk));
    if (retval) {
	dev_err(dev, "mclk missing or invalid\n");
	return retval;
    }

    retval = of_property_read_u32(dev->of_node, "mclk_source",
					(unsigned int *) &(sensor->mclk_source));
    if (retval) {
	dev_err(dev, "mclk_source missing or invalid\n");
	return retval;
    }

    retval = of_property_read_u32(dev->of_node, "ipu_id",
					&sensor->ipu_id);
    if (retval) {
	dev_err(dev, "ipu_id missing or invalid\n");
	return retval;
    }

    retval = of_property_read_u32(dev->of_node, "csi_id",
					&(sensor->csi));
    if (retval) {
        dev_err(dev, "csi_id invalid\n");
	return retval;
    }

    printk(KERN_INFO " %s called !\n", __FUNCTION__);

    // For now bin driver to v4l2_cap_0

    gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
    if (!IS_ERR(gpr)) {
 	if (of_machine_is_compatible("fsl,imx6q") ||
	    of_machine_is_compatible("fsl,imx6qp")) {
		int mask = sensor->csi ? (1 << 20) : (1 << 19);
		if (sensor->csi != sensor->ipu_id) {
			printk(KERN_ERR "%s: csi_id != ipu_id\n", __func__);
			return -ENODEV;
		}

		regmap_update_bits(gpr, IOMUXC_GPR1, mask, mask);
	} else if (of_machine_is_compatible("fsl,imx6dl")) {
		int mask = sensor->csi ? (7 << 3) : (7 << 0);
		int val =  sensor->csi ? (4 << 3) : (4 << 0);

		regmap_update_bits(gpr, IOMUXC_GPR13, mask, val);
	} else {
		printk(KERN_ERR "%s: unable to set IPU muxing: unknown machine\n",
		__func__);
	}
    } else {
	printk(KERN_ERR "%s: failed to find fsl,imx6q-iomux-gpr regmap\n",
	       __func__);
    }

    gemmi_cam_bridge_int_device = devm_kzalloc(dev,
		 sizeof(struct v4l2_int_device), GFP_KERNEL);

    strscpy(gemmi_cam_bridge_int_device->name, client->name,
            sizeof(gemmi_cam_bridge_int_device->name));

    gemmi_cam_bridge_int_device->module = THIS_MODULE;
    gemmi_cam_bridge_int_device->type = v4l2_int_type_slave;
    gemmi_cam_bridge_int_device->u.slave =
	    &gemmi_cam_bridge_slave[sensor->csi];
    gemmi_cam_bridge_int_device->priv = sensor;

    sensor->v4l2_int_device = gemmi_cam_bridge_int_device;

    retval = v4l2_int_device_register(gemmi_cam_bridge_int_device);
    if( retval == 0 )
    {
        printk(KERN_INFO "In gemmi_cam_bridge for csi %d master %s!\n",
	       sensor->csi,
	       gemmi_cam_bridge_int_device->u.slave->master->name);
    }
    else {
        printk(KERN_ERR "   ERROR: v4l2_int_device_register gemmi_cam_bridge was not succesfull\n");
    }

    return retval;
}

/*!
 * Function set in i2c_driver struct.
 * Called on rmmod gemmi_cam_bridge_camera.ko
 */
static void gemmi_cam_bridge_remove(struct i2c_client *client)
{
    struct v4l2_int_device* gemmi_cam_bridge_int_device
	     = i2c_get_clientdata(client);

    printk(KERN_INFO "In gemmi_cam_bridge_remove\n");

    v4l2_int_device_unregister(gemmi_cam_bridge_int_device);
}

/*!
 * GEMMI_CAM_BRIDGE init function.
 * Called by insmod gemmi_cam_bridge_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int gemmi_cam_bridge_init(void)
{
    u8 err = 0;

    printk(KERN_INFO "In gemmi_cam_bridge_init !\n");

    /* Tells the i2c driver what functions to call for this driver. */
    /* even if this driver doesn^t need the i2c interface */
    err = i2c_add_driver(&gemmi_cam_bridge_i2c_driver);
    if (err != 0){
        pr_err("%s:driver registration failed, error=%d \n",
               __func__, err);
    } else {
        printk(KERN_INFO "In gemmi_cam_bridge i2c registered %d!\n", err);
    }

    return err;
}

/*!
 * GEMMI_CAM_BRIDGE cleanup function.
 * Called on rmmod gemmi_cam_bridge_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit gemmi_cam_bridge_clean(void)
{
    printk(KERN_INFO "In gemmi_cam_bridge_clean!\n");

    i2c_del_driver(&gemmi_cam_bridge_i2c_driver);
}

module_init(gemmi_cam_bridge_init);
module_exit(gemmi_cam_bridge_clean);

MODULE_AUTHOR("Bruetsch Elektronik AG");
MODULE_DESCRIPTION("GEMMI_CAM_BRIDGE Camera Driver");
MODULE_LICENSE("GPL");
