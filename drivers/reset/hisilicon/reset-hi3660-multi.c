// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2016-2017 Linaro Ltd.
 * Copyright (c) 2016-2017 HiSilicon Technologies Co., Ltd.
 * Copyright (c) 2026 Tildeguy <tildeguy@mainlining.org>
 */
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

struct hi3660_reset_multi_controller {
	struct reset_controller_dev rst;
	struct regmap *map;
	u32 mask;
};

#define to_hi3660_reset_multi_controller(_rst) \
	container_of(_rst, struct hi3660_reset_multi_controller, rst)

static int hi3660_reset_multi_program_hw(struct reset_controller_dev *rcdev,
				   unsigned long offset, bool assert)
{
	struct hi3660_reset_multi_controller *rc = to_hi3660_reset_multi_controller(rcdev);

	if (assert)
		return regmap_write(rc->map, offset, rc->mask);
	else
		return regmap_write(rc->map, offset + 4, rc->mask);
}

static int hi3660_reset_multi_assert(struct reset_controller_dev *rcdev,
			       unsigned long idx)
{
	return hi3660_reset_multi_program_hw(rcdev, idx, true);
}

static int hi3660_reset_multi_deassert(struct reset_controller_dev *rcdev,
				 unsigned long idx)
{
	return hi3660_reset_multi_program_hw(rcdev, idx, false);
}

static int hi3660_reset_multi_dev(struct reset_controller_dev *rcdev,
			    unsigned long idx)
{
	int err;

	err = hi3660_reset_multi_assert(rcdev, idx);
	if (err)
		return err;

	return hi3660_reset_multi_deassert(rcdev, idx);
}

static const struct reset_control_ops hi3660_reset_multi_ops = {
	.reset    = hi3660_reset_multi_dev,
	.assert   = hi3660_reset_multi_assert,
	.deassert = hi3660_reset_multi_deassert,
};

static int hi3660_reset_multi_xlate(struct reset_controller_dev *rcdev,
			      const struct of_phandle_args *reset_spec)
{
	return reset_spec->args[0]; /* offset */
}

static int hi3660_reset_multi_probe(struct platform_device *pdev)
{
	struct hi3660_reset_multi_controller *rc;
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int ret, bits_count;
	u32 *bits;

	rc = devm_kzalloc(dev, sizeof(*rc), GFP_KERNEL);
	if (!rc)
		return -ENOMEM;

	rc->map = syscon_regmap_lookup_by_phandle(np, "hisilicon,rst-syscon");
	if (IS_ERR(rc->map)) {
		return dev_err_probe(dev, PTR_ERR(rc->map),
			"failed to get hisilicon,rst-syscon\n");
	}
	
	bits_count = of_property_count_u32_elems(np, "hisilicon,rst-bits");
	if (bits_count < 0) {
		return dev_err_probe(dev, -ENODEV,
			"failed to get hisilicon,rst-bits\n");
	}

	bits = devm_kcalloc(dev, bits_count, sizeof(*bits), GFP_KERNEL);
	if (!bits)
	  return -ENOMEM;

	ret = of_property_read_u32_array(np, "hisilicon,rst-bits", bits, bits_count);
	if (ret) {
		return dev_err_probe(dev, ret,
			"failed to get hisilicon,rst-bits\n");
	}

	rc->mask = 0;
	for (int i = 0; i < bits_count; i++) {
		rc->mask |= 1 << bits[i];
	}

	rc->rst.ops = &hi3660_reset_multi_ops,
	rc->rst.of_node = np;
	rc->rst.of_reset_n_cells = 1;
	rc->rst.of_xlate = hi3660_reset_multi_xlate;

	return reset_controller_register(&rc->rst);
}

static const struct of_device_id hi3660_reset_multi_match[] = {
	{ .compatible = "hisilicon,hi3660-reset-multi", },
	{ .compatible = "hisilicon,hi6250-usb-ahbif", },
	{},
};
MODULE_DEVICE_TABLE(of, hi3660_reset_multi_match);

static struct platform_driver hi3660_reset_multi_driver = {
	.probe = hi3660_reset_multi_probe,
	.driver = {
		.name = "hi3660-reset-multi",
		.of_match_table = hi3660_reset_multi_match,
	},
};

static int __init hi3660_reset_multi_init(void)
{
	return platform_driver_register(&hi3660_reset_multi_driver);
}
arch_initcall(hi3660_reset_multi_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HiSilicon Hi3660 Reset Multi Driver");
