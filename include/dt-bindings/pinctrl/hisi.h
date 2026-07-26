/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This header provides constants for hisilicon pinctrl bindings.
 *
 * Copyright (c) 2015 HiSilicon Limited.
 * Copyright (c) 2015 Linaro Limited.
 */

#ifndef _DT_BINDINGS_PINCTRL_HISI_H
#define _DT_BINDINGS_PINCTRL_HISI_H

/* iomg bit definition */
#define MUX_M0		0
#define MUX_M1		1
#define MUX_M2		2
#define MUX_M3		3
#define MUX_M4		4
#define MUX_M5		5
#define MUX_M6		6
#define MUX_M7		7

/* iocg bit definition */
#define PULL_MASK	(3)      /* 0x03 */
#define PULL_DIS	(0)      /* 0x00 */
#define PULL_UP		(1 << 0) /* 0x01 */
#define PULL_DOWN	(1 << 1) /* 0x02 */

/* drive strength definition */
#define DRIVE_MASK	(7 << 4) /* 0x70 */
#define DRIVE1_02MA	(0 << 4) /* 0x00 */
#define DRIVE1_04MA	(1 << 4) /* 0x10 */
#define DRIVE1_08MA	(2 << 4) /* 0x20 */
#define DRIVE1_10MA	(3 << 4) /* 0x30 */
#define DRIVE2_02MA	(0 << 4) /* 0x00 */
#define DRIVE2_04MA	(1 << 4) /* 0x10 */
#define DRIVE2_08MA	(2 << 4) /* 0x20 */
#define DRIVE2_10MA	(3 << 4) /* 0x30 */
#define DRIVE3_04MA	(0 << 4) /* 0x00 */
#define DRIVE3_08MA	(1 << 4) /* 0x10 */
#define DRIVE3_12MA	(2 << 4) /* 0x20 */
#define DRIVE3_16MA	(3 << 4) /* 0x30 */
#define DRIVE3_20MA	(4 << 4) /* 0x40 */
#define DRIVE3_24MA	(5 << 4) /* 0x50 */
#define DRIVE3_32MA	(6 << 4) /* 0x60 */
#define DRIVE3_40MA	(7 << 4) /* 0x70 */
#define DRIVE4_02MA	(0 << 4) /* 0x00 */
#define DRIVE4_04MA	(2 << 4) /* 0x20 */
#define DRIVE4_08MA	(4 << 4) /* 0x40 */
#define DRIVE4_10MA	(6 << 4) /* 0x60 */

/* drive strength definition for hi3660 */
#define DRIVE6_MASK	(15 << 4) /* 0xf0 */
#define DRIVE6_04MA	(0 << 4)  /* 0x00 */
#define DRIVE6_12MA	(4 << 4)  /* 0x40 */
#define DRIVE6_19MA	(8 << 4)  /* 0x80 */
#define DRIVE6_27MA	(10 << 4) /* 0xa0 */
#define DRIVE6_32MA	(15 << 4) /* 0xf0 */
#define DRIVE7_02MA	(0 << 4)  /* 0x00 */
#define DRIVE7_04MA	(1 << 4)  /* 0x10 */
#define DRIVE7_06MA	(2 << 4)  /* 0x20 */
#define DRIVE7_08MA	(3 << 4)  /* 0x30 */
#define DRIVE7_10MA	(4 << 4)  /* 0x40 */
#define DRIVE7_12MA	(5 << 4)  /* 0x50 */
#define DRIVE7_14MA	(6 << 4)  /* 0x60 */
#define DRIVE7_16MA	(7 << 4)  /* 0x70 */
#endif
