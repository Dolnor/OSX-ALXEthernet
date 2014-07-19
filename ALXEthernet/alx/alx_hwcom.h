/*
 * Copyright (c) 2012 Qualcomm Atheros, Inc.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _ALX_HWCOMMON_H_
#define _ALX_HWCOMMON_H_

#include "alx_sw.h"


#define BIT_ALL	    0xffffffffUL

#define FIELD_GETX(_x, _name)   (((_x) >> (_name##_SHIFT)) & (_name##_MASK))
#define FIELD_SETS(_x, _name, _v)   (\
(_x) =                               \
((_x) & ~((_name##_MASK) << (_name##_SHIFT)))            |\
(((u16)(_v) & (_name##_MASK)) << (_name##_SHIFT)))
#define FIELD_SETL(_x, _name, _v)   (\
(_x) =                               \
((_x) & ~((_name##_MASK) << (_name##_SHIFT)))            |\
(((u32)(_v) & (_name##_MASK)) << (_name##_SHIFT)))
#define FIELDL(_name, _v) (((u32)(_v) & (_name##_MASK)) << (_name##_SHIFT))
#define FIELDS(_name, _v) (((u16)(_v) & (_name##_MASK)) << (_name##_SHIFT))


#define LX_ERR_SUCCESS          0x0000
#define LX_ERR_ALOAD            0x0001
#define LX_ERR_RSTMAC           0x0002
#define LX_ERR_PARM             0x0003
#define LX_ERR_MIIBUSY          0x0004

/* link capability */
#define LX_LC_10H               0x01
#define LX_LC_10F               0x02
#define LX_LC_100H              0x04
#define LX_LC_100F              0x08
#define LX_LC_1000F             0x10
#define LX_LC_ALL               \
	(LX_LC_10H|LX_LC_10F|LX_LC_100H|LX_LC_100F|LX_LC_1000F)

/* options for MAC contrl */
#define LX_FLT_DIRECT           BIT(0)
#define LX_FLT_BROADCAST        BIT(1)  /* 1:enable rx-broadcast */
#define LX_FLT_MULTI_ALL        BIT(2)
#define LX_FLT_PROMISC          BIT(3)
#define LX_VLAN_STRIP           BIT(4)
#define LX_LOOPBACK             BIT(5)
#define LX_FC_TXEN              BIT(6)
#define LX_FC_RXEN              BIT(7)
#define LX_ADD_FCS              BIT(8)
#define LX_SINGLE_PAUSE         BIT(9)
#define LX_MACSPEED_1000        BIT(10)  /* 1:1000M, 0:10/100M */
#define LX_MACDUPLEX_FULL       BIT(11)  /* 1:full, 0:half */


/* interop between drivers */
#define LX_DRV_TYPE_MASK                0x1FUL
#define LX_DRV_TYPE_SHIFT               27
#define LX_DRV_TYPE_UNKNOWN             0
#define LX_DRV_TYPE_BIOS                1
#define LX_DRV_TYPE_BTROM               2
#define LX_DRV_TYPE_PKT                 3
#define LX_DRV_TYPE_NDS2                4
#define LX_DRV_TYPE_UEFI                5
#define LX_DRV_TYPE_NDS5                6
#define LX_DRV_TYPE_NDS62               7
#define LX_DRV_TYPE_NDS63               8
#define LX_DRV_TYPE_LNX                 9
#define LX_DRV_TYPE_ODI16               10
#define LX_DRV_TYPE_ODI32               11
#define LX_DRV_TYPE_FRBSD               12
#define LX_DRV_TYPE_NTBSD               13
#define LX_DRV_TYPE_WCE                 14
#define LX_DRV_PHY_AUTO                 BIT(26)  /* 1:auto, 0:force */
#define LX_DRV_PHY_1000                 BIT(25)
#define LX_DRV_PHY_100                  BIT(24)
#define LX_DRV_PHY_10                   BIT(23)
#define LX_DRV_PHY_DUPLEX               BIT(22)  /* 1:full, 0:half */
#define LX_DRV_PHY_FC                   BIT(21)  /* 1:en flow control */
#define LX_DRV_PHY_MASK                 0x1FUL
#define LX_DRV_PHY_SHIFT                21
#define LX_DRV_PHY_UNKNOWN              0
#define LX_DRV_DISABLE                  BIT(18)
#define LX_DRV_WOLS5_EN                 BIT(17)
#define LX_DRV_WOLS5_BIOS_EN            BIT(16)
#define LX_DRV_AZ_EN                    BIT(12)
#define LX_DRV_WOLPATTERN_EN            BIT(11)
#define LX_DRV_WOLLINKUP_EN             BIT(10)
#define LX_DRV_WOLMAGIC_EN              BIT(9)
#define LX_DRV_WOLCAP_BIOS_EN           BIT(8)
#define LX_DRV_ASPM_SPD1000LMT_MASK     0x3UL
#define LX_DRV_ASPM_SPD1000LMT_SHIFT    4
#define LX_DRV_ASPM_SPD1000LMT_100M     0
#define LX_DRV_ASPM_SPD1000LMT_NO       1
#define LX_DRV_ASPM_SPD1000LMT_1M       2
#define LX_DRV_ASPM_SPD1000LMT_10M      3
#define LX_DRV_ASPM_SPD100LMT_MASK      0x3UL
#define LX_DRV_ASPM_SPD100LMT_SHIFT     2
#define LX_DRV_ASPM_SPD100LMT_1M        0
#define LX_DRV_ASPM_SPD100LMT_10M       1
#define LX_DRV_ASPM_SPD100LMT_100M      2
#define LX_DRV_ASPM_SPD100LMT_NO        3
#define LX_DRV_ASPM_SPD10LMT_MASK       0x3UL
#define LX_DRV_ASPM_SPD10LMT_SHIFT      0
#define LX_DRV_ASPM_SPD10LMT_1M         0
#define LX_DRV_ASPM_SPD10LMT_10M        1
#define LX_DRV_ASPM_SPD10LMT_100M       2
#define LX_DRV_ASPM_SPD10LMT_NO         3

/* flag of phy inited */
#define LX_PHY_INITED           0x003F


#endif/*_ALX_HWCOMMON_H_*/
