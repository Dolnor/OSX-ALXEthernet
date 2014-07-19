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
 *
 */

#include "alx.h"
#include "alc_hw.h"

/* NIC */
static int alc_identify_nic(struct alx_hw *hw)
{
	return 0;
}


/* PHY */
static int alc_read_phy_reg(struct alx_hw *hw, u16 reg_addr, u16 *phy_data)
{
	unsigned long  flags;
	int  retval = 0;

	spin_lock_irqsave(&hw->mdio_lock, flags);

	retval = l1c_read_phy(hw, false, ALX_MDIO_DEV_TYPE_NORM, hw->link_up,
			      reg_addr, phy_data);
	if (retval)
		alx_hw_err(hw, "error:%u, when read phy reg\n", retval);

	spin_unlock_irqrestore(&hw->mdio_lock, flags);
	return retval;
}


static int alc_write_phy_reg(struct alx_hw *hw, u16 reg_addr, u16 phy_data)
{
	unsigned long  flags;
	int  retval = 0;

	spin_lock_irqsave(&hw->mdio_lock, flags);

	retval = l1c_write_phy(hw, false, ALX_MDIO_DEV_TYPE_NORM, hw->link_up,
			       reg_addr, phy_data);
	if (retval)
		alx_hw_err(hw, "error:%u, when write phy reg\n", retval);

	spin_unlock_irqrestore(&hw->mdio_lock, flags);
	return retval;
}

#ifdef CONFIG_ALX_DEBUGFS

static int alc_read_ext_phy_reg(struct alx_hw *hw, u8 type, u16 reg_addr,
				u16 *phy_data)
{
	unsigned long  flags;
	int  retval = 0;

	spin_lock_irqsave(&hw->mdio_lock, flags);

	retval = l1c_read_phy(hw, true, type, false, reg_addr, phy_data);
	if (retval)
		alx_hw_err(hw, "error:%u, when read ext phy reg\n", retval);

	spin_unlock_irqrestore(&hw->mdio_lock, flags);
	return retval;
}


static int alc_write_ext_phy_reg(struct alx_hw *hw, u8 type, u16 reg_addr,
				 u16 phy_data)
{
	unsigned long  flags;
	int  retval = 0;

	spin_lock_irqsave(&hw->mdio_lock, flags);

	retval = l1c_write_phy(hw, true, type, false, reg_addr, phy_data);
	if (retval)
		alx_hw_err(hw, "error:%u, when write ext phy reg\n", retval);

  spin_unlock_irqrestore(&hw->mdio_lock, flags);
  return retval;
}

#endif


static int alc_init_phy(struct alx_hw *hw)
{
	u16 phy_id[2];
	int retval;

	spin_lock_init(&hw->mdio_lock);

	retval = alc_read_phy_reg(hw, MII_PHYSID1, &phy_id[0]);
	if (retval)
		return retval;
	retval = alc_read_phy_reg(hw, MII_PHYSID2, &phy_id[1]);
	if (retval)
		return retval;

	memcpy(&hw->phy_id, phy_id, sizeof(hw->phy_id));

	hw->autoneg_advertised = LX_LC_ALL;

	return retval;
}


static int alc_reset_phy(struct alx_hw *hw)
{
	bool pws_en, az_en, ptp_en;
	int retval = 0;

	pws_en = az_en = ptp_en = false;
	CLI_HW_FLAG(PWSAVE_EN);
	CLI_HW_FLAG(AZ_EN);
	CLI_HW_FLAG(PTP_EN);

	if (CHK_HW_FLAG(PWSAVE_CAP)) {
		pws_en = true;
		SET_HW_FLAG(PWSAVE_EN);
	}

	if (CHK_HW_FLAG(AZ_CAP)) {
		az_en = true;
		SET_HW_FLAG(AZ_EN);
	}

	if (CHK_HW_FLAG(PTP_CAP)) {
		ptp_en = true;
		SET_HW_FLAG(PTP_EN);
	}

	alx_hw_info(hw, "reset PHY, pws = %d, az = %d, ptp = %d\n",
		    pws_en, az_en, ptp_en);

	if (l1c_reset_phy(hw, pws_en, az_en, ptp_en)) {
		alx_hw_err(hw, "error when reset phy\n");
		retval = -EINVAL;
	}

	return retval;
}


/* LINK */
static int alc_setup_phy_link(struct alx_hw *hw, u8 speed, bool autoneg,
			      bool fc)
{
	int retval = 0;

	if (!CHK_HW_FLAG(GIGA_CAP))
		speed &= ~LX_LC_1000F;

	if (l1c_init_phy_spdfc(hw, autoneg, speed, fc)) {
		alx_hw_err(hw, "error when init phy speed and fc\n");
		retval = -EINVAL;
	}

	return retval;
}

static int alc_check_phy_link(struct alx_hw *hw, u8 *speed, bool *link_up)
{
	u16 bmsr, giga;
	int retval;

	alc_read_phy_reg(hw, MII_BMSR, &bmsr);
	retval = alc_read_phy_reg(hw, MII_BMSR, &bmsr);
	if (retval)
		return retval;

	if (!(bmsr & BMSR_LSTATUS)) {
		*link_up = false;
		*speed = 0;
		return retval;
	}
	*link_up = true;

	/* Read PHY Specific Status Register (17) */
	retval = alc_read_phy_reg(hw, L1C_MII_GIGA_PSSR, &giga);
	if (retval)
		return retval;


	if (!(giga & L1C_GIGA_PSSR_SPD_DPLX_RESOLVED)) {
		alx_hw_err(hw, "error for speed duplex resolved\n");
		return -EINVAL;
	}

	switch (giga & L1C_GIGA_PSSR_SPEED) {
	case L1C_GIGA_PSSR_1000MBS:
		if (giga & L1C_GIGA_PSSR_DPLX)
			*speed = LX_LC_1000F;
		else
			alx_hw_err(hw, "1000M half is invalid\n");
		break;
	case L1C_GIGA_PSSR_100MBS:
		if (giga & L1C_GIGA_PSSR_DPLX)
			*speed = LX_LC_100F;
		else
			*speed = LX_LC_100H;
		break;
	case L1C_GIGA_PSSR_10MBS:
		if (giga & L1C_GIGA_PSSR_DPLX)
			*speed = LX_LC_10F;
		else
			*speed = LX_LC_10H;
		break;
	default:
		*speed = 0;
		retval = -EINVAL;
		break;
	}

	return retval;
}

static int alc_post_phy_link(struct alx_hw *hw, bool az_en,
			     bool link_up, u8 speed)
{
	return l1c_post_phy_link(hw, az_en, link_up, speed);
}

/*
 * 1. stop_mac
 * 2. reset mac & dma by reg1400(MASTER)
 * 3. control speed/duplex, hash-alg
 * 4. clock switch setting
 */
static int alc_reset_mac(struct alx_hw *hw)
{
	int retval = 0;

	if (l1c_reset_mac(hw)) {
		alx_hw_err(hw, "error when reset mac\n");
		retval = -EINVAL;
	}
	return retval;
}


static int alc_start_mac(struct alx_hw *hw)
{
	u16 en_ctrl = 0;
	int retval = 0;

	/* set link speed param */
	switch (hw->link_speed) {
	case LX_LC_1000F:
		en_ctrl |= LX_MACSPEED_1000;
		/* fall through */
	case LX_LC_100F:
	case LX_LC_10F:
		en_ctrl |= LX_MACDUPLEX_FULL;
		break;
	}

	/* set fc param*/
	switch (hw->cur_fc_mode) {
	case alx_fc_full:
		en_ctrl |= LX_FC_RXEN; /* Flow Control RX Enable */
		en_ctrl |= LX_FC_TXEN; /* Flow Control TX Enable */
		break;
	case alx_fc_rx_pause:
		en_ctrl |= LX_FC_RXEN; /* Flow Control RX Enable */
		break;
	case alx_fc_tx_pause:
		en_ctrl |= LX_FC_TXEN; /* Flow Control TX Enable */
		break;
	default:
		break;
	}

	if (hw->fc_single_pause)
		en_ctrl |= LX_SINGLE_PAUSE;


	en_ctrl |= LX_FLT_DIRECT; /* RX Enable; and TX Always Enable */
	en_ctrl |= LX_ADD_FCS;

	en_ctrl |= hw->flags & ALX_HW_FLAG_LX_MASK;

	if (l1c_enable_mac(hw, true, en_ctrl)) {
		alx_hw_err(hw, "error when start mac\n");
		retval = -EINVAL;
	}
	return retval;
}


/*
 * 1. stop RXQ (reg15A0) and TXQ (reg1590)
 * 2. stop MAC (reg1480)
 */
static int alc_stop_mac(struct alx_hw *hw)
{
	int retval = 0;

	if (l1c_enable_mac(hw, false, 0)) {
		alx_hw_err(hw, "error when stop mac\n");
		retval = -EINVAL;
	}
	return retval;
}


static int alc_init_mac(struct alx_hw *hw, u16 rxbuf_sz, u16 rx_qnum,
			  u16 rxring_sz, u16 tx_qnum,  u16 txring_sz)
{
	int retval;

	retval = l1c_init_mac(hw, hw->mac_addr, hw->dma.tpdmem_hi[0],
			      hw->dma.tpdmem_lo, tx_qnum, txring_sz,
			      hw->dma.rfdmem_hi[0], hw->dma.rfdmem_lo[0],
			      hw->dma.rrdmem_lo[0], rxring_sz, rxbuf_sz,
			      hw->smb_timer, hw->mtu + ALX_ETH_LENGTH_OF_HEADER,
			      hw->imt_mod, true);
	if (retval)
		alx_hw_err(hw, "error when config mac\n");

	return retval;
}


static int alc_reset_pcie(struct alx_hw *hw, bool l0s_en, bool l1_en)
{
	int retval = 0;

	if (!CHK_HW_FLAG(L0S_CAP))
		l0s_en = false;

	if (l0s_en)
		SET_HW_FLAG(L0S_EN);
	else
		CLI_HW_FLAG(L0S_EN);


	if (!CHK_HW_FLAG(L1_CAP))
		l1_en = false;

	if (l1_en)
		SET_HW_FLAG(L1_EN);
	else
		CLI_HW_FLAG(L1_EN);

	if (l1c_reset_pcie(hw, l0s_en, l1_en)) {
		alx_hw_err(hw, "error when reset pcie\n");
		retval = -EINVAL;
	}
	return retval;
}


static int alc_config_aspm(struct alx_hw *hw, bool l0s_en, bool l1_en)
{
	u8  link_stat;
	int retval = 0;

	if (!CHK_HW_FLAG(L0S_CAP))
		l0s_en = false;

	if (l0s_en)
		SET_HW_FLAG(L0S_EN);
	else
		CLI_HW_FLAG(L0S_EN);

	if (!CHK_HW_FLAG(L1_CAP))
		l1_en = false;

	if (l1_en)
		SET_HW_FLAG(L1_EN);
	else
		CLI_HW_FLAG(L1_EN);

	link_stat = hw->link_up ? LX_LC_ALL : 0;
	if (l1c_enable_aspm(hw, l0s_en, l1_en, link_stat)) {
		alx_hw_err(hw, "error when enable aspm\n");
		retval = -EINVAL;
	}
	return retval;
}


static int alc_config_wol(struct alx_hw *hw, u32 wufc)
{
	u32 wol = 0;

	/* turn on magic packet event */
	if (wufc & ALX_WOL_MAGIC) {
		wol |= L1C_WOL0_MAGIC_EN | L1C_WOL0_PME_MAGIC_EN;
		if (hw->mac_type == alx_mac_l2cb_v1 &&
		    hw->pci_revid == ALX_REV_ID_AR8152_V1_1) {
			wol |= L1C_WOL0_PATTERN_EN | L1C_WOL0_PME_PATTERN_EN;
		}
		/* magic packet maybe Broadcast&multicast&Unicast frame
		 * move to l1c_powersaving
		 */
	}

	/* turn on link up event */
	if (wufc & ALX_WOL_PHY) {
		wol |=  L1C_WOL0_LINK_EN | L1C_WOL0_PME_LINK;
		/* only link up can wake up */
		alc_write_phy_reg(hw, L1C_MII_IER, L1C_IER_LINK_UP);
	}

	alx_mem_w32(hw, L1C_WOL0, wol);
	return 0;
}


static void alc_update_mac_filter(struct alx_hw *hw)
{
	u32 mac;
  u32 flg_hw_map[] = {
		ALX_HW_FLAG_BROADCAST_EN, L1C_MAC_CTRL_BRD_EN,
		ALX_HW_FLAG_VLANSTRIP_EN, L1C_MAC_CTRL_VLANSTRIP,
		ALX_HW_FLAG_PROMISC_EN, L1C_MAC_CTRL_PROMISC_EN,
		ALX_HW_FLAG_MULTIALL_EN, L1C_MAC_CTRL_MULTIALL_EN,
		ALX_HW_FLAG_LOOPBACK_EN, L1C_MAC_CTRL_LPBACK_EN
		};
	int i;

	alx_mem_r32(hw, L1C_MAC_CTRL, &mac);

	for (i = 0; i < ARRAY_SIZE(flg_hw_map); i += 2) {
		if (hw->flags & flg_hw_map[i])
			mac |= flg_hw_map[i + 1];
		else
			mac &= ~flg_hw_map[i + 1];
	}

	alx_mem_w32(hw, L1C_MAC_CTRL, mac);
}


static int alc_config_pow_save(struct alx_hw *hw, u8 speed, bool wol_en,
			       bool tx_en, bool rx_en, bool pws_en)
{
	int retval = 0;

	if (l1c_powersaving(hw, speed, wol_en, tx_en, rx_en, pws_en)) {
		alx_hw_err(hw, "error when set power saving\n");
		retval = -EINVAL;
	}
	return retval;
}


/* RAR, Multicast, VLAN */
static void alc_set_mac_addr(struct alx_hw *hw, u8 *addr)
{
	u32 sta;

	/*
	 * for example: 00-0B-6A-F6-00-DC
	 * 0<-->6AF600DC, 1<-->000B.
	 */

	/* low dword */
	sta = addr[2] << 24 | addr[3] << 16 | addr[4] << 8  | addr[5];
	alx_mem_w32(hw, L1C_STAD0, sta);

	/* high dword */
	sta = addr[0] << 8 | addr[1];
	alx_mem_w32(hw, L1C_STAD1, sta);
}

static int alc_get_mac_addr(struct alx_hw *hw, u8 *addr)
{
	int retval = 0;

	if (l1c_get_perm_macaddr(hw, addr)) {
		alx_hw_err(hw, "error when get permanent mac address\n");
		retval = -EINVAL;
	}
	return retval;
}

static int alc_set_mc_addr(struct alx_hw *hw, u8 *addr)
{
	u32 crc32, bit, reg, mta;

  /*
   * For multicast-all, set every bit in the hash table.
   * Probably not needed, but just a precaution in case the MC-all
   * register flag gets ignored.
   */
  if (unlikely(hw->flags & ALX_HW_FLAG_MULTIALL_EN))
  {
    alx_mem_w32(hw, L1C_HASH_TBL0, 0xffffffff);
    alx_mem_w32(hw, L1C_HASH_TBL1, 0xffffffff);
    
    return 0;
  }

	/*
	 * set hash value for a multicast address hash calcu processing.
	 * 1. calcu 32bit CRC for multicast address
	 * 2. reverse crc with MSB to LSB
	 */
	crc32 = ALX_ETH_CRC(addr, ALX_ETH_LENGTH_OF_ADDRESS);

	/*
	 * The HASH Table  is a register array of 2 32-bit registers.
	 * It is treated like an array of 64 bits.  We want to set
	 * bit BitArray[hash_value]. So we figure out what register
	 * the bit is in, read it, OR in the new bit, then write
	 * back the new value.  The register is determined by the
	 * upper 7 bits of the hash value and the bit within that
	 * register are determined by the lower 5 bits of the value.
	 */
	reg = (crc32 >> 31) & 0x1;
	bit = (crc32 >> 26) & 0x1F;

	alx_mem_r32(hw, L1C_HASH_TBL0 + (reg<<2), &mta);
	mta |= (0x1 << bit);
	alx_mem_w32(hw, L1C_HASH_TBL0 + (reg<<2), mta);
	return 0;
}


static int alc_clear_mc_addr(struct alx_hw *hw)
{
	alx_mem_w32(hw, L1C_HASH_TBL0, 0);
	alx_mem_w32(hw, L1C_HASH_TBL1, 0);
	return 0;
}


/* RTX */
static void alc_config_tx(struct alx_hw *hw)
{
}


/* INTR */
static int alc_ack_phy_intr(struct alx_hw *hw)
{
	u16 isr;
	return alc_read_phy_reg(hw, L1C_MII_ISR, &isr);
}


static int alc_enable_legacy_intr(struct alx_hw *hw)
{
	alx_mem_w32(hw, L1C_ISR, ~((u32) L1C_ISR_DIS));
	alx_mem_w32(hw, L1C_IMR, hw->intr_mask);
	return 0;
}


static int alc_disable_legacy_intr(struct alx_hw *hw)
{
	alx_mem_w32(hw, L1C_ISR, L1C_ISR_DIS);
	alx_mem_w32(hw, L1C_IMR, 0);
	alx_mem_flush(hw);
	return 0;
}


/*
 * NV Ram
 */
static int alc_check_nvram(struct alx_hw *hw, bool *exist)
{
	*exist = false;
	return 0;
}


static int alc_read_nvram(struct alx_hw *hw, u16 offset, u32 *data)
{
	int i;
	u32 ectrl1, ectrl2, edata;
	int retval = 0;

	if (offset & 0x3)
		return retval; /* address do not align */

	alx_mem_r32(hw, L1C_EFUSE_CTRL2, &ectrl2);
	if (!(ectrl2 & L1C_EFUSE_CTRL2_CLK_EN))
		alx_mem_w32(hw, L1C_EFUSE_CTRL2, ectrl2|L1C_EFUSE_CTRL2_CLK_EN);

	alx_mem_w32(hw, L1C_EFUSE_DATA, 0);
	ectrl1 = FIELDL(L1C_EFUSE_CTRL_ADDR, offset);
	alx_mem_w32(hw, L1C_EFUSE_CTRL, ectrl1);

	for (i = 0; i < 10; i++) {
		udelay(100);
		alx_mem_r32(hw, L1C_EFUSE_CTRL, &ectrl1);
		if (ectrl1 & L1C_EFUSE_CTRL_FLAG)
			break;
	}
	if (ectrl1 & L1C_EFUSE_CTRL_FLAG) {
		alx_mem_r32(hw, L1C_EFUSE_CTRL, &ectrl1);
		alx_mem_r32(hw, L1C_EFUSE_DATA, &edata);
		edata = (ectrl1 << 16) | (edata >> 16);
		*data = be32_to_cpu(*(__be32 *)&edata);
		return retval;
	}

	if (!(ectrl2 & L1C_EFUSE_CTRL2_CLK_EN))
		alx_mem_w32(hw, L1C_EFUSE_CTRL2, ectrl2);

	return retval;
}


static int alc_write_nvram(struct alx_hw *hw, u16 offset, u32 data)
{
	return 0;
}


/* fc */
static int alc_get_fc_mode(struct alx_hw *hw, enum alx_fc_mode *mode)
{
	u16 bmsr, giga;
	int i;
	int retval = 0;

	for (i = 0; i < ALX_MAX_SETUP_LNK_CYCLE; i++) {
		alc_read_phy_reg(hw, MII_BMSR, &bmsr);
		alc_read_phy_reg(hw, MII_BMSR, &bmsr);
		if (bmsr & BMSR_LSTATUS) {
			/* Read phy Specific Status Register (17) */
			retval = alc_read_phy_reg(hw, L1C_MII_GIGA_PSSR, &giga);
			if (retval)
				return retval;

			if (!(giga & L1C_GIGA_PSSR_SPD_DPLX_RESOLVED)) {
				alx_hw_err(hw,
					"error for speed duplex resolved\n");
				return -EINVAL;
			}

			if ((giga & L1C_GIGA_PSSR_FC_TXEN) &&
			    (giga & L1C_GIGA_PSSR_FC_RXEN)) {
				*mode = alx_fc_full;
			} else if (giga & L1C_GIGA_PSSR_FC_TXEN) {
				*mode = alx_fc_tx_pause;
			} else if (giga & L1C_GIGA_PSSR_FC_RXEN) {
				*mode = alx_fc_rx_pause;
			} else {
				*mode = alx_fc_none;
			}
			break;
		}
		mdelay(100);
	}

	if (i == ALX_MAX_SETUP_LNK_CYCLE) {
		alx_hw_err(hw, "error when get flow control mode\n");
		retval = -EINVAL;
	}
	return retval;
}


static int alc_config_fc(struct alx_hw *hw)
{
	u32 mac;
	int retval = 0;

	if (hw->disable_fc_autoneg) {
		hw->fc_was_autonegged = false;
		hw->cur_fc_mode = hw->req_fc_mode;
	} else {
		hw->fc_was_autonegged = true;
		retval = alc_get_fc_mode(hw, &hw->cur_fc_mode);
		if (retval)
			return retval;
	}

	alx_mem_r32(hw, L1C_MAC_CTRL, &mac);

	switch (hw->cur_fc_mode) {
	case alx_fc_none: /* 0 */
		mac &= ~(L1C_MAC_CTRL_RXFC_EN | L1C_MAC_CTRL_TXFC_EN);
		break;
	case alx_fc_rx_pause: /* 1 */
		mac &= ~L1C_MAC_CTRL_TXFC_EN;
		mac |= L1C_MAC_CTRL_RXFC_EN;
		break;
	case alx_fc_tx_pause: /* 2 */
		mac |= L1C_MAC_CTRL_TXFC_EN;
		mac &= ~L1C_MAC_CTRL_RXFC_EN;
		break;
	case alx_fc_full: /* 3 */
	case alx_fc_default: /* 4 */
		mac |= (L1C_MAC_CTRL_TXFC_EN | L1C_MAC_CTRL_RXFC_EN);
		break;
	default:
		alx_hw_err(hw, "flow control param set incorrectly\n");
		return -EINVAL;
	}

	alx_mem_w32(hw, L1C_MAC_CTRL, mac);
	return retval;
}


/* ethtool */
static void alc_get_ethtool_regs(struct alx_hw *hw, void *buff)
{
	int i;
	u32 *val = (u32 *)buff;
	static const int reg[] = {
		/* 0 */
		L1C_LNK_CAP, L1C_PMCTRL, L1C_HALFD, L1C_SLD, L1C_MASTER,
		L1C_MANU_TIMER, L1C_IRQ_MODU_TIMER, L1C_PHY_CTRL, L1C_LNK_CTRL,
		L1C_MAC_STS,

		/* 10 */
		L1C_MDIO, L1C_SERDES, L1C_MAC_CTRL, L1C_GAP, L1C_STAD0,
		L1C_STAD1, L1C_HASH_TBL0, L1C_HASH_TBL1, L1C_RXQ0, L1C_RXQ1,

		/* 20 */
		L1C_RXQ2, L1C_RXQ3, L1C_TXQ0, L1C_TXQ1, L1C_TXQ2, L1C_MTU,
		L1C_WOL0, L1C_WOL1, L1C_WOL2,
	};

	for (i = 0; i < ARRAY_SIZE(reg); i++)
		alx_mem_r32(hw, reg[i], &val[i]);
}


/******************************************************************************/
static int alc_get_hw_capabilities(struct alx_hw *hw)
{
	/*
	 * because there is some hareware error on some platforms, just disable
	 * this feature when link connected.
	 */
	CLI_HW_FLAG(L0S_CAP);
	CLI_HW_FLAG(L1_CAP);

	if ((hw->mac_type == alx_mac_l1c) ||
	    (hw->mac_type == alx_mac_l1d_v1) ||
	    (hw->mac_type == alx_mac_l1d_v2))
		SET_HW_FLAG(GIGA_CAP);

	SET_HW_FLAG(PWSAVE_CAP);
  
  SET_HW_FLAG(BROADCAST_EN);
	return 0;
}


/* alc_set_hw_info */
static int alc_set_hw_infos(struct alx_hw *hw)
{
	hw->rxstat_reg = 0x1700;
	hw->rxstat_sz  = 0x60;
	hw->txstat_reg = 0x1760;
	hw->txstat_sz  = 0x68;

	hw->rx_prod_reg[0] = L1C_RFD_PIDX;
	hw->rx_cons_reg[0] = L1C_RFD_CIDX;

	hw->tx_prod_reg[0] = L1C_TPD_PRI0_PIDX;
	hw->tx_cons_reg[0] = L1C_TPD_PRI0_CIDX;
	hw->tx_prod_reg[1] = L1C_TPD_PRI1_PIDX;
	hw->tx_cons_reg[1] = L1C_TPD_PRI1_CIDX;

	hw->hwreg_sz = 0x80;
	hw->eeprom_sz = 0;

	return 0;
}


/**
 *  alc_init_hw_callbacks - Inits func ptrs and MAC type
 *  @hw: pointer to hardware structure
 **/
int alc_init_hw_callbacks(struct alx_hw *hw)
{
	/* NIC */
	hw->cbs.identify_nic   = &alc_identify_nic;
	/* MAC*/
	hw->cbs.reset_mac      = &alc_reset_mac;
	hw->cbs.start_mac      = &alc_start_mac;
	hw->cbs.stop_mac       = &alc_stop_mac;
	hw->cbs.init_mac       = &alc_init_mac;
	hw->cbs.get_mac_addr   = &alc_get_mac_addr;
	hw->cbs.set_mac_addr   = &alc_set_mac_addr;
	hw->cbs.set_mc_addr    = &alc_set_mc_addr;
	hw->cbs.clear_mc_addr  = &alc_clear_mc_addr;

	/* PHY */
	hw->cbs.init_phy          = &alc_init_phy;
	hw->cbs.reset_phy         = &alc_reset_phy;
	hw->cbs.read_phy_reg      = &alc_read_phy_reg;
	hw->cbs.write_phy_reg     = &alc_write_phy_reg;
	hw->cbs.check_phy_link    = &alc_check_phy_link;
	hw->cbs.setup_phy_link    = &alc_setup_phy_link;
  hw->cbs.post_phy_link     = &alc_post_phy_link;

	/* Interrupt */
	hw->cbs.ack_phy_intr	= &alc_ack_phy_intr;
	hw->cbs.enable_legacy_intr  = &alc_enable_legacy_intr;
	hw->cbs.disable_legacy_intr = &alc_disable_legacy_intr;

	/* Configure */
	hw->cbs.config_tx         = &alc_config_tx;
	hw->cbs.config_fc         = &alc_config_fc;
	hw->cbs.config_aspm       = &alc_config_aspm;
	hw->cbs.config_wol        = &alc_config_wol;
	hw->cbs.update_mac_filter = &alc_update_mac_filter;
	hw->cbs.config_pow_save   = &alc_config_pow_save;
	hw->cbs.reset_pcie        = &alc_reset_pcie;

	/* NVRam */
	hw->cbs.check_nvram = &alc_check_nvram;
	hw->cbs.read_nvram  = &alc_read_nvram;
	hw->cbs.write_nvram = &alc_write_nvram;

	/* Others */
	hw->cbs.get_ethtool_regs = alc_get_ethtool_regs;

#ifdef CONFIG_ALX_DEBUGFS
	hw->cbs.read_ext_phy_reg  = &alc_read_ext_phy_reg;
	hw->cbs.write_ext_phy_reg = &alc_write_ext_phy_reg;
#endif

	/* get hw capabilitites to HW->flags */
	alc_get_hw_capabilities(hw);
	alc_set_hw_infos(hw);

	alx_hw_info(hw, "HW Flags = 0x%x\n", (uint)hw->flags);
	return 0;
}
