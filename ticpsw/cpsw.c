/*
 * Texas Instruments Ethernet Switch Driver
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/net_tstamp.h>
#include <linux/phy.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_device.h>

#include "cpsw.h"

#include "cpsw_ale.h"
#include "cpts.h"
#include "davinci_cpdma.h"

#define RX_RING_SIZE 8

/*static inline void rtskb_tx_timestamp(struct rtskb *skb){
	if (skb->xmit_stamp)
		*skb->xmit_stamp = cpu_to_be64(rtdm_clock_read() + *skb->xmit_stamp);
}*/

#define AM33XX_CTRL_MAC_LO_REG(offset, id) ((offset) + 0x8 * (id))
#define AM33XX_CTRL_MAC_HI_REG(offset, id) ((offset) + 0x8 * (id) + 0x4)

int cpsw_am33xx_cm_get_macid(struct device *dev, u16 offset, int slave,
			     u8 *mac_addr)
{
	u32 macid_lo;
	u32 macid_hi;
	struct regmap *syscon;

	syscon = syscon_regmap_lookup_by_phandle(dev->of_node, "syscon");
	if (IS_ERR(syscon)) {
		if (PTR_ERR(syscon) == -ENODEV)
			return 0;
		return PTR_ERR(syscon);
	}

	regmap_read(syscon, AM33XX_CTRL_MAC_LO_REG(offset, slave),
		    &macid_lo);
	regmap_read(syscon, AM33XX_CTRL_MAC_HI_REG(offset, slave),
		    &macid_hi);

	mac_addr[5] = (macid_lo >> 8) & 0xff;
	mac_addr[4] = macid_lo & 0xff;
	mac_addr[3] = (macid_hi >> 24) & 0xff;
	mac_addr[2] = (macid_hi >> 16) & 0xff;
	mac_addr[1] = (macid_hi >> 8) & 0xff;
	mac_addr[0] = macid_hi & 0xff;

	return 0;
}
EXPORT_SYMBOL_GPL(cpsw_am33xx_cm_get_macid);

static inline int rtskb_tailroom(const struct rtskb *skb){
	return skb->end - skb->tail;
}

#define CPSW_DEBUG	(NETIF_MSG_HW		| NETIF_MSG_WOL		| \
			 NETIF_MSG_DRV		| NETIF_MSG_LINK	| \
			 NETIF_MSG_IFUP		| NETIF_MSG_INTR	| \
			 NETIF_MSG_PROBE	| NETIF_MSG_TIMER	| \
			 NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	| \
			 NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	| \
			 NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	| \
			 NETIF_MSG_RX_STATUS)

#define cpsw_info(priv, type, format, ...)		\
do {								\
	if (netif_msg_##type(priv) && net_ratelimit())		\
		dev_info(priv->dev, format, ## __VA_ARGS__);	\
} while (0)

#define cpsw_err(priv, type, format, ...)		\
do {								\
	if (netif_msg_##type(priv) && net_ratelimit())		\
		dev_err(priv->dev, format, ## __VA_ARGS__);	\
} while (0)

#define cpsw_dbg(priv, type, format, ...)		\
do {								\
	if (netif_msg_##type(priv) && net_ratelimit())		\
		dev_dbg(priv->dev, format, ## __VA_ARGS__);	\
} while (0)

#define cpsw_notice(priv, type, format, ...)		\
do {								\
	if (netif_msg_##type(priv) && net_ratelimit())		\
		dev_notice(priv->dev, format, ## __VA_ARGS__);	\
} while (0)

#define ALE_ALL_PORTS		0x7

#define CPSW_MAJOR_VERSION(reg)		(reg >> 8 & 0x7)
#define CPSW_MINOR_VERSION(reg)		(reg & 0xff)
#define CPSW_RTL_VERSION(reg)		((reg >> 11) & 0x1f)

#define CPSW_VERSION_1		0x19010a
#define CPSW_VERSION_2		0x19010c

#define HOST_PORT_NUM		0
#define SLIVER_SIZE		0x40

#define CPSW1_HOST_PORT_OFFSET	0x028
#define CPSW1_SLAVE_OFFSET	0x050
#define CPSW1_SLAVE_SIZE	0x040
#define CPSW1_CPDMA_OFFSET	0x100
#define CPSW1_STATERAM_OFFSET	0x200
#define CPSW1_CPTS_OFFSET	0x500
#define CPSW1_ALE_OFFSET	0x600
#define CPSW1_SLIVER_OFFSET	0x700

#define CPSW2_HOST_PORT_OFFSET	0x108
#define CPSW2_SLAVE_OFFSET	0x200
#define CPSW2_SLAVE_SIZE	0x100
#define CPSW2_CPDMA_OFFSET	0x800
#define CPSW2_STATERAM_OFFSET	0xa00
#define CPSW2_CPTS_OFFSET	0xc00
#define CPSW2_ALE_OFFSET	0xd00
#define CPSW2_SLIVER_OFFSET	0xd80
#define CPSW2_BD_OFFSET		0x2000

#define CPDMA_RXTHRESH		0x0c0
#define CPDMA_RXFREE		0x0e0
#define CPDMA_TXHDP		0x00
#define CPDMA_RXHDP		0x20
#define CPDMA_TXCP		0x40
#define CPDMA_RXCP		0x60

#define CPSW_POLL_WEIGHT	64
#define CPSW_MIN_PACKET_SIZE	60
#define CPSW_MAX_PACKET_SIZE	(1500 + 14 + 4 + 4)

#define RX_PRIORITY_MAPPING	0x76543210
#define TX_PRIORITY_MAPPING	0x33221100
#define CPDMA_TX_PRIORITY_MAP	0x76543210

#define cpsw_enable_irq(priv)	\
	do {			\
		u32 i;		\
		for (i = 0; i < priv->num_irqs; i++) \
			rtdm_irq_enable(&priv->irqs_table[i]); \
	} while (0);
#define cpsw_disable_irq(priv)	\
	do {			\
		u32 i;		\
		for (i = 0; i < priv->num_irqs; i++) \
			rtdm_irq_disable(&priv->irqs_table[i]); \
	} while (0);

static int debug_level;
module_param(debug_level, int, 0);
MODULE_PARM_DESC(debug_level, "cpsw debug level (NETIF_MSG bits)");

static int ale_ageout = 10;
module_param(ale_ageout, int, 0);
MODULE_PARM_DESC(ale_ageout, "cpsw ale ageout interval (seconds)");

static int rx_packet_max = CPSW_MAX_PACKET_SIZE;
module_param(rx_packet_max, int, 0);
MODULE_PARM_DESC(rx_packet_max, "maximum receive packet size (bytes)");

struct cpsw_wr_regs {
	u32	id_ver;
	u32	soft_reset;
	u32	control;
	u32	int_control;
	u32	c0_rx_thresh_en;
	u32	c0_rx_en;
	u32	c0_tx_en;
	u32	c0_misc_en;
	u32	c1_rx_thresh_en;
	u32	c1_rx_en;
	u32	c1_tx_en;
	u32	c1_misc_en;
	u32	c2_rx_thresh_en;
	u32	c2_rx_en;
	u32	c2_tx_en;
	u32	c2_misc_en;
	u32	c0_rx_thresh_stat;
	u32	c0_rx_stat;
	u32	c0_tx_stat;
	u32	c0_misc_stat;
	u32	c1_rx_thresh_stat;
	u32	c1_rx_stat;
	u32	c1_tx_stat;
	u32	c1_misc_stat;
	u32	c2_rx_thresh_stat;
	u32	c2_rx_stat;
	u32	c2_tx_stat;
	u32	c2_misc_stat;
	u32	c0_rx_imax;
	u32	c0_tx_imax;
	u32	c1_rx_imax;
	u32	c1_tx_imax;
	u32	c2_rx_imax;
	u32	c2_tx_imax;
	u32	rgmii_ctl;
};

struct cpsw_ss_regs {
	u32	id_ver;
	u32	control;
	u32	soft_reset;
	u32	stat_port_en;
	u32	ptype;
	u32	soft_idle;
	u32	thru_rate;
	u32	gap_thresh;
	u32	tx_start_wds;
	u32	flow_control;
	u32	vlan_ltype;
	u32	ts_ltype;
	u32	dlr_ltype;
};

/* CPSW_PORT_V1 */
#define CPSW1_MAX_BLKS      0x00 /* Maximum FIFO Blocks */
#define CPSW1_BLK_CNT       0x04 /* FIFO Block Usage Count (Read Only) */
#define CPSW1_TX_IN_CTL     0x08 /* Transmit FIFO Control */
#define CPSW1_PORT_VLAN     0x0c /* VLAN Register */
#define CPSW1_TX_PRI_MAP    0x10 /* Tx Header Priority to Switch Pri Mapping */
#define CPSW1_TS_CTL        0x14 /* Time Sync Control */
#define CPSW1_TS_SEQ_LTYPE  0x18 /* Time Sync Sequence ID Offset and Msg Type */
#define CPSW1_TS_VLAN       0x1c /* Time Sync VLAN1 and VLAN2 */

/* CPSW_PORT_V2 */
#define CPSW2_CONTROL       0x00 /* Control Register */
#define CPSW2_MAX_BLKS      0x08 /* Maximum FIFO Blocks */
#define CPSW2_BLK_CNT       0x0c /* FIFO Block Usage Count (Read Only) */
#define CPSW2_TX_IN_CTL     0x10 /* Transmit FIFO Control */
#define CPSW2_PORT_VLAN     0x14 /* VLAN Register */
#define CPSW2_TX_PRI_MAP    0x18 /* Tx Header Priority to Switch Pri Mapping */
#define CPSW2_TS_SEQ_MTYPE  0x1c /* Time Sync Sequence ID Offset and Msg Type */

/* CPSW_PORT_V1 and V2 */
#define SA_LO               0x20 /* CPGMAC_SL Source Address Low */
#define SA_HI               0x24 /* CPGMAC_SL Source Address High */
#define SEND_PERCENT        0x28 /* Transmit Queue Send Percentages */

/* CPSW_PORT_V2 only */
#define RX_DSCP_PRI_MAP0    0x30 /* Rx DSCP Priority to Rx Packet Mapping */
#define RX_DSCP_PRI_MAP1    0x34 /* Rx DSCP Priority to Rx Packet Mapping */
#define RX_DSCP_PRI_MAP2    0x38 /* Rx DSCP Priority to Rx Packet Mapping */
#define RX_DSCP_PRI_MAP3    0x3c /* Rx DSCP Priority to Rx Packet Mapping */
#define RX_DSCP_PRI_MAP4    0x40 /* Rx DSCP Priority to Rx Packet Mapping */
#define RX_DSCP_PRI_MAP5    0x44 /* Rx DSCP Priority to Rx Packet Mapping */
#define RX_DSCP_PRI_MAP6    0x48 /* Rx DSCP Priority to Rx Packet Mapping */
#define RX_DSCP_PRI_MAP7    0x4c /* Rx DSCP Priority to Rx Packet Mapping */

/* Bit definitions for the CPSW2_CONTROL register */
#define PASS_PRI_TAGGED     (1<<24) /* Pass Priority Tagged */
#define VLAN_LTYPE2_EN      (1<<21) /* VLAN LTYPE 2 enable */
#define VLAN_LTYPE1_EN      (1<<20) /* VLAN LTYPE 1 enable */
#define DSCP_PRI_EN         (1<<16) /* DSCP Priority Enable */
#define TS_320              (1<<14) /* Time Sync Dest Port 320 enable */
#define TS_319              (1<<13) /* Time Sync Dest Port 319 enable */
#define TS_132              (1<<12) /* Time Sync Dest IP Addr 132 enable */
#define TS_131              (1<<11) /* Time Sync Dest IP Addr 131 enable */
#define TS_130              (1<<10) /* Time Sync Dest IP Addr 130 enable */
#define TS_129              (1<<9)  /* Time Sync Dest IP Addr 129 enable */
#define TS_BIT8             (1<<8)  /* ts_ttl_nonzero? */
#define TS_ANNEX_D_EN       (1<<4)  /* Time Sync Annex D enable */
#define TS_LTYPE2_EN        (1<<3)  /* Time Sync LTYPE 2 enable */
#define TS_LTYPE1_EN        (1<<2)  /* Time Sync LTYPE 1 enable */
#define TS_TX_EN            (1<<1)  /* Time Sync Transmit Enable */
#define TS_RX_EN            (1<<0)  /* Time Sync Receive Enable */

#define CTRL_TS_BITS \
	(TS_320 | TS_319 | TS_132 | TS_131 | TS_130 | TS_129 | TS_BIT8 | \
	 TS_ANNEX_D_EN | TS_LTYPE1_EN)

#define CTRL_ALL_TS_MASK (CTRL_TS_BITS | TS_TX_EN | TS_RX_EN)
#define CTRL_TX_TS_BITS  (CTRL_TS_BITS | TS_TX_EN)
#define CTRL_RX_TS_BITS  (CTRL_TS_BITS | TS_RX_EN)

/* Bit definitions for the CPSW2_TS_SEQ_MTYPE register */
#define TS_SEQ_ID_OFFSET_SHIFT   (16)    /* Time Sync Sequence ID Offset */
#define TS_SEQ_ID_OFFSET_MASK    (0x3f)
#define TS_MSG_TYPE_EN_SHIFT     (0)     /* Time Sync Message Type Enable */
#define TS_MSG_TYPE_EN_MASK      (0xffff)

/* The PTP event messages - Sync, Delay_Req, Pdelay_Req, and Pdelay_Resp. */
#define EVENT_MSG_BITS ((1<<0) | (1<<1) | (1<<2) | (1<<3))

/* Bit definitions for the CPSW1_TS_CTL register */
#define CPSW_V1_TS_RX_EN		BIT(0)
#define CPSW_V1_TS_TX_EN		BIT(4)
#define CPSW_V1_MSG_TYPE_OFS		16

/* Bit definitions for the CPSW1_TS_SEQ_LTYPE register */
#define CPSW_V1_SEQ_ID_OFS_SHIFT	16

struct cpsw_host_regs {
	u32	max_blks;
	u32	blk_cnt;
	u32	flow_thresh;
	u32	port_vlan;
	u32	tx_pri_map;
	u32	cpdma_tx_pri_map;
	u32	cpdma_rx_chan_map;
};

struct cpsw_sliver_regs {
	u32	id_ver;
	u32	mac_control;
	u32	mac_status;
	u32	soft_reset;
	u32	rx_maxlen;
	u32	__reserved_0;
	u32	rx_pause;
	u32	tx_pause;
	u32	__reserved_1;
	u32	rx_pri_map;
};

struct cpsw_slave {
	void __iomem			*regs;
	struct cpsw_sliver_regs __iomem	*sliver;
	int				slave_num;
	u32				mac_control;
	struct cpsw_slave_data		*data;
	struct phy_device		*phy;
};

static inline u32 slave_read(struct cpsw_slave *slave, u32 offset)
{
	return __raw_readl(slave->regs + offset);
}

static inline void slave_write(struct cpsw_slave *slave, u32 val, u32 offset)
{
	__raw_writel(val, slave->regs + offset);
}

struct cpsw_priv {
	rtdm_lock_t			lock;
	struct platform_device		*pdev;
	struct rtnet_device		*ndev;
	struct resource			*cpsw_res;
	struct resource			*cpsw_wr_res;
	struct device			*dev;
	struct cpsw_platform_data	data;
	struct cpsw_ss_regs __iomem	*regs;
	struct cpsw_wr_regs __iomem	*wr_regs;
	struct cpsw_host_regs __iomem	*host_port_regs;
	u32				msg_enable;
	u32				version;
	struct net_device_stats		stats;
	int				rx_packet_max;
	int				host_port;
	struct clk			*clk;
	u8				mac_addr[ETH_ALEN];
	struct cpsw_slave		*slaves;
	struct cpdma_ctlr		*dma;
	struct cpdma_chan		*txch, *rxch;
	struct cpsw_ale			*ale;
	/* snapshot of IRQ numbers */
	rtdm_irq_t irqs_table[4];
	u32 num_irqs;
	bool irq_enabled;
	struct cpts cpts;

	struct rtskb_pool skb_pool;
};

static inline struct rtskb *dev_alloc_rtskb_ip_align(struct rtnet_device *ndev, unsigned int size){
	struct cpsw_priv *priv = ndev->priv;
	struct rtskb_pool *pool = &priv->skb_pool;
	struct rtskb *skb = alloc_rtskb(size + NET_IP_ALIGN, pool);

	if(skb)
		skb->rtdev = ndev;
	if(NET_IP_ALIGN && skb)
		rtskb_reserve(skb, NET_IP_ALIGN);
	return skb;
}

#define napi_to_priv(napi)	container_of(napi, struct cpsw_priv, napi)
#define for_each_slave(priv, func, arg...)			\
	do {							\
		int idx;					\
		for (idx = 0; idx < (priv)->data.slaves; idx++)	\
			(func)((priv)->slaves + idx, ##arg);	\
	} while (0)

#if 0
static void cpsw_ndo_set_rx_mode(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	if (ndev->flags & IFF_PROMISC) {
		/* Enable promiscuous mode */
		dev_err(priv->dev, "Ignoring Promiscuous mode\n");
		return;
	}

	/* Clear all mcast from ALE */
	cpsw_ale_flush_multicast(priv->ale, ALE_ALL_PORTS << priv->host_port);

	if (!netdev_mc_empty(ndev)) {
		struct netdev_hw_addr *ha;

		/* program multicast address list into ALE register */
		netdev_for_each_mc_addr(ha, ndev) {
			cpsw_ale_add_mcast(priv->ale, (u8 *)ha->addr,
				ALE_ALL_PORTS << priv->host_port, 0, 0);
		}
	}
}
#endif

static void cpsw_intr_enable(struct cpsw_priv *priv)
{
	__raw_writel(0xFF, &priv->wr_regs->c0_tx_en);
	__raw_writel(0xFF, &priv->wr_regs->c0_rx_en);

	cpdma_ctlr_int_ctrl(priv->dma, true);
	return;
}

static void cpsw_intr_disable(struct cpsw_priv *priv)
{
	__raw_writel(0, &priv->wr_regs->c0_tx_en);
	__raw_writel(0, &priv->wr_regs->c0_rx_en);

	cpdma_ctlr_int_ctrl(priv->dma, false);
	return;
}

void cpsw_tx_handler(void *token, int len, int status)
{
	struct rtskb		*skb = token;
	struct rtnet_device	*ndev = skb->rtdev;
	struct cpsw_priv	*priv = ndev->priv;

	//rtdm_printk("cpsw_tx_handler(%x, %d, %d)\n", token, len, status);

	if (unlikely(rtnetif_queue_stopped(ndev)))
		rtnetif_wake_queue(ndev);
	cpts_tx_timestamp(&priv->cpts, skb);
	priv->stats.tx_packets++;
	priv->stats.tx_bytes += len;
	dev_kfree_rtskb(skb);
}

void cpsw_rx_handler(void *token, int len, int status)
{
	struct rtskb		*skb = token;
	struct rtnet_device	*ndev = skb->rtdev;
	struct cpsw_priv	*priv = ndev->priv;
	int			ret = 0;
	nanosecs_abs_t time_stamp = rtdm_clock_read();

	/* free and bail if we are shutting down */
	if (unlikely(!rtnetif_running(ndev)) ||
			unlikely(!rtnetif_carrier_ok(ndev))) {
		dev_kfree_rtskb(skb);
		return;
	}

	if (likely(status >= 0)) {
		skb->time_stamp = time_stamp;
		rtskb_put(skb, len);
		cpts_rx_timestamp(&priv->cpts, skb);
		skb->protocol = rt_eth_type_trans(skb, ndev);
		rtnetif_rx(skb);
		priv->stats.rx_bytes += len;
		priv->stats.rx_packets++;
		skb = NULL;
	}

	if (unlikely(!rtnetif_running(ndev))) {
		if (skb)
			dev_kfree_rtskb(skb);
		return;
	}

	if (likely(!skb)) {
		skb = dev_alloc_rtskb_ip_align(ndev, priv->rx_packet_max);
		if (WARN_ON(!skb))
			return;

		ret = cpdma_chan_submit(priv->rxch, skb, skb->data,
					rtskb_tailroom(skb), GFP_KERNEL);
	}
	WARN_ON(ret < 0);
}

static inline int cpsw_get_slave_port(struct cpsw_priv *priv, u32 slave_num)
{
	if (priv->host_port == 0)
		return slave_num + 1;
	else
		return slave_num;
}

#if 0
static int cpsw_poll(struct cpsw_priv *priv, int budget)
{
	int			num_tx, num_rx, num_total_tx, num_total_rx;
	int			budget_left;

	budget_left = budget;

	/* read status and throw away */
	(void)__raw_readl(&priv->wr_regs->c0_tx_stat);

	/* handle all transmits */
	num_total_tx = 0;
	while (budget_left > 0 &&
		(num_tx = cpdma_chan_process(priv->txch, 128)) > 0) {
		budget_left -= num_tx;
		num_total_tx += num_tx;
	}

	if (num_total_tx > 0 && budget_left > 0)
		cpdma_ctlr_eoi(priv->dma, 0x02);

	/* read status and throw away */
	(void)__raw_readl(&priv->wr_regs->c0_rx_stat);

	/* handle all receives */
	num_total_rx = 0;
	while (budget_left > 0 &&
		(num_rx = cpdma_chan_process(priv->rxch, budget_left)) > 0) {
		budget_left -= num_rx;
		num_total_rx += num_rx;
	}

	if (num_total_rx > 0 && budget_left > 0)
		cpdma_ctlr_eoi(priv->dma, 0x01);

	if ((num_total_rx + num_total_tx) < budget) {
		cpsw_intr_enable(priv);
		if (priv->irq_enabled == false) {
			cpsw_enable_irq(priv);
			priv->irq_enabled = true;
		}
	}

	rtdm_printk("lost packets\n");
	return num_total_rx + num_total_rx;
}
#endif

#if 0
static int cpsw_interrupt(rtdm_irq_t *irq_handle)
{
	struct cpsw_priv *priv = rtdm_irq_get_arg(irq_handle, struct cpsw_priv);

	if (likely(rtnetif_running(priv->ndev))) {
		cpsw_intr_disable(priv);
		if (priv->irq_enabled == true) {
			cpsw_disable_irq(priv);
			priv->irq_enabled = false;
		}
		cpsw_poll(priv, CPSW_POLL_WEIGHT);
	}

	return IRQ_HANDLED;
}
#endif

static int cpsw_rx_thresh_pend_irq(rtdm_irq_t *irq_handle)
{
	/* not handling this interrupt yet */
	return RTDM_IRQ_HANDLED;
}

static int cpsw_rx_pend_irq(rtdm_irq_t *irq_handle)
{
	struct cpsw_priv *priv = rtdm_irq_get_arg(irq_handle, struct cpsw_priv);
	int num_rx, total_rx;
	u32 rx_stat;
	rtdm_lockctx_t context;

	rx_stat = __raw_readl(&priv->wr_regs->c0_rx_stat) & 0xff;
	if (rx_stat == 0)
		return RTDM_IRQ_NONE;

	//rtdm_printk("cpsw_rx_pend_irq: %d\n", rx_stat);

	rtdm_lock_get_irqsave(&priv->lock, context);

	total_rx = 0;
	while ((num_rx = cpdma_chan_process(priv->rxch, RX_RING_SIZE)) > 0)
		total_rx += num_rx;

	cpdma_ctlr_eoi(priv->dma, 0x01);

	rtdm_lock_put_irqrestore(&priv->lock, context);

	if(total_rx > 0)
		rt_mark_stack_mgr(priv->ndev);

	return RTDM_IRQ_HANDLED;
}

static int cpsw_tx_pend_irq(rtdm_irq_t *irq_handle)
{
	struct cpsw_priv *priv = rtdm_irq_get_arg(irq_handle, struct cpsw_priv);
	int num_tx, total_tx;
	u32 tx_stat;

	tx_stat = __raw_readl(&priv->wr_regs->c0_tx_stat) & 0xff;
	if (tx_stat == 0)
		return RTDM_IRQ_NONE;

	//rtdm_printk("cpsw_tx_pend_irq: %d\n", tx_stat);

	total_tx = 0;
	while ((num_tx = cpdma_chan_process(priv->txch, 128)) > 0)
		total_tx += num_tx;

	cpdma_ctlr_eoi(priv->dma, 0x02);

	return RTDM_IRQ_HANDLED;
}

static int cpsw_misc_pend_irq(rtdm_irq_t *irq_handle)
{
	/* not handling this interrupt yet */
	return RTDM_IRQ_HANDLED;
}

static inline void soft_reset(const char *module, void __iomem *reg)
{
	unsigned long timeout = jiffies + HZ;

	__raw_writel(1, reg);
	do {
		cpu_relax();
	} while ((__raw_readl(reg) & 1) && time_after(timeout, jiffies));

	WARN(__raw_readl(reg) & 1, "failed to soft-reset %s\n", module);
}

#define mac_hi(mac)	(((mac)[0] << 0) | ((mac)[1] << 8) |	\
			 ((mac)[2] << 16) | ((mac)[3] << 24))
#define mac_lo(mac)	(((mac)[4] << 0) | ((mac)[5] << 8))

static void cpsw_set_slave_mac(struct cpsw_slave *slave,
			       struct cpsw_priv *priv)
{
	slave_write(slave, mac_hi(priv->mac_addr), SA_HI);
	slave_write(slave, mac_lo(priv->mac_addr), SA_LO);
}

static void _cpsw_adjust_link(struct cpsw_slave *slave,
			      struct cpsw_priv *priv, bool *link)
{
	struct phy_device	*phy = slave->phy;
	u32			mac_control = 0;
	u32			slave_port;

	if (!phy)
		return;

	slave_port = cpsw_get_slave_port(priv, slave->slave_num);

	if (phy->link) {
		mac_control = priv->data.mac_control;

		/* enable forwarding */
		cpsw_ale_control_set(priv->ale, slave_port,
				     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

		if (phy->speed == 1000)
			mac_control |= BIT(7);	/* GIGABITEN	*/
		if (phy->duplex)
			mac_control |= BIT(0);	/* FULLDUPLEXEN	*/

		/* set speed_in input in case RMII mode is used in 100Mbps */
		if (phy->speed == 100)
			mac_control |= BIT(15);

		*link = true;
	} else {
		mac_control = 0;
		/* disable forwarding */
		cpsw_ale_control_set(priv->ale, slave_port,
				     ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);
	}

	if (mac_control != slave->mac_control) {
		phy_print_status(phy);
		__raw_writel(mac_control, &slave->sliver->mac_control);
	}

	slave->mac_control = mac_control;
}

static void cpsw_adjust_link(struct net_device *ndev_)
{
	struct rtnet_device	*ndev = (struct rtnet_device *)ndev_;
	struct cpsw_priv	*priv = ndev->priv;
	bool			link = false;

	for_each_slave(priv, _cpsw_adjust_link, priv, &link);

	if (link) {
		rtnetif_carrier_on(ndev);
		if (rtnetif_running(ndev))
			rtnetif_wake_queue(ndev);
	} else {
		rtnetif_carrier_off(ndev);
		rtnetif_stop_queue(ndev);
	}
}

static inline int __show_stat(char *buf, int maxlen, const char *name, u32 val)
{
	static char *leader = "........................................";

	if (!val)
		return 0;
	else
		return snprintf(buf, maxlen, "%s %s %10d\n", name,
				leader + strlen(name), val);
}

static void cpsw_slave_open(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	char name[32];
	u32 slave_port;

	sprintf(name, "slave-%d", slave->slave_num);

	soft_reset(name, &slave->sliver->soft_reset);

	/* setup priority mapping */
	__raw_writel(RX_PRIORITY_MAPPING, &slave->sliver->rx_pri_map);

	switch (priv->version) {
	case CPSW_VERSION_1:
		slave_write(slave, TX_PRIORITY_MAPPING, CPSW1_TX_PRI_MAP);
		break;
	case CPSW_VERSION_2:
		slave_write(slave, TX_PRIORITY_MAPPING, CPSW2_TX_PRI_MAP);
		break;
	}

	/* setup max packet size, and mac address */
	__raw_writel(priv->rx_packet_max, &slave->sliver->rx_maxlen);
	cpsw_set_slave_mac(slave, priv);

	slave->mac_control = 0;	/* no link yet */

	slave_port = cpsw_get_slave_port(priv, slave->slave_num);

	cpsw_ale_add_mcast(priv->ale, priv->ndev->broadcast,
			   1 << slave_port, 0, ALE_MCAST_FWD_2);

	slave->phy = phy_connect((struct net_device*) priv->ndev, slave->data->phy_id,
				 &cpsw_adjust_link, slave->data->phy_if);
	if (IS_ERR(slave->phy)) {
		dev_err(priv->dev, "phy %s not found on slave %d\n",
			slave->data->phy_id, slave->slave_num);
		slave->phy = NULL;
	} else {
		dev_info(priv->dev, "phy found : id is : 0x%x\n",
			 slave->phy->phy_id);
		phy_start(slave->phy);
	}
}

static void cpsw_init_host_port(struct cpsw_priv *priv)
{
	/* soft reset the controller and initialize ale */
	soft_reset("cpsw", &priv->regs->soft_reset);
	cpsw_ale_start(priv->ale);

	/* switch to vlan unaware mode */
	cpsw_ale_control_set(priv->ale, 0, ALE_VLAN_AWARE, 0);

	/* setup host port priority mapping */
	__raw_writel(CPDMA_TX_PRIORITY_MAP,
		     &priv->host_port_regs->cpdma_tx_pri_map);
	__raw_writel(0, &priv->host_port_regs->cpdma_rx_chan_map);

	cpsw_ale_control_set(priv->ale, priv->host_port,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_ale_add_ucast(priv->ale, priv->mac_addr, priv->host_port, 0);
	cpsw_ale_add_mcast(priv->ale, priv->ndev->broadcast,
			   1 << priv->host_port, 0, ALE_MCAST_FWD_2);
}

static int cpsw_ndo_open(struct rtnet_device *ndev)
{
	struct cpsw_priv *priv = ndev->priv;
	int i, ret;
	u32 reg;

	cpsw_intr_disable(priv);
	rtnetif_carrier_off(ndev);

	pm_runtime_get_sync(&priv->pdev->dev);

	reg = priv->version;

	dev_info(priv->dev, "initializing cpsw version %d.%d (%d)\n",
		 CPSW_MAJOR_VERSION(reg), CPSW_MINOR_VERSION(reg),
		 CPSW_RTL_VERSION(reg));

	/* initialize host and slave ports */
	cpsw_init_host_port(priv);
	for_each_slave(priv, cpsw_slave_open, priv);

	/* setup tx dma to fixed prio and zero offset */
	cpdma_control_set(priv->dma, CPDMA_TX_PRIO_FIXED, 1);
	cpdma_control_set(priv->dma, CPDMA_RX_BUFFER_OFFSET, 0);

	/* disable priority elevation and enable statistics on all ports */
	__raw_writel(0, &priv->regs->ptype);

	/* enable statistics collection only on the host port */
	__raw_writel(0x7, &priv->regs->stat_port_en);

	for (i = 0; i < RX_RING_SIZE; i++) {
		struct rtskb *skb;

		ret = -ENOMEM;
		skb = dev_alloc_rtskb_ip_align(ndev, priv->rx_packet_max);
		if (!skb)
			break;
		ret = cpdma_chan_submit(priv->rxch, skb, skb->data,
					rtskb_tailroom(skb), GFP_KERNEL);
		if (WARN_ON(ret < 0))
			break;
	}
	/* continue even if we didn't manage to submit all receive descs */
	cpsw_info(priv, ifup, "submitted %d rx descriptors\n", i);

	cpdma_ctlr_start(priv->dma);
	cpsw_intr_enable(priv);
	cpdma_ctlr_eoi(priv->dma, 0x01);
	cpdma_ctlr_eoi(priv->dma, 0x02);

	return 0;
}

static void cpsw_slave_stop(struct cpsw_slave *slave, struct cpsw_priv *priv)
{
	if (!slave->phy)
		return;
	phy_stop(slave->phy);
	phy_disconnect(slave->phy);
	slave->phy = NULL;
}

static int cpsw_ndo_stop(struct rtnet_device *ndev)
{
	struct cpsw_priv *priv = ndev->priv;

	cpsw_info(priv, ifdown, "shutting down cpsw device\n");
	if (priv->irq_enabled == true) {
		cpsw_disable_irq(priv);
		priv->irq_enabled = false;
	}
	rtnetif_stop_queue(priv->ndev);
	rtnetif_carrier_off(priv->ndev);
	cpsw_intr_disable(priv);
	cpdma_ctlr_int_ctrl(priv->dma, false);
	cpdma_ctlr_stop(priv->dma);
	cpsw_ale_stop(priv->ale);
	for_each_slave(priv, cpsw_slave_stop, priv);
	pm_runtime_put_sync(&priv->pdev->dev);
	return 0;
}

static netdev_tx_t cpsw_ndo_start_xmit(struct rtskb *skb,
				       struct rtnet_device *ndev)
{
	struct cpsw_priv *priv = ndev->priv;
	int ret;

	if (!rtskb_padto(skb, CPSW_MIN_PACKET_SIZE)) {
		cpsw_err(priv, tx_err, "packet pad failed\n");
		priv->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	#if 0
	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP && priv->cpts.tx_enable)
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	#endif

	rtskb_tx_timestamp(skb);

	ret = cpdma_chan_submit(priv->txch, skb, skb->data,
				skb->len, GFP_KERNEL);
	if (unlikely(ret != 0)) {
		cpsw_err(priv, tx_err, "desc submit failed\n");
		goto fail;
	}

	return NETDEV_TX_OK;
fail:
	priv->stats.tx_dropped++;
	rtnetif_stop_queue(ndev);
	return NETDEV_TX_BUSY;
}

#if 0
static void cpsw_ndo_change_rx_flags(struct net_device *ndev, int flags)
{
	/*
	 * The switch cannot operate in promiscuous mode without substantial
	 * headache.  For promiscuous mode to work, we would need to put the
	 * ALE in bypass mode and route all traffic to the host port.
	 * Subsequently, the host will need to operate as a "bridge", learn,
	 * and flood as needed.  For now, we simply complain here and
	 * do nothing about it :-)
	 */
	if ((flags & IFF_PROMISC) && (ndev->flags & IFF_PROMISC))
		dev_err(&ndev->dev, "promiscuity ignored!\n");

	/*
	 * The switch cannot filter multicast traffic unless it is configured
	 * in "VLAN Aware" mode.  Unfortunately, VLAN awareness requires a
	 * whole bunch of additional logic that this driver does not implement
	 * at present.
	 */
	if ((flags & IFF_ALLMULTI) && !(ndev->flags & IFF_ALLMULTI))
		dev_err(&ndev->dev, "multicast traffic cannot be filtered!\n");
}
#endif

#ifdef CONFIG_TI_CPTS

static void cpsw_hwtstamp_v1(struct cpsw_priv *priv)
{
	struct cpsw_slave *slave = &priv->slaves[priv->data.cpts_active_slave];
	u32 ts_en, seq_id;

	if (!priv->cpts.tx_enable && !priv->cpts.rx_enable) {
		slave_write(slave, 0, CPSW1_TS_CTL);
		return;
	}

	seq_id = (30 << CPSW_V1_SEQ_ID_OFS_SHIFT) | ETH_P_1588;
	ts_en = EVENT_MSG_BITS << CPSW_V1_MSG_TYPE_OFS;

	if (priv->cpts.tx_enable)
		ts_en |= CPSW_V1_TS_TX_EN;

	if (priv->cpts.rx_enable)
		ts_en |= CPSW_V1_TS_RX_EN;

	slave_write(slave, ts_en, CPSW1_TS_CTL);
	slave_write(slave, seq_id, CPSW1_TS_SEQ_LTYPE);
}

static void cpsw_hwtstamp_v2(struct cpsw_priv *priv)
{
	struct cpsw_slave *slave = &priv->slaves[priv->data.cpts_active_slave];
	u32 ctrl, mtype;

	ctrl = slave_read(slave, CPSW2_CONTROL);
	ctrl &= ~CTRL_ALL_TS_MASK;

	if (priv->cpts.tx_enable)
		ctrl |= CTRL_TX_TS_BITS;

	if (priv->cpts.rx_enable)
		ctrl |= CTRL_RX_TS_BITS;

	mtype = (30 << TS_SEQ_ID_OFFSET_SHIFT) | EVENT_MSG_BITS;

	slave_write(slave, mtype, CPSW2_TS_SEQ_MTYPE);
	slave_write(slave, ctrl, CPSW2_CONTROL);
	__raw_writel(ETH_P_1588, &priv->regs->ts_ltype);
}

static int cpsw_hwtstamp_ioctl(struct net_device *dev, struct ifreq *ifr)
{
	struct cpsw_priv *priv = netdev_priv(dev);
	struct cpts *cpts = &priv->cpts;
	struct hwtstamp_config cfg;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	/* reserved for future extensions */
	if (cfg.flags)
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
		cpts->tx_enable = 0;
		break;
	case HWTSTAMP_TX_ON:
		cpts->tx_enable = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		cpts->rx_enable = 0;
		break;
	case HWTSTAMP_FILTER_ALL:
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		return -ERANGE;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		cpts->rx_enable = 1;
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		break;
	default:
		return -ERANGE;
	}

	switch (priv->version) {
	case CPSW_VERSION_1:
		cpsw_hwtstamp_v1(priv);
		break;
	case CPSW_VERSION_2:
		cpsw_hwtstamp_v2(priv);
		break;
	default:
		return -ENOTSUPP;
	}

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

#endif /*CONFIG_TI_CPTS*/

#if 0
static int cpsw_ndo_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
	if (!rtnetif_running(dev))
		return -EINVAL;

#ifdef CONFIG_TI_CPTS
	if (cmd == SIOCSHWTSTAMP)
		return cpsw_hwtstamp_ioctl(dev, req);
#endif
	return -ENOTSUPP;
}
#endif

#if 0
static void cpsw_ndo_tx_timeout(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	cpsw_err(priv, tx_err, "transmit timeout, restarting dma\n");
	priv->stats.tx_errors++;
	cpsw_intr_disable(priv);
	cpdma_ctlr_int_ctrl(priv->dma, false);
	cpdma_chan_stop(priv->txch);
	cpdma_chan_start(priv->txch);
	cpdma_ctlr_int_ctrl(priv->dma, true);
	cpsw_intr_enable(priv);
	cpdma_ctlr_eoi(priv->dma, 0x01);
	cpdma_ctlr_eoi(priv->dma, 0x02);
}
#endif

static struct net_device_stats *cpsw_ndo_get_stats(struct rtnet_device *ndev)
{
	struct cpsw_priv *priv = ndev->priv;
	return &priv->stats;
}

#if 0
#ifdef CONFIG_NET_POLL_CONTROLLER
static void cpsw_ndo_poll_controller(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);

	cpsw_intr_disable(priv);
	cpdma_ctlr_int_ctrl(priv->dma, false);
	if (!priv->data.disable_napi)
		cpsw_interrupt(ndev->irq, priv);
	else {
		/* bah! */
		cpsw_rx_pend_irq(ndev->irq, priv);
		cpsw_tx_pend_irq(ndev->irq, priv);
	}
	cpdma_ctlr_int_ctrl(priv->dma, true);
	cpsw_intr_enable(priv);
	cpdma_ctlr_eoi(priv->dma, 0x01);
	cpdma_ctlr_eoi(priv->dma, 0x02);
}
#endif
#endif

#if 0
static struct net_device_ops cpsw_netdev_ops = {
	.ndo_open		= cpsw_ndo_open,
	.ndo_stop		= cpsw_ndo_stop,
	.ndo_start_xmit		= cpsw_ndo_start_xmit,
	.ndo_change_rx_flags	= cpsw_ndo_change_rx_flags,
	.ndo_do_ioctl		= cpsw_ndo_ioctl,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_tx_timeout		= cpsw_ndo_tx_timeout,
	.ndo_get_stats		= cpsw_ndo_get_stats,
	.ndo_set_rx_mode	= cpsw_ndo_set_rx_mode,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= cpsw_ndo_poll_controller,
#endif
};
#endif

static void cpsw_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	strcpy(info->driver, "TI CPSW Driver v1.0");
	strcpy(info->version, "1.0");
	strcpy(info->bus_info, priv->pdev->name);
}

static u32 cpsw_get_msglevel(struct net_device *ndev)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	return priv->msg_enable;
}

static void cpsw_set_msglevel(struct net_device *ndev, u32 value)
{
	struct cpsw_priv *priv = netdev_priv(ndev);
	priv->msg_enable = value;
}

static int cpsw_get_ts_info(struct net_device *ndev,
			    struct ethtool_ts_info *info)
{
#ifdef CONFIG_TI_CPTS
	struct cpsw_priv *priv = netdev_priv(ndev);

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;
	info->phc_index = priv->cpts.phc_index;
	info->tx_types =
		(1 << HWTSTAMP_TX_OFF) |
		(1 << HWTSTAMP_TX_ON);
	info->rx_filters =
		(1 << HWTSTAMP_FILTER_NONE) |
		(1 << HWTSTAMP_FILTER_PTP_V2_EVENT);
#else
	info->so_timestamping =
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE;
	info->phc_index = -1;
	info->tx_types = 0;
	info->rx_filters = 0;
#endif
	return 0;
}

static const struct ethtool_ops cpsw_ethtool_ops = {
	.get_drvinfo	= cpsw_get_drvinfo,
	.get_msglevel	= cpsw_get_msglevel,
	.set_msglevel	= cpsw_set_msglevel,
	.get_link	= ethtool_op_get_link,
	.get_ts_info	= cpsw_get_ts_info,
};

static void cpsw_slave_init(struct cpsw_slave *slave, struct cpsw_priv *priv,
			    u32 slave_reg_ofs, u32 sliver_reg_ofs)
{
	void __iomem		*regs = priv->regs;
	int			slave_num = slave->slave_num;
	struct cpsw_slave_data	*data = priv->data.slave_data + slave_num;

	slave->data	= data;
	slave->regs	= regs + slave_reg_ofs;
	slave->sliver	= regs + sliver_reg_ofs;
}

static int cpsw_probe_dt(struct cpsw_platform_data *data,
			 struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device_node *slave_node;
	int i = 0, ret;
	u32 prop;

	if (!node)
		return -EINVAL;

	if (of_property_read_u32(node, "slaves", &prop)) {
		dev_err(&pdev->dev, "Missing slaves property in the DT.\n");
		return -EINVAL;
	}
	data->slaves = prop;

	if (of_property_read_u32(node, "active_slave", &prop)) {
		dev_err(&pdev->dev, "Missing active_slave property in the DT.\n");
		return -EINVAL;
	}
	data->active_slave = prop;

	if (of_property_read_u32(node, "cpts_clock_mult", &prop)) {
		dev_err(&pdev->dev, "Missing cpts_clock_mult property in the DT.\n");
		return -EINVAL;
	}
	data->cpts_clock_mult = prop;

	if (of_property_read_u32(node, "cpts_clock_shift", &prop)) {
		dev_err(&pdev->dev, "Missing cpts_clock_shift property in the DT.\n");
		return -EINVAL;
	}
	data->cpts_clock_shift = prop;

	data->slave_data = devm_kzalloc(&pdev->dev, data->slaves
					* sizeof(struct cpsw_slave_data),
					GFP_KERNEL);
	if (!data->slave_data)
		return -ENOMEM;

	if (of_property_read_u32(node, "cpdma_channels", &prop)) {
		dev_err(&pdev->dev, "Missing cpdma_channels property in the DT.\n");
		return -EINVAL;
	}
	data->channels = prop;

	if (of_property_read_u32(node, "ale_entries", &prop)) {
		dev_err(&pdev->dev, "Missing ale_entries property in the DT.\n");
		return -EINVAL;
	}
	data->ale_entries = prop;

	if (of_property_read_u32(node, "bd_ram_size", &prop)) {
		dev_err(&pdev->dev, "Missing bd_ram_size property in the DT.\n");
		return -EINVAL;
	}
	data->bd_ram_size = prop;

	if (of_property_read_u32(node, "rx_descs", &prop)) {
		dev_err(&pdev->dev, "Missing rx_descs property in the DT.\n");
		return -EINVAL;
	}
	data->rx_descs = prop;

	if (of_property_read_u32(node, "mac_control", &prop)) {
		dev_err(&pdev->dev, "Missing mac_control property in the DT.\n");
		return -EINVAL;
	}
	data->mac_control = prop;

	if (of_property_read_bool(node, "dual_emac"))
		data->dual_emac = 1;

	/*
	 * Populate all the child nodes here...
	 */
	ret = of_platform_populate(node, NULL, NULL, &pdev->dev);
	/* We do not want to force this, as in some cases may not have child */
	if (ret)
		dev_warn(&pdev->dev, "Doesn't have any child node\n");

	for_each_child_of_node(node, slave_node) {
		struct cpsw_slave_data *slave_data = data->slave_data + i;
		const void *mac_addr = NULL;
		u32 phyid;
		int lenp;
		const __be32 *parp;
		struct device_node *mdio_node;
		struct platform_device *mdio;

		/* This is no slave child node, continue */
		if (strcmp(slave_node->name, "slave"))
			continue;

		parp = of_get_property(slave_node, "phy_id", &lenp);
		if ((parp == NULL) || (lenp != (sizeof(void *) * 2))) {
			dev_err(&pdev->dev, "Missing slave[%d] phy_id property\n", i);
			goto no_phy_slave;
		}
		mdio_node = of_find_node_by_phandle(be32_to_cpup(parp));
		phyid = be32_to_cpup(parp+1);
		mdio = of_find_device_by_node(mdio_node);
	        of_node_put(mdio_node);
		if (!mdio) {
			dev_err(&pdev->dev, "Missing mdio platform device\n");
			return -EINVAL;
		}
		snprintf(slave_data->phy_id, sizeof(slave_data->phy_id),
			 PHY_ID_FMT, mdio->name, phyid);

		 slave_data->phy_if = of_get_phy_mode(slave_node);
		 if (slave_data->phy_if < 0) {
			 dev_err(&pdev->dev, "Missing or malformed slave[%d] phy-mode property\n",
				 i);
			 return slave_data->phy_if;
		 }

no_phy_slave:
		mac_addr = of_get_mac_address(slave_node);
		if (mac_addr) {
			memcpy(slave_data->mac_addr, mac_addr, ETH_ALEN);
		} else {
			if (of_machine_is_compatible("ti,am33xx")) {
				ret = cpsw_am33xx_cm_get_macid(&pdev->dev,
							0x630, i,
							slave_data->mac_addr);
				if (ret)
					return ret;
			}
		}
		if (data->dual_emac) {
			if (of_property_read_u32(slave_node, "dual_emac_res_vlan",
						 &prop)) {
				dev_err(&pdev->dev, "Missing dual_emac_res_vlan in DT.\n");
				slave_data->dual_emac_res_vlan = i+1;
				dev_err(&pdev->dev, "Using %d as Reserved VLAN for %d slave\n",
					slave_data->dual_emac_res_vlan, i);
			} else {
				slave_data->dual_emac_res_vlan = prop;
			}
		}

		i++;
		if (i == data->slaves)
			break;
		}

 	return 0;
 }

static rtdm_irq_handler_t cpsw_get_irq_handler(struct cpsw_priv *priv, int irq_idx)
{
	static const rtdm_irq_handler_t non_napi_irq_tab[4] = {
		cpsw_rx_thresh_pend_irq, cpsw_rx_pend_irq,
		cpsw_tx_pend_irq, cpsw_misc_pend_irq
	};

	if ((unsigned int)irq_idx >= 4)
		return NULL;

#if 0
	if (!priv->data.disable_napi)
		return cpsw_interrupt;
#endif

	return non_napi_irq_tab[irq_idx];
}

static int cpsw_probe(struct platform_device *pdev)
{
	struct cpsw_platform_data	*data = pdev->dev.platform_data;
	struct rtnet_device		*ndev;
	struct cpsw_priv		*priv;
	struct cpdma_params		dma_params;
	struct cpsw_ale_params		ale_params;
	void __iomem			*ss_regs, *wr_regs;
	struct resource			*res;
	u32 slave_offset, sliver_offset, slave_size;
	rtdm_irq_handler_t		irqh;
	int ret = 0, i, j, k = 0;

	ndev = rt_alloc_etherdev(sizeof(struct cpsw_priv), RX_RING_SIZE*2);
	if (ndev == NULL){
		return -ENOMEM;
	}
	rtdev_alloc_name(ndev, "rteth%d");
	rt_rtdev_connect(ndev, &RTDEV_manager);
	//RTNET_SET_MODULE_OWNER(ndev);
	ndev->vers = RTDEV_VERS_2_0;

	priv = ndev->priv;
	platform_set_drvdata(pdev, ndev);
	rtdm_lock_init(&priv->lock);
	priv->pdev = pdev;
	priv->ndev = ndev;
	priv->dev  = &pdev->dev;
	priv->msg_enable = netif_msg_init(debug_level, CPSW_DEBUG);
	priv->rx_packet_max = max(rx_packet_max, 128);
	priv->irq_enabled = false;

	if (rtskb_module_pool_init(&priv->skb_pool, RX_RING_SIZE*2) < RX_RING_SIZE*2) {
		rtskb_pool_release(&priv->skb_pool);
		ret = -ENOMEM;
		goto clean_real_ndev_ret;
	}

	/*
	 * This may be required here for child devices.
	 */
	pm_runtime_enable(&pdev->dev);

	if (cpsw_probe_dt(&priv->data, pdev)) {
		pr_err("cpsw: platform data missing\n");
		ret = -ENODEV;
		goto clean_ndev_ret;
	}
	data = &priv->data;

	pr_info("DT probed OK\n");

	if (is_valid_ether_addr(data->slave_data[0].mac_addr)) {
		memcpy(priv->mac_addr, data->slave_data[0].mac_addr, ETH_ALEN);
		pr_info("Detected MACID = %pM", priv->mac_addr);
	} else {
		eth_random_addr(priv->mac_addr);
		pr_info("Random MACID = %pM", priv->mac_addr);
	}

	memcpy(ndev->dev_addr, priv->mac_addr, ETH_ALEN);

	priv->slaves = kzalloc(sizeof(struct cpsw_slave) * data->slaves,
			       GFP_KERNEL);
	if (!priv->slaves) {
		ret = -EBUSY;
		goto clean_ndev_ret;
	}
	for (i = 0; i < data->slaves; i++)
		priv->slaves[i].slave_num = i;

	priv->clk = clk_get(&pdev->dev, "fck");
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "fck is not found\n");
		ret = -ENODEV;
		goto clean_slave_ret;
	}

	priv->cpsw_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!priv->cpsw_res) {
		dev_err(priv->dev, "error getting i/o resource\n");
		ret = -ENOENT;
		goto clean_clk_ret;
	}
	if (!request_mem_region(priv->cpsw_res->start,
				resource_size(priv->cpsw_res), ndev->name)) {
		dev_err(priv->dev, "failed request i/o region\n");
		ret = -ENXIO;
		goto clean_clk_ret;
	}
	ss_regs = ioremap(priv->cpsw_res->start, resource_size(priv->cpsw_res));
	if (!ss_regs) {
		dev_err(priv->dev, "unable to map i/o region\n");
		goto clean_cpsw_iores_ret;
	}
	priv->regs = ss_regs;
	priv->version = __raw_readl(&priv->regs->id_ver);
	priv->host_port = HOST_PORT_NUM;

	priv->cpsw_wr_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!priv->cpsw_wr_res) {
		dev_err(priv->dev, "error getting i/o resource\n");
		ret = -ENOENT;
		goto clean_iomap_ret;
	}
	if (!request_mem_region(priv->cpsw_wr_res->start,
			resource_size(priv->cpsw_wr_res), ndev->name)) {
		dev_err(priv->dev, "failed request i/o region\n");
		ret = -ENXIO;
		goto clean_iomap_ret;
	}
	wr_regs = ioremap(priv->cpsw_wr_res->start,
				resource_size(priv->cpsw_wr_res));
	if (!wr_regs) {
		dev_err(priv->dev, "unable to map i/o region\n");
		goto clean_cpsw_wr_iores_ret;
	}
	priv->wr_regs = wr_regs;

	memset(&dma_params, 0, sizeof(dma_params));
	memset(&ale_params, 0, sizeof(ale_params));

	switch (priv->version) {
	case CPSW_VERSION_1:
		priv->host_port_regs = ss_regs + CPSW1_HOST_PORT_OFFSET;
		priv->cpts.reg       = ss_regs + CPSW1_CPTS_OFFSET;
		dma_params.dmaregs   = ss_regs + CPSW1_CPDMA_OFFSET;
		dma_params.txhdp     = ss_regs + CPSW1_STATERAM_OFFSET;
		ale_params.ale_regs  = ss_regs + CPSW1_ALE_OFFSET;
		slave_offset         = CPSW1_SLAVE_OFFSET;
		slave_size           = CPSW1_SLAVE_SIZE;
		sliver_offset        = CPSW1_SLIVER_OFFSET;
		dma_params.desc_mem_phys = 0;
		break;
	case CPSW_VERSION_2:
		priv->host_port_regs = ss_regs + CPSW2_HOST_PORT_OFFSET;
		priv->cpts.reg       = ss_regs + CPSW2_CPTS_OFFSET;
		dma_params.dmaregs   = ss_regs + CPSW2_CPDMA_OFFSET;
		dma_params.txhdp     = ss_regs + CPSW2_STATERAM_OFFSET;
		ale_params.ale_regs  = ss_regs + CPSW2_ALE_OFFSET;
		slave_offset         = CPSW2_SLAVE_OFFSET;
		slave_size           = CPSW2_SLAVE_SIZE;
		sliver_offset        = CPSW2_SLIVER_OFFSET;
		dma_params.desc_mem_phys =
			(u32 __force) priv->cpsw_res->start + CPSW2_BD_OFFSET;
		break;
	default:
		dev_err(priv->dev, "unknown version 0x%08x\n", priv->version);
		ret = -ENODEV;
		goto clean_cpsw_wr_iores_ret;
	}
	for (i = 0; i < priv->data.slaves; i++) {
		struct cpsw_slave *slave = &priv->slaves[i];
		cpsw_slave_init(slave, priv, slave_offset, sliver_offset);
		slave_offset  += slave_size;
		sliver_offset += SLIVER_SIZE;
	}

	dma_params.dev		= &pdev->dev;
	dma_params.rxthresh	= dma_params.dmaregs + CPDMA_RXTHRESH;
	dma_params.rxfree	= dma_params.dmaregs + CPDMA_RXFREE;
	dma_params.rxhdp	= dma_params.txhdp + CPDMA_RXHDP;
	dma_params.txcp		= dma_params.txhdp + CPDMA_TXCP;
	dma_params.rxcp		= dma_params.txhdp + CPDMA_RXCP;

	dma_params.num_chan		= data->channels;
	dma_params.has_soft_reset	= true;
	dma_params.min_packet_size	= CPSW_MIN_PACKET_SIZE;
	dma_params.desc_mem_size	= data->bd_ram_size;
	dma_params.desc_align		= 16;
	dma_params.has_ext_regs		= true;
	dma_params.desc_hw_addr         = dma_params.desc_mem_phys;

	priv->dma = cpdma_ctlr_create(&dma_params);
	if (!priv->dma) {
		dev_err(priv->dev, "error initializing dma\n");
		ret = -ENOMEM;
		goto clean_wr_iomap_ret;
	}

	priv->txch = cpdma_chan_create(priv->dma, tx_chan_num(0),
				       cpsw_tx_handler);
	priv->rxch = cpdma_chan_create(priv->dma, rx_chan_num(0),
				       cpsw_rx_handler);

	if (WARN_ON(!priv->txch || !priv->rxch)) {
		dev_err(priv->dev, "error initializing dma channels\n");
		ret = -ENOMEM;
		goto clean_dma_ret;
	}

	ale_params.dev			= &pdev->dev;
	ale_params.ale_ageout		= ale_ageout;
	ale_params.ale_entries		= data->ale_entries;
	ale_params.ale_ports		= data->slaves;

	priv->ale = cpsw_ale_create(&ale_params);
	if (!priv->ale) {
		dev_err(priv->dev, "error initializing ale engine\n");
		ret = -ENODEV;
		goto clean_dma_ret;
	}

	ndev->irq = platform_get_irq(pdev, 0);
	if (ndev->irq < 0) {
		dev_err(priv->dev, "error getting irq resource\n");
		ret = -ENOENT;
		goto clean_ale_ret;
	}

#if 0
	dev_info(&pdev->dev, "NAPI %s\n", priv->data.disable_napi ?
			"disabled" : "enabled");
#endif

	rt_stack_connect(ndev, &STACK_manager);
	/* get interrupts */
	j = k = 0;
	while ((res = platform_get_resource(pdev, IORESOURCE_IRQ, j++))) {
		for (i = res->start; k < 4 && i <= res->end; i++) {
			irqh = cpsw_get_irq_handler(priv, k);
			if (irqh == NULL) {
				dev_err(&pdev->dev, "Unable to get handler "
						"for #%d (%d)\n", k, i);
				goto clean_ale_ret;
			}
			if (rtdm_irq_request(&priv->irqs_table[k], i, irqh, RTDM_IRQTYPE_SHARED,
						dev_name(&pdev->dev), priv)) {
				dev_err(priv->dev, "error attaching irq\n");
				goto clean_ale_ret;
			}
			k++;
		}
	}
	priv->num_irqs = k;

	ndev->flags |= IFF_ALLMULTI;	/* see cpsw_ndo_change_rx_flags() */

	#if 0
	ndev->netdev_ops = &cpsw_netdev_ops;
	SET_ETHTOOL_OPS(ndev, &cpsw_ethtool_ops);
	#endif
	ndev->open		= cpsw_ndo_open;
	ndev->hard_start_xmit 	= cpsw_ndo_start_xmit;
	ndev->get_stats    	= cpsw_ndo_get_stats;
	ndev->stop 		= cpsw_ndo_stop;

	/* register the network device */
	ret = rt_register_rtnetdev(ndev);
	if (ret) {
		dev_err(priv->dev, "error registering net device\n");
		ret = -ENODEV;
		goto clean_irq_ret;
	}

	if (cpts_register(&pdev->dev, &priv->cpts,
			  data->cpts_clock_mult, data->cpts_clock_shift))
		dev_err(priv->dev, "error registering cpts device\n");

	cpsw_notice(priv, probe, "initialized device (regs %x, irq %d)\n",
		  priv->cpsw_res->start, ndev->irq);

	return 0;

clean_irq_ret:
	for(i = 0; i < priv->num_irqs; i++)
		rtdm_irq_free(&priv->irqs_table[i]);
	rt_stack_disconnect(ndev);
clean_ale_ret:
	cpsw_ale_destroy(priv->ale);
clean_dma_ret:
	cpdma_chan_destroy(priv->txch);
	cpdma_chan_destroy(priv->rxch);
	cpdma_ctlr_destroy(priv->dma);
clean_wr_iomap_ret:
	iounmap(priv->wr_regs);
clean_cpsw_wr_iores_ret:
	release_mem_region(priv->cpsw_wr_res->start,
			   resource_size(priv->cpsw_wr_res));
clean_iomap_ret:
	iounmap(priv->regs);
clean_cpsw_iores_ret:
	release_mem_region(priv->cpsw_res->start,
			   resource_size(priv->cpsw_res));
clean_clk_ret:
	clk_put(priv->clk);
clean_slave_ret:
	pm_runtime_disable(&pdev->dev);
	kfree(priv->slaves);
clean_ndev_ret:
	rtskb_pool_release(&priv->skb_pool);
clean_real_ndev_ret:
	rt_rtdev_disconnect(ndev);
	rtdev_free(ndev);
	return ret;
}

static int cpsw_remove(struct platform_device *pdev)
{
	struct rtnet_device *ndev = platform_get_drvdata(pdev);
	struct cpsw_priv *priv = ndev->priv;
	int i;

	pr_info("removing device");
	platform_set_drvdata(pdev, NULL);

	cpts_unregister(&priv->cpts);
	for(i = 0; i < priv->num_irqs; i++)
		rtdm_irq_free(&priv->irqs_table[i]);
	rt_stack_disconnect(ndev);
	cpsw_ale_destroy(priv->ale);
	cpdma_chan_destroy(priv->txch);
	cpdma_chan_destroy(priv->rxch);
	cpdma_ctlr_destroy(priv->dma);
	iounmap(priv->regs);
	release_mem_region(priv->cpsw_res->start,
			   resource_size(priv->cpsw_res));
	iounmap(priv->wr_regs);
	release_mem_region(priv->cpsw_wr_res->start,
			   resource_size(priv->cpsw_wr_res));
	pm_runtime_disable(&pdev->dev);
	clk_put(priv->clk);
	kfree(priv->slaves);
	rtskb_pool_release(&priv->skb_pool);
	rt_rtdev_disconnect(ndev);
	rtdev_free(ndev);

	return 0;
}

static int cpsw_suspend(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct rtnet_device	*ndev = platform_get_drvdata(pdev);

	if (rtnetif_running(ndev))
		cpsw_ndo_stop(ndev);
	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int cpsw_resume(struct device *dev)
{
	struct platform_device	*pdev = to_platform_device(dev);
	struct rtnet_device	*ndev = platform_get_drvdata(pdev);

	pm_runtime_get_sync(&pdev->dev);
	if (rtnetif_running(ndev))
		cpsw_ndo_open(ndev);
	return 0;
}

static const struct dev_pm_ops cpsw_pm_ops = {
	.suspend	= cpsw_suspend,
	.resume		= cpsw_resume,
};

static const struct of_device_id cpsw_of_mtable[] = {
	{ .compatible = "ti,cpsw", },
	{ /* sentinel */ },
};

static struct platform_driver cpsw_driver = {
	.driver = {
		.name	 = "cpsw",
		.owner	 = THIS_MODULE,
		.pm	 = &cpsw_pm_ops,
		.of_match_table = of_match_ptr(cpsw_of_mtable),
	},
	.probe = cpsw_probe,
	.remove = cpsw_remove,
};

static int __init cpsw_init(void)
{
	return platform_driver_register(&cpsw_driver);
}
late_initcall(cpsw_init);

static void __exit cpsw_exit(void)
{
	platform_driver_unregister(&cpsw_driver);
}
module_exit(cpsw_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cyril Chemparathy <cyril@ti.com>");
MODULE_AUTHOR("Mugunthan V N <mugunthanvnm@ti.com>");
MODULE_DESCRIPTION("RT TI CPSW Ethernet driver");
