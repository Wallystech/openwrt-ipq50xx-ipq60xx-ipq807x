// SPDX-License-Identifier: GPL-2.0+
/*
 * net/dsa/tag_trailer.c - Trailer tag format handling
 * Copyright (c) 2008-2009 Marvell Semiconductor
 */

#include <linux/etherdevice.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/rtl838x.h>

#include "dsa_priv.h"

static struct sk_buff *trailer_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dsa_port *dp = dsa_slave_to_port(dev);
	struct sk_buff *nskb;
	int padlen;
	u8 *trailer;

	/*
	 * We have to make sure that the trailer ends up as the very
	 * last 4 bytes of the packet.  This means that we have to pad
	 * the packet to the minimum ethernet frame size, if necessary,
	 * before adding the trailer.
	 */
	padlen = 0;
	if (skb->len < 60)
		padlen = 60 - skb->len;

	nskb = alloc_skb(NET_IP_ALIGN + skb->len + padlen + 4, GFP_ATOMIC);
	if (!nskb)
		return NULL;
	skb_reserve(nskb, NET_IP_ALIGN);

	skb_reset_mac_header(nskb);
	skb_set_network_header(nskb, skb_network_header(skb) - skb->head);
	skb_set_transport_header(nskb, skb_transport_header(skb) - skb->head);
	skb_copy_and_csum_dev(skb, skb_put(nskb, skb->len));
	consume_skb(skb);

	if (padlen) {
		skb_put_zero(nskb, padlen);
	}

	trailer = skb_put(nskb, 4);
	trailer[0] = 0x80;

	trailer[1] = dp->index;
	trailer[2] = 0x10;
	trailer[3] = 0x00;

	return nskb;
}

static struct sk_buff *trailer_rcv(struct sk_buff *skb, struct net_device *dev,
				   struct packet_type *pt)
{
	struct dsa_port *cpu_dp = dev->dsa_ptr;
	struct dsa_switch *ds = cpu_dp->ds;
	int i;
	struct rtl838x_switch_priv *priv = ds->priv;
	u8 *trailer;
	bool trunk = false;
	int source_port;

	if (skb_linearize(skb))
		return NULL;

	trailer = skb_tail_pointer(skb) - 4;

//	pr_info("lag member %X:%X:%X:%X found\n", trailer[0], trailer[1], trailer[2],trailer[3]);
	if (trailer[0] != 0x80 || (trailer[1] & 0x80) != 0x00 ||
	    (trailer[2] & 0xef) != 0x00 || trailer[3] != 0x00)
		return NULL;

	if (trailer[1] & 0x40) { // forward
		skb->offload_fwd_mark = 1;
		struct dsa_switch *ds = cpu_dp->ds;
		struct rtl838x_switch_priv *priv = ds->priv;
		skb->offload_fwd_mark = 1;
		if (priv->lagmembers & (1ULL << source_port)) {
			pr_info("lag member %d found\n", source_port);
			trunk = true;
		}
	}
	source_port = trailer[1] & 0x3f;
	
	if (trunk) {

		/* The exact source port is not available in the tag,
		 * so we inject the frame directly on the upper
		 * team/bond.
		 */
		skb->dev = dsa_lag_dev(cpu_dp->dst, source_port);
	} else {
		skb->dev = dsa_master_find_slave(dev, 0, source_port);
	}
	if (!skb->dev)
		return NULL;

	if (pskb_trim_rcsum(skb, skb->len - 4))
		return NULL;

	return skb;
}

static const struct dsa_device_ops trailer_netdev_ops = {
	.name	= "rtl83xx",
	.proto	= DSA_TAG_PROTO_RTL83XX,
	.xmit	= trailer_xmit,
	.rcv	= trailer_rcv,
	.overhead = 4,
};

MODULE_LICENSE("GPL");
MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_RTL83XX);

module_dsa_tag_driver(trailer_netdev_ops);
