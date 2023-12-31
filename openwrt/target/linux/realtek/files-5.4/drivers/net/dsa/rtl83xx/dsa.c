
#include <net/dsa.h>
#include <linux/if_bridge.h>

#include <asm/mach-rtl838x/mach-rtl83xx.h>
#include "rtl83xx.h"

extern int rtl930x_read_sds_phy(int phy_addr, int page, int phy_reg);
extern int rtl930x_write_sds_phy(int phy_addr, int page, int phy_reg, u16 v);
extern struct rtl83xx_soc_info soc_info;


static void rtl83xx_init_stats(struct rtl838x_switch_priv *priv)
{
	mutex_lock(&priv->reg_mutex);

	/* Enable statistics module: all counters plus debug.
	 * On RTL839x all counters are enabled by default
	 */
	if (priv->family_id == RTL8380_FAMILY_ID)
		sw_w32_mask(0, 3, RTL838X_STAT_CTRL);

	/* Reset statistics counters */
	sw_w32_mask(0, 1, priv->r->stat_rst);

	mutex_unlock(&priv->reg_mutex);
}

static void rtl83xx_enable_phy_polling(struct rtl838x_switch_priv *priv)
{
	int i;
	u64 v = 0;

	msleep(1000);
	/* Enable all ports with a PHY, including the SFP-ports */
	for (i = 0; i < priv->cpu_port; i++) {
		if (priv->ports[i].phy)
			v |= BIT_ULL(i);
	}

	pr_debug("%s: %16llx\n", __func__, v);
	priv->r->set_port_reg_le(v, priv->r->smi_poll_ctrl);

	/* PHY update complete, there is no global PHY polling enable bit on the 9300 */
	if (priv->family_id == RTL8390_FAMILY_ID)
		sw_w32_mask(0, BIT(7), RTL839X_SMI_GLB_CTRL);
	else if(priv->family_id == RTL9300_FAMILY_ID)
		sw_w32_mask(0, 0x8000, RTL838X_SMI_GLB_CTRL);
}

const struct rtl83xx_mib_desc rtl83xx_mib[] = {
	MIB_DESC(2, 0xf8, "ifInOctets"),
	MIB_DESC(2, 0xf0, "ifOutOctets"),
	MIB_DESC(1, 0xec, "dot1dTpPortInDiscards"),
	MIB_DESC(1, 0xe8, "ifInUcastPkts"),
	MIB_DESC(1, 0xe4, "ifInMulticastPkts"),
	MIB_DESC(1, 0xe0, "ifInBroadcastPkts"),
	MIB_DESC(1, 0xdc, "ifOutUcastPkts"),
	MIB_DESC(1, 0xd8, "ifOutMulticastPkts"),
	MIB_DESC(1, 0xd4, "ifOutBroadcastPkts"),
	MIB_DESC(1, 0xd0, "ifOutDiscards"),
	MIB_DESC(1, 0xcc, ".3SingleCollisionFrames"),
	MIB_DESC(1, 0xc8, ".3MultipleCollisionFrames"),
	MIB_DESC(1, 0xc4, ".3DeferredTransmissions"),
	MIB_DESC(1, 0xc0, ".3LateCollisions"),
	MIB_DESC(1, 0xbc, ".3ExcessiveCollisions"),
	MIB_DESC(1, 0xb8, ".3SymbolErrors"),
	MIB_DESC(1, 0xb4, ".3ControlInUnknownOpcodes"),
	MIB_DESC(1, 0xb0, ".3InPauseFrames"),
	MIB_DESC(1, 0xac, ".3OutPauseFrames"),
	MIB_DESC(1, 0xa8, "DropEvents"),
	MIB_DESC(1, 0xa4, "tx_BroadcastPkts"),
	MIB_DESC(1, 0xa0, "tx_MulticastPkts"),
	MIB_DESC(1, 0x9c, "CRCAlignErrors"),
	MIB_DESC(1, 0x98, "tx_UndersizePkts"),
	MIB_DESC(1, 0x94, "rx_UndersizePkts"),
	MIB_DESC(1, 0x90, "rx_UndersizedropPkts"),
	MIB_DESC(1, 0x8c, "tx_OversizePkts"),
	MIB_DESC(1, 0x88, "rx_OversizePkts"),
	MIB_DESC(1, 0x84, "Fragments"),
	MIB_DESC(1, 0x80, "Jabbers"),
	MIB_DESC(1, 0x7c, "Collisions"),
	MIB_DESC(1, 0x78, "tx_Pkts64Octets"),
	MIB_DESC(1, 0x74, "rx_Pkts64Octets"),
	MIB_DESC(1, 0x70, "tx_Pkts65to127Octets"),
	MIB_DESC(1, 0x6c, "rx_Pkts65to127Octets"),
	MIB_DESC(1, 0x68, "tx_Pkts128to255Octets"),
	MIB_DESC(1, 0x64, "rx_Pkts128to255Octets"),
	MIB_DESC(1, 0x60, "tx_Pkts256to511Octets"),
	MIB_DESC(1, 0x5c, "rx_Pkts256to511Octets"),
	MIB_DESC(1, 0x58, "tx_Pkts512to1023Octets"),
	MIB_DESC(1, 0x54, "rx_Pkts512to1023Octets"),
	MIB_DESC(1, 0x50, "tx_Pkts1024to1518Octets"),
	MIB_DESC(1, 0x4c, "rx_StatsPkts1024to1518Octets"),
	MIB_DESC(1, 0x48, "tx_Pkts1519toMaxOctets"),
	MIB_DESC(1, 0x44, "rx_Pkts1519toMaxOctets"),
	MIB_DESC(1, 0x40, "rxMacDiscards")
};


/* DSA callbacks */


static enum dsa_tag_protocol rtl83xx_get_tag_protocol(struct dsa_switch *ds, int port)
{
	/* The switch does not tag the frames, instead internally the header
	 * structure for each packet is tagged accordingly.
	 */
	return DSA_TAG_PROTO_RTL83XX;
}

/*
 * Initialize all VLANS
 */
static void rtl83xx_vlan_setup(struct rtl838x_switch_priv *priv)
{
	struct rtl838x_vlan_info info;
	int i;

	pr_debug("In %s\n", __func__);

	priv->r->vlan_profile_setup(0);
	priv->r->vlan_profile_setup(1);
	pr_info("UNKNOWN_MC_PMASK: %016llx\n", priv->r->read_mcast_pmask(UNKNOWN_MC_PMASK));
	priv->r->vlan_profile_dump(0);

	info.fid = 0;			// Default Forwarding ID / MSTI
	info.hash_uc_fid = false;	// Do not build the L2 lookup hash with FID, but VID
	info.hash_mc_fid = false;	// Do the same for Multicast packets
	info.profile_id = 0;		// Use default Vlan Profile 0
	info.tagged_ports = 0;		// Initially no port members

	// Initialize all vlans 0-4095
	for (i = 0; i < MAX_VLANS; i ++)
		priv->r->vlan_set_tagged(i, &info);

	// reset PVIDs; defaults to 1 on reset
	for (i = 0; i <= priv->ds->num_ports; i++)
		sw_w32(0, priv->r->vlan_port_pb + (i << 2));

	// Set forwarding action based on inner VLAN tag
	for (i = 0; i < priv->cpu_port; i++)
		priv->r->vlan_fwd_on_inner(i, true);
}

static void rtl83xx_setup_bpdu_traps(struct rtl838x_switch_priv *priv)
{
	int i;
	for (i = 0; i < priv->cpu_port; i++)
		priv->r->set_receive_management_action(i, BPDU, TRAP2CPU);
}

static int rtl83xx_setup(struct dsa_switch *ds)
{
	int i;
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 port_bitmap = BIT_ULL(priv->cpu_port);

	pr_debug("%s called\n", __func__);

	/* Disable MAC polling the PHY so that we can start configuration */
	priv->r->set_port_reg_le(0ULL, priv->r->smi_poll_ctrl);

	for (i = 0; i < ds->num_ports; i++)
		priv->ports[i].enable = false;
	priv->ports[priv->cpu_port].enable = true;

	/* Isolate ports from each other: traffic only CPU <-> port */
	/* Setting bit j in register RTL838X_PORT_ISO_CTRL(i) allows
	 * traffic from source port i to destination port j
	 */
	for (i = 0; i < priv->cpu_port; i++) {
		if (priv->ports[i].phy) {
			priv->r->set_port_reg_be(BIT_ULL(priv->cpu_port) | BIT_ULL(i),
					      priv->r->port_iso_ctrl(i));
			port_bitmap |= BIT_ULL(i);
		}
	}
	priv->r->set_port_reg_be(port_bitmap, priv->r->port_iso_ctrl(priv->cpu_port));

	if (priv->family_id == RTL8380_FAMILY_ID)
		rtl838x_print_matrix();
	else
		rtl839x_print_matrix();

	rtl83xx_init_stats(priv);

	rtl83xx_vlan_setup(priv);

	rtl83xx_setup_bpdu_traps(priv);

	ds->configure_vlan_while_not_filtering = true;

	priv->r->l2_learning_setup();

	/* Enable MAC Polling PHY again */
	rtl83xx_enable_phy_polling(priv);
	pr_debug("Please wait until PHY is settled\n");
	msleep(1000);

	priv->r->pie_init(priv);

	return 0;
}

static int rtl930x_setup(struct dsa_switch *ds)
{
	int i;
	struct rtl838x_switch_priv *priv = ds->priv;
	u32 port_bitmap = BIT(priv->cpu_port);

	pr_debug("%s called\n", __func__);

	// Enable CSTI STP mode
//	sw_w32(1, RTL930X_ST_CTRL);

	/* Disable MAC polling the PHY so that we can start configuration */
	sw_w32(0, RTL930X_SMI_POLL_CTRL);

	// Disable all ports except CPU port
	for (i = 0; i < ds->num_ports; i++)
		priv->ports[i].enable = false;
	priv->ports[priv->cpu_port].enable = true;

	for (i = 0; i < priv->cpu_port; i++) {
		if (priv->ports[i].phy) {
			priv->r->traffic_set(i, BIT_ULL(priv->cpu_port) | BIT_ULL(i));
			port_bitmap |= BIT_ULL(i);
		}
	}
	priv->r->traffic_set(priv->cpu_port, port_bitmap);

	rtl930x_print_matrix();

	// TODO: Initialize statistics

	rtl83xx_vlan_setup(priv);

	ds->configure_vlan_while_not_filtering = true;

	rtl83xx_enable_phy_polling(priv);

	priv->r->pie_init(priv);

	return 0;
}

static void rtl83xx_phylink_validate(struct dsa_switch *ds, int port,
				     unsigned long *supported,
				     struct phylink_link_state *state)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	pr_debug("In %s port %d, state is %d", __func__, port, state->interface);

	if (!phy_interface_mode_is_rgmii(state->interface) &&
	    state->interface != PHY_INTERFACE_MODE_NA &&
	    state->interface != PHY_INTERFACE_MODE_1000BASEX &&
	    state->interface != PHY_INTERFACE_MODE_MII &&
	    state->interface != PHY_INTERFACE_MODE_REVMII &&
	    state->interface != PHY_INTERFACE_MODE_GMII &&
	    state->interface != PHY_INTERFACE_MODE_QSGMII &&
	    state->interface != PHY_INTERFACE_MODE_INTERNAL &&
	    state->interface != PHY_INTERFACE_MODE_SGMII) {
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		dev_err(ds->dev,
			"Unsupported interface: %d for port %d\n",
			state->interface, port);
		return;
	}

	/* Allow all the expected bits */
	phylink_set(mask, Autoneg);
	phylink_set_port_modes(mask);
	phylink_set(mask, Pause);
	phylink_set(mask, Asym_Pause);

	/* With the exclusion of MII and Reverse MII, we support Gigabit,
	 * including Half duplex
	 */
	if (state->interface != PHY_INTERFACE_MODE_MII &&
	    state->interface != PHY_INTERFACE_MODE_REVMII) {
		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseT_Half);
	}

	/* On both the 8380 and 8382, ports 24-27 are SFP ports */
	if (port >= 24 && port <= 27 && priv->family_id == RTL8380_FAMILY_ID)
		phylink_set(mask, 1000baseX_Full);

	/* On the RTL839x family of SoCs, ports 48 to 51 are SFP ports */
	if (port >= 48 && port <= 51 && priv->family_id == RTL8390_FAMILY_ID)
		phylink_set(mask, 1000baseX_Full);

	phylink_set(mask, 10baseT_Half);
	phylink_set(mask, 10baseT_Full);
	phylink_set(mask, 100baseT_Half);
	phylink_set(mask, 100baseT_Full);

	bitmap_and(supported, supported, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
}

static void rtl93xx_phylink_validate(struct dsa_switch *ds, int port,
				     unsigned long *supported,
				     struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	pr_info("In %s port %d, state is %d (%s)", __func__, port, state->interface,
		phy_modes(state->interface));

	if (!phy_interface_mode_is_rgmii(state->interface) &&
	    state->interface != PHY_INTERFACE_MODE_NA &&
	    state->interface != PHY_INTERFACE_MODE_1000BASEX &&
	    state->interface != PHY_INTERFACE_MODE_MII &&
	    state->interface != PHY_INTERFACE_MODE_REVMII &&
	    state->interface != PHY_INTERFACE_MODE_GMII &&
	    state->interface != PHY_INTERFACE_MODE_QSGMII &&
	    state->interface != PHY_INTERFACE_MODE_XGMII &&
	    state->interface != PHY_INTERFACE_MODE_HSGMII &&
	    state->interface != PHY_INTERFACE_MODE_10GKR &&
	    state->interface != PHY_INTERFACE_MODE_INTERNAL &&
	    state->interface != PHY_INTERFACE_MODE_SGMII) {
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		dev_err(ds->dev,
			"Unsupported interface: %d for port %d\n",
			state->interface, port);
		return;
	}

	/* Allow all the expected bits */
	phylink_set(mask, Autoneg);
	phylink_set_port_modes(mask);
	phylink_set(mask, Pause);
	phylink_set(mask, Asym_Pause);

	/* With the exclusion of MII and Reverse MII, we support Gigabit,
	 * including Half duplex
	 */
	if (state->interface != PHY_INTERFACE_MODE_MII &&
	    state->interface != PHY_INTERFACE_MODE_REVMII) {
		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseT_Half);
	}

	/* On the RTL9300 family of SoCs, ports 26 to 27 may be SFP ports TODO: take out of .dts */
	if (port >= 26 && port <= 27)
		phylink_set(mask, 1000baseX_Full);

	phylink_set(mask, 10baseT_Half);
	phylink_set(mask, 10baseT_Full);
	phylink_set(mask, 100baseT_Half);
	phylink_set(mask, 100baseT_Full);

	bitmap_and(supported, supported, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
}

static int rtl83xx_phylink_mac_link_state(struct dsa_switch *ds, int port,
					  struct phylink_link_state *state)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 speed;
	u64 link;

	if (port < 0 || port > priv->cpu_port)
		return -EINVAL;

	state->link = 0;
	link = priv->r->get_port_reg_le(priv->r->mac_link_sts);
	if (link & BIT_ULL(port))
		state->link = 1;
	pr_debug("%s: link state port %d: %llx\n", __func__, port, link & BIT_ULL(port));

	state->duplex = 0;
	if (priv->r->get_port_reg_le(priv->r->mac_link_dup_sts) & BIT_ULL(port))
		state->duplex = 1;

	speed = priv->r->get_port_reg_le(priv->r->mac_link_spd_sts(port));
	speed >>= (port % 16) << 1;
	switch (speed & 0x3) {
	case 0:
		state->speed = SPEED_10;
		break;
	case 1:
		state->speed = SPEED_100;
		break;
	case 2:
		state->speed = SPEED_1000;
		break;
	case 3:
		if (priv->family_id == RTL9300_FAMILY_ID
			&& (port == 24 || port == 26)) /* Internal serdes */
			state->speed = SPEED_2500;
		else
			state->speed = SPEED_100; /* Is in fact 500Mbit */
	}

	state->pause &= (MLO_PAUSE_RX | MLO_PAUSE_TX);
	if (priv->r->get_port_reg_le(priv->r->mac_rx_pause_sts) & BIT_ULL(port))
		state->pause |= MLO_PAUSE_RX;
	if (priv->r->get_port_reg_le(priv->r->mac_tx_pause_sts) & BIT_ULL(port))
		state->pause |= MLO_PAUSE_TX;
	return 1;
}

static int rtl93xx_phylink_mac_link_state(struct dsa_switch *ds, int port,
					  struct phylink_link_state *state)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 speed;
	u64 link;

	if (port < 0 || port > priv->cpu_port)
		return -EINVAL;

	/*
	 * On the RTL9300 for at least the RTL8226B PHY, the MAC-side link
	 * state needs to be read twice in order to read a correct result.
	 * This would not be necessary for ports connected e.g. to RTL8218D
	 * PHYs.
	 */
	state->link = 0;
	link = priv->r->get_port_reg_le(priv->r->mac_link_sts);
	link = priv->r->get_port_reg_le(priv->r->mac_link_sts);
	if (link & BIT_ULL(port))
		state->link = 1;
	pr_info("%s: link state port %d: %llx, media %08x\n", __func__, port,
		link & BIT_ULL(port), sw_r32(RTL930X_MAC_LINK_MEDIA_STS));

	state->duplex = 0;
	if (priv->r->get_port_reg_le(priv->r->mac_link_dup_sts) & BIT_ULL(port))
		state->duplex = 1;

	speed = priv->r->get_port_reg_le(priv->r->mac_link_spd_sts(port));
	speed >>= (port % 8) << 2;
	switch (speed & 0xf) {
	case 0:
		state->speed = SPEED_10;
		break;
	case 1:
		state->speed = SPEED_100;
		break;
	case 2:
	case 7:
		state->speed = SPEED_1000;
		break;
	case 4:
		state->speed = SPEED_10000;
		break;
	case 5:
	case 8:
		state->speed = SPEED_2500;
		break;
	case 6:
		state->speed = SPEED_5000;
		break;
	default:
		pr_err("%s: unknown speed: %d\n", __func__, (u32)speed & 0xf);
	}

	pr_info("%s: speed is: %d %d\n", __func__, (u32)speed & 0xf, state->speed);
	state->pause &= (MLO_PAUSE_RX | MLO_PAUSE_TX);
	if (priv->r->get_port_reg_le(priv->r->mac_rx_pause_sts) & BIT_ULL(port))
		state->pause |= MLO_PAUSE_RX;
	if (priv->r->get_port_reg_le(priv->r->mac_tx_pause_sts) & BIT_ULL(port))
		state->pause |= MLO_PAUSE_TX;
	return 1;
}

static void rtl83xx_config_interface(int port, phy_interface_t interface)
{
	u32 old, int_shift, sds_shift;

	switch (port) {
	case 24:
		int_shift = 0;
		sds_shift = 5;
		break;
	case 26:
		int_shift = 3;
		sds_shift = 0;
		break;
	default:
		return;
	}

	old = sw_r32(RTL838X_SDS_MODE_SEL);
	switch (interface) {
	case PHY_INTERFACE_MODE_1000BASEX:
		if ((old >> sds_shift & 0x1f) == 4)
			return;
		sw_w32_mask(0x7 << int_shift, 1 << int_shift, RTL838X_INT_MODE_CTRL);
		sw_w32_mask(0x1f << sds_shift, 4 << sds_shift, RTL838X_SDS_MODE_SEL);
		break;
	case PHY_INTERFACE_MODE_SGMII:
		if ((old >> sds_shift & 0x1f) == 2)
			return;
		sw_w32_mask(0x7 << int_shift, 2 << int_shift, RTL838X_INT_MODE_CTRL);
		sw_w32_mask(0x1f << sds_shift, 2 << sds_shift, RTL838X_SDS_MODE_SEL);
		break;
	default:
		return;
	}
	pr_debug("configured port %d for interface %s\n", port, phy_modes(interface));
}

static void rtl83xx_phylink_mac_config(struct dsa_switch *ds, int port,
					unsigned int mode,
					const struct phylink_link_state *state)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u32 reg;
	int speed_bit = priv->family_id == RTL8380_FAMILY_ID ? 4 : 3;
//	int force_fc = priv->family_id == RTL8380_FAMILY_ID ? RTL838X_MAC_FORCE_FC_EN : RTL839X_MAC_FORCE_FC_EN;

	pr_info("%s port %d, mode %x, phy-mode: %s, speed %d, link %d\n", __func__,
		port, mode, phy_modes(state->interface), state->speed, state->link);

	if (port == priv->cpu_port) {
		/* Set Speed, duplex, flow control
		 * FORCE_EN | LINK_EN | NWAY_EN | DUP_SEL
		 * | SPD_SEL = 0b10 | FORCE_FC_EN | PHY_MASTER_SLV_MANUAL_EN
		 * | MEDIA_SEL
		 */
		if (priv->family_id == RTL8380_FAMILY_ID) {
			sw_w32(0x6192F, priv->r->mac_force_mode_ctrl(priv->cpu_port));
			/* allow CRC errors on CPU-port */
			sw_w32_mask(0, 0x8, RTL838X_MAC_PORT_CTRL(priv->cpu_port));
		} else {
			sw_w32_mask(0, 3, priv->r->mac_force_mode_ctrl(priv->cpu_port));
		}
		return;
	}

	reg = sw_r32(priv->r->mac_force_mode_ctrl(port));
	/* Auto-Negotiation does not work for MAC in RTL8390 */
	if (priv->family_id == RTL8380_FAMILY_ID) {
		if (mode == MLO_AN_PHY || phylink_autoneg_inband(mode)) {
			pr_debug("PHY autonegotiates\n");
			reg |= RTL830X_NWAY_EN;
			sw_w32(reg, priv->r->mac_force_mode_ctrl(port));
			rtl83xx_config_interface(port, state->interface);
			return;
		}
	}

	if (mode != MLO_AN_FIXED)
		pr_debug("Fixed state.\n");

	/* Clear id_mode_dis bit, and the existing port mode, let
	 * RGMII_MODE_EN bet set by mac_link_{up,down}
	 */
	switch(priv->family_id)
	{
	case RTL8380_FAMILY_ID:
		reg &= ~(RTL830X_RX_PAUSE_EN | RTL830X_TX_PAUSE_EN);
		if (state->pause & MLO_PAUSE_TXRX_MASK) {
			if (state->pause & MLO_PAUSE_TX)
				reg |= RTL830X_TX_PAUSE_EN;
			reg |= RTL830X_RX_PAUSE_EN;
		}
	break;
	case RTL8390_FAMILY_ID:
		reg &= ~(RTL839X_RX_PAUSE_EN | RTL839X_TX_PAUSE_EN);
		if (state->pause & MLO_PAUSE_TXRX_MASK) {
			if (state->pause & MLO_PAUSE_TX)
				reg |= RTL839X_TX_PAUSE_EN;
			reg |= RTL839X_RX_PAUSE_EN;
		}
	break;
	case RTL9300_FAMILY_ID:
		reg &= ~(RTL930X_RX_PAUSE_EN | RTL930X_TX_PAUSE_EN);
		if (state->pause & MLO_PAUSE_TXRX_MASK) {
			if (state->pause & MLO_PAUSE_TX)
				reg |= RTL930X_TX_PAUSE_EN;
			reg |= RTL930X_RX_PAUSE_EN;
		}
	break;
	case RTL9310_FAMILY_ID:
		reg &= ~(RTL931X_RX_PAUSE_EN | RTL931X_TX_PAUSE_EN);
		if (state->pause & MLO_PAUSE_TXRX_MASK) {
			if (state->pause & MLO_PAUSE_TX)
				reg |= RTL931X_TX_PAUSE_EN;
			reg |= RTL931X_RX_PAUSE_EN;
		}
	break;
	}
	reg &= ~(3 << speed_bit);
//	reg &= ~(1 << force_fc);
	switch (state->speed) {
	case SPEED_1000:
		reg |= 2 << speed_bit;
		break;
	case SPEED_100:
		reg |= 1 << speed_bit;
		break;
	case SPEED_10:
		reg = 0;
		break;
	case SPEED_2500:
		reg = 5 << speed_bit;
		break;
	case SPEED_5000:
		reg = 6 << speed_bit;
		break;
	case SPEED_10000:
		reg = 4 << speed_bit;
	}
	switch(priv->family_id)
	{
	case RTL8380_FAMILY_ID:
		reg &= ~(RTL830X_DUPLEX_MODE | RTL830X_FORCE_LINK_EN);
	break;
	case RTL8390_FAMILY_ID:
		reg &= ~(RTL839X_DUPLEX_MODE | RTL839X_FORCE_LINK_EN);
	break;
	case RTL9300_FAMILY_ID:
		reg &= ~(RTL930X_DUPLEX_MODE | RTL930X_FORCE_LINK_EN);
	break;
	case RTL9310_FAMILY_ID:
		reg &= ~(RTL931X_DUPLEX_MODE | RTL931X_FORCE_LINK_EN);
	break;
	}
	if (priv->lagmembers & (1ULL << port)) {
		switch(priv->family_id)
		{
		case RTL8380_FAMILY_ID:
			reg |= (RTL830X_DUPLEX_MODE | RTL830X_FORCE_LINK_EN);
		break;
		case RTL8390_FAMILY_ID:
			reg |= (RTL839X_DUPLEX_MODE | RTL839X_FORCE_LINK_EN);
		break;
		case RTL9300_FAMILY_ID:
			reg |= (RTL930X_DUPLEX_MODE | RTL930X_FORCE_LINK_EN);
		break;
		case RTL9310_FAMILY_ID:
			reg |= (RTL931X_DUPLEX_MODE | RTL931X_FORCE_LINK_EN);
		break;
		}
	}

	switch(priv->family_id)
	{
	case RTL8380_FAMILY_ID:
		if (state->link)
			reg |= RTL830X_FORCE_LINK_EN;
		if (state->duplex == DUPLEX_FULL)
			reg |= RTL830X_DUPLEX_MODE;
	break;
	case RTL8390_FAMILY_ID:
		if (state->link)
			reg |= RTL839X_FORCE_LINK_EN;
		if (state->duplex == DUPLEX_FULL)
			reg |= RTL839X_DUPLEX_MODE;
	break;
	case RTL9300_FAMILY_ID:
		if (state->link)
			reg |= RTL930X_FORCE_LINK_EN;
		if (state->duplex == DUPLEX_FULL)
			reg |= RTL930X_DUPLEX_MODE;
	break;
	case RTL9310_FAMILY_ID:
		if (state->link)
			reg |= RTL931X_FORCE_LINK_EN;
		if (state->duplex == DUPLEX_FULL)
			reg |= RTL931X_DUPLEX_MODE;
	break;
	}
	// Disable AN
	if (priv->family_id == RTL8380_FAMILY_ID)
		reg &= ~RTL830X_NWAY_EN;
	sw_w32(reg, priv->r->mac_force_mode_ctrl(port));
}

static void rtl93xx_phylink_mac_config(struct dsa_switch *ds, int port,
					unsigned int mode,
					const struct phylink_link_state *state)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	int sds_num, sds_mode;
	u32 reg, v;
	u32 *p1 = 0xb8003308, *p2 = 0xb800330c;

	pr_info("%s port %d, mode %x, phy-mode: %s, speed %d, link %d\n", __func__,
		port, mode, phy_modes(state->interface), state->speed, state->link);
	pr_info("%s: %08x %08x\n", __func__, *p1, *p2);
	*p1 |= BIT(15);
	*p2 &= ~BIT(15);

	// BUG: Make this work on RTL93XX
	if (priv->family_id >= RTL9310_FAMILY_ID)
		return;

	// Nothing to be done for the CPU-port
	if (port == priv->cpu_port)
		return;

	// On the RTL930X, ports 24 to 27 are using an internal SerDes
	if (port >=24 && port <= 27) {
		sds_num = port - 18; // Port 24 mapped to SerDes 6, 25 to 7 ...
		switch (state->interface) {
		case PHY_INTERFACE_MODE_HSGMII:
			sds_mode = 0x12;
			break;
		case PHY_INTERFACE_MODE_1000BASEX:
			sds_mode = 0x04;
			break;
		case PHY_INTERFACE_MODE_XGMII:
			sds_mode = 0x10;
			break;
		case PHY_INTERFACE_MODE_10GKR:
			sds_mode = 0x1b;
			break;
		case PHY_INTERFACE_MODE_USXGMII:
			sds_mode = 0x0d;
			break;
		default:
			pr_err("%s: unknown serdes mode: %s\n",
			       __func__, phy_modes(state->interface));
			return;
		}
		rtl9300_sds_rst(sds_num, sds_mode);
	}

	reg = sw_r32(priv->r->mac_force_mode_ctrl(port));
	reg &= ~(0xf << 3);
	switch (state->speed) {
	case SPEED_10000:
		reg |= 4 << 3;
		break;
	case SPEED_2500:
		reg |= 5 << 3;
		break;
	case SPEED_5000:
		reg = 6 << 3;
		break;
	case SPEED_1000:
		pr_info("Setting PHY speed to 1000M\n");
		// BUG: SDS-num is hard-coded!
		v = rtl930x_read_sds_phy(8, 2, 0);
		v &= ~(BIT(6) | BIT(13));
		v |= BIT(6);
		rtl930x_write_sds_phy(8, 2, 0, v);

		reg |= 2 << 3;
		break;
	default:
		reg |= 2 << 3;
		break;
	}

	if (state->link)
		reg |= RTL930X_FORCE_LINK_EN;

	if (state->duplex == DUPLEX_FULL)
			reg |= RTL930X_DUPLEX_MODE;

	reg |= 1; // Force Link up
	sw_w32(reg, priv->r->mac_force_mode_ctrl(port));
}

static void rtl83xx_phylink_mac_link_down(struct dsa_switch *ds, int port,
				     unsigned int mode,
				     phy_interface_t interface)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	/* Stop TX/RX to port */
	pr_debug("%s: port %d down\n", __func__, port);
	sw_w32_mask(0x3, 0, priv->r->mac_port_ctrl(port));
}

static void rtl93xx_phylink_mac_link_down(struct dsa_switch *ds, int port,
				     unsigned int mode,
				     phy_interface_t interface)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	/* Stop TX/RX to port */
	sw_w32_mask(0x3, 0, priv->r->mac_port_ctrl(port));

	// No longer force link
	sw_w32_mask(3, 0, priv->r->mac_force_mode_ctrl(port));
}

static void rtl83xx_phylink_mac_link_up(struct dsa_switch *ds, int port,
				   unsigned int mode,
				   phy_interface_t interface,
				   struct phy_device *phydev)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	/* Restart TX/RX to port */
	sw_w32_mask(0, 0x3, priv->r->mac_port_ctrl(port));
}

static void rtl93xx_phylink_mac_link_up(struct dsa_switch *ds, int port,
				   unsigned int mode,
				   phy_interface_t interface,
				   struct phy_device *phydev)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	/* Restart TX/RX to port */
	sw_w32_mask(0, 0x3, priv->r->mac_port_ctrl(port));
}

static void rtl83xx_get_strings(struct dsa_switch *ds,
				int port, u32 stringset, u8 *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < ARRAY_SIZE(rtl83xx_mib); i++)
		strncpy(data + i * ETH_GSTRING_LEN, rtl83xx_mib[i].name,
			ETH_GSTRING_LEN);
}

static void rtl83xx_get_ethtool_stats(struct dsa_switch *ds, int port,
				      uint64_t *data)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	const struct rtl83xx_mib_desc *mib;
	int i;
	u64 h;

	for (i = 0; i < ARRAY_SIZE(rtl83xx_mib); i++) {
		mib = &rtl83xx_mib[i];

		data[i] = sw_r32(priv->r->stat_port_std_mib + (port << 8) + 252 - mib->offset);
		if (mib->size == 2) {
			h = sw_r32(priv->r->stat_port_std_mib + (port << 8) + 248 - mib->offset);
			data[i] |= h << 32;
		}
	}
}

static int rtl83xx_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return 0;

	return ARRAY_SIZE(rtl83xx_mib);
}

static u64 rtl83xx_mc_group_del_port(struct rtl838x_switch_priv *priv, int mc_group, int port);
static u64 rtl83xx_mc_group_add_port(struct rtl838x_switch_priv *priv, int mc_group, int port);

static void store_mcgroups(struct rtl838x_switch_priv *priv, int port)
{
	int mc_group;
	for (mc_group = 0; mc_group < MAX_MC_GROUPS; mc_group++) {
		u64 portmask = priv->r->read_mcast_pmask(mc_group);
		if (portmask & BIT_ULL(port)) {
			priv->mc_group_saves[mc_group] = port;
			rtl83xx_mc_group_del_port(priv, mc_group, port);
		}
	}
}
static void load_mcgroups(struct rtl838x_switch_priv *priv, int port)
{
	int mc_group;
	for (mc_group = 0; mc_group < MAX_MC_GROUPS; mc_group++) {
		if (priv->mc_group_saves[mc_group] == port) {
			rtl83xx_mc_group_add_port(priv, mc_group, port);
			priv->mc_group_saves[mc_group] = -1;
		}
	}
}

static int rtl83xx_port_enable(struct dsa_switch *ds, int port,
				struct phy_device *phydev)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 v;

	pr_debug("%s: %x %d", __func__, (u32) priv, port);


	priv->ports[port].enable = true;

	/* enable inner tagging on egress, do not keep any tags */
	if (priv->family_id == RTL9310_FAMILY_ID)
		sw_w32(BIT(4), priv->r->vlan_port_tag_sts_ctrl + (port << 2));
	else
		sw_w32(1, priv->r->vlan_port_tag_sts_ctrl + (port << 2));

	if (dsa_is_cpu_port(ds, port))
		return 0;


	/* add port to switch mask of CPU_PORT */
	priv->r->traffic_enable(priv->cpu_port, port);
	load_mcgroups(priv, port);
	
	if (priv->is_lagmember[port]) {
		pr_info("%s: %d is lag slave. ignore\n", __func__, port);
		return 0;
	}
	/* add all other ports in the same bridge to switch mask of port */
	v = priv->r->traffic_get(port);
	v |= priv->ports[port].pm;
	priv->r->traffic_set(port, v);


	// TODO: Figure out if this is necessary
	if (priv->family_id == RTL9300_FAMILY_ID) {
		sw_w32_mask(0, BIT(port), RTL930X_L2_PORT_SABLK_CTRL);
		sw_w32_mask(0, BIT(port), RTL930X_L2_PORT_DABLK_CTRL);
	}

	return 0;
}


static void rtl83xx_port_disable(struct dsa_switch *ds, int port)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 v;

	pr_debug("%s %x: %d", __func__, (u32)priv, port);
	/* you can only disable user ports */
	if (!dsa_is_user_port(ds, port))
		return;

	// BUG: This does not work on RTL931X
	/* remove port from switch mask of CPU_PORT */
	priv->r->traffic_disable(priv->cpu_port, port);
	store_mcgroups(priv, port);

	/* remove all other ports in the same bridge from switch mask of port */
	v = priv->r->traffic_get(port);
	v &= ~priv->ports[port].pm;
	priv->r->traffic_set(port, v);

	priv->ports[port].enable = false;
}

static int rtl83xx_set_mac_eee(struct dsa_switch *ds, int port,
			       struct ethtool_eee *e)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_debug("%s: %d\n", __func__, port);
	if (e->eee_enabled && !priv->eee_enabled) {
		pr_info("Globally enabling EEE\n");
		priv->r->init_eee(priv, true);
	}

	priv->r->port_eee_set(priv, port, e->eee_enabled);

	if (e->eee_enabled)
		pr_info("Enabled EEE for port %d\n", port);
	else
		pr_info("Disabled EEE for port %d\n", port);
	return 0;
}

static int rtl83xx_get_mac_eee(struct dsa_switch *ds, int port,
			       struct ethtool_eee *e)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_debug("%s: %d\n", __func__, port);
	e->supported = SUPPORTED_100baseT_Full | SUPPORTED_1000baseT_Full;

	priv->r->eee_port_ability(priv, e, port);

	e->eee_enabled = priv->ports[port].eee_enabled;

	e->eee_active = !!(e->advertised & e->lp_advertised);

	return 0;
}

static int rtl93xx_get_mac_eee(struct dsa_switch *ds, int port,
			       struct ethtool_eee *e)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	e->supported = SUPPORTED_100baseT_Full | SUPPORTED_1000baseT_Full
			| SUPPORTED_2500baseX_Full;

	priv->r->eee_port_ability(priv, e, port);

	e->eee_enabled = priv->ports[port].eee_enabled;

	e->eee_active = !!(e->advertised & e->lp_advertised);

	return 0;
}

/*
 * Set Switch L2 Aging time, t is time in milliseconds
 * t = 0: aging is disabled
 */
static int rtl83xx_set_l2aging(struct dsa_switch *ds, u32 t)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	int t_max = priv->family_id == RTL8380_FAMILY_ID ? 0x7fffff : 0x1FFFFF;

	/* Convert time in mseconds to internal value */
	if (t > 0x10000000) { /* Set to maximum */
		t = t_max;
	} else {
		if (priv->family_id == RTL8380_FAMILY_ID)
			t = ((t * 625) / 1000 + 127) / 128;
		else
			t = (t * 5 + 2) / 3;
	}
	sw_w32(t, priv->r->l2_ctrl_1);
	return 0;
}

static int rtl83xx_port_bridge_join(struct dsa_switch *ds, int port,
					struct net_device *bridge)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 port_bitmap = BIT_ULL(priv->cpu_port), v;
	int i;

	pr_debug("%s %x: %d %llx", __func__, (u32)priv, port, port_bitmap);
	if (priv->is_lagmember[port]) {
		pr_info("%s: %d is lag slave. ignore\n", __func__, port);
		return 0;
	}

	mutex_lock(&priv->reg_mutex);
	for (i = 0; i < ds->num_ports; i++) {
		/* Add this port to the port matrix of the other ports in the
		 * same bridge. If the port is disabled, port matrix is kept
		 * and not being setup until the port becomes enabled.
		 */
		if (dsa_is_user_port(ds, i) && i != port && !priv->is_lagmember[i]) {
			if (dsa_to_port(ds, i)->bridge_dev != bridge)
				continue;
			if (priv->ports[i].enable)
				priv->r->traffic_enable(i, port);

			priv->ports[i].pm |= BIT_ULL(port);
			port_bitmap |= BIT_ULL(i);
		}
	}
	load_mcgroups(priv, port);

	/* Add all other ports to this port matrix. */
	if (priv->ports[port].enable) {
		priv->r->traffic_enable(priv->cpu_port, port);
		v = priv->r->traffic_get(port);
		v |= port_bitmap;
		priv->r->traffic_set(port, v);
	}
	priv->ports[port].pm |= port_bitmap;
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static void rtl83xx_port_bridge_leave(struct dsa_switch *ds, int port,
					struct net_device *bridge)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 port_bitmap = BIT_ULL(priv->cpu_port), v;
	int i;

	pr_debug("%s %x: %d", __func__, (u32)priv, port);
	mutex_lock(&priv->reg_mutex);
	for (i = 0; i < ds->num_ports; i++) {
		/* Remove this port from the port matrix of the other ports
		 * in the same bridge. If the port is disabled, port matrix
		 * is kept and not being setup until the port becomes enabled.
		 * And the other port's port matrix cannot be broken when the
		 * other port is still a VLAN-aware port.
		 */
		if (dsa_is_user_port(ds, i) && i != port) {
			if (dsa_to_port(ds, i)->bridge_dev != bridge)
				continue;
			if (priv->ports[i].enable) {
				priv->r->traffic_disable(i, port);
			}
			priv->ports[i].pm |= BIT_ULL(port);
			port_bitmap &= ~BIT_ULL(i);
		}
	}
	store_mcgroups(priv, port);

	/* Add all other ports to this port matrix. */
	if (priv->ports[port].enable) {
		v = priv->r->traffic_get(port);
		v |= port_bitmap;
		priv->r->traffic_set(port, v);
	}
	priv->ports[port].pm &= ~port_bitmap;

	mutex_unlock(&priv->reg_mutex);
}

void rtl83xx_port_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	u32 msti = 0;
	u32 port_state[4];
	int index, bit;
	int pos = port;
	struct rtl838x_switch_priv *priv = ds->priv;
	int n = priv->port_width << 1;

	/* Ports above or equal CPU port can never be configured */
	if (port >= priv->cpu_port)
		return;

	mutex_lock(&priv->reg_mutex);

	/* For the RTL839x and following, the bits are left-aligned, 838x and 930x
	 * have 64 bit fields, 839x and 931x have 128 bit fields
	 */
	if (priv->family_id == RTL8390_FAMILY_ID)
		pos += 12;
	if (priv->family_id == RTL9300_FAMILY_ID)
		pos += 3;
	if (priv->family_id == RTL9310_FAMILY_ID)
		pos += 8;

	index = n - (pos >> 4) - 1;
	bit = (pos << 1) % 32;

	priv->r->stp_get(priv, msti, port_state);

	pr_debug("Current state, port %d: %d\n", port, (port_state[index] >> bit) & 3);
	port_state[index] &= ~(3 << bit);

	switch (state) {
	case BR_STATE_DISABLED: /* 0 */
		port_state[index] |= (0 << bit);
		break;
	case BR_STATE_BLOCKING:  /* 4 */
	case BR_STATE_LISTENING: /* 1 */
		port_state[index] |= (1 << bit);
		break;
	case BR_STATE_LEARNING: /* 2 */
		port_state[index] |= (2 << bit);
		break;
	case BR_STATE_FORWARDING: /* 3*/
		port_state[index] |= (3 << bit);
	default:
		break;
	}

	priv->r->stp_set(priv, msti, port_state);

	mutex_unlock(&priv->reg_mutex);
}

void rtl83xx_fast_age(struct dsa_switch *ds, int port)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	int s = priv->family_id == RTL8390_FAMILY_ID ? 2 : 0;

	pr_debug("FAST AGE port %d\n", port);
	mutex_lock(&priv->reg_mutex);
	/* RTL838X_L2_TBL_FLUSH_CTRL register bits, 839x has 1 bit larger
	 * port fields:
	 * 0-4: Replacing port
	 * 5-9: Flushed/replaced port
	 * 10-21: FVID
	 * 22: Entry types: 1: dynamic, 0: also static
	 * 23: Match flush port
	 * 24: Match FVID
	 * 25: Flush (0) or replace (1) L2 entries
	 * 26: Status of action (1: Start, 0: Done)
	 */
	sw_w32(1 << (26 + s) | 1 << (23 + s) | port << (5 + (s / 2)), priv->r->l2_tbl_flush_ctrl);

	do { } while (sw_r32(priv->r->l2_tbl_flush_ctrl) & BIT(26 + s));

	mutex_unlock(&priv->reg_mutex);
}

void rtl930x_fast_age(struct dsa_switch *ds, int port)
{
	struct rtl838x_switch_priv *priv = ds->priv;


	pr_debug("FAST AGE port %d\n", port);
	mutex_lock(&priv->reg_mutex);
	sw_w32(port << 11, RTL930X_L2_TBL_FLUSH_CTRL + 4);

	sw_w32(BIT(26) | BIT(30), RTL930X_L2_TBL_FLUSH_CTRL);

	do { } while (sw_r32(priv->r->l2_tbl_flush_ctrl) & BIT(30));

	mutex_unlock(&priv->reg_mutex);
}

static int rtl83xx_vlan_filtering(struct dsa_switch *ds, int port,
				  bool vlan_filtering)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	pr_debug("%s: port %d state %d\n", __func__, port, vlan_filtering);
	mutex_lock(&priv->reg_mutex);

	if (vlan_filtering) {
		/* Enable ingress and egress filtering
		 * The VLAN_PORT_IGR_FILTER register uses 2 bits for each port to define
		 * the filter action:
		 * 0: Always Forward
		 * 1: Drop packet
		 * 2: Trap packet to CPU port
		 * The Egress filter used 1 bit per state (0: DISABLED, 1: ENABLED)
		 */
		if (port != priv->cpu_port)
			sw_w32_mask(0b10 << ((port % 16) << 1), 0b01 << ((port % 16) << 1),
				    priv->r->vlan_port_igr_filter + ((port >> 4) << 2));
		sw_w32_mask(0, BIT(port % 32), priv->r->vlan_port_egr_filter + ((port >> 5) << 2));
		//sw_w32_mask(BIT(port % 32), 0, priv->r->vlan_port_egr_filter + ((port >> 5) << 2));  //BUG
	} else {
		/* Disable ingress and egress filtering */
		if (port != priv->cpu_port)
			sw_w32_mask(0b11 << ((port % 16) << 1), 0,
				    priv->r->vlan_port_igr_filter + ((port >> 4) << 2));
		sw_w32_mask(BIT(port % 32), 0, priv->r->vlan_port_egr_filter + ((port >> 5) << 2));
	}

	/* Do we need to do something to the CPU-Port, too? */
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static int rtl83xx_vlan_prepare(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan)
{
	struct rtl838x_vlan_info info;
	struct rtl838x_switch_priv *priv = ds->priv;

	mutex_lock(&priv->reg_mutex);
	priv->r->vlan_tables_read(0, &info);

	pr_debug("VLAN 0: Tagged ports %llx, untag %llx, profile %d, MC# %d, UC# %d, FID %x\n",
		info.tagged_ports, info.untagged_ports, info.profile_id,
		info.hash_mc_fid, info.hash_uc_fid, info.fid);

	priv->r->vlan_tables_read(1, &info);
	pr_debug("VLAN 1: Tagged ports %llx, untag %llx, profile %d, MC# %d, UC# %d, FID %x\n",
		info.tagged_ports, info.untagged_ports, info.profile_id,
		info.hash_mc_fid, info.hash_uc_fid, info.fid);
	priv->r->vlan_set_untagged(1, info.untagged_ports);
	pr_debug("SET: Untagged ports, VLAN %d: %llx\n", 1, info.untagged_ports);

	priv->r->vlan_set_tagged(1, &info);
	pr_debug("SET: Tagged ports, VLAN %d: %llx\n", 1, info.tagged_ports);

	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static void rtl83xx_vlan_add(struct dsa_switch *ds, int port,
			    const struct switchdev_obj_port_vlan *vlan)
{
	struct rtl838x_vlan_info info;
	struct rtl838x_switch_priv *priv = ds->priv;
	int v;

	pr_debug("%s port %d, vid_end %d, vid_end %d, flags %x\n", __func__,
		port, vlan->vid_begin, vlan->vid_end, vlan->flags);

	if (vlan->vid_begin > 4095 || vlan->vid_end > 4095) {
		dev_err(priv->dev, "VLAN out of range: %d - %d",
			vlan->vid_begin, vlan->vid_end);
		return;
	}

	mutex_lock(&priv->reg_mutex);

	if (vlan->flags & BRIDGE_VLAN_INFO_PVID) {
		for (v = vlan->vid_begin; v <= vlan->vid_end; v++) {
			if (!v)
				continue;
			/* Set both inner and outer PVID of the port */
			sw_w32((v << 16) | v << 2, priv->r->vlan_port_pb + (port << 2));
			priv->ports[port].pvid = vlan->vid_end;
		}
	}

	for (v = vlan->vid_begin; v <= vlan->vid_end; v++) {
		/* Get port memberships of this vlan */
		priv->r->vlan_tables_read(v, &info);

		/* new VLAN? */
		if (!info.tagged_ports) {
			info.fid = 0;
			info.hash_mc_fid = false;
			info.hash_uc_fid = false;
			info.profile_id = 0;
		}

		/* sanitize untagged_ports - must be a subset */
		if (info.untagged_ports & ~info.tagged_ports)
			info.untagged_ports = 0;

		info.tagged_ports |= BIT_ULL(port);
		if (vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED)
			info.untagged_ports |= BIT_ULL(port);

		priv->r->vlan_set_untagged(v, info.untagged_ports);
		pr_debug("Untagged ports, VLAN %d: %llx\n", v, info.untagged_ports);

		priv->r->vlan_set_tagged(v, &info);
		pr_debug("Tagged ports, VLAN %d: %llx\n", v, info.tagged_ports);
	}

	mutex_unlock(&priv->reg_mutex);
}

static int rtl83xx_vlan_del(struct dsa_switch *ds, int port,
			    const struct switchdev_obj_port_vlan *vlan)
{
	struct rtl838x_vlan_info info;
	struct rtl838x_switch_priv *priv = ds->priv;
	int v;
	u16 pvid;

	pr_debug("%s: port %d, vid_end %d, vid_end %d, flags %x\n", __func__,
		port, vlan->vid_begin, vlan->vid_end, vlan->flags);

	if (vlan->vid_begin > 4095 || vlan->vid_end > 4095) {
		dev_err(priv->dev, "VLAN out of range: %d - %d",
			vlan->vid_begin, vlan->vid_end);
		return -ENOTSUPP;
	}

	mutex_lock(&priv->reg_mutex);
	pvid = priv->ports[port].pvid;

	for (v = vlan->vid_begin; v <= vlan->vid_end; v++) {
		/* Reset to default if removing the current PVID */
		if (v == pvid)
			sw_w32(0, priv->r->vlan_port_pb + (port << 2));

		/* Get port memberships of this vlan */
		priv->r->vlan_tables_read(v, &info);

		/* remove port from both tables */
		info.untagged_ports &= (~BIT_ULL(port));
		info.tagged_ports &= (~BIT_ULL(port));

		priv->r->vlan_set_untagged(v, info.untagged_ports);
		pr_debug("Untagged ports, VLAN %d: %llx\n", v, info.untagged_ports);

		priv->r->vlan_set_tagged(v, &info);
		pr_debug("Tagged ports, VLAN %d: %llx\n", v, info.tagged_ports);
	}
	mutex_unlock(&priv->reg_mutex);

	return 0;
}

static void dump_l2_entry(struct rtl838x_l2_entry *e)
{
	pr_info("MAC: %02x:%02x:%02x:%02x:%02x:%02x vid: %d, rvid: %d, port: %d, valid: %d\n",
		e->mac[0], e->mac[1], e->mac[2], e->mac[3], e->mac[4], e->mac[5],
		e->vid, e->rvid, e->port, e->valid);

	if (e->type != L2_MULTICAST) {
		pr_info("Type: %d, is_static: %d, is_ip_mc: %d, is_ipv6_mc: %d, block_da: %d\n",
			e->type, e->is_static, e->is_ip_mc, e->is_ipv6_mc, e->block_da);
		pr_info("  block_sa: %d, susp: %d, nh: %d, age: %d, is_trunk: %d, trunk: %d\n",
		e->block_sa, e->suspended, e->next_hop, e->age, e->is_trunk, e->trunk);
	}
	if (e->type == L2_MULTICAST)
		pr_info("  L2_MULTICAST mc_portmask_index: %d\n", e->mc_portmask_index);
	if (e->is_ip_mc || e->is_ipv6_mc)
		pr_info("  mc_portmask_index: %d, mc_gip: %d, mc_sip: %d\n",
			e->mc_portmask_index, e->mc_gip, e->mc_sip);
	pr_info("  stack_dev: %d\n", e->stack_dev);
	if (e->next_hop)
		pr_info("  nh_route_id: %d\n", e->nh_route_id);
}

static void rtl83xx_setup_l2_uc_entry(struct rtl838x_l2_entry *e, int port, int vid, u64 mac)
{
	e->is_ip_mc = e->is_ipv6_mc  = false;
	e->valid = true;
	e->age = 3;
	e->port = port,
	e->vid = vid;
	u64_to_ether_addr(mac, e->mac);
}

static void rtl83xx_setup_l2_mc_entry(struct rtl838x_switch_priv *priv,
				      struct rtl838x_l2_entry *e, int vid, u64 mac, int mc_group)
{
	e->is_ip_mc = e->is_ipv6_mc  = false;
	e->valid = true;
	e->mc_portmask_index = mc_group;
	e->type = L2_MULTICAST;
	e->rvid = e->vid = vid;
	pr_debug("%s: vid: %d, rvid: %d\n", __func__, e->vid, e->rvid);
	u64_to_ether_addr(mac, e->mac);
}

/*
 * Uses the seed to identify a hash bucket in the L2 using the derived hash key and then loops
 * over the entries in the bucket until either a matching entry is found or an empty slot
 * Returns the filled in rtl838x_l2_entry and the index in the bucket when an entry was found
 * when an empty slot was found and must exist is false, the index of the slot is returned
 * when no slots are available returns -1
 */
static int rtl83xx_find_l2_hash_entry(struct rtl838x_switch_priv *priv, u64 seed,
				     bool must_exist, struct rtl838x_l2_entry *e)
{
	int i, idx = -1;
	u32 key = priv->r->l2_hash_key(priv, seed);
	u64 entry;

	pr_debug("%s: using key %x, for seed %016llx\n", __func__, key, seed);
	// Loop over all entries in the hash-bucket and over the second block on 93xx SoCs
	for (i = 0; i < priv->l2_bucket_size; i++) {
		entry = priv->r->read_l2_entry_using_hash(key, i, e);
		pr_debug("valid %d, mac %016llx\n", e->valid, ether_addr_to_u64(&e->mac[0]));
		if (must_exist && !e->valid)
			continue;
		if (!e->valid || ((entry & 0x0fffffffffffffffULL) == seed)) {
			idx = i > 3 ? ((key >> 14) & 0xffff) | i >> 1 : ((key << 2) | i) & 0xffff;
			break;
		}
	}

	return idx;
}

/*
 * Uses the seed to identify an entry in the CAM by looping over all its entries
 * Returns the filled in rtl838x_l2_entry and the index in the CAM when an entry was found
 * when an empty slot was found the index of the slot is returned
 * when no slots are available returns -1
 */
static int rtl83xx_find_l2_cam_entry(struct rtl838x_switch_priv *priv, u64 seed,
				     bool must_exist, struct rtl838x_l2_entry *e)
{
	int i, idx = -1;
	u64 entry;

	for (i = 0; i < 64; i++) {
		entry = priv->r->read_cam(i, e);
		if (!must_exist && !e->valid) {
			if (idx < 0) /* First empty entry? */
				idx = i;
			break;
		} else if ((entry & 0x0fffffffffffffffULL) == seed) {
			pr_debug("Found entry in CAM\n");
			idx = i;
			break;
		}
	}
	return idx;
}

static int rtl83xx_port_fdb_add(struct dsa_switch *ds, int port,
				const unsigned char *addr, u16 vid)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 mac = ether_addr_to_u64(addr);
	struct rtl838x_l2_entry e;
	int err = 0, idx;
	u64 seed = priv->r->l2_hash_seed(mac, vid);
	pr_debug("%s: %d\n", __func__, port);
	if (priv->is_lagmember[port]) {
		pr_info("%s: %d is lag slave. ignore\n", __func__, port);
		return 0;
	}
	mutex_lock(&priv->reg_mutex);

	idx = rtl83xx_find_l2_hash_entry(priv, seed, false, &e);

	// Found an existing or empty entry
	if (idx >= 0) {
		rtl83xx_setup_l2_uc_entry(&e, port, vid, mac);
		priv->r->write_l2_entry_using_hash(idx >> 2, idx & 0x3, &e);
		goto out;
	}

	// Hash buckets full, try CAM
	rtl83xx_find_l2_cam_entry(priv, seed, false, &e);

	if (idx >= 0) {
		rtl83xx_setup_l2_uc_entry(&e, port, vid, mac);
		priv->r->write_cam(idx, &e);
		goto out;
	}

	err = -ENOTSUPP;
out:
	mutex_unlock(&priv->reg_mutex);
	return err;
}

static int rtl83xx_port_fdb_del(struct dsa_switch *ds, int port,
			   const unsigned char *addr, u16 vid)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 mac = ether_addr_to_u64(addr);
	struct rtl838x_l2_entry e;
	int err = 0, idx;
	u64 seed = priv->r->l2_hash_seed(mac, vid);

	pr_debug("In %s, mac %llx, vid: %d\n", __func__, mac, vid);
	mutex_lock(&priv->reg_mutex);

	idx = rtl83xx_find_l2_hash_entry(priv, seed, true, &e);

	pr_debug("Found entry index %d, key %d and bucket %d\n", idx, idx >> 2, idx & 3);
	if (idx >= 0) {
		e.valid = false;
		dump_l2_entry(&e);
		priv->r->write_l2_entry_using_hash(idx >> 2, idx & 0x3, &e);
		goto out;
	}

	/* Check CAM for spillover from hash buckets */
	rtl83xx_find_l2_cam_entry(priv, seed, true, &e);

	if (idx >= 0) {
		e.valid = false;
		priv->r->write_cam(idx, &e);
		goto out;
	}
	err = -ENOENT;
out:
	mutex_unlock(&priv->reg_mutex);
	return err;
}

static int rtl83xx_port_fdb_dump(struct dsa_switch *ds, int port,
				 dsa_fdb_dump_cb_t *cb, void *data)
{
	struct rtl838x_l2_entry e;
	struct rtl838x_switch_priv *priv = ds->priv;
	int i;
	u32 fid, pkey, key;
	u64 mac, seed;

	mutex_lock(&priv->reg_mutex);

	for (i = 0; i < priv->fib_entries; i++) {
		priv->r->read_l2_entry_using_hash(i >> 2, i & 0x3, &e);

		if (!e.valid)
			continue;

		if (e.port == port || e.port == RTL930X_PORT_IGNORE) {
			fid = ((i >> 2) & 0x3ff) | (e.rvid & ~0x3ff);
			mac = ether_addr_to_u64(&e.mac[0]);
			pkey = priv->r->l2_hash_key(priv, priv->r->l2_hash_seed(mac, fid));
			fid = (pkey & 0x3ff) | (fid & ~0x3ff);
			pr_debug("-> index %d, key %x, bucket %d, dmac %016llx, fid: %x rvid: %x\n",
				i, i >> 2, i & 0x3, mac, fid, e.rvid);
			dump_l2_entry(&e);
			seed = priv->r->l2_hash_seed(mac, e.rvid);
			key = priv->r->l2_hash_key(priv, seed);
			pr_debug("seed: %016llx, key based on rvid: %08x\n", seed, key);
			cb(e.mac, e.vid, e.is_static, data);
		}
		if (e.type == L2_MULTICAST) {
			u64 portmask = priv->r->read_mcast_pmask(e.mc_portmask_index);
			if (portmask & BIT_ULL(port)) {
				dump_l2_entry(&e);
				pr_debug("  PM: %016llx\n", portmask);
			}
		}
	}

	for (i = 0; i < 64; i++) {
		priv->r->read_cam(i, &e);

		if (!e.valid)
			continue;

		if (e.port == port)
			cb(e.mac, e.vid, e.is_static, data);
	}

	
	mutex_unlock(&priv->reg_mutex);
	return 0;
}

static int rtl83xx_port_mdb_prepare(struct dsa_switch *ds, int port,
					const struct switchdev_obj_port_mdb *mdb)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	if (priv->id >= 0x9300)
		return -EOPNOTSUPP;

	return 0;
}

static int rtl83xx_mc_group_alloc(struct rtl838x_switch_priv *priv, int port)
{
	int mc_group = find_first_zero_bit(priv->mc_group_bm, MAX_MC_GROUPS - 1);
	u64 portmask;

	if (mc_group >= MAX_MC_GROUPS - 1)
		return -1;

	pr_debug("%s: %d\n", __func__, port);
	if (priv->is_lagmember[port]) {
		pr_info("%s: %d is lag slave. ignore\n", __func__, port);
		return 0;
	}
	pr_debug("Using MC group %d\n", mc_group);
	set_bit(mc_group, priv->mc_group_bm);
	mc_group++;  // We cannot use group 0, as this is used for lookup miss flooding
	portmask = BIT_ULL(port) | BIT_ULL(priv->cpu_port); 
	priv->r->write_mcast_pmask(mc_group, portmask);

	return mc_group;
}

static u64 rtl83xx_mc_group_add_port(struct rtl838x_switch_priv *priv, int mc_group, int port)
{
	u64 portmask = priv->r->read_mcast_pmask(mc_group);

	pr_debug("%s: %d\n", __func__, port);
	if (priv->is_lagmember[port]) {
		pr_info("%s: %d is lag slave. ignore\n", __func__, port);
		return portmask;
	}
	portmask |= BIT_ULL(port);
	priv->r->write_mcast_pmask(mc_group, portmask);

	return portmask;
}

static u64 rtl83xx_mc_group_del_port(struct rtl838x_switch_priv *priv, int mc_group, int port)
{
	u64 portmask = priv->r->read_mcast_pmask(mc_group);

	pr_debug("%s: %d\n", __func__, port);
	if (priv->is_lagmember[port]) {
		pr_info("%s: %d is lag slave. ignore\n", __func__, port);
		return portmask;
	}
	priv->r->write_mcast_pmask(mc_group, portmask);
	if (portmask == BIT_ULL(priv->cpu_port)) {
		portmask &= ~BIT_ULL(priv->cpu_port);
		priv->r->write_mcast_pmask(mc_group, portmask);
		clear_bit(mc_group, priv->mc_group_bm);
	}

	return portmask;
}

static void rtl83xx_port_mdb_add(struct dsa_switch *ds, int port,
			const struct switchdev_obj_port_mdb *mdb)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 mac = ether_addr_to_u64(mdb->addr);
	struct rtl838x_l2_entry e;
	int err = 0, idx;
	int vid = mdb->vid;
	u64 seed = priv->r->l2_hash_seed(mac, vid);
	int mc_group;

	pr_debug("%s: %d\n", __func__, port);
	if (priv->is_lagmember[port]) {
		pr_info("%s: %d is lag slave. ignore\n", __func__, port);
		return;
	}
	pr_debug("In %s port %d, mac %llx, vid: %d\n", __func__, port, mac, vid);
	mutex_lock(&priv->reg_mutex);

	idx = rtl83xx_find_l2_hash_entry(priv, seed, false, &e);
	
	// Found an existing or empty entry
	if (idx >= 0) {
		if (e.valid) {
			pr_debug("Found an existing entry %016llx, mc_group %d\n",
				ether_addr_to_u64(e.mac), e.mc_portmask_index);
			rtl83xx_mc_group_add_port(priv, e.mc_portmask_index, port);
		} else {
			pr_debug("New entry for seed %016llx\n", seed);
			mc_group = rtl83xx_mc_group_alloc(priv, port);
			if (mc_group < 0) {
				err = -ENOTSUPP;
				goto out;
			}
			rtl83xx_setup_l2_mc_entry(priv, &e, vid, mac, mc_group);
			priv->r->write_l2_entry_using_hash(idx >> 2, idx & 0x3, &e);
		}
		goto out;
	}

	// Hash buckets full, try CAM
	rtl83xx_find_l2_cam_entry(priv, seed, false, &e);

	if (idx >= 0) {
		if (e.valid) {
			pr_debug("Found existing CAM entry %016llx, mc_group %d\n",
				 ether_addr_to_u64(e.mac), e.mc_portmask_index);
			rtl83xx_mc_group_add_port(priv, e.mc_portmask_index, port);
		} else {
			pr_debug("New entry\n");
			mc_group = rtl83xx_mc_group_alloc(priv, port);
			if (mc_group < 0) {
				err = -ENOTSUPP;
				goto out;
			}
			rtl83xx_setup_l2_mc_entry(priv, &e, vid, mac, mc_group);
			priv->r->write_cam(idx, &e);
		}
		goto out;
	}

	err = -ENOTSUPP;
out:
	mutex_unlock(&priv->reg_mutex);
	if (err)
		dev_err(ds->dev, "failed to add MDB entry\n");
}

int rtl83xx_port_mdb_del(struct dsa_switch *ds, int port,
			const struct switchdev_obj_port_mdb *mdb)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 mac = ether_addr_to_u64(mdb->addr);
	struct rtl838x_l2_entry e;
	int err = 0, idx;
	int vid = mdb->vid;
	u64 seed = priv->r->l2_hash_seed(mac, vid);
	u64 portmask;
	pr_debug("%s: %d\n", __func__, port);
	if (priv->is_lagmember[port]) {
		pr_info("%s: %d is lag slave. ignore\n", __func__, port);
		return 0;
	}

	pr_debug("In %s, port %d, mac %llx, vid: %d\n", __func__, port, mac, vid);
	mutex_lock(&priv->reg_mutex);

	idx = rtl83xx_find_l2_hash_entry(priv, seed, true, &e);

	pr_debug("Found entry index %d, key %d and bucket %d\n", idx, idx >> 2, idx & 3);
	if (idx >= 0) {
		portmask = rtl83xx_mc_group_del_port(priv, e.mc_portmask_index, port);
		if (!portmask) {
			e.valid = false;
			// dump_l2_entry(&e);
			priv->r->write_l2_entry_using_hash(idx >> 2, idx & 0x3, &e);
		}
		goto out;
	}

	/* Check CAM for spillover from hash buckets */
	rtl83xx_find_l2_cam_entry(priv, seed, true, &e);

	if (idx >= 0) {
		portmask = rtl83xx_mc_group_del_port(priv, e.mc_portmask_index, port);
		if (!portmask) {
			e.valid = false;
			// dump_l2_entry(&e);
			priv->r->write_cam(idx, &e);
		}
		goto out;
	}
	// TODO: Re-enable with a newer kernel: err = -ENOENT;
out:
	mutex_unlock(&priv->reg_mutex);
	return err;
}

static int rtl83xx_port_mirror_add(struct dsa_switch *ds, int port,
				   struct dsa_mall_mirror_tc_entry *mirror,
				   bool ingress)
{
	/* We support 4 mirror groups, one destination port per group */
	int group, err = 0;
	struct rtl838x_switch_priv *priv = ds->priv;
	struct rtl838x_vlan_info info;
	int ctrl_reg, dpm_reg, spm_reg;
	int v;
	pr_debug("In %s\n", __func__);

	mutex_lock(&priv->reg_mutex);

	for (group = 0; group < 4; group++) {
		if (priv->mirror_group_ports[group] == mirror->to_local_port)
			break;
	}
	if (group >= 4) {
		for (group = 0; group < 4; group++) {
			if (priv->mirror_group_ports[group] < 0)
				break;
		}
	}

	if (group >= 4) {
		err = -ENOSPC;
		goto out;
	}

	ctrl_reg = priv->r->mir_ctrl + (group * 4);
	dpm_reg = priv->r->mir_dpm + ((group << 2) * priv->port_width);
	spm_reg = priv->r->mir_spm + ((group << 2) * priv->port_width);
	pr_debug("Using group %d local port %d, port %d\n", group, mirror->to_local_port, port);
	

	if (priv->family_id == RTL8380_FAMILY_ID) {
		/* Enable mirroring to port across VLANs (bit 11) */
		sw_w32(1 << 11 | ( mirror->to_local_port << 4) | 1, ctrl_reg);
	} else {
		/* Enable mirroring to destination port */
		sw_w32(( mirror->to_local_port << 4) | 1, ctrl_reg);
	}

	if (ingress && (priv->r->get_port_reg_be(spm_reg) & (1ULL << port))) {
		err = -EEXIST;
		goto out;
	}
	if ((!ingress) && (priv->r->get_port_reg_be(dpm_reg) & (1ULL << port))) {
		err = -EEXIST;
		goto out;
	}
	if (ingress)
		priv->r->mask_port_reg_be(0, 1ULL << port, spm_reg);
	else
		priv->r->mask_port_reg_be(0, 1ULL << port, dpm_reg);

	priv->mirror_group_ports[group] = mirror->to_local_port;
	out:
	mutex_unlock(&priv->reg_mutex);
	return err;
}

static void rtl83xx_port_mirror_del(struct dsa_switch *ds, int port,
				    struct dsa_mall_mirror_tc_entry *mirror)
{
	int group = 0;
	struct rtl838x_switch_priv *priv = ds->priv;
	int ctrl_reg, dpm_reg, spm_reg;

	pr_debug("In %s\n", __func__);
	mutex_lock(&priv->reg_mutex);
	for (group = 0; group < 4; group++) {
		if (priv->mirror_group_ports[group] == mirror->to_local_port)
			break;
	}
	if (group >= 4)
		goto out;

	ctrl_reg = priv->r->mir_ctrl + group * 4;
	dpm_reg = priv->r->mir_dpm + (group << 2) * priv->port_width;
	spm_reg = priv->r->mir_spm + (group << 2) * priv->port_width;

	if (mirror->ingress) {
		/* Ingress, clear source port matrix */
		priv->r->mask_port_reg_be(1ULL << port, 0, spm_reg);
	} else {
		/* Egress, clear destination port matrix */
		priv->r->mask_port_reg_be(1ULL << port, 0, dpm_reg);
	}
	
	if (!(priv->r->get_port_reg_be(dpm_reg) & (1ULL << port)) && !(priv->r->get_port_reg_be(spm_reg) & (1ULL << port))) {

		priv->mirror_group_ports[group] = -1;
		sw_w32(0, ctrl_reg);
	}
	out:
	mutex_unlock(&priv->reg_mutex);
}

static bool rtl83xx_lag_can_offload(struct dsa_switch *ds,
				      struct net_device *lag,
				      struct netdev_lag_upper_info *info)
{
	struct dsa_port *dp;
	int id;

	id = dsa_lag_id(ds->dst, lag);
	if (id < 0 || id >= ds->num_lag_ids)
		return false;

	if (info->tx_type != NETDEV_LAG_TX_TYPE_HASH) {
		return false;
	}
	if (info->hash_type != NETDEV_LAG_HASH_L2 && info->hash_type != NETDEV_LAG_HASH_L23)
		return false;

	return true;
}

static void add_portmatrix(struct dsa_switch *ds,int port)
{
	int i;
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 port_bitmap = BIT_ULL(priv->cpu_port), v;

	for (i = 0; i < ds->num_ports; i++) {
		/* Add this port to the port matrix of the other ports in the
		 * same bridge. If the port is disabled, port matrix is kept
		 * and not being setup until the port becomes enabled.
		 */
		if (dsa_is_user_port(ds, i) && i != port && !priv->is_lagmember[i]) {
			if (priv->ports[i].enable) {
				priv->r->traffic_enable(i, port);
			}
			priv->ports[i].pm |= BIT_ULL(port);
			port_bitmap |= BIT_ULL(i);
		}
		if (priv->is_lagmember[i] || i == port) {
		
			priv->ports[i].pm &= ~BIT_ULL(i);
			port_bitmap &= ~BIT_ULL(i);
		
		}
	}
	load_mcgroups(priv, port);
	/* Add all other ports to this port matrix. */
	if (priv->ports[port].enable) {
		priv->r->traffic_enable(priv->cpu_port, port);
		v = priv->r->traffic_get(port);
		v |= port_bitmap;
		priv->r->traffic_set(port, v);
	}
	priv->ports[port].pm |= port_bitmap;


}
/* pointless? */
static int rtl83xx_port_lag_change(struct dsa_switch *ds, int port)
{
	struct dsa_port *dp;
	struct net_device *lag;
	unsigned int id;
	int num_tx, i,a;
	struct rtl838x_switch_priv *priv = ds->priv;
	u64 port_bitmap = BIT_ULL(priv->cpu_port), v;
	pr_debug("%s: %d\n", __func__, port);
	mutex_lock(&priv->reg_mutex);

	mutex_unlock(&priv->reg_mutex);
	return 0;
}

void rtl83xx_port_stp_state_set(struct dsa_switch *ds, int port, u8 state);

static int rtl83xx_port_lag_join(struct dsa_switch *ds, int port,
				   struct net_device *lag,
				   struct netdev_lag_upper_info *info)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	int i, err = 0;

	if (!rtl83xx_lag_can_offload(ds, lag, info))
		return -EOPNOTSUPP;

	mutex_lock(&priv->reg_mutex);

	for (i = 0; i < priv->n_lags; i++) {
		if ((!priv->lag_devs[i]) || (priv->lag_devs[i] == lag))
			break;
	}
	if (port >= priv->cpu_port) {
		err = -EINVAL;
		goto out;
	}
	pr_info("port_lag_join: group %d, port %d\n",i, port); 
	if (!priv->lag_devs[i])
		priv->lag_devs[i] = lag;

	if (priv->lag_primary[i]==-1) {
		priv->lag_primary[i]=port;
	} else
		priv->is_lagmember[port] = 1;

	priv->lagmembers |= (1ULL << port);

	pr_debug("lag_members = %llX\n", priv->lagmembers);
	err = rtl83xx_lag_add(priv->ds, i, port, info);
	if (err) {
		err = -EINVAL;
		goto out;
	}

out:
	mutex_unlock(&priv->reg_mutex);
	return err;

}

static int rtl83xx_port_lag_leave(struct dsa_switch *ds, int port,
				    struct net_device *lag)
{
	int i, group = -1, err;
	struct rtl838x_switch_priv *priv = ds->priv;
	
	mutex_lock(&priv->reg_mutex);
	for (i=0;i<priv->n_lags;i++) {
		if (priv->lags_port_members[i] & BIT_ULL(port)) {
			group = i;
			break;
		}
	}
	
	if (group == -1) {
		pr_info("port_lag_leave: port %d is not a member\n", port); 
		err = -EINVAL;
		goto out;
	}

	if (port >= priv->cpu_port) {
		err = -EINVAL;
		goto out;
	}
	pr_info("port_lag_del: group %d, port %d\n",group, port); 
	priv->lagmembers &=~ (1ULL << port);
	priv->lag_primary[i] = -1;
	priv->is_lagmember[port] = 0;
	pr_debug("lag_members = %llX\n", priv->lagmembers);
	err = rtl83xx_lag_del(priv->ds, group, port);
	if (err) {
		err = -EINVAL;
		goto out;
	}
	if (!priv->lags_port_members[i])
		priv->lag_devs[i] = NULL;

out:
	mutex_unlock(&priv->reg_mutex);
	return 0;
}
static int rtl83xx_port_pre_bridge_flags(struct dsa_switch *ds, int port, unsigned long flags, struct netlink_ext_ack *extack)
{
	struct rtl838x_switch_priv *priv = ds->priv;
	unsigned long features = 0;
	pr_debug("%s: %d %lX\n", __func__, port, flags);
	if (priv->r->enable_learning)
		    features |= BR_LEARNING;
	if (priv->r->enable_flood)
		    features |= BR_FLOOD;
	if (priv->r->enable_mcast_flood)
		    features |= BR_MCAST_FLOOD;
	if (priv->r->enable_bcast_flood)
		    features |= BR_BCAST_FLOOD;
	if (flags & ~(features))
		return -EINVAL;
	
	return 0;
}

static int rtl83xx_port_bridge_flags(struct dsa_switch *ds, int port, unsigned long flags, struct netlink_ext_ack *extack)
{
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_debug("%s: %d %lX\n", __func__, port, flags);
	if (priv->r->enable_learning)
		priv->r->enable_learning(port, !!(flags & BR_LEARNING));
	    
	if (priv->r->enable_flood)
		priv->r->enable_flood(port, !!(flags & BR_FLOOD));

	if (priv->r->enable_mcast_flood)
		priv->r->enable_mcast_flood(port, !!(flags & BR_MCAST_FLOOD));

	if (priv->r->enable_bcast_flood)
		priv->r->enable_bcast_flood(port, !!(flags & BR_BCAST_FLOOD));

	return 0;
	
}

int dsa_phy_read(struct dsa_switch *ds, int phy_addr, int phy_reg)
{
	u32 val;
	u32 offset = 0;
	struct rtl838x_switch_priv *priv = ds->priv;

	pr_debug("%s: %X:%X\n", __func__, phy_addr, phy_reg);
	if (priv->family_id == RTL8380_FAMILY_ID) {
	if (phy_addr >= 24 && phy_addr <= 27
		&& priv->ports[24].phy == PHY_RTL838X_SDS) {
		if (phy_addr == 26)
			offset = 0x100;
		val = sw_r32(RTL838X_SDS4_FIB_REG0 + offset + (phy_reg << 2)) & 0xffff;
		return val;
	}
	}
	read_phy(phy_addr, 0, phy_reg, &val);
	return val;
}

int dsa_phy_write(struct dsa_switch *ds, int phy_addr, int phy_reg, u16 val)
{
	u32 offset = 0;
	struct rtl838x_switch_priv *priv = ds->priv;
	pr_debug("%s: %X:%X\n", __func__, phy_addr, phy_reg);

	if (priv->family_id == RTL8380_FAMILY_ID) {
	if (phy_addr >= 24 && phy_addr <= 27
	     && priv->ports[24].phy == PHY_RTL838X_SDS) {
		if (phy_addr == 26)
			offset = 0x100;
		sw_w32(val, RTL838X_SDS4_FIB_REG0 + offset + (phy_reg << 2));
		return 0;
	}
	}
	return write_phy(phy_addr, 0, phy_reg, val);
}

const struct dsa_switch_ops rtl83xx_switch_ops = {
	.get_tag_protocol	= rtl83xx_get_tag_protocol,
	.setup			= rtl83xx_setup,

	.phy_read		= dsa_phy_read,
	.phy_write		= dsa_phy_write,

	.phylink_validate	= rtl83xx_phylink_validate,
	.phylink_mac_link_state	= rtl83xx_phylink_mac_link_state,
	.phylink_mac_config	= rtl83xx_phylink_mac_config,
	.phylink_mac_link_down	= rtl83xx_phylink_mac_link_down,
	.phylink_mac_link_up	= rtl83xx_phylink_mac_link_up,


	.get_strings		= rtl83xx_get_strings,
	.get_ethtool_stats	= rtl83xx_get_ethtool_stats,
	.get_sset_count		= rtl83xx_get_sset_count,

	.port_enable		= rtl83xx_port_enable,
	.port_disable		= rtl83xx_port_disable,

	.get_mac_eee		= rtl83xx_get_mac_eee,
	.set_mac_eee		= rtl83xx_set_mac_eee,

	.set_ageing_time	= rtl83xx_set_l2aging,
	.port_bridge_join	= rtl83xx_port_bridge_join,
	.port_bridge_leave	= rtl83xx_port_bridge_leave,
	.port_stp_state_set	= rtl83xx_port_stp_state_set,
	.port_fast_age		= rtl83xx_fast_age,

	.port_vlan_filtering	= rtl83xx_vlan_filtering,
	.port_vlan_prepare	= rtl83xx_vlan_prepare,
	.port_vlan_add		= rtl83xx_vlan_add,
	.port_vlan_del		= rtl83xx_vlan_del,

	.port_fdb_add		= rtl83xx_port_fdb_add,
	.port_fdb_del		= rtl83xx_port_fdb_del,
	.port_fdb_dump		= rtl83xx_port_fdb_dump,

	.port_mdb_prepare	= rtl83xx_port_mdb_prepare,
	.port_mdb_add		= rtl83xx_port_mdb_add,
	.port_mdb_del		= rtl83xx_port_mdb_del,

	.port_mirror_add	= rtl83xx_port_mirror_add,
	.port_mirror_del	= rtl83xx_port_mirror_del,

	.port_lag_change	= rtl83xx_port_lag_change,
	.port_lag_join		= rtl83xx_port_lag_join,
	.port_lag_leave		= rtl83xx_port_lag_leave,
	.port_pre_bridge_flags  = rtl83xx_port_pre_bridge_flags,
	.port_bridge_flags  = rtl83xx_port_bridge_flags,
};

const struct dsa_switch_ops rtl930x_switch_ops = {
	.get_tag_protocol	= rtl83xx_get_tag_protocol,
	.setup			= rtl930x_setup,

	.phy_read		= dsa_phy_read,
	.phy_write		= dsa_phy_write,

	.phylink_validate	= rtl93xx_phylink_validate,
	.phylink_mac_link_state	= rtl93xx_phylink_mac_link_state,
	.phylink_mac_config	= rtl93xx_phylink_mac_config,
	.phylink_mac_link_down	= rtl93xx_phylink_mac_link_down,
	.phylink_mac_link_up	= rtl93xx_phylink_mac_link_up,

	.get_strings		= rtl83xx_get_strings,
	.get_ethtool_stats	= rtl83xx_get_ethtool_stats,
	.get_sset_count		= rtl83xx_get_sset_count,

	.port_enable		= rtl83xx_port_enable,
	.port_disable		= rtl83xx_port_disable,

	.get_mac_eee		= rtl93xx_get_mac_eee,
	.set_mac_eee		= rtl83xx_set_mac_eee,

	.set_ageing_time	= rtl83xx_set_l2aging,
	.port_bridge_join	= rtl83xx_port_bridge_join,
	.port_bridge_leave	= rtl83xx_port_bridge_leave,
	.port_stp_state_set	= rtl83xx_port_stp_state_set,
	.port_fast_age		= rtl930x_fast_age,

	.port_vlan_filtering	= rtl83xx_vlan_filtering,
	.port_vlan_prepare	= rtl83xx_vlan_prepare,
	.port_vlan_add		= rtl83xx_vlan_add,
	.port_vlan_del		= rtl83xx_vlan_del,

	.port_fdb_add		= rtl83xx_port_fdb_add,
	.port_fdb_del		= rtl83xx_port_fdb_del,
	.port_fdb_dump		= rtl83xx_port_fdb_dump,

	.port_mdb_prepare	= rtl83xx_port_mdb_prepare,
	.port_mdb_add		= rtl83xx_port_mdb_add,
	.port_mdb_del		= rtl83xx_port_mdb_del,

	.port_lag_change	= rtl83xx_port_lag_change,
	.port_lag_join		= rtl83xx_port_lag_join,
	.port_lag_leave		= rtl83xx_port_lag_leave,
	
	.port_pre_bridge_flags  = rtl83xx_port_pre_bridge_flags,
	.port_bridge_flags  = rtl83xx_port_bridge_flags,
};
