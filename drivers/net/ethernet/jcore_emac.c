/*
 * sei_emac.c	SEI EMAC driver
 *
 * Copyright (c) 2012 Smart Energy Instruments Inc.
 *               by Oleksandr G Zhadan
 *
 */
#include <linux/init.h>
#include <net/sock.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/udp.h>
#include <linux/platform_device.h>
#include <linux/etherdevice.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <net/inet_sock.h>

#include <linux/net_tstamp.h>
#include <linux/timecounter.h>
#include <linux/clocksource.h>

#define Dprintk(fmt, ...)

#define AFprintkTX(format, args...)
//#define AFprintkTX printk

#define AFprintkRX(format, args...)
//#define AFprintkRX printk

//#define DEBUG_ISR
#ifdef DEBUG_ISR
#define AFprintkISR printk
#else
#define AFprintkISR(format, args...)
#endif

//#define DEBUG_TX
#ifdef DEBUG_TX
#define AFprintkTXP printk
#else
#define AFprintkTXP(format, args...)
#endif

#define ASSERT(cond) { if((cond)) { panic("Assertion failed %s at %s:%d\n", #cond, __FILE__, __LINE__); } }

#define TX_TIMEOUT (2 * HZ)

#define	SEI_EMAC_PINGPONG_BUFFERS	4

#define SEI_EMAC_BASE		0xABCE0000
#define SEI_EMAC_CONTROL	0xABCE0000
#define SEI_EMAC_STATUS		0xABCE0000
#define SEI_EMAC_DEBUG		0xABCE0010

/* Base/Control/Status registers bits */
#define SEI_EMAC_RESET		0x00000000
#define SEI_EMAC_ENABLE_RX	0x00000002
#define SEI_EMAC_ENABLE_TX	0x00000004
#define SEI_EMAC_BUSY		0x00000004
#define SEI_EMAC_XMIT		0x00000004
#define SEI_EMAC_MCAST		0x00000008
#define SEI_EMAC_READ		0x00000010
#define SEI_EMAC_ENABLE_INT_RX	0x00000020
#define SEI_EMAC_ENABLE_INT_TX	0x00000040
#define SEI_EMAC_PROMISC	0x00000080
#define SEI_EMAC_COMPLETE	0x00000100
#define SEI_EMAC_CRC		0x00000200

#define SEI_EMAC_TX_LEN	0xABCE0004
#define SEI_EMAC_MACL	0xABCE0008
#define SEI_EMAC_MACH	0xABCE000C
#define SEI_EMAC_RX_BUF	0xABCE1000
#define SEI_EMAC_TX_BUF	0xABCE1800

#define SEI_EMAC_MAX_MULTICAST_ADDRS	4

// This is the system clock as kept by the FPGA
#define SEI_TS_SEC_MSW			0xABCD0220
#define SEI_TS_SEC_LSW			0xABCD0224
#define SEI_TS_NSEC			0xABCD0228

#define SEI_EMAC_TX_SOF_TS_SEC_MSW	0xABCE0020
#define SEI_EMAC_TX_SOF_TS_SEC_LSW	0xABCE0024
#define SEI_EMAC_TX_SOF_TS_NSEC		0xABCE0028

#define SEI_EMAC_RX_SOF_TS_SEC_MSW	0xABCE0030
#define SEI_EMAC_RX_SOF_TS_SEC_LSW	0xABCE0034
#define SEI_EMAC_RX_SOF_TS_NSEC		0xABCE0038

#define SEI_EMAC_TS_OFFSET		0xABCE0040 // where to insert the TS in the L2 frame, > 12

#define SEI_EMAC_MCAST_MASK0		0xABCE0060 // mask for mcast frames
#define SEI_EMAC_MCAST_MASK1		0xABCE0064 // mask for mcast frames
#define SEI_EMAC_MCAST_MASK2		0xABCE0068 // mask for mcast frames
#define SEI_EMAC_MCAST_MASK3		0xABCE006C // mask for mcast frames




#define READ_DEBUG_REG (readl(SEI_EMAC_DEBUG))

enum { FALSE, TRUE };

static u32* sei_emac_hw_mcast_list[] = {
  (u32*)SEI_EMAC_MCAST_MASK0,
  (u32*)SEI_EMAC_MCAST_MASK1,
  (u32*)SEI_EMAC_MCAST_MASK2,
  (u32*)SEI_EMAC_MCAST_MASK3
};

#define sei_emac_hwtstamp_is_none(cfg) ((cfg) == HWTSTAMP_FILTER_NONE)

#define PTP_EVENT_PORT		319
#define PTP_GENERAL_PORT	320
#define PTP_TS_OFFSET_V1	40 // in payload, on frame depends on UDPv4, UDPv6 or ETH_P_1588
#define PTP_TS_OFFSET_V2	34 // in payload, on frame depends on UDPv4, UDPv6 or ETH_P_1588

/* Values for the IEEE 1588 messageType field */
#define SYNC                  0x0
#define DELAY_REQ             0x1
#define PDELAY_REQ            0x2
#define PDELAY_RESP           0x3
#define FOLLOW_UP             0x8
#define DELAY_RESP            0x9
#define PDELAY_RESP_FOLLOW_UP 0xA
#define ANNOUNCE              0xB
#define SIGNALING             0xC
#define MANAGEMENT            0xD

#define MAX_TX_ITEMS	16
#define MAX_RX_ITEMS	32
#define MAX_RX_RING_SZ	MAX_RX_ITEMS/2

static int ptp_msg_types[] = {
  SYNC,
  DELAY_REQ,
  PDELAY_REQ,
  PDELAY_RESP,
  FOLLOW_UP,
  DELAY_RESP,
  PDELAY_RESP_FOLLOW_UP,
  ANNOUNCE,
  SIGNALING,
  MANAGEMENT,
};
#define PTP_MSG_TYPE_SZ (sizeof(ptp_msg_types) / sizeof(ptp_msg_types[0]))

struct {
  u32 tx_pkt;						///< All TXed packets
  u32 tx_pkt_ts;					///< Pushed to RTL with TX hw tstamp
  u32 tx_pkt_ts_mark;					///< Marked for TX hw tstamp in SKB flags
  u32 tx_pkt_inval_off;					///< Calculated offset is invalid (< 14)
  u32 tx_pkt_inval_ts;					///< TX Timestamp is (0, 0)
  u32 rx_pkt;						///< RXed packets when SKBs in RX ring available
  u32 all_irq;						///< All IRQs (TX/RX/CRC-error)
  u32 tx_irq;
  u32 rx_irq;
  u32 rx_err;
  u32 rx_full;						///< RX ring full (rx_items)
  u32 tx_full;						///< TX ring full (tx_items)
  u32 rx_refill;					///< how many times we (actually) refilled the RX ring
  u32 rx_refill_sz[MAX_RX_RING_SZ];
  u32 rx_pingpong_stats[SEI_EMAC_PINGPONG_BUFFERS];
  u32 ptp_rx_stats[MANAGEMENT+1];
  struct sei_emac_private* seip; ///< XXX HACK!
  u32 multicast_all;					///< Shadow of private data multicast_all
} Stats;

typedef struct {
	struct sk_buff* skb;
	u32    tx_ts_off;
	u64    sec;
	u32    nsec;
} tx_item_t;

typedef struct {
	struct sk_buff* skb;
	u64    sec;
	u32    nsec;
} rx_item_t;

struct sei_emac_private {
	u32 sig; ///< Signature: 0xdeadbeef
	struct net_device* dev;

	int opened;
	volatile int tx;
	volatile u32 tx_ts_off;
	struct sk_buff* tx_skb;

	spinlock_t rx_lock;
	spinlock_t tx_lock;

	tx_item_t tx_items[MAX_TX_ITEMS];
	rx_item_t rx_items[MAX_RX_ITEMS];
	struct sk_buff* rx_ring[MAX_RX_RING_SZ]; ///< pre-allocated and refilled in BH to speed up ISR
	struct work_struct refill_work;
	int refill_warning; ///< warn only once per rx ring full event

	struct napi_struct napi;

	// flags
	u32   promisc;
	u32   multicast_all;

        /* OS defined structs */
	volatile struct hwtstamp_config stamp_cfg;

        struct cyclecounter cycles;
        struct timecounter clock;
        struct hwtstamp_config hwtstamp_config;
};

#if 0
static int hex_dump8(const char* msg, const u8* p, int const sz);
#endif

static inline void read_ts(u64* sec, u32* nsec)
{
	u32 sec_msw;
	u32 sec_lsw;
	u32 nsec_;
	if(unlikely(sec == NULL || nsec == NULL)) return;

	do {
		sec_msw = readl(SEI_TS_SEC_MSW);
		sec_lsw = readl(SEI_TS_SEC_LSW);
		nsec_   = readl(SEI_TS_NSEC);
	} while (sec_msw != readl(SEI_TS_SEC_MSW) || sec_lsw != readl(SEI_TS_SEC_LSW));
	
	*sec  = ((sec_msw + 0LL) << 32) | (sec_lsw + 0LL);
	*nsec = nsec_;
}

static inline void read_tx_ts(u64* sec, u32* nsec)
{
	u32 sec_msw;
	u32 sec_lsw;
	u32 nsec_;
	if(unlikely(sec == NULL || nsec == NULL)) return;

	sec_msw = readl(SEI_EMAC_TX_SOF_TS_SEC_MSW);
	sec_lsw = readl(SEI_EMAC_TX_SOF_TS_SEC_LSW);
	nsec_   = readl(SEI_EMAC_TX_SOF_TS_NSEC);

	*sec  = ((sec_msw + 0LL) << 32) | (sec_lsw + 0LL);
	*nsec = nsec_;
}

static inline void read_rx_ts(u64* sec, u32* nsec)
{
	u32 sec_msw;
	u32 sec_lsw;
	u32 nsec_;
	if(unlikely(sec == NULL || nsec == NULL)) return;

	sec_msw = readl(SEI_EMAC_RX_SOF_TS_SEC_MSW);
	sec_lsw = readl(SEI_EMAC_RX_SOF_TS_SEC_LSW);
	nsec_   = readl(SEI_EMAC_RX_SOF_TS_NSEC);

	*sec  = ((sec_msw + 0LL) << 32) | (sec_lsw + 0LL);
	*nsec = nsec_;
}

static void sei_emac_set_hw_addr(struct net_device *dev)
{
	u8 addr[ETH_ALEN];
	uint32_t addr_lo = readl(SEI_EMAC_MACL);
	uint32_t addr_hi = readl(SEI_EMAC_MACH);

	addr[5] = addr_lo & 0xff;
	addr[4] = addr_lo >> 8;
	addr[3] = addr_lo >> 16;
	addr[2] = addr_lo >> 24;
	addr[1] = addr_hi & 0xff;
	addr[0] = addr_hi >> 8;
	eth_hw_addr_set(dev, addr);
}

static int sei_emac_set_mac_address(struct net_device *dev, void *p)
{
	struct sockaddr *addr = p;
	uint32_t addr_lo = 0;
	uint32_t addr_hi = 0;

	if (netif_running(dev))
		return -EBUSY;

	eth_hw_addr_set(dev, addr->sa_data);
	addr_lo |= (dev->dev_addr[5] | (dev->dev_addr[4] << 8) |
		    (dev->dev_addr[3] << 16) | (dev->dev_addr[2] << 24));
	addr_hi |= (dev->dev_addr[1] | (dev->dev_addr[0] << 8));

	writel(addr_lo, SEI_EMAC_MACL);
	writel(addr_hi, SEI_EMAC_MACH);

	return 0;
}

static inline void sei_emac_configure(const struct sei_emac_private* seip)
{
	uint32_t ctrl;
	if(seip == NULL) return;

	ctrl = readl(SEI_EMAC_CONTROL);

        if(seip->promisc)
		ctrl |= SEI_EMAC_PROMISC;
	else 
		ctrl &= ~SEI_EMAC_PROMISC;

        if(seip->multicast_all)
		ctrl |= SEI_EMAC_MCAST;
	else 
		ctrl &= ~SEI_EMAC_MCAST;

#ifdef DEBUG
	printk("%s: %s%sCONTROL=0x%08x\n", __func__, 
               (seip->promisc? "p ": ""),
               (seip->multicast_all? "m ": ""),
               ctrl);
#endif

	writel(ctrl, SEI_EMAC_CONTROL);
}

static void sei_emac_set_multicast_list(struct net_device *dev)
{
	struct sei_emac_private* seip = netdev_priv(dev);
        
        if (dev->flags & IFF_PROMISC)
                seip->promisc = 1;
        else
                seip->promisc = 0;

        if (dev->flags & IFF_ALLMULTI)
                seip->multicast_all = 1;
        else if (dev->flags & IFF_MULTICAST) {
		if(netdev_mc_count(dev) > SEI_EMAC_MAX_MULTICAST_ADDRS)
                	seip->multicast_all = 1;
		else {
			int ha_count = 0;
			struct netdev_hw_addr* ha = NULL;

			netdev_for_each_mc_addr(ha, dev) {
				int k = -1;
				u8* addr = ha->addr;
				const u32 addr32 = addr[0]<<24 | addr[1]<<16 | addr[2] << 8;
				for(k = 0; k < ha_count; k++) {
					if(*sei_emac_hw_mcast_list[k] == addr32) goto skip;
				}
				Dprintk(KERN_INFO "SEI EMAC: mcast MAC at %d is %08x\n", ha_count, addr32);
				*sei_emac_hw_mcast_list[ha_count++] = addr32;
				continue;
			   skip:
				Dprintk(KERN_INFO "SEI EMAC: mcast MAC %08x is programmed, skipping.\n", addr32);
			}

                	seip->multicast_all = 0;
		}
        } else    
                seip->multicast_all = 0;
                
	Stats.multicast_all = seip->multicast_all;

        sei_emac_configure(seip);
}               

static inline void sei_wait_until_ready(struct net_device *dev)
{
	while (readl(SEI_EMAC_STATUS) & SEI_EMAC_BUSY)
		asm("nop; nop");
}

static inline void sei_emac_stop(struct net_device *dev)
{
	sei_wait_until_ready(dev);
	writel(SEI_EMAC_RESET, SEI_EMAC_BASE);
}

static inline void sei_emac_start(struct net_device *dev)
{
	volatile uint32_t ctrl = readl(SEI_EMAC_CONTROL);

	//ctrl |= (SEI_EMAC_ENABLE_RX | SEI_EMAC_ENABLE_INT_RX | SEI_EMAC_READ);

	ctrl |= (SEI_EMAC_ENABLE_RX | SEI_EMAC_ENABLE_INT_RX | SEI_EMAC_ENABLE_INT_TX | SEI_EMAC_READ);
	ctrl &= ~(SEI_EMAC_XMIT);
	
	writel(ctrl, SEI_EMAC_CONTROL);
}

static void sei_emac_restart(struct net_device *dev)
{
	sei_emac_stop(dev);
	udelay(10);
	sei_emac_start(dev);
}

static void sei_emac_timeout(struct net_device *dev, unsigned woo)
{
	dev->stats.tx_errors++;
	sei_emac_restart(dev);
	netif_wake_queue(dev);
}


static inline int sei_emac_can_ts_tx(const u8 msg_type)
{
	switch(msg_type) {
	  case SIGNALING:
	  case MANAGEMENT:
	  //case DELAY_REQ:
	  //case PDELAY_REQ:
		return 0;
	  default:
		break;
	}

	return 1;
}

static inline const char* type_ts(const u8 msg_type)
{
	switch(msg_type) {
	  case SYNC:                  return "SYNC"; break;
	  case DELAY_REQ:             return "DELAY_REQ"; break;
	  case PDELAY_REQ:            return "PDELAY_REQ"; break;
	  case PDELAY_RESP:           return "PDELAY_RESP"; break;
	  case FOLLOW_UP:             return "FOLLOW_UP"; break;
	  case DELAY_RESP:            return "DELAY_RESP"; break;
	  case PDELAY_RESP_FOLLOW_UP: return "PDELAY_RESP_FOLLOW_UP"; break;
	  case ANNOUNCE:              return "ANNOUNCE"; break;
	  case SIGNALING:             return "SIGNALING"; break;
	  case MANAGEMENT:            return "MANAGEMENT"; break;
	  default:
		break;
	}

	return "???";
}

static inline int sei_emac_ptp_ver(const u8* ptp_hdr)
{
	return ptp_hdr[1] & 0x0F;
}
static inline int sei_emac_ptpv2_type(const u8* ptp_hdr)
{
	return ptp_hdr[0] & 0x0F;
}

static u32 sei_emac_tx_ts_offset(const struct sk_buff* skb)
{
	u16 l2proto  = 0;
        u8  ptp_ver  = 0;
        u8  ptp_type = -1;
        u16 ptp_port = 0;

	u32 fpga_off = 0;

	if(skb == NULL) return 0;

	if(! (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)) return 0;

	do {
	  struct sock* sk;
          u8* skb_data;
	  if(!skb) continue;
	  if(!skb->sk) continue;

	  sk = skb->sk;
          skb_data = skb->data;

	  l2proto = ((struct ethhdr*)skb_data)->h_proto;
	  AFprintkTX("%s: L2 proto = 0x%x\n", __func__, l2proto);
	  if(l2proto == htons(ETH_P_1588)) {
	    const u8* ptp_hdr = skb_data + sizeof(struct ethhdr);
            ptp_ver = sei_emac_ptp_ver(ptp_hdr);

            fpga_off = sizeof(struct ethhdr);
            fpga_off += (ptp_ver == 2)? PTP_TS_OFFSET_V2: PTP_TS_OFFSET_V1;

            if(ptp_ver == 2) {
              ptp_type = sei_emac_ptpv2_type(ptp_hdr);
	      AFprintkTX("%s: PTP %s\n", __func__, type_ts(ptp_type));
	      if(! sei_emac_can_ts_tx(ptp_type))
                fpga_off = 0;
            }

            continue;
          }

	  AFprintkTX("%s: proto = %d\n", __func__, sk->sk_protocol);
	  if(sk->sk_protocol != IPPROTO_UDP) continue;

	  AFprintkTX("%s: af = %d\n", __func__, sk->sk_family);
	  if(sk->sk_family == AF_INET) { // UDPv4
            struct udphdr* udph = (struct udphdr*)(skb_data + sizeof(struct ethhdr) + sizeof(struct iphdr));
	    u8* ptp_hdr;
	    AFprintkTX("%s: dport = %d\n", __func__, ntohs(udph->dest));

            ptp_port = ntohs(udph->dest);
            if(ptp_port != PTP_EVENT_PORT &&
               ptp_port != PTP_GENERAL_PORT) continue;

	    ptp_hdr = (u8*)udph + sizeof(struct udphdr);
            ptp_ver = sei_emac_ptp_ver(ptp_hdr);

	    fpga_off = sizeof(struct ethhdr);
            fpga_off += sizeof(struct iphdr) + sizeof(struct udphdr);
            fpga_off += (ptp_ver == 2)? PTP_TS_OFFSET_V2: PTP_TS_OFFSET_V1;

            if(ptp_ver == 2) {
              ptp_type = sei_emac_ptpv2_type(ptp_hdr);
	      AFprintkTX("%s: PTP %s\n", __func__, type_ts(ptp_type));
	      if(! sei_emac_can_ts_tx(ptp_type))
                fpga_off = 0;
            }

	    continue;
	  }

	  if(sk->sk_family == AF_INET6) { // UDPv6
	    struct udphdr* udph = (struct udphdr*)(skb_data + sizeof(struct ethhdr) + sizeof(struct ipv6hdr));
	    u8* ptp_hdr;
	    AFprintkTX("%s: dport = %d\n", __func__, ntohs(udph->dest));

            ptp_port = ntohs(udph->dest);
            if(ptp_port != PTP_EVENT_PORT &&
               ptp_port != PTP_GENERAL_PORT) continue;

	    ptp_hdr = (u8*)udph + sizeof(struct udphdr);
            ptp_ver = sei_emac_ptp_ver(ptp_hdr);

	    fpga_off = sizeof(struct ethhdr);
	    fpga_off += sizeof(struct ipv6hdr) + sizeof(struct udphdr);
            fpga_off += (ptp_ver == 2)? PTP_TS_OFFSET_V2: PTP_TS_OFFSET_V1;

            if(ptp_ver == 2) {
              ptp_type = sei_emac_ptpv2_type(ptp_hdr);
	      AFprintkTX("%s: PTP %s\n", __func__, type_ts(ptp_type));
	      if(! sei_emac_can_ts_tx(ptp_type))
                fpga_off = 0;
            }

	    continue;
	  }
	} while(0);

	AFprintkTX("%s: l2proto 0x%x PTPv%d port %d type %s fpga_off = %d\n", __func__,
                 l2proto, ptp_ver, ptp_port, type_ts(ptp_type),
                 fpga_off);
	return fpga_off;
}

static u32 sei_emac_rx_is_ptp(const struct sk_buff* skb)
{
	u32 ts_off   = 0;
        u8  ptp_ver  = 0;
        u8  ptp_type = -1;
        u16 ptp_port = 0;
	u8  udp_ver  = 0;
	u16 l2proto;
	u8 IP_ver;
	u8* ptp_hdr;

	if(skb == NULL) return 0;

	l2proto = skb->protocol;
	if(l2proto == htons(ETH_P_1588)) {
	    const u8* ptp_hdr = skb->data;
            ptp_ver = sei_emac_ptp_ver(ptp_hdr);

            ts_off = (ptp_ver == 2)? PTP_TS_OFFSET_V2: PTP_TS_OFFSET_V1;
	    goto done_ptp;
	}

	if(l2proto != htons(ETH_P_IP) && l2proto != htons(ETH_P_IPV6)) goto done_notptp;

	IP_ver = ((u8*)skb->data)[0] >> 4;
	if(IP_ver != 4 && IP_ver != 6) goto done_notptp;

	udp_ver = IP_ver;
        if(IP_ver == 4) { // UDPv4
		struct udphdr* udph = (struct udphdr*)(skb->data + sizeof(struct iphdr));
		ptp_port = ntohs(udph->dest);
	} else { // UDPv6
		struct udphdr* udph = (struct udphdr*)(skb->data + sizeof(struct ipv6hdr));
		ptp_port = ntohs(udph->dest);
	}

        if(ptp_port != PTP_EVENT_PORT &&
           ptp_port != PTP_GENERAL_PORT) goto done_notptp;

	ptp_hdr = skb->data;
        ptp_hdr += (IP_ver == 4)? sizeof(struct iphdr): sizeof(struct ipv6hdr);
	ptp_hdr += sizeof(struct udphdr);

	ptp_ver = sei_emac_ptp_ver(ptp_hdr);

	ts_off = (IP_ver == 4)? sizeof(struct iphdr): sizeof(struct ipv6hdr);
        ts_off += sizeof(struct udphdr);

	if(ptp_ver == 1) {
	      ts_off += PTP_TS_OFFSET_V1;
	} else if(ptp_ver == 2) {
	      ts_off += PTP_TS_OFFSET_V2;
              ptp_type = sei_emac_ptpv2_type(ptp_hdr);
	} else { ts_off = 0; }

 done_ptp:
	if(ptp_type >= 0 && ptp_type <= MANAGEMENT)
		Stats.ptp_rx_stats[ptp_type]++;
	AFprintkRX("%s: l2proto 0x%x PTPv%d UDPv%d port %d type %s ts_off = %d\n", __func__,
                 l2proto, ptp_ver, udp_ver, ptp_port, type_ts(ptp_type),
                 ts_off);
	return ts_off;

 done_notptp:
	return 0;
}

static int sei_emac_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct sei_emac_private *seip = netdev_priv(dev);
	unsigned long flags;
	bool isHwTS;
	bool isHwTX;

	if (!seip->opened)
		return NETDEV_TX_BUSY;

	if (readl(SEI_EMAC_STATUS) & SEI_EMAC_BUSY)
		sei_wait_until_ready(dev);

	spin_lock_irqsave(&seip->tx_lock, flags);

#ifdef DEBUG_TX
	AFprintkTXP("Sending length=%u bytes\n", skb->len);
	if (1) {
		uint8_t *buf = (uint8_t *)skb->data;
		uint8_t *buf_end = buf + skb->len;
		int i = 0;
		while (buf < buf_end) {
			for (i = 0 ; (i < 8) && (buf < buf_end); i++) {
				AFprintkTXP("%02X ", *(buf++));
			}
			AFprintkTXP("\n");
		}
		AFprintkTXP("CRC:");
		for (i = 0 ; i < 4; i++) {
			AFprintkTXP(" %02X", *(buf++));
		}
		AFprintkTXP("\n");
	}
#endif
	memcpy((void *)SEI_EMAC_TX_BUF, skb->data, skb->len);
	if (skb->len < 60)
		writel(60, SEI_EMAC_TX_LEN);
	else 
		writel(skb->len, SEI_EMAC_TX_LEN);

	isHwTS = FALSE;
	if(skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) {
		Stats.tx_pkt_ts_mark++;
		isHwTS = TRUE;
	}
	Dprintk("%s: seip->stamp_cfg.tx_type = %d\n", __func__, seip->stamp_cfg.tx_type);
	isHwTX = seip->stamp_cfg.tx_type == HWTSTAMP_TX_ON;
	if(isHwTX && isHwTS) {
		u32 fpga_off = sei_emac_tx_ts_offset(skb);

		if(fpga_off > 14) { // gt L2 len
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

			seip->tx_ts_off = fpga_off; // before it's clobbered

			// HACK: FPGA-assist reversed ;)
	  		if(skb->sk->sk_protocol == IPPROTO_UDP) {
				fpga_off |= 1L << 31; // indicate to FPGA this is UDP and needs checksum update
				if(skb->sk->sk_family == AF_INET6) 
					fpga_off |= 1L << 30; // indicate to FPGA this is UDP/IPv6
			}

			///printk("SEI_EMAC_TS_OFFSET := 0x%08x\n", fpga_off);

			writel(fpga_off, SEI_EMAC_TS_OFFSET);

			Stats.tx_pkt_ts++;

			seip->tx_skb    = skb; // store SKB for later: do HW ts in ISR
			seip->tx        = 1;
		} else {
			Stats.tx_pkt_inval_off++;
		}
	} else {
		seip->tx        = 0;
		seip->tx_ts_off = 0;
		seip->tx_skb    = NULL;
	}

	// do TX
	writel((readl(SEI_EMAC_STATUS) | SEI_EMAC_ENABLE_TX), SEI_EMAC_CONTROL);

	Stats.tx_pkt++;

	spin_unlock_irqrestore(&seip->tx_lock, flags);

	netif_trans_update(dev); // 4.7
	//dev->trans_start = jiffies; // 4.6
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

static void sei_emac_systim_to_hwtstamp(struct sei_emac_private* seip,
                                        struct skb_shared_hwtstamps* shhwtstamps,
					u64 regval)
{
	u64 ns;
	if(seip == NULL || shhwtstamps == NULL) return;

        ns = timecounter_cyc2time(&seip->clock, regval);

        memset(shhwtstamps, 0, sizeof(struct skb_shared_hwtstamps));
        shhwtstamps->hwtstamp = ns_to_ktime(ns);
}

/** \brief Refills the RX ring of buffers
 * \note Called from locked context
 */
static int sei_emac_refill_rx_ring(struct net_device *dev)
{
	struct sei_emac_private* seip;
	int i;
	int refill_count = 0;

	if(dev == NULL) return -EINVAL;
	seip = netdev_priv(dev);

	for(i = 0; i < MAX_RX_RING_SZ; i++) {
		struct sk_buff* skb;
		if(seip->rx_ring[i] != NULL) continue;

		skb = netdev_alloc_skb(dev, 1600 + NET_IP_ALIGN);
		if(skb == NULL) {
			panic("SEI EMAC: Cannot fill RX ring!\n");
			return -ENOMEM;
		}

                skb_reserve(skb, NET_IP_ALIGN);
		seip->rx_ring[i] = skb;
		refill_count++;
	}

	if(refill_count) {
		Stats.rx_refill++;
		Stats.rx_refill_sz[refill_count - 1]++;
		seip->refill_warning = 0;
	}

	return refill_count;
}

/** \brief Detaches one SKB from RX ring
 * \note Called from locked context
 */
static struct sk_buff* sei_emac_pop_rx_ring(const struct net_device *dev)
{
	struct sei_emac_private* seip;
	int i;
	struct sk_buff* skb;

	if(dev == NULL) return NULL;
	seip = netdev_priv(dev);

	for(i = 0; i < MAX_RX_RING_SZ; i++) {
		if(seip->rx_ring[i] == NULL) continue;

		skb = seip->rx_ring[i];
		seip->rx_ring[i] = NULL;
		return skb;
	}

	return NULL;
}

static inline int sei_emac_poll_tx(struct sei_emac_private* seip, int budget)
{
	int work_done = 0;
	if(seip == NULL) return 0;
	ASSERT(seip->sig != 0xdeadbeefL);

	do {
		tx_item_t tx;
		int i;
		unsigned long flags;
		u64 regval;
		struct skb_shared_hwtstamps shhwtstamps;
		memset(&tx, 0, sizeof(tx));

		spin_lock_irqsave(&seip->tx_lock, flags);
		for(i = MAX_TX_ITEMS - 1; i >= 0; i--) {
			if(seip->tx_items[i].skb != NULL) {
				memcpy(&tx, &seip->tx_items[i], sizeof(tx_item_t));
				memset(&seip->tx_items[i], 0, sizeof(tx_item_t));
				break;
			}
		}
		spin_unlock_irqrestore(&seip->tx_lock, flags);

		if(tx.skb == NULL) break;

		Dprintk("TX offs %u HW ts %llu.%u (0x%llx/0x%08x)\n",
		       tx.tx_ts_off,
		       tx.sec, tx.nsec,
		       tx.sec, tx.nsec);

		if(tx.tx_ts_off > 0 && tx.sec == 0 && tx.nsec == 0) 
			Stats.tx_pkt_inval_ts++;

		regval = tx.sec*1000000000LL + tx.nsec;

		sei_emac_systim_to_hwtstamp(seip, &shhwtstamps, regval);

		skb_tstamp_tx(tx.skb, &shhwtstamps);

		work_done++;
	} while(work_done < budget);

	return work_done;
}

static inline int sei_emac_poll_rx(struct sei_emac_private* seip, int budget)
{
	int work_done = 0;
	if(seip == NULL) return 0;
	ASSERT(seip->sig != 0xdeadbeefL);

	do {
		rx_item_t rx;
		int i;
		unsigned long flags;
		u32 ts_off;
		u64 regval;
		memset(&rx, 0, sizeof(rx));

		spin_lock_irqsave(&seip->rx_lock, flags);
		for(i = MAX_RX_ITEMS - 1; i >= 0; i--) {
			if(seip->rx_items[i].skb != NULL) {
				memcpy(&rx, &seip->rx_items[i], sizeof(rx_item_t));
				memset(&seip->rx_items[i], 0, sizeof(rx_item_t));
				break;
			}
		}
		spin_unlock_irqrestore(&seip->rx_lock, flags);

		if(rx.skb == NULL) break;

		ts_off = sei_emac_rx_is_ptp(rx.skb); // do pkt parsing and find if is PTP

		if(!sei_emac_hwtstamp_is_none(seip->stamp_cfg.rx_filter) &&
		   rx.sec > 0 && ts_off > 0) {
			Dprintk("RX fltr 0x%x off %u %llu.%us\n", seip->stamp_cfg.rx_filter, ts_off, rx.sec, (unsigned int)rx.nsec);

			regval = rx.sec*1000000000LL + rx.nsec;
			sei_emac_systim_to_hwtstamp(seip, skb_hwtstamps(rx.skb), regval);
		}

		netif_rx(rx.skb);

		work_done++;
	} while(work_done < budget);

	return work_done;
}

static int sei_emac_poll(struct napi_struct* napi, int budget)
{
	struct sei_emac_private* seip = container_of(napi, struct sei_emac_private, napi);
	unsigned int work_done = 0;
	unsigned long flags;

	ASSERT(seip->sig != 0xdeadbeefL);

	work_done += sei_emac_poll_rx(seip, budget/2);
	work_done += sei_emac_poll_tx(seip, budget/2);

	spin_lock_irqsave(&seip->rx_lock, flags);
	sei_emac_refill_rx_ring(seip->dev);
	spin_unlock_irqrestore(&seip->rx_lock, flags);

	napi_complete(napi);

        return work_done;
}

static void sei_emac_refill_work(struct work_struct* work)
{
        struct sei_emac_private* seip =
                container_of(work, struct sei_emac_private, refill_work);
	unsigned long flags;
	int cnt;

	ASSERT(seip->sig != 0xdeadbeefL);

	spin_lock_irqsave(&seip->rx_lock, flags);
	cnt = sei_emac_refill_rx_ring(seip->dev);
	spin_unlock_irqrestore(&seip->rx_lock, flags);

	printk("SEI EMAC rx ring refill kicked by ISR, added %d skb(s)\n", cnt);
}

static inline void sei_emac_interrupt_tx(struct sei_emac_private* seip) // this is an ISR
{
	tx_item_t* tx = NULL;
	if(seip == NULL) return;

	ASSERT(seip->sig != 0xdeadbeefL);

	//unsigned long flags;
	//spin_lock_irqsave(&seip->tx_lock, flags);

	if(seip->tx != 0 && seip->tx_skb != NULL) {
		u64 sec  = 0;
		u32 nsec = 0;
		int i;
		read_tx_ts(&sec, &nsec);
		Dprintk("TXi %llu.%us\n", sec, (unsigned int)nsec);

		Stats.tx_irq++;

		for(i = 0; i < MAX_TX_ITEMS; i++) {
			if(seip->tx_items[i].skb == NULL) { tx = &seip->tx_items[i]; break; }
		}
		if(tx == NULL) {
			Stats.tx_full++;
			printk(KERN_ERR "%s: no empty slots in tx_items! (%d)\n", __func__, Stats.tx_full);
		} else {
			tx->skb       = seip->tx_skb;
			tx->tx_ts_off = seip->tx_ts_off;
			tx->sec       = sec;
			tx->nsec      = nsec;
		}

		seip->tx        = 0;
		seip->tx_skb    = NULL;
		seip->tx_ts_off = 0;
	}

	//spin_unlock_irqrestore(&seip->tx_lock, flags);

	if(tx == NULL) return;

	if (likely(napi_schedule_prep(&seip->napi))) // not in locked context
		__napi_schedule(&seip->napi);
}

static irqreturn_t sei_emac_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct sei_emac_private *seip = netdev_priv(dev);
	uint32_t status;
	int rx_count;
	uint32_t pkt_len;
	struct sk_buff* skb;
	//unsigned long flags;

	Stats.all_irq++;

	status = readl(SEI_EMAC_STATUS);

        AFprintkISR("isr status 0x%08x debug 0x%08x\n", status, READ_DEBUG_REG);

	if ((status & SEI_EMAC_BUSY) == 0) // a TX-complete IRQ
		sei_emac_interrupt_tx(seip);

	for (rx_count = 0;
             rx_count < SEI_EMAC_PINGPONG_BUFFERS && (status & SEI_EMAC_COMPLETE) != 0;
             rx_count++) { // a RX IRQ
		if (status & SEI_EMAC_CRC) {
			dev->stats.rx_errors++;
			Stats.rx_err++;

#ifdef DEBUG_ISR
			/* Log contents of CRC error packet */
			pkt_len = status >> 16;
			AFprintkISR("CRC Error pkt length=%u bytes\n", pkt_len);
			if (0) {
				uint8_t *buf = (uint8_t *)SEI_EMAC_RX_BUF;
				uint8_t *buf_end = buf + pkt_len;
				int i = 0;
				while (buf < buf_end) {
					for (i = 0 ; (i < 8) && (buf < buf_end); i++) {
						AFprintkISR("%02X ", *(buf++));
					}
					AFprintkISR("\n");
				}
				AFprintkISR("CRC:");
				for (i = 0 ; i < 4; i++) {
					AFprintkISR(" %02X", *(buf++));
				}
				AFprintkISR("\n");
			}
#endif

			goto next; //out;
		}

		if(rx_count == 0)
			Stats.rx_irq++;

		Stats.rx_pingpong_stats[rx_count]++;

		pkt_len = status >> 16;

#ifdef DEBUG_ISR
		//AFprintkISR("Good Pkt  pkt length=%u bytes\n", pkt_len);
		if (1) {
			uint8_t *buf = (uint8_t *)SEI_EMAC_RX_BUF;
			uint8_t *buf_end = buf + pkt_len;
			int i = 0;
			AFprintkISR("Good Pkt  pkt length=%u bytes\n", pkt_len);
			while (buf < buf_end) {
				for (i = 0 ; (i < 8) && (buf < buf_end); i++) {
					AFprintkISR("%02X ", *(buf++));
				}
				AFprintkISR("\n");
			}
			AFprintkISR("CRC:");
			for (i = 0 ; i < 4; i++) {
				AFprintkISR(" %02X", *(buf++));
			}
			AFprintkISR("\n");
		}
#endif
		skb = NULL; //netdev_alloc_skb(dev, pkt_len + NET_IP_ALIGN);

		//spin_lock_irqsave(&seip->rx_lock, flags);
		skb = sei_emac_pop_rx_ring(dev);
		//spin_unlock_irqrestore(&seip->rx_lock, flags);

		if (skb != NULL) {
			u64 sec  = 0;
			u32 nsec = 0;
			rx_item_t* rx = NULL;
			int i;

			//skb_reserve(skb, NET_IP_ALIGN);
			memcpy(skb_put(skb, pkt_len), (uint8_t *)SEI_EMAC_RX_BUF, pkt_len);
			skb->dev = dev;
			skb->protocol = eth_type_trans(skb, dev);

			if(! sei_emac_hwtstamp_is_none(seip->stamp_cfg.rx_filter)) {
				read_rx_ts(&sec, &nsec);
				Dprintk("RXi %llu.%us\n", sec, (unsigned int)nsec);
			}

			//unsigned long flags;
			//spin_lock_irqsave(&seip->rx_lock, flags);

			for(i = 0; i < MAX_RX_ITEMS; i++) {
				if(seip->rx_items[i].skb == NULL) { rx = &seip->rx_items[i]; break; }
			}
			if(rx == NULL) {
				Stats.rx_full++;
				printk(KERN_ERR "%s: no empty slots in rx_items! (%d)\n", __func__, Stats.rx_full);
				kfree_skb(skb);
			} else {
				Stats.rx_pkt++;
				rx->skb       = skb;
				rx->sec       = sec;
				rx->nsec      = nsec;
			}
			//spin_unlock_irqrestore(&seip->rx_lock, flags);

			if(rx != NULL) {
				dev->stats.rx_packets++;
				dev->stats.rx_bytes += pkt_len;
			}

			// must kick BH regardless of value of rx to give it a chance to run
			if (likely(napi_schedule_prep(&seip->napi))) // not in locked context
				__napi_schedule(&seip->napi);
		} else {
			if(! seip->refill_warning) {
				seip->refill_warning = 1;
				printk(KERN_ERR "%s: no skbs in rx_ring! (rx_count = %d)\n", __func__, rx_count);
			}
			dev->stats.rx_dropped++;

			// WHOOAA NAPI BH did not get a chance to run!!
			schedule_work(&seip->refill_work);
		}

	    next:
		writel((readl(SEI_EMAC_STATUS) | SEI_EMAC_READ), SEI_EMAC_CONTROL);

		status = readl(SEI_EMAC_STATUS);

		AFprintkISR("followup isr status 0x%08x debug 0x%08x\n", status, READ_DEBUG_REG);

		// RX is slow so we poll here in case we have a TX event
		if ((status & SEI_EMAC_BUSY) == 0) // a TX-complete IRQ
			sei_emac_interrupt_tx(seip);
	} // END for SEI_EMAC_PINGPONG_BUFFERS

//out:
	return IRQ_HANDLED;
}

static int sei_emac_open(struct net_device *dev)
{
	struct sei_emac_private *seip = netdev_priv(dev);
	int ha_count;
	int r;

	seip->stamp_cfg.rx_filter = HWTSTAMP_FILTER_NONE;
	seip->stamp_cfg.tx_type   = HWTSTAMP_TX_OFF;
	seip->multicast_all = 0;
	Stats.multicast_all = seip->multicast_all;

	for(ha_count = 0; ha_count < SEI_EMAC_MAX_MULTICAST_ADDRS; ha_count++)
		*sei_emac_hw_mcast_list[ha_count] = 0;

	r = sei_emac_refill_rx_ring(dev);
	if(r < 0) return r;

	sei_emac_restart(dev);
	seip->opened = 1;

	netif_start_queue(dev);
	napi_enable(&seip->napi);
	netif_carrier_on(dev);

	return 0;
}

static int sei_emac_close(struct net_device *dev)
{
	struct sei_emac_private *seip = netdev_priv(dev);
	int ha_count;
	int i;

	seip->stamp_cfg.rx_filter = HWTSTAMP_FILTER_NONE;
	seip->stamp_cfg.tx_type   = HWTSTAMP_TX_OFF;
	seip->multicast_all = 0;
	Stats.multicast_all = seip->multicast_all;

	seip->opened = 0;
	napi_disable(&seip->napi);
	netif_stop_queue(dev);
	netif_carrier_off(dev);

	for(ha_count = 0; ha_count < SEI_EMAC_MAX_MULTICAST_ADDRS; ha_count++)
		*sei_emac_hw_mcast_list[ha_count] = 0;
	sei_emac_stop(dev);

	for(i = 0; i < MAX_RX_RING_SZ; i++) {
		if(seip->rx_ring[i] == NULL) continue;

		kfree_skb(seip->rx_ring[i]);
		seip->rx_ring[i] = NULL;
	}

	return 0;
}

static int sei_emac_hwtstamp_ioctl(struct net_device* dev, struct ifreq* ifr, int cmd)
{
	struct sei_emac_private* seip = netdev_priv(dev);

        struct hwtstamp_config config;
        if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
                return -EFAULT;

        pr_debug("%s config flag:0x%x, tx_type:0x%x, rx_filter:0x%x\n",
                        __func__, config.flags, config.tx_type, config.rx_filter);

        /* reserved for future extensions */
        if (config.flags)
                return -EINVAL;

        if ((config.tx_type != HWTSTAMP_TX_OFF) &&
                        (config.tx_type != HWTSTAMP_TX_ON))
                return -ERANGE;

        switch (config.rx_filter) {
          case HWTSTAMP_FILTER_NONE: // Dont allow any timestamping
                //config.rx_filter = HWTSTAMP_FILTER_NONE;
                break;
          case HWTSTAMP_FILTER_ALL: // Dont allow any timestamping
                //config.rx_filter = HWTSTAMP_FILTER_ALL;
                break;
          case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:     // PTP v1, UDP, any kind of event packet
          case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:      // PTP v1, UDP, Sync packet
          case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ: // PTP v1, UDP, Delay_req packet
                config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
                break;
          case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:     // PTP v2, UDP, any kind of event packet
          case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:      // PTP v2, UDP, Sync packet
          case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ: // PTP v2, UDP, Delay_req packet
                config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
                break;
          case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:     // 802.AS1, Ethernet, any kind of event packet
          case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:      // 802.AS1, Ethernet, Sync packet
          case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ: // 802.AS1, Ethernet, Delay_req packet
                config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L2_EVENT;
                break;
	  case HWTSTAMP_FILTER_PTP_V2_EVENT:        // PTP v2/802.AS1, any layer, any kind of event packet
	  case HWTSTAMP_FILTER_PTP_V2_SYNC:         // PTP v2/802.AS1, any layer, Sync packet
	  case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:    // PTP v2/802.AS1, any layer, Delay_req packet
                config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
                break;
          default:
		printk("%s config.rx_filter = %d invalid\n", __func__, config.rx_filter);
                return -ERANGE;
        }

        if (config.tx_type == HWTSTAMP_TX_OFF && sei_emac_hwtstamp_is_none(config.rx_filter)) {
		// disable in HW
        } else {
		// enable in HW

                timecounter_init(&seip->clock,
                                &seip->cycles,
                                ktime_to_ns(ktime_get_real()));
        }

        seip->stamp_cfg = config;

	Dprintk("%s: tx_type %d rx_filter %d\n", __func__, seip->stamp_cfg.tx_type, seip->stamp_cfg.rx_filter);

        return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
                -EFAULT : 0;
}

static int sei_emac_ioctl(struct net_device* dev, struct ifreq* ifr, int cmd)
{
	///struct sei_emac_private* seip = netdev_priv(dev);

	if(!netif_running(dev)) return -EINVAL;

	switch(cmd) {
	  case SIOCSHWTSTAMP:
		return sei_emac_hwtstamp_ioctl(dev, ifr, cmd);
		break;
	  default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int sei_emac_proc_show(struct seq_file* m, void* v)
{
	int i;
	int ha_count;

	seq_printf(m, "MCAST ALL=%d\n", Stats.multicast_all);

        for(ha_count = 0; ha_count < SEI_EMAC_MAX_MULTICAST_ADDRS; ha_count++) {
		const u32 addr32 = *sei_emac_hw_mcast_list[ha_count];
		seq_printf(m, "MCAST[%d]=%08x\n", ha_count, addr32);
	}

	if(Stats.seip != NULL) {
		seq_printf(m, "HW Accel TX type: 0x%x RX filter: 0x%x\n",
			   Stats.seip->stamp_cfg.tx_type,
			   Stats.seip->stamp_cfg.rx_filter);
	}
        seq_printf(m, "TX pkt: %u pkt+ts2hw: %u mark: %u invaloff: %u invalts: %u irq:%u\n" \
		      "RX pkt: %u irq:%u err:%u\n", 
		   Stats.tx_pkt, Stats.tx_pkt_ts, Stats.tx_pkt_ts_mark, Stats.tx_pkt_inval_off, Stats.tx_pkt_inval_ts, Stats.tx_irq, 
		   Stats.rx_pkt, Stats.rx_irq, Stats.rx_err
		   );
        seq_printf(m, "RX pingpong ( ");
	for(i = 0; i < SEI_EMAC_PINGPONG_BUFFERS; i++)
        	seq_printf(m, "%u ", Stats.rx_pingpong_stats[i]);
	seq_printf(m, ")\n");

        seq_printf(m, "RX refill: %u SZ=( ", Stats.rx_refill);
	for(i = 0; i < MAX_RX_RING_SZ; i++)
        	seq_printf(m, "%u ", Stats.rx_refill_sz[i]);
	seq_printf(m, ")\n");
	for(i = 0; i < PTP_MSG_TYPE_SZ; i++) {
		const int msgType = ptp_msg_types[i];
		const u32 cnt = Stats.ptp_rx_stats[msgType];
		if(cnt == 0) continue;
        	seq_printf(m, "RX/PTP %s: %u\n", type_ts(msgType), cnt);
	}

        return 0;
}

static int sei_emac_proc_open(struct inode *inode, struct file *file)
{
        return single_open(file, sei_emac_proc_show, NULL);
}

static const struct proc_ops emac_proc_fops = {
        .proc_open           = sei_emac_proc_open,
        .proc_read           = seq_read,
        .proc_lseek         = seq_lseek,
        .proc_release        = single_release,
};

static const struct net_device_ops sei_emac_netdev_ops = {
	.ndo_open            = sei_emac_open,
	.ndo_stop            = sei_emac_close,
	.ndo_start_xmit      = sei_emac_start_xmit,
	.ndo_tx_timeout      = sei_emac_timeout,
        .ndo_set_rx_mode     = sei_emac_set_multicast_list,
	.ndo_set_mac_address = sei_emac_set_mac_address,
	.ndo_do_ioctl        = sei_emac_ioctl,
};

static int sei_emac_hw_init(struct net_device *dev, int index)
{
	dev->watchdog_timeo = TX_TIMEOUT;
	dev->netdev_ops = &sei_emac_netdev_ops;
	sei_emac_set_hw_addr(dev);
	//sei_emac_restart(dev);

	return 0;
}

/**
 * emac_read_clock - read raw cycle counter (to be used by time counter)
 */
static u64 sei_emac_read_clock(const struct cyclecounter *tc)
{
	// struct sei_emac_private* seip = container_of(tc, struct sei_emac_private, cycles);

	u64 sec  = 0;
	u32 nsec = 0;
	u64 stamp;
	read_ts(&sec, &nsec);

	///printk("CLK %llu.%us\n", sec, (unsigned int)nsec);

	stamp = sec*1000000000LL + nsec;

        return stamp;
}

/**
 * emac_init_hw_timer - Initialize hardware timer used with IEEE 1588 timestamp
 */
static void sei_emac_init_hw_timer(struct sei_emac_private* seip)
{
	if(seip == NULL) return;

	memset(&seip->cycles, 0, sizeof(seip->cycles));
	seip->cycles.read = sei_emac_read_clock;
	seip->cycles.mask = CLOCKSOURCE_MASK(64);

	seip->cycles.mult  = 1; // XXX
	seip->cycles.shift = 0; //XXX

	timecounter_init(&seip->clock,
			 &seip->cycles,
			 ktime_to_ns(ktime_get_real()));

        /* Initialize hwstamp config */
        seip->stamp_cfg.rx_filter = HWTSTAMP_FILTER_NONE;
        seip->stamp_cfg.tx_type   = HWTSTAMP_TX_OFF;
}

static int sei_emac_probe(struct platform_device *pdev)
{
	struct sei_emac_private *seip;
	struct net_device *dev;
	int irq, ret = 0;
	struct resource *r;

	memset(&Stats, 0, sizeof(Stats));

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return -ENXIO;

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (!r)
		return -EBUSY;

	/* Init network device */
	dev = alloc_etherdev(sizeof(struct sei_emac_private));
	if (!dev)
		return -ENOMEM;

	SET_NETDEV_DEV(dev, &pdev->dev);

	/* setup board info structure */
	seip = netdev_priv(dev);
	memset(seip, 0, sizeof(*seip));

	seip->sig = 0xdeadbeefL;
	seip->dev = dev;

	INIT_WORK(&seip->refill_work, sei_emac_refill_work);
	spin_lock_init(&seip->rx_lock);
	spin_lock_init(&seip->tx_lock);

	netif_napi_add(dev, &seip->napi, sei_emac_poll);

	seip->tx = 0;

	dev->base_addr = (unsigned long)ioremap(r->start, resource_size(r));

	if (!dev->base_addr) {
		ret = -ENOMEM;
		goto failed_ioremap;
	}

	platform_set_drvdata(pdev, dev);

	sei_emac_init_hw_timer(seip);

	irq = platform_get_irq(pdev, 0);

	ret =
	    request_irq(irq, sei_emac_interrupt, IRQF_NO_THREAD, pdev->name,
			dev);
	if (!ret) {
		ret = sei_emac_hw_init(dev, 0);
		if (!ret) {
			ret = register_netdev(dev);
			if (!ret) {
				Stats.seip = seip;
				proc_create("sei_emac", 0, NULL, &emac_proc_fops);
				return 0;
			}
		}
	}

	free_irq(irq, dev);
	iounmap((void __iomem *)dev->base_addr);

failed_ioremap:
	cancel_work_sync(&seip->refill_work);
	free_netdev(dev);

	return ret;
}

static int sei_emac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	sei_emac_stop(ndev);
	iounmap((void __iomem *)ndev->base_addr);
	unregister_netdev(ndev);
	free_netdev(ndev);

	return 0;
}

static int sei_emac_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct net_device *net_dev = platform_get_drvdata(pdev);

	if (net_dev && netif_running(net_dev))
		sei_emac_close(net_dev);

	return 0;
}

static int sei_emac_resume(struct platform_device *pdev)
{
	struct net_device *net_dev = platform_get_drvdata(pdev);

	if (net_dev && netif_running(net_dev))
		sei_emac_open(net_dev);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id jcore_emac_of_match[] = {
	{ .compatible = "jcore,emac" },
	{},
};
MODULE_DEVICE_TABLE(of, jcore_emac_of_match);

static struct platform_driver sei_emac_driver = {
	.driver = {
		.name = "jcore-emac",
		.owner = THIS_MODULE,
		.of_match_table = jcore_emac_of_match,
	},
	.probe = sei_emac_probe,
	.remove = sei_emac_remove,
	.suspend = sei_emac_suspend,
	.resume = sei_emac_resume,
};

static int __init sei_emac_init(void)
{
	uint32_t addr_lo = readl(SEI_EMAC_MACL);
	uint32_t addr_hi = readl(SEI_EMAC_MACH);

	/* Just in case if loader did not setup MAC address */
	if (!addr_hi) {
		writel(0x00000001, SEI_EMAC_MACH);
		addr_hi = readl(SEI_EMAC_MACH);
	}

	if (!addr_lo) {
		writel(0x02caca00, SEI_EMAC_MACL);
		addr_lo = readl(SEI_EMAC_MACL);
	}

	printk(KERN_INFO "SEI EMAC: MAC is %04x%08x\n", addr_hi, addr_lo);

	return platform_driver_register(&sei_emac_driver);
}

static void __exit sei_emac_exit(void)
{
	platform_driver_unregister(&sei_emac_driver);
}

module_init(sei_emac_init);
module_exit(sei_emac_exit);

MODULE_AUTHOR("Oleksandr G Zhadan");
MODULE_DESCRIPTION("SEI Emac driver");
