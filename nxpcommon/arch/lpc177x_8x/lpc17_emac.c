/**********************************************************************
* $Id$		lpc17_emac.c			2011-11-20
*//**
* @file		lpc17_emac.c
* @brief	LPC17 ethernet driver for LWIP
* @version	1.0
* @date		20. Nov. 2011
* @author	NXP MCU SW Application Team
* 
* Copyright(C) 2011, NXP Semiconductor
* All rights reserved.
*
***********************************************************************
* Software that is described herein is for illustrative purposes only
* which provides customers with programming information regarding the
* products. This software is supplied "AS IS" without any warranties.
* NXP Semiconductors assumes no responsibility or liability for the
* use of the software, conveys no license or title under any patent,
* copyright, or mask work right to the product. NXP Semiconductors
* reserves the right to make changes in the software without
* notification. NXP Semiconductors also make no representation or
* warranty that such application will be suitable for the specified
* use without further testing or modification.
**********************************************************************/

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "netif/etharp.h"
#include "netif/ppp_oe.h"

#include "lpc177x_8x_emac.h"
#include "lpc177x_8x_clkpwr.h"
#include "lpc17_emac.h"
#include "lpc_emac_config.h"

/** @defgroup lwip_emac_DRIVER	lpc17 EMAC driver for LWIP
 * @ingroup lwip_emac
 *
 * This driver is currently for the LPC177x_8x devices only, although
 * the LPC32x0 and LPC2000 series devices share the same ethernet
 * controller.
 *
 * @{
 */

/** \brief  Write a value via the MII link (non-blocking)

    This function will write a value on the MII link interface to a PHY
	or a connected device. The function will return immediately without
	a status. Status needs to be polled later to determine if the write
	was successful.

    \param [in]      PhyReg  PHY register to write to
    \param [in]      Value   Value to write
 */
void lpc_mii_write_noblock(u32_t PhyReg, u32_t Value)
{
	/* Write value at PHY address and register */
	LPC_EMAC->MADR = (LPC_PHYDEF_PHYADDR << 8) | PhyReg;
	LPC_EMAC->MWTD = Value;
}

/** \brief  Write a value via the MII link (blocking)

    This function will write a value on the MII link interface to a PHY
	or a connected device. The function will block until complete.

    \param [in]      PhyReg  PHY register to write to
    \param [in]      Value   Value to write
	\returns         0 if the write was successful, otherwise !0
 */
err_t lpc_mii_write(u32_t PhyReg, u32_t Value)
{
	u32_t mst = 250;
	err_t sts = ERR_OK;

	/* Write value at PHY address and register */
	lpc_mii_write_noblock(PhyReg, Value);

	/* Wait for unbusy status */
	while (mst > 0) {
		sts = LPC_EMAC->MIND;
		if ((sts & EMAC_MIND_BUSY) == 0)
			mst = 0;
		else {
			mst--;
			msDelay(1);
		}
	}

	if (sts != 0)
		sts = ERR_TIMEOUT;

	return sts;
}

/** \brief  Reads current MII link busy status

    This function will return the current MII link busy status and is meant to
	be used with non-blocking functions for monitor PHY status such as
	connection state.

	\returns         !0 if the MII link is busy, otherwise 0
 */
u32_t lpc_mii_is_busy(void)
{
	return (u32_t) (LPC_EMAC->MIND & EMAC_MIND_BUSY);
}

/** \brief  Starts a read operation via the MII link (non-blocking)

    This function returns the current value in the MII data register. It is
	meant to be used with the non-blocking oeprations. This value should
	only be read after a non-block read command has been issued and the
	MII status has been determined to be good.

    \returns          The current value in the MII value register
 */
u32_t lpc_mii_read_data(void)
{
	u32_t data = LPC_EMAC->MRDD;
	LPC_EMAC->MCMD = 0;

	return data;
}

/** \brief  Starts a read operation via the MII link (non-blocking)

    This function will start a read operation on the MII link interface
	from a PHY or a connected device. The function will not block and
	the status mist be polled until complete. Once complete, the data
	can be read.

    \param [in]      PhyReg  PHY register to read from
 */
void lpc_mii_read_noblock(u32_t PhyReg) 
{
	/* Read value at PHY address and register */
	LPC_EMAC->MADR = (LPC_PHYDEF_PHYADDR << 8) | PhyReg;
	LPC_EMAC->MCMD = EMAC_MCMD_READ;
}

/** \brief  Read a value via the MII link (blocking)

    This function will read a value on the MII link interface from a PHY
	or a connected device. The function will block until complete.

    \param [in]      PhyReg  PHY register to read from
    \param [in]      data    Pointer to where to save data read via MII
	\returns         0 if the read was successful, otherwise !0
 */
err_t lpc_mii_read(u32_t PhyReg, u32_t *data) 
{
	u32_t mst = 250;
	err_t sts = ERR_OK;

	/* Read value at PHY address and register */
	lpc_mii_read_noblock(PhyReg);

	/* Wait for unbusy status */
	while (mst > 0) {
		sts = LPC_EMAC->MIND & ~EMAC_MIND_MII_LINK_FAIL;
		if ((sts & EMAC_MIND_BUSY) == 0) {
			mst = 0;
			*data = LPC_EMAC->MRDD;
		} else {
			mst--;
			msDelay(1);
		}
	}

	LPC_EMAC->MCMD = 0;

	if (sts != 0)
		sts = ERR_TIMEOUT;

	return sts;
}

#if LPC_PBUF_RX_ZEROCOPY
/** \brief  Queues a pbuf into the RX descriptor list

    \param lpc_enetdata Pointer to the drvier data structure
    \param p            Pointer to pbuf to queue
 */
static void lpc_rxqueue_pbuf(struct lpc_enetdata *lpc_enetdata, struct pbuf *p)
{
	u32_t idx;

	/* Get next free descriptor index */
	idx = lpc_enetdata->rx_fill_desc_index;

	/* Setup descriptor and clear statuses */
	lpc_enetdata->prxd[idx].control = EMAC_RCTRL_INT | ((u32_t) (p->len - 1));
	lpc_enetdata->prxd[idx].packet = (u32_t) p->payload;
	lpc_enetdata->prxs[idx].statusinfo = 0xFFFFFFFF;
	lpc_enetdata->prxs[idx].statushashcrc = 0xFFFFFFFF;

	/* Save pbuf pointer for push to network layer later */
	lpc_enetdata->rxb[idx] = p;

	/* Wrap at end of descriptor list */
	idx++;
	if (idx >= LPC_NUM_BUFF_RXDESCS)
		idx = 0;

	/* Queue descriptor(s) */
	lpc_enetdata->rx_free_descs -= 1;
	lpc_enetdata->rx_fill_desc_index = idx;
	LPC_EMAC->RxConsumeIndex = idx;

	LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
		("lpc_rxqueue_pbuf: pbuf packet queued: %p (free desc=%d)\n", p,
			lpc_enetdata->rx_free_descs));
}

/** \brief  Attempt to allocate and requeue a new pbuf for RX

    \param     netif Pointer to the netif structure
    \returns         1 if a packet was allocated and requeued, otherwise 0
 */
s32_t lpc_rx_queue(struct netif *netif)
{
	struct pbuf *p;
	struct lpc_enetdata *lpc_enetdata = netif->state;

	/* Exit of there are no descriptor available */
	if (lpc_enetdata->rx_free_descs == 0)
		return 0;

	/* Allocate a pbuf from the pool. We need to allocate at the maximum size
	   as we don't know the size of the yet to be received packet. */
	p = pbuf_alloc(PBUF_RAW, (u16_t) EMAC_ETH_MAX_FLEN, PBUF_RAM);
	if (p == NULL) {
		LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
			("lpc_rx_queue: could not allocate RX pbuf (free desc=%d)\n",
			lpc_enetdata->rx_free_descs));
		return 0;
	}

	/* pbufs allocated from the RAM pool should be non-chained. */
	LWIP_ASSERT("lpc_rx_queue: pbuf is not contiguous (chained)",
		pbuf_clen(p) <= 1);

	/* Queue packet */
	lpc_rxqueue_pbuf(lpc_enetdata, p);

	return 1;
}

/** \brief  Sets up the RX descriptor ring buffers.

    This function sets up the descriptor list used for receive packets.

    \param [in]  lpc_enetdata  Pointer to driver data structure
	\returns                   Always returns ERR_OK
 */
static err_t lpc_rx_setup(struct lpc_enetdata *lpc_enetdata)
{
	s32_t idx;

	/* Setup pointers to RX structures */
	LPC_EMAC->RxDescriptor = (u32_t) &lpc_enetdata->prxd[0];
	LPC_EMAC->RxStatus = (u32_t) &lpc_enetdata->prxs[0];
	LPC_EMAC->RxDescriptorNumber = LPC_NUM_BUFF_RXDESCS - 1;

	lpc_enetdata->rx_free_descs = LPC_NUM_BUFF_RXDESCS;
	lpc_enetdata->rx_fill_desc_index = 0;

	/* Build RX buffer and descriptors */
	for (idx = 0; idx < LPC_NUM_BUFF_RXDESCS; idx++)
		lpc_rx_queue(lpc_enetdata->netif);

	return ERR_OK;
}

/** \brief  Allocates a pbuf and returns the data from the incoming packet.

    \param netif the lwip network interface structure for this lpc_enetif
    \return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *lpc_low_level_input(struct netif *netif)
{
	struct lpc_enetdata *lpc_enetdata = netif->state;
	struct pbuf *p = NULL, *q;
	u32_t idx, length;
	u8_t *src;

	/* Monitor RX overrun status. This should never happen unless
	   (possibly) the internal bus is behing held up by something.
	   Unless your system is running at a very low clock speed or
	   there are possibilities that the internal buses may be held
	   up for a long time, this can probably safely be removed. */
	if (LPC_EMAC->IntStatus & EMAC_INT_RX_OVERRUN) {
		LINK_STATS_INC(link.err);
		LINK_STATS_INC(link.drop);

		/* Temporarily disable RX */
		LPC_EMAC->MAC1 &= ~EMAC_MAC1_REC_EN;

		/* Reset the RX side */
		LPC_EMAC->MAC1 |= EMAC_MAC1_RES_RX;
		LPC_EMAC->IntClear = EMAC_INT_RX_OVERRUN;

		/* De-allocate all queued RX pbufs */
		for (idx = 0; idx < LPC_NUM_BUFF_RXDESCS; idx++) {
			if (lpc_enetdata->rxb[idx] != NULL) {
				pbuf_free(lpc_enetdata->rxb[idx]);
				lpc_enetdata->rxb[idx] = NULL;
			}
		}

		/* Start RX side again */
		lpc_rx_setup(lpc_enetdata);

		/* Re-enable RX */
		LPC_EMAC->MAC1 |= EMAC_MAC1_REC_EN;

		return NULL;
	}

	/* Determine if a frame has been received */
	length = 0;
	idx = LPC_EMAC->RxConsumeIndex;
	if (LPC_EMAC->RxProduceIndex != idx) {
		/* Handle errors */
		if (lpc_enetdata->prxs[idx].statusinfo & EMAC_RINFO_ERR) {
#if LINK_STATS
			if (lpc_enetdata->prxs[idx].statusinfo & (EMAC_RINFO_CRC_ERR |
				EMAC_RINFO_SYM_ERR | EMAC_RINFO_ALIGN_ERR))
				LINK_STATS_INC(link.chkerr);
			if (lpc_enetdata->prxs[idx].statusinfo & EMAC_RINFO_LEN_ERR)
				LINK_STATS_INC(link.lenerr);
#endif

			/* Drop the frame */
			LINK_STATS_INC(link.drop);

			/* Free the pbuf associated with this descriptor. The RX
			   queue function will queue a new pbuf later. */
			if (lpc_enetdata->rxb[idx] != NULL) {
				pbuf_free(lpc_enetdata->rxb[idx]);
				lpc_enetdata->rxb[idx] = NULL;
				lpc_enetdata->rx_free_descs++;
			}

			LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
				("lpc_low_level_input: Packet dropped with errors\n"));
		}
		else {
			/* A packet is waiting, get length */
			length = (lpc_enetdata->prxs[idx].statusinfo & 0x7FF) + 1;

			/* Zero-copy */
			p = lpc_enetdata->rxb[idx];
			p->len = (u16_t) length;

			/* Free pbuf from desriptor */
			lpc_enetdata->rxb[idx] = NULL;
			lpc_enetdata->rx_free_descs++;

			LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
				("lpc_low_level_input: Packet received: %p, size %d (index=%d)\n",
				p, length, idx));

			/* Save size */
			p->tot_len = (u16_t) length;
			LINK_STATS_INC(link.recv);
		}
	}

	return p;  
}

#else /* Copied RX obuf driver starts here */
/** \brief  Sets up the RX descriptor ring buffers.

    This function sets up the descriptor list used for receive packets.

    \param [in]  lpc_enetdata  Pointer to driver data structure
	\returns                   Always returns ERR_OK
 */
static err_t lpc_rx_setup(struct lpc_enetdata *lpc_enetdata)
{
	s32_t idx;

	/* Setup pointers to RX structures */
	LPC_EMAC->RxDescriptor = (u32_t) &lpc_enetdata->prxd[0];
	LPC_EMAC->RxStatus = (u32_t) &lpc_enetdata->prxs[0];
	LPC_EMAC->RxDescriptorNumber = LPC_NUM_BUFF_RXDESCS - 1;

	/* Build RX descriptors */
	for (idx = 0; idx < LPC_NUM_BUFF_RXDESCS; idx++) {
		/* Setup DMA RX descriptor */
		lpc_enetdata->prxd[idx].packet = (u32_t) &lpc_enetdata->lpc_rx_buffs[idx][0];
		lpc_enetdata->prxd[idx].control = EMAC_RCTRL_INT |
			(EMAC_ETH_MAX_FLEN - 1);
		lpc_enetdata->prxs[idx].statusinfo = 0xFFFFFFFF;
		lpc_enetdata->prxs[idx].statushashcrc = 0xFFFFFFFF;
	}

	return ERR_OK;
}

/** \brief  Allocates a pbuf and returns the data from the incoming packet.

    \param netif the lwip network interface structure for this lpc_enetif
    \return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *lpc_low_level_input(struct netif *netif)
{
	struct lpc_enetdata *lpc_enetdata = netif->state;
	struct pbuf *p = NULL, *q;
	u32_t idx, length;
	u8_t *src;

	/* Monitor RX overrun status. This should never happen unless
	   (possibly) the internal bus is behing held up by something.
	   Unless your system is running at a very low clock speed or
	   there are possibilities that the internal buses may be held
	   up for a long time, this can probably safely be removed. */
	if (LPC_EMAC->IntStatus & EMAC_INT_RX_OVERRUN) {
		LINK_STATS_INC(link.err);
		LINK_STATS_INC(link.drop);

		/* Temporarily disable RX */
		LPC_EMAC->MAC1 &= ~EMAC_MAC1_REC_EN;

		/* Reset the RX side */
		LPC_EMAC->MAC1 |= EMAC_MAC1_RES_RX;
		LPC_EMAC->IntClear = EMAC_INT_RX_OVERRUN;

		/* Start RX side again */
		lpc_rx_setup(lpc_enetdata);

		/* Re-enable RX */
		LPC_EMAC->MAC1 |= EMAC_MAC1_REC_EN;

		return NULL;
	}

	/* Determine if a frame has been received */
	idx = LPC_EMAC->RxConsumeIndex;
	if (LPC_EMAC->RxProduceIndex != idx) {
		/* Handle errors */
		if (lpc_enetdata->prxs[idx].statusinfo & (EMAC_RINFO_NO_DESCR |
			EMAC_RINFO_ALIGN_ERR | EMAC_RINFO_LEN_ERR | EMAC_RINFO_SYM_ERR |
			EMAC_RINFO_CRC_ERR)) {
#if LINK_STATS
			if (lpc_enetdata->prxs[idx].statusinfo & (EMAC_RINFO_CRC_ERR |
				EMAC_RINFO_SYM_ERR | EMAC_RINFO_ALIGN_ERR))
				LINK_STATS_INC(link.chkerr);
			if (lpc_enetdata->prxs[idx].statusinfo & EMAC_RINFO_LEN_ERR)
				LINK_STATS_INC(link.lenerr);
#endif

			/* Drop the frame */
			LINK_STATS_INC(link.drop);

			LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
				("lpc_low_level_input: Packet dropped with errors\n"));
		}
		else {
			/* A packet is waiting, get length */
			length = (lpc_enetdata->prxs[idx].statusinfo & 0x7FF) + 1;

			/* Allocate a pbuf chain of pbufs from the pool. */
			p = pbuf_alloc(PBUF_RAW, (u16_t) length, PBUF_POOL);
			if (p == NULL) {
				/* Buffer in hardware is not lost, but cannot get packet now */
				LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
					("lpc_low_level_input: Could not allocate input pbuf\n"));
				return NULL;
			}

			/* Copy buffer */
			src = (u8_t *) lpc_enetdata->prxd[idx].packet;
		    for(q = p; q != NULL; q = q->next) {
				MEMCPY((u8_t *) q->payload, src, q->len);
				src += q->len;
			}

			LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
				("lpc_low_level_input: Packet received: %p, size %d (index=%d)\n",
				p, length, idx));

			/* Save size */
			p->tot_len = (u16_t) length;
			LINK_STATS_INC(link.recv);
		}

			idx++;
			if (idx >= LPC_NUM_BUFF_RXDESCS)
				idx = 0;
			LPC_EMAC->RxConsumeIndex = idx;
	}

	return p;  
}
#endif /* End of RX copied or zero-copy driver section */

/** \brief  Attempt to read a packet from the EMAC interface.

    \param[in] netif the lwip network interface structure for this lpc_enetif
 */
void lpc_enetif_input(struct netif *netif)
{
	struct lpc_enetif *lpc_enetif = netif->state;
	struct eth_hdr *ethhdr;
	struct pbuf *p;

	/* move received packet into a new pbuf */
	p = lpc_low_level_input(netif);
	if (p == NULL)
		return;

	/* points to packet payload, which starts with an Ethernet header */
	ethhdr = p->payload;

	switch (htons(ethhdr->type)) {
		case ETHTYPE_IP:
		case ETHTYPE_ARP:
#if PPPOE_SUPPORT
		case ETHTYPE_PPPOEDISC:
		case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
			/* full packet send to tcpip_thread to process */
			if (netif->input(p, netif) != ERR_OK) {
				LWIP_DEBUGF(NETIF_DEBUG, ("lpc_enetif_input: IP input error\n"));
				/* Free buffer */
				pbuf_free(p);
			}
			break;

		default:
			/* Return buffer */
			pbuf_free(p);
			break;
	}
}

/** \brief  Determine if the passed address is usable for the ethernet
            DMA controller.

    \param [in] addr Address of packet to check for DMA safe operation
    \return          1 if the packet address is not safe, otherwise 0
 */
static s32_t lpc_packet_addr_notsafe(void *addr) {
	/* Check for legal address ranges */
	if ((((u32_t) addr >= 0x20000000) && ((u32_t) addr < 0x20008000)) ||
		(((u32_t) addr >= 0x80000000) && ((u32_t) addr < 0xF0000000)))
		return 0;

	return 1;
}

#if LPC_PBUF_TX_ZEROCOPY
/** \brief  Sets up the TX descriptor ring buffers.

    This function sets up the descriptor list used for transmit packets.

    \param [in]      lpc_enetdata  Pointer to driver data structure
 */
static err_t lpc_tx_setup(struct lpc_enetdata *lpc_enetdata)
{
	s32_t idx;

	/* Setup pointers to TX structures */
	LPC_EMAC->TxDescriptor = (u32_t) &lpc_enetdata->ptxd[0];
	LPC_EMAC->TxStatus = (u32_t) &lpc_enetdata->ptxs[0];
	LPC_EMAC->TxDescriptorNumber = LPC_NUM_BUFF_TXDESCS - 1;

	lpc_enetdata->lpc_last_tx_idx = 0;

	/* Build TX descriptors for local buffers */
	for (idx = 0; idx < LPC_NUM_BUFF_TXDESCS; idx++) {
		lpc_enetdata->ptxd[idx].control = 0;
		lpc_enetdata->ptxs[idx].statusinfo = 0xFFFFFFFF;
	}

	return ERR_OK;
}

/** \brief  Free TX buffers that are complete

    \param [in] lpc_enetdata  Pointer to driver data structure
    \param [in] cidx  EMAC current descriptor comsumer index
 */
static void lpc_tx_reclaim_st(struct lpc_enetdata *lpc_enetdata, u32_t cidx)
{
	while (cidx != lpc_enetdata->lpc_last_tx_idx) {
		if (lpc_enetdata->txb[lpc_enetdata->lpc_last_tx_idx] != NULL) {
			LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
				("lpc_tx_reclaim_st: Freeing packet %p (index %d)\n",
				lpc_enetdata->txb[lpc_enetdata->lpc_last_tx_idx],
				lpc_enetdata->lpc_last_tx_idx));
			pbuf_free(lpc_enetdata->txb[lpc_enetdata->lpc_last_tx_idx]);
		 	lpc_enetdata->txb[lpc_enetdata->lpc_last_tx_idx] = NULL;
		}

		lpc_enetdata->lpc_last_tx_idx++;
		if (lpc_enetdata->lpc_last_tx_idx >= LPC_NUM_BUFF_TXDESCS)
			lpc_enetdata->lpc_last_tx_idx = 0;
	}
}

/** \brief  User call for freeingTX buffers that are complete

    \param [in] netif the lwip network interface structure for this lpc_enetif
 */
void lpc_tx_reclaim(struct netif *netif)
{
	lpc_tx_reclaim_st((struct lpc_enetdata *) netif->state,
		LPC_EMAC->TxConsumeIndex);
}

 /** \brief  Polls if an available TX descriptor is ready. Can be used to
             determine if the low level transmit function will block.

    \param [in] netif the lwip network interface structure for this lpc_enetif
    \return 0 if no descriptors are read, or >0
 */
s32_t lpc_tx_ready(struct netif *netif)
{
	s32_t fb;
	u32_t idx, cidx;

	cidx = LPC_EMAC->TxConsumeIndex;
	lpc_tx_reclaim_st((struct lpc_enetdata *) netif->state, cidx);

	idx = LPC_EMAC->TxProduceIndex;

	/* Determine number of free buffers */
	if (idx == cidx)
		fb = LPC_NUM_BUFF_TXDESCS;
	else if (cidx > idx)
		fb = (LPC_NUM_BUFF_TXDESCS - 1) -
			((idx + LPC_NUM_BUFF_TXDESCS) - cidx);
	else
		fb = (LPC_NUM_BUFF_TXDESCS - 1) - (cidx - idx);

	return fb;
}

/** \brief  Low level output of a packet. Never call this from an
            interrupt context, as it may block until TX descriptors
			become available.

    \param netif the lwip network interface structure for this lpc_enetif
    \param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
    \return ERR_OK if the packet could be sent
           an err_t value if the packet couldn't be sent
 */
static err_t lpc_low_level_output(struct netif *netif, struct pbuf *p)
{
	struct lpc_enetdata *lpc_enetdata = netif->state;
	struct pbuf *q;
	u8_t *dst;
	u32_t idx, sz = 0;
	err_t err = ERR_OK;
	struct pbuf *np;
	u32_t dn, notdmasafe = 0;

	/* Error handling for TX underruns. This should never happen unless
	   something is holding the bus or the clocks are going too slow. It
	   can probably be safely removed. */
 	if (LPC_EMAC->IntStatus & EMAC_INT_TX_UNDERRUN) {
		LINK_STATS_INC(link.err);
		LINK_STATS_INC(link.drop);

		/* Reset the TX side */
		LPC_EMAC->MAC1 |= EMAC_MAC1_RES_TX;
		LPC_EMAC->IntClear = EMAC_INT_TX_UNDERRUN;

		/* De-allocate all queued TX pbufs */
		for (idx = 0; idx < LPC_NUM_BUFF_RXDESCS; idx++) {
			if (lpc_enetdata->txb[idx] != NULL) {
				pbuf_free(lpc_enetdata->txb[idx]);
				lpc_enetdata->txb[idx] = NULL;
			}
		}

		/* Start TX side again */
		lpc_tx_setup(lpc_enetdata);
	}

	/* Zero-copy TX buffers may be fragmented across mutliple payload
	   chains. Determine the number of descriptors needed for the
	   transfer. The pbuf chaining can be a mess! */
	dn = (u32_t) pbuf_clen(p);

	/* Test to make sure packet addresses are DMA safe. A DMA safe
	   address is once that uses external memory or periphheral RAM.
	   IRAM and FLASH are not safe! */
	for (q = p; q != NULL; q = q->next)
		notdmasafe += lpc_packet_addr_notsafe(q->payload);

#if LPC_TX_PBUF_BOUNCE_EN==1
	/* If the pbuf is not DMA safe, a new bounce buffer (pbuf) will be
	   created that will be used instead. This requires an copy from the
	   non-safe DMA region to the new pbuf */
	if (notdmasafe) {
		/* Allocate a pbuf in DMA memory */
		np = pbuf_alloc(PBUF_RAW, p->tot_len, PBUF_RAM);
		if (np == NULL)
			return ERR_MEM;	

		/* This buffer better be contiguous! */
		LWIP_ASSERT("lpc_low_level_output: New transmit pbuf is chained",
			(pbuf_clen(np) == 1));

		/* Copy to DMA safe pbuf */
		dst = (u8_t *) np->payload;
	 	for(q = p; q != NULL; q = q->next) {
			/* Copy the buffer to the descriptor's buffer */
	  		MEMCPY(dst, (u8_t *) q->payload, q->len);
		  dst += q->len;
		}
		np->len = p->tot_len; 

		LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
			("lpc_low_level_output: Switched to DMA safe buffer, old=%p, new=%p\n",
			q, np));

		/* use the new buffer for descrptor queueing. The original pbuf will
		   be de-allocated outsuide this driver. */
		p = np;
		dn = 1;
	}
#else
	if (notdmasafe)
		LWIP_ASSERT("lpc_low_level_output: Not a DMA safe pbuf",
			(notdmasafe == 0));
#endif

	/* Wait until enough descriptors are available for the transfer. */
	/* THIS WILL BLOCK UNTIL THERE ARE ENOUGH DESCRIPTORS AVAILABLE */
	while (dn > lpc_tx_ready(netif));

	/* Get free TX buffer index */
	idx = LPC_EMAC->TxProduceIndex;

	/* Setup transfers */
	q = p;
	while (dn > 0) {
		dn--;

		lpc_enetdata->txb[idx] = q;

		/* Only save pointer to free on last descriptor */
		if (dn == 0) {
			/* Save size of packet and signal it's ready */
			lpc_enetdata->ptxd[idx].control = (q->len - 1) | EMAC_TCTRL_INT |
				EMAC_TCTRL_LAST;
		}
		else {
			/* Save size of packet, descriptor is not last */
			lpc_enetdata->ptxd[idx].control = (q->len - 1) | EMAC_TCTRL_INT;
		}

		LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
			("lpc_low_level_output: pbuf packet(%p) sent, chain#=%d,"
			" size = %d (index=%d)\n", q->payload, dn, q->len, idx));

		lpc_enetdata->ptxd[idx].packet = (u32_t) q->payload;

		/* Increment packet reference to prevent de-allocation only if buffers
		   are zero-copy - the TX cleanup task will de-allocate it once the
		   hardware is done with it. */
		if (!notdmasafe)
			pbuf_ref(q);
		q = q->next;

		idx++;
		if (idx >= LPC_NUM_BUFF_TXDESCS)
			idx = 0;
	}

	LPC_EMAC->TxProduceIndex = idx;

	LINK_STATS_INC(link.xmit);

	return ERR_OK;
}

#else /* Start of copied pbuf transmit driver */
/** \brief  Sets up the TX descriptor ring buffers.

    This function sets up the descriptor list used for transmit packets.

    \param [in]      lpc_enetdata  Pointer to driver data structure
 */
static err_t lpc_tx_setup(struct lpc_enetdata *lpc_enetdata)
{
	s32_t idx;

	/* Setup pointers to TX structures */
	LPC_EMAC->TxDescriptor = (u32_t) &lpc_enetdata->ptxd[0];
	LPC_EMAC->TxStatus = (u32_t) &lpc_enetdata->ptxs[0];
	LPC_EMAC->TxDescriptorNumber = LPC_NUM_BUFF_TXDESCS - 1;

	/* Build TX descriptors for local buffers */
	for (idx = 0; idx < LPC_NUM_BUFF_TXDESCS; idx++) {
		lpc_enetdata->ptxd[idx].packet = (u32_t) &lpc_enetdata->lpc_tx_buffs[idx][0];
		lpc_enetdata->ptxd[idx].control = 0;
		lpc_enetdata->ptxs[idx].statusinfo = 0xFFFFFFFF;
	}

	return ERR_OK;
}

 /** \brief  Polls if an available TX descriptor is ready. Can be used to
             determine if the low level transmit function will block.

    \param netif the lwip network interface structure for this lpc_enetif
    \return 0 if no descriptors are read, or >0
 */
s32_t lpc_tx_ready(struct netif *netif)
{
	s32_t fb;
	u32_t idx, cidx;

	cidx = LPC_EMAC->TxConsumeIndex;
	idx = LPC_EMAC->TxProduceIndex;

	/* Determine number of free buffers */
	if (idx == cidx)
		fb = LPC_NUM_BUFF_TXDESCS;
	else if (cidx > idx)
		fb = (LPC_NUM_BUFF_TXDESCS - 1) -
			((idx + LPC_NUM_BUFF_TXDESCS) - cidx);
	else
		fb = (LPC_NUM_BUFF_TXDESCS - 1) - (cidx - idx);

	return fb;
}

/** \brief  Low level output of a packet. Never call this from an
            interrupt context, as it may block until TX descriptors
			become available.

    \param netif the lwip network interface structure for this lpc_enetif
    \param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
    \return ERR_OK if the packet could be sent
           an err_t value if the packet couldn't be sent
 */
static err_t lpc_low_level_output(struct netif *netif, struct pbuf *p)
{
	struct lpc_enetdata *lpc_enetdata = netif->state;
	struct pbuf *q;
	u8_t *dst;
	u32_t idx, sz = 0;
	err_t err = ERR_OK;

	/* Error handling for TX underruns. This should never happen unless
	   something is holding the bus or the clocks are going too slow. It
	   can probably be safely removed. */
 	if (LPC_EMAC->IntStatus & EMAC_INT_TX_UNDERRUN) {
		LINK_STATS_INC(link.err);
		LINK_STATS_INC(link.drop);

		/* Reset the TX side */
		LPC_EMAC->MAC1 |= EMAC_MAC1_RES_TX;
		LPC_EMAC->IntClear = EMAC_INT_TX_UNDERRUN;

		/* Start TX side again */
		lpc_tx_setup(lpc_enetdata);
	}

	/* Wait for a single buffer to become available */
	while (lpc_tx_ready(netif) == 0);

	/* Get free TX buffer index */
	idx = LPC_EMAC->TxProduceIndex;

	/* Copy pbuf to ethernet send buffer */
	dst = (u8_t *) lpc_enetdata->ptxd[idx].packet;
 	for(q = p; q != NULL; q = q->next) {
		/* Copy the buffer to the descriptor's buffer */
	  MEMCPY(dst, (u8_t *) q->payload, q->len);
	  dst += q->len;
	  sz += q->len;
	}

	/* Save size of packet and signal it's ready */
	lpc_enetdata->ptxd[idx].control = (sz - 1) | EMAC_TCTRL_INT |
		EMAC_TCTRL_LAST;

	/*SGet next index for transmit descriptor */
	idx++;
	if (idx >= LPC_NUM_BUFF_TXDESCS)
		idx = 0;
	LPC_EMAC->TxProduceIndex = idx;

	LWIP_DEBUGF(UDP_LPC_EMAC | LWIP_DBG_TRACE,
		("lpc_low_level_output: pbuf packet(%p) sent, chains = %d,"
		" size = %d\n", p, (u32_t) pbuf_clen(p), p->tot_len));

	LINK_STATS_INC(link.xmit);

	return ERR_OK;
}
#endif  /* End of TX copied or zero-copy driver section */

/** \brief  LPC EMAC interrupt handler.

    This function handles the transmit, receive, and error interrupt of
	the LPC177x_8x. This is meant to be used when NO_SYS=0. It is not
	implemented here and is just a spotholder for RTOS support.
 */
void ENET_IRQHandler(void)
{
	;
}

/** \brief  Low level init of the MAC and PHY.

    \param [in]      netif  Pointer to LWIP netif structure
 */
static err_t low_level_init(struct netif *netif)
{
	struct lpc_enetdata *lpc_enetdata = netif->state;
	err_t err = ERR_OK;

	/* Enable MII clocking */
	CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCENET, ENABLE);

	/* Reset all MAC logic */
	LPC_EMAC->MAC1 = EMAC_MAC1_RES_TX | EMAC_MAC1_RES_MCS_TX |
		EMAC_MAC1_RES_RX | EMAC_MAC1_RES_MCS_RX |
		EMAC_MAC1_SIM_RES | EMAC_MAC1_SOFT_RES;
	LPC_EMAC->Command = EMAC_CR_REG_RES | EMAC_CR_TX_RES | EMAC_CR_RX_RES;
	msDelay(10);

	/* Initial MAC initialization */
	LPC_EMAC->MAC1 = EMAC_MAC1_PASS_ALL;
	LPC_EMAC->MAC2 = EMAC_MAC2_CRC_EN | EMAC_MAC2_PAD_EN |
		EMAC_MAC2_VLAN_PAD_EN;
	LPC_EMAC->MAXF = EMAC_ETH_MAX_FLEN;

	/* Set RMII management clock rate to lowest speed */
	LPC_EMAC->MCFG = EMAC_MCFG_CLK_SEL(7) | EMAC_MCFG_RES_MII;
	LPC_EMAC->MCFG &= ~EMAC_MCFG_RES_MII;

	/* Maximum number of retries, 0x37 collision window, gap */
	LPC_EMAC->CLRT = EMAC_CLRT_DEF;
	LPC_EMAC->IPGR = EMAC_IPGR_P1_DEF | EMAC_IPGR_P2_DEF;

#if LPC_EMAC_RMII
	/* RMII setup */
	LPC_EMAC->Command = EMAC_CR_PASS_RUNT_FRM | EMAC_CR_RMII;
#else
	/* MII setup */
	LPC_EMAC->CR = EMAC_CR_PASS_RUNT_FRM;
#endif

	/* Initialize the PHY and reset */
	err = lpc_phy_init(netif);
	if (err != ERR_OK)
 		return err;

	/* Save station address */
	LPC_EMAC->SA2 = (u32_t) netif->hwaddr[0] |
		(((u32_t) netif->hwaddr[1]) << 8);
	LPC_EMAC->SA1 = (u32_t) netif->hwaddr[2] |
		(((u32_t) netif->hwaddr[3]) << 8);
	LPC_EMAC->SA0 = (u32_t) netif->hwaddr[4] |
		(((u32_t) netif->hwaddr[5]) << 8);

	/* Setup transmit and receive descriptors */
	if (lpc_tx_setup(lpc_enetdata) != ERR_OK)
		return ERR_BUF;
	if (lpc_rx_setup(lpc_enetdata) != ERR_OK)
		return ERR_BUF;

	/* Enable packet reception */
#if IP_SOF_BROADCAST_RECV
	LPC_EMAC->RxFilterCtrl = EMAC_RFC_PERFECT_EN | EMAC_RFC_BCAST_EN;
#else
	LPC_EMAC->RxFilterCtrl = EMAC_RFC_PERFECT_EN;
#endif

	/* Clear and enable rx/tx interrupts */
	LPC_EMAC->IntClear = 0xFFFF;
	LPC_EMAC->IntEnable =
#if NO_SYS
		0;
#else
		EMAC_INT_TX_DONE | EMAC_INT_RX_DONE |
		EMAC_INT_RX_OVERRUN | EMAC_INT_TX_UNDERRUN;
#endif

	/* Not all interrupts are used, only descriptor completion
	   and some error interrupts are needed. RX status errors
	   are handled at packet reception time. */
	LPC_EMAC->Command |= EMAC_CR_RX_EN | EMAC_CR_TX_EN;
	LPC_EMAC->MAC1 |= EMAC_MAC1_REC_EN;

	return err;
}

/**
 * This function provides a method for the PHY to setup the EMAC
 * for the PHY negotiated duplex mode.
 *
 * @param[in] full_duplex 0 = half duplex, 1 = full duplex
 */
void lpc_emac_set_duplex(int full_duplex)
{
	if (full_duplex) {
		LPC_EMAC->MAC2    |= EMAC_MAC2_FULL_DUP;
		LPC_EMAC->Command |= EMAC_CR_FULL_DUP;
		LPC_EMAC->IPGT     = EMAC_IPGT_FULL_DUP;
	} else {
		LPC_EMAC->MAC2    &= ~EMAC_MAC2_FULL_DUP;
		LPC_EMAC->Command &= ~EMAC_CR_FULL_DUP;
		LPC_EMAC->IPGT = EMAC_IPGT_HALF_DUP;
	}
}

/**
 * This function provides a method for the PHY to setup the EMAC
 * for the PHY negotiated bit rate.
 *
 * @param[in] mbs_100     0 = 10mbs mode, 1 = 100mbs mode
 */
void lpc_emac_set_speed(int mbs_100)
{
	if (mbs_100)
		LPC_EMAC->SUPP = EMAC_SUPP_SPEED;
	else
		LPC_EMAC->SUPP = 0;
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param[in] netif the lwip network interface structure for this lpc_enetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t lpc_enetif_init(struct netif *netif)
{
	err_t err;

	LWIP_ASSERT("netif != NULL", (netif != NULL));
    
	lpc_enetdata.netif = netif;

	/* set MAC hardware address */
	board_get_macaddr(netif->hwaddr);
	netif->hwaddr_len = ETHARP_HWADDR_LEN;

 	/* maximum transfer unit */
	netif->mtu = 1500;

	/* device capabilities */
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_UP |
		NETIF_FLAG_ETHERNET;

	/* Initialize the hardware */
	netif->state = &lpc_enetdata;
	err = low_level_init(netif);
	if (err != ERR_OK)
		return err;

#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	netif->hostname = "lwiplpc";
#endif /* LWIP_NETIF_HOSTNAME */

	netif->name[0] = 'e';
	netif->name[1] = 'n';

	netif->output = etharp_output;
	netif->linkoutput = lpc_low_level_output;

	return ERR_OK;
}

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
