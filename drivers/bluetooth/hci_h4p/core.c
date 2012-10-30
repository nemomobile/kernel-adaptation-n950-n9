/*
 * This file is part of hci_h4p bluetooth driver
 *
 * Copyright (C) 2005-2008 Nokia Corporation.
 *
 * Contact: Ville Tervo <ville.tervo@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/serial_reg.h>
#include <linux/skbuff.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <linux/kthread.h>

#include <mach/hardware.h>
#include <mach/irqs.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <net/bluetooth/hci.h>

#include "hci_h4p.h"

static struct task_struct *h4p_thread;

/* This should be used in function that cannot release clocks */
static void hci_h4p_set_clk(struct hci_h4p_info *info, int *clock, int enable)
{
	unsigned long flags;

	spin_lock_irqsave(&info->clocks_lock, flags);
	if (enable && !*clock) {
		NBT_DBG_POWER("Enabling %p\n", clock);
		clk_enable(info->uart_fclk);
#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)
		if (cpu_is_omap24xx() || cpu_is_omap34xx())
			clk_enable(info->uart_iclk);
#endif
		if (atomic_read(&info->clk_users) == 0)
			hci_h4p_restore_regs(info);
		atomic_inc(&info->clk_users);
	}

	if (!enable && *clock) {
		NBT_DBG_POWER("Disabling %p\n", clock);
		if (atomic_dec_and_test(&info->clk_users))
			hci_h4p_store_regs(info);
		clk_disable(info->uart_fclk);
#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)
		if (cpu_is_omap24xx() || cpu_is_omap34xx())
			clk_disable(info->uart_iclk);
#endif
	}

	*clock = enable;
	spin_unlock_irqrestore(&info->clocks_lock, flags);
}

static void hci_h4p_lazy_clock_release(unsigned long data)
{
	struct hci_h4p_info *info = (struct hci_h4p_info *)data;
	unsigned long flags;

	spin_lock_irqsave(&info->lock, flags);
	if (!info->tx_enabled)
		hci_h4p_set_clk(info, &info->tx_clocks_en, 0);
	spin_unlock_irqrestore(&info->lock, flags);
}

/* Power management functions */
void hci_h4p_smart_idle(struct hci_h4p_info *info, bool enable)
{
	u8 v;

	v = hci_h4p_inb(info, UART_OMAP_SYSC);
	v &= ~(UART_OMAP_SYSC_IDLEMASK);

	if (enable)
		v |= UART_OMAP_SYSC_SMART_IDLE;
	else
		v |= UART_OMAP_SYSC_NO_IDLE;

	hci_h4p_outb(info, UART_OMAP_SYSC, v);
}

static inline void h4p_schedule_pm(struct hci_h4p_info *info)
{
	if (unlikely(!h4p_thread))
		return;

	set_bit(H4P_SCHED_TRANSFER_MODE, &info->pm_flags);

	if (unlikely(!test_bit(H4P_TRANSFER_MODE, &info->pm_flags)))
		wake_up_process(h4p_thread);
}

static void hci_h4p_disable_tx(struct hci_h4p_info *info)
{
	NBT_DBG_POWER("\n");

	if (!info->pm_enabled)
		return;

	/* Re-enable smart-idle */
	hci_h4p_smart_idle(info, 1);

	gpio_set_value(info->bt_wakeup_gpio, 0);
	mod_timer(&info->lazy_release, jiffies + msecs_to_jiffies(100));
	info->tx_enabled = 0;
}

void hci_h4p_enable_tx(struct hci_h4p_info *info)
{
	unsigned long flags;
	NBT_DBG_POWER("\n");

	if (!info->pm_enabled)
		return;

	h4p_schedule_pm(info);

	spin_lock_irqsave(&info->lock, flags);
	del_timer(&info->lazy_release);
	hci_h4p_set_clk(info, &info->tx_clocks_en, 1);
	info->tx_enabled = 1;
	gpio_set_value(info->bt_wakeup_gpio, 1);
	hci_h4p_outb(info, UART_IER, hci_h4p_inb(info, UART_IER) |
		     UART_IER_THRI);
	/*
	 * Disable smart-idle as UART TX interrupts
	 * are not wake-up capable
	 */
	hci_h4p_smart_idle(info, 0);

	spin_unlock_irqrestore(&info->lock, flags);
}

static void hci_h4p_disable_rx(struct hci_h4p_info *info)
{
	if (!info->pm_enabled)
		return;

	info->rx_enabled = 0;

	if (hci_h4p_inb(info, UART_LSR) & UART_LSR_DR)
		return;

	if (!(hci_h4p_inb(info, UART_LSR) & UART_LSR_TEMT))
		return;

	__hci_h4p_set_auto_ctsrts(info, 0, UART_EFR_RTS);
	info->autorts = 0;
	hci_h4p_set_clk(info, &info->rx_clocks_en, 0);
}

static void hci_h4p_enable_rx(struct hci_h4p_info *info)
{
	if (!info->pm_enabled)
		return;

	h4p_schedule_pm(info);

	hci_h4p_set_clk(info, &info->rx_clocks_en, 1);
	info->rx_enabled = 1;

	if (!(hci_h4p_inb(info, UART_LSR) & UART_LSR_TEMT))
		return;

	__hci_h4p_set_auto_ctsrts(info, 1, UART_EFR_RTS);
	info->autorts = 1;
}

/* Negotiation functions */
int hci_h4p_send_alive_packet(struct hci_h4p_info *info)
{
	struct hci_h4p_alive_hdr *hdr;
	struct hci_h4p_alive_pkt *pkt;
	struct sk_buff *skb;
	unsigned long flags;
	int len;

	NBT_DBG("Sending alive packet\n");

	len = H4_TYPE_SIZE + sizeof(*hdr) + sizeof(*pkt);
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0x00, len);
	*skb_put(skb, 1) = H4_ALIVE_PKT;
	hdr = (struct hci_h4p_alive_hdr *)skb_put(skb, sizeof(*hdr));
	hdr->dlen = sizeof(*pkt);
	pkt = (struct hci_h4p_alive_pkt *)skb_put(skb, sizeof(*pkt));
	pkt->mid = H4P_ALIVE_REQ;

	skb_queue_tail(&info->txq, skb);
	spin_lock_irqsave(&info->lock, flags);
	hci_h4p_outb(info, UART_IER, hci_h4p_inb(info, UART_IER) |
		     UART_IER_THRI);
	spin_unlock_irqrestore(&info->lock, flags);

	NBT_DBG("Alive packet sent\n");

	return 0;
}

static void hci_h4p_alive_packet(struct hci_h4p_info *info,
				 struct sk_buff *skb)
{
	struct hci_h4p_alive_hdr *hdr;
	struct hci_h4p_alive_pkt *pkt;

	NBT_DBG("Received alive packet\n");
	hdr = (struct hci_h4p_alive_hdr *)skb->data;
	if (hdr->dlen != sizeof(*pkt)) {
		dev_err(info->dev, "Corrupted alive message\n");
		info->init_error = -EIO;
		goto finish_alive;
	}

	pkt = (struct hci_h4p_alive_pkt *)skb_pull(skb, sizeof(*hdr));
	if (pkt->mid != H4P_ALIVE_RESP) {
		dev_err(info->dev, "Could not negotiate hci_h4p settings\n");
		info->init_error = -EINVAL;
	}

finish_alive:
	complete(&info->init_completion);
	kfree_skb(skb);
}

static int hci_h4p_send_negotiation(struct hci_h4p_info *info)
{
	struct hci_h4p_neg_cmd *neg_cmd;
	struct hci_h4p_neg_hdr *neg_hdr;
	struct sk_buff *skb;
	unsigned long flags;
	int err, len;
	u16 sysclk;

	NBT_DBG("Sending negotiation..\n");

	switch (info->bt_sysclk) {
	case BT_SYSCLK_12:
		sysclk = 12000;
		break;
	case BT_SYSCLK_38_4:
		sysclk = 38400;
		break;
	default:
		return -EINVAL;
	}

	len = sizeof(*neg_cmd) + sizeof(*neg_hdr) + H4_TYPE_SIZE;
	skb = bt_skb_alloc(len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	memset(skb->data, 0x00, len);
	*skb_put(skb, 1) = H4_NEG_PKT;
	neg_hdr = (struct hci_h4p_neg_hdr *)skb_put(skb, sizeof(*neg_hdr));
	neg_cmd = (struct hci_h4p_neg_cmd *)skb_put(skb, sizeof(*neg_cmd));

	neg_hdr->dlen = sizeof(*neg_cmd);
	neg_cmd->ack = H4P_NEG_REQ;
	neg_cmd->baud = cpu_to_le16(BT_BAUDRATE_DIVIDER/MAX_BAUD_RATE);
	neg_cmd->proto = H4P_PROTO_BYTE;
	neg_cmd->sys_clk = cpu_to_le16(sysclk);

	hci_h4p_change_speed(info, INIT_SPEED);

	hci_h4p_set_rts(info, 1);
	info->init_error = 0;
	init_completion(&info->init_completion);
	skb_queue_tail(&info->txq, skb);
	spin_lock_irqsave(&info->lock, flags);
	hci_h4p_outb(info, UART_IER, hci_h4p_inb(info, UART_IER) |
		     UART_IER_THRI);
	spin_unlock_irqrestore(&info->lock, flags);

	if (!wait_for_completion_interruptible_timeout(&info->init_completion,
				msecs_to_jiffies(1000)))
		return -ETIMEDOUT;

	if (info->init_error < 0)
		return info->init_error;

	/* Change to operational settings */
	hci_h4p_set_auto_ctsrts(info, 0, UART_EFR_RTS);
	hci_h4p_set_rts(info, 0);
	hci_h4p_change_speed(info, MAX_BAUD_RATE);

	err = hci_h4p_wait_for_cts(info, 1, 100);
	if (err < 0)
		return err;

	hci_h4p_set_auto_ctsrts(info, 1, UART_EFR_RTS);
	init_completion(&info->init_completion);
	err = hci_h4p_send_alive_packet(info);

	if (err < 0)
		return err;

	if (!wait_for_completion_interruptible_timeout(&info->init_completion,
				msecs_to_jiffies(1000)))
		return -ETIMEDOUT;

	if (info->init_error < 0)
		return info->init_error;

	NBT_DBG("Negotiation succesful\n");
	return 0;
}

static void hci_h4p_negotiation_packet(struct hci_h4p_info *info,
				       struct sk_buff *skb)
{
	struct hci_h4p_neg_hdr *hdr;
	struct hci_h4p_neg_evt *evt;

	hdr = (struct hci_h4p_neg_hdr *)skb->data;
	if (hdr->dlen != sizeof(*evt)) {
		info->init_error = -EIO;
		goto finish_neg;
	}

	evt = (struct hci_h4p_neg_evt *)skb_pull(skb, sizeof(*hdr));

	if (evt->ack != H4P_NEG_ACK) {
		dev_err(info->dev, "Could not negotiate hci_h4p settings\n");
		info->init_error = -EINVAL;
	}

	info->man_id = evt->man_id;
	info->ver_id = evt->ver_id;

finish_neg:

	complete(&info->init_completion);
	kfree_skb(skb);
}

/* H4 packet handling functions */
static int hci_h4p_get_hdr_len(struct hci_h4p_info *info, u8 pkt_type)
{
	long retval;

	switch (pkt_type) {
	case H4_EVT_PKT:
		retval = HCI_EVENT_HDR_SIZE;
		break;
	case H4_ACL_PKT:
		retval = HCI_ACL_HDR_SIZE;
		break;
	case H4_SCO_PKT:
		retval = HCI_SCO_HDR_SIZE;
		break;
	case H4_NEG_PKT:
		retval = H4P_NEG_HDR_SIZE;
		break;
	case H4_ALIVE_PKT:
		retval = H4P_ALIVE_HDR_SIZE;
		break;
	case H4_RADIO_PKT:
		retval = H4_RADIO_HDR_SIZE;
		break;
	default:
		dev_err(info->dev, "Unknown H4 packet type 0x%.2x\n", pkt_type);
		retval = -1;
		break;
	}

	return retval;
}

static unsigned int hci_h4p_get_data_len(struct hci_h4p_info *info,
					 struct sk_buff *skb)
{
	long retval = -1;
	struct hci_acl_hdr *acl_hdr;
	struct hci_sco_hdr *sco_hdr;
	struct hci_event_hdr *evt_hdr;
	struct hci_h4p_neg_hdr *neg_hdr;
	struct hci_h4p_alive_hdr *alive_hdr;
	struct hci_h4p_radio_hdr *radio_hdr;

	switch (bt_cb(skb)->pkt_type) {
	case H4_EVT_PKT:
		evt_hdr = (struct hci_event_hdr *)skb->data;
		retval = evt_hdr->plen;
		break;
	case H4_ACL_PKT:
		acl_hdr = (struct hci_acl_hdr *)skb->data;
		retval = le16_to_cpu(acl_hdr->dlen);
		break;
	case H4_SCO_PKT:
		sco_hdr = (struct hci_sco_hdr *)skb->data;
		retval = sco_hdr->dlen;
		break;
	case H4_RADIO_PKT:
		radio_hdr = (struct hci_h4p_radio_hdr *)skb->data;
		retval = radio_hdr->dlen;
		break;
	case H4_NEG_PKT:
		neg_hdr = (struct hci_h4p_neg_hdr *)skb->data;
		retval = neg_hdr->dlen;
		break;
	case H4_ALIVE_PKT:
		alive_hdr = (struct hci_h4p_alive_hdr *)skb->data;
		retval = alive_hdr->dlen;
		break;
	}

	return retval;
}

static inline void hci_h4p_recv_frame(struct hci_h4p_info *info,
				      struct sk_buff *skb)
{
	if (unlikely(!test_bit(HCI_RUNNING, &info->hdev->flags))) {
		if (bt_cb(skb)->pkt_type == H4_NEG_PKT) {
			hci_h4p_negotiation_packet(info, skb);
			info->rx_state = WAIT_FOR_PKT_TYPE;
			return;
		}
		if (bt_cb(skb)->pkt_type == H4_ALIVE_PKT) {
			hci_h4p_alive_packet(info, skb);
			info->rx_state = WAIT_FOR_PKT_TYPE;
			return;
		}

		if (!test_bit(HCI_UP, &info->hdev->flags)) {
			NBT_DBG("fw_event\n");
			hci_h4p_parse_fw_event(info, skb);
			return;
		}
	}

	hci_recv_frame(skb);
	NBT_DBG("Frame sent to upper layer\n");
}

static inline void hci_h4p_handle_byte(struct hci_h4p_info *info, u8 byte)
{
	switch (info->rx_state) {
	case WAIT_FOR_PKT_TYPE:
		bt_cb(info->rx_skb)->pkt_type = byte;
		info->rx_count = hci_h4p_get_hdr_len(info, byte);
		if (info->rx_count < 0) {
			info->hdev->stat.err_rx++;
			kfree_skb(info->rx_skb);
			info->rx_skb = NULL;
		} else {
			info->rx_state = WAIT_FOR_HEADER;
		}
		break;
	case WAIT_FOR_HEADER:
		info->rx_count--;
		*skb_put(info->rx_skb, 1) = byte;
		if (info->rx_count == 0) {
			info->rx_count = hci_h4p_get_data_len(info,
							      info->rx_skb);
			if (info->rx_count > skb_tailroom(info->rx_skb)) {
				dev_err(info->dev, "Too long frame.\n");
				info->garbage_bytes = info->rx_count -
					skb_tailroom(info->rx_skb);
				kfree_skb(info->rx_skb);
				info->rx_skb = NULL;
				break;
			}
			info->rx_state = WAIT_FOR_DATA;

		}
		break;
	case WAIT_FOR_DATA:
		info->rx_count--;
		*skb_put(info->rx_skb, 1) = byte;
		break;
	default:
		WARN_ON(1);
		break;
	}

	if (info->rx_count == 0) {
		/* H4+ devices should allways send word aligned
		 * packets */
		if (!(info->rx_skb->len % 2))
			info->garbage_bytes++;
		hci_h4p_recv_frame(info, info->rx_skb);
		info->rx_skb = NULL;
	}
}

static void hci_h4p_rx_tasklet(unsigned long data)
{
	u8 byte;
	struct hci_h4p_info *info = (struct hci_h4p_info *)data;

	NBT_DBG("tasklet woke up\n");
	NBT_DBG_TRANSFER("rx_tasklet woke up\ndata ");

	while (hci_h4p_inb(info, UART_LSR) & UART_LSR_DR) {
		byte = hci_h4p_inb(info, UART_RX);
		if (info->garbage_bytes) {
			info->garbage_bytes--;
			continue;
		}
		if (info->rx_skb == NULL) {
			info->rx_skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE,
						    GFP_ATOMIC | GFP_DMA);
			if (!info->rx_skb) {
				dev_err(info->dev,
					"No memory for new packet\n");
				goto finish_rx;
			}
			info->rx_state = WAIT_FOR_PKT_TYPE;
			info->rx_skb->dev = (void *)info->hdev;
		}
		info->hdev->stat.byte_rx++;
		NBT_DBG_TRANSFER_NF("0x%.2x  ", byte);
		hci_h4p_handle_byte(info, byte);
	}

	if (!info->rx_enabled) {
		if (hci_h4p_inb(info, UART_LSR) & UART_LSR_TEMT &&
						  info->autorts) {
			__hci_h4p_set_auto_ctsrts(info, 0 , UART_EFR_RTS);
			info->autorts = 0;
		}
		/* Flush posted write to avoid spurious interrupts */
		hci_h4p_inb(info, UART_OMAP_SCR);
		hci_h4p_set_clk(info, &info->rx_clocks_en, 0);
	}

finish_rx:
	NBT_DBG_TRANSFER_NF("\n");
	NBT_DBG("rx_ended\n");
}

static void hci_h4p_tx_tasklet(unsigned long data)
{
	unsigned int sent = 0;
	struct sk_buff *skb;
	struct hci_h4p_info *info = (struct hci_h4p_info *)data;

	NBT_DBG("tasklet woke up\n");
	NBT_DBG_TRANSFER("tx_tasklet woke up\n data ");

	if (info->autorts != info->rx_enabled) {
		if (hci_h4p_inb(info, UART_LSR) & UART_LSR_TEMT) {
			if (info->autorts && !info->rx_enabled) {
				__hci_h4p_set_auto_ctsrts(info, 0,
							  UART_EFR_RTS);
				info->autorts = 0;
			}
			if (!info->autorts && info->rx_enabled) {
				__hci_h4p_set_auto_ctsrts(info, 1,
							  UART_EFR_RTS);
				info->autorts = 1;
			}
		} else {
			hci_h4p_outb(info, UART_OMAP_SCR,
				     hci_h4p_inb(info, UART_OMAP_SCR) |
				     UART_OMAP_SCR_EMPTY_THR);
			goto finish_tx;
		}
	}

	skb = skb_dequeue(&info->txq);
	if (!skb) {
		/* No data in buffer */
		NBT_DBG("skb ready\n");
		if (hci_h4p_inb(info, UART_LSR) & UART_LSR_TEMT) {
			hci_h4p_outb(info, UART_IER,
				     hci_h4p_inb(info, UART_IER) &
				     ~UART_IER_THRI);
			hci_h4p_inb(info, UART_OMAP_SCR);
			hci_h4p_disable_tx(info);
			return;
		} else
			hci_h4p_outb(info, UART_OMAP_SCR,
				     hci_h4p_inb(info, UART_OMAP_SCR) |
				     UART_OMAP_SCR_EMPTY_THR);
		goto finish_tx;
	}

	/* Copy data to tx fifo */
	while (!(hci_h4p_inb(info, UART_OMAP_SSR) & UART_OMAP_SSR_TXFULL) &&
	       (sent < skb->len)) {
		NBT_DBG_TRANSFER_NF("0x%.2x ", skb->data[sent]);
		hci_h4p_outb(info, UART_TX, skb->data[sent]);
		sent++;
	}

	info->hdev->stat.byte_tx += sent;
	NBT_DBG_TRANSFER_NF("\n");
	if (skb->len == sent) {
		kfree_skb(skb);
	} else {
		skb_pull(skb, sent);
		skb_queue_head(&info->txq, skb);
	}

	hci_h4p_outb(info, UART_OMAP_SCR, hci_h4p_inb(info, UART_OMAP_SCR) &
						     ~UART_OMAP_SCR_EMPTY_THR);
	hci_h4p_outb(info, UART_IER, hci_h4p_inb(info, UART_IER) |
						 UART_IER_THRI);

finish_tx:
	/* Flush posted write to avoid spurious interrupts */
	hci_h4p_inb(info, UART_OMAP_SCR);

}

static irqreturn_t hci_h4p_interrupt(int irq, void *data)
{
	struct hci_h4p_info *info = (struct hci_h4p_info *)data;
	u8 iir, msr;
	int ret;

	ret = IRQ_NONE;

	iir = hci_h4p_inb(info, UART_IIR);
	if (iir & UART_IIR_NO_INT)
		return IRQ_HANDLED;

	NBT_DBG("In interrupt handler iir 0x%.2x\n", iir);

	iir &= UART_IIR_ID;

	if (iir == UART_IIR_MSI) {
		msr = hci_h4p_inb(info, UART_MSR);
		ret = IRQ_HANDLED;
	}
	if (iir == UART_IIR_RLSI) {
		hci_h4p_inb(info, UART_RX);
		hci_h4p_inb(info, UART_LSR);
		ret = IRQ_HANDLED;
	}

	if (iir == UART_IIR_RDI) {
		hci_h4p_rx_tasklet((unsigned long)data);
		ret = IRQ_HANDLED;
	}

	if (iir == UART_IIR_THRI) {
		hci_h4p_tx_tasklet((unsigned long)data);
		ret = IRQ_HANDLED;
	}

	return ret;
}

static irqreturn_t hci_h4p_wakeup_interrupt(int irq, void *dev_inst)
{
	struct hci_h4p_info *info = dev_inst;
	int should_wakeup;
	struct hci_dev *hdev;

	if (!info->hdev)
		return IRQ_HANDLED;

	should_wakeup = gpio_get_value(info->host_wakeup_gpio);
	hdev = info->hdev;

	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		if (should_wakeup == 1)
			complete_all(&info->test_completion);

		return IRQ_HANDLED;
	}

	NBT_DBG_POWER("gpio interrupt %d\n", should_wakeup);

	/* Check if wee have missed some interrupts */
	if (info->rx_enabled == should_wakeup)
		return IRQ_HANDLED;

	if (should_wakeup)
		hci_h4p_enable_rx(info);
	else
		hci_h4p_disable_rx(info);

	return IRQ_HANDLED;
}

static inline void hci_h4p_set_pm_limits(struct hci_h4p_info *info, bool set)
{
	struct omap_bluetooth_config *bt_config = info->dev->platform_data;

	if (unlikely(!bt_config || !bt_config->set_pm_limits))
		return;

	if (set && !test_bit(H4P_ACTIVE_MODE, &info->pm_flags)) {
		bt_config->set_pm_limits(info->dev, set);
		set_bit(H4P_ACTIVE_MODE, &info->pm_flags);
		BT_DBG("Change pm constraints to: %s", set ?
				"set" : "clear");
		return;
	}

	if (!set && test_bit(H4P_ACTIVE_MODE, &info->pm_flags)) {
		bt_config->set_pm_limits(info->dev, set);
		clear_bit(H4P_ACTIVE_MODE, &info->pm_flags);
		BT_DBG("Change pm constraints to: %s",
				set ? "set" : "clear");
		return;
	}

	BT_DBG("pm constraints remains: %s",
			set ? "set" : "clear");
}

static int h4p_run(void *data)
{
#define TIMEOUT_MIN msecs_to_jiffies(100)
#define TIMEOUT_MAX msecs_to_jiffies(2000)
	struct hci_h4p_info *info = data;
	unsigned long last_jiffies = jiffies;
	unsigned long timeout = TIMEOUT_MIN;
	unsigned long elapsed;
	BT_DBG("");
	set_user_nice(current, -10);

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (!test_bit(H4P_SCHED_TRANSFER_MODE, &info->pm_flags)) {
			if (timeout != TIMEOUT_MIN) {
				BT_DBG("Exit from active mode. Rest. constr.");
				hci_h4p_set_pm_limits(info, false);
			}

			BT_DBG("No pending events. Sleeping.");
			schedule();
		}

		set_bit(H4P_TRANSFER_MODE, &info->pm_flags);
		clear_bit(H4P_SCHED_TRANSFER_MODE, &info->pm_flags);

		elapsed = jiffies - last_jiffies;

		BT_DBG("Wake up. %u msec expired since last BT activity.",
				jiffies_to_msecs(elapsed));
		BT_DBG("Timeout before calculation = %u",
				jiffies_to_msecs(timeout));

		/* Empiric analyzer  :-) */
		if (elapsed < TIMEOUT_MIN) {
			timeout <<= 1;
			timeout = (timeout > TIMEOUT_MAX) ?
				TIMEOUT_MAX : timeout;
		} else {
			timeout = (elapsed > timeout - TIMEOUT_MIN) ?
				TIMEOUT_MIN : timeout - elapsed;
		}

		BT_DBG("Timeout after calculation = %u",
				jiffies_to_msecs(timeout));

		/* Sometimes we get couple of HCI command during (e)SCO
		   connection. Turn ON transfer mode _ONLY_ if there is
		   still BT activity after 100ms sleep */
		if (timeout == TIMEOUT_MIN)
			BT_DBG("Do not enable transfer mode yet");
		else {
			hci_h4p_set_pm_limits(info, true);
			BT_DBG("Set active mode for %u msec.",
					jiffies_to_msecs(timeout));
		}

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(timeout);

		last_jiffies = jiffies;
		clear_bit(H4P_TRANSFER_MODE, &info->pm_flags);
	}

	hci_h4p_set_pm_limits(info, false);

	return 0;
}

static int hci_h4p_reset(struct hci_h4p_info *info)
{
	int err;

	err = hci_h4p_reset_uart(info);
	if (err < 0) {
		dev_err(info->dev, "Uart reset failed\n");
		return err;
	}
	hci_h4p_init_uart(info);
	hci_h4p_set_rts(info, 0);

	gpio_set_value(info->reset_gpio, 0);
	gpio_set_value(info->bt_wakeup_gpio, 1);
	msleep(10);

	if (gpio_get_value(info->host_wakeup_gpio) == 1) {
		dev_err(info->dev, "host_wakeup_gpio not low\n");
		return -EPROTO;
	}

	INIT_COMPLETION(info->test_completion);
	gpio_set_value(info->reset_gpio, 1);

	if (!wait_for_completion_interruptible_timeout(&info->test_completion,
						       msecs_to_jiffies(100))) {
		dev_err(info->dev, "wakeup test timed out\n");
		complete_all(&info->test_completion);
		return -EPROTO;
	}

	err = hci_h4p_wait_for_cts(info, 1, 100);
	if (err < 0) {
		dev_err(info->dev, "No cts from bt chip\n");
		return err;
	}

	hci_h4p_set_rts(info, 1);

	return 0;
}

/* hci callback functions */
static int hci_h4p_hci_flush(struct hci_dev *hdev)
{
	struct hci_h4p_info *info;
	info = hci_get_drvdata(hdev);

	skb_queue_purge(&info->txq);

	return 0;
}

static int hci_h4p_bt_wakeup_test(struct hci_h4p_info *info)
{
	/* Test Sequence:
	 * Host de-asserts the BT_WAKE_UP line.
	 * Host polls the UART_CTS line, waiting for it to be de-asserted.
	 * Host asserts the BT_WAKE_UP line.
	 * Host polls the UART_CTS line, waiting for it to be asserted.
	 * Host de-asserts the BT_WAKE_UP line (allow the Bluetooth device to
	 * sleep).
	 * Host polls the UART_CTS line, waiting for it to be de-asserted.
	 */
	int err;
	int ret = -ECOMM;

	if (!info)
		return -EINVAL;

	/* Disable wakeup interrupts */
	disable_irq(gpio_to_irq(info->host_wakeup_gpio));

	gpio_set_value(info->bt_wakeup_gpio, 0);
	err = hci_h4p_wait_for_cts(info, 0, 100);
	if (err) {
		dev_warn(info->dev, "bt_wakeup_test: fail: "
			 "CTS low timed out: %d\n", err);
		goto out;
	}

	gpio_set_value(info->bt_wakeup_gpio, 1);
	err = hci_h4p_wait_for_cts(info, 1, 100);
	if (err) {
		dev_warn(info->dev, "bt_wakeup_test: fail: "
			 "CTS high timed out: %d\n", err);
		goto out;
	}

	gpio_set_value(info->bt_wakeup_gpio, 0);
	err = hci_h4p_wait_for_cts(info, 0, 100);
	if (err) {
		dev_warn(info->dev, "bt_wakeup_test: fail: "
			 "CTS re-low timed out: %d\n", err);
		goto out;
	}

	ret = 0;

out:

	/* Re-enable wakeup interrupts */
	enable_irq(gpio_to_irq(info->host_wakeup_gpio));

	return ret;
}

static int hci_h4p_hci_open(struct hci_dev *hdev)
{
	struct hci_h4p_info *info;
	int err, retries = 0;
	struct sk_buff_head fw_queue;
	unsigned long flags;

	info = hci_get_drvdata(hdev);

	if (test_bit(HCI_RUNNING, &hdev->flags))
		return 0;

	/* TI1271 has HW bug and boot up might fail. Retry up to three times */
again:

	info->rx_enabled = 1;
	info->rx_state = WAIT_FOR_PKT_TYPE;
	info->rx_count = 0;
	info->garbage_bytes = 0;
	info->rx_skb = NULL;
	info->pm_enabled = 0;
	init_completion(&info->fw_completion);
	hci_h4p_set_clk(info, &info->tx_clocks_en, 1);
	hci_h4p_set_clk(info, &info->rx_clocks_en, 1);
	skb_queue_head_init(&fw_queue);

	err = hci_h4p_reset(info);
	if (err < 0)
		goto err_clean;

	hci_h4p_set_auto_ctsrts(info, 1, UART_EFR_CTS | UART_EFR_RTS);
	info->autorts = 1;

	err = hci_h4p_send_negotiation(info);

	err = hci_h4p_read_fw(info, &fw_queue);
	if (err < 0) {
		dev_err(info->dev, "Cannot read firmware\n");
		return err;
	}

	err = hci_h4p_send_fw(info, &fw_queue);
	if (err < 0) {
		dev_err(info->dev, "Sending firmware failed.\n");
		goto err_clean;
	}

	info->pm_enabled = 1;

	err = hci_h4p_bt_wakeup_test(info);
	if (err < 0) {
		dev_err(info->dev, "BT wakeup test failed.\n");
		goto err_clean;
	}

	spin_lock_irqsave(&info->lock, flags);
	info->rx_enabled = gpio_get_value(info->host_wakeup_gpio);
	hci_h4p_set_clk(info, &info->rx_clocks_en, info->rx_enabled);
	spin_unlock_irqrestore(&info->lock, flags);

	hci_h4p_set_clk(info, &info->tx_clocks_en, 0);

	kfree_skb(info->alive_cmd_skb);
	info->alive_cmd_skb = NULL;
	set_bit(HCI_RUNNING, &hdev->flags);

	NBT_DBG("hci up and running\n");
	return 0;

err_clean:
	hci_h4p_hci_flush(hdev);
	hci_h4p_reset_uart(info);
	del_timer_sync(&info->lazy_release);
	hci_h4p_set_clk(info, &info->tx_clocks_en, 0);
	hci_h4p_set_clk(info, &info->rx_clocks_en, 0);
	gpio_set_value(info->reset_gpio, 0);
	gpio_set_value(info->bt_wakeup_gpio, 0);
	skb_queue_purge(&fw_queue);
	kfree_skb(info->alive_cmd_skb);
	info->alive_cmd_skb = NULL;
	kfree_skb(info->rx_skb);
	info->rx_skb = NULL;

	if (retries++ < 3) {
		dev_err(info->dev, "FW loading try %d fail. Retry.\n", retries);
		goto again;
	}

	return err;
}

static int hci_h4p_hci_close(struct hci_dev *hdev)
{
	struct hci_h4p_info *info = hci_get_drvdata(hdev);

	if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags))
		return 0;

	/* Wake up h4p_thread which removes pm constraints */
	wake_up_process(h4p_thread);

	hci_h4p_hci_flush(hdev);
	hci_h4p_set_clk(info, &info->tx_clocks_en, 1);
	hci_h4p_set_clk(info, &info->rx_clocks_en, 1);
	hci_h4p_reset_uart(info);
	del_timer_sync(&info->lazy_release);
	hci_h4p_set_clk(info, &info->tx_clocks_en, 0);
	hci_h4p_set_clk(info, &info->rx_clocks_en, 0);
	gpio_set_value(info->reset_gpio, 0);
	gpio_set_value(info->bt_wakeup_gpio, 0);
	kfree_skb(info->rx_skb);

	return 0;
}

static void hci_h4p_hci_destruct(struct hci_dev *hdev)
{
}

static int hci_h4p_hci_send_frame(struct sk_buff *skb)
{
	struct hci_h4p_info *info;
	struct hci_dev *hdev = (struct hci_dev *)skb->dev;
	int err = 0;

	if (!hdev) {
		printk(KERN_WARNING "hci_h4p: Frame for unknown device\n");
		return -ENODEV;
	}

	NBT_DBG("dev %p, skb %p\n", hdev, skb);

	info = hci_get_drvdata(hdev);

	if (!test_bit(HCI_RUNNING, &hdev->flags)) {
		dev_warn(info->dev, "Frame for non-running device\n");
		return -EIO;
	}

	switch (bt_cb(skb)->pkt_type) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	}

	/* Push frame type to skb */
	*skb_push(skb, 1) = (bt_cb(skb)->pkt_type);
	/* We should allways send word aligned data to h4+ devices */
	if (skb->len % 2) {
		err = skb_pad(skb, 1);
		if (!err)
			*skb_put(skb, 1) = 0x00;
	}
	if (err)
		return err;

	skb_queue_tail(&info->txq, skb);
	hci_h4p_enable_tx(info);

	return 0;
}

static int hci_h4p_hci_ioctl(struct hci_dev *hdev, unsigned int cmd,
			     unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static int hci_h4p_register_hdev(struct hci_h4p_info *info)
{
	struct hci_dev *hdev;

	/* Initialize and register HCI device */

	hdev = hci_alloc_dev();
	if (!hdev) {
		dev_err(info->dev, "Can't allocate memory for device\n");
		return -ENOMEM;
	}
	info->hdev = hdev;

	hdev->bus = HCI_UART;
	hci_set_drvdata(hdev, info);

	hdev->open = hci_h4p_hci_open;
	hdev->close = hci_h4p_hci_close;
	hdev->flush = hci_h4p_hci_flush;
	hdev->send = hci_h4p_hci_send_frame;
	hdev->ioctl = hci_h4p_hci_ioctl;

	set_bit(HCI_QUIRK_NO_RESET, &hdev->quirks);
	SET_HCIDEV_DEV(hdev, info->dev);

	if (hci_register_dev(hdev) < 0) {
		dev_err(info->dev, "hci_register failed %s.\n", hdev->name);
		return -ENODEV;
	}

	return 0;
}

static int hci_h4p_probe(struct platform_device *pdev)
{
	struct omap_bluetooth_config *bt_config;
	struct hci_h4p_info *info;
	int irq, err;

	dev_info(&pdev->dev, "Registering HCI H4P device\n");
	info = kzalloc(sizeof(struct hci_h4p_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->tx_enabled = 1;
	info->rx_enabled = 1;
	irq = 0;
	spin_lock_init(&info->lock);
	spin_lock_init(&info->clocks_lock);
	skb_queue_head_init(&info->txq);

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "Could not get Bluetooth config data\n");
		kfree(info);
		return -ENODATA;
	}

	bt_config = pdev->dev.platform_data;
	info->chip_type = bt_config->chip_type;
	info->bt_wakeup_gpio = bt_config->bt_wakeup_gpio;
	info->host_wakeup_gpio = bt_config->host_wakeup_gpio;
	info->reset_gpio = bt_config->reset_gpio;
	info->reset_gpio_shared = bt_config->reset_gpio_shared;
	info->bt_sysclk = bt_config->bt_sysclk;

	NBT_DBG("RESET gpio: %d\n", info->reset_gpio);
	NBT_DBG("BTWU gpio: %d\n", info->bt_wakeup_gpio);
	NBT_DBG("HOSTWU gpio: %d\n", info->host_wakeup_gpio);
	NBT_DBG("Uart: %d\n", bt_config->bt_uart);
	NBT_DBG("sysclk: %d\n", info->bt_sysclk);

	init_completion(&info->test_completion);
	complete_all(&info->test_completion);

	if (!info->reset_gpio_shared) {
		err = gpio_request(info->reset_gpio, "bt_reset");
		if (err < 0) {
			dev_err(&pdev->dev, "Cannot get GPIO line %d\n",
				info->reset_gpio);
			goto cleanup_setup;
		}
	}

	err = gpio_request(info->bt_wakeup_gpio, "bt_wakeup");
	if (err < 0) {
		dev_err(info->dev, "Cannot get GPIO line 0x%d",
			info->bt_wakeup_gpio);
		if (!info->reset_gpio_shared)
			gpio_free(info->reset_gpio);
		goto cleanup_setup;
	}

	err = gpio_request(info->host_wakeup_gpio, "host_wakeup");
	if (err < 0) {
		dev_err(info->dev, "Cannot get GPIO line %d",
		       info->host_wakeup_gpio);
		if (!info->reset_gpio_shared)
			gpio_free(info->reset_gpio);
		gpio_free(info->bt_wakeup_gpio);
		goto cleanup_setup;
	}

	gpio_direction_output(info->reset_gpio, 0);
	gpio_direction_output(info->bt_wakeup_gpio, 0);
	gpio_direction_input(info->host_wakeup_gpio);

	switch (bt_config->bt_uart) {
	case 1:
		if (cpu_is_omap16xx()) {
			irq = INT_UART1;
			info->uart_fclk = clk_get(NULL, "uart1_ck");
		} else if (cpu_is_omap24xx()) {
			irq = INT_24XX_UART1_IRQ;
			info->uart_iclk = clk_get(NULL, "uart1_ick");
			info->uart_fclk = clk_get(NULL, "uart1_fck");
		}
		info->uart_base = ioremap(OMAP2_UART1_BASE, SZ_2K);
		break;
	case 2:
		if (cpu_is_omap16xx()) {
			irq = INT_UART2;
			info->uart_fclk = clk_get(NULL, "uart2_ck");
		} else {
			irq = INT_24XX_UART2_IRQ;
			info->uart_iclk = clk_get(NULL, "uart2_ick");
			info->uart_fclk = clk_get(NULL, "uart2_fck");
		}
		info->uart_base = ioremap(OMAP2_UART2_BASE, SZ_2K);
		break;
	case 3:
		if (cpu_is_omap16xx()) {
			irq = INT_UART3;
			info->uart_fclk = clk_get(NULL, "uart3_ck");
		} else {
			irq = INT_24XX_UART3_IRQ;
			info->uart_iclk = clk_get(NULL, "uart3_ick");
			info->uart_fclk = clk_get(NULL, "uart3_fck");
		}
		info->uart_base = ioremap(OMAP2_UART3_BASE, SZ_2K);
		break;
	default:
		dev_err(info->dev, "No uart defined\n");
		goto cleanup;
	}

	info->irq = irq;
	err = request_irq(irq, hci_h4p_interrupt, IRQF_DISABLED, "hci_h4p",
			  info);
	if (err < 0) {
		dev_err(info->dev, "hci_h4p: unable to get IRQ %d\n", irq);
		goto cleanup;
	}

	err = request_irq(gpio_to_irq(info->host_wakeup_gpio),
			  hci_h4p_wakeup_interrupt,  IRQF_TRIGGER_FALLING |
			  IRQF_TRIGGER_RISING | IRQF_DISABLED,
			  "hci_h4p_wkup", info);
	if (err < 0) {
		dev_err(info->dev, "hci_h4p: unable to get wakeup IRQ %d\n",
			  gpio_to_irq(info->host_wakeup_gpio));
		free_irq(irq, info);
		goto cleanup;
	}

	err = enable_irq_wake(gpio_to_irq(info->host_wakeup_gpio));
	if (err < 0) {
		dev_err(info->dev, "hci_h4p: unable to set wakeup for IRQ %d\n",
				gpio_to_irq(info->host_wakeup_gpio));
		free_irq(irq, info);
		free_irq(gpio_to_irq(info->host_wakeup_gpio), info);
		goto cleanup;
	}

	init_timer_deferrable(&info->lazy_release);
	info->lazy_release.function = hci_h4p_lazy_clock_release;
	info->lazy_release.data = (unsigned long)info;
	hci_h4p_set_clk(info, &info->tx_clocks_en, 1);
	err = hci_h4p_reset_uart(info);
	if (err < 0)
		goto cleanup_irq;
	gpio_set_value(info->reset_gpio, 0);
	hci_h4p_set_clk(info, &info->tx_clocks_en, 0);

	platform_set_drvdata(pdev, info);

	if (hci_h4p_register_hdev(info) < 0) {
		dev_err(info->dev, "failed to register hci_h4p hci device\n");
		goto cleanup_irq;
	}

	h4p_thread = kthread_run(h4p_run, info, "h4p_pm");
	if (IS_ERR(h4p_thread)) {
		err = PTR_ERR(h4p_thread);
		goto cleanup_irq;
	}

	return 0;

cleanup_irq:
	free_irq(irq, (void *)info);
	free_irq(gpio_to_irq(info->host_wakeup_gpio), info);
cleanup:
	gpio_set_value(info->reset_gpio, 0);
	if (!info->reset_gpio_shared)
		gpio_free(info->reset_gpio);
	gpio_free(info->bt_wakeup_gpio);
	gpio_free(info->host_wakeup_gpio);

cleanup_setup:

	kfree(info);
	return err;

}

static int hci_h4p_remove(struct platform_device *pdev)
{
	struct hci_h4p_info *info;

	info = platform_get_drvdata(pdev);

	kthread_stop(h4p_thread);

	hci_h4p_hci_close(info->hdev);
	free_irq(gpio_to_irq(info->host_wakeup_gpio), info);
	hci_unregister_dev(info->hdev);
	hci_free_dev(info->hdev);
	if (!info->reset_gpio_shared)
		gpio_free(info->reset_gpio);
	gpio_free(info->bt_wakeup_gpio);
	gpio_free(info->host_wakeup_gpio);
	free_irq(info->irq, (void *) info);
	kfree(info);

	return 0;
}

static struct platform_driver hci_h4p_driver = {
	.probe		= hci_h4p_probe,
	.remove		= hci_h4p_remove,
	.driver		= {
		.name	= "hci_h4p",
	},
};

static int __init hci_h4p_init(void)
{
	int err = 0;

	/* Register the driver with LDM */
	err = platform_driver_register(&hci_h4p_driver);
	if (err < 0)
		printk(KERN_WARNING "failed to register hci_h4p driver\n");

	return err;
}

static void __exit hci_h4p_exit(void)
{
	platform_driver_unregister(&hci_h4p_driver);
}

module_init(hci_h4p_init);
module_exit(hci_h4p_exit);

MODULE_ALIAS("platform:hci_h4p");
MODULE_DESCRIPTION("h4 driver with nokia extensions");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ville Tervo");
