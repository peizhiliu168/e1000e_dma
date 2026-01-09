#include "e1000_simple.h"
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/timer.h>

#define DRV_NAME "simple_e1000e"

/* Globals for Debugging */
static struct dentry *e1000_dbg_dir;
static u64 next_rx_phy_addr = 0;

struct simple_e1000_adapter {
  struct net_device *netdev;
  struct pci_dev *pdev;
  u8 __iomem *hw_addr; /* Memory mapped I/O addr */

  /* RX Ring */
  struct e1000_rx_desc *rx_ring;
  dma_addr_t rx_ring_dma;
  struct sk_buff *rx_skb[E1000_NUM_RX_DESC];
  u32 rx_next_to_clean;

  /* TX Ring */
  struct e1000_tx_desc *tx_ring;
  dma_addr_t tx_ring_dma;
  struct sk_buff *tx_skb[E1000_NUM_TX_DESC];
  u32 tx_next_to_use;
  u32 tx_next_to_clean;

  /* Polling Timer */
  struct timer_list poll_timer;
};

/* Timer interval in milliseconds */
#define POLL_INTERVAL_MS 10

/* Function Prototypes */
static void e1000_poll_timer(struct timer_list *t);

/*
 * Hardware Reset and Setup
 */
static void e1000_reset_hw(struct simple_e1000_adapter *adapter) {
  u32 ctrl;
  u8 __iomem *hw = adapter->hw_addr;

  /* Assert global reset */
  ctrl = E1000_READ_REG(hw, E1000_CTRL);
  E1000_WRITE_REG(hw, E1000_CTRL, ctrl | E1000_CTRL_RST);
  msleep(20);

  /* Disable interrupts (we are polling) */
  E1000_WRITE_REG(hw, E1000_IMC, 0xFFFFFFFF);

  /* Set Link Up, Force Speed (1000ish but actually bits are different for
   * 1000), Force Duplex */
  /* For 82574L/e1000e, SLU usually bypasses PHY auto-neg if we also set
   * reasonable defaults */
  ctrl = E1000_READ_REG(hw, E1000_CTRL);
  ctrl |= E1000_CTRL_SLU;     /* Set Link Up */
  ctrl |= E1000_CTRL_FRCSPD;  /* Force Speed */
  ctrl |= E1000_CTRL_FRCDPLX; /* Force Duplex */
  /* Also clear SW Isolate if set in PHY_CTRL? No, just stick to MAC CTRL for
   * now */

  E1000_WRITE_REG(hw, E1000_CTRL, ctrl);
}

static void e1000_set_mac(struct simple_e1000_adapter *adapter) {
  struct net_device *netdev = adapter->netdev;
  u8 *mac = netdev->dev_addr;
  u32 ral, rah;

  if (!is_valid_ether_addr(mac))
    return;

  /* RAL0: Low 32 bits (mac[0]..mac[3]) */
  ral = (mac[0]) | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
  /* RAH0: High 16 bits (mac[4]..mac[5]) | AV (Address Valid) */
  rah = (mac[4]) | (mac[5] << 8) | E1000_RAH_AV;

  E1000_WRITE_REG(adapter->hw_addr, E1000_RAL0, ral);
  E1000_WRITE_REG(adapter->hw_addr, E1000_RAH0, rah);

  pr_info("%s: Programmed MAC: %pM\n", DRV_NAME, mac);
}

static void e1000_alloc_rx_buffers(struct simple_e1000_adapter *adapter) {
  struct pci_dev *pdev = adapter->pdev;
  int i;

  for (i = 0; i < E1000_NUM_RX_DESC; i++) {
    struct sk_buff *skb;
    dma_addr_t dma;

    skb = netdev_alloc_skb_ip_align(adapter->netdev, E1000_RX_BUFFER_SIZE);
    if (!skb) {
      pr_err("%s: Failed to allocate RX skb\n", DRV_NAME);
      break;
    }

    dma = dma_map_single(&pdev->dev, skb->data, E1000_RX_BUFFER_SIZE,
                         DMA_FROM_DEVICE);
    if (dma_mapping_error(&pdev->dev, dma)) {
      dev_kfree_skb(skb);
      pr_err("%s: Failed to map RX dma\n", DRV_NAME);
      break;
    }

    adapter->rx_skb[i] = skb;
    adapter->rx_ring[i].buffer_addr = cpu_to_le64(dma);
    adapter->rx_ring[i].status = 0;
  }
}

static void e1000_configure_rx(struct simple_e1000_adapter *adapter) {
  u8 __iomem *hw = adapter->hw_addr;
  u64 rdba = adapter->rx_ring_dma;
  u32 rctl;

  /* Disable receiver while configuring */
  E1000_WRITE_REG(hw, E1000_RCTL, 0);

  /* Setup Ring Registers */
  E1000_WRITE_REG(hw, E1000_RDBAL, (rdba & 0x00000000ffffffffULL));
  E1000_WRITE_REG(hw, E1000_RDBAH, (rdba >> 32));
  E1000_WRITE_REG(hw, E1000_RDLEN,
                  E1000_NUM_RX_DESC * sizeof(struct e1000_rx_desc));

  /* Setup Head and Tail */
  E1000_WRITE_REG(hw, E1000_RDH, 0);
  /* Tail should point to the last valid descriptor.
     Since we filled all, it is NUM - 1 */
  E1000_WRITE_REG(hw, E1000_RDT, E1000_NUM_RX_DESC - 1);

  /* Allocate buffers for the ring */
  e1000_alloc_rx_buffers(adapter);

  /* Enable Receiver */
  /* EN (Enable), BAM (Broadcast Accept), UPE (Unicast Promiscuous), SZ_2048
   * (Size), SECRC (Strip CRC) */
  rctl = E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_UPE | E1000_RCTL_SZ_2048 |
         E1000_RCTL_SECRC;
  E1000_WRITE_REG(hw, E1000_RCTL, rctl);
}

static void e1000_configure_tx(struct simple_e1000_adapter *adapter) {
  u8 __iomem *hw = adapter->hw_addr;
  u64 tdba = adapter->tx_ring_dma;
  u32 tctl;
  u32 tipg;

  /* Setup Ring Registers */
  E1000_WRITE_REG(hw, E1000_TDBAL, (tdba & 0x00000000ffffffffULL));
  E1000_WRITE_REG(hw, E1000_TDBAH, (tdba >> 32));
  E1000_WRITE_REG(hw, E1000_TDLEN,
                  E1000_NUM_TX_DESC * sizeof(struct e1000_tx_desc));

  E1000_WRITE_REG(hw, E1000_TDH, 0);
  E1000_WRITE_REG(hw, E1000_TDT, 0);

  adapter->tx_next_to_use = 0;
  adapter->tx_next_to_clean = 0;

  /* Set standard IPG (Inter Packet Gap) values for 82574L */
  /* IPGT=8, IPGR1=4, IPGR2=6 -> roughly standard values */
  tipg = (10) | (10 << 10) | (10 << 20);
  E1000_WRITE_REG(hw, E1000_TIPG, tipg);

  /* Enable Transmitter */
  /* PSP (Pad Short Packets), CT=0x0f (Collision Threshold), COLD (Collision
   * Distance) */
  tctl = E1000_TCTL_EN | E1000_TCTL_PSP | (0xF << 4) | (0x40 << 12);
  E1000_WRITE_REG(hw, E1000_TCTL, tctl);
}

/*
 * Timer Polling Logic
 */
static void e1000_clean_rx_ring(struct simple_e1000_adapter *adapter) {
  struct pci_dev *pdev = adapter->pdev;
  struct net_device *netdev = adapter->netdev;
  unsigned int i = adapter->rx_next_to_clean;

  /* Check descriptors starting from next_to_clean */
  while (adapter->rx_ring[i].status & E1000_RXD_STAT_DD) {
    struct sk_buff *skb = adapter->rx_skb[i];
    u16 length = le16_to_cpu(adapter->rx_ring[i].length);

    dma_unmap_single(&pdev->dev, le64_to_cpu(adapter->rx_ring[i].buffer_addr),
                     E1000_RX_BUFFER_SIZE, DMA_FROM_DEVICE);

    /* Construct new SKB for upper layers */
    skb_put(skb, length);

    pr_info("%s: Received packet, len=%d\n", DRV_NAME, length);
    print_hex_dump(KERN_INFO, "rx_data: ", DUMP_PREFIX_OFFSET, 16, 1, skb->data,
                   min_t(int, length, 64), true);

    skb->protocol = eth_type_trans(skb, netdev);
    netif_rx(skb);

    /* Stats */
    netdev->stats.rx_packets++;
    netdev->stats.rx_bytes += length;

    /* Re-allocate buffer for this descriptor */
    skb = netdev_alloc_skb_ip_align(netdev, E1000_RX_BUFFER_SIZE);
    if (skb) {
      dma_addr_t dma = dma_map_single(&pdev->dev, skb->data,
                                      E1000_RX_BUFFER_SIZE, DMA_FROM_DEVICE);
      adapter->rx_skb[i] = skb;

      if (next_rx_phy_addr != 0) {
        pr_info("%s: Overriding RX DMA address to 0x%llx\n", DRV_NAME,
                next_rx_phy_addr);
        adapter->rx_ring[i].buffer_addr = cpu_to_le64(next_rx_phy_addr);
        next_rx_phy_addr = 0; /* One shot */
      } else {
        adapter->rx_ring[i].buffer_addr = cpu_to_le64(dma);
      }
    } else {
      /* Reuse the old one if allocation fails?
         Actually we just consumed the old one. We passed it to netif_rx.
         So if alloc fails, we have a hole.
         For simplicity we assume alloc works. If not, this desc is dead. */
      adapter->rx_ring[i].buffer_addr = 0;
    }

    /* Clear Status for next use */
    adapter->rx_ring[i].status = 0;

    i++;
    if (i == E1000_NUM_RX_DESC)
      i = 0;
  }

  adapter->rx_next_to_clean = i;

  /* Update Tail to let HW know we freed up descriptors.
     Tail should point to the last available descriptor.
     If next_to_clean is X, it means X is the one we want HW to write to NEXT,
     so TAIL should be X-1 (modulo).
  */
  {
    u32 t = i;
    if (t == 0)
      t = E1000_NUM_RX_DESC;
    E1000_WRITE_REG(adapter->hw_addr, E1000_RDT, t - 1);
  }
}

static void e1000_clean_tx_ring(struct simple_e1000_adapter *adapter) {
  struct pci_dev *pdev = adapter->pdev;
  unsigned int i = adapter->tx_next_to_clean;

  while (i != adapter->tx_next_to_use) {
    /* Check if the descriptor is done */
    /* If not done, stop cleaning */
    if (!(adapter->tx_ring[i].upper.fields.status & E1000_TXD_STAT_DD))
      break;

    /* Free the skb associated with this descriptor */
    if (adapter->tx_skb[i]) {
      dma_unmap_single(&pdev->dev, le64_to_cpu(adapter->tx_ring[i].buffer_addr),
                       adapter->tx_skb[i]->len, DMA_TO_DEVICE);
      dev_kfree_skb_any(adapter->tx_skb[i]);
      adapter->tx_skb[i] = NULL;
    }

    i++;
    if (i == E1000_NUM_TX_DESC)
      i = 0;
  }

  adapter->tx_next_to_clean = i;

  /* Wake queue if we have space now */
  if (netif_queue_stopped(adapter->netdev)) {
    netif_wake_queue(adapter->netdev);
  }
}

static void e1000_poll_timer(struct timer_list *t) {
  struct simple_e1000_adapter *adapter = from_timer(adapter, t, poll_timer);

  e1000_clean_rx_ring(adapter);
  e1000_clean_tx_ring(adapter);

  /* Heartbeat to show liveness */
  {
    static int heartbeat = 0;
    if (++heartbeat >= 1000) {
      u32 status = readl(adapter->hw_addr + E1000_STATUS);
      u32 rdh = readl(adapter->hw_addr + E1000_RDH);
      u32 rdt = readl(adapter->hw_addr + E1000_RDT);
      pr_info("%s: Heartbeat RCTL=0x%08x STATUS=0x%08x RDH=%u RDT=%u\n",
              DRV_NAME, readl(adapter->hw_addr + E1000_RCTL), status, rdh, rdt);
      heartbeat = 0;
    }
  }

  /* Restart timer */
  mod_timer(&adapter->poll_timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));
}

/*
 * Net Device Operations
 */
static int e1000_open(struct net_device *netdev) {
  struct simple_e1000_adapter *adapter = netdev_priv(netdev);

  e1000_reset_hw(adapter);
  e1000_set_mac(adapter);
  e1000_configure_rx(adapter);
  e1000_configure_tx(adapter);

  /* Start the polling timer */
  timer_setup(&adapter->poll_timer, e1000_poll_timer, 0);
  mod_timer(&adapter->poll_timer, jiffies + msecs_to_jiffies(POLL_INTERVAL_MS));

  netif_start_queue(netdev);
  return 0;
}

static int e1000_stop(struct net_device *netdev) {
  struct simple_e1000_adapter *adapter = netdev_priv(netdev);

  del_timer_sync(&adapter->poll_timer);
  netif_stop_queue(netdev);

  /* Also clean up rings here ideally */
  return 0;
}

static netdev_tx_t e1000_start_xmit(struct sk_buff *skb,
                                    struct net_device *netdev) {
  struct simple_e1000_adapter *adapter = netdev_priv(netdev);
  struct pci_dev *pdev = adapter->pdev;
  dma_addr_t dma;
  u32 i = adapter->tx_next_to_use;
  struct e1000_tx_desc *tx_desc;

  /* Ensure we have space.
     Simple check: If next_to_use == next_to_clean - 1, full. */
  if (((i + 1) % E1000_NUM_TX_DESC) == adapter->tx_next_to_clean) {
    netif_stop_queue(netdev);
    return NETDEV_TX_BUSY;
  }

  dma = dma_map_single(&pdev->dev, skb->data, skb->len, DMA_TO_DEVICE);
  if (dma_mapping_error(&pdev->dev, dma)) {
    dev_kfree_skb(skb);
    return NETDEV_TX_OK;
  }

  adapter->tx_skb[i] = skb;
  tx_desc = &adapter->tx_ring[i];
  tx_desc->buffer_addr = cpu_to_le64(dma);
  tx_desc->lower.data = cpu_to_le32(E1000_TXD_CMD_EOP | E1000_TXD_CMD_IFCS |
                                    E1000_TXD_CMD_RS | skb->len);
  tx_desc->upper.data = 0;

  i++;
  if (i == E1000_NUM_TX_DESC)
    i = 0;
  adapter->tx_next_to_use = i;

  /* Trigger HW */
  E1000_WRITE_REG(adapter->hw_addr, E1000_TDT, i);

  netdev->stats.tx_packets++;
  netdev->stats.tx_bytes += skb->len;

  return NETDEV_TX_OK;
}

static const struct net_device_ops e1000_netdev_ops = {
    .ndo_open = e1000_open,
    .ndo_stop = e1000_stop,
    .ndo_start_xmit = e1000_start_xmit,
    .ndo_validate_addr = eth_validate_addr,
    .ndo_set_mac_address = eth_mac_addr,
};

/*
 * PCI Driver Operations
 */
static int e1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
  struct net_device *netdev;
  struct simple_e1000_adapter *adapter;
  int err;

  err = pci_enable_device(pdev);
  if (err)
    return err;

  err = pci_request_regions(pdev, DRV_NAME);
  if (err)
    goto err_disable;

  pci_set_master(pdev);

  netdev = alloc_etherdev(sizeof(struct simple_e1000_adapter));
  if (!netdev) {
    err = -ENOMEM;
    goto err_release;
  }

  SET_NETDEV_DEV(netdev, &pdev->dev);
  adapter = netdev_priv(netdev);
  adapter->netdev = netdev;
  adapter->pdev = pdev;

  adapter->hw_addr = pci_ioremap_bar(pdev, 0);
  if (!adapter->hw_addr) {
    err = -EIO;
    goto err_free_netdev;
  }

  /* Allocate Descriptor Rings */
  adapter->rx_ring = dma_alloc_coherent(
      &pdev->dev, sizeof(struct e1000_rx_desc) * E1000_NUM_RX_DESC,
      &adapter->rx_ring_dma, GFP_KERNEL);
  if (!adapter->rx_ring)
    goto err_iounmap;

  adapter->tx_ring = dma_alloc_coherent(
      &pdev->dev, sizeof(struct e1000_tx_desc) * E1000_NUM_TX_DESC,
      &adapter->tx_ring_dma, GFP_KERNEL);
  if (!adapter->tx_ring)
    goto err_free_rx;

  netdev->netdev_ops = &e1000_netdev_ops;
  netdev->netdev_ops = &e1000_netdev_ops;
  /* Hardcode MAC: DE:AD:BE:EF:CA:FE */
  {
    u8 mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE};
    memcpy(netdev->dev_addr, mac, 6);
  }

  pr_info("%s: Assigned MAC: %pM\n", DRV_NAME, netdev->dev_addr);

  err = register_netdev(netdev);
  if (err)
    goto err_free_tx;

  pci_set_drvdata(pdev, netdev);
  pr_info("%s: Intel(R) PRO/1000 Network Driver - Simple Polling Mode\n",
          DRV_NAME);

  /* Setup DebugFS */
  e1000_dbg_dir = debugfs_create_dir("simple_e1000e", NULL);
  debugfs_create_x64("next_rx_addr", 0600, e1000_dbg_dir, &next_rx_phy_addr);

  return 0;

err_free_tx:
  dma_free_coherent(&pdev->dev,
                    sizeof(struct e1000_tx_desc) * E1000_NUM_TX_DESC,
                    adapter->tx_ring, adapter->tx_ring_dma);
err_free_rx:
  dma_free_coherent(&pdev->dev,
                    sizeof(struct e1000_rx_desc) * E1000_NUM_RX_DESC,
                    adapter->rx_ring, adapter->rx_ring_dma);
err_iounmap:
  iounmap(adapter->hw_addr);
err_free_netdev:
  free_netdev(netdev);
err_release:
  pci_release_regions(pdev);
err_disable:
  pci_disable_device(pdev);
  return err;
}

static void e1000_remove(struct pci_dev *pdev) {
  struct net_device *netdev = pci_get_drvdata(pdev);
  struct simple_e1000_adapter *adapter = netdev_priv(netdev);

  debugfs_remove_recursive(e1000_dbg_dir);

  unregister_netdev(netdev);

  dma_free_coherent(&pdev->dev,
                    sizeof(struct e1000_tx_desc) * E1000_NUM_TX_DESC,
                    adapter->tx_ring, adapter->tx_ring_dma);
  dma_free_coherent(&pdev->dev,
                    sizeof(struct e1000_rx_desc) * E1000_NUM_RX_DESC,
                    adapter->rx_ring, adapter->rx_ring_dma);

  iounmap(adapter->hw_addr);
  free_netdev(netdev);
  pci_release_regions(pdev);
  pci_disable_device(pdev);
}

static const struct pci_device_id e1000_pci_tbl[] = {
    {PCI_DEVICE(0x8086, INTEL_E1000_82574L)}, {0}};
MODULE_DEVICE_TABLE(pci, e1000_pci_tbl);

static struct pci_driver e1000_driver = {
    .name = DRV_NAME,
    .id_table = e1000_pci_tbl,
    .probe = e1000_probe,
    .remove = e1000_remove,
};

module_pci_driver(e1000_driver);

MODULE_AUTHOR("Antigravity");
MODULE_DESCRIPTION("Simple Polling e1000e Driver");
MODULE_LICENSE("GPL");
