#ifndef _E1000_SIMPLE_H_
#define _E1000_SIMPLE_H_

#include <linux/types.h>

/* PCI Device ID */
#define INTEL_E1000_82574L 0x10D3

/* Register Offsets */
#define E1000_CTRL 0x00000     /* Device Control - RW */
#define E1000_STATUS 0x00008   /* Device Status - RO */
#define E1000_CTRL_EXT 0x00018 /* Extended Device Control - RW */
#define E1000_ICR 0x000C0      /* Interrupt Cause Read - R/clr */
#define E1000_ITR 0x000C4      /* Interrupt Throttling Rate - RW */
#define E1000_ICS 0x000C8      /* Interrupt Cause Set - WO */
#define E1000_IMS 0x000D0      /* Interrupt Mask Set - RW */
#define E1000_IMC 0x000D8      /* Interrupt Mask Clear - WO */

#define E1000_RCTL 0x00100 /* RX Control - RW */
#define E1000_TCTL 0x00400 /* TX Control - RW */
#define E1000_TIPG 0x00410 /* TX Inter-packet gap -RW */

/* Receive Rings */
#define E1000_RDBAL 0x02800 /* RX Descriptor Base Address Low - RW */
#define E1000_RDBAH 0x02804 /* RX Descriptor Base Address High - RW */
#define E1000_RDLEN 0x02808 /* RX Descriptor Length - RW */
#define E1000_RDH 0x02810   /* RX Descriptor Head - RW */
#define E1000_RDT 0x02818   /* RX Descriptor Tail - RW */

/* Receive Address Registers */
#define E1000_RAL0 0x05400
#define E1000_RAH0 0x05404
#define E1000_RAH_AV 0x80000000 /* Receive Address Valid */

/* Transmit Rings */
#define E1000_TDBAL 0x03800 /* TX Descriptor Base Address Low - RW */
#define E1000_TDBAH 0x03804 /* TX Descriptor Base Address High - RW */
#define E1000_TDLEN 0x03808 /* TX Descriptor Length - RW */
#define E1000_TDH 0x03810   /* TX Descriptor Head - RW */
#define E1000_TDT 0x03818   /* TX Descriptor Tail - RW */

/* Helper Macros for Register Access */
#define E1000_WRITE_REG(hw, reg, value) writel((value), ((hw) + (reg)))
#define E1000_READ_REG(hw, reg) readl(((hw) + (reg)))

/* Control Register Bit Definitions */
#define E1000_CTRL_SLU 0x00000040     /* Set Link Up */
#define E1000_CTRL_FRCSPD 0x00000800  /* Force Speed */
#define E1000_CTRL_FRCDPLX 0x00001000 /* Force Duplex */
#define E1000_CTRL_RST 0x04000000     /* Global Reset */

/* RCTL Bit Definitions */
#define E1000_RCTL_EN 0x00000002         /* Enable Receiver */
#define E1000_RCTL_SBP 0x00000004        /* Store Bad Packets */
#define E1000_RCTL_UPE 0x00000008        /* Unicast Promiscuous Enabled */
#define E1000_RCTL_MPE 0x00000010        /* Multicast Promiscuous Enabled */
#define E1000_RCTL_LPE 0x00000020        /* Long Packet Enable */
#define E1000_RCTL_LBM_NO 0x00000000     /* No Loopback mode */
#define E1000_RCTL_RDMTS_HALF 0x00000000 /* RX Desc Min Threshold Size */
#define E1000_RCTL_MO_0 0x00000000       /* Multicast Offset */
#define E1000_RCTL_BAM 0x00008000        /* Broadcast Accept Mode */
#define E1000_RCTL_SZ_2048 0x00000000    /* RX Buffer Size 2048 */
#define E1000_RCTL_SECRC 0x04000000      /* Strip Ethernet CRC */

/* TCTL Bit Definitions */
#define E1000_TCTL_EN 0x00000002   /* Enable Transmit */
#define E1000_TCTL_PSP 0x00000008  /* Pad Short Packets */
#define E1000_TCTL_CT 0x00000ff0   /* Collision Threshold */
#define E1000_TCTL_COLD 0x003ff000 /* Collision Distance */
#define E1000_TCTL_RTLC 0x01000000 /* Re-transmit on Late Collision */

/* Descriptor status bits */
#define E1000_RXD_STAT_DD 0x01        /* Descriptor Done */
#define E1000_RXD_STAT_EOP 0x02       /* End of Packet */
#define E1000_TXD_STAT_DD 0x01        /* Descriptor Done */
#define E1000_TXD_CMD_EOP 0x01000000  /* End of Packet */
#define E1000_TXD_CMD_IFCS 0x02000000 /* Insert FCS (CRC) */
#define E1000_TXD_CMD_RS 0x08000000   /* Report Status */

#define E1000_RX_BUFFER_SIZE 2048
#define E1000_NUM_RX_DESC 64
#define E1000_NUM_TX_DESC 64

/* Legacy RX Descriptor */
struct e1000_rx_desc {
  __le64 buffer_addr; /* Address of the descriptor's data buffer */
  __le16 length;      /* Length of data DMAed into data buffer */
  __le16 csum;        /* Packet checksum */
  u8 status;          /* Descriptor status */
  u8 errors;          /* Descriptor Errors */
  __le16 special;     /* VLAN info */
};

/* Legacy TX Descriptor */
struct e1000_tx_desc {
  __le64 buffer_addr; /* Address of the descriptor's data buffer */
  union {
    __le32 data;
    struct {
      __le16 length; /* Data buffer length */
      u8 cso;        /* Checksum offset */
      u8 cmd;        /* Descriptor control */
    } flags;
  } lower;
  union {
    __le32 data;
    struct {
      u8 status;      /* Descriptor status */
      u8 css;         /* Checksum start */
      __le16 special; /* VLAN info */
    } fields;
  } upper;
};

#endif /* _E1000_SIMPLE_H_ */
