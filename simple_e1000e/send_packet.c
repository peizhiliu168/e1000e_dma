#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#define PAYLOAD_LEN 64
#define DRV_NAME "send_packet"

/* Helper to parse MAC address string "XX:XX:XX..." to bytes */
static int parse_mac(const char *mac_str, unsigned char *mac_out) {
  if (sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac_out[0], &mac_out[1],
             &mac_out[2], &mac_out[3], &mac_out[4], &mac_out[5]) != 6) {
    return -1;
  }
  return 0;
}

int main(int argc, char *argv[]) {
  int sockfd;
  struct ifreq if_idx;
  struct ifreq if_mac;
  struct sockaddr_ll socket_address;
  char ifName[IFNAMSIZ];
  unsigned char target_mac[6];
  char sendbuf[1024];
  struct ethhdr *eh = (struct ethhdr *)sendbuf;
  int tx_len = 0;

  if (argc == 2) {
    /* Default MAC if not provided */
    parse_mac("de:ad:be:ef:ca:fe", target_mac);
  } else if (argc == 3) {
    if (parse_mac(argv[2], target_mac) != 0) {
      fprintf(stderr, "Invalid MAC address format: %s\n", argv[2]);
      return 1;
    }
  } else {
    printf("Usage: %s <interface> [target_mac]\n", argv[0]);
    printf("Example: %s eth0\n", argv[0]);
    return 1;
  }

  strncpy(ifName, argv[1], IFNAMSIZ - 1);

  /* Open RAW socket to send on */
  if ((sockfd = socket(AF_PACKET, SOCK_RAW, IPPROTO_RAW)) == -1) {
    perror("socket");
    return 1;
  }

  /* Get the index of the interface to send on */
  memset(&if_idx, 0, sizeof(struct ifreq));
  strncpy(if_idx.ifr_name, ifName, IFNAMSIZ - 1);
  if (ioctl(sockfd, SIOCGIFINDEX, &if_idx) < 0) {
    perror("SIOCGIFINDEX");
    return 1;
  }

  /* Get the MAC address of the interface to send on */
  memset(&if_mac, 0, sizeof(struct ifreq));
  strncpy(if_mac.ifr_name, ifName, IFNAMSIZ - 1);
  if (ioctl(sockfd, SIOCGIFHWADDR, &if_mac) < 0) {
    perror("SIOCGIFHWADDR");
    return 1;
  }

  /* Construct Ethernet header */
  memset(sendbuf, 0, sizeof(sendbuf));
  /* Destination MAC */
  memcpy(eh->h_dest, target_mac, 6);
  /* Source MAC (our interface) */
  memcpy(eh->h_source, if_mac.ifr_hwaddr.sa_data, 6);
  /* Ethertype (0x0800 for IP, but we are just sending raw payload really) */
  eh->h_proto = htons(ETH_P_IP);
  tx_len += sizeof(struct ethhdr);

  /* Payload */
  const char *msg = "HELLO E1000E FROM C APP";
  int msg_len = strlen(msg);
  memcpy(sendbuf + tx_len, msg, msg_len);
  tx_len += msg_len;

  /* Pad with some A's */
  memset(sendbuf + tx_len, 'A', PAYLOAD_LEN - msg_len);
  tx_len += (PAYLOAD_LEN - msg_len);

  /* Send packet */
  socket_address.sll_ifindex = if_idx.ifr_ifindex;
  socket_address.sll_halen = ETH_ALEN;
  memcpy(socket_address.sll_addr, target_mac, 6);

  printf("Sending packet to %02x:%02x:%02x:%02x:%02x:%02x on interface %s "
         "(index %d)\n",
         target_mac[0], target_mac[1], target_mac[2], target_mac[3],
         target_mac[4], target_mac[5], ifName, if_idx.ifr_ifindex);

  if (sendto(sockfd, sendbuf, tx_len, 0, (struct sockaddr *)&socket_address,
             sizeof(struct sockaddr_ll)) < 0) {
    perror("sendto");
    return 1;
  }

  printf("Packet Sent (%d bytes)!\n", tx_len);

  close(sockfd);
  return 0;
}
