#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

/* Standard page size */
#define PAGE_SIZE 4096

uintptr_t client_virt_to_phys(void *addr) {
  int fd = open("/proc/self/pagemap", O_RDONLY);
  if (fd < 0) {
    perror("open pagemap");
    return 0;
  }

  uintptr_t virt = (uintptr_t)addr;
  off_t offset = (virt / PAGE_SIZE) * 8;
  uint64_t entry;

  if (lseek(fd, offset, SEEK_SET) < 0) {
    perror("lseek");
    close(fd);
    return 0;
  }

  if (read(fd, &entry, 8) != 8) {
    perror("read");
    close(fd);
    return 0;
  }

  close(fd);

  /* Bit 63 indicates if the page is present in RAM */
  if (!(entry & (1ULL << 63))) {
    fprintf(stderr, "Page not present (swapped out or not allocated)\n");
    return 0;
  }

  /* Bits 0-54 are the Page Frame Number (PFN) */
  uintptr_t pfn = entry & ((1ULL << 55) - 1);
  return (pfn * PAGE_SIZE) + (virt % PAGE_SIZE);
}

int main(int argc, char *argv[]) {
  void *buf;
  /* Allocate aligned memory */
  if (posix_memalign(&buf, PAGE_SIZE, PAGE_SIZE)) {
    perror("posix_memalign");
    return 1;
  }

  /* Lock memory to prevent swapping and ensure it stays in RAM */
  if (mlock(buf, PAGE_SIZE)) {
    perror("mlock");
    fprintf(stderr, "Try running as root to allow mlock.\n");
    return 1;
  }

  /* Touch the memory to ensure it's faulted in */
  memset(buf, 0xEE, PAGE_SIZE);

  uintptr_t phys = client_virt_to_phys(buf);
  if (!phys)
    return 1;

  printf("Allocated 4KB buffer.\n");
  printf("Virtual Addr:  %p\n", buf);
  printf("Physical Addr: 0x%lx\n", phys);
  printf("\n");
  printf("Changes to this buffer will be visible here.\n");
  printf("Usage: sudo ./set_rx_addr.sh 0x%lx\n", phys);
  printf("\nPress Enter to exit (and free memory)...\n");
  getchar();

  return 0;
}
