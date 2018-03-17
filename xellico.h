#ifndef __XELLICO_H_
#define __XELLICO_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/queue.h>
#include <netinet/in.h>
#include <setjmp.h>
#include <stdarg.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>

#include <rte_common.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_eal.h>
#include <rte_launch.h>
#include <rte_atomic.h>
#include <rte_cycles.h>
#include <rte_prefetch.h>
#include <rte_lcore.h>
#include <rte_per_lcore.h>
#include <rte_branch_prediction.h>
#include <rte_interrupts.h>
#include <rte_pci.h>
#include <rte_random.h>
#include <rte_debug.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>

static inline int
xellico_boot_dpdk (int argc, char** argv)
{
  int ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid EAL arguments\n");
  return ret;
}

static inline size_t
rte_socket_count (void)
{
  const size_t rte_max_socket = 128;
  uint8_t socket_enable[rte_max_socket];
  memset (socket_enable, 0x0, sizeof(socket_enable));

  for (size_t i=0; i<RTE_MAX_LCORE; i++) {
    if (rte_lcore_is_enabled (i))
      {
        uint8_t socket_id = rte_lcore_to_socket_id(i);
        socket_enable[socket_id] = 1;
      }
  }

  size_t socket_count = 0;
  for (size_t i=0; i<rte_max_socket; i++)
    socket_count += socket_enable[i];
  return socket_count;
}

#endif /* __XELLICO_H_ */

