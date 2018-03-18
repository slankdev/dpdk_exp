
/*
 * The use of this software is limited to education, research, and evaluation
 * purposes only.  Commercial use is strictly prohibited.  For all other uses,
 * contact the author(s).
 * Copyright(c) 2018 Souta Kawahara
 * Copyright(c) 2018 Hiroki Shirokura
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include "xellico.h"
#include "dpdk_misc.h"

#define RTE_LOGTYPE_XELLICO RTE_LOGTYPE_USER1
#define NB_MBUF   81920
#define MAX_PKT_BURST 32
#define BURST_TX_DRAIN_US 100 /* TX drain every ~100us */
#define MEMPOOL_CACHE_SIZE 256
#define MAX_RX_QUEUE_PER_LCORE 16

static volatile bool force_quit;
static uint32_t l2fwd_dst_ports[RTE_MAX_ETHPORTS];

struct lcore_queue_conf
{
  uint32_t n_rx_port;
  uint32_t rx_port_list[MAX_RX_QUEUE_PER_LCORE];
  uint32_t rx_queue_list[MAX_RX_QUEUE_PER_LCORE];
  struct rte_eth_dev_tx_buffer *tx_buffer[RTE_MAX_ETHPORTS];
} __rte_cache_aligned;

struct lcore_queue_conf lcore_queue_conf[RTE_MAX_LCORE];

static const struct rte_eth_conf port_conf = {
  .rxmode = {
    .split_hdr_size = 0,
    .header_split   = 0, /**< Header Split disabled */
    .hw_ip_checksum = 0, /**< IP checksum offload disabled */
    .hw_vlan_filter = 0, /**< VLAN filtering disabled */
    .jumbo_frame    = 0, /**< Jumbo Frame Support disabled */
    .hw_strip_crc   = 1, /**< CRC stripped by hardware */
    .mq_mode = ETH_MQ_RX_RSS,
  },
  .txmode = {
    .mq_mode = ETH_MQ_TX_NONE,
  },
  .rx_adv_conf = {
    .rss_conf = {
      .rss_key = NULL,
      .rss_hf = ETH_RSS_IP|ETH_RSS_TCP|ETH_RSS_UDP,
    },
  },
};

struct rte_mempool* pktmbuf_pool[RTE_MAX_LCORE];

static void
dump_queue_conf (struct lcore_queue_conf* qconf)
{
  printf ("qconf: n_rx_port=%u \n", qconf->n_rx_port);
  for (size_t j=0; j<qconf->n_rx_port; j++)
  printf ("   port%u queue%u\n",
      qconf->rx_port_list[j],
      qconf->rx_queue_list[j]);
}

static void
dump_queue_confs (struct lcore_queue_conf* qconfs, size_t n_qconfs)
{
  for (size_t i=0; i<n_qconfs; i++)
    {
      struct lcore_queue_conf* qconf = (qconfs + i);
      dump_queue_conf (qconf);
    }
}

static void
init_qconf_buffer (void)
{
  const size_t nb_ports = rte_eth_dev_count();
  for (size_t i=0; i<RTE_MAX_LCORE; i++)
    {
      for (size_t portid=0; portid<nb_ports; portid++)
        {
          struct rte_eth_dev_tx_buffer* txbuff =
            lcore_queue_conf[i].tx_buffer[portid];
          rte_eth_tx_buffer_init (txbuff, MAX_PKT_BURST);
        }
    }
}

static void
init_queue_conf (void)
{
  const size_t nb_ports = rte_eth_dev_count();
  for (size_t i=0; i<RTE_MAX_LCORE; i++)
    {
      for (size_t portid=0; portid<nb_ports; portid++)
        {
          struct rte_eth_dev_tx_buffer* txbuff = rte_zmalloc_socket ("tx_buffer",
              RTE_ETH_TX_BUFFER_SIZE (MAX_PKT_BURST), 0,
              rte_eth_dev_socket_id (portid));
          if (txbuff == NULL)
            rte_exit(EXIT_FAILURE, "Cannot allocate buffer for tx on port %u\n",
                (unsigned) portid);
          lcore_queue_conf[i].tx_buffer[portid] = txbuff;
          /* rte_eth_tx_buffer_init (txbuff, MAX_PKT_BURST); */
        }
    }
#if 0
  const unsigned int l2fwd_rx_queue_per_lcore = 1;
  unsigned rx_lcore_id = 0;
  struct lcore_queue_conf *qconf = NULL;
  uint8_t nb_ports = rte_eth_dev_count ();
  /* Initialize the port/queue configuration of each logical core */
  for (uint8_t portid = 0; portid < nb_ports; portid++)
    {
      /* get the lcore_id for this port */
      while (rte_lcore_is_enabled (rx_lcore_id) == 0 ||
             lcore_queue_conf[rx_lcore_id].n_rx_port ==
             l2fwd_rx_queue_per_lcore)
        {
          rx_lcore_id++;
          if (rx_lcore_id >= RTE_MAX_LCORE)
            rte_exit(EXIT_FAILURE, "Not enough cores\n");
        }

      if (qconf != &lcore_queue_conf[rx_lcore_id])
        /* Assigned a new logical core in the loop above. */
        qconf = &lcore_queue_conf[rx_lcore_id];

      qconf->rx_port_list[qconf->n_rx_port] = portid;
      qconf->n_rx_port++;
      printf("Lcore %u: RX port %u\n", rx_lcore_id, (unsigned) portid);
    }
#else
  lcore_queue_conf[0].n_rx_port = 1;
  lcore_queue_conf[0].rx_port_list[0] = 0;
  lcore_queue_conf[0].rx_queue_list[0] = 0;

  lcore_queue_conf[1].n_rx_port = 1;
  lcore_queue_conf[1].rx_port_list[0] = 1;
  lcore_queue_conf[1].rx_queue_list[0] = 0;

  lcore_queue_conf[2].n_rx_port = 1;
  lcore_queue_conf[2].rx_port_list[0] = 0;
  lcore_queue_conf[2].rx_queue_list[0] = 1;

  lcore_queue_conf[3].n_rx_port = 1;
  lcore_queue_conf[3].rx_port_list[0] = 1;
  lcore_queue_conf[3].rx_queue_list[0] = 1;
#endif
}

static inline void
l2fwd_simple_forward (struct rte_mbuf *m, unsigned portid)
{
  uint32_t lcore_id = rte_lcore_id();
  struct lcore_queue_conf* qconf = &lcore_queue_conf[lcore_id];
  unsigned dst_port = l2fwd_dst_ports[portid];
  unsigned dst_queue = lcore_id;
  struct rte_eth_dev_tx_buffer* buffer = qconf->tx_buffer[dst_port];
  rte_eth_tx_buffer(dst_port, dst_queue, buffer, m);
}

/* main processing loop */
static void
l2fwd_main_loop (void)
{
  unsigned lcore_id = rte_lcore_id ();
  struct lcore_queue_conf *qconf = &lcore_queue_conf[lcore_id];

  if (qconf->n_rx_port == 0)
    {
      RTE_LOG (INFO, XELLICO, "lcore %u has nothing to do\n", lcore_id);
      if (lcore_id == 31)
        while (!force_quit)
          {
            dump_mempool(pktmbuf_pool[0]);
            printf("---\n");
            dump_mempool(pktmbuf_pool[1]);
            printf("==============\n");
            sleep(1);
          }
      return;
    }

  RTE_LOG (INFO, XELLICO, "entering main loop on lcore %u\n", lcore_id);

  for (size_t i = 0; i < qconf->n_rx_port; i++)
    {
      unsigned portid = qconf->rx_port_list[i];
      unsigned queueid = qconf->rx_queue_list[i];
      RTE_LOG (INFO, XELLICO,
          " -- lcoreid=%u portid=%u queueid=%u\n",
          lcore_id, portid, queueid);
    }

  uint64_t prev_tsc = 0;
  const uint64_t drain_tsc =
      (rte_get_tsc_hz () + US_PER_S - 1)
      / US_PER_S * BURST_TX_DRAIN_US;
  printf ("Staring Main_loop on lcore%u \n", rte_lcore_id());
  dump_queue_conf (qconf);
  while (!force_quit)
    {
      /*
       * TX burst queue drain
       */
      uint64_t cur_tsc = rte_rdtsc ();
      uint64_t diff_tsc = cur_tsc - prev_tsc;
      if (unlikely (diff_tsc > drain_tsc))
        {
          for (size_t i = 0; i < qconf->n_rx_port; i++)
            {
              uint32_t dst_portid = l2fwd_dst_ports[qconf->rx_port_list[i]];
              uint32_t dst_queueid = rte_lcore_id();
              struct rte_eth_dev_tx_buffer *buffer = qconf->tx_buffer[dst_portid];
              rte_eth_tx_buffer_flush (dst_portid, dst_queueid, buffer);
            }
          prev_tsc = cur_tsc;
        }

      /*
       * Read packet from RX queues
       */
      for (size_t i = 0; i < qconf->n_rx_port; i++)
        {
          struct rte_mbuf *pkts_burst[MAX_PKT_BURST];

          uint32_t in_portid = qconf->rx_port_list[i];
          uint32_t in_queueid = qconf->rx_queue_list[i];
          unsigned nb_rx = rte_eth_rx_burst ((uint8_t) in_portid,
              in_queueid, pkts_burst, MAX_PKT_BURST);

          for (size_t j = 0; j < nb_rx; j++)
            {
              struct rte_mbuf *m = pkts_burst[j];

#if 1 // DELAY
              size_t nd = 0;
              for (size_t d=0; d<100; d++) nd += d;
#endif

              rte_prefetch0 (rte_pktmbuf_mtod (m, void *));
              l2fwd_simple_forward (m, in_portid);
            }
        }
    }
}

static int
l2fwd_launch_one_lcore (__attribute__ ((unused)) void *dummy)
{
  l2fwd_main_loop ();
  return 0;
}

static void
signal_handler (int signum)
{
  if (signum == SIGINT || signum == SIGTERM) {
    printf("\n\nSignal %d received, preparing to exit...\n",
        signum);
    force_quit = true;
  }
}

int
main (int argc, char **argv)
{
  int ret = xellico_boot_dpdk (argc, argv);
  argc -= ret;
  argv += ret;

  force_quit = false;
  signal (SIGINT, signal_handler);
  signal (SIGTERM, signal_handler);

  /* create the mbuf pool */
  for (size_t i=0; i<rte_socket_count(); i++)
    {
      char str[128];
      snprintf (str, sizeof (str), "mbuf_pool[%zd]", i);
      pktmbuf_pool[i] = rte_pktmbuf_pool_create (str, NB_MBUF,
        MEMPOOL_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, i);
      if (pktmbuf_pool[i] == NULL)
        rte_exit (EXIT_FAILURE, "Cannot init mbuf pool %s\n", str);
      RTE_LOG (INFO, XELLICO, "create mempool %s\n", str);
    }

  uint8_t nb_ports = rte_eth_dev_count ();
  if (nb_ports == 0)
    rte_exit (EXIT_FAILURE, "No Ethernet ports - bye\n");

  /* reset l2fwd_dst_ports */
  for (uint8_t portid = 0; portid < RTE_MAX_ETHPORTS; portid++)
    l2fwd_dst_ports[portid] = 0;

  /*
   * Each logical core is assigned a dedicated TX queue on each port.
   */
  uint8_t last_port = 0;
  unsigned nb_ports_in_mask = 0;
  for (uint8_t portid = 0; portid < nb_ports; portid++)
    {
      if (nb_ports_in_mask % 2)
        {
          l2fwd_dst_ports[portid] = last_port;
          l2fwd_dst_ports[last_port] = portid;
        }
      else
        last_port = portid;

      nb_ports_in_mask++;
    }
  if (nb_ports_in_mask % 2)
    {
      printf("Notice: odd number of ports in portmask.\n");
      l2fwd_dst_ports[last_port] = last_port;
    }

  init_queue_conf();
  dump_queue_confs(lcore_queue_conf, 40);

  uint8_t nb_ports_available = nb_ports;
  for (uint8_t portid = 0; portid < nb_ports; portid++)
    {
      const size_t nb_rxq = 2;
      const size_t nb_txq = rte_lcore_count();
      printf("Initializing port %u... \n", (unsigned) portid);
      ret = rte_eth_dev_configure (portid, nb_rxq, nb_txq, &port_conf);
      if (ret < 0)
        rte_exit (EXIT_FAILURE, "Cannot configure device: err=%d, port=%u\n",
            ret, (unsigned) portid);

      uint16_t nb_rxd = 128;
      uint16_t nb_txd = 512;
      ret = rte_eth_dev_adjust_nb_rx_tx_desc (portid, &nb_rxd, &nb_txd);
      if (ret < 0)
        rte_exit(EXIT_FAILURE,
           "Cannot adjust number of descriptors: err=%d, port=%u\n",
           ret, (unsigned) portid);

      /* init one RX queue */
      uint8_t port_socket_id = rte_eth_dev_socket_id(portid);
      for (uint32_t q=0; q<nb_rxq; q++)
        {
          ret = rte_eth_rx_queue_setup (portid, q, nb_rxd,
                     rte_eth_dev_socket_id (portid),
                     NULL, pktmbuf_pool[port_socket_id]);
          if (ret < 0)
            rte_exit(EXIT_FAILURE,
                "rte_eth_rx_queue_setup:err=%d, port=%u, queue=%u\n",
                ret, (unsigned) portid, q);
        }

      /* init one TX queue on each port */
      for (uint32_t q=0; q<nb_txq; q++)
        {
          ret = rte_eth_tx_queue_setup (portid, q, nb_txd,
              rte_eth_dev_socket_id (portid), NULL);
          if (ret < 0)
            rte_exit(EXIT_FAILURE,
                "rte_eth_tx_queue_setup:err=%d, port=%u, queue=%u\n",
                ret, (unsigned) portid, q);
        }
    }

  init_qconf_buffer();

  for (uint8_t portid = 0; portid < nb_ports; portid++)
    {
      printf("slankdev portid=%u", portid);
      ret = rte_eth_dev_start (portid);
      if (ret < 0)
        rte_exit(EXIT_FAILURE, "rte_eth_dev_start:err=%d, port=%u\n",
            ret, (unsigned) portid);

      printf("done: \n");
      rte_eth_promiscuous_enable (portid);
    }

  if (!nb_ports_available)
    {
      rte_exit(EXIT_FAILURE,
        "All available ports are disabled. Please set portmask.\n");
    }

  rte_eal_mp_remote_launch (l2fwd_launch_one_lcore, NULL, CALL_MASTER);
  rte_eal_mp_wait_lcore ();

  for (uint8_t portid = 0; portid < nb_ports; portid++)
    {
      printf ("Closing port %d...", portid);
      rte_eth_dev_stop (portid);
      rte_eth_dev_close (portid);
      printf (" Done\n");
    }
  printf ("Bye...\n");

  return ret;
}


