
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
#include "xellico.h"

static volatile bool force_quit;
#define RTE_LOGTYPE_XELLICO RTE_LOGTYPE_USER1
#define NB_MBUF   8192
#define MAX_PKT_BURST 32
#define BURST_TX_DRAIN_US 100 /* TX drain every ~100us */
#define MEMPOOL_CACHE_SIZE 256
static uint16_t nb_rxd = 128;
static uint16_t nb_txd = 512;
static uint32_t l2fwd_enabled_port_mask = 0x3;
static uint32_t l2fwd_dst_ports[RTE_MAX_ETHPORTS];
static unsigned int l2fwd_rx_queue_per_lcore = 1;

#define MAX_RX_QUEUE_PER_LCORE 16
#define MAX_TX_QUEUE_PER_PORT 16
struct lcore_queue_conf
{
	unsigned n_rx_port;
	unsigned rx_port_list[MAX_RX_QUEUE_PER_LCORE];
} __rte_cache_aligned;
struct lcore_queue_conf lcore_queue_conf[RTE_MAX_LCORE];

static struct rte_eth_dev_tx_buffer *tx_buffer[RTE_MAX_ETHPORTS];

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

struct rte_mempool * l2fwd_pktmbuf_pool = NULL;

#define MAX_TIMER_PERIOD 86400 /* 1 day max */
static uint64_t timer_period = 10; /* default period is 10 seconds */

static void
l2fwd_simple_forward(struct rte_mbuf *m, unsigned portid)
{
	unsigned dst_port;
	struct rte_eth_dev_tx_buffer *buffer;
	dst_port = l2fwd_dst_ports[portid];
	buffer = tx_buffer[dst_port];
	rte_eth_tx_buffer(dst_port, 0, buffer, m);
}

/* main processing loop */
static void
l2fwd_main_loop (void)
{
	uint64_t prev_tsc, diff_tsc, cur_tsc;
	unsigned portid, nb_rx;
	struct lcore_queue_conf *qconf;
	struct rte_eth_dev_tx_buffer *buffer;

	prev_tsc = 0;

	unsigned lcore_id = rte_lcore_id ();
	qconf = &lcore_queue_conf[lcore_id];

	if (qconf->n_rx_port == 0)
		{
			RTE_LOG (INFO, XELLICO, "lcore %u has nothing to do\n", lcore_id);
			return;
		}

	RTE_LOG (INFO, XELLICO, "entering main loop on lcore %u\n", lcore_id);

	for (size_t i = 0; i < qconf->n_rx_port; i++)
		{
			portid = qconf->rx_port_list[i];
			RTE_LOG (INFO, XELLICO,
					" -- lcoreid=%u portid=%u\n",
					lcore_id, portid);
		}

	const uint64_t drain_tsc =
		  (rte_get_tsc_hz () + US_PER_S - 1)
			/ US_PER_S * BURST_TX_DRAIN_US;
	while (!force_quit) {

		cur_tsc = rte_rdtsc ();

		/*
		 * TX burst queue drain
		 */
		diff_tsc = cur_tsc - prev_tsc;
		if (unlikely (diff_tsc > drain_tsc)) {

			for (size_t i = 0; i < qconf->n_rx_port; i++) {

				portid = l2fwd_dst_ports[qconf->rx_port_list[i]];
				buffer = tx_buffer[portid];

				rte_eth_tx_buffer_flush (portid, 0, buffer);
			}

			prev_tsc = cur_tsc;
		}

		/*
		 * Read packet from RX queues
		 */
		for (size_t i = 0; i < qconf->n_rx_port; i++) {
			struct rte_mbuf *pkts_burst[MAX_PKT_BURST];

			portid = qconf->rx_port_list[i];
			nb_rx = rte_eth_rx_burst ((uint8_t) portid, 0,
						 pkts_burst, MAX_PKT_BURST);

			for (size_t j = 0; j < nb_rx; j++) {
				struct rte_mbuf *m = pkts_burst[j];
				rte_prefetch0(rte_pktmbuf_mtod (m, void *));
				l2fwd_simple_forward (m, portid);
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
	uint8_t nb_ports;
	uint8_t nb_ports_available;
	uint8_t last_port;
	unsigned nb_ports_in_mask = 0;

	int ret = xellico_boot_dpdk (argc, argv);
	argc -= ret;
	argv += ret;

	force_quit = false;
	signal (SIGINT, signal_handler);
	signal (SIGTERM, signal_handler);

	/* convert to number of cycles */
	timer_period *= rte_get_timer_hz();

	/* create the mbuf pool */
	l2fwd_pktmbuf_pool = rte_pktmbuf_pool_create ("mbuf_pool", NB_MBUF,
		MEMPOOL_CACHE_SIZE, 0, RTE_MBUF_DEFAULT_BUF_SIZE, rte_socket_id ());
	if (l2fwd_pktmbuf_pool == NULL)
		rte_exit (EXIT_FAILURE, "Cannot init mbuf pool\n");

	nb_ports = rte_eth_dev_count ();
	if (nb_ports == 0)
		rte_exit (EXIT_FAILURE, "No Ethernet ports - bye\n");

	/* reset l2fwd_dst_ports */
	for (uint8_t portid = 0; portid < RTE_MAX_ETHPORTS; portid++)
		l2fwd_dst_ports[portid] = 0;
	last_port = 0;

	/*
	 * Each logical core is assigned a dedicated TX queue on each port.
	 */
	for (uint8_t portid = 0; portid < nb_ports; portid++)
		{
			if ((l2fwd_enabled_port_mask & (1 << portid)) == 0)
				continue;

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

	unsigned rx_lcore_id = 0;
	struct lcore_queue_conf *qconf = NULL;

	/* Initialize the port/queue configuration of each logical core */
	for (uint8_t portid = 0; portid < nb_ports; portid++)
		{
			/* skip ports that are not enabled */
			if ((l2fwd_enabled_port_mask & (1 << portid)) == 0)
				continue;

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

	nb_ports_available = nb_ports;

	for (uint8_t portid = 0; portid < nb_ports; portid++)
		{

			if ((l2fwd_enabled_port_mask & (1 << portid)) == 0)
				{
					printf("Skipping disabled port %u\n", (unsigned) portid);
					nb_ports_available--;
					continue;
				}

			printf("Initializing port %u... ", (unsigned) portid);
			fflush(stdout);
			ret = rte_eth_dev_configure (portid, 1, 1, &port_conf);
			if (ret < 0)
				rte_exit (EXIT_FAILURE, "Cannot configure device: err=%d, port=%u\n",
						ret, (unsigned) portid);

			ret = rte_eth_dev_adjust_nb_rx_tx_desc (portid, &nb_rxd, &nb_txd);
			if (ret < 0)
				rte_exit(EXIT_FAILURE,
					 "Cannot adjust number of descriptors: err=%d, port=%u\n",
					 ret, (unsigned) portid);

			/* init one RX queue */
			fflush (stdout);
			ret = rte_eth_rx_queue_setup (portid, 0, nb_rxd,
								 rte_eth_dev_socket_id (portid),
								 NULL, l2fwd_pktmbuf_pool);
			if (ret < 0)
				rte_exit(EXIT_FAILURE, "rte_eth_rx_queue_setup:err=%d, port=%u\n",
						ret, (unsigned) portid);

			/* init one TX queue on each port */
			fflush (stdout);
			ret = rte_eth_tx_queue_setup (portid, 0, nb_txd,
					rte_eth_dev_socket_id (portid), NULL);
			if (ret < 0)
				rte_exit(EXIT_FAILURE, "rte_eth_tx_queue_setup:err=%d, port=%u\n",
					ret, (unsigned) portid);

			/* Initialize TX buffers */
			tx_buffer[portid] = rte_zmalloc_socket("tx_buffer",
					RTE_ETH_TX_BUFFER_SIZE (MAX_PKT_BURST), 0,
					rte_eth_dev_socket_id (portid));
			if (tx_buffer[portid] == NULL)
				rte_exit(EXIT_FAILURE, "Cannot allocate buffer for tx on port %u\n",
						(unsigned) portid);
			rte_eth_tx_buffer_init (tx_buffer[portid], MAX_PKT_BURST);

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

	rte_eal_mp_remote_launch(l2fwd_launch_one_lcore, NULL, CALL_MASTER);
	rte_eal_mp_wait_lcore ();

	for (uint8_t portid = 0; portid < nb_ports; portid++)
		{
			if ((l2fwd_enabled_port_mask & (1 << portid)) == 0)
				continue;
			printf ("Closing port %d...", portid);
			rte_eth_dev_stop (portid);
			rte_eth_dev_close (portid);
			printf (" Done\n");
		}
	printf ("Bye...\n");

	return ret;
}


