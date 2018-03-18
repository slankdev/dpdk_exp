
#pragma once

static inline void
rte_pktmbuf_free_bulk(struct rte_mbuf* m_list[], size_t npkts)
{
  while (npkts--)
    rte_pktmbuf_free(*m_list++);
}

static inline void
dump_mempool (struct rte_mempool* mp)
{
  printf ("name     : %s \n", mp->name);
  printf ("socket_id: %u \n", mp->socket_id);
  printf ("size     : %u (using %.0f%%) \n", mp->size,
      rte_mempool_in_use_count (mp) / (float) (mp->size) * 100);
  printf ("in-use   : %u \n", rte_mempool_in_use_count (mp));
  printf ("avail    : %u \n", rte_mempool_avail_count (mp));
}
