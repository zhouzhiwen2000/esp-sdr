/* Included after the pinned IDF DMA implementation. All entry points execute
 * under esp_eth's TX mutex. Only this board's single MAC is supported.
 * Complete SRAM frames are loaned until an explicit drain. The optional
 * contiguous-IQ fallback copies headers and loans payloads separately.
 * Ordinary lwIP traffic reclaims completed loans before touching driver buffers.
 */
#include "esp_attr.h"
#include "esp_system.h"
#include "esp_timer.h"

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#error "S31 direct TX requires uncached internal SRAM"
#endif
#ifndef CONFIG_ETH_TRANSMIT_MUTEX
#error "S31 direct TX requires the public driver transmit mutex"
#endif
const uint32_t s31_emac_direct_token = 0x53333144;
const uint32_t s31_emac_frame_token = 0x53333150;
const uint32_t s31_emac_drain_token = 0x53333146;
static struct {
    eth_dma_tx_descriptor_t *desc;
    uint32_t original_address;
} s31_loans[CONFIG_ETH_DMA_TX_BUFFER_NUM];

static unsigned IRAM_ATTR s31_reclaim_loans(void)
{
    unsigned pending = 0;
    for (unsigned i = 0; i < CONFIG_ETH_DMA_TX_BUFFER_NUM; ++i) {
        eth_dma_tx_descriptor_t *d = s31_loans[i].desc;
        if (!d) continue;
        if (d->TDES0.Own == EMAC_LL_DMADESC_OWNER_CPU) {
            __sync_synchronize();
            d->Buffer1Addr = s31_loans[i].original_address;
            s31_loans[i].desc = NULL;
        } else ++pending;
    }
    return pending;
}

uint32_t IRAM_ATTR emac_esp_dma_transmit_frame(emac_esp_dma_handle_t dma,
                                             uint8_t *buf, uint32_t length)
{
    s31_reclaim_loans();
    return s31_emac_transmit_frame_copy(dma, buf, length);
}

uint32_t IRAM_ATTR emac_esp_dma_transmit_frame_ext(emac_esp_dma_handle_t dma,
        emac_esp_dma_transmit_buff_t *buffers, uint32_t count, eth_mac_time_t *ts)
{
    if ((const void *)ts == (const void *)&s31_emac_drain_token) {
        int64_t deadline = esp_timer_get_time() + 100000;
        for (;;) {
            s31_reclaim_loans();
            bool pending = false;
            uintptr_t begin = (uintptr_t)buffers[0].buf;
            uintptr_t end = begin + buffers[0].size;
            for (unsigned i = 0; i < CONFIG_ETH_DMA_TX_BUFFER_NUM; ++i) {
                eth_dma_tx_descriptor_t *d = s31_loans[i].desc;
                if (d && d->Buffer1Addr >= begin && d->Buffer1Addr < end) pending = true;
            }
            if (!pending) break;
            /* Never return borrowed memory to its producer while DMA owns it. */
            if (esp_timer_get_time() > deadline)
                esp_system_abort("S31 TX DMA did not release borrowed buffers");
        }
        uint32_t length = 0;
        for (unsigned i = 0; i < count; ++i) length += buffers[i].size;
        return length;
    }
    s31_reclaim_loans();
    if ((const void *)ts == (const void *)&s31_emac_frame_token) {
        if(count!=1 || !buffers[0].size || buffers[0].size>1536)return 0;
        eth_dma_tx_descriptor_t *d=dma->tx_desc;
        if(d->TDES0.Own)return 0;
        unsigned slot=0;
        while(slot<CONFIG_ETH_DMA_TX_BUFFER_NUM && s31_loans[slot].desc)++slot;
        if(slot==CONFIG_ETH_DMA_TX_BUFFER_NUM)return 0;
        s31_loans[slot].desc=d;s31_loans[slot].original_address=d->Buffer1Addr;
        d->Buffer1Addr=(uint32_t)buffers[0].buf;
        d->TDES0.Value=0;d->TDES0.SecondAddressChained=1;
        d->TDES0.FirstSegment=1;d->TDES0.LastSegment=1;
        d->TDES0.Value|=dma->tx_desc_flags & (EMAC_TDES0_FS_CTRL_FLAGS_MASK|EMAC_TDES0_LS_CTRL_FLAGS_MASK);
        d->TDES1.TransmitBuffer1Size=buffers[0].size;
        __sync_synchronize();
        d->TDES0.Own=EMAC_LL_DMADESC_OWNER_DMA;
        __sync_synchronize();
        dma->tx_desc=(eth_dma_tx_descriptor_t *)d->Buffer2NextDescAddr;
        emac_hal_transmit_poll_demand(&dma->hal);
        return buffers[0].size;
    }
    if ((const void *)ts != (const void *)&s31_emac_direct_token)
        return s31_emac_transmit_frame_ext_copy(dma, buffers, count, ts);

    _Static_assert(CONFIG_ETH_DMA_TX_BUFFER_NUM >= 6, "six TX descriptors required");
    if (count != 2 || !buffers[0].size || !buffers[1].size ||
            buffers[0].size > CONFIG_ETH_DMA_BUFFER_SIZE || buffers[1].size > 1536)
        return 0;
    eth_dma_tx_descriptor_t *desc[2] = {dma->tx_desc,
        (eth_dma_tx_descriptor_t *)dma->tx_desc->Buffer2NextDescAddr};
    if (desc[0]->TDES0.Own || desc[1]->TDES0.Own) return 0;
    unsigned slot = 0;
    while (slot < CONFIG_ETH_DMA_TX_BUFFER_NUM && s31_loans[slot].desc) ++slot;
    if (slot == CONFIG_ETH_DMA_TX_BUFFER_NUM) return 0;
    s31_loans[slot].desc = desc[1];
    s31_loans[slot].original_address = desc[1]->Buffer1Addr;
    memcpy((void *)desc[0]->Buffer1Addr, buffers[0].buf, buffers[0].size);
    desc[1]->Buffer1Addr = (uint32_t)buffers[1].buf;
    for (unsigned i = 0; i < 2; ++i) {
        desc[i]->TDES0.Value = 0;
        desc[i]->TDES0.SecondAddressChained = 1;
        desc[i]->TDES0.FirstSegment = i == 0;
        desc[i]->TDES0.LastSegment = i == 1;
        desc[i]->TDES0.Value |= dma->tx_desc_flags & (i == 0 ?
            EMAC_TDES0_FS_CTRL_FLAGS_MASK : EMAC_TDES0_LS_CTRL_FLAGS_MASK);
        desc[i]->TDES1.TransmitBuffer1Size = buffers[i].size;
    }
    __sync_synchronize();
    desc[1]->TDES0.Own = EMAC_LL_DMADESC_OWNER_DMA;
    __sync_synchronize();
    desc[0]->TDES0.Own = EMAC_LL_DMADESC_OWNER_DMA;
    __sync_synchronize();
    dma->tx_desc = (eth_dma_tx_descriptor_t *)desc[1]->Buffer2NextDescAddr;
    emac_hal_transmit_poll_demand(&dma->hal);
    return buffers[0].size + buffers[1].size;
}
