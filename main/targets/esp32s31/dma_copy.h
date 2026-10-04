#pragma once
#include "esp_attr.h"
#include "esp_private/gdma.h"
#include "hal/dma_types.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_system.h"

#define S31_DMA_PAYLOAD_BYTES 1344u
#define S31_DMA_HEADROOM 128u
#define S31_DMA_SLOTS 3u
#define S31_DMA_STRIDE (S31_DMA_HEADROOM+S31_DMA_PAYLOAD_BYTES)
#define S31_DMA_STAGE_BYTES (S31_DMA_SLOTS*S31_DMA_STRIDE)

/* Fixed SRAM-to-SRAM transfer, no allocation in the sample path. The caller
 * owns both buffers until completion. RX scatters IQ into packet payloads,
 * leaving room for headers; only one outstanding copy is allowed. */
typedef struct {
    gdma_channel_handle_t tx, rx;
    dma_descriptor_align8_t *tx_desc, *rx_desc;
    bool tx_connected, rx_connected, busy;
    unsigned length, rx_count;
} s31_dma_copy_t;

static void s31_dma_copy_deinit(s31_dma_copy_t *c)
{
    if(c->tx) (void)gdma_stop(c->tx);
    if(c->rx) (void)gdma_stop(c->rx);
    heap_caps_free(c->tx_desc);heap_caps_free(c->rx_desc);
    if(c->tx_connected) (void)gdma_disconnect(c->tx);
    if(c->rx_connected) (void)gdma_disconnect(c->rx);
    if(c->tx) (void)gdma_del_channel(c->tx);
    if(c->rx) (void)gdma_del_channel(c->rx);
    *c=(s31_dma_copy_t){0};
}

static esp_err_t s31_dma_copy_init(s31_dma_copy_t *c)
{
    esp_err_t error;
#define COPY_TRY(x) do { error=(x); if(error!=ESP_OK)goto fail; } while(0)
    gdma_channel_alloc_config_t alloc={0};
    COPY_TRY(gdma_new_axi_channel(&alloc,&c->tx,&c->rx));
    gdma_strategy_config_t strategy={.owner_check=true,.auto_update_desc=true};
    COPY_TRY(gdma_apply_strategy(c->tx,&strategy));
    COPY_TRY(gdma_apply_strategy(c->rx,&strategy));
    gdma_transfer_config_t transfer={.max_data_burst_size=64};
    COPY_TRY(gdma_config_transfer(c->tx,&transfer));
    COPY_TRY(gdma_config_transfer(c->rx,&transfer));
    uint32_t mask=0;
    COPY_TRY(gdma_get_free_m2m_trig_id_mask(c->tx,&mask));
    if(!mask) {error=ESP_ERR_NOT_FOUND;goto fail;}
    gdma_trigger_t trigger=GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_M2M,0);
    trigger.instance_id=__builtin_ctz(mask);
    COPY_TRY(gdma_connect(c->tx,trigger));c->tx_connected=true;
    COPY_TRY(gdma_connect(c->rx,trigger));c->rx_connected=true;
    COPY_TRY(gdma_reset(c->tx));COPY_TRY(gdma_reset(c->rx));
    c->tx_desc=heap_caps_aligned_calloc(8,1,sizeof(dma_descriptor_align8_t),MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA);
    c->rx_desc=heap_caps_aligned_calloc(8,S31_DMA_SLOTS,sizeof(dma_descriptor_align8_t),MALLOC_CAP_INTERNAL|MALLOC_CAP_DMA);
    if(!c->tx_desc || !c->rx_desc) {error=ESP_ERR_NO_MEM;goto fail;}
    return ESP_OK;
fail:
    s31_dma_copy_deinit(c);return error;
#undef COPY_TRY
}

static esp_err_t IRAM_ATTR s31_dma_copy_start(s31_dma_copy_t *c,void *dst,const void *src,unsigned length)
{
    if(c->busy || !length || length>S31_DMA_SLOTS*S31_DMA_PAYLOAD_BYTES || ((uintptr_t)src&3u) || ((uintptr_t)dst&3u))
        return ESP_ERR_INVALID_ARG;
    /* A completed, NULL-terminated transfer can be restarted with a new head,
     * as in IDF's async memcpy driver. Reset once during initialization only. */
    *c->tx_desc=(dma_descriptor_align8_t){.dw0={.size=length,.length=length,.suc_eof=1,.owner=1},.buffer=(void *)src};
    c->rx_count=(length+S31_DMA_PAYLOAD_BYTES-1)/S31_DMA_PAYLOAD_BYTES;
    for(unsigned i=0,left=length;i<c->rx_count;i++) {
        unsigned size=left>S31_DMA_PAYLOAD_BYTES?S31_DMA_PAYLOAD_BYTES:left;
        c->rx_desc[i]=(dma_descriptor_align8_t){.dw0={.size=size,.owner=1},
            .buffer=(uint8_t *)dst+i*S31_DMA_STRIDE+S31_DMA_HEADROOM,
            .next=i+1<c->rx_count?&c->rx_desc[i+1]:NULL};
        left-=size;
    }
    __sync_synchronize();
    ESP_ERROR_CHECK(gdma_start(c->rx,(intptr_t)c->rx_desc));
    ESP_ERROR_CHECK(gdma_start(c->tx,(intptr_t)c->tx_desc));
    c->length=length;c->busy=true;
    return ESP_OK;
}

static esp_err_t IRAM_ATTR s31_dma_copy_wait(s31_dma_copy_t *c)
{
    if(!c->busy)return ESP_OK;
    int64_t deadline=esp_timer_get_time()+100000;
    volatile dma_descriptor_align8_t *tx=c->tx_desc,*rx=c->rx_desc;
    for(;;) {
        bool pending=tx->dw0.owner;
        for(unsigned i=0;i<c->rx_count;i++)pending|=rx[i].dw0.owner;
        if(!pending)break;
        if(esp_timer_get_time()>deadline)esp_system_abort("S31 SRAM copy DMA timeout");
    }
    __sync_synchronize();
    c->busy=false;
    unsigned length=0;
    for(unsigned i=0;i<c->rx_count;i++) {
        if(rx[i].dw0.err_eof)return ESP_FAIL;
        length+=rx[i].dw0.length;
    }
    return length!=c->length ? ESP_FAIL : ESP_OK;
}

static esp_err_t s31_dma_copy(s31_dma_copy_t *c,void *dst,const void *src,unsigned length)
{
    esp_err_t error=s31_dma_copy_start(c,dst,src,length);
    return error==ESP_OK ? s31_dma_copy_wait(c) : error;
}
