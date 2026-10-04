"""Exercise MAC buffer loans independently of the physical DMA engine."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define IRAM_ATTR
#define CONFIG_ETH_TRANSMIT_MUTEX 1
#define CONFIG_ETH_DMA_TX_BUFFER_NUM 8
#define CONFIG_ETH_DMA_BUFFER_SIZE 1536
#define EMAC_LL_DMADESC_OWNER_CPU 0
#define EMAC_LL_DMADESC_OWNER_DMA 1
#define EMAC_TDES0_FS_CTRL_FLAGS_MASK 0x0FCC0000
#define EMAC_TDES0_LS_CTRL_FLAGS_MASK 0x40000000
/* Match the ownership, chaining, first/last descriptor bit positions. */
typedef struct desc {
    volatile union { uint32_t Value; struct {
        uint32_t pad0:20, SecondAddressChained:1, pad1:7;
        uint32_t FirstSegment:1, LastSegment:1, InterruptOnComplete:1, Own:1;
    }; } TDES0;
    struct { uint32_t TransmitBuffer1Size; } TDES1;
    uint32_t Buffer1Addr, Buffer2NextDescAddr;
} eth_dma_tx_descriptor_t;
typedef struct { int hal; uint32_t tx_desc_flags; eth_dma_tx_descriptor_t *tx_desc; } dma_t;
typedef dma_t *emac_esp_dma_handle_t;
typedef struct { uint8_t *buf; uint32_t size; } emac_esp_dma_transmit_buff_t;
typedef struct { uint32_t seconds, nanoseconds; } eth_mac_time_t;
static eth_dma_tx_descriptor_t descriptors[8];
static uint8_t stock[8][1536], header[90], payload_a[4032], payload_b[4032], unrelated[16];
static int64_t esp_timer_get_time(void) { static int64_t tick; return ++tick; }
static void esp_system_abort(const char *s) { (void)s; exit(77); }
static void emac_hal_transmit_poll_demand(int *hal) { (void)hal; }
static uint32_t s31_emac_transmit_frame_copy(dma_t *dma, uint8_t *p, uint32_t n) {
    eth_dma_tx_descriptor_t *d=dma->tx_desc;
    assert(!d->TDES0.Own);
    assert(d->Buffer1Addr==(uintptr_t)stock[d-descriptors]);
    memcpy((void *)(uintptr_t)d->Buffer1Addr,p,n);
    dma->tx_desc=(void *)(uintptr_t)d->Buffer2NextDescAddr;
    return n;
}
static uint32_t s31_emac_transmit_frame_ext_copy(dma_t *dma,
        emac_esp_dma_transmit_buff_t *b,uint32_t n,eth_mac_time_t *ts) {
    (void)ts;assert(n==1);return s31_emac_transmit_frame_copy(dma,b[0].buf,b[0].size);
}
#include "@ADAPTER@"
static void complete_all(void) { for(unsigned i=0;i<8;i++)descriptors[i].TDES0.Own=0; }
static void drain(dma_t *dma,uint8_t *p,uint32_t n) {
    emac_esp_dma_transmit_buff_t b={p,n};
    assert(emac_esp_dma_transmit_frame_ext(dma,&b,1,(void *)&s31_emac_drain_token)==n);
}
static void submit(dma_t *dma,uint8_t *p) {
    eth_dma_tx_descriptor_t *first=dma->tx_desc;
    eth_dma_tx_descriptor_t *last=(void *)(uintptr_t)first->Buffer2NextDescAddr;
    emac_esp_dma_transmit_buff_t b[2]={{header,sizeof(header)},{p,1344}};
    assert(emac_esp_dma_transmit_frame_ext(dma,b,2,(void *)&s31_emac_direct_token)==1434);
    assert(first->TDES0.Own && last->TDES0.Own);
    assert(first->TDES0.FirstSegment && last->TDES0.LastSegment);
    assert(last->Buffer1Addr==(uintptr_t)p);
    assert(memcmp((void *)(uintptr_t)first->Buffer1Addr,header,sizeof(header))==0);
}
static void submit_frame(dma_t *dma,uint8_t *p) {
    eth_dma_tx_descriptor_t *d=dma->tx_desc;
    emac_esp_dma_transmit_buff_t b={p,1430};
    assert(emac_esp_dma_transmit_frame_ext(dma,&b,1,(void *)&s31_emac_frame_token)==1430);
    assert(d->TDES0.Own && d->TDES0.FirstSegment && d->TDES0.LastSegment);
    assert(d->Buffer1Addr==(uintptr_t)p);
    assert(d->TDES1.TransmitBuffer1Size==1430);
}
int main(int argc,char **argv) {
    (void)argv;
    for(unsigned i=0;i<8;i++) {
        descriptors[i].Buffer1Addr=(uintptr_t)stock[i];
        descriptors[i].Buffer2NextDescAddr=(uintptr_t)&descriptors[(i+1)%8];
    }
    memset(header,0x39,sizeof(header));
    dma_t dma={.tx_desc=descriptors};
    if(argc>2) {submit_frame(&dma,payload_a+42);drain(&dma,payload_a,sizeof(payload_a));return 1;}
    submit(&dma,payload_a);
    if(argc>1) {drain(&dma,payload_a,sizeof(payload_a));return 1;} /* must fail closed */
    submit(&dma,payload_b);
    drain(&dma,unrelated,sizeof(unrelated)); /* unrelated live loan must not block */
    descriptors[0].TDES0.Own=descriptors[1].TDES0.Own=0;
    drain(&dma,payload_a,sizeof(payload_a));
    assert(descriptors[1].Buffer1Addr==(uintptr_t)stock[1]);
    assert(descriptors[3].Buffer1Addr==(uintptr_t)payload_b);
    assert(emac_esp_dma_transmit_frame(&dma,header,sizeof(header))==sizeof(header));
    complete_all();
    dma.tx_desc=&descriptors[3]; /* normal traffic must restore a completed loan */
    emac_esp_dma_transmit_buff_t normal={header,sizeof(header)};
    assert(emac_esp_dma_transmit_frame_ext(&dma,&normal,1,NULL)==sizeof(header));
    for(unsigned j=0;j<100;j++) {
        for(unsigned i=0;i<3;i++)submit(&dma,payload_a+i*1344);
        complete_all();drain(&dma,payload_a,sizeof(payload_a));
        for(unsigned i=0;i<8;i++)assert(descriptors[i].Buffer1Addr==(uintptr_t)stock[i]);
        assert(emac_esp_dma_transmit_frame(&dma,header,sizeof(header))==sizeof(header));
    }
    dma.tx_desc->TDES0.Own=1;
    emac_esp_dma_transmit_buff_t b[2]={{header,sizeof(header)},{payload_a,1344}};
    assert(emac_esp_dma_transmit_frame_ext(&dma,b,2,(void *)&s31_emac_direct_token)==0);
    assert(s31_reclaim_loans()==0);
    complete_all();
    for(unsigned j=0;j<100;j++) {
        eth_dma_tx_descriptor_t *d=dma.tx_desc;
        memset(stock[d-descriptors],0x5a,sizeof(stock[0]));
        submit_frame(&dma,payload_a+42);
        /* Borrow the complete frame without copying its header or payload. */
        for(unsigned k=0;k<sizeof(stock[0]);k++)assert(stock[d-descriptors][k]==0x5a);
        submit_frame(&dma,payload_b+42);
        d->TDES0.Own=0;drain(&dma,payload_a,sizeof(payload_a));
        dma.tx_desc=d;
        assert(emac_esp_dma_transmit_frame(&dma,header,sizeof(header))==sizeof(header));
        complete_all();drain(&dma,payload_b,sizeof(payload_b));
        for(unsigned i=0;i<8;i++)assert(descriptors[i].Buffer1Addr==(uintptr_t)stock[i]);
        /* Filling the ring must fail without mutating a live descriptor. */
        for(unsigned i=0;i<8;i++)submit_frame(&dma,payload_a+42);
        d=dma.tx_desc;uint32_t address=d->Buffer1Addr,flags=d->TDES0.Value;
        emac_esp_dma_transmit_buff_t frame={payload_b+42,1430};
        assert(emac_esp_dma_transmit_frame_ext(&dma,&frame,1,(void *)&s31_emac_frame_token)==0);
        assert(d->Buffer1Addr==address && d->TDES0.Value==flags);
        complete_all();drain(&dma,payload_a,sizeof(payload_a));
    }
    return 0;
}
'''


@unittest.skipUnless(sys.platform == 'linux' and shutil.which('cc'), 'Linux host C compiler required')
class DmaLifetimeTests(unittest.TestCase):
    def test_borrow_reclaim_normal_traffic_wrap_and_timeout(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)
            for name in ('esp_attr.h', 'esp_system.h', 'esp_timer.h'):
                (path / name).write_text('')
            source = HARNESS.replace('@ADAPTER@', str(ROOT / 'main/targets/esp32s31/emac_direct.inc'))
            (path / 'test.c').write_text(source)
            exe = path / 'test'
            # Non-PIE static storage stays in the 32-bit descriptor address range.
            subprocess.run(['cc', '-std=gnu11', '-O2', '-no-pie', '-Wno-int-to-pointer-cast',
                            '-Wno-pointer-to-int-cast', '-I', str(path), str(path / 'test.c'),
                            '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True, timeout=10)
            self.assertEqual(subprocess.run([str(exe), 'timeout'], timeout=10).returncode, 77)
            self.assertEqual(subprocess.run([str(exe), 'timeout', 'frame'], timeout=10).returncode, 77)


if __name__ == '__main__':
    unittest.main()
