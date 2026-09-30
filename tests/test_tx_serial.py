"""Exercise binary upload routing and deadlines using production transport code."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TxSerial(unittest.TestCase):
    @unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
    def test_payload_stays_on_owner_and_times_out(self):
        source = (ROOT / 'main/common/burst_serial.c').read_text()
        read = source[source.index('static int read_port('):source.index('int burst_serial_poll_line(')]
        deadline = source[source.index('static int64_t transfer_deadline('):source.index('bool IRAM_ATTR burst_serial_send(')]
        receive = source[source.index('bool burst_serial_receive('):]
        stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "burst_serial.h"
#define CONFIG_ESP_SDR_UART_ENABLED 1
#define SOC_USB_SERIAL_JTAG_SUPPORTED 1
#define UART_NUM_0 0
static burst_serial_port_t active_port;
static unsigned uart_baud=2000000;
static int64_t now;
static uint8_t input[2][256];
static size_t sizes[2],offsets[2];
static unsigned reads[2];
static int64_t esp_timer_get_time(void) { now+=1000;return now; }
static void vTaskDelay(int ticks) { now+=ticks*1000; }
static int read_bytes(unsigned port,void *buf,size_t size) {
 reads[port]++;
 size_t left=sizes[port]-offsets[port];if(size>left)size=left;
 if(size>7)size=7; /* Deliberately fragment USB and UART reads. */
 memcpy(buf,input[port]+offsets[port],size);offsets[port]+=size;return size;
}
static int uart_read_bytes(unsigned port,void *buf,size_t size,unsigned wait) {return read_bytes(BURST_SERIAL_UART,buf,size);}
static int usb_serial_jtag_ll_read_rxfifo(void *buf,size_t size) {return read_bytes(BURST_SERIAL_USB,buf,size);}
'''
        check = r'''
int main(void) {
 for(unsigned port=0;port<2;port++) {
  memset(offsets,0,sizeof(offsets));memset(reads,0,sizeof(reads));
  active_port=port;sizes[port]=256;sizes[1-port]=256;
  for(unsigned i=0;i<256;i++)input[port][i]=i;
  uint8_t payload[256];assert(burst_serial_receive(payload,256));
  for(unsigned i=0;i<256;i++)assert(payload[i]==i);
  assert(offsets[port]==256 && reads[port]>1 && reads[1-port]==0);
  int64_t started=now;assert(!burst_serial_receive(payload,1));
  assert(now-started>=2000000 && now-started<3100000 && reads[1-port]==0);
 }
 active_port=BURST_SERIAL_UART;uart_baud=115200;now=0;
 assert(transfer_deadline(65520)>7600000); /* Full frame fits even at low baud. */
 return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'serial.c';path.write_text(stub+read+deadline+receive+check)
            binary=Path(tmp)/'serial'
            subprocess.run(['cc','-std=c11','-Werror=implicit-function-declaration',
                            '-I'+str(ROOT/'main/common'),str(path),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)
