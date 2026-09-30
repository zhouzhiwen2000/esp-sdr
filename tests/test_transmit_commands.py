"""Run restored TX upload, packing, leases and command handlers with simulated RF."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which('cc'), 'Host C compiler unavailable')
class TransmitCommands(unittest.TestCase):
    def test_c5_and_s3_transmitter(self):
        stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#define IQ_WORDS 16380u
static uint32_t samples[IQ_WORDS+4];
#define IQ_BUFFER samples
#define SRAM_OWNER_REG 0x1234u
#define tx_mux 0
#define taskENTER_CRITICAL(p) ((void)0)
#define taskEXIT_CRITICAL(p) ((void)0)
static uint32_t regs[8];
static uint32_t *reg_ptr(unsigned addr) {
 switch(addr) {
 case SRAM_OWNER_REG:return &regs[0];
 case 0x600a9004:return &regs[1];
 case 0x600a900c:return &regs[2];
 case 0x600a0c04:return &regs[3];
 case 0x60033d5c:return &regs[4];
 case 0x60033d64:return &regs[5];
 default:assert(false);return NULL;
 }
}
#define REG_READ(a) (*reg_ptr(a))
#define REG_WRITE(a,b) (*reg_ptr(a)=(b))
static unsigned frequency_mhz=2412,preparations,emissions,last_rate,last_repeats;
static bool rx_ready=true,rf_active,dma_ok=true;
static unsigned uploaded_count;
static uint8_t uploaded[IQ_WORDS*4];
static size_t uploaded_size,uploaded_pos;
static char response[1024];
static int64_t now;
static uint32_t tx_cycles=42,tx_late=0;
static int64_t esp_timer_get_time(void) { return now; }
static void vTaskDelay(int t) { now+=t*1000; }
static void prepare_rx(void) { preparations++;rx_ready=true;rf_active=false; }
static void reply(const char *s) { assert(strlen(response)+strlen(s)<sizeof(response));strcat(response,s); }
static void esp_phy_wifi_tx_tone(unsigned on,unsigned f,unsigned gain) { rf_active=on; }
static void phy_stop_tx_tone(unsigned arg) {}
static void s3_tune(unsigned mhz) { assert(mhz==frequency_mhz); }
static void adctrig(unsigned a,unsigned b,unsigned c,unsigned d,unsigned e,unsigned f,unsigned g,unsigned h,unsigned i) {
 /* Clock initialization destroys the aperture: it must happen before upload. */
 memset(samples,0xcc,sizeof(samples));
}
static uint32_t esp_rom_crc32_le(uint32_t crc,const uint8_t *data,size_t size) {
 crc=~crc;
 while(size--) { crc^=*data++;for(unsigned i=0;i<8;i++)crc=(crc>>1)^(0xedb88320u & -(crc&1)); }
 return ~crc;
}
static bool receive_bytes(void *data,size_t size) {
 if(size>uploaded_size-uploaded_pos)return false;
 memcpy(data,uploaded+uploaded_pos,size);uploaded_pos+=size;return true;
}
static size_t wire_size(unsigned n,unsigned format) { return format==16?n*2:format==20?(n*20+7)/8:n*4; }
static bool emit_dma(unsigned n,unsigned rate,unsigned repeats) {
 assert(rf_active);emissions++;last_rate=rate;last_repeats=repeats;uploaded_count=n;return dma_ok;
}
static void emit_iq(uint32_t *words,unsigned n,unsigned rate,unsigned repeats) {
 assert(emit_dma(n,rate,repeats));
}
#include "tx_unpack.h"
#include "continuous_tone.h"
'''
        check = r'''
static bool command(const char *s) { response[0]=0;return tx_command(s); }
static void payload(void) {
 const uint8_t data[]={0x80,0x7f,0xff,0x01,0x00,0x00};
 memcpy(uploaded,data,sizeof(data));uploaded_size=sizeof(data);uploaded_pos=0;
}
static void tx_payload(const char *name,unsigned rate,unsigned repeat,bool good_crc) {
 payload();char line[100];uint32_t crc=esp_rom_crc32_le(0,uploaded,uploaded_size);
 if(!good_crc)crc^=1;
 if(repeat)snprintf(line,sizeof(line),"%s 3 %u %u %x",name,rate,repeat,crc);
 else snprintf(line,sizeof(line),"%s 3 %u %x",name,rate,crc);
 assert(command(line));
}
int main(void) {
 regs[0]=0xabcdef00;regs[1]=0x55;regs[3]=0x12345678;
 tx_payload("TX16",40000000,0,true);
 assert(emissions==1 && uploaded_count==3 && last_rate==40000000 && last_repeats==1);
 assert(samples[0]==((0x80u<<2)|(0x7fu<<12)) && samples[1]==((0xffu<<2)|(1u<<12)) && samples[2]==0);
 assert(strstr(response,"READY\nSENT 3 42 0\n") && rx_ready && !rf_active);
 tx_payload("LOOP16",80000000,2,true);assert(emissions==2 && last_repeats==2);
 assert(strstr(response,"SENT 6 "));
 tx_payload("TX16",40000000,0,false);assert(emissions==2 && strstr(response,"ERR crc\n") && !rf_active);
 uploaded_size=0;uploaded_pos=0;
 command("TX16 3 40000000 0");assert(emissions==2 && strstr(response,"ERR upload_timeout\n"));
 dma_ok=false;tx_payload("TX16",40000000,0,true);
 assert(emissions==3 && strstr(response,"ERR dma_timeout\n") && !rf_active && rx_ready);dma_ok=true;
 /* Test expansion of odd-sized IQ10 payloads, including sign bits, in place. */
 const uint32_t expected[]={0x8007f,0xfffff,0x12345};
 const uint8_t packed[]={0x7f,0x00,0xf8,0xff,0xff,0x45,0x23,0x01};
 memcpy(uploaded,packed,sizeof(packed));uploaded_pos=0;uploaded_size=sizeof(packed);
 char line[100];snprintf(line,sizeof(line),"TX20 3 80000000 %x",esp_rom_crc32_le(0,uploaded,uploaded_size));
 assert(command(line));assert(!memcmp(samples,expected,sizeof(expected)));
 /* Full words remain accepted. */
 memcpy(uploaded,expected,sizeof(expected));uploaded_pos=0;uploaded_size=sizeof(expected);
 snprintf(line,sizeof(line),"TX 3 80000000 %x",esp_rom_crc32_le(0,uploaded,uploaded_size));
 assert(command(line));assert(!memcmp(samples,expected,sizeof(expected)));
 unsigned before=emissions;
 const char *bad[]={"TX16 0 40000000 0","TX16 16381 40000000 0","TX20 3 123 0",
  "TX16 3 40000000 0 junk","LOOP20 1 40000000 256 0","LOOP20 16380 40000000 255 0",
  "TXRUN 1 40000000 0 16","TXRUN 1 40000000 1001 16","TXRUN 1 40000000 1 32",
  "REPLAY20 16381 40000000 0","REPLAY16 1 20000000 0",
  "TX3 40000000 0","TX163 40000000 0","TX203 40000000 0",
  "LOOP163 40000000 1 0","TXRUN3 40000000 1 16"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++)assert(!command(bad[i]));
 assert(emissions==before);
#if CONFIG_IDF_TARGET_ESP32C5
 tx_payload("TX16",250000,0,true);assert(emissions==before+1 && last_rate==250000);
#else
 assert(!command("TX16 3 250000 0"));assert(emissions==before);
#endif
 /* A run uploads a checksum, waits for READY, then uploads one binary frame. */
 payload();uint8_t data[6];memcpy(data,uploaded,6);uint32_t crc=esp_rom_crc32_le(0,data,6);
 for(unsigned j=0;j<2;j++) {
  for(unsigned k=0;k<4;k++)uploaded[j*10+k]=crc>>(8*k);
  memcpy(uploaded+j*10+4,data,6);
 }
 uploaded_size=20;uploaded_pos=0;before=emissions;
 assert(command("TXRUN 3 40000000 2 16"));assert(emissions==before+2 && strstr(response,"END\n"));
 /* Continuous TX owns its lease independently of INFO and query traffic. */
 assert(command("CW START"));assert(cw_active && rf_active && !rx_ready);
 now=4000000;assert(command("CW KEEP"));cw_service();assert(cw_active);
 assert(!command("INFO"));assert(cw_active);
 now=9000000;cw_service();assert(!cw_active && !rf_active && rx_ready);
 assert(command("CW KEEP"));assert(strstr(response,"ERR cw_inactive"));
 assert(command("CW START"));assert(!command("FREQ 2437"));assert(!cw_active && !rf_active);
 tx_payload("REPLAY16",40000000,0,true);
 assert(replay_active && !cw_active && rf_active && !rx_ready && strstr(response,"PLAYING"));
#if CONFIG_IDF_TARGET_ESP32C5
 assert(regs[2]&0x80080000u);
#else
 assert(regs[5]&0x80080000u);
#endif
 assert(!command("CAPS"));assert(replay_active);
 now+=4000000;assert(command("REPLAY KEEP"));replay_service();assert(replay_active);
 now+=5000000;replay_service();assert(!replay_active && rx_ready && !rf_active);
 assert(regs[0]==0xabcdef00 && regs[3]==0x12345678);
 assert(command("REPLAY KEEP"));assert(strstr(response,"ERR replay_inactive"));
 tx_payload("REPLAY16",80000000,0,true);assert(replay_active);
 assert(command("CW START"));assert(!replay_active && cw_active);
 assert(!command("RELEASE"));assert(!cw_active && !replay_active && rx_ready);
 tx_payload("REPLAY16",80000000,0,true);assert(replay_active);
 tx_payload("TX16",40000000,0,true);assert(!replay_active && !rf_active && rx_ready);
 return 0;
}
'''
        for chip in ('esp32c5', 'esp32s3'):
            with self.subTest(chip=chip), tempfile.TemporaryDirectory() as tmp:
                source = (ROOT / f'main/targets/{chip}/transmitter.h').read_text()
                transmit = source[source.index('static bool transmit('):source.index('#include "continuous_replay.h"')]
                # Only the RISC-V memory fence is host-incompatible in continuous replay.
                replay = (ROOT / 'main/common/continuous_replay.h').read_text().replace(
                    '__asm__ __volatile__("fence rw, rw" ::: "memory");', '')
                path = Path(tmp) / 'transmit.c'
                path.write_text(stub + transmit + replay + '\n#include "tx_commands.h"\n' + check)
                binary = Path(tmp) / 'transmit'
                subprocess.run(['cc', '-std=c11', '-Werror=implicit-function-declaration',
                                '-fsanitize=undefined', '-g',
                                f'-DCONFIG_IDF_TARGET_ESP32C5={int(chip == "esp32c5")}',
                                '-I' + str(ROOT / 'main/common'), str(path), '-o', str(binary)], check=True)
                subprocess.run([str(binary)], check=True)
