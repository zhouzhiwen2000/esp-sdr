/* LP-core live-TCM debug-probe experiment.
 *
 * This binary is excluded from the production continuous-real-IF build. It is
 * retained only so disabling that backend still permits controlled research
 * on the legacy TCM path.
 */
#include <stdint.h>

#define RF_DUMP_BEGIN 0x2f060000u

volatile uint32_t command;
volatile uint32_t completed;
volatile uint32_t match_words;

int main(void) {
  for (;;) {
    if (command == 1u) {
      volatile uint32_t *const replay =
          (volatile uint32_t *)(uintptr_t)RF_DUMP_BEGIN;
      uint32_t matches = 0u;
      for (uint32_t word = 0u; word < 16u; ++word) {
        uint32_t value = replay[word];
        uint32_t raw_q = (value >> 10u) & 0x3ffu;
        int32_t q = (raw_q & 0x200u) != 0u
                        ? (int32_t)raw_q - 1024
                        : (int32_t)raw_q;
        uint32_t replaced = (value & ~(0x3ffu << 10u)) |
                            (((uint32_t)(-q) & 0x3ffu) << 10u);
        replay[word] = replaced;
        if (replay[word] == replaced) {
          ++matches;
        }
      }
      match_words = matches;
      ++completed;
      command = 0u;
    }
  }
}
