
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_signature_check();
void shutdown_signature_check();

// Returns 0 if ok
int check_u2f_block_signature(const uint8_t *u2fBlock);

#ifdef __cplusplus
}
#endif
