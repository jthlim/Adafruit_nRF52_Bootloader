
#include "signature_check.h"

#include "nrf52840.h"
#include "nrf_cc310_bl.h"

static const uint8_t publicKeyData[] = {
    0x0e, 0x19, 0xd5, 0x30, 0x08, 0xca, 0x53, 0x89, 0x1c, 0xc7, 0x90,
    0x53, 0xc9, 0xbd, 0x34, 0x91, 0xc6, 0x19, 0x8c, 0x36, 0x79, 0x78,
    0xea, 0xcc, 0x76, 0xed, 0x70, 0x43, 0x9c, 0x73, 0xf7, 0xac, 0xa3,
    0x8b, 0x58, 0x83, 0x23, 0xfa, 0xab, 0xe8, 0xd8, 0x27, 0x1f, 0x6f,
    0x26, 0xa6, 0x30, 0xaf, 0xe3, 0x2d, 0x87, 0x61, 0x78, 0x79, 0xbc,
    0xc3, 0x17, 0x6b, 0xe0, 0x28, 0x37, 0xad, 0x6b, 0x0d,
};

void init_signature_check() {
  // Enable the CC310 HW.
  NRF_CRYPTOCELL->ENABLE = 1;

  // Initialize the CC310_BL run-time library
  nrf_cc310_bl_init();
}

void shutdown_signature_check() {
  //
  NRF_CRYPTOCELL->ENABLE = 0;
}

// Returns 0 if ok
int check_u2f_block_signature(const uint8_t *u2fBlock) {
  uint32_t address = *(const uint32_t *)(u2fBlock + 12);

  // Only need signed data for 0-0xa4000 or 0xf4000-0x100000
  int isSignedAddressRange =
      address < 0xa4000 || (0xf4000 <= address && address < 0x100000);
  if (!isSignedAddressRange) {
    return 0;
  }

  nrf_cc310_bl_hash_context_sha256_t context;
  nrf_cc310_bl_hash_sha256_init(&context);
  nrf_cc310_bl_hash_sha256_update(&context, u2fBlock + 12, 4);
  nrf_cc310_bl_hash_sha256_update(&context, u2fBlock + 32, 256);
  nrf_cc310_bl_hash_digest_sha256_t digest;
  nrf_cc310_bl_hash_sha256_finalize(&context, &digest);

  nrf_cc310_bl_ecdsa_verify_context_secp256r1_t ecdsaContext;
  nrf_cc310_bl_ecdsa_verify_init_secp256r1(
      &ecdsaContext,
      (const nrf_cc310_bl_ecc_public_key_secp256r1_t *)publicKeyData);

  return nrf_cc310_bl_ecdsa_verify_hash_secp256r1(
      &ecdsaContext,
      (const nrf_cc310_bl_ecc_signature_secp256r1_t *)(u2fBlock + 32 + 256),
      digest, sizeof(digest));
}
