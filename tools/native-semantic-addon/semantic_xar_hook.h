/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
#ifndef NBT_SEMANTIC_XAR_HOOK_H
#define NBT_SEMANTIC_XAR_HOOK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Install writable-GOT hooks for open/open64/fopen in NavigationBasic so any
 * lazily opened stock GvID5VBU1025_Guiding{Day,Night}Standard.xar is served
 * from a validated in-memory override in /dev/shmem/. Idempotent & thread-safe. */
void semantic_xar_install_hooks(void);

/* Scan NavigationBasic's RW shared-memory style pool (0x28000000..0x2c000000)
 * safely via /proc/<pid>/as pread and patch any already-loaded stock Day/Night
 * GvID5VBU1025 XAR buffers directly in RAM (zero disk edits).
 * Returns total number of stock XAR buffers patched or already patched. */
unsigned semantic_xar_patch_shm_in_place(int as_fd);

#ifdef __cplusplus
}
#endif

#endif /* NBT_SEMANTIC_XAR_HOOK_H */
