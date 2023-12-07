/*
 * oad_config.h
 */

#ifndef OAD_NATIVE_OAD_OAD_CONFIG_H_
#define OAD_NATIVE_OAD_OAD_CONFIG_H_

// rfOADClient (MCUBoot)
#if defined(MCUBOOT) && !defined(FEATURE_OAD_SERVER_ONLY)
#define MCUBOOT_SLOT_SIZE       0x0D000
#endif

// rfOADServer (MCUBoot)
#if defined(MCUBOOT) && defined(FEATURE_OAD_SERVER_ONLY)
#define MCUBOOT_IMG_HDR_BASE    0x0
#define MCUBOOT_IMG_HDR_SZ      0x20
#define ENTRY_SECTION_BASE      (MCUBOOT_IMG_HDR_BASE + MCUBOOT_IMG_HDR_SZ)
#define ENTRY_SECTION_SZ        0x100
#define OAD_IMG_HDR_BASE        (ENTRY_SECTION_BASE + ENTRY_SECTION_SZ)
#endif

#endif /* OAD_NATIVE_OAD_OAD_CONFIG_H_ */
