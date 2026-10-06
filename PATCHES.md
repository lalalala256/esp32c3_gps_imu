# Out-of-tree patches

Changes this project needs that live **outside** its own source tree. None of them
are visible from `main/`, so they are easy to lose.

## 1. IDF: skip the SDIO probe in SPI mode

**File:** `$IDF_PATH/components/sdmmc/sdmmc_init.c`, in `sdmmc_card_init()`

```diff
     const bool is_spi = host_is_spi(card);
     const bool always = true;
-    const bool io_supported = true;
+
+    const bool io_supported = !is_spi;
+
+    if (!io_supported) {
+        card->is_mem = 1;
+        card->is_sdio = 0;
+    }
```

### Why

`sdmmc_card_init()` runs `sdmmc_init_io()` before `sdmmc_init_spi_crc()`, and
`sdmmc_init_io()` sends **CMD5 (IO_SEND_OP_COND)** to look for an SDIO card.

The microSD card used here stops accepting **CMD59 (CRC_ON_OFF)** once it has seen
CMD5. `sdmmc_init_spi_crc()` is the next step, so it fails with
`ESP_ERR_NOT_SUPPORTED (0x106)` and the whole mount dies. Because the SDSPI driver
pins the identification clock to `SDMMC_FREQ_PROBING` (400 kHz) inside
`sdspi_host_init_slot`, all three of this project's mount attempts fail in exactly
the same way, which makes it look like a wiring problem.

Reproduced independently by the in-firmware probe, identically at 100, 200 and
400 kHz:

| probe sequence | CMD59 result |
|---|---|
| CMD0, CMD8, CMD59 | `0x01` accepted |
| CMD0, CMD8, CMD58, CMD59 | `0x01` accepted (control: a legit extra command is harmless) |
| CMD0, CMD8, **CMD5**, CMD59 | **`0x05` illegal command** |

The card is fine in a PC card reader because that speaks native SD mode and never
sends CMD5.

`card->is_mem` has to be set explicitly: the card struct was just `memset` to zero
and it is `sdmmc_init_io()` that records "memory card" when CMD5 goes unanswered.
Without it the card is treated as SDIO and ACMD41/CID/CSD are all skipped.

### Reverting

```sh
cd $IDF_PATH && git checkout -- components/sdmmc/sdmmc_init.c
```

A pristine copy is also kept at `/tmp/sdmmc_init.c.orig` (not durable).

## 2. FATFS long file names (`sdkconfig.defaults`, in-tree)

`CONFIG_FATFS_LFN_HEAP=y`. The CSV names are not valid 8.3 names -
`gnss_0001.csv` is a 9.3 name and `events_0001.csv` is 11.3 - so on the default
`CONFIG_FATFS_LFN_NONE` the first two `fopen()` calls succeed and the third fails
with `FR_INVALID_NAME`, leaving the logger degraded to serial-only. This one *is*
in the repo, and is listed here only because the symptom is so easy to
misattribute to the card.
