#include <stdio.h>
#include <stdint.h>
#include <windows.h>

#include <ufprog/osdef.h>
#include <ufprog/log.h>
#include <ufprog/spi.h>

#define CMD_RESET        0xFF
#define CMD_WREN         0x06
#define CMD_GET_FEATURE  0x0F
#define CMD_SET_FEATURE  0x1F
#define CMD_BLOCK_ERASE  0xD8

#define REG_PROTECTION   0xA0
#define REG_CONFIG       0xB0
#define REG_STATUS       0xC0

#define STATUS_OIP       0x01
#define STATUS_WEL       0x02
#define STATUS_E_FAIL    0x04

/*
 * ESMT F50L1G41LB
 *
 * 1 Gbit = 128 MiB main array
 * Block size = 128 KiB
 * Number of blocks = 1024
 * Pages per block = 64
 *
 * Block N row address = N << 6
 *
 * This deliberately erases EVERY block, including blocks that may
 * have been factory-marked bad.
 */

#define TOTAL_BLOCKS     1024
#define PAGES_PER_BLOCK  64

static int transfer(struct ufprog_spi *spi,
                    const uint8_t *tx, size_t txlen,
                    uint8_t *rx, size_t rxlen)
{
    struct ufprog_spi_transfer xfer[2] = {0};
    unsigned n = 0;

    if (txlen) {
        xfer[n].buf.tx = tx;
        xfer[n].len = txlen;
        xfer[n].dir = SPI_DATA_OUT;
        xfer[n].end = !rxlen;
        n++;
    }

    if (rxlen) {
        xfer[n].buf.rx = rx;
        xfer[n].len = rxlen;
        xfer[n].dir = SPI_DATA_IN;
        xfer[n].end = true;
        n++;
    }

    if (!n)
        return -1;

    return ufprog_spi_generic_xfer(spi, xfer, n);
}

static int command(struct ufprog_spi *spi, uint8_t opcode)
{
    return transfer(spi, &opcode, 1, NULL, 0);
}

static int read_feature(struct ufprog_spi *spi,
                        uint8_t addr,
                        uint8_t *value)
{
    uint8_t tx[2] = {
        CMD_GET_FEATURE,
        addr
    };

    uint8_t rx = 0;

    if (transfer(spi, tx, sizeof(tx), &rx, 1))
        return -1;

    *value = rx;

    return 0;
}

static int set_feature(struct ufprog_spi *spi,
                       uint8_t addr,
                       uint8_t value)
{
    uint8_t tx[3] = {
        CMD_SET_FEATURE,
        addr,
        value
    };

    return transfer(spi, tx, sizeof(tx), NULL, 0);
}

static int write_enable(struct ufprog_spi *spi)
{
    uint8_t status;

    if (command(spi, CMD_WREN))
        return -1;

    if (read_feature(spi, REG_STATUS, &status))
        return -1;

    if (!(status & STATUS_WEL))
        return -1;

    return 0;
}

static int wait_ready(struct ufprog_spi *spi)
{
    unsigned i;

    for (i = 0; i < 10000; i++) {
        uint8_t status;

        if (read_feature(spi, REG_STATUS, &status))
            return -1;

        if (!(status & STATUS_OIP))
            return 0;

        Sleep(1);
    }

    return -1;
}

static int reset_chip(struct ufprog_spi *spi)
{
    if (command(spi, CMD_RESET))
        return -1;

    Sleep(2);

    return 0;
}

static int erase_block(struct ufprog_spi *spi, uint32_t block)
{
    uint32_t row = block * PAGES_PER_BLOCK;

    uint8_t tx[4];

    tx[0] = CMD_BLOCK_ERASE;
    tx[1] = (uint8_t)(row >> 16);
    tx[2] = (uint8_t)(row >> 8);
    tx[3] = (uint8_t)row;

    if (write_enable(spi))
        return -1;

    if (transfer(spi, tx, sizeof(tx), NULL, 0))
        return -1;

    if (wait_ready(spi))
        return -2;

    {
        uint8_t status;

        if (read_feature(spi, REG_STATUS, &status))
            return -1;

        if (status & STATUS_E_FAIL)
            return -3;
    }

    return 0;
}

static void print_status(struct ufprog_spi *spi)
{
    uint8_t status;

    if (read_feature(spi, REG_STATUS, &status))
        return;

    printf("Status: %02X  OIP=%u WEL=%u E_FAIL=%u\n",
           status,
           !!(status & STATUS_OIP),
           !!(status & STATUS_WEL),
           !!(status & STATUS_E_FAIL));
}

int wmain(void)
{
    struct ufprog_spi *spi = NULL;
    ufprog_status ret;

    set_os_default_log_print();
    os_init();

    printf("\n");
    printf("============================================================\n");
    printf(" ESMT F50L1G41LB FULL CHIP ERASE\n");
    printf("============================================================\n");
    printf("\n");
    printf("WARNING: THIS IS FULLY DESTRUCTIVE.\n");
    printf("ALL %u BLOCKS WILL BE ERASED.\n", TOTAL_BLOCKS);
    printf("FACTORY-MARKED BAD BLOCKS WILL ALSO BE ATTEMPTED.\n");
    printf("NO DATA WILL BE RECOVERABLE AFTER THIS OPERATION.\n");
    printf("\n");
    printf("Capacity : 1 Gbit / 128 MiB\n");
    printf("Blocks   : %u\n", TOTAL_BLOCKS);
    printf("Block    : 128 KiB\n");
    printf("Pages    : %u per block\n", PAGES_PER_BLOCK);
    printf("\n");

    ret = ufprog_spi_open_device("ch341-libusb", false, &spi);

    if (ret) {
        fprintf(stderr,
                "ERROR: unable to open ch341-libusb: %u\n",
                ret);
        return 1;
    }

    printf("Controller opened successfully.\n");

    printf("\nRESET\n");

    if (reset_chip(spi)) {
        printf("RESET FAILED\n");
        goto cleanup;
    }

    printf("RESET OK\n");

    /*
     * Disable protection.
     */
    {
        uint8_t protection;

        if (read_feature(spi, REG_PROTECTION, &protection)) {
            printf("Unable to read protection register\n");
            goto cleanup;
        }

        printf("\nProtection A0 before erase: %02X\n", protection);

        if (write_enable(spi)) {
            printf("WRITE ENABLE failed while clearing protection\n");
            goto cleanup;
        }

        if (set_feature(spi, REG_PROTECTION, 0x00)) {
            printf("SET FEATURE A0 failed\n");
            goto cleanup;
        }

        if (wait_ready(spi)) {
            printf("Timeout clearing protection\n");
            goto cleanup;
        }

        if (read_feature(spi, REG_PROTECTION, &protection)) {
            printf("Unable to verify protection register\n");
            goto cleanup;
        }

        printf("Protection A0 after erase setup: %02X\n",
               protection);

        if (protection != 0x00) {
            printf("ERROR: protection is still enabled\n");
            goto cleanup;
        }
    }

    printf("\n");
    printf("============================================================\n");
    printf(" STARTING FULL CHIP ERASE\n");
    printf("============================================================\n\n");

    unsigned failed = 0;

    for (uint32_t block = 0;
         block < TOTAL_BLOCKS;
         block++) {

        uint32_t row = block * PAGES_PER_BLOCK;

        printf("\rBlock %4u/%u  Row %06X",
               block + 1,
               TOTAL_BLOCKS,
               row);

        fflush(stdout);

        int result = erase_block(spi, block);

        if (result == 0) {
            printf("  OK\n");
        } else if (result == -2) {
            printf("  TIMEOUT\n");
            failed++;
        } else if (result == -3) {
            uint8_t status = 0;

            read_feature(spi, REG_STATUS, &status);

            printf("  ERASE FAIL  STATUS=%02X\n",
                   status);

            failed++;
        } else {
            printf("  ERROR\n");
            failed++;
        }

        /*
         * Re-arm the device after every operation.
         * This also gives the chip a clean state before
         * proceeding to the next block.
         */
        if (reset_chip(spi)) {
            printf("\nRESET FAILED AFTER BLOCK %u\n",
                   block);
            failed++;
            break;
        }
    }

    printf("\n");
    printf("============================================================\n");
    printf(" FULL CHIP ERASE FINISHED\n");
    printf("============================================================\n");

    printf("\nFailed blocks: %u / %u\n",
           failed,
           TOTAL_BLOCKS);

    print_status(spi);

    if (!failed)
        printf("\n*** ALL BLOCKS ERASED SUCCESSFULLY ***\n");
    else
        printf("\n*** ERASE COMPLETED WITH FAILURES ***\n");

    printf("\nFINAL RESET\n");

    if (reset_chip(spi))
        printf("FINAL RESET FAILED\n");
    else
        printf("FINAL RESET OK\n");

cleanup:

    if (spi)
        ufprog_spi_close_device(spi);

    printf("\n=== FULL CHIP ERASE COMPLETE ===\n");

    return 0;
}