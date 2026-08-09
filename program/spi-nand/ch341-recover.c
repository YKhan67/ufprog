#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <windows.h>

#include <ufprog/osdef.h>
#include <ufprog/device.h>
#include <ufprog/spi.h>
#include <ufprog/api_spi.h>

#define TEST_PAGE       0
#define TEST_LEN        256
#define COLUMN          0x0000

#define CMD_RESET       0xFF
#define CMD_WREN        0x06
#define CMD_GET_FEATURE 0x0F
#define CMD_PAGE_READ   0x13
#define CMD_PROG_LOAD   0x02
#define CMD_PROG_EXEC   0x10
#define CMD_READ_CACHE  0x03

#define REG_PROTECTION  0xA0
#define REG_CONFIG      0xB0
#define REG_STATUS      0xC0

#define STATUS_OIP      0x01
#define STATUS_WEL      0x02
#define STATUS_E_FAIL   0x04
#define STATUS_P_FAIL   0x08
#define STATUS_ECC_MASK 0x30

static int xfer(struct ufprog_spi *spi,
                const void *tx, size_t txlen,
                void *rx, size_t rxlen)
{
    struct ufprog_spi_transfer xfers[2];
    uint32_t count = 0;
    ufprog_status ret;

    memset(xfers, 0, sizeof(xfers));

    if (tx && txlen) {
        xfers[count].buf.tx = tx;
        xfers[count].len = txlen;
        xfers[count].dir = SPI_DATA_OUT;
        xfers[count].buswidth = 1;
        xfers[count].dtr = 0;
        xfers[count].end = (rx && rxlen) ? 0 : 1;
        count++;
    }

    if (rx && rxlen) {
        xfers[count].buf.rx = rx;
        xfers[count].len = rxlen;
        xfers[count].dir = SPI_DATA_IN;
        xfers[count].buswidth = 1;
        xfers[count].dtr = 0;
        xfers[count].end = 1;
        count++;
    }

    if (!count)
        return -1;

    ret = ufprog_spi_generic_xfer(spi, xfers, count);

    if (ret != UFP_OK) {
        printf("SPI transfer failed: %d\n", ret);
        return -1;
    }

    return 0;
}

static int command(struct ufprog_spi *spi, uint8_t opcode)
{
    return xfer(spi, &opcode, 1, NULL, 0);
}

static int reset_chip(struct ufprog_spi *spi)
{
    printf("RESET\n");

    if (command(spi, CMD_RESET))
        return -1;

    Sleep(2);

    printf("RESET OK\n");
    return 0;
}

static int get_feature(struct ufprog_spi *spi,
                       uint8_t addr,
                       uint8_t *value)
{
    uint8_t tx[2];
    uint8_t rx = 0;

    tx[0] = CMD_GET_FEATURE;
    tx[1] = addr;

    if (xfer(spi, tx, sizeof(tx), &rx, 1))
        return -1;

    *value = rx;

    return 0;
}

static int read_status(struct ufprog_spi *spi, uint8_t *status)
{
    return get_feature(spi, REG_STATUS, status);
}

static void print_status(uint8_t status)
{
    printf("Status C0: %02X  OIP=%u WEL=%u E_FAIL=%u P_FAIL=%u ECC=%02X\n",
           status,
           !!(status & STATUS_OIP),
           !!(status & STATUS_WEL),
           !!(status & STATUS_E_FAIL),
           !!(status & STATUS_P_FAIL),
           (status & STATUS_ECC_MASK) >> 4);
}

static int wait_ready(struct ufprog_spi *spi, unsigned timeout_ms)
{
    unsigned elapsed;

    for (elapsed = 0; elapsed < timeout_ms; elapsed++) {
        uint8_t status = 0;

        if (read_status(spi, &status))
            return -1;

        if (!(status & STATUS_OIP))
            return 0;

        Sleep(1);
    }

    fprintf(stderr, "ERROR: timeout waiting for NAND ready\n");
    return -1;
}

static int write_enable(struct ufprog_spi *spi)
{
    uint8_t status = 0;

    printf("WRITE ENABLE\n");

    if (command(spi, CMD_WREN))
        return -1;

    if (read_status(spi, &status))
        return -1;

    printf("After WREN: C0=%02X WEL=%u\n",
           status,
           !!(status & STATUS_WEL));

    if (!(status & STATUS_WEL)) {
        fprintf(stderr, "ERROR: WEL did not become 1\n");
        return -1;
    }

    return 0;
}

static int page_read(struct ufprog_spi *spi, uint32_t page)
{
    uint8_t tx[4];

    tx[0] = CMD_PAGE_READ;
    tx[1] = (uint8_t)(page >> 16);
    tx[2] = (uint8_t)(page >> 8);
    tx[3] = (uint8_t)page;

    printf("PAGE READ 13h: %02X %02X %02X %02X\n",
           tx[0], tx[1], tx[2], tx[3]);

    if (xfer(spi, tx, sizeof(tx), NULL, 0))
        return -1;

    if (wait_ready(spi, 1000))
        return -1;

    return 0;
}

static int read_cache(struct ufprog_spi *spi,
                       uint16_t column,
                       uint8_t *data,
                       size_t len)
{
    uint8_t tx[3];

    tx[0] = CMD_READ_CACHE;
    tx[1] = (uint8_t)(column >> 8);
    tx[2] = (uint8_t)column;

    /*
     * 03h READ CACHE:
     *
     *   CS low
     *   03h
     *   column MSB
     *   column LSB
     *   data...
     *   CS high
     */
    if (xfer(spi, tx, sizeof(tx), data, len))
        return -1;

    return 0;
}

static int program_load(struct ufprog_spi *spi,
                        uint16_t column,
                        const uint8_t *data,
                        size_t len)
{
    uint8_t header[3];
    struct ufprog_spi_transfer xfers[2];
    ufprog_status ret;

    header[0] = CMD_PROG_LOAD;
    header[1] = (uint8_t)(column >> 8);
    header[2] = (uint8_t)column;

    memset(xfers, 0, sizeof(xfers));

    /*
     * 02h PROGRAM LOAD
     *
     * CS LOW
     *
     *   02h
     *   column[15:8]
     *   column[7:0]
     *   data...
     *
     * CS HIGH
     *
     * The header and data MUST remain within the same CS
     * assertion. end=0 on the header keeps CS asserted.
     */

    xfers[0].buf.tx = header;
    xfers[0].len = sizeof(header);
    xfers[0].dir = SPI_DATA_OUT;
    xfers[0].buswidth = 1;
    xfers[0].dtr = 0;
    xfers[0].end = 0;

    xfers[1].buf.tx = data;
    xfers[1].len = len;
    xfers[1].dir = SPI_DATA_OUT;
    xfers[1].buswidth = 1;
    xfers[1].dtr = 0;
    xfers[1].end = 1;

    ret = ufprog_spi_generic_xfer(spi, xfers, 2);

    if (ret != UFP_OK) {
        printf("PROGRAM LOAD failed: %d\n", ret);
        return -1;
    }

    return 0;
}

static int program_execute(struct ufprog_spi *spi, uint32_t page)
{
    uint8_t tx[4];

    /*
     * 10h PROGRAM EXECUTE
     *
     * This MUST be a separate SPI transaction from 02h.
     *
     * First transaction:
     *
     *   CS LOW
     *   02h
     *   column
     *   data
     *   CS HIGH
     *
     * Second transaction:
     *
     *   CS LOW
     *   10h
     *   row[23:16]
     *   row[15:8]
     *   row[7:0]
     *   CS HIGH
     */

    tx[0] = CMD_PROG_EXEC;
    tx[1] = (uint8_t)(page >> 16);
    tx[2] = (uint8_t)(page >> 8);
    tx[3] = (uint8_t)page;

    printf("PROGRAM EXECUTE 10h: %02X %02X %02X %02X\n",
           tx[0], tx[1], tx[2], tx[3]);

    if (xfer(spi, tx, sizeof(tx), NULL, 0))
        return -1;

    if (wait_ready(spi, 5000))
        return -1;

    return 0;
}

static void dump_data(const char *title,
                      const uint8_t *data,
                      size_t len)
{
    size_t i;

    if (title)
        printf("%s\n", title);

    for (i = 0; i < len; i += 16) {
        size_t j;

        printf("%04zX:", i);

        for (j = 0; j < 16 && i + j < len; j++)
            printf(" %02X", data[i + j]);

        printf("\n");
    }
}

static void make_test_pattern(uint8_t *data, size_t len)
{
    size_t i;

    for (i = 0; i < len; i++)
        data[i] = (uint8_t)(0xA5 ^ (uint8_t)i);
}

static int verify_page(struct ufprog_spi *spi,
                       uint32_t page,
                       const uint8_t *expected,
                       size_t len)
{
    uint8_t actual[TEST_LEN];
    size_t i;
    unsigned mismatches = 0;

    printf("\nREAD PAGE %u AFTER PROGRAM\n", page);

    if (page_read(spi, page))
        return -1;

    memset(actual, 0, sizeof(actual));

    if (read_cache(spi, COLUMN, actual, len))
        return -1;

    dump_data("READBACK:", actual, len);

    for (i = 0; i < len; i++) {
        if (actual[i] != expected[i]) {
            if (mismatches < 32) {
                printf("MISMATCH offset %03zX: wrote %02X read %02X\n",
                       i,
                       expected[i],
                       actual[i]);
            }

            mismatches++;
        }
    }

    if (mismatches) {
        printf("VERIFY FAILED: %u mismatches\n", mismatches);
        return -1;
    }

    printf("VERIFY SUCCESS: %zu bytes match\n", len);

    return 0;
}

int wmain(void)
{
    struct ufprog_controller_device *dev = NULL;
    struct ufprog_spi *spi = NULL;

    uint8_t protection = 0;
    uint8_t config = 0;
    uint8_t status = 0;

    uint8_t existing[TEST_LEN];
    uint8_t pattern[TEST_LEN];

    ufprog_status ret;
    int result = 1;

    printf("=== ESMT F50L1G41LB SINGLE-PAGE PROGRAM/READ TEST ===\n");
    printf("WARNING: PAGE 0 WILL BE MODIFIED. NO ERASE IS PERFORMED.\n\n");

    /*
     * Open the CH341 controller through the controller layer.
     *
     * Do NOT use:
     *
     *     ufprog_spi_open_device("ch341-libusb", ...)
     *
     * here.
     *
     * The correct path is:
     *
     *     controller_open_device_by_name()
     *             |
     *             v
     *     ufprog_spi_attach_device()
     */

    printf("Opening controller device 'ch341-libusb'...\n");

    ret = ufprog_controller_open_device_by_name(
        "ch341-libusb",
        IF_SPI,
        false,
        &dev
    );

    if (ret != UFP_OK) {
        printf("ERROR: unable to open ch341-libusb: %d\n", ret);
        goto cleanup;
    }

    printf("Controller opened successfully.\n");

    ret = ufprog_spi_attach_device(dev, &spi);

    if (ret != UFP_OK) {
        printf("ERROR: unable to attach SPI interface: %d\n", ret);
        goto cleanup;
    }

    printf("SPI interface attached successfully.\n");

    if (ufprog_spi_set_mode(spi, 0) != UFP_OK) {
        printf("ERROR: unable to set SPI mode 0\n");
        goto cleanup;
    }

    if (ufprog_spi_set_speed_closest(spi, 1000000, NULL) != UFP_OK)
        printf("WARNING: unable to set 1 MHz; continuing\n");

    if (reset_chip(spi))
        goto cleanup;

    printf("\nINITIAL FEATURES\n");

    if (get_feature(spi, REG_PROTECTION, &protection))
        goto cleanup;

    if (get_feature(spi, REG_CONFIG, &config))
        goto cleanup;

    if (read_status(spi, &status))
        goto cleanup;

    printf("Protection (A0):   %02X\n", protection);
    printf("Configuration (B0): %02X\n", config);
    print_status(status);

    printf("\nREAD EXISTING PAGE 0\n");

    if (page_read(spi, TEST_PAGE))
        goto cleanup;

    memset(existing, 0, sizeof(existing));

    if (read_cache(spi, COLUMN, existing, sizeof(existing)))
        goto cleanup;

    dump_data("EXISTING DATA (first 64 bytes):", existing, 64);

    make_test_pattern(pattern, sizeof(pattern));

    dump_data("\nTEST PATTERN (first 64 bytes):", pattern, 64);

    /*
     * Set WEL immediately before PROGRAM LOAD.
     */

    printf("\n");

    if (write_enable(spi))
        goto cleanup;

    /*
     * PROGRAM LOAD
     */

    printf("\nPROGRAM LOAD 02h\n");
    printf("Column=%04X len=%zu\n",
           COLUMN,
           sizeof(pattern));

    if (program_load(spi,
                     COLUMN,
                     pattern,
                     sizeof(pattern)))
        goto cleanup;

    /*
     * Confirm that the program-load transaction completed.
     *
     * WEL should still be set because PROGRAM LOAD only fills the
     * internal program buffer. The actual NAND program operation
     * happens only after 10h.
     */

    if (read_status(spi, &status))
        goto cleanup;

    printf("AFTER PROGRAM LOAD: ");
    print_status(status);

    if (!(status & STATUS_WEL)) {
        printf("WARNING: WEL cleared after PROGRAM LOAD\n");
    }

    /*
     * PROGRAM EXECUTE
     *
     * Separate CS transaction.
     */

    printf("\nPROGRAM EXECUTE\n");

    if (program_execute(spi, TEST_PAGE))
        goto cleanup;

    /*
     * Check final program status.
     */

    if (read_status(spi, &status))
        goto cleanup;

    printf("\nSTATUS AFTER PROGRAM EXECUTE\n");
    print_status(status);

    if (status & STATUS_P_FAIL) {
        printf("ERROR: NAND reported PROGRAM FAIL (P_FAIL=1)\n");
        goto cleanup;
    }

    if (status & STATUS_E_FAIL)
        printf("WARNING: E_FAIL is set\n");

    /*
     * Read the page back.
     */

    if (verify_page(spi,
                    TEST_PAGE,
                    pattern,
                    sizeof(pattern)))
        goto cleanup;

    printf("\nFINAL FEATURES\n");

    if (get_feature(spi, REG_PROTECTION, &protection))
        goto cleanup;

    if (get_feature(spi, REG_CONFIG, &config))
        goto cleanup;

    if (read_status(spi, &status))
        goto cleanup;

    printf("Protection (A0):     %02X\n", protection);
    printf("Configuration (B0): %02X\n", config);
    print_status(status);

    printf("\n=== TEST COMPLETE ===\n");

    result = 0;

cleanup:

    if (spi)
        ufprog_spi_close_device(spi);

    return result;
}