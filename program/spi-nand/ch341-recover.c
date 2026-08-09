#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <windows.h>

#include <ufprog/api_spi.h>
#include <ufprog/spi.h>

#define PAGE_SIZE       2048
#define TEST_LEN        256
#define COLUMN          0x0000
#define TEST_PAGE       0

#define CMD_RESET       0xFF
#define CMD_WREN        0x06
#define CMD_GET_FEATURE 0x0F
#define CMD_SET_FEATURE 0x1F

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

#define SPI_DATA_OUT    0
#define SPI_DATA_IN     1

static void delay_ms(unsigned ms)
{
    Sleep(ms);
}

static void print_status(uint8_t s)
{
    printf("Status C0: %02X  OIP=%u WEL=%u E_FAIL=%u P_FAIL=%u ECC=%02X\n",
           s,
           !!(s & STATUS_OIP),
           !!(s & STATUS_WEL),
           !!(s & STATUS_E_FAIL),
           !!(s & STATUS_P_FAIL),
           (s & STATUS_ECC_MASK) >> 4);
}

static int xfer(struct ufprog_spi *spi,
                const void *tx, size_t txlen,
                void *rx, size_t rxlen)
{
    struct ufprog_spi_transfer xfers[2];
    uint32_t n = 0;
    ufprog_status ret;

    memset(xfers, 0, sizeof(xfers));

    if (tx && txlen) {
        xfers[n].buf.tx = tx;
        xfers[n].len = txlen;
        xfers[n].dir = SPI_DATA_OUT;
        xfers[n].buswidth = 1;
        xfers[n].dtr = 0;
        xfers[n].end = rx && rxlen ? 0 : 1;
        n++;
    }

    if (rx && rxlen) {
        xfers[n].buf.rx = rx;
        xfers[n].len = rxlen;
        xfers[n].dir = SPI_DATA_IN;
        xfers[n].buswidth = 1;
        xfers[n].dtr = 0;
        xfers[n].end = 1;
        n++;
    }

    if (!n)
        return -1;

    ret = ufprog_spi_generic_xfer(spi, xfers, n);

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

    delay_ms(2);

    printf("RESET OK\n");
    return 0;
}

static int get_feature(struct ufprog_spi *spi, uint8_t addr, uint8_t *value)
{
    uint8_t tx[2];
    uint8_t rx;

    tx[0] = CMD_GET_FEATURE;
    tx[1] = addr;

    if (xfer(spi, tx, sizeof(tx), &rx, 1))
        return -1;

    *value = rx;
    return 0;
}

static int wait_ready(struct ufprog_spi *spi, unsigned timeout_ms)
{
    unsigned elapsed = 0;

    while (elapsed < timeout_ms) {
        uint8_t status;

        if (read_status(spi, &status))
            return -1;

        if (!(status & STATUS_OIP)) {
            return 0;
        }

        delay_ms(1);
        elapsed++;
    }

    printf("ERROR: timeout waiting for OIP=0\n");
    return -1;
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
     * 03h is:
     *
     *   03h
     *   column[15:8]
     *   column[7:0]
     *   data...
     *
     * The first transaction sends the opcode and column while
     * keeping CS asserted. The second receives the data.
     */
    if (xfer(spi, tx, sizeof(tx), data, len))
        return -1;

    return 0;
}

static void dump_data(const uint8_t *data, size_t len)
{
    size_t i;

    for (i = 0; i < len; i += 16) {
        size_t j;
        printf("%04zX:", i);

        for (j = 0; j < 16 && i + j < len; j++)
            printf(" %02X", data[i + j]);

        printf("\n");
    }
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
     * IMPORTANT:
     *
     * 02h PROGRAM LOAD consists of:
     *
     *   02h
     *   column MSB
     *   column LSB
     *   data...
     *
     * CS must remain asserted across the header and data.
     * The final transfer has end=1, releasing CS only after data.
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

    tx[0] = CMD_PROG_EXEC;
    tx[1] = (uint8_t)(page >> 16);
    tx[2] = (uint8_t)(page >> 8);
    tx[3] = (uint8_t)page;

    /*
     * CRITICAL:
     *
     * This is a completely separate SPI transaction from 02h.
     *
     * The sequence on the wire is:
     *
     *   CS low
     *   02h + column + data
     *   CS high
     *
     *   CS low
     *   10h + row/page address
     *   CS high
     *
     * Do NOT combine 02h and 10h into one transaction.
     */
    printf("PROGRAM EXECUTE 10h: %02X %02X %02X %02X\n",
           tx[0], tx[1], tx[2], tx[3]);

    if (xfer(spi, tx, sizeof(tx), NULL, 0))
        return -1;

    if (wait_ready(spi, 5000))
        return -1;

    return 0;
}

static int verify_page(struct ufprog_spi *spi,
                       uint32_t page,
                       const uint8_t *expected,
                       size_t len)
{
    uint8_t actual[TEST_LEN];
    size_t i;
    int mismatches = 0;

    printf("\nREAD PAGE %u AFTER PROGRAM\n", page);

    if (page_read(spi, page))
        return -1;

    memset(actual, 0, sizeof(actual));

    if (read_cache(spi, COLUMN, actual, len))
        return -1;

    printf("READBACK:\n");
    dump_data(actual, len);

    for (i = 0; i < len; i++) {
        if (actual[i] != expected[i]) {
            if (mismatches < 32) {
                printf("MISMATCH offset %03zX: wrote %02X read %02X\n",
                       i, expected[i], actual[i]);
            }

            mismatches++;
        }
    }

    if (mismatches) {
        printf("VERIFY FAILED: %d mismatches\n", mismatches);
        return -1;
    }

    printf("VERIFY SUCCESS: %zu bytes match\n", len);
    return 0;
}

static int make_test_pattern(uint8_t *data, size_t len)
{
    size_t i;

    for (i = 0; i < len; i++)
        data[i] = (uint8_t)(0xA5 ^ i);

    return 0;
}

int wmain(void)
{
    struct ufprog_spi *spi = NULL;
    uint8_t protection;
    uint8_t config;
    uint8_t status;
    uint8_t data[TEST_LEN];
    int ret = 1;

    printf("=== ESMT F50L1G41LB SINGLE-PAGE PROGRAM/READ TEST ===\n");
    printf("WARNING: PAGE 0 WILL BE MODIFIED. NO ERASE IS PERFORMED.\n\n");

    /*
     * The ufprog controller/plugin must expose the CH341 SPI device.
     *
     * This uses the normal ufprog device name. If your installed
     * configuration exposes a different name, change this string.
     */
    printf("Opening SPI device 'ch341-libusb'...\n");

    if (ufprog_spi_open_device("ch341-libusb", false, &spi) != UFP_OK) {
        printf("ERROR: unable to open ch341-libusb\n");
        return 1;
    }

    printf("Opened interface device 'ch341-libusb'\n\n");

    if (ufprog_spi_set_mode(spi, 0) != UFP_OK) {
        printf("ERROR: unable to set SPI mode 0\n");
        goto cleanup;
    }

    if (ufprog_spi_set_speed_closest(spi, 1000000, NULL) != UFP_OK) {
        printf("WARNING: unable to set 1 MHz; continuing\n");
    }

    if (reset_chip(spi))
        goto cleanup;

    printf("\nINITIAL FEATURES\n");

    if (get_feature(spi, REG_PROTECTION, &protection))
        goto cleanup;

    if (get_feature(spi, REG_CONFIG, &config))
        goto cleanup;

    if (read_status(spi, &status))
        goto cleanup;

    printf("Protection A0: %02X\n", protection);
    printf("Configuration B0: %02X\n", config);
    print_status(status);

    printf("\nREAD EXISTING PAGE 0\n");

    if (page_read(spi, TEST_PAGE))
        goto cleanup;

    {
        uint8_t existing[TEST_LEN];

        memset(existing, 0, sizeof(existing));

        if (read_cache(spi, COLUMN, existing, sizeof(existing)))
            goto cleanup;

        printf("EXISTING DATA (first 64 bytes):\n");
        dump_data(existing, 64);
    }

    make_test_pattern(data, sizeof(data));

    printf("\nTEST PATTERN (first 64 bytes):\n");
    dump_data(data, 64);

    /*
     * Verify WEL before loading the cache.
     */
    printf("\nWRITE ENABLE\n");

    if (write_enable(spi))
        goto cleanup;

    if (read_status(spi, &status))
        goto cleanup;

    print_status(status);

    if (!(status & STATUS_WEL)) {
        printf("ERROR: WEL did not become 1\n");
        goto cleanup;
    }

    /*
     * PROGRAM LOAD
     */
    printf("\nPROGRAM LOAD 02h\n");
    printf("Column=%04X len=%zu\n", COLUMN, sizeof(data));

    if (program_load(spi, COLUMN, data, sizeof(data)))
        goto cleanup;

    /*
     * PROGRAM EXECUTE
     *
     * This is intentionally separate from program_load().
     */
    printf("\nPROGRAM EXECUTE\n");

    if (program_execute(spi, TEST_PAGE))
        goto cleanup;

    /*
     * Check final status, especially P_FAIL.
     */
    if (read_status(spi, &status))
        goto cleanup;

    printf("\nSTATUS AFTER PROGRAM EXECUTE\n");
    print_status(status);

    if (status & STATUS_P_FAIL) {
        printf("ERROR: NAND reported PROGRAM FAIL (P_FAIL=1)\n");
        goto cleanup;
    }

    if (status & STATUS_E_FAIL) {
        printf("WARNING: E_FAIL is set\n");
    }

    /*
     * Read the page back.
     */
    if (verify_page(spi, TEST_PAGE, data, sizeof(data)))
        goto cleanup;

    printf("\nFINAL FEATURES\n");

    if (get_feature(spi, REG_PROTECTION, &protection))
        goto cleanup;

    if (get_feature(spi, REG_CONFIG, &config))
        goto cleanup;

    if (read_status(spi, &status))
        goto cleanup;

    printf("Protection A0: %02X\n", protection);
    printf("Configuration B0: %02X\n", config);
    print_status(status);

    printf("\n=== TEST COMPLETE ===\n");

    ret = 0;

cleanup:

    if (spi)
        ufprog_spi_close_device(spi);

    return ret;
}