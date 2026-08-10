/*
 * ch341-recover.c
 *
 * ESMT F50L1G41LB
 *
 * FULL MAIN-AREA RESTORE
 *
 * 1. Open CH341
 * 2. Reset
 * 3. READ AND VERIFY NAND ID
 * 4. Clear protection
 * 5. Verify protection is clear
 * 6. ERASE all 1024 blocks
 * 7. PROGRAM 65,536 pages from 128 MiB BIN
 * 8. IMMEDIATELY VERIFY each programmed page
 * 9. FINAL VERIFY of every page
 * 10. Retry erase/program/verify failures
 * 11. Report exact failed blocks/pages
 * 12. Final reset/status/ID
 *
 * BIN:
 * 128 MiB MAIN AREA ONLY
 * 134,217,728 bytes
 *
 * GEOMETRY:
 * Main page       = 2048 bytes
 * OOB             = 64 bytes
 * Pages/block     = 64
 * Blocks          = 1024
 * Total pages     = 65536
 *
 * The BIN does NOT contain OOB.
 *
 * Manufacturer command sequence:
 *
 * WRITE ENABLE       06h
 * BLOCK ERASE       D8h + row
 *
 * PROGRAM LOAD      02h + column + 2048 bytes
 * PROGRAM EXECUTE   10h + row
 *
 * PAGE READ         13h + row
 * READ CACHE        03h + column + dummy + data
 *
 * READ ID           9Fh + 00h + 5 bytes
 *
 * Expected ID:
 *
 * C8 01 7F 7F 7F
 *
 * GET FEATURE       0Fh + address
 * SET FEATURE       1Fh + address + value
 *
 * Status C0h:
 *
 * bit 5: ECC_S1
 * bit 4: ECC_S0
 * bit 3: P_FAIL
 * bit 2: E_FAIL
 * bit 1: WEL
 * bit 0: OIP
 *
 * ECC:
 *
 * 00 = no error
 * 10h = 1-bit corrected
 * 20h = 2-bit or more, uncorrectable
 * 30h = reserved
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <windows.h>
#include <time.h>

#include <ufprog/osdef.h>
#include <ufprog/log.h>
#include <ufprog/spi.h>

/* ============================================================
 * DEVICE GEOMETRY
 * ============================================================ */

#define NAND_BLOCKS             1024U
#define NAND_PAGES_PER_BLOCK   64U
#define NAND_TOTAL_PAGES       (NAND_BLOCKS * NAND_PAGES_PER_BLOCK)

#define NAND_PAGE_SIZE         2048U
#define NAND_OOB_SIZE          64U

#define NAND_MAIN_SIZE         ((size_t)NAND_TOTAL_PAGES * NAND_PAGE_SIZE)
#define NAND_BLOCK_MAIN_SIZE   ((size_t)NAND_PAGES_PER_BLOCK * NAND_PAGE_SIZE)

/* ============================================================
 * COMMANDS
 * ============================================================ */

#define CMD_WRITE_ENABLE       0x06
#define CMD_WRITE_DISABLE      0x04
#define CMD_RESET              0xFF

#define CMD_READ_ID            0x9F

#define CMD_GET_FEATURE        0x0F
#define CMD_SET_FEATURE        0x1F

#define CMD_PAGE_READ          0x13
#define CMD_READ_CACHE         0x03

#define CMD_PROGRAM_LOAD       0x02
#define CMD_PROGRAM_EXECUTE    0x10

#define CMD_BLOCK_ERASE        0xD8

/* ============================================================
 * FEATURE REGISTERS
 * ============================================================ */

#define REG_PROTECTION         0xA0
#define REG_CONFIGURATION      0xB0
#define REG_STATUS             0xC0

/* ============================================================
 * STATUS BITS
 * ============================================================ */

#define STATUS_OIP             0x01
#define STATUS_WEL             0x02
#define STATUS_EFAIL           0x04
#define STATUS_PFAIL           0x08

#define STATUS_ECC_MASK        0x30
#define STATUS_ECC_CORRECTED   0x10
#define STATUS_ECC_UNCORRECTED 0x20

/* ============================================================
 * EXPECTED NAND ID
 * ============================================================ */

#define NAND_ID_LENGTH         5U

static const uint8_t expected_nand_id[NAND_ID_LENGTH] = {
    0xC8,
    0x01,
    0x7F,
    0x7F,
    0x7F
};

/* ============================================================
 * RETRIES
 * ============================================================ */

#define ERASE_RETRIES          3
#define PROGRAM_RETRIES        3
#define VERIFY_RETRIES         2

/* ============================================================
 * BIN PATH
 * ============================================================ */

#define BIN_PATH "D:\\prj\\ufprog\\stc\\_hardware_dump.bin"

/* ============================================================
 * GLOBAL PROGRESS
 * ============================================================ */

static unsigned last_percent = 999;

/* ============================================================
 * CTRL-C / CONSOLE CONTROL
 * ============================================================ */

static volatile LONG stop_requested = 0;
static struct ufprog_spi *active_spi = NULL;

static BOOL WINAPI console_ctrl_handler(DWORD ctrl_type)
{
    switch (ctrl_type) {

    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:

        InterlockedExchange(&stop_requested, 1);

        return TRUE;

    default:
        return FALSE;
    }
}

static bool abort_requested(void)
{
    return InterlockedCompareExchange(&stop_requested,
                                      0,
                                      0) != 0;
}

/* ============================================================
 * FAILED PAGE TRACKING
 * ============================================================ */

static bool failed_erase_blocks[NAND_BLOCKS];
static bool failed_program_pages[NAND_TOTAL_PAGES];
static bool failed_immediate_verify_pages[NAND_TOTAL_PAGES];
static bool failed_final_verify_pages[NAND_TOTAL_PAGES];

static unsigned failed_erase_count = 0;
static unsigned failed_program_count = 0;
static unsigned failed_immediate_verify_count = 0;
static unsigned failed_final_verify_count = 0;

/* ============================================================
 * GENERIC SPI TRANSFER
 * ============================================================ */

static int transfer(struct ufprog_spi *spi,
                    const uint8_t *tx,
                    size_t txlen,
                    uint8_t *rx,
                    size_t rxlen)
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
        return 0;

    return ufprog_spi_generic_xfer(spi, xfer, n);
}

/* ============================================================
 * SEND COMMAND
 * ============================================================ */

static int command(struct ufprog_spi *spi, uint8_t opcode)
{
    return transfer(spi, &opcode, 1, NULL, 0);
}

/* ============================================================
 * GET FEATURE
 * ============================================================ */

static int get_feature(struct ufprog_spi *spi,
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

/* ============================================================
 * SET FEATURE
 * ============================================================ */

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

/* ============================================================
 * RESET
 * ============================================================ */

static int reset_chip(struct ufprog_spi *spi)
{
    if (command(spi, CMD_RESET))
        return -1;

    Sleep(2);

    return 0;
}

/* ============================================================
 * WAIT READY
 * ============================================================ */

static int wait_ready(struct ufprog_spi *spi,
                       unsigned timeout_ms,
                       uint8_t *final_status)
{
    unsigned elapsed = 0;
    uint8_t status = 0;

    while (elapsed < timeout_ms) {

        if (abort_requested())
            return -3;

        if (get_feature(spi, REG_STATUS, &status))
            return -1;

        if (!(status & STATUS_OIP)) {

            if (final_status)
                *final_status = status;

            return 0;
        }

        Sleep(1);
        elapsed++;
    }

    if (final_status)
        *final_status = status;

    return -2;
}

/* ============================================================
 * WRITE ENABLE
 * ============================================================ */

static int write_enable(struct ufprog_spi *spi)
{
    uint8_t status = 0;

    if (abort_requested())
        return -3;

    if (command(spi, CMD_WRITE_ENABLE))
        return -1;

    if (get_feature(spi, REG_STATUS, &status))
        return -1;

    if (!(status & STATUS_WEL))
        return -1;

    return 0;
}

/* ============================================================
 * WRITE DISABLE
 * ============================================================ */

static void write_disable(struct ufprog_spi *spi)
{
    if (spi)
        command(spi, CMD_WRITE_DISABLE);
}

/* ============================================================
 * CLEAR PROTECTION
 * ============================================================ */

static int clear_protection(struct ufprog_spi *spi)
{
    uint8_t protection = 0;

    if (get_feature(spi,
                    REG_PROTECTION,
                    &protection))
        return -1;

    if (protection == 0)
        return 0;

    if (write_enable(spi))
        return -1;

    if (set_feature(spi,
                    REG_PROTECTION,
                    0x00))
        return -1;

    if (wait_ready(spi,
                   1000,
                   NULL))
        return -1;

    if (get_feature(spi,
                    REG_PROTECTION,
                    &protection))
        return -1;

    if (protection != 0)
        return -1;

    return 0;
}

/* ============================================================
 * READ ID
 *
 * F50L1G41LB datasheet:
 *
 * 9Fh
 * followed by address 00h
 * followed by five read cycles
 *
 * Expected:
 *
 * C8h 01h 7Fh 7Fh 7Fh
 * ============================================================ */

static int read_id(struct ufprog_spi *spi)
{
    uint8_t tx[2] = {
        CMD_READ_ID,
        0x00
    };

    uint8_t rx[NAND_ID_LENGTH] = {0};

    if (transfer(spi,
                 tx,
                 sizeof(tx),
                 rx,
                 sizeof(rx)))
        return -1;

    printf("ID:");

    for (unsigned i = 0;
         i < NAND_ID_LENGTH;
         i++) {

        printf(" %02X",
               rx[i]);
    }

    printf("\n");

    if (memcmp(rx,
               expected_nand_id,
               NAND_ID_LENGTH) != 0)
        return -2;

    return 0;
}

/* ============================================================
 * BLOCK ERASE ONCE
 * ============================================================ */

static int erase_block_once(struct ufprog_spi *spi,
                            unsigned block)
{
    uint32_t row =
        block * NAND_PAGES_PER_BLOCK;

    uint8_t tx[4];
    uint8_t status = 0;

    tx[0] = CMD_BLOCK_ERASE;
    tx[1] = (uint8_t)((row >> 16) & 0xFF);
    tx[2] = (uint8_t)((row >> 8) & 0xFF);
    tx[3] = (uint8_t)(row & 0xFF);

    /*
     * Manufacturer requires WEL before BLOCK ERASE.
     */

    if (write_enable(spi))
        return -1;

    if (transfer(spi,
                 tx,
                 sizeof(tx),
                 NULL,
                 0))
        return -1;

    if (wait_ready(spi,
                   10000,
                   &status))
        return -1;

    if (status & STATUS_EFAIL)
        return -1;

    return 0;
}

/* ============================================================
 * BLOCK ERASE WITH RETRIES
 * ============================================================ */

static int erase_block(struct ufprog_spi *spi,
                       unsigned block)
{
    for (unsigned attempt = 0;
         attempt < ERASE_RETRIES;
         attempt++) {

        if (abort_requested())
            return -3;

        if (attempt) {

            reset_chip(spi);

            /*
             * Re-clear protection after reset just in case.
             */

            clear_protection(spi);
        }

        if (erase_block_once(spi,
                             block) == 0)
            return 0;
    }

    return -1;
}

/* ============================================================
 * PROGRAM LOAD
 *
 * 02h + column + 2048 bytes
 * ============================================================ */

static int program_load(struct ufprog_spi *spi,
                        const uint8_t *data)
{
    uint8_t *tx;

    size_t txlen =
        3 + NAND_PAGE_SIZE;

    tx =
        (uint8_t *)malloc(txlen);

    if (!tx)
        return -1;

    tx[0] = CMD_PROGRAM_LOAD;
    tx[1] = 0x00;
    tx[2] = 0x00;

    memcpy(&tx[3],
           data,
           NAND_PAGE_SIZE);

    int ret =
        transfer(spi,
                 tx,
                 txlen,
                 NULL,
                 0);

    free(tx);

    return ret;
}

/* ============================================================
 * PROGRAM PAGE ONCE
 *
 * Correct sequence:
 *
 * WRITE ENABLE
 * PROGRAM LOAD
 * PROGRAM EXECUTE
 * WAIT READY
 * CHECK P_FAIL
 * ============================================================ */

static int program_page_once(struct ufprog_spi *spi,
                             unsigned page,
                             const uint8_t *data)
{
    uint32_t row = page;

    uint8_t tx_exec[4];
    uint8_t status = 0;

    /*
     * WEL must be set before PROGRAM EXECUTE.
     */

    if (write_enable(spi))
        return -1;

    /*
     * Load 2048 bytes into the NAND cache.
     */

    if (program_load(spi,
                     data))
        return -1;

    /*
     * PROGRAM EXECUTE.
     */

    tx_exec[0] = CMD_PROGRAM_EXECUTE;
    tx_exec[1] = (uint8_t)((row >> 16) & 0xFF);
    tx_exec[2] = (uint8_t)((row >> 8) & 0xFF);
    tx_exec[3] = (uint8_t)(row & 0xFF);

    if (transfer(spi,
                 tx_exec,
                 sizeof(tx_exec),
                 NULL,
                 0))
        return -1;

    /*
     * Wait for PROGRAM EXECUTE to finish.
     */

    if (wait_ready(spi,
                   10000,
                   &status))
        return -1;

    /*
     * P_FAIL = 1 means programming failed.
     */

    if (status & STATUS_PFAIL)
        return -1;

    return 0;
}

/* ============================================================
 * PROGRAM PAGE WITH RETRIES
 * ============================================================ */

static int program_page(struct ufprog_spi *spi,
                        unsigned page,
                        const uint8_t *data)
{
    for (unsigned attempt = 0;
         attempt < PROGRAM_RETRIES;
         attempt++) {

        if (abort_requested())
            return -3;

        if (attempt) {

            reset_chip(spi);

            /*
             * Protection is normally volatile/non-locked,
             * but verify it again after reset.
             */

            clear_protection(spi);
        }

        if (program_page_once(spi,
                              page,
                              data) == 0)
            return 0;
    }

    return -1;
}

/* ============================================================
 * PAGE READ
 *
 * 13h + 24-bit row
 * ============================================================ */

static int page_read(struct ufprog_spi *spi,
                     unsigned page)
{
    uint32_t row = page;

    uint8_t tx[4];

    uint8_t status = 0;

    tx[0] = CMD_PAGE_READ;

    tx[1] = (uint8_t)((row >> 16) & 0xFF);
    tx[2] = (uint8_t)((row >> 8) & 0xFF);
    tx[3] = (uint8_t)(row & 0xFF);

    if (transfer(spi,
                 tx,
                 sizeof(tx),
                 NULL,
                 0))
        return -1;

    if (wait_ready(spi,
                   10000,
                   &status))
        return -1;

    /*
     * ECC status is evaluated after READ CACHE,
     * not here.
     */

    return 0;
}

/* ============================================================
 * READ CACHE
 *
 * 03h
 * column MSB
 * column LSB
 * dummy
 * data
 * ============================================================ */

static int read_cache(struct ufprog_spi *spi,
                      uint16_t column,
                      uint8_t *data,
                      size_t len)
{
    uint8_t tx[4];

    tx[0] = CMD_READ_CACHE;
    tx[1] = (uint8_t)((column >> 8) & 0xFF);
    tx[2] = (uint8_t)(column & 0xFF);
    tx[3] = 0x00;

    return transfer(spi,
                    tx,
                    sizeof(tx),
                    data,
                    len);
}

/* ============================================================
 * READ PAGE MAIN AREA
 *
 * Returns:
 *
 * 0  = successful read
 * -1 = communication failure
 * -2 = uncorrectable ECC
 * ============================================================ */

static int read_page_main(struct ufprog_spi *spi,
                          unsigned page,
                          uint8_t *data)
{
    uint8_t status = 0;

    if (page_read(spi,
                  page))
        return -1;

    if (read_cache(spi,
                   0x0000,
                   data,
                   NAND_PAGE_SIZE))
        return -1;

    if (get_feature(spi,
                    REG_STATUS,
                    &status))
        return -1;

    /*
     * ESMT F50L1G41LB:
     *
     * 00 = no errors
     * 10h = 1-bit corrected
     * 20h = 2-bit or more, uncorrectable
     * 30h = reserved
     *
     * A corrected 1-bit error is acceptable for comparison
     * because the data returned by the NAND ECC is corrected.
     */

    if ((status & STATUS_ECC_MASK) ==
        STATUS_ECC_UNCORRECTED)
        return -2;

    return 0;
}

/* ============================================================
 * COMPARE PAGE
 * ============================================================ */

static int compare_page(const uint8_t *a,
                        const uint8_t *b,
                        size_t *first_bad_offset,
                        size_t *bad_count)
{
    size_t first = SIZE_MAX;
    size_t bad = 0;

    for (size_t i = 0;
         i < NAND_PAGE_SIZE;
         i++) {

        if (a[i] != b[i]) {

            if (first == SIZE_MAX)
                first = i;

            bad++;
        }
    }

    if (first_bad_offset)
        *first_bad_offset = first;

    if (bad_count)
        *bad_count = bad;

    return bad ? -1 : 0;
}

/* ============================================================
 * VERIFY ONE PAGE
 * ============================================================ */

static int verify_page(struct ufprog_spi *spi,
                       unsigned page,
                       const uint8_t *expected,
                       uint8_t *buffer,
                       size_t *first_bad,
                       size_t *bad_count)
{
    for (unsigned attempt = 0;
         attempt < VERIFY_RETRIES;
         attempt++) {

        if (abort_requested())
            return -3;

        if (attempt)
            reset_chip(spi);

        int ret =
            read_page_main(spi,
                           page,
                           buffer);

        if (ret)
            continue;

        if (compare_page(buffer,
                         expected,
                         first_bad,
                         bad_count) == 0)
            return 0;
    }

    return -1;
}

/* ============================================================
 * PROGRESS BAR
 *
 * Only one line changes.
 * No scrolling during normal operation.
 * ============================================================ */

static void progress(const char *stage,
                     unsigned current,
                     unsigned total)
{
    unsigned percent;

    if (!total)
        percent = 100;
    else
        percent =
            (unsigned)(((uint64_t)current * 100ULL) /
                       (uint64_t)total);

    if (percent > 100)
        percent = 100;

    if (percent == last_percent)
        return;

    last_percent = percent;

    unsigned width = 30;

    unsigned filled =
        (percent * width) / 100;

    printf("\r%-16s [",
           stage);

    for (unsigned i = 0;
         i < width;
         i++) {

        putchar(i < filled ? '#' : '.');
    }

    printf("] %3u%%",
           percent);

    fflush(stdout);
}

/* ============================================================
 * TIME FORMAT
 * ============================================================ */

static void print_elapsed(const char *name,
                          double seconds)
{
    unsigned total =
        (unsigned)(seconds + 0.5);

    unsigned hours =
        total / 3600;

    unsigned minutes =
        (total % 3600) / 60;

    unsigned secs =
        total % 60;

    printf("%-18s: %02u:%02u:%02u\n",
           name,
           hours,
           minutes,
           secs);
}

/* ============================================================
 * LOAD BIN
 * ============================================================ */

static int load_bin(const char *path,
                    uint8_t **buffer)
{
    FILE *fp;
    uint8_t *buf;
    size_t got;

    fp = fopen(path, "rb");

    if (!fp) {

        fprintf(stderr,
                "\nERROR: cannot open BIN:\n%s\n",
                path);

        return -1;
    }

    if (fseek(fp,
              0,
              SEEK_END)) {

        fclose(fp);
        return -1;
    }

    long size = ftell(fp);

    if (size < 0) {

        fclose(fp);
        return -1;
    }

    if ((size_t)size != NAND_MAIN_SIZE) {

        fprintf(stderr,
                "\nERROR: incorrect BIN size.\n"
                "Actual   : %ld bytes\n"
                "Expected : %zu bytes\n",
                size,
                NAND_MAIN_SIZE);

        fclose(fp);
        return -1;
    }

    if (fseek(fp,
              0,
              SEEK_SET)) {

        fclose(fp);
        return -1;
    }

    buf =
        (uint8_t *)malloc(NAND_MAIN_SIZE);

    if (!buf) {

        fclose(fp);

        fprintf(stderr,
                "\nERROR: unable to allocate BIN buffer.\n");

        return -1;
    }

    got =
        fread(buf,
              1,
              NAND_MAIN_SIZE,
              fp);

    fclose(fp);

    if (got != NAND_MAIN_SIZE) {

        free(buf);

        fprintf(stderr,
                "\nERROR: incomplete BIN read.\n");

        return -1;
    }

    *buffer = buf;

    return 0;
}

/* ============================================================
 * CLEANUP
 * ============================================================ */

static void cleanup(struct ufprog_spi *spi)
{
    if (!spi)
        return;

    /*
     * Stop accepting writes.
     */

    write_disable(spi);

    /*
     * Leave the NAND in a known idle state.
     */

    reset_chip(spi);

    /*
     * Close the CH341 device.
     */

    ufprog_spi_close_device(spi);
}

/* ============================================================
 * MAIN
 * ============================================================ */

int wmain(void)
{
    struct ufprog_spi *spi = NULL;

    ufprog_status ret;

    uint8_t *bin = NULL;
    uint8_t *page_buffer = NULL;

    uint8_t protection = 0;
    uint8_t configuration = 0;
    uint8_t status = 0;

    unsigned erase_failed = 0;
    unsigned program_failed = 0;
    unsigned immediate_verify_failed = 0;
    unsigned final_verify_failed = 0;

    unsigned pages_programmed = 0;
    unsigned pages_immediately_verified = 0;
    unsigned pages_final_verified = 0;

    bool interrupted = false;

    LARGE_INTEGER freq;
    LARGE_INTEGER t_start;
    LARGE_INTEGER t_erase_start;
    LARGE_INTEGER t_program_start;
    LARGE_INTEGER t_final_verify_start;
    LARGE_INTEGER t_end;

    double erase_seconds = 0.0;
    double program_seconds = 0.0;
    double final_verify_seconds = 0.0;
    double total_seconds = 0.0;

    SetConsoleCtrlHandler(console_ctrl_handler,
                          TRUE);

    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t_start);

    /* ========================================================
     * LOAD BIN
     * ======================================================== */

    if (load_bin(BIN_PATH,
                 &bin))
        goto cleanup_before_open;

    page_buffer =
        (uint8_t *)malloc(NAND_PAGE_SIZE);

    if (!page_buffer) {

        fprintf(stderr,
                "\nERROR: unable to allocate page buffer.\n");

        goto cleanup_before_open;
    }

    /* ========================================================
     * UFPROG INITIALIZATION
     * ======================================================== */

    set_os_default_log_print();

    os_init();

    /* ========================================================
     * OPEN CH341
     * ======================================================== */

    ret =
        ufprog_spi_open_device(
            "ch341-libusb",
            false,
            &spi);

    if (ret) {

        fprintf(stderr,
                "\nERROR: CH341 open failed: %u\n",
                ret);

        goto cleanup_before_open;
    }

    active_spi = spi;

    /* ========================================================
     * RESET
     * ======================================================== */

    if (reset_chip(spi)) {

        fprintf(stderr,
                "\nERROR: initial RESET failed.\n");

        goto cleanup;
    }

    /* ========================================================
     * NAND ID CHECK
     *
     * MUST OCCUR BEFORE ANY ERASE.
     * ======================================================== */

    printf("\nChecking NAND ID before erase...\n");

    int id_ret = read_id(spi);

    if (id_ret == -1) {

        fprintf(stderr,
                "\nERROR: NAND ID read communication failed.\n"
                "NO ERASE HAS BEEN PERFORMED.\n");

        goto cleanup;
    }

    if (id_ret == -2) {

        fprintf(stderr,
                "\nERROR: unexpected NAND ID.\n"
                "Expected: C8 01 7F 7F 7F\n");

        /*
         * Read the ID again only for reporting.
         */

        uint8_t tx_id[2] = {
            CMD_READ_ID,
            0x00
        };

        uint8_t rx_id[NAND_ID_LENGTH] = {0};

        if (transfer(spi,
                     tx_id,
                     sizeof(tx_id),
                     rx_id,
                     sizeof(rx_id)) == 0) {

            printf("Read    :");

            for (unsigned i = 0;
                 i < NAND_ID_LENGTH;
                 i++) {

                printf(" %02X",
                       rx_id[i]);
            }

            printf("\n");
        }

        fprintf(stderr,
                "\nERROR: NAND ID verification failed.\n"
                "NO ERASE HAS BEEN PERFORMED.\n");

        goto cleanup;
    }

    printf("NAND ID verified: C8 01 7F 7F 7F\n");

    /* ========================================================
     * CLEAR PROTECTION
     * ======================================================== */

    if (clear_protection(spi)) {

        fprintf(stderr,
                "\nERROR: unable to clear protection.\n");

        goto cleanup;
    }

    /*
     * Verify protection is really zero.
     */

    if (get_feature(spi,
                    REG_PROTECTION,
                    &protection)) {

        fprintf(stderr,
                "\nERROR: cannot read protection register.\n");

        goto cleanup;
    }

    if (protection != 0) {

        fprintf(stderr,
                "\nERROR: protection register remains %02X.\n",
                protection);

        goto cleanup;
    }

    /* ========================================================
     * STAGE 1 — ERASE
     * ======================================================== */

    QueryPerformanceCounter(&t_erase_start);

    last_percent = 999;

    for (unsigned block = 0;
         block < NAND_BLOCKS;
         block++) {

        if (abort_requested()) {
            interrupted = true;
            break;
        }

        progress("STAGE 1/3 ERASE",
                 block + 1,
                 NAND_BLOCKS);

        int erase_ret =
            erase_block(spi,
                        block);

        if (erase_ret == -3) {
            interrupted = true;
            break;
        }

        if (erase_ret) {

            failed_erase_blocks[block] = true;

            erase_failed++;
            failed_erase_count++;
        }
    }

    printf("\n");

    if (interrupted) {

        fprintf(stderr,
                "\n*** CTRL-C RECEIVED ***\n"
                "Stopping safely before further NAND operations.\n");

        goto cleanup;
    }

    QueryPerformanceCounter(&t_program_start);

    erase_seconds =
        (double)(t_program_start.QuadPart -
                 t_erase_start.QuadPart) /
        (double)freq.QuadPart;

    /* ========================================================
     * STAGE 2 — PROGRAM + IMMEDIATE VERIFY
     * ======================================================== */

    last_percent = 999;

    for (unsigned page = 0;
         page < NAND_TOTAL_PAGES;
         page++) {

        if (abort_requested()) {
            interrupted = true;
            break;
        }

        progress("STAGE 2/3 PROGRAM",
                 page + 1,
                 NAND_TOTAL_PAGES);

        const uint8_t *expected =
            bin +
            ((size_t)page * NAND_PAGE_SIZE);

        /*
         * PROGRAM
         */

        int program_ret =
            program_page(spi,
                         page,
                         expected);

        if (program_ret == -3) {
            interrupted = true;
            break;
        }

        if (program_ret) {

            failed_program_pages[page] = true;

            program_failed++;
            failed_program_count++;

            /*
             * Do not count this page as immediately
             * verified because it was never successfully
             * programmed.
             */

            continue;
        }

        pages_programmed++;

        /*
         * IMMEDIATE READBACK VERIFY
         *
         * This is deliberately performed directly after
         * PROGRAM EXECUTE.
         */

        size_t first_bad = SIZE_MAX;
        size_t bad_count = 0;

        int verify_ret =
            verify_page(spi,
                        page,
                        expected,
                        page_buffer,
                        &first_bad,
                        &bad_count);

        if (verify_ret == -3) {
            interrupted = true;
            break;
        }

        if (verify_ret) {

            failed_immediate_verify_pages[page] = true;

            immediate_verify_failed++;
            failed_immediate_verify_count++;

        } else {

            pages_immediately_verified++;
        }
    }

    printf("\n");

    if (interrupted) {

        fprintf(stderr,
                "\n*** CTRL-C RECEIVED ***\n"
                "Stopping safely during Stage 2.\n");

        goto cleanup;
    }

    QueryPerformanceCounter(&t_final_verify_start);

    program_seconds =
        (double)(t_final_verify_start.QuadPart -
                 t_program_start.QuadPart) /
        (double)freq.QuadPart;

    /* ========================================================
     * STAGE 3 — FINAL FULL VERIFY
     * ======================================================== */

    last_percent = 999;

    for (unsigned page = 0;
         page < NAND_TOTAL_PAGES;
         page++) {

        if (abort_requested()) {
            interrupted = true;
            break;
        }

        progress("STAGE 3/3 VERIFY",
                 page + 1,
                 NAND_TOTAL_PAGES);

        const uint8_t *expected =
            bin +
            ((size_t)page * NAND_PAGE_SIZE);

        size_t first_bad = SIZE_MAX;
        size_t bad_count = 0;

        int verify_ret =
            verify_page(spi,
                        page,
                        expected,
                        page_buffer,
                        &first_bad,
                        &bad_count);

        if (verify_ret == -3) {
            interrupted = true;
            break;
        }

        if (verify_ret) {

            failed_final_verify_pages[page] = true;

            final_verify_failed++;
            failed_final_verify_count++;

        } else {

            pages_final_verified++;
        }
    }

    printf("\n");

    if (interrupted) {

        fprintf(stderr,
                "\n*** CTRL-C RECEIVED ***\n"
                "Stopping safely during Stage 3.\n");

        goto cleanup;
    }

    QueryPerformanceCounter(&t_end);

    final_verify_seconds =
        (double)(t_end.QuadPart -
                 t_final_verify_start.QuadPart) /
        (double)freq.QuadPart;

    total_seconds =
        (double)(t_end.QuadPart -
                 t_start.QuadPart) /
        (double)freq.QuadPart;

    /* ========================================================
     * FINAL RESET
     * ======================================================== */

    reset_chip(spi);

    /* ========================================================
     * FINAL FEATURES
     * ======================================================== */

    get_feature(spi,
                REG_PROTECTION,
                &protection);

    get_feature(spi,
                REG_CONFIGURATION,
                &configuration);

    get_feature(spi,
                REG_STATUS,
                &status);

    /* ========================================================
     * FINAL ID
     * ======================================================== */

    printf("\nFinal NAND ID:\n");

    read_id(spi);

    /* ========================================================
     * FINAL SUMMARY
     * ======================================================== */

    printf("\n");
    printf("============================================================\n");
    printf(" FULL BIN RESTORE SUMMARY\n");
    printf("============================================================\n");

    printf("BIN                  : %s\n",
           BIN_PATH);

    printf("BIN size             : %zu bytes\n",
           NAND_MAIN_SIZE);

    printf("Blocks               : %u\n",
           NAND_BLOCKS);

    printf("Pages                : %u\n",
           NAND_TOTAL_PAGES);

    printf("Page size            : %u bytes\n",
           NAND_PAGE_SIZE);

    printf("Block size           : %llu KiB\n",
           (unsigned long long)
           (NAND_BLOCK_MAIN_SIZE / 1024ULL));

    printf("\n");

    printf("Pages programmed     : %u / %u\n",
           pages_programmed,
           NAND_TOTAL_PAGES);

    printf("Immediate verified   : %u / %u\n",
           pages_immediately_verified,
           NAND_TOTAL_PAGES);

    printf("Final verified       : %u / %u\n",
           pages_final_verified,
           NAND_TOTAL_PAGES);

    printf("\n");

    printf("Erase failures       : %u\n",
           erase_failed);

    printf("Program failures     : %u\n",
           program_failed);

    printf("Immediate verify     : %u\n",
           immediate_verify_failed);

    printf("Final verify         : %u\n",
           final_verify_failed);

    printf("\n");

    print_elapsed("Erase time",
                  erase_seconds);

    print_elapsed("Program+verify",
                  program_seconds);

    print_elapsed("Final verify",
                  final_verify_seconds);

    print_elapsed("TOTAL TIME",
                  total_seconds);

    /* ========================================================
     * FINAL STATUS
     * ======================================================== */

    printf("\n");

    printf("Final A0             : %02X\n",
           protection);

    printf("Final B0             : %02X\n",
           configuration);

    printf("Final C0             : %02X\n",
           status);

    printf("OIP                  : %u\n",
           (status & STATUS_OIP) ? 1U : 0U);

    printf("WEL                  : %u\n",
           (status & STATUS_WEL) ? 1U : 0U);

    printf("E_FAIL               : %u\n",
           (status & STATUS_EFAIL) ? 1U : 0U);

    printf("P_FAIL               : %u\n",
           (status & STATUS_PFAIL) ? 1U : 0U);

    printf("ECC status           : %02X\n",
           status & STATUS_ECC_MASK);

    /* ========================================================
     * FAILED BLOCK SUMMARY
     * ======================================================== */

    bool any_failed_block = false;

    for (unsigned block = 0;
         block < NAND_BLOCKS;
         block++) {

        bool failed =
            failed_erase_blocks[block];

        unsigned first_page =
            block * NAND_PAGES_PER_BLOCK;

        unsigned last_page =
            first_page +
            NAND_PAGES_PER_BLOCK - 1;

        for (unsigned page = first_page;
             page <= last_page;
             page++) {

            if (failed_program_pages[page] ||
                failed_immediate_verify_pages[page] ||
                failed_final_verify_pages[page]) {

                failed = true;
                break;
            }
        }

        if (failed) {
            any_failed_block = true;
            break;
        }
    }

    if (any_failed_block) {

        printf("\n");
        printf("*** FAILED BLOCK LIST ***\n");

        for (unsigned block = 0;
             block < NAND_BLOCKS;
             block++) {

            bool erase_fail =
                failed_erase_blocks[block];

            unsigned first_page =
                block * NAND_PAGES_PER_BLOCK;

            unsigned last_page =
                first_page +
                NAND_PAGES_PER_BLOCK - 1;

            bool block_has_failure =
                erase_fail;

            for (unsigned page = first_page;
                 page <= last_page;
                 page++) {

                if (failed_program_pages[page] ||
                    failed_immediate_verify_pages[page] ||
                    failed_final_verify_pages[page]) {

                    block_has_failure = true;
                    break;
                }
            }

            if (!block_has_failure)
                continue;

            printf("\nBlock %4u  pages %5u-%-5u\n",
                   block,
                   first_page,
                   last_page);

            if (erase_fail)
                printf("  ERASE: FAIL\n");

            /*
             * PROGRAM failures
             */

            for (unsigned page = first_page;
                 page <= last_page;
                 page++) {

                if (failed_program_pages[page]) {

                    printf("  Page %5u: PROGRAM FAIL\n",
                           page);
                }
            }

            /*
             * Immediate verification failures
             */

            for (unsigned page = first_page;
                 page <= last_page;
                 page++) {

                if (failed_immediate_verify_pages[page]) {

                    printf("  Page %5u: IMMEDIATE VERIFY FAIL\n",
                           page);
                }
            }

            /*
             * Final verification failures
             */

            for (unsigned page = first_page;
                 page <= last_page;
                 page++) {

                if (failed_final_verify_pages[page]) {

                    printf("  Page %5u: FINAL VERIFY FAIL\n",
                           page);
                }
            }
        }
    }

    /* ========================================================
     * FINAL RESULT
     * ======================================================== */

    printf("\n");

    if (erase_failed ||
        program_failed ||
        immediate_verify_failed ||
        final_verify_failed) {

        printf("*** RESTORE COMPLETED WITH FAILURES ***\n");

    } else {

        printf("*** FULL BIN RESTORE VERIFIED SUCCESSFULLY ***\n");
    }

    printf("\n");
    printf("============================================================\n");
    printf(" END\n");
    printf("============================================================\n");

    /* ========================================================
     * CLEANUP
     * ======================================================== */

    cleanup(spi);

    active_spi = NULL;

    free(page_buffer);
    free(bin);

    SetConsoleCtrlHandler(console_ctrl_handler,
                           FALSE);

    if (erase_failed ||
        program_failed ||
        immediate_verify_failed ||
        final_verify_failed)
        return 2;

    return 0;

/* ============================================================
 * CLEANUP AFTER OPEN / BEFORE OPEN
 * ============================================================ */

cleanup:
    cleanup(spi);

    active_spi = NULL;

    free(page_buffer);
    free(bin);

    SetConsoleCtrlHandler(console_ctrl_handler,
                           FALSE);

    if (abort_requested()) {
        fprintf(stderr,
                "\n*** CLEANUP COMPLETE — NAND LEFT IN RESET/IDLE STATE ***\n");
        return 130;
    }

    return 1;

cleanup_before_open:
    free(page_buffer);
    free(bin);

    SetConsoleCtrlHandler(console_ctrl_handler,
                           FALSE);

    return 1;
}