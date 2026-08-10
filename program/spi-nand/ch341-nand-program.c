/*
 * ch341-nand-write.c
 *
 * ESMT F50L1G41LB SPI NAND programmer
 *
 * BIN FILE:
 *     D:\prj\ufprog\stc_hardware_dump.bin
 *
 * Operations:
 *
 *   1. Open CH341
 *   2. Read NAND status
 *   3. ERASE NAND blocks required by image
 *   4. Program image page-by-page
 *   5. Verify every programmed page
 *   6. Progress bar only during normal operation
 *   7. Final summary
 *   8. Ctrl+C safe cancellation
 *   9. NAND reset during cleanup when possible
 *
 * NAND:
 *
 *   Manufacturer : ESMT
 *   Device       : F50L1G41LB
 *   Expected ID  : C8 01 7F 7F 7F
 *
 * Geometry:
 *
 *   Page size    : 2048 bytes
 *   Pages/block  : 64
 *   Block size   : 128 KiB
 *
 * Commands:
 *
 *   RESET        : FF
 *   READ ID     : 9F + dummy/address + data
 *   GET FEATURE  : 0F + address + data
 *   WRITE ENABLE : 06
 *   BLOCK ERASE  : D8 + row address
 *   PAGE PROGRAM : 02 + column + data
 *   READ PAGE    : 13 + row address
 *   READ CACHE   : 03 + column + dummy + data
 *
 * Feature registers:
 *
 *   A0 Protection
 *   B0 Configuration
 *   C0 Status
 *
 * This program performs destructive NAND operations.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <windows.h>

#include <ufprog/osdef.h>
#include <ufprog/log.h>
#include <ufprog/spi.h>


/* ============================================================
 * NAND GEOMETRY
 * ============================================================ */

#define NAND_PAGE_SIZE             2048U
#define NAND_SPARE_SIZE            64U
#define NAND_PAGES_PER_BLOCK       64U
#define NAND_BLOCK_SIZE            (NAND_PAGE_SIZE * NAND_PAGES_PER_BLOCK)

#define NAND_TOTAL_SIZE            (128U * 1024U * 1024U)

#define NAND_TOTAL_PAGES           (NAND_TOTAL_SIZE / NAND_PAGE_SIZE)
#define NAND_TOTAL_BLOCKS          (NAND_TOTAL_SIZE / NAND_BLOCK_SIZE)


/* ============================================================
 * INPUT FILE
 * ============================================================ */

#define IMAGE_PATH \
    "D:\\prj\\ufprog\\stc\\_hardware_dump.bin"


/* ============================================================
 * COMMANDS
 * ============================================================ */

#define CMD_RESET                  0xFF
#define CMD_READ_ID                0x9F

#define CMD_WRITE_ENABLE           0x06
#define CMD_WRITE_DISABLE          0x04

#define CMD_GET_FEATURE            0x0F
#define CMD_SET_FEATURE            0x1F

#define CMD_BLOCK_ERASE            0xD8
#define CMD_PAGE_PROGRAM           0x02

#define CMD_PAGE_READ              0x13
#define CMD_READ_CACHE             0x03


/* ============================================================
 * FEATURE REGISTERS
 * ============================================================ */

#define REG_PROTECTION             0xA0
#define REG_CONFIGURATION          0xB0
#define REG_STATUS                 0xC0


/* ============================================================
 * STATUS BITS
 * ============================================================ */

#define STATUS_OIP                 0x01
#define STATUS_WEL                 0x02
#define STATUS_EFAIL               0x04
#define STATUS_PFAIL               0x08

#define STATUS_ECC_MASK            0x30
#define STATUS_ECC_NONE            0x00
#define STATUS_ECC_CORRECTED       0x10
#define STATUS_ECC_UNCORRECTED     0x20
#define STATUS_ECC_RESERVED        0x30


/* ============================================================
 * EXPECTED ID
 * ============================================================ */

static const uint8_t expected_id[5] = {
    0xC8,
    0x01,
    0x7F,
    0x7F,
    0x7F
};


/* ============================================================
 * GLOBAL STATE
 * ============================================================ */

static volatile BOOL g_ctrl_c = FALSE;

static struct ufprog_spi *g_spi = NULL;


/* ============================================================
 * CTRL+C HANDLER
 *
 * IMPORTANT:
 *
 * Never perform SPI operations from this handler.
 *
 * The handler only sets the cancellation flag.
 * ============================================================ */

static BOOL WINAPI console_handler(DWORD signal)
{
    switch (signal) {

    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:

        g_ctrl_c = TRUE;

        return TRUE;

    default:
        return FALSE;
    }
}


/* ============================================================
 * SPI TRANSFER
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
 * SIMPLE COMMAND
 * ============================================================ */

static int command(struct ufprog_spi *spi,
                   uint8_t opcode)
{
    return transfer(spi,
                    &opcode,
                    1,
                    NULL,
                    0);
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

    if (transfer(spi,
                 tx,
                 sizeof(tx),
                 &rx,
                 1))
        return -1;

    *value = rx;

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

        if (get_feature(spi,
                        REG_STATUS,
                        &status))
            return -1;

        if (!(status & STATUS_OIP)) {

            if (final_status)
                *final_status = status;

            return 0;
        }

        /*
         * Ctrl+C is intentionally checked while waiting.
         *
         * We do NOT interrupt the SPI transaction.
         * We simply stop once the current NAND operation
         * has completed.
         */

        if (g_ctrl_c) {

            if (final_status)
                *final_status = status;

            return -3;
        }

        Sleep(1);

        elapsed++;
    }

    if (final_status)
        *final_status = status;

    return -2;
}


/* ============================================================
 * RESET NAND
 * ============================================================ */

static int reset_chip(struct ufprog_spi *spi)
{
    if (command(spi,
                CMD_RESET))
        return -1;

    Sleep(2);

    return 0;
}


/* ============================================================
 * READ ID
 *
 * Transaction:
 *
 *     9F
 *     00
 *     ID clocks
 *
 * The library's generic transfer supplies the read clocks.
 * ============================================================ */

static int read_id(struct ufprog_spi *spi,
                   uint8_t *id,
                   size_t len)
{
    uint8_t tx[2] = {
        CMD_READ_ID,
        0x00
    };

    if (!id || !len || len > 32)
        return -1;

    memset(id,
           0,
           len);

    return transfer(spi,
                    tx,
                    sizeof(tx),
                    id,
                    len);
}


/* ============================================================
 * WRITE ENABLE
 * ============================================================ */

static int write_enable(struct ufprog_spi *spi)
{
    uint8_t status = 0;

    if (command(spi,
                CMD_WRITE_ENABLE))
        return -1;

    /*
     * Confirm WEL.
     */

    if (get_feature(spi,
                    REG_STATUS,
                    &status))
        return -1;

    if (!(status & STATUS_WEL))
        return -1;

    return 0;
}


/* ============================================================
 * WRITE DISABLE
 * ============================================================ */

static int write_disable(struct ufprog_spi *spi)
{
    return command(spi,
                   CMD_WRITE_DISABLE);
}


/* ============================================================
 * BLOCK ERASE
 *
 * Row address:
 *
 *     24-bit page address
 *
 * The page address selects the block.
 * ============================================================ */

static int erase_block(struct ufprog_spi *spi,
                        uint32_t block)
{
    uint32_t row;

    uint8_t tx[4];

    uint8_t status = 0;

    row = block * NAND_PAGES_PER_BLOCK;

    if (write_enable(spi))
        return -1;

    tx[0] = CMD_BLOCK_ERASE;

    tx[1] = (uint8_t)((row >> 16) & 0xFF);
    tx[2] = (uint8_t)((row >> 8) & 0xFF);
    tx[3] = (uint8_t)(row & 0xFF);

    if (transfer(spi,
                 tx,
                 sizeof(tx),
                 NULL,
                 0)) {

        (void)write_disable(spi);

        return -1;
    }

    /*
     * Erase can take significantly longer than program.
     */

    if (wait_ready(spi,
                   10000,
                   &status)) {

        (void)write_disable(spi);

        return -1;
    }

    (void)write_disable(spi);

    if (status & STATUS_EFAIL)
        return -1;

    return 0;
}


/* ============================================================
 * PAGE PROGRAM
 *
 * Command:
 *
 *     02
 *     column MSB
 *     column LSB
 *     data...
 *
 * Column starts at zero.
 * ============================================================ */

static int program_page(struct ufprog_spi *spi,
                        uint32_t page,
                        const uint8_t *data,
                        size_t len)
{
    uint8_t *tx;

    size_t txlen;

    uint8_t status = 0;

    uint32_t row;

    if (!data)
        return -1;

    if (!len || len > NAND_PAGE_SIZE)
        return -1;

    row = page;

    /*
     * PAGE PROGRAM does not take a row address.
     *
     * The row is selected by PAGE READ (13h) into cache.
     *
     * For normal SPI NAND programming:
     *
     *     02 + column + data
     *
     * where the current cache corresponds to the target page.
     *
     * We therefore use column zero and issue the program
     * command after selecting the page through PAGE READ.
     */

    (void)row;

    txlen = 3 + len;

    tx = (uint8_t *)malloc(txlen);

    if (!tx)
        return -1;

    tx[0] = CMD_PAGE_PROGRAM;
    tx[1] = 0x00;
    tx[2] = 0x00;

    memcpy(&tx[3],
           data,
           len);

    if (write_enable(spi)) {

        free(tx);

        return -1;
    }

    if (transfer(spi,
                 tx,
                 txlen,
                 NULL,
                 0)) {

        free(tx);

        (void)write_disable(spi);

        return -1;
    }

    free(tx);

    if (wait_ready(spi,
                   5000,
                   &status)) {

        (void)write_disable(spi);

        return -1;
    }

    (void)write_disable(spi);

    if (status & STATUS_PFAIL)
        return -1;

    if (status & STATUS_ECC_UNCORRECTED)
        return -1;

    return 0;
}


/* ============================================================
 * PAGE READ
 *
 * Loads NAND page into cache.
 * ============================================================ */

static int page_read(struct ufprog_spi *spi,
                      uint32_t page)
{
    uint8_t tx[4];

    uint8_t status = 0;

    tx[0] = CMD_PAGE_READ;

    tx[1] = (uint8_t)((page >> 16) & 0xFF);
    tx[2] = (uint8_t)((page >> 8) & 0xFF);
    tx[3] = (uint8_t)(page & 0xFF);

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

    if (status & STATUS_ECC_UNCORRECTED)
        return -2;

    return 0;
}


/* ============================================================
 * READ CACHE
 *
 *     03
 *     column MSB
 *     column LSB
 *     dummy
 *     data...
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
 * PROGRAM + VERIFY PAGE
 * ============================================================ */

static int program_and_verify_page(struct ufprog_spi *spi,
                                   uint32_t page,
                                   const uint8_t *data,
                                   size_t len)
{
    uint8_t *verify;

    int ret;

    /*
     * First load the target page into cache.
     *
     * This is harmless/read-only and ensures the cache/row
     * state is explicitly selected before programming.
     *
     * For a normal SPI NAND page program, the program command
     * operates on the current cache contents.
     */

    if (page_read(spi,
                  page))
        return -1;

    /*
     * Program the supplied image page.
     */

    ret = program_page(spi,
                       page,
                       data,
                       len);

    if (ret)
        return ret;

    /*
     * Reload the programmed page.
     */

    if (page_read(spi,
                  page))
        return -1;

    verify = (uint8_t *)malloc(len);

    if (!verify)
        return -1;

    memset(verify,
           0,
           len);

    if (read_cache(spi,
                   0x0000,
                   verify,
                   len)) {

        free(verify);

        return -1;
    }

    ret = memcmp(verify,
                 data,
                 len);

    free(verify);

    if (ret)
        return -2;

    return 0;
}


/* ============================================================
 * PROGRESS BAR
 * ============================================================ */

static void progress_bar(uint64_t current,
                         uint64_t total,
                         const char *operation)
{
    const unsigned width = 50;

    unsigned filled;

    unsigned percent;

    uint64_t scaled;

    if (!total)
        return;

    if (current > total)
        current = total;

    scaled = current * 100ULL / total;

    percent = (unsigned)scaled;

    filled = (unsigned)(current * width / total);

    if (filled > width)
        filled = width;

    printf("\r%s [",
           operation);

    for (unsigned i = 0;
         i < width;
         i++) {

        if (i < filled)
            putchar('=');
        else if (i == filled)
            putchar('>');
        else
            putchar(' ');
    }

    printf("] %3u%%",
           percent);

    fflush(stdout);
}


/* ============================================================
 * LOAD IMAGE
 * ============================================================ */

static int load_image(const char *path,
                      uint8_t **data,
                      size_t *size)
{
    FILE *fp;

    long file_size;

    uint8_t *buffer;

    size_t read_size;

    if (!path || !data || !size)
        return -1;

    *data = NULL;
    *size = 0;

    fp = fopen(path,
               "rb");

    if (!fp)
        return -1;

    if (fseek(fp,
              0,
              SEEK_END)) {

        fclose(fp);

        return -1;
    }

    file_size = ftell(fp);

    if (file_size < 0) {

        fclose(fp);

        return -1;
    }

    if (fseek(fp,
              0,
              SEEK_SET)) {

        fclose(fp);

        return -1;
    }

    if ((uint64_t)file_size > NAND_TOTAL_SIZE) {

        fclose(fp);

        return -2;
    }

    if (file_size == 0) {

        fclose(fp);

        return -3;
    }

    buffer = (uint8_t *)malloc((size_t)file_size);

    if (!buffer) {

        fclose(fp);

        return -1;
    }

    read_size = fread(buffer,
                      1,
                      (size_t)file_size,
                      fp);

    fclose(fp);

    if (read_size != (size_t)file_size) {

        free(buffer);

        return -1;
    }

    *data = buffer;
    *size = read_size;

    return 0;
}


/* ============================================================
 * CALCULATE PAGE COUNT
 * ============================================================ */

static uint32_t image_page_count(size_t image_size)
{
    return (uint32_t)(
        (image_size + NAND_PAGE_SIZE - 1) /
        NAND_PAGE_SIZE
    );
}


/* ============================================================
 * CALCULATE BLOCK COUNT
 * ============================================================ */

static uint32_t image_block_count(size_t image_size)
{
    uint32_t pages;

    pages = image_page_count(image_size);

    return (pages + NAND_PAGES_PER_BLOCK - 1) /
           NAND_PAGES_PER_BLOCK;
}


/* ============================================================
 * ERASE IMAGE AREA
 *
 * Only blocks required by the image are erased.
 * ============================================================ */

static int erase_image_area(struct ufprog_spi *spi,
                            size_t image_size,
                            uint32_t *blocks_done)
{
    uint32_t blocks;

    uint32_t i;

    if (blocks_done)
        *blocks_done = 0;

    blocks = image_block_count(image_size);

    for (i = 0;
         i < blocks;
         i++) {

        if (g_ctrl_c)
            return -3;

        progress_bar(i,
                     blocks,
                     "Erasing ");

        if (erase_block(spi,
                        i)) {

            progress_bar(i,
                         blocks,
                         "Erasing ");

            printf("\n");

            return -1;
        }

        if (blocks_done)
            *blocks_done = i + 1;

        progress_bar(i + 1,
                     blocks,
                     "Erasing ");
    }

    printf("\n");

    return 0;
}


/* ============================================================
 * PROGRAM IMAGE
 * ============================================================ */

static int program_image(struct ufprog_spi *spi,
                         const uint8_t *image,
                         size_t image_size,
                         uint32_t *pages_done,
                         uint64_t *bytes_done)
{
    uint32_t pages;

    uint8_t page_buffer[NAND_PAGE_SIZE];

    if (pages_done)
        *pages_done = 0;

    if (bytes_done)
        *bytes_done = 0;

    pages = image_page_count(image_size);

    for (uint32_t page = 0;
         page < pages;
         page++) {

        size_t offset;

        size_t remaining;

        size_t len;

        int ret;

        if (g_ctrl_c)
            return -3;

        offset =
            (size_t)page *
            NAND_PAGE_SIZE;

        remaining =
            image_size -
            offset;

        len =
            remaining > NAND_PAGE_SIZE ?
            NAND_PAGE_SIZE :
            remaining;

        /*
         * Fill unused part of the final NAND page with FF.
         *
         * This avoids leaving stale data beyond the end of
         * the image inside the final page.
         */

        memset(page_buffer,
               0xFF,
               sizeof(page_buffer));

        memcpy(page_buffer,
               image + offset,
               len);

        /*
         * Program and verify the full NAND page.
         */

        ret = program_and_verify_page(spi,
                                       page,
                                       page_buffer,
                                       NAND_PAGE_SIZE);

        if (ret) {

            progress_bar(page,
                         pages,
                         "Writing ");

            printf("\n");

            return ret;
        }

        if (pages_done)
            *pages_done = page + 1;

        if (bytes_done) {

            uint64_t done =
                (uint64_t)(page + 1) *
                NAND_PAGE_SIZE;

            if (done > image_size)
                done = image_size;

            *bytes_done = done;
        }

        progress_bar(page + 1,
                     pages,
                     "Writing ");
    }

    printf("\n");

    return 0;
}


/* ============================================================
 * VERIFY COMPLETE IMAGE
 *
 * Full read-back verification.
 * ============================================================ */

static int verify_image(struct ufprog_spi *spi,
                        const uint8_t *image,
                        size_t image_size,
                        uint32_t *pages_done)
{
    uint32_t pages;

    uint8_t page_buffer[NAND_PAGE_SIZE];

    if (pages_done)
        *pages_done = 0;

    pages = image_page_count(image_size);

    for (uint32_t page = 0;
         page < pages;
         page++) {

        size_t offset;

        size_t remaining;

        size_t len;

        if (g_ctrl_c)
            return -3;

        offset =
            (size_t)page *
            NAND_PAGE_SIZE;

        remaining =
            image_size -
            offset;

        len =
            remaining > NAND_PAGE_SIZE ?
            NAND_PAGE_SIZE :
            remaining;

        memset(page_buffer,
               0,
               sizeof(page_buffer));

        if (page_read(spi,
                      page)) {

            progress_bar(page,
                         pages,
                         "Verify  ");

            printf("\n");

            return -1;
        }

        if (read_cache(spi,
                       0x0000,
                       page_buffer,
                       NAND_PAGE_SIZE)) {

            progress_bar(page,
                         pages,
                         "Verify  ");

            printf("\n");

            return -1;
        }

        if (memcmp(page_buffer,
                   image + offset,
                   len)) {

            progress_bar(page,
                         pages,
                         "Verify  ");

            printf("\n");

            return -2;
        }

        /*
         * For the final page, bytes after the image are expected
         * to remain FF because program_image filled the remainder
         * with FF.
         */

        if (len < NAND_PAGE_SIZE) {

            for (size_t i = len;
                 i < NAND_PAGE_SIZE;
                 i++) {

                if (page_buffer[i] != 0xFF) {

                    progress_bar(page,
                                 pages,
                                 "Verify  ");

                    printf("\n");

                    return -2;
                }
            }
        }

        if (pages_done)
            *pages_done = page + 1;

        progress_bar(page + 1,
                     pages,
                     "Verify  ");
    }

    printf("\n");

    return 0;
}


/* ============================================================
 * FINAL STATUS
 * ============================================================ */

static int get_final_status(struct ufprog_spi *spi,
                            uint8_t *status)
{
    if (!status)
        return -1;

    return get_feature(spi,
                       REG_STATUS,
                       status);
}


/* ============================================================
 * MAIN
 * ============================================================ */

int wmain(void)
{
    ufprog_status ret;

    struct ufprog_spi *spi = NULL;

    uint8_t *image = NULL;

    size_t image_size = 0;

    uint32_t erase_blocks_done = 0;

    uint32_t write_pages_done = 0;

    uint32_t verify_pages_done = 0;

    uint64_t bytes_done = 0;

    uint8_t final_status = 0;

    int result = 1;

    bool handler_installed = false;

    /*
     * --------------------------------------------------------
     * Ctrl+C handler
     * --------------------------------------------------------
     */

    if (SetConsoleCtrlHandler(console_handler,
                              TRUE)) {

        handler_installed = true;
    }

    /*
     * --------------------------------------------------------
     * UFPROG initialization
     * --------------------------------------------------------
     */

    set_os_default_log_print();

    os_init();

    /*
     * --------------------------------------------------------
     * Load image
     * --------------------------------------------------------
     */

    {
        int load_ret;

        load_ret =
            load_image(IMAGE_PATH,
                       &image,
                       &image_size);

        if (load_ret) {

            fprintf(stderr,
                    "ERROR: unable to load image: %s\n",
                    IMAGE_PATH);

            if (handler_installed)
                SetConsoleCtrlHandler(console_handler,
                                      FALSE);

            return 1;
        }
    }

    /*
     * --------------------------------------------------------
     * Open CH341
     * --------------------------------------------------------
     */

    printf("Opening CH341...\n");

    ret =
        ufprog_spi_open_device(
            "ch341-libusb",
            false,
            &spi);

    if (ret) {

        fprintf(stderr,
                "ERROR: CH341 open failed: %u\n",
                ret);

        free(image);

        if (handler_installed)
            SetConsoleCtrlHandler(console_handler,
                                  FALSE);

        return 1;
    }

    g_spi = spi;

    /*
     * --------------------------------------------------------
     * Check Ctrl+C
     * --------------------------------------------------------
     */

    if (g_ctrl_c) {

        result = 130;

        goto cleanup;
    }

    /*
     * --------------------------------------------------------
     * Reset NAND
     * --------------------------------------------------------
     */

    if (reset_chip(spi)) {

        fprintf(stderr,
                "ERROR: NAND reset failed.\n");

        result = 1;

        goto cleanup;
    }

    if (g_ctrl_c) {

        result = 130;

        goto cleanup;
    }

    /*
     * --------------------------------------------------------
     * Verify NAND ID
     *
     * We do not abort solely because the ID is wrong.
     *
     * The user requested recovery/write operation and the
     * diagnostic already established that the NAND responds
     * to feature/page commands.
     * --------------------------------------------------------
     */

    {
        uint8_t id[5] = {0};

        printf("Checking NAND ID...\n");

        if (read_id(spi,
                    id,
                    sizeof(id))) {

            fprintf(stderr,
                    "ERROR: unable to read NAND ID.\n");

            result = 1;

            goto cleanup;
        }

        printf("NAND ID:");

        for (unsigned i = 0;
             i < sizeof(id);
             i++) {

            printf(" %02X",
                   id[i]);
        }

        printf("\n");

        if (memcmp(id,
                   expected_id,
                   sizeof(expected_id)) == 0) {

            printf("NAND ID matches expected device.\n");

        } else {

            printf("WARNING: NAND ID does not match expected "
                   "F50L1G41LB ID.\n");

            printf("Expected: C8 01 7F 7F 7F\n");
            printf("Read    :");

            for (unsigned i = 0;
                 i < sizeof(id);
                 i++) {

                printf(" %02X",
                       id[i]);
            }

            printf("\n");
            printf("Continuing because the device is responding "
                   "to NAND commands.\n");
        }
    }

    if (g_ctrl_c) {

        result = 130;

        goto cleanup;
    }

    /*
     * --------------------------------------------------------
     * Read current status
     * --------------------------------------------------------
     */

    if (get_feature(spi,
                    REG_STATUS,
                    &final_status)) {

        fprintf(stderr,
                "ERROR: unable to read NAND status.\n");

        result = 1;

        goto cleanup;
    }

    if (final_status & STATUS_OIP) {

        if (wait_ready(spi,
                       10000,
                       &final_status)) {

            fprintf(stderr,
                    "ERROR: NAND did not become ready.\n");

            result = 1;

            goto cleanup;
        }
    }

    if (g_ctrl_c) {

        result = 130;

        goto cleanup;
    }

    /*
     * --------------------------------------------------------
     * ERASE
     * --------------------------------------------------------
     */

    if (erase_image_area(spi,
                         image_size,
                         &erase_blocks_done)) {

        if (g_ctrl_c) {

            result = 130;

            goto cleanup;
        }

        fprintf(stderr,
                "ERROR: NAND erase failed.\n");

        result = 1;

        goto cleanup;
    }

    /*
     * --------------------------------------------------------
     * Check cancellation after erase.
     * --------------------------------------------------------
     */

    if (g_ctrl_c) {

        result = 130;

        goto cleanup;
    }

    /*
     * --------------------------------------------------------
     * PROGRAM
     * --------------------------------------------------------
     */

    {
        int write_ret;

        write_ret =
            program_image(spi,
                          image,
                          image_size,
                          &write_pages_done,
                          &bytes_done);

        if (write_ret) {

            if (g_ctrl_c) {

                result = 130;

                goto cleanup;
            }

            fprintf(stderr,
                    "ERROR: NAND programming failed.\n");

            result = 1;

            goto cleanup;
        }
    }

    /*
     * --------------------------------------------------------
     * Check cancellation before verification.
     * --------------------------------------------------------
     */

    if (g_ctrl_c) {

        result = 130;

        goto cleanup;
    }

    /*
     * --------------------------------------------------------
     * FULL VERIFY
     * --------------------------------------------------------
     */

    {
        int verify_ret;

        verify_ret =
            verify_image(spi,
                         image,
                         image_size,
                         &verify_pages_done);

        if (verify_ret) {

            if (g_ctrl_c) {

                result = 130;

                goto cleanup;
            }

            fprintf(stderr,
                    "ERROR: NAND verification failed.\n");

            result = 2;

            goto cleanup;
        }
    }

    /*
     * --------------------------------------------------------
     * SUCCESS
     * --------------------------------------------------------
     */

    result = 0;


/* ============================================================
 * CLEANUP
 * ============================================================ */

cleanup:

    /*
     * If Ctrl+C occurred, wait for the NAND to finish any
     * operation that was already in progress.
     *
     * We deliberately do this before reset/close.
     */

    if (spi) {

        uint8_t status = 0;

        if (get_feature(spi,
                        REG_STATUS,
                        &status) == 0) {

            if (status & STATUS_OIP) {

                /*
                 * Give the currently running NAND operation
                 * time to complete.
                 */

                (void)wait_ready(spi,
                                 10000,
                                 &status);
            }
        }
    }

    /*
     * Final NAND reset.
     *
     * This is cleanup only.
     *
     * No erase/program operation is initiated here.
     */

    if (spi) {

        (void)reset_chip(spi);

        (void)get_final_status(spi,
                               &final_status);
    }

    /*
     * Close CH341.
     */

    if (spi) {

        ufprog_spi_close_device(spi);

        spi = NULL;

        g_spi = NULL;
    }

    /*
     * Free image.
     */

    if (image) {

        free(image);

        image = NULL;
    }

    /*
     * Remove Ctrl+C handler.
     */

    if (handler_installed) {

        SetConsoleCtrlHandler(console_handler,
                              FALSE);
    }


    /* ========================================================
     * FINAL SUMMARY
     * ======================================================== */

    printf("\n");
    printf("============================================================\n");
    printf(" NAND WRITE SUMMARY\n");
    printf("============================================================\n");

    printf("Image              : %s\n",
           IMAGE_PATH);

    printf("Image size         : %llu bytes\n",
           (unsigned long long)image_size);

    printf("Erase blocks       : %u / %u\n",
           erase_blocks_done,
           image_block_count(image_size));

    printf("Pages written      : %u / %u\n",
           write_pages_done,
           image_page_count(image_size));

    printf("Bytes written      : %llu / %llu\n",
           (unsigned long long)bytes_done,
           (unsigned long long)image_size);

    printf("Pages verified     : %u / %u\n",
           verify_pages_done,
           image_page_count(image_size));

    printf("Final C0 status    : %02X\n",
           final_status);

    printf("\n");

    if (result == 0) {

        printf("RESULT             : SUCCESS\n");
        printf("NAND image written and verified successfully.\n");

    } else if (result == 130) {

        printf("RESULT             : CANCELLED\n");
        printf("Ctrl+C was received.\n");
        printf("No new NAND operation was started after cancellation.\n");

    } else if (result == 2) {

        printf("RESULT             : VERIFY FAILED\n");
        printf("The written image could not be verified completely.\n");

    } else {

        printf("RESULT             : FAILED\n");
        printf("NAND programming did not complete successfully.\n");
    }

    printf("============================================================\n");

    return result;
}