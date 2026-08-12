/*
 * ch341-recover.c
 *
 * ESMT F50L1G41LB
 *
 * FULL MAIN-AREA RESTORE
 *
 * IMPORTANT:
 *   This program deliberately uses UFProg's SPI abstraction.
 *   It does NOT attempt to imitate SNANDer.
 *
 * F50L1G41LB:
 *
 *   JEDEC READ ID:
 *       9Fh
 *       00h
 *       5 read cycles
 *
 *   Expected:
 *       C8 01 7F 7F 7F
 *
 * Geometry:
 *
 *   Main page       2048 bytes
 *   OOB             64 bytes
 *   Pages/block     64
 *   Blocks          1024
 *   Total pages     65536
 *   Main capacity   134217728 bytes
 *
 * ECC:
 *
 *   Internal ECC is enabled after power-up.
 *   PROGRAM causes the NAND itself to calculate/store ECC.
 *   READ causes the NAND itself to correct data.
 *
 * Bad blocks:
 *
 *   Factory bad-block marker:
 *       column 0x800
 *       page 0 of block
 *       page 1 of block
 *
 *   Marker != FFh means bad block.
 *
 * IMPORTANT RESTORE POLICY:
 *
 *   - ID must pass before any destructive operation.
 *   - Factory-marked bad blocks are NEVER erased/programmed.
 *   - Physical page numbering is NOT compressed/repacked.
 *     The BIN remains mapped to its original physical page numbers.
 *   - This preserves the physical layout of the 128 MiB dump.
 *
 * Restore:
 *
 *   1. Open CH341 through UFProg
 *   2. Reset
 *   3. Repeated raw ID verification
 *   4. Read/report configuration/status
 *   5. Scan factory bad-block markers
 *   6. Clear protection
 *   7. Erase all GOOD blocks only
 *   8. Program all pages in GOOD blocks
 *   9. Immediately verify each programmed page
 *  10. Final full verification of all programmed pages
 *  11. Final reset/status/configuration/ID
 *
 * BIN:
 *
 *   128 MiB MAIN AREA ONLY
 *   134217728 bytes
 *
 * No OOB data is required for ECC generation.
 * The F50L1G41LB generates its ECC internally during PROGRAM.
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

#define NAND_MAIN_SIZE \
    ((size_t)NAND_TOTAL_PAGES * NAND_PAGE_SIZE)

#define NAND_BLOCK_MAIN_SIZE \
    ((size_t)NAND_PAGES_PER_BLOCK * NAND_PAGE_SIZE)


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
 * BAD BLOCK MARKER
 * ============================================================ */

#define BAD_BLOCK_COLUMN       0x0800U
#define BAD_BLOCK_PAGE0        0U
#define BAD_BLOCK_PAGE1        1U


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

#define ERASE_RETRIES          3U
#define PROGRAM_RETRIES        3U
#define VERIFY_RETRIES         2U

#define ID_TEST_COUNT          5U


/* ============================================================
 * BIN PATH
 * ============================================================ */

#define BIN_PATH \
    "D:\\prj\\ufprog\\stc\\_hardware_dump.bin"


/* ============================================================
 * GLOBAL PROGRESS
 * ============================================================ */

static unsigned last_percent = 999;


/* ============================================================
 * CTRL-C
 * ============================================================ */

static volatile LONG stop_requested = 0;

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
    return InterlockedCompareExchange(
        &stop_requested,
        0,
        0
    ) != 0;
}


/* ============================================================
 * FAILURE TRACKING
 * ============================================================ */

static bool failed_erase_blocks[NAND_BLOCKS];

static bool failed_program_pages[NAND_TOTAL_PAGES];

static bool failed_immediate_verify_pages[NAND_TOTAL_PAGES];

static bool failed_final_verify_pages[NAND_TOTAL_PAGES];

static bool factory_bad_blocks[NAND_BLOCKS];

static unsigned failed_erase_count = 0;
static unsigned failed_program_count = 0;
static unsigned failed_immediate_verify_count = 0;
static unsigned failed_final_verify_count = 0;

static unsigned factory_bad_block_count = 0;


/* ============================================================
 * GENERIC SPI TRANSFER
 *
 * The UFProg controller owns the actual CH341 transaction.
 *
 * For TX + RX:
 *
 *   TX transfer has end=false
 *   RX transfer has end=true
 *
 * Therefore CS remains asserted between the command/address
 * phase and the receive-clock phase.
 * ============================================================ */

static int transfer(
    struct ufprog_spi *spi,
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

    return ufprog_spi_generic_xfer(
        spi,
        xfer,
        n
    );
}


/* ============================================================
 * SEND COMMAND
 * ============================================================ */

static int command(
    struct ufprog_spi *spi,
    uint8_t opcode)
{
    return transfer(
        spi,
        &opcode,
        1,
        NULL,
        0
    );
}


/* ============================================================
 * GET FEATURE
 * ============================================================ */

static int get_feature(
    struct ufprog_spi *spi,
    uint8_t addr,
    uint8_t *value)
{
    uint8_t tx[2] = {
        CMD_GET_FEATURE,
        addr
    };

    uint8_t rx = 0;

    if (transfer(
            spi,
            tx,
            sizeof(tx),
            &rx,
            1))
        return -1;

    *value = rx;

    return 0;
}


/* ============================================================
 * SET FEATURE
 * ============================================================ */

static int set_feature(
    struct ufprog_spi *spi,
    uint8_t addr,
    uint8_t value)
{
    uint8_t tx[3] = {
        CMD_SET_FEATURE,
        addr,
        value
    };

    return transfer(
        spi,
        tx,
        sizeof(tx),
        NULL,
        0
    );
}


/* ============================================================
 * RESET
 * ============================================================ */

static int reset_chip(
    struct ufprog_spi *spi)
{
    if (command(
            spi,
            CMD_RESET))
        return -1;

    /*
     * Give the NAND time to complete RESET.
     *
     * This is deliberately conservative because the purpose
     * here is reliable identification/recovery rather than
     * maximum throughput.
     */

    Sleep(10);

    return 0;
}


/* ============================================================
 * WAIT READY
 * ============================================================ */

static int wait_ready(
    struct ufprog_spi *spi,
    unsigned timeout_ms,
    uint8_t *final_status)
{
    unsigned elapsed = 0;

    uint8_t status = 0;

    while (elapsed < timeout_ms) {

        if (abort_requested())
            return -3;

        if (get_feature(
                spi,
                REG_STATUS,
                &status))
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

static int write_enable(
    struct ufprog_spi *spi)
{
    uint8_t status = 0;

    if (abort_requested())
        return -3;

    if (command(
            spi,
            CMD_WRITE_ENABLE))
        return -1;

    if (get_feature(
            spi,
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

static void write_disable(
    struct ufprog_spi *spi)
{
    if (spi)
        command(
            spi,
            CMD_WRITE_DISABLE);
}


/* ============================================================
 * CLEAR PROTECTION
 * ============================================================ */

static int clear_protection(
    struct ufprog_spi *spi)
{
    uint8_t protection = 0;

    if (get_feature(
            spi,
            REG_PROTECTION,
            &protection))
        return -1;

    printf(
        "Protection A0 before clear : %02X\n",
        protection
    );

    if (protection == 0)
        return 0;

    /*
     * Protection register is written through SET FEATURE.
     */

    if (write_enable(spi))
        return -1;

    if (set_feature(
            spi,
            REG_PROTECTION,
            0x00))
        return -1;

    if (wait_ready(
            spi,
            1000,
            NULL))
        return -1;

    if (get_feature(
            spi,
            REG_PROTECTION,
            &protection))
        return -1;

    printf(
        "Protection A0 after clear  : %02X\n",
        protection
    );

    if (protection != 0)
        return -1;

    return 0;
}


/* ============================================================
 * READ ID
 *
 * F50L1G41LB:
 *
 *   9F
 *   00
 *   read 5 bytes
 *
 * Expected:
 *
 *   C8 01 7F 7F 7F
 *
 * The receive phase is handled by UFProg's SPI layer.
 * ============================================================ */

static int read_id(
    struct ufprog_spi *spi,
    uint8_t result[NAND_ID_LENGTH])
{
    uint8_t tx[2] = {
        CMD_READ_ID,
        0x00
    };

    memset(
        result,
        0,
        NAND_ID_LENGTH
    );

    if (transfer(
            spi,
            tx,
            sizeof(tx),
            result,
            NAND_ID_LENGTH))
        return -1;

    return 0;
}


/* ============================================================
 * PRINT ID
 * ============================================================ */

static void print_id(
    const uint8_t id[NAND_ID_LENGTH])
{
    printf(
        "%02X %02X %02X %02X %02X",
        id[0],
        id[1],
        id[2],
        id[3],
        id[4]
    );
}


/* ============================================================
 * VERIFY ID
 * ============================================================ */

static bool id_is_correct(
    const uint8_t id[NAND_ID_LENGTH])
{
    return memcmp(
        id,
        expected_nand_id,
        NAND_ID_LENGTH
    ) == 0;
}


/* ============================================================
 * ID DIAGNOSTIC
 *
 * Multiple RESET + READ-ID cycles.
 *
 * This is deliberately performed before:
 *
 *   - protection modification
 *   - erase
 *   - program
 *
 * The objective is to establish whether the NAND consistently
 * returns the correct silicon ID through the current UFProg/
 * CH341 path.
 * ============================================================ */

static int diagnostic_read_id(
    struct ufprog_spi *spi)
{
    unsigned correct = 0;

    printf("\n");
    printf("============================================================\n");
    printf(" F50L1G41LB CH341/UFPROG ID DIAGNOSTIC\n");
    printf("============================================================\n");

    printf(
        "Expected ID: "
        "C8 01 7F 7F 7F\n"
    );

    printf(
        "Transaction : "
        "9F 00 + 5 read cycles\n"
    );

    printf("\n");

    for (unsigned test = 0;
         test < ID_TEST_COUNT;
         test++) {

        uint8_t id[NAND_ID_LENGTH] = {0};

        if (abort_requested())
            return -3;

        /*
         * Reset before every ID test.
         *
         * This prevents the diagnostic from accidentally
         * depending on the NAND's previous command state.
         */

        if (reset_chip(spi)) {

            printf(
                "ID test %u: RESET FAILED\n",
                test + 1
            );

            continue;
        }

        Sleep(5);

        int ret =
            read_id(
                spi,
                id
            );

        if (ret) {

            printf(
                "ID test %u: SPI TRANSFER FAILED\n",
                test + 1
            );

            continue;
        }

        printf(
            "ID test %u: ",
            test + 1
        );

        print_id(id);

        if (id_is_correct(id)) {

            printf(
                "  <-- CORRECT\n"
            );

            correct++;

        } else {

            printf(
                "  <-- WRONG\n"
            );
        }
    }

    printf("\n");
    printf(
        "Correct ID reads: %u / %u\n",
        correct,
        ID_TEST_COUNT
    );

    /*
     * Require every diagnostic sample to agree.
     *
     * A single correct read is not enough for a recovery
     * operation because this program is allowed to erase.
     */

    if (correct != ID_TEST_COUNT) {

        printf("\n");
        printf(
            "*** ID SAFETY GATE FAILED ***\n"
        );

        printf(
            "NO ERASE OR PROGRAM OPERATION WILL BE PERFORMED.\n"
        );

        return -2;
    }

    printf("\n");
    printf(
        "*** NAND ID SAFETY GATE PASSED ***\n"
    );

    return 0;
}


/* ============================================================
 * READ CACHE
 * ============================================================ */

static int read_cache(
    struct ufprog_spi *spi,
    uint16_t column,
    uint8_t *data,
    size_t len)
{
    uint8_t tx[4];

    tx[0] = CMD_READ_CACHE;
    tx[1] = (uint8_t)((column >> 8) & 0xFF);
    tx[2] = (uint8_t)(column & 0xFF);

    /*
     * READ FROM CACHE 03h uses one dummy byte.
     */

    tx[3] = 0x00;

    return transfer(
        spi,
        tx,
        sizeof(tx),
        data,
        len
    );
}


/* ============================================================
 * PAGE READ
 * ============================================================ */

static int page_read(
    struct ufprog_spi *spi,
    unsigned page)
{
    uint32_t row = page;

    uint8_t tx[4];

    uint8_t status = 0;

    tx[0] = CMD_PAGE_READ;

    tx[1] =
        (uint8_t)((row >> 16) & 0xFF);

    tx[2] =
        (uint8_t)((row >> 8) & 0xFF);

    tx[3] =
        (uint8_t)(row & 0xFF);

    if (transfer(
            spi,
            tx,
            sizeof(tx),
            NULL,
            0))
        return -1;

    if (wait_ready(
            spi,
            10000,
            &status))
        return -1;

    return 0;
}


/* ============================================================
 * READ MAIN PAGE
 *
 * Returns:
 *
 *   0  success
 *  -1  communication failure
 *  -2  uncorrectable ECC
 * ============================================================ */

static int read_page_main(
    struct ufprog_spi *spi,
    unsigned page,
    uint8_t *data)
{
    uint8_t status = 0;

    if (page_read(
            spi,
            page))
        return -1;

    if (read_cache(
            spi,
            0x0000,
            data,
            NAND_PAGE_SIZE))
        return -1;

    if (get_feature(
            spi,
            REG_STATUS,
            &status))
        return -1;

    /*
     * F50L1G41LB:
     *
     * 00 = no ECC error
     * 10 = corrected
     * 20 = uncorrectable
     */

    if ((status & STATUS_ECC_MASK) ==
        STATUS_ECC_UNCORRECTED)
        return -2;

    return 0;
}


/* ============================================================
 * READ OOB BYTE
 *
 * Used ONLY for factory bad-block detection.
 *
 * Column 0x800 is the first byte of spare/OOB.
 * ============================================================ */

static int read_oob_byte(
    struct ufprog_spi *spi,
    unsigned page,
    uint8_t *value)
{
    if (page_read(
            spi,
            page))
        return -1;

    if (read_cache(
            spi,
            BAD_BLOCK_COLUMN,
            value,
            1))
        return -1;

    return 0;
}


/* ============================================================
 * CHECK ONE BLOCK'S FACTORY BAD-BLOCK MARKER
 *
 * ESMT specifies:
 *
 *   Non-FF = bad
 *
 * Marker is checked on:
 *
 *   page 0
 *   page 1
 *
 * at column 2048.
 *
 * Returns:
 *
 *   0  good
 *   1  bad
 *  -1  communication failure
 * ============================================================ */

static int check_factory_bad_block(
    struct ufprog_spi *spi,
    unsigned block,
    uint8_t *marker0,
    uint8_t *marker1)
{
    unsigned page0 =
        block * NAND_PAGES_PER_BLOCK;

    unsigned page1 =
        page0 + 1;

    uint8_t m0 = 0xFF;
    uint8_t m1 = 0xFF;

    int ret =
        read_oob_byte(
            spi,
            page0,
            &m0
        );

    if (ret)
        return -1;

    ret =
        read_oob_byte(
            spi,
            page1,
            &m1
        );

    if (ret)
        return -1;

    if (marker0)
        *marker0 = m0;

    if (marker1)
        *marker1 = m1;

    if (m0 != 0xFF ||
        m1 != 0xFF)
        return 1;

    return 0;
}

static void progress(
    const char *stage,
    unsigned current,
    unsigned total);

/* ============================================================
 * SCAN FACTORY BAD BLOCKS
 * ============================================================ */

static int scan_factory_bad_blocks(
    struct ufprog_spi *spi)
{
    printf("\n");
    printf("============================================================\n");
    printf(" FACTORY BAD-BLOCK SCAN\n");
    printf("============================================================\n");

    printf(
        "Marker location: column 0x800, pages 0 and 1\n"
    );

    printf(
        "Marker rule    : FF = good, anything else = bad\n"
    );

    printf("\n");

    memset(
        factory_bad_blocks,
        0,
        sizeof(factory_bad_blocks)
    );

    factory_bad_block_count = 0;

    for (unsigned block = 0;
         block < NAND_BLOCKS;
         block++) {

        if (abort_requested())
            return -3;

        uint8_t marker0 = 0xFF;
        uint8_t marker1 = 0xFF;

        int ret =
            check_factory_bad_block(
                spi,
                block,
                &marker0,
                &marker1
            );

        if (ret < 0) {

            fprintf(
                stderr,
                "\nERROR: unable to read bad-block marker "
                "for block %u.\n",
                block
            );

            return -1;
        }

        if (ret > 0) {

            factory_bad_blocks[block] = true;

            factory_bad_block_count++;

            printf(
                "BAD BLOCK %4u : page0=%02X page1=%02X\n",
                block,
                marker0,
                marker1
            );
        }

        progress(
            "BAD-BLOCK SCAN",
            block + 1,
            NAND_BLOCKS
        );
    }

    printf("\n\n");

    printf(
        "Factory bad blocks: %u / %u\n",
        factory_bad_block_count,
        NAND_BLOCKS
    );

    /*
     * The device specification permits initial invalid blocks.
     * We therefore do not reject the NAND merely because it has
     * factory bad blocks.
     */

    return 0;
}


/* ============================================================
 * BLOCK ERASE ONCE
 * ============================================================ */

static int erase_block_once(
    struct ufprog_spi *spi,
    unsigned block)
{
    uint32_t row =
        block * NAND_PAGES_PER_BLOCK;

    uint8_t tx[4];

    uint8_t status = 0;

    tx[0] = CMD_BLOCK_ERASE;

    tx[1] =
        (uint8_t)((row >> 16) & 0xFF);

    tx[2] =
        (uint8_t)((row >> 8) & 0xFF);

    tx[3] =
        (uint8_t)(row & 0xFF);

    if (write_enable(spi))
        return -1;

    if (transfer(
            spi,
            tx,
            sizeof(tx),
            NULL,
            0))
        return -1;

    if (wait_ready(
            spi,
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

static int erase_block(
    struct ufprog_spi *spi,
    unsigned block)
{
    /*
     * Factory bad blocks must never reach this function.
     */

    if (factory_bad_blocks[block])
        return -4;

    for (unsigned attempt = 0;
         attempt < ERASE_RETRIES;
         attempt++) {

        if (abort_requested())
            return -3;

        if (attempt) {

            reset_chip(spi);

            clear_protection(spi);
        }

        if (erase_block_once(
                spi,
                block) == 0)
            return 0;
    }

    return -1;
}


/* ============================================================
 * PROGRAM LOAD
 *
 * 02h + column + 2048 bytes
 *
 * No OOB is supplied.
 *
 * Internal ECC in F50L1G41LB generates the ECC data itself.
 * ============================================================ */

static int program_load(
    struct ufprog_spi *spi,
    const uint8_t *data)
{
    size_t txlen =
        3 + NAND_PAGE_SIZE;

    uint8_t *tx =
        (uint8_t *)malloc(txlen);

    if (!tx)
        return -1;

    tx[0] = CMD_PROGRAM_LOAD;

    tx[1] = 0x00;
    tx[2] = 0x00;

    memcpy(
        &tx[3],
        data,
        NAND_PAGE_SIZE
    );

    int ret =
        transfer(
            spi,
            tx,
            txlen,
            NULL,
            0
        );

    free(tx);

    return ret;
}


/* ============================================================
 * PROGRAM PAGE ONCE
 * ============================================================ */

static int program_page_once(
    struct ufprog_spi *spi,
    unsigned page,
    const uint8_t *data)
{
    uint32_t row = page;

    uint8_t tx_exec[4];

    uint8_t status = 0;

    if (write_enable(spi))
        return -1;

    /*
     * Load exactly one complete 2048-byte main page.
     *
     * This allows the NAND's internal ECC to generate the
     * correct ECC for the complete page.
     */

    if (program_load(
            spi,
            data))
        return -1;

    tx_exec[0] =
        CMD_PROGRAM_EXECUTE;

    tx_exec[1] =
        (uint8_t)((row >> 16) & 0xFF);

    tx_exec[2] =
        (uint8_t)((row >> 8) & 0xFF);

    tx_exec[3] =
        (uint8_t)(row & 0xFF);

    if (transfer(
            spi,
            tx_exec,
            sizeof(tx_exec),
            NULL,
            0))
        return -1;

    if (wait_ready(
            spi,
            10000,
            &status))
        return -1;

    if (status & STATUS_PFAIL)
        return -1;

    return 0;
}


/* ============================================================
 * PROGRAM PAGE WITH RETRIES
 * ============================================================ */

static int program_page(
    struct ufprog_spi *spi,
    unsigned page,
    const uint8_t *data)
{
    unsigned block =
        page / NAND_PAGES_PER_BLOCK;

    /*
     * Never program factory-marked bad blocks.
     */

    if (factory_bad_blocks[block])
        return -4;

    for (unsigned attempt = 0;
         attempt < PROGRAM_RETRIES;
         attempt++) {

        if (abort_requested())
            return -3;

        if (attempt) {

            reset_chip(spi);

            clear_protection(spi);
        }

        if (program_page_once(
                spi,
                page,
                data) == 0)
            return 0;
    }

    return -1;
}


/* ============================================================
 * COMPARE PAGE
 * ============================================================ */

static int compare_page(
    const uint8_t *a,
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
 * VERIFY PAGE
 * ============================================================ */

static int verify_page(
    struct ufprog_spi *spi,
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
            read_page_main(
                spi,
                page,
                buffer
            );

        if (ret)
            continue;

        if (compare_page(
                buffer,
                expected,
                first_bad,
                bad_count) == 0)
            return 0;
    }

    return -1;
}


/* ============================================================
 * PROGRESS
 * ============================================================ */

static void progress(
    const char *stage,
    unsigned current,
    unsigned total)
{
    unsigned percent;

    if (!total)
        percent = 100;
    else
        percent =
            (unsigned)(
                ((uint64_t)current * 100ULL) /
                (uint64_t)total
            );

    if (percent > 100)
        percent = 100;

    if (percent == last_percent)
        return;

    last_percent = percent;

    unsigned width = 30;

    unsigned filled =
        (percent * width) / 100;

    printf(
        "\r%-18s [",
        stage
    );

    for (unsigned i = 0;
         i < width;
         i++) {

        putchar(
            i < filled
                ? '#'
                : '.'
        );
    }

    printf(
        "] %3u%%",
        percent
    );

    fflush(stdout);
}


/* ============================================================
 * TIME FORMAT
 * ============================================================ */

static void print_elapsed(
    const char *name,
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

    printf(
        "%-22s: %02u:%02u:%02u\n",
        name,
        hours,
        minutes,
        secs
    );
}


/* ============================================================
 * LOAD BIN
 * ============================================================ */

static int load_bin(
    const char *path,
    uint8_t **buffer)
{
    FILE *fp;

    uint8_t *buf;

    size_t got;

    fp = fopen(
        path,
        "rb"
    );

    if (!fp) {

        fprintf(
            stderr,
            "\nERROR: cannot open BIN:\n%s\n",
            path
        );

        return -1;
    }

    if (fseek(
            fp,
            0,
            SEEK_END)) {

        fclose(fp);
        return -1;
    }

    long size =
        ftell(fp);

    if (size < 0) {

        fclose(fp);
        return -1;
    }

    if ((size_t)size != NAND_MAIN_SIZE) {

        fprintf(
            stderr,
            "\nERROR: incorrect BIN size.\n"
            "Actual   : %ld bytes\n"
            "Expected : %zu bytes\n",
            size,
            NAND_MAIN_SIZE
        );

        fclose(fp);

        return -1;
    }

    if (fseek(
            fp,
            0,
            SEEK_SET)) {

        fclose(fp);

        return -1;
    }

    buf =
        (uint8_t *)malloc(
            NAND_MAIN_SIZE
        );

    if (!buf) {

        fclose(fp);

        fprintf(
            stderr,
            "\nERROR: unable to allocate BIN buffer.\n"
        );

        return -1;
    }

    got =
        fread(
            buf,
            1,
            NAND_MAIN_SIZE,
            fp
        );

    fclose(fp);

    if (got != NAND_MAIN_SIZE) {

        free(buf);

        fprintf(
            stderr,
            "\nERROR: incomplete BIN read.\n"
        );

        return -1;
    }

    *buffer = buf;

    return 0;
}


/* ============================================================
 * PRINT FEATURES
 * ============================================================ */

static void print_features(
    struct ufprog_spi *spi)
{
    uint8_t a0 = 0;
    uint8_t b0 = 0;
    uint8_t c0 = 0;

    printf("\n");
    printf(
        "============================================================\n"
    );
    printf(
        " NAND FEATURE REGISTERS\n"
    );
    printf(
        "============================================================\n"
    );

    if (get_feature(
            spi,
            REG_PROTECTION,
            &a0) == 0) {

        printf(
            "A0 Protection : %02X\n",
            a0
        );
    } else {

        printf(
            "A0 Protection : READ FAILED\n"
        );
    }

    if (get_feature(
            spi,
            REG_CONFIGURATION,
            &b0) == 0) {

        printf(
            "B0 Configuration : %02X\n",
            b0
        );
    } else {

        printf(
            "B0 Configuration : READ FAILED\n"
        );
    }

    if (get_feature(
            spi,
            REG_STATUS,
            &c0) == 0) {

        printf(
            "C0 Status      : %02X\n",
            c0
        );

        printf(
            "  OIP          : %u\n",
            (c0 & STATUS_OIP) ? 1U : 0U
        );

        printf(
            "  WEL          : %u\n",
            (c0 & STATUS_WEL) ? 1U : 0U
        );

        printf(
            "  E_FAIL       : %u\n",
            (c0 & STATUS_EFAIL) ? 1U : 0U
        );

        printf(
            "  P_FAIL       : %u\n",
            (c0 & STATUS_PFAIL) ? 1U : 0U
        );

        printf(
            "  ECC          : %02X\n",
            c0 & STATUS_ECC_MASK
        );
    } else {

        printf(
            "C0 Status      : READ FAILED\n"
        );
    }

    printf("\n");
}


/* ============================================================
 * CLEANUP
 * ============================================================ */

static void cleanup(
    struct ufprog_spi *spi)
{
    if (!spi)
        return;

    write_disable(spi);

    reset_chip(spi);

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

    unsigned blocks_erased = 0;
    unsigned blocks_skipped = 0;

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


    /* ========================================================
     * CTRL-C
     * ======================================================== */

    SetConsoleCtrlHandler(
        console_ctrl_handler,
        TRUE
    );


    QueryPerformanceFrequency(&freq);

    QueryPerformanceCounter(&t_start);


    /* ========================================================
     * LOAD BIN
     * ======================================================== */

    if (load_bin(
            BIN_PATH,
            &bin))
        goto cleanup_before_open;


    page_buffer =
        (uint8_t *)malloc(
            NAND_PAGE_SIZE
        );

    if (!page_buffer) {

        fprintf(
            stderr,
            "\nERROR: unable to allocate page buffer.\n"
        );

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
            &spi
        );

    if (ret) {

        fprintf(
            stderr,
            "\nERROR: CH341 open failed: %u\n",
            ret
        );

        goto cleanup_before_open;
    }


    /* ========================================================
     * INITIAL RESET
     * ======================================================== */

    printf(
        "\nResetting NAND...\n"
    );

    if (reset_chip(spi)) {

        fprintf(
            stderr,
            "\nERROR: initial RESET failed.\n"
        );

        goto cleanup;
    }


    /* ========================================================
     * ID SAFETY GATE
     *
     * NOTHING DESTRUCTIVE BEFORE THIS PASSES.
     * ======================================================== */

    if (diagnostic_read_id(spi)) {

        fprintf(
            stderr,
            "\n"
            "*** ABORTED ***\n"
            "NAND ID did not pass the required "
            "five-sample test.\n"
            "NO ERASE HAS BEEN PERFORMED.\n"
        );

        goto cleanup;
    }


    /* ========================================================
     * FEATURES BEFORE MODIFICATION
     * ======================================================== */

    print_features(spi);


    /* ========================================================
     * FACTORY BAD-BLOCK SCAN
     *
     * Still completely non-destructive.
     * ======================================================== */

    if (scan_factory_bad_blocks(spi)) {

        fprintf(
            stderr,
            "\nERROR: factory bad-block scan failed.\n"
            "NO ERASE HAS BEEN PERFORMED.\n"
        );

        goto cleanup;
    }


    /* ========================================================
     * CLEAR PROTECTION
     * ======================================================== */

    printf(
        "\nClearing NAND protection...\n"
    );

    if (clear_protection(spi)) {

        fprintf(
            stderr,
            "\nERROR: unable to clear protection.\n"
        );

        goto cleanup;
    }


    /* ========================================================
     * VERIFY PROTECTION
     * ======================================================== */

    if (get_feature(
            spi,
            REG_PROTECTION,
            &protection)) {

        fprintf(
            stderr,
            "\nERROR: cannot read protection register.\n"
        );

        goto cleanup;
    }

    if (protection != 0) {

        fprintf(
            stderr,
            "\nERROR: protection remains %02X.\n",
            protection
        );

        goto cleanup;
    }


    /* ========================================================
     * STAGE 1 — ERASE GOOD BLOCKS
     * ======================================================== */

    printf("\n");
    printf(
        "============================================================\n"
    );
    printf(
        " STAGE 1/3 — ERASE\n"
    );
    printf(
        "============================================================\n"
    );

    printf(
        "Factory bad blocks will NOT be erased.\n"
    );

    QueryPerformanceCounter(
        &t_erase_start
    );

    last_percent = 999;

    for (unsigned block = 0;
         block < NAND_BLOCKS;
         block++) {

        if (abort_requested()) {

            interrupted = true;
            break;
        }

        progress(
            "STAGE 1/3 ERASE",
            block + 1,
            NAND_BLOCKS
        );

        if (factory_bad_blocks[block]) {

            blocks_skipped++;

            continue;
        }

        int erase_ret =
            erase_block(
                spi,
                block
            );

        if (erase_ret == -3) {

            interrupted = true;
            break;
        }

        if (erase_ret) {

            failed_erase_blocks[block] = true;

            erase_failed++;
            failed_erase_count++;

        } else {

            blocks_erased++;
        }
    }

    printf("\n");

    if (interrupted) {

        fprintf(
            stderr,
            "\n*** CTRL-C RECEIVED ***\n"
            "Stopping safely before programming.\n"
        );

        goto cleanup;
    }


    QueryPerformanceCounter(
        &t_program_start
    );

    erase_seconds =
        (double)(
            t_program_start.QuadPart -
            t_erase_start.QuadPart
        ) /
        (double)freq.QuadPart;


    /* ========================================================
     * STAGE 2 — PROGRAM + IMMEDIATE VERIFY
     * ======================================================== */

    printf("\n");
    printf(
        "============================================================\n"
    );
    printf(
        " STAGE 2/3 — PROGRAM + IMMEDIATE VERIFY\n"
    );
    printf(
        "============================================================\n"
    );

    last_percent = 999;

    for (unsigned page = 0;
         page < NAND_TOTAL_PAGES;
         page++) {

        if (abort_requested()) {

            interrupted = true;
            break;
        }

        unsigned block =
            page / NAND_PAGES_PER_BLOCK;

        progress(
            "STAGE 2/3 PROGRAM",
            page + 1,
            NAND_TOTAL_PAGES
        );


        /*
         * Do NOT program pages belonging to factory bad blocks.
         *
         * We do not shift/repack the BIN.
         *
         * Physical page N corresponds to BIN page N.
         */

        if (factory_bad_blocks[block])
            continue;


        const uint8_t *expected =
            bin +
            ((size_t)page * NAND_PAGE_SIZE);


        /* ----------------------------------------------------
         * PROGRAM
         * ---------------------------------------------------- */

        int program_ret =
            program_page(
                spi,
                page,
                expected
            );

        if (program_ret == -3) {

            interrupted = true;
            break;
        }

        if (program_ret) {

            failed_program_pages[page] = true;

            program_failed++;
            failed_program_count++;

            continue;
        }

        pages_programmed++;


        /* ----------------------------------------------------
         * IMMEDIATE VERIFY
         * ---------------------------------------------------- */

        size_t first_bad = SIZE_MAX;

        size_t bad_count = 0;

        int verify_ret =
            verify_page(
                spi,
                page,
                expected,
                page_buffer,
                &first_bad,
                &bad_count
            );

        if (verify_ret == -3) {

            interrupted = true;
            break;
        }

        if (verify_ret) {

            failed_immediate_verify_pages[page] = true;

            immediate_verify_failed++;
            failed_immediate_verify_count++;

            printf(
                "\nImmediate verify failure "
                "page %u, first mismatch 0x%zX, "
                "mismatches %zu\n",
                page,
                first_bad,
                bad_count
            );

        } else {

            pages_immediately_verified++;
        }
    }

    printf("\n");

    if (interrupted) {

        fprintf(
            stderr,
            "\n*** CTRL-C RECEIVED ***\n"
            "Stopping safely during Stage 2.\n"
        );

        goto cleanup;
    }


    QueryPerformanceCounter(
        &t_final_verify_start
    );

    program_seconds =
        (double)(
            t_final_verify_start.QuadPart -
            t_program_start.QuadPart
        ) /
        (double)freq.QuadPart;


    /* ========================================================
     * STAGE 3 — FINAL FULL VERIFY
     * ======================================================== */

    printf("\n");
    printf(
        "============================================================\n"
    );
    printf(
        " STAGE 3/3 — FINAL VERIFY\n"
    );
    printf(
        "============================================================\n"
    );

    last_percent = 999;

    for (unsigned page = 0;
         page < NAND_TOTAL_PAGES;
         page++) {

        if (abort_requested()) {

            interrupted = true;
            break;
        }

        unsigned block =
            page / NAND_PAGES_PER_BLOCK;

        progress(
            "STAGE 3/3 VERIFY",
            page + 1,
            NAND_TOTAL_PAGES
        );


        /*
         * Factory bad blocks are intentionally skipped.
         */

        if (factory_bad_blocks[block])
            continue;


        const uint8_t *expected =
            bin +
            ((size_t)page * NAND_PAGE_SIZE);


        size_t first_bad = SIZE_MAX;

        size_t bad_count = 0;

        int verify_ret =
            verify_page(
                spi,
                page,
                expected,
                page_buffer,
                &first_bad,
                &bad_count
            );

        if (verify_ret == -3) {

            interrupted = true;
            break;
        }

        if (verify_ret) {

            failed_final_verify_pages[page] = true;

            final_verify_failed++;
            failed_final_verify_count++;

            printf(
                "\nFinal verify failure "
                "page %u, first mismatch 0x%zX, "
                "mismatches %zu\n",
                page,
                first_bad,
                bad_count
            );

        } else {

            pages_final_verified++;
        }
    }

    printf("\n");

    if (interrupted) {

        fprintf(
            stderr,
            "\n*** CTRL-C RECEIVED ***\n"
            "Stopping safely during Stage 3.\n"
        );

        goto cleanup;
    }


    /* ========================================================
     * TIMING
     * ======================================================== */

    QueryPerformanceCounter(
        &t_end
    );

    final_verify_seconds =
        (double)(
            t_end.QuadPart -
            t_final_verify_start.QuadPart
        ) /
        (double)freq.QuadPart;

    total_seconds =
        (double)(
            t_end.QuadPart -
            t_start.QuadPart
        ) /
        (double)freq.QuadPart;


    /* ========================================================
     * FINAL RESET
     * ======================================================== */

    printf(
        "\nFinal NAND reset...\n"
    );

    reset_chip(spi);


    /* ========================================================
     * FINAL FEATURES
     * ======================================================== */

    get_feature(
        spi,
        REG_PROTECTION,
        &protection
    );

    get_feature(
        spi,
        REG_CONFIGURATION,
        &configuration
    );

    get_feature(
        spi,
        REG_STATUS,
        &status
    );


    /* ========================================================
     * FINAL ID
     * ======================================================== */

    uint8_t final_id[NAND_ID_LENGTH] = {0};

    printf(
        "\nFinal NAND ID: "
    );

    if (read_id(
            spi,
            final_id) == 0) {

        print_id(final_id);

        if (id_is_correct(final_id)) {

            printf(
                "  <-- CORRECT\n"
            );

        } else {

            printf(
                "  <-- WRONG\n"
            );
        }

    } else {

        printf(
            "READ FAILED\n"
        );
    }


    /* ========================================================
     * SUMMARY
     * ======================================================== */

    printf("\n");
    printf(
        "============================================================\n"
    );
    printf(
        " FULL BIN RESTORE SUMMARY\n"
    );
    printf(
        "============================================================\n"
    );

    printf(
        "BIN                  : %s\n",
        BIN_PATH
    );

    printf(
        "BIN size             : %zu bytes\n",
        NAND_MAIN_SIZE
    );

    printf(
        "Blocks               : %u\n",
        NAND_BLOCKS
    );

    printf(
        "Pages                : %u\n",
        NAND_TOTAL_PAGES
    );

    printf(
        "Page size            : %u bytes\n",
        NAND_PAGE_SIZE
    );

    printf(
        "Block size            : %llu KiB\n",
        (unsigned long long)(
            NAND_BLOCK_MAIN_SIZE / 1024ULL
        )
    );

    printf("\n");

    printf(
        "Factory bad blocks   : %u\n",
        factory_bad_block_count
    );

    printf(
        "Blocks erased        : %u\n",
        blocks_erased
    );

    printf(
        "Blocks skipped       : %u\n",
        blocks_skipped
    );

    printf("\n");

    printf(
        "Pages programmed     : %u\n",
        pages_programmed
    );

    printf(
        "Immediate verified   : %u\n",
        pages_immediately_verified
    );

    printf(
        "Final verified       : %u\n",
        pages_final_verified
    );

    printf("\n");

    printf(
        "Erase failures       : %u\n",
        erase_failed
    );

    printf(
        "Program failures     : %u\n",
        program_failed
    );

    printf(
        "Immediate verify     : %u\n",
        immediate_verify_failed
    );

    printf(
        "Final verify         : %u\n",
        final_verify_failed
    );

    printf("\n");

    print_elapsed(
        "Erase time",
        erase_seconds
    );

    print_elapsed(
        "Program + verify",
        program_seconds
    );

    print_elapsed(
        "Final verify",
        final_verify_seconds
    );

    print_elapsed(
        "TOTAL TIME",
        total_seconds
    );


    /* ========================================================
     * FINAL REGISTERS
     * ======================================================== */

    printf("\n");

    printf(
        "Final A0             : %02X\n",
        protection
    );

    printf(
        "Final B0             : %02X\n",
        configuration
    );

    printf(
        "Final C0             : %02X\n",
        status
    );

    printf(
        "OIP                  : %u\n",
        (status & STATUS_OIP) ? 1U : 0U
    );

    printf(
        "WEL                  : %u\n",
        (status & STATUS_WEL) ? 1U : 0U
    );

    printf(
        "E_FAIL               : %u\n",
        (status & STATUS_EFAIL) ? 1U : 0U
    );

    printf(
        "P_FAIL               : %u\n",
        (status & STATUS_PFAIL) ? 1U : 0U
    );

    printf(
        "ECC status           : %02X\n",
        status & STATUS_ECC_MASK
    );


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
            NAND_PAGES_PER_BLOCK -
            1;

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
        printf(
            "*** FAILED BLOCK LIST ***\n"
        );

        for (unsigned block = 0;
             block < NAND_BLOCKS;
             block++) {

            bool erase_fail =
                failed_erase_blocks[block];

            unsigned first_page =
                block * NAND_PAGES_PER_BLOCK;

            unsigned last_page =
                first_page +
                NAND_PAGES_PER_BLOCK -
                1;

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

            printf(
                "\nBlock %4u  pages %5u-%-5u\n",
                block,
                first_page,
                last_page
            );

            if (erase_fail)
                printf(
                    "  ERASE: FAIL\n"
                );

            for (unsigned page = first_page;
                 page <= last_page;
                 page++) {

                if (failed_program_pages[page]) {

                    printf(
                        "  Page %5u: PROGRAM FAIL\n",
                        page
                    );
                }
            }

            for (unsigned page = first_page;
                 page <= last_page;
                 page++) {

                if (failed_immediate_verify_pages[page]) {

                    printf(
                        "  Page %5u: IMMEDIATE VERIFY FAIL\n",
                        page
                    );
                }
            }

            for (unsigned page = first_page;
                 page <= last_page;
                 page++) {

                if (failed_final_verify_pages[page]) {

                    printf(
                        "  Page %5u: FINAL VERIFY FAIL\n",
                        page
                    );
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

        printf(
            "*** RESTORE COMPLETED WITH FAILURES ***\n"
        );

    } else {

        printf(
            "*** FULL BIN RESTORE VERIFIED SUCCESSFULLY ***\n"
        );
    }

    printf("\n");

    printf(
        "============================================================\n"
    );

    printf(
        " END\n"
    );

    printf(
        "============================================================\n"
    );


    /* ========================================================
     * CLEANUP
     * ======================================================== */

    cleanup(spi);

    spi = NULL;

    free(page_buffer);
    free(bin);

    SetConsoleCtrlHandler(
        console_ctrl_handler,
        FALSE
    );


    if (erase_failed ||
        program_failed ||
        immediate_verify_failed ||
        final_verify_failed)
        return 2;

    return 0;


/* ============================================================
 * CLEANUP AFTER OPEN
 * ============================================================ */

cleanup:

    cleanup(spi);

    spi = NULL;

    free(page_buffer);
    free(bin);

    SetConsoleCtrlHandler(
        console_ctrl_handler,
        FALSE
    );

    if (abort_requested()) {

        fprintf(
            stderr,
            "\n*** CLEANUP COMPLETE ***\n"
            "NAND left in reset/idle state.\n"
        );

        return 130;
    }

    return 1;


/* ============================================================
 * CLEANUP BEFORE OPEN
 * ============================================================ */

cleanup_before_open:

    free(page_buffer);
    free(bin);

    SetConsoleCtrlHandler(
        console_ctrl_handler,
        FALSE
    );

    return 1;
}