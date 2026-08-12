/*
 * ch341-nand-write.c
 *
 * ESMT F50L1G41LB
 *
 * MAIN-AREA NAND RECOVERY WRITER
 *
 * IMPORTANT:
 *
 * NAND ID is deliberately NOT validated.
 *
 * The physical NAND ID is known to be faulty/unreliable and this
 * program is intended to force the original dump back onto the chip.
 *
 * Programming sequence:
 *
 *     WRITE ENABLE       06h
 *     PROGRAM LOAD      02h + column + 2048 bytes
 *     PROGRAM EXECUTE   10h + row
 *     WAIT OIP = 0
 *     CHECK P_FAIL
 *
 * Erase sequence:
 *
 *     WRITE ENABLE       06h
 *     BLOCK ERASE        D8h + row
 *     WAIT OIP = 0
 *     CHECK E_FAIL
 *
 * Optional verification:
 *
 *     Disabled by default.
 *
 *     If VERIFY_AFTER_PROGRAM is changed to 1, the entire NAND is
 *     verified in a separate pass AFTER programming has completed.
 *
 * NAND GEOMETRY:
 *
 *     Main page       = 2048 bytes
 *     OOB             = 64 bytes
 *     Pages/block     = 64
 *     Blocks          = 1024
 *     Total pages     = 65536
 *     Main area       = 128 MiB
 *
 * CTRL+C:
 *
 *     - Stops starting new NAND operations.
 *     - If an erase/program/read operation has already been issued,
 *       wait_ready() continues until OIP clears.
 *     - Restores original A0/B0 configuration.
 *     - Resets NAND.
 *     - Closes CH341.
 *     - Returns exit code 130.
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
 * CONFIGURATION
 * ============================================================ */

/*
 * 0 = FAST WRITE MODE
 *
 * No read-back verification is performed while programming.
 *
 * 1 = Perform a complete verification pass after programming.
 *
 * Recommended for recovery speed:
 *
 *     0
 */
#define VERIFY_AFTER_PROGRAM       0


/*
 * Number of retries for an erase operation.
 */
#define ERASE_RETRIES              3


/*
 * Number of retries for a program operation.
 */
#define PROGRAM_RETRIES            3


/*
 * Number of retries for read-back verification.
 *
 * Only used when VERIFY_AFTER_PROGRAM = 1.
 */
#define VERIFY_RETRIES             2


/*
 * Maximum NAND operation timeout.
 *
 * This is deliberately generous.
 *
 * The actual NAND program/erase time is much shorter, but this
 * protects against a device that remains busy unexpectedly.
 */
#define NAND_OPERATION_TIMEOUT_MS  10000


/*
 * BIN FILE
 *
 * IMPORTANT:
 * Backslashes must be escaped in a C string.
 */
#define BIN_PATH \
    "D:\\prj\\ufprog\\stc\\_hardware_dump.bin"


/*
 * Initial delay before polling NAND status.
 *
 * The F50L1G41LB page-program operation has a sub-millisecond
 * typical/max internal program time, so there is no benefit in
 * immediately issuing a status transaction.
 *
 * This delay also reduces unnecessary CH341/USB transactions.
 */
#define PROGRAM_STATUS_INITIAL_DELAY_MS   1


/*
 * Initial delay before polling erase status.
 */
#define ERASE_STATUS_INITIAL_DELAY_MS     2


/*
 * Delay between status polls when NAND remains busy.
 */
#define STATUS_POLL_INTERVAL_MS            1


/* ============================================================
 * NAND GEOMETRY
 * ============================================================ */

#define NAND_BLOCKS               1024U
#define NAND_PAGES_PER_BLOCK     64U

#define NAND_TOTAL_PAGES \
    (NAND_BLOCKS * NAND_PAGES_PER_BLOCK)

#define NAND_PAGE_SIZE           2048U
#define NAND_OOB_SIZE              64U

#define NAND_MAIN_SIZE \
    ((size_t)NAND_TOTAL_PAGES * NAND_PAGE_SIZE)

#define NAND_BLOCK_MAIN_SIZE \
    ((size_t)NAND_PAGES_PER_BLOCK * NAND_PAGE_SIZE)


/* ============================================================
 * NAND COMMANDS
 * ============================================================ */

#define CMD_WRITE_ENABLE         0x06
#define CMD_WRITE_DISABLE        0x04
#define CMD_RESET                0xFF

#define CMD_READ_ID              0x9F

#define CMD_GET_FEATURE          0x0F
#define CMD_SET_FEATURE          0x1F

#define CMD_PAGE_READ            0x13
#define CMD_READ_CACHE           0x03

#define CMD_PROGRAM_LOAD         0x02
#define CMD_PROGRAM_EXECUTE      0x10

#define CMD_BLOCK_ERASE          0xD8


/* ============================================================
 * FEATURE REGISTERS
 * ============================================================ */

#define REG_PROTECTION           0xA0
#define REG_CONFIGURATION        0xB0
#define REG_STATUS               0xC0


/* ============================================================
 * STATUS BITS
 * ============================================================ */

#define STATUS_OIP               0x01
#define STATUS_WEL               0x02
#define STATUS_EFAIL             0x04
#define STATUS_PFAIL             0x08

#define STATUS_ECC_MASK          0x30
#define STATUS_ECC_CORRECTED     0x10
#define STATUS_ECC_UNCORRECTED   0x20


/* ============================================================
 * NAND ID
 * ============================================================ */

/*
 * The ID is intentionally NOT compared against an expected value.
 *
 * The device ID is known to be faulty.
 *
 * We still read and print the ID because it is useful diagnostic
 * information, but it can never stop the recovery write.
 */
#define NAND_ID_LENGTH           5U


/* ============================================================
 * CTRL-C STATE
 * ============================================================ */

static volatile LONG stop_requested = 0;


/* ============================================================
 * ORIGINAL NAND CONFIGURATION
 * ============================================================ */

static uint8_t original_protection = 0;
static uint8_t original_configuration = 0;

static bool original_configuration_valid = false;


/* ============================================================
 * STATISTICS
 * ============================================================ */

static unsigned required_blocks = 0;
static unsigned required_pages = 0;

static unsigned blocks_erased = 0;
static unsigned pages_programmed = 0;

#if VERIFY_AFTER_PROGRAM
static unsigned pages_verified = 0;
#endif

static unsigned erase_failures = 0;
static unsigned program_failures = 0;

#if VERIFY_AFTER_PROGRAM
static unsigned verify_failures = 0;
#endif


/* ============================================================
 * FAILED OPERATION TRACKING
 * ============================================================ */

static bool failed_erase_blocks[NAND_BLOCKS];

static bool failed_program_pages[NAND_TOTAL_PAGES];

#if VERIFY_AFTER_PROGRAM
static bool failed_verify_pages[NAND_TOTAL_PAGES];
#endif


/* ============================================================
 * PROGRESS
 * ============================================================ */

static unsigned last_percent = 999;


/* ============================================================
 * CTRL-C HANDLER
 * ============================================================ */

static BOOL WINAPI console_ctrl_handler(DWORD ctrl_type)
{
    switch (ctrl_type) {

    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:

        /*
         * Do NOT abort the USB device here.
         *
         * The main loop will notice this flag and stop
         * starting new NAND operations.
         *
         * If a NAND operation is already running,
         * wait_ready() deliberately continues polling
         * until OIP becomes zero.
         */

        InterlockedExchange(&stop_requested, 1);

        return TRUE;

    default:
        return FALSE;
    }
}


/* ============================================================
 * ABORT REQUESTED
 * ============================================================ */

static bool abort_requested(void)
{
    return InterlockedCompareExchange(
        &stop_requested,
        0,
        0
    ) != 0;
}


/* ============================================================
 * GENERIC SPI TRANSFER
 * ============================================================ */

static int transfer(
    struct ufprog_spi *spi,
    const uint8_t *tx,
    size_t txlen,
    uint8_t *rx,
    size_t rxlen
)
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
    uint8_t opcode
)
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
    uint8_t *value
)
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
    uint8_t value
)
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
    struct ufprog_spi *spi
)
{
    if (command(
            spi,
            CMD_RESET))
        return -1;

    Sleep(2);

    return 0;
}


/* ============================================================
 * WAIT UNTIL NAND READY
 *
 * IMPORTANT:
 *
 * Once an erase/program/page-read operation has actually
 * been issued, Ctrl+C does NOT cause this function to return
 * immediately.
 *
 * We must wait for OIP=0 so cleanup does not reset/close
 * CH341 while the NAND is still busy.
 *
 * The first status poll is delayed slightly to avoid
 * unnecessary immediate USB transactions.
 * ============================================================ */

static int wait_ready(
    struct ufprog_spi *spi,
    unsigned timeout_ms,
    unsigned initial_delay_ms,
    uint8_t *final_status
)
{
    unsigned elapsed = 0;

    uint8_t status = 0;

    /*
     * Give the NAND some time to complete its internal
     * operation before performing the first status query.
     */
    if (initial_delay_ms) {

        Sleep(initial_delay_ms);

        elapsed += initial_delay_ms;
    }

    while (elapsed < timeout_ms) {

        /*
         * Do NOT test abort_requested() here.
         *
         * The NAND operation has already started.
         *
         * We must wait for it to finish.
         */

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

        Sleep(STATUS_POLL_INTERVAL_MS);

        elapsed += STATUS_POLL_INTERVAL_MS;
    }

    if (final_status)
        *final_status = status;

    return -2;
}


/* ============================================================
 * WRITE ENABLE
 * ============================================================ */

static int write_enable(
    struct ufprog_spi *spi
)
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
    struct ufprog_spi *spi
)
{
    if (spi)
        command(
            spi,
            CMD_WRITE_DISABLE
        );
}


/* ============================================================
 * READ NAND ID
 *
 * IMPORTANT:
 *
 * This function ONLY reports the ID.
 *
 * It NEVER rejects the NAND.
 * ============================================================ */

static int read_id(
    struct ufprog_spi *spi
)
{
    uint8_t tx[2] = {
        CMD_READ_ID,
        0x00
    };

    uint8_t rx[NAND_ID_LENGTH] = {0};

    if (transfer(
            spi,
            tx,
            sizeof(tx),
            rx,
            sizeof(rx)))
        return -1;

    printf("ID:");

    for (unsigned i = 0;
         i < NAND_ID_LENGTH;
         i++) {

        printf(
            " %02X",
            rx[i]
        );
    }

    printf("\n");

    /*
     * Deliberately no ID comparison.
     *
     * The ID is known to be faulty and therefore cannot
     * be used as a write authorization check.
     */

    return 0;
}


/* ============================================================
 * SAVE ORIGINAL CONFIGURATION
 * ============================================================ */

static int save_configuration(
    struct ufprog_spi *spi
)
{
    if (get_feature(
            spi,
            REG_PROTECTION,
            &original_protection))
        return -1;

    if (get_feature(
            spi,
            REG_CONFIGURATION,
            &original_configuration))
        return -1;

    original_configuration_valid = true;

    return 0;
}


/* ============================================================
 * CLEAR PROTECTION
 * ============================================================ */

static int clear_protection(
    struct ufprog_spi *spi
)
{
    uint8_t protection = 0;

    if (get_feature(
            spi,
            REG_PROTECTION,
            &protection))
        return -1;

    if (protection == 0)
        return 0;

    /*
     * Clear protection.
     */

    if (set_feature(
            spi,
            REG_PROTECTION,
            0x00))
        return -1;

    /*
     * Allow feature operation to settle.
     */

    Sleep(2);

    if (get_feature(
            spi,
            REG_PROTECTION,
            &protection))
        return -1;

    if (protection != 0)
        return -1;

    return 0;
}


/* ============================================================
 * RESTORE ORIGINAL CONFIGURATION
 * ============================================================ */

static int restore_configuration(
    struct ufprog_spi *spi
)
{
    int ret = 0;

    if (!spi)
        return -1;

    if (!original_configuration_valid)
        return 0;

    /*
     * Restore B0 first.
     */

    if (set_feature(
            spi,
            REG_CONFIGURATION,
            original_configuration))
        ret = -1;

    Sleep(2);

    /*
     * Restore A0.
     */

    if (set_feature(
            spi,
            REG_PROTECTION,
            original_protection))
        ret = -1;

    Sleep(2);

    return ret;
}


/* ============================================================
 * ERASE ONE BLOCK
 * ============================================================ */

static int erase_block_once(
    struct ufprog_spi *spi,
    unsigned block
)
{
    uint32_t row;

    uint8_t tx[4];

    uint8_t status = 0;

    row =
        block *
        NAND_PAGES_PER_BLOCK;

    tx[0] = CMD_BLOCK_ERASE;

    tx[1] =
        (uint8_t)((row >> 16) & 0xFF);

    tx[2] =
        (uint8_t)((row >> 8) & 0xFF);

    tx[3] =
        (uint8_t)(row & 0xFF);

    /*
     * Do not start a new operation after Ctrl+C.
     */

    if (abort_requested())
        return -3;

    if (write_enable(spi))
        return -1;

    /*
     * BLOCK ERASE has now been issued.
     *
     * From this point wait_ready() ignores Ctrl+C
     * until OIP clears.
     */

    if (transfer(
            spi,
            tx,
            sizeof(tx),
            NULL,
            0))
        return -1;

    if (wait_ready(
            spi,
            NAND_OPERATION_TIMEOUT_MS,
            ERASE_STATUS_INITIAL_DELAY_MS,
            &status))
        return -1;

    /*
     * E_FAIL must be clear.
     */

    if (status & STATUS_EFAIL)
        return -1;

    return 0;
}


/* ============================================================
 * ERASE BLOCK WITH RETRIES
 * ============================================================ */

static int erase_block(
    struct ufprog_spi *spi,
    unsigned block
)
{
    for (unsigned attempt = 0;
         attempt < ERASE_RETRIES;
         attempt++) {

        /*
         * Never begin a retry after Ctrl+C.
         */

        if (abort_requested())
            return -3;

        if (attempt) {

            /*
             * The previous erase has already finished.
             *
             * Reset is therefore safe here.
             */

            if (reset_chip(spi))
                return -1;

            /*
             * Restore write protection state needed for
             * another erase.
             */

            if (clear_protection(spi))
                return -1;
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
 * ============================================================ */

static int program_load(
    struct ufprog_spi *spi,
    const uint8_t *data
)
{
    size_t txlen =
        3 + NAND_PAGE_SIZE;

    uint8_t *tx =
        (uint8_t *)malloc(txlen);

    if (!tx)
        return -1;

    tx[0] = CMD_PROGRAM_LOAD;

    /*
     * Column address = 0.
     */

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
 * PROGRAM ONE PAGE
 * ============================================================ */

static int program_page_once(
    struct ufprog_spi *spi,
    unsigned page,
    const uint8_t *data
)
{
    uint32_t row =
        page;

    uint8_t tx_exec[4];

    uint8_t status = 0;

    /*
     * Do not begin another operation after Ctrl+C.
     */

    if (abort_requested())
        return -3;

    /*
     * WRITE ENABLE
     */

    if (write_enable(spi))
        return -1;

    /*
     * PROGRAM LOAD
     *
     * Loads the complete 2048-byte main-area page into
     * the NAND cache register.
     */

    if (program_load(
            spi,
            data))
        return -1;

    /*
     * Ctrl+C could have arrived while the cache was being
     * loaded.
     *
     * Do not issue PROGRAM EXECUTE if stop was requested.
     */

    if (abort_requested())
        return -3;

    /*
     * PROGRAM EXECUTE
     */

    tx_exec[0] =
        CMD_PROGRAM_EXECUTE;

    tx_exec[1] =
        (uint8_t)((row >> 16) & 0xFF);

    tx_exec[2] =
        (uint8_t)((row >> 8) & 0xFF);

    tx_exec[3] =
        (uint8_t)(row & 0xFF);

    /*
     * Once this command is issued, the NAND operation is
     * considered in progress.
     *
     * wait_ready() will wait regardless of Ctrl+C.
     */

    if (transfer(
            spi,
            tx_exec,
            sizeof(tx_exec),
            NULL,
            0))
        return -1;

    if (wait_ready(
            spi,
            NAND_OPERATION_TIMEOUT_MS,
            PROGRAM_STATUS_INITIAL_DELAY_MS,
            &status))
        return -1;

    /*
     * P_FAIL must be clear.
     */

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
    const uint8_t *data
)
{
    for (unsigned attempt = 0;
         attempt < PROGRAM_RETRIES;
         attempt++) {

        /*
         * Never start another program operation after Ctrl+C.
         */

        if (abort_requested())
            return -3;

        if (attempt) {

            /*
             * Previous program operation has already finished.
             *
             * Reset is therefore safe here.
             */

            if (reset_chip(spi))
                return -1;

            if (clear_protection(spi))
                return -1;
        }

        int ret =
            program_page_once(
                spi,
                page,
                data
            );

        if (ret == 0)
            return 0;

        if (ret == -3)
            return -3;
    }

    return -1;
}


#if VERIFY_AFTER_PROGRAM


/* ============================================================
 * PAGE READ
 * ============================================================ */

static int page_read(
    struct ufprog_spi *spi,
    unsigned page
)
{
    uint32_t row =
        page;

    uint8_t tx[4];

    uint8_t status = 0;

    if (abort_requested())
        return -3;

    tx[0] =
        CMD_PAGE_READ;

    tx[1] =
        (uint8_t)((row >> 16) & 0xFF);

    tx[2] =
        (uint8_t)((row >> 8) & 0xFF);

    tx[3] =
        (uint8_t)(row & 0xFF);

    /*
     * Once PAGE READ has been issued, wait for completion.
     */

    if (transfer(
            spi,
            tx,
            sizeof(tx),
            NULL,
            0))
        return -1;

    if (wait_ready(
            spi,
            NAND_OPERATION_TIMEOUT_MS,
            1,
            &status))
        return -1;

    return 0;
}


/* ============================================================
 * READ CACHE
 * ============================================================ */

static int read_cache(
    struct ufprog_spi *spi,
    uint16_t column,
    uint8_t *data,
    size_t len
)
{
    uint8_t tx[4];

    tx[0] =
        CMD_READ_CACHE;

    tx[1] =
        (uint8_t)((column >> 8) & 0xFF);

    tx[2] =
        (uint8_t)(column & 0xFF);

    tx[3] =
        0x00;

    return transfer(
        spi,
        tx,
        sizeof(tx),
        data,
        len
    );
}


/* ============================================================
 * READ MAIN AREA OF PAGE
 * ============================================================ */

static int read_page_main(
    struct ufprog_spi *spi,
    unsigned page,
    uint8_t *data
)
{
    uint8_t status = 0;

    int ret =
        page_read(
            spi,
            page
        );

    if (ret)
        return ret;

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
     * ECC:
     *
     * 00 = no error
     * 10 = corrected
     * 20 = uncorrectable
     */

    if ((status & STATUS_ECC_MASK) ==
        STATUS_ECC_UNCORRECTED)
        return -2;

    return 0;
}


/* ============================================================
 * COMPARE PAGE
 * ============================================================ */

static int compare_page(
    const uint8_t *a,
    const uint8_t *b
)
{
    return memcmp(
        a,
        b,
        NAND_PAGE_SIZE
    ) == 0 ? 0 : -1;
}


/* ============================================================
 * VERIFY ONE PAGE
 * ============================================================ */

static int verify_page(
    struct ufprog_spi *spi,
    unsigned page,
    const uint8_t *expected,
    uint8_t *buffer
)
{
    for (unsigned attempt = 0;
         attempt < VERIFY_RETRIES;
         attempt++) {

        if (abort_requested())
            return -3;

        int ret =
            read_page_main(
                spi,
                page,
                buffer
            );

        if (ret == -3)
            return -3;

        if (ret)
            continue;

        if (compare_page(
                buffer,
                expected) == 0)
            return 0;
    }

    return -1;
}


#endif /* VERIFY_AFTER_PROGRAM */


/* ============================================================
 * PROGRESS BAR
 * ============================================================ */

static void progress(
    const char *stage,
    unsigned current,
    unsigned total
)
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

    last_percent =
        percent;

    const unsigned width =
        40;

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
 * LOAD BIN
 * ============================================================ */

static int load_bin(
    const char *path,
    uint8_t **buffer
)
{
    FILE *fp;

    uint8_t *buf;

    size_t got;

    fp =
        fopen(
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

    if ((size_t)size !=
        NAND_MAIN_SIZE) {

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

    if (got !=
        NAND_MAIN_SIZE) {

        free(buf);

        fprintf(
            stderr,
            "\nERROR: incomplete BIN read.\n"
        );

        return -1;
    }

    *buffer =
        buf;

    return 0;
}


/* ============================================================
 * CLEANUP
 *
 * This function is deliberately safe to call after Ctrl+C.
 *
 * At this point all active NAND operations should already
 * have completed because wait_ready() does NOT abort while
 * OIP is set.
 * ============================================================ */

static void cleanup(
    struct ufprog_spi *spi
)
{
    if (!spi)
        return;

    /*
     * Prevent accidental further writes.
     */

    write_disable(spi);

    /*
     * Restore original A0/B0.
     *
     * This is performed before reset.
     */

    if (original_configuration_valid) {

        if (restore_configuration(spi)) {

            fprintf(
                stderr,
                "\nWARNING: failed to completely restore "
                "original NAND configuration.\n"
            );
        }
    }

    /*
     * Leave NAND in known reset/idle state.
     */

    if (reset_chip(spi)) {

        fprintf(
            stderr,
            "\nWARNING: NAND reset failed during cleanup.\n"
        );
    }

    /*
     * Close CH341.
     */

    ufprog_spi_close_device(spi);
}


/* ============================================================
 * MAIN
 * ============================================================ */

int wmain(void)
{
    struct ufprog_spi *spi =
        NULL;

    ufprog_status ret;

    uint8_t *bin =
        NULL;

#if VERIFY_AFTER_PROGRAM
    uint8_t *page_buffer =
        NULL;
#endif

    bool interrupted =
        false;

    bool opened =
        false;

    LARGE_INTEGER freq;

    LARGE_INTEGER start_time;

    LARGE_INTEGER end_time;

    QueryPerformanceFrequency(
        &freq
    );

    QueryPerformanceCounter(
        &start_time
    );


    /* ========================================================
     * CTRL-C HANDLER
     * ======================================================== */

    SetConsoleCtrlHandler(
        console_ctrl_handler,
        TRUE
    );


    /* ========================================================
     * LOAD IMAGE
     * ======================================================== */

    printf(
        "\n"
        "CH341 NAND Recovery Writer\n"
        "==========================\n\n"
    );

    printf(
        "FAST WRITE MODE: %s\n",
#if VERIFY_AFTER_PROGRAM
        "WRITE + FINAL VERIFY"
#else
        "WRITE ONLY"
#endif
    );

    printf(
        "\n"
        "NAND ID validation: DISABLED\n"
        "The reported NAND ID will NOT prevent writing.\n\n"
    );

    printf(
        "Image:\n"
        "  %s\n\n",
        BIN_PATH
    );

    if (load_bin(
            BIN_PATH,
            &bin)) {

        goto failure_before_open;
    }

    printf(
        "Image size: %zu bytes\n",
        NAND_MAIN_SIZE
    );


    /* ========================================================
     * CALCULATE REQUIRED GEOMETRY
     * ======================================================== */

    required_pages =
        (unsigned)(
            NAND_MAIN_SIZE /
            NAND_PAGE_SIZE
        );

    required_blocks =
        (required_pages +
         NAND_PAGES_PER_BLOCK -
         1) /
        NAND_PAGES_PER_BLOCK;

    printf(
        "Pages required : %u\n"
        "Blocks required: %u\n\n",
        required_pages,
        required_blocks
    );


#if VERIFY_AFTER_PROGRAM

    /* ========================================================
     * PAGE BUFFER
     * ======================================================== */

    page_buffer =
        (uint8_t *)malloc(
            NAND_PAGE_SIZE
        );

    if (!page_buffer) {

        fprintf(
            stderr,
            "\nERROR: unable to allocate page buffer.\n"
        );

        goto failure_before_open;
    }

#endif


    /* ========================================================
     * UFPROG INITIALIZATION
     * ======================================================== */

    set_os_default_log_print();

    os_init();


    /* ========================================================
     * OPEN CH341
     * ======================================================== */

    printf(
        "Opening CH341..."
    );

    fflush(stdout);

    ret =
        ufprog_spi_open_device(
            "ch341-libusb",
            false,
            &spi
        );

    if (ret) {

        printf(
            " FAILED\n"
        );

        fprintf(
            stderr,
            "ERROR: CH341 open failed: %u\n",
            ret
        );

        goto failure;
    }

    opened = true;

    printf(
        " OK\n"
    );


    /* ========================================================
     * INITIAL RESET
     * ======================================================== */

    printf(
        "Resetting NAND..."
    );

    fflush(stdout);

    if (reset_chip(
            spi)) {

        printf(
            " FAILED\n"
        );

        fprintf(
            stderr,
            "ERROR: NAND reset failed.\n"
        );

        goto failure;
    }

    printf(
        " OK\n"
    );


    /* ========================================================
     * READ NAND ID
     *
     * ID IS DISPLAYED ONLY.
     *
     * NO VALIDATION.
     * ======================================================== */

    printf(
        "Reading NAND ID...\n"
    );

    if (read_id(spi)) {

        fprintf(
            stderr,
            "ERROR: NAND ID communication failed.\n"
        );

        goto failure;
    }

    printf(
        "NAND ID accepted for forced recovery write.\n"
    );


    /* ========================================================
     * SAVE ORIGINAL CONFIGURATION
     * ======================================================== */

    if (save_configuration(
            spi)) {

        fprintf(
            stderr,
            "\nERROR: unable to read NAND configuration.\n"
        );

        goto failure;
    }

    printf(
        "Original A0: %02X\n"
        "Original B0: %02X\n",
        original_protection,
        original_configuration
    );


    /* ========================================================
     * CLEAR PROTECTION
     * ======================================================== */

    printf(
        "Clearing protection..."
    );

    fflush(stdout);

    if (clear_protection(
            spi)) {

        printf(
            " FAILED\n"
        );

        fprintf(
            stderr,
            "ERROR: unable to clear NAND protection.\n"
        );

        goto failure;
    }

    printf(
        " OK\n"
    );


    /* ========================================================
     * ERASE
     * ======================================================== */

    printf(
        "\nErasing %u blocks...\n",
        required_blocks
    );

    last_percent =
        999;

    for (unsigned block = 0;
         block < required_blocks;
         block++) {

        /*
         * Ctrl+C means:
         *
         * stop starting new operations.
         */

        if (abort_requested()) {

            interrupted =
                true;

            break;
        }

        int erase_ret =
            erase_block(
                spi,
                block
            );

        if (erase_ret == -3) {

            interrupted =
                true;

            break;
        }

        if (erase_ret) {

            failed_erase_blocks[block] =
                true;

            erase_failures++;

            fprintf(
                stderr,
                "\nWARNING: Block %u erase failed.\n",
                block
            );

        } else {

            blocks_erased++;
        }

        progress(
            "ERASE",
            block + 1,
            required_blocks
        );
    }

    printf(
        "\n"
    );

    if (interrupted)
        goto cleanup_and_exit;


    /* ========================================================
     * PROGRAM
     * ======================================================== */

    printf(
        "\n"
        "Programming %u pages...\n",
        required_pages
    );

    printf(
        "Read-back verification during programming: DISABLED\n"
    );

    printf(
        "The writer will program sequentially from page 0 "
        "through page %u.\n\n",
        required_pages - 1
    );

    last_percent =
        999;

    for (unsigned page = 0;
         page < required_pages;
         page++) {

        /*
         * Do not start a new page after Ctrl+C.
         */

        if (abort_requested()) {

            interrupted =
                true;

            break;
        }

        const uint8_t *expected =
            bin +
            ((size_t)page *
             NAND_PAGE_SIZE);

        int program_ret =
            program_page(
                spi,
                page,
                expected
            );

        if (program_ret == -3) {

            interrupted =
                true;

            break;
        }

        if (program_ret) {

            failed_program_pages[page] =
                true;

            program_failures++;

            fprintf(
                stderr,
                "\nWARNING: Page %u program failed.\n",
                page
            );

            /*
             * Continue to next page.
             *
             * This preserves the attempt-complete-image
             * recovery behaviour.
             */

        } else {

            pages_programmed++;
        }

        progress(
            "PROGRAM",
            page + 1,
            required_pages
        );
    }

    printf(
        "\n"
    );

    if (interrupted)
        goto cleanup_and_exit;


#if VERIFY_AFTER_PROGRAM

    /* ========================================================
     * FINAL VERIFICATION PASS
     * ======================================================== */

    printf(
        "\n"
        "============================================================\n"
        " FINAL VERIFICATION PASS\n"
        "============================================================\n\n"
    );

    printf(
        "Verifying %u pages...\n",
        required_pages
    );

    last_percent =
        999;

    for (unsigned page = 0;
         page < required_pages;
         page++) {

        /*
         * Do not start a new read after Ctrl+C.
         */

        if (abort_requested()) {

            interrupted =
                true;

            break;
        }

        const uint8_t *expected =
            bin +
            ((size_t)page *
             NAND_PAGE_SIZE);

        int verify_ret =
            verify_page(
                spi,
                page,
                expected,
                page_buffer
            );

        if (verify_ret == -3) {

            interrupted =
                true;

            break;
        }

        if (verify_ret) {

            failed_verify_pages[page] =
                true;

            verify_failures++;

            fprintf(
                stderr,
                "\nWARNING: Page %u verification failed.\n",
                page
            );

        } else {

            pages_verified++;
        }

        progress(
            "VERIFY",
            page + 1,
            required_pages
        );
    }

    printf(
        "\n"
    );

    if (interrupted)
        goto cleanup_and_exit;

#endif


    /* ========================================================
     * FINAL STATUS BEFORE CLEANUP
     * ======================================================== */

    {
        uint8_t final_status =
            0;

        uint8_t final_protection =
            0;

        uint8_t final_configuration =
            0;

        get_feature(
            spi,
            REG_STATUS,
            &final_status
        );

        get_feature(
            spi,
            REG_PROTECTION,
            &final_protection
        );

        get_feature(
            spi,
            REG_CONFIGURATION,
            &final_configuration
        );

        printf(
            "\n"
            "Final NAND status:\n"
            "  C0: %02X\n"
            "  A0: %02X\n"
            "  B0: %02X\n",
            final_status,
            final_protection,
            final_configuration
        );
    }


    /* ========================================================
     * CLEANUP
     * ======================================================== */

cleanup_and_exit:

    if (interrupted) {

        fprintf(
            stderr,
            "\n"
            "*** CTRL+C RECEIVED ***\n"
            "No new NAND operations will be started.\n"
        );
    }

    /*
     * IMPORTANT:
     *
     * If Ctrl+C arrived during a NAND operation,
     * that operation has already completed before we
     * reach here because wait_ready() deliberately waits
     * for OIP=0.
     */

    cleanup(
        spi
    );

    spi =
        NULL;

    opened =
        false;


    /* ========================================================
     * END TIME
     * ======================================================== */

    QueryPerformanceCounter(
        &end_time
    );

    double elapsed =
        (double)(
            end_time.QuadPart -
            start_time.QuadPart
        ) /
        (double)freq.QuadPart;


    /* ========================================================
     * FINAL SUMMARY
     * ======================================================== */

    printf(
        "\n"
        "============================================================\n"
        " NAND WRITE SUMMARY\n"
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
        "Page size            : %u bytes\n",
        NAND_PAGE_SIZE
    );

    printf(
        "Pages required       : %u\n",
        required_pages
    );

    printf(
        "Blocks required      : %u\n",
        required_blocks
    );

    printf(
        "\n"
    );

    printf(
        "Blocks erased        : %u / %u\n",
        blocks_erased,
        required_blocks
    );

    printf(
        "Pages programmed     : %u / %u\n",
        pages_programmed,
        required_pages
    );

#if VERIFY_AFTER_PROGRAM

    printf(
        "Pages verified       : %u / %u\n",
        pages_verified,
        required_pages
    );

#else

    printf(
        "Read-back verify     : DISABLED\n"
    );

#endif

    printf(
        "\n"
    );

    printf(
        "Erase failures       : %u\n",
        erase_failures
    );

    printf(
        "Program failures     : %u\n",
        program_failures
    );

#if VERIFY_AFTER_PROGRAM

    printf(
        "Verify failures      : %u\n",
        verify_failures
    );

#endif

    printf(
        "\n"
    );

    printf(
        "Original A0          : %02X\n",
        original_protection
    );

    printf(
        "Original B0          : %02X\n",
        original_configuration
    );

    printf(
        "\n"
    );


    if (interrupted) {

        printf(
            "*** INTERRUPTED BY USER ***\n"
        );

        printf(
            "NAND configuration restored.\n"
        );

        printf(
            "NAND reset performed.\n"
        );

        printf(
            "CH341 closed.\n"
        );

        printf(
            "Exit code: 130\n"
        );

        printf(
            "============================================================\n"
        );

#if VERIFY_AFTER_PROGRAM
        free(
            page_buffer
        );
#endif

        free(
            bin
        );

        SetConsoleCtrlHandler(
            console_ctrl_handler,
            FALSE
        );

        return 130;
    }


#if VERIFY_AFTER_PROGRAM

    if (erase_failures ||
        program_failures ||
        verify_failures ||
        blocks_erased != required_blocks ||
        pages_programmed != required_pages) {

        printf(
            "*** WRITE COMPLETED WITH FAILURES ***\n"
        );

    } else if (pages_verified ==
               required_pages) {

        printf(
            "*** WRITE AND VERIFY SUCCESSFUL ***\n"
        );

    } else {

        printf(
            "*** WRITE COMPLETED BUT VERIFY FAILED ***\n"
        );
    }

#else

    if (erase_failures ||
        program_failures ||
        blocks_erased != required_blocks ||
        pages_programmed != required_pages) {

        printf(
            "*** WRITE COMPLETED WITH FAILURES ***\n"
        );

    } else {

        printf(
            "*** WRITE COMPLETED SUCCESSFULLY ***\n"
        );
    }

#endif


    printf(
        "\n"
        "Elapsed time         : %.1f seconds\n",
        elapsed
    );


    /* ========================================================
     * FAILED BLOCK/PAGE REPORT
     * ======================================================== */

    bool any_failure =
        false;

    for (unsigned block = 0;
         block < required_blocks;
         block++) {

        if (failed_erase_blocks[block]) {

            if (!any_failure) {

                printf(
                    "\nFAILED OPERATIONS:\n"
                );

                any_failure =
                    true;
            }

            printf(
                "  Block %u: ERASE FAILED\n",
                block
            );
        }
    }

    for (unsigned page = 0;
         page < required_pages;
         page++) {

        if (failed_program_pages[page]) {

            if (!any_failure) {

                printf(
                    "\nFAILED OPERATIONS:\n"
                );

                any_failure =
                    true;
            }

            printf(
                "  Page %u: PROGRAM FAILED\n",
                page
            );
        }

#if VERIFY_AFTER_PROGRAM

        if (failed_verify_pages[page]) {

            if (!any_failure) {

                printf(
                    "\nFAILED OPERATIONS:\n"
                );

                any_failure =
                    true;
            }

            printf(
                "  Page %u: VERIFY FAILED\n",
                page
            );
        }

#endif
    }

    if (!any_failure) {

        printf(
            "\n"
            "No failed blocks or pages reported.\n"
        );
    }

    printf(
        "============================================================\n"
    );


#if VERIFY_AFTER_PROGRAM

    free(
        page_buffer
    );

#endif

    free(
        bin
    );

    SetConsoleCtrlHandler(
        console_ctrl_handler,
        FALSE
    );


    if (erase_failures ||
        program_failures ||
#if VERIFY_AFTER_PROGRAM
        verify_failures ||
#endif
        blocks_erased != required_blocks ||
        pages_programmed != required_pages)
        return 2;

    return 0;


/* ============================================================
 * FAILURE PATH
 * ============================================================ */

failure:

    if (opened) {

        cleanup(
            spi
        );

        spi =
            NULL;

        opened =
            false;
    }

#if VERIFY_AFTER_PROGRAM

    free(
        page_buffer
    );

#endif

    free(
        bin
    );

    SetConsoleCtrlHandler(
        console_ctrl_handler,
        FALSE
    );

    if (abort_requested())
        return 130;

    return 1;


/* ============================================================
 * FAILURE BEFORE CH341 OPEN
 * ============================================================ */

failure_before_open:

#if VERIFY_AFTER_PROGRAM

    free(
        page_buffer
    );

#endif

    free(
        bin
    );

    SetConsoleCtrlHandler(
        console_ctrl_handler,
        FALSE
    );

    return 1;
}