/*
 * ch341-nand-id-test.c
 *
 * READ-ONLY DIAGNOSTIC FOR:
 * ESMT F50L1G41LB SPI NAND
 *
 * IMPORTANT:
 *
 * READ ID uses the hardware SPI sequence:
 *
 *     9Fh + 00h(dummy) + clock ID bytes
 *
 * Expected ID:
 *
 *     C8 01 7F 7F 7F
 *
 * This program performs NO:
 *
 *     ERASE
 *     PROGRAM
 *     SET FEATURE
 *     WRITE ENABLE
 *     WRITE DISABLE
 *
 * Diagnostics:
 *
 *     1. Open CH341
 *     2. Reset NAND
 *     3. Check ready/status
 *     4. READ ID using 9F + 00 dummy
 *     5. Read feature registers A0/B0/C0
 *     6. Decode C0
 *     7. Variable-length ID-clock test
 *     8. Repeated RESET + ID test
 *     9. PAGE READ page 0
 *    10. READ CACHE page 0
 *    11. Repeated ID/status/feature test
 *    12. Final RESET
 *    13. Final status
 *    14. Proper Ctrl+C cleanup
 *
 * NO DESTRUCTIVE OPERATIONS ARE PERFORMED.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <windows.h>

#include <ufprog/osdef.h>
#include <ufprog/log.h>
#include <ufprog/spi.h>

/* ============================================================
 * DEVICE GEOMETRY
 * ============================================================ */

#define NAND_PAGE_SIZE         2048U

/* ============================================================
 * COMMANDS
 * ============================================================ */

#define CMD_RESET              0xFF
#define CMD_READ_ID            0x9F

#define CMD_GET_FEATURE        0x0F
#define CMD_SET_FEATURE        0x1F

#define CMD_PAGE_READ          0x13
#define CMD_READ_CACHE         0x03

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

static const uint8_t expected_id[5] = {
    0xC8,
    0x01,
    0x7F,
    0x7F,
    0x7F
};

/* ============================================================
 * GLOBAL Ctrl+C STATE
 * ============================================================ */

static volatile BOOL g_ctrl_c = FALSE;

static struct ufprog_spi *g_spi = NULL;

/* ============================================================
 * CTRL+C HANDLER
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

        /*
         * Do not perform SPI operations from inside the
         * Windows console handler.
         *
         * The main thread checks g_ctrl_c and performs
         * the actual cleanup.
         */

        return TRUE;

    default:
        return FALSE;
    }
}

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

    return ufprog_spi_generic_xfer(spi,
                                   xfer,
                                   n);
}

/* ============================================================
 * SEND COMMAND
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
 * RESET CHIP
 * ============================================================ */

static int reset_chip(struct ufprog_spi *spi)
{
    printf("\nSending NAND RESET (FFh)...\n");

    if (command(spi,
                CMD_RESET))
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

        if (g_ctrl_c)
            return -3;

        if (get_feature(spi,
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
 * READ ID
 *
 * F50L1G41LB READ-ID hardware SPI sequence:
 *
 *     9Fh
 *     00h dummy byte
 *     ID clocks
 *
 * The complete SPI transaction is:
 *
 *     TX: 9F 00 00 00 00 00
 *     RX: -- -- ID ID ID ID ID
 *
 * The first two transmitted bytes are command + dummy.
 * The received bytes are captured only after those two
 * transmitted bytes.
 * ============================================================ */

static int read_id(struct ufprog_spi *spi,
                   uint8_t *id,
                   size_t id_len)
{
    uint8_t tx[2] = {
        CMD_READ_ID,
        0x00
    };

    if (!id || !id_len)
        return -1;

    if (id_len > 32)
        return -1;

    memset(id,
           0,
           id_len);

    if (transfer(spi,
                 tx,
                 sizeof(tx),
                 id,
                 id_len))
        return -1;

    return 0;
}

/* ============================================================
 * PRINT ID
 * ============================================================ */

static void print_id(const uint8_t *id,
                     size_t len)
{
    printf("ID:");

    for (size_t i = 0;
         i < len;
         i++) {

        printf(" %02X",
               id[i]);
    }

    printf("\n");
}

/* ============================================================
 * CHECK EXPECTED ID
 * ============================================================ */

static int check_expected_id(const uint8_t *id)
{
    if (!id)
        return -1;

    if (memcmp(id,
               expected_id,
               sizeof(expected_id)) == 0)
        return 0;

    return -1;
}

/* ============================================================
 * DIAGNOSTIC ID
 * ============================================================ */

static int diagnostic_id(struct ufprog_spi *spi)
{
    uint8_t id[5] = {0};

    printf("\n");
    printf("READ ID command: 9Fh + 00h dummy\n");

    if (read_id(spi,
                id,
                sizeof(id))) {

        fprintf(stderr,
                "ERROR: READ ID communication failed.\n");

        return -1;
    }

    print_id(id,
             sizeof(id));

    printf("Expected ID: C8 01 7F 7F 7F\n");

    if (check_expected_id(id) == 0) {

        printf("RESULT: NAND ID MATCHES.\n");

        return 0;
    }

    printf("RESULT: NAND ID DOES NOT MATCH.\n");

    return 1;
}

/* ============================================================
 * FEATURE REGISTER DIAGNOSTIC
 * ============================================================ */

static int diagnostic_features(struct ufprog_spi *spi)
{
    uint8_t protection = 0;
    uint8_t configuration = 0;
    uint8_t status = 0;

    printf("\n");

    if (get_feature(spi,
                    REG_PROTECTION,
                    &protection)) {

        fprintf(stderr,
                "ERROR: unable to read A0 Protection register.\n");

        return -1;
    }

    if (get_feature(spi,
                    REG_CONFIGURATION,
                    &configuration)) {

        fprintf(stderr,
                "ERROR: unable to read B0 Configuration register.\n");

        return -1;
    }

    if (get_feature(spi,
                    REG_STATUS,
                    &status)) {

        fprintf(stderr,
                "ERROR: unable to read C0 Status register.\n");

        return -1;
    }

    printf("A0 Protection      : %02X\n",
           protection);

    printf("B0 Configuration   : %02X\n",
           configuration);

    printf("C0 Status          : %02X\n",
           status);

    printf("\n");

    printf("C0 decoded:\n");

    printf("OIP              : %u\n",
           (status & STATUS_OIP) ? 1U : 0U);

    printf("WEL              : %u\n",
           (status & STATUS_WEL) ? 1U : 0U);

    printf("E_FAIL           : %u\n",
           (status & STATUS_EFAIL) ? 1U : 0U);

    printf("P_FAIL           : %u\n",
           (status & STATUS_PFAIL) ? 1U : 0U);

    printf("ECC bits         : %02X\n",
           status & STATUS_ECC_MASK);

    return 0;
}

/* ============================================================
 * VARIABLE LENGTH ID TEST
 * ============================================================ */

static int variable_length_id_test(struct ufprog_spi *spi)
{
    printf("\n");

    printf("Variable-length READ ID test using 9Fh + 00h dummy...\n");

    for (unsigned len = 1;
         len <= 16;
         len++) {

        if (g_ctrl_c)
            return -3;

        uint8_t id[16] = {0};

        if (read_id(spi,
                    id,
                    len)) {

            printf("Length %2u : COMMUNICATION ERROR\n",
                   len);

            continue;
        }

        printf("Length %2u :",
               len);

        for (unsigned i = 0;
             i < len;
             i++) {

            printf(" %02X",
                   id[i]);
        }

        printf("\n");
    }

    return 0;
}

/* ============================================================
 * REPEATED RESET + ID TEST
 * ============================================================ */

static int repeated_reset_id_test(struct ufprog_spi *spi)
{
    printf("\n");

    printf("Repeated RESET + ID test...\n");

    for (unsigned attempt = 1;
         attempt <= 5;
         attempt++) {

        if (g_ctrl_c)
            return -3;

        printf("\nReset/ID attempt %u...\n",
               attempt);

        if (reset_chip(spi)) {

            fprintf(stderr,
                    "RESET failed on attempt %u.\n",
                    attempt);

            continue;
        }

        uint8_t id[5] = {0};

        if (read_id(spi,
                    id,
                    sizeof(id))) {

            fprintf(stderr,
                    "READ ID failed on attempt %u.\n",
                    attempt);

            continue;
        }

        print_id(id,
                 sizeof(id));
    }

    return 0;
}

/* ============================================================
 * PAGE READ
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

    printf("\n");
    printf("Sending PAGE READ (13h) row %06X...\n",
           page);

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

    printf("C0 after PAGE READ : %02X\n",
           status);

    printf("OIP                : %u\n",
           (status & STATUS_OIP) ? 1U : 0U);

    printf("ECC bits           : %02X\n",
           status & STATUS_ECC_MASK);

    return 0;
}

/* ============================================================
 * READ CACHE
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

    /*
     * F50L1G41LB READ CACHE sequence:
     *
     *     03h
     *     Column MSB
     *     Column LSB
     *     Dummy
     *     Data
     */

    tx[3] = 0x00;

    return transfer(spi,
                    tx,
                    sizeof(tx),
                    data,
                    len);
}

/* ============================================================
 * PAGE 0 READ/CACHE DIAGNOSTIC
 * ============================================================ */

static int diagnostic_page_zero(struct ufprog_spi *spi)
{
    uint8_t data[16] = {0};

    uint8_t status = 0;

    if (page_read(spi,
                  0))
        return -1;

    printf("\n");

    printf("Sending READ CACHE (03h), column 0000...\n");

    if (read_cache(spi,
                   0x0000,
                   data,
                   sizeof(data))) {

        fprintf(stderr,
                "ERROR: READ CACHE communication failed.\n");

        return -1;
    }

    printf("Cache data:");

    for (unsigned i = 0;
         i < sizeof(data);
         i++) {

        printf(" %02X",
               data[i]);
    }

    printf("\n");

    if (get_feature(spi,
                    REG_STATUS,
                    &status)) {

        fprintf(stderr,
                "ERROR: unable to read C0 after READ CACHE.\n");

        return -1;
    }

    printf("C0 after READ CACHE: %02X\n",
           status);

    printf("ECC bits           : %02X\n",
           status & STATUS_ECC_MASK);

    return 0;
}

/* ============================================================
 * REPEATED ID ONLY
 * ============================================================ */

static int repeated_id_test(struct ufprog_spi *spi)
{
    printf("\n");

    printf("Repeated READ ID test...\n");

    for (unsigned attempt = 1;
         attempt <= 10;
         attempt++) {

        if (g_ctrl_c)
            return -3;

        uint8_t id[5] = {0};

        if (read_id(spi,
                    id,
                    sizeof(id))) {

            printf("Attempt %u: COMMUNICATION ERROR\n",
                   attempt);

            continue;
        }

        printf("Attempt %u:",
               attempt);

        for (unsigned i = 0;
             i < sizeof(id);
             i++) {

            printf(" %02X",
                   id[i]);
        }

        printf("\n");
    }

    return 0;
}

/* ============================================================
 * COMBINED ID + FEATURE TEST
 * ============================================================ */

static int combined_test(struct ufprog_spi *spi)
{
    printf("\n");

    printf("Combined RESET/STATUS/ID/FEATURE test...\n");

    for (unsigned attempt = 1;
         attempt <= 10;
         attempt++) {

        if (g_ctrl_c)
            return -3;

        uint8_t id[5] = {0};

        uint8_t protection = 0;
        uint8_t configuration = 0;
        uint8_t status = 0;

        if (reset_chip(spi)) {

            printf("Attempt %u: RESET FAIL\n",
                   attempt);

            continue;
        }

        if (get_feature(spi,
                        REG_PROTECTION,
                        &protection)) {

            printf("Attempt %u: A0 READ FAIL\n",
                   attempt);

            continue;
        }

        if (get_feature(spi,
                        REG_CONFIGURATION,
                        &configuration)) {

            printf("Attempt %u: B0 READ FAIL\n",
                   attempt);

            continue;
        }

        if (get_feature(spi,
                        REG_STATUS,
                        &status)) {

            printf("Attempt %u: C0 READ FAIL\n",
                   attempt);

            continue;
        }

        if (read_id(spi,
                    id,
                    sizeof(id))) {

            printf("Attempt %u: ID READ FAIL\n",
                   attempt);

            continue;
        }

        printf("Attempt %u: A0=%02X B0=%02X C0=%02X ID=",
               attempt,
               protection,
               configuration,
               status);

        for (unsigned i = 0;
             i < sizeof(id);
             i++) {

            printf("%02X",
                   id[i]);

            if (i + 1 < sizeof(id))
                printf(" ");
        }

        printf("\n");
    }

    return 0;
}

/* ============================================================
 * CLEANUP
 * ============================================================ */

static int cleanup_and_return(struct ufprog_spi *spi,
                              int code)
{
    printf("\nClosing CH341...\n");

    if (spi)
        ufprog_spi_close_device(spi);

    g_spi = NULL;

    SetConsoleCtrlHandler(console_handler,
                           FALSE);

    return code;
}

/* ============================================================
 * MAIN
 * ============================================================ */

int wmain(void)
{
    ufprog_status ret;

    struct ufprog_spi *spi = NULL;

    uint8_t status = 0;

    int id_result = -1;

    /*
     * Install Ctrl+C handler before opening programmer.
     */

    if (!SetConsoleCtrlHandler(console_handler,
                               TRUE)) {

        fprintf(stderr,
                "WARNING: unable to install Ctrl+C handler.\n");
    }

    /* ========================================================
     * UFPROG INITIALIZATION
     * ======================================================== */

    set_os_default_log_print();

    os_init();

    /* ========================================================
     * OPEN CH341
     * ======================================================== */

    printf("Opening CH341...\n");

    ret =
        ufprog_spi_open_device(
            "ch341-libusb",
            false,
            &spi);

    if (ret) {

        fprintf(stderr,
                "\nERROR: CH341 open failed: %u\n",
                ret);

        SetConsoleCtrlHandler(console_handler,
                               FALSE);

        return 1;
    }

    g_spi = spi;

    /* ========================================================
     * CHECK CTRL+C
     * ======================================================== */

    if (g_ctrl_c) {

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("Cleaning up...\n");

        return cleanup_and_return(spi,
                                  130);
    }

    /* ========================================================
     * RESET
     * ======================================================== */

    if (reset_chip(spi)) {

        fprintf(stderr,
                "\nERROR: NAND RESET failed.\n");

        return cleanup_and_return(spi,
                                  1);
    }

    /* ========================================================
     * WAIT FOR READY
     * ======================================================== */

    printf("Checking NAND ready/status...\n");

    if (wait_ready(spi,
                   1000,
                   &status)) {

        fprintf(stderr,
                "\nERROR: NAND did not become ready.\n");

        return cleanup_and_return(spi,
                                  1);
    }

    printf("Initial C0 Status   : %02X\n",
           status);

    /* ========================================================
     * FIRST ID TEST
     * ======================================================== */

    id_result =
        diagnostic_id(spi);

    if (id_result < 0) {

        fprintf(stderr,
                "\nERROR: NAND ID communication failed.\n");

        return cleanup_and_return(spi,
                                  1);
    }

    /* ========================================================
     * FEATURE REGISTER TEST
     * ======================================================== */

    if (g_ctrl_c) {

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("Cleaning up...\n");

        return cleanup_and_return(spi,
                                  130);
    }

    if (diagnostic_features(spi)) {

        fprintf(stderr,
                "\nERROR: feature-register diagnostic failed.\n");

        return cleanup_and_return(spi,
                                  1);
    }

    /* ========================================================
     * VARIABLE LENGTH ID TEST
     * ======================================================== */

    if (g_ctrl_c) {

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("Cleaning up...\n");

        return cleanup_and_return(spi,
                                  130);
    }

    if (variable_length_id_test(spi)) {

        if (g_ctrl_c) {

            printf("\n");
            printf("Ctrl+C received.\n");
            printf("Cleaning up...\n");

            return cleanup_and_return(spi,
                                      130);
        }
    }

    /* ========================================================
     * REPEATED RESET + ID TEST
     * ======================================================== */

    if (g_ctrl_c) {

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("Cleaning up...\n");

        return cleanup_and_return(spi,
                                  130);
    }

    if (repeated_reset_id_test(spi)) {

        if (g_ctrl_c) {

            printf("\n");
            printf("Ctrl+C received.\n");
            printf("Cleaning up...\n");

            return cleanup_and_return(spi,
                                      130);
        }
    }

    /* ========================================================
     * PAGE 0 READ/CACHE TEST
     * ======================================================== */

    if (g_ctrl_c) {

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("Cleaning up...\n");

        return cleanup_and_return(spi,
                                  130);
    }

    if (diagnostic_page_zero(spi)) {

        if (g_ctrl_c) {

            printf("\n");
            printf("Ctrl+C received.\n");
            printf("Cleaning up...\n");

            return cleanup_and_return(spi,
                                      130);
        }

        fprintf(stderr,
                "\nWARNING: PAGE READ/READ CACHE diagnostic failed.\n");
    }

    /* ========================================================
     * REPEATED ID TEST
     * ======================================================== */

    if (g_ctrl_c) {

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("Cleaning up...\n");

        return cleanup_and_return(spi,
                                  130);
    }

    if (repeated_id_test(spi)) {

        if (g_ctrl_c) {

            printf("\n");
            printf("Ctrl+C received.\n");
            printf("Cleaning up...\n");

            return cleanup_and_return(spi,
                                      130);
        }
    }

    /* ========================================================
     * COMBINED TEST
     * ======================================================== */

    if (g_ctrl_c) {

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("Cleaning up...\n");

        return cleanup_and_return(spi,
                                  130);
    }

    if (combined_test(spi)) {

        if (g_ctrl_c) {

            printf("\n");
            printf("Ctrl+C received.\n");
            printf("Cleaning up...\n");

            return cleanup_and_return(spi,
                                      130);
        }
    }

    /* ========================================================
     * FINAL RESET
     *
     * Still completely non-destructive.
     * ======================================================== */

    if (!g_ctrl_c) {

        printf("\n");
        printf("Performing final NAND RESET...\n");

        if (reset_chip(spi)) {

            fprintf(stderr,
                    "\nWARNING: final RESET failed.\n");
        }
    }

    /* ========================================================
     * FINAL STATUS
     * ======================================================== */

    if (!g_ctrl_c) {

        if (!get_feature(spi,
                         REG_STATUS,
                         &status)) {

            printf("Final C0 Status     : %02X\n",
                   status);
        }
    }

    /* ========================================================
     * RESULT
     * ======================================================== */

    printf("\n");
    printf("============================================================\n");
    printf(" DIAGNOSTIC RESULT\n");
    printf("============================================================\n");

    if (g_ctrl_c) {

        printf("Ctrl+C received.\n");
        printf("No destructive operation was performed.\n");

    } else if (id_result == 0) {

        printf("NAND ID is correct:\n");
        printf("C8 01 7F 7F 7F\n");
        printf("\n");
        printf("The NAND is responding with the expected ID.\n");

    } else {

        printf("NAND ID is NOT correct.\n");
        printf("\n");
        printf("No erase was performed.\n");
        printf("No program operation was performed.\n");
        printf("No feature register was modified.\n");
    }

    printf("============================================================\n");

    /* ========================================================
     * CLEANUP
     * ======================================================== */

    return cleanup_and_return(
        spi,
        g_ctrl_c ? 130 : (id_result == 0 ? 0 : 2)
    );
}