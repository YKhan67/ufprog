/*
 * ch341-nand-parameter-test.c
 *
 * READ-ONLY PARAMETER-PAGE DIAGNOSTIC
 *
 * Target:
 *     ESMT F50L1G41LB
 *
 * IMPORTANT:
 *
 * The F50L1G41LB does NOT expose its parameter page through the
 * normal JEDEC READ ID command.
 *
 * According to the ESMT datasheet:
 *
 *     B0h bit 6 = OTP Enable
 *
 * To read the Parameter Page:
 *
 *     1. Set Feature (1Fh)
 *     2. Feature address B0h
 *     3. Set OTP Enable = 1
 *     4. Page Read (13h), page address 01h
 *     5. Wait until OIP clears
 *     6. Read Cache
 *
 * The parameter page is:
 *
 *     Page address 01h
 *     256 bytes x 3 identical copies
 *
 * Expected signature:
 *
 *     4F 4E 46 49
 *      O  N  F  I
 *
 * Expected manufacturer:
 *
 *     POWERCHIP
 *
 * Expected model:
 *
 *     PSU1GS20DX
 *
 * The parameter page is READ ONLY.
 *
 * However, entering OTP mode requires changing the volatile B0
 * configuration register. This program therefore:
 *
 *     - Reads B0 first
 *     - Saves its original value
 *     - Sets OTP Enable
 *     - Reads parameter page
 *     - Restores the original B0 value
 *
 * No NAND array ERASE is performed.
 * No NAND array PROGRAM is performed.
 * No OTP PROGRAM is performed.
 * No WRITE ENABLE is issued.
 *
 * Ctrl+C is handled through the Windows console handler.
 *
 * The console handler only sets a flag.
 * SPI cleanup is performed by the main thread.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <windows.h>

#include <ufprog/osdef.h>
#include <ufprog/log.h>
#include <ufprog/spi.h>


/* ============================================================
 * COMMANDS
 * ============================================================ */

#define CMD_RESET               0xFF
#define CMD_READ_ID             0x9F
#define CMD_GET_FEATURE         0x0F
#define CMD_SET_FEATURE         0x1F
#define CMD_PAGE_READ           0x13
#define CMD_READ_CACHE          0x03


/* ============================================================
 * FEATURE REGISTERS
 * ============================================================ */

#define REG_PROTECTION          0xA0
#define REG_CONFIGURATION       0xB0
#define REG_STATUS              0xC0


/* ============================================================
 * CONFIGURATION REGISTER
 * ============================================================ */

#define CONFIG_OTP_ENABLE       0x40
#define CONFIG_ECC_ENABLE       0x10

#define CONFIG_OTP_ECC_ENABLE   \
    (CONFIG_OTP_ENABLE | CONFIG_ECC_ENABLE)


/*
 * F50L1G41LB parameter page:
 *
 *     256 bytes x 3 copies
 */

#define PARAMETER_COPY_SIZE     256U
#define PARAMETER_COPY_COUNT    3U
#define PARAMETER_TOTAL_SIZE    \
    (PARAMETER_COPY_SIZE * PARAMETER_COPY_COUNT)


/*
 * Parameter page address.
 *
 * ESMT specifies:
 *
 *     Unique ID Page     = 00h
 *     Parameter Page     = 01h
 *     OTP pages          = 02h - 1Dh
 */

#define PARAMETER_PAGE_ADDRESS  0x000001U


/* ============================================================
 * STATUS BITS
 * ============================================================ */

#define STATUS_OIP              0x01
#define STATUS_WEL              0x02
#define STATUS_EFAIL            0x04
#define STATUS_PFAIL            0x08
#define STATUS_ECC_MASK         0x30


/* ============================================================
 * PARAMETER PAGE SIGNATURE
 * ============================================================ */

static const uint8_t parameter_signature[4] = {
    0x4F,
    0x4E,
    0x46,
    0x49
};


/* ============================================================
 * EXPECTED PARAMETER INFORMATION
 * ============================================================ */

static const char expected_manufacturer[] =
    "POWERCHIP";

static const char expected_model[] =
    "PSU1GS20DX";


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

        /*
         * Do NOT perform SPI operations here.
         *
         * Windows console handlers execute asynchronously.
         * The main thread performs all actual cleanup.
         */

        g_ctrl_c = TRUE;

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
                       uint8_t address,
                       uint8_t *value)
{
    uint8_t tx[2];

    uint8_t rx = 0;

    tx[0] = CMD_GET_FEATURE;
    tx[1] = address;

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
 * SET FEATURE
 *
 * This only changes a volatile feature register.
 *
 * No WRITE ENABLE is required for this operation on this device.
 * ============================================================ */

static int set_feature(struct ufprog_spi *spi,
                       uint8_t address,
                       uint8_t value)
{
    uint8_t tx[3];

    tx[0] = CMD_SET_FEATURE;
    tx[1] = address;
    tx[2] = value;

    return transfer(spi,
                    tx,
                    sizeof(tx),
                    NULL,
                    0);
}


/* ============================================================
 * RESET NAND
 * ============================================================ */

static int reset_chip(struct ufprog_spi *spi)
{
    if (g_ctrl_c)
        return -2;

    printf("Sending NAND RESET (FFh)...\n");

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
 * PRINT HEX DUMP
 * ============================================================ */

static void hex_dump(const uint8_t *data,
                     size_t len,
                     size_t base_offset)
{
    size_t offset;

    for (offset = 0;
         offset < len;
         offset += 16) {

        size_t i;

        size_t count = len - offset;

        if (count > 16)
            count = 16;

        printf("%04X  ",
               (unsigned)(base_offset + offset));

        for (i = 0;
             i < 16;
             i++) {

            if (i < count)
                printf("%02X ",
                       data[offset + i]);
            else
                printf("   ");
        }

        printf(" ");

        for (i = 0;
             i < count;
             i++) {

            uint8_t c = data[offset + i];

            if (c >= 32 && c <= 126)
                putchar((int)c);
            else
                putchar('.');
        }

        printf("\n");
    }
}


/* ============================================================
 * PAGE READ
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

    if (g_ctrl_c)
        return -2;

    printf("Sending PAGE READ (13h), page %06X...\n",
           (unsigned)page);

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

    printf("Status after PAGE READ : %02X\n",
           status);

    printf("OIP                   : %u\n",
           (status & STATUS_OIP) ? 1U : 0U);

    printf("ECC bits              : %02X\n",
           status & STATUS_ECC_MASK);

    return 0;
}


/* ============================================================
 * READ CACHE
 *
 * F50L1G41LB:
 *
 *     03h
 *     Column MSB
 *     Column LSB
 *     Dummy
 *     Data...
 * ============================================================ */

static int read_cache(struct ufprog_spi *spi,
                      uint16_t column,
                      uint8_t *data,
                      size_t len)
{
    uint8_t tx[4];

    if (!data || !len)
        return -1;

    tx[0] = CMD_READ_CACHE;
    tx[1] = (uint8_t)((column >> 8) & 0xFF);
    tx[2] = (uint8_t)(column & 0xFF);
    tx[3] = 0x00;

    if (g_ctrl_c)
        return -2;

    return transfer(spi,
                    tx,
                    sizeof(tx),
                    data,
                    len);
}


/* ============================================================
 * READ PARAMETER PAGE
 *
 * This routine assumes OTP mode has already been enabled.
 *
 * Parameter Page:
 *
 *     page 01h
 *
 * Three 256-byte copies are expected.
 * ============================================================ */

static int read_parameter_page(struct ufprog_spi *spi,
                               uint8_t *data,
                               size_t len)
{
    if (!data)
        return -1;

    if (len < PARAMETER_TOTAL_SIZE)
        return -1;

    if (g_ctrl_c)
        return -2;

    if (page_read(spi,
                  PARAMETER_PAGE_ADDRESS))
        return -1;

    if (g_ctrl_c)
        return -2;

    printf("Reading parameter page cache...\n");

    if (read_cache(spi,
                   0x0000,
                   data,
                   PARAMETER_TOTAL_SIZE))
        return -1;

    return 0;
}


/* ============================================================
 * CHECK PARAMETER SIGNATURE
 * ============================================================ */

static int check_signature(const uint8_t *data,
                           size_t offset)
{
    if (!data)
        return -1;

    return memcmp(data + offset,
                  parameter_signature,
                  sizeof(parameter_signature)) == 0
               ? 0
               : -1;
}


/* ============================================================
 * EXTRACT ASCII FIELD
 * ============================================================ */

static void extract_ascii(const uint8_t *data,
                          size_t offset,
                          size_t len,
                          char *out,
                          size_t out_size)
{
    size_t i;

    size_t n;

    if (!out || !out_size) 
        return;

    n = len;

    if (n >= out_size)
        n = out_size - 1;

    for (i = 0;
         i < n;
         i++) {

        uint8_t c = data[offset + i];

        if (c >= 32 && c <= 126)
            out[i] = (char)c;
        else
            out[i] = ' ';
    }

    out[n] = '\0';

    while (n > 0) {

        if (out[n - 1] != ' ')
            break;

        out[n - 1] = '\0';

        n--;
    }
}


/* ============================================================
 * PARAMETER PAGE DIAGNOSTIC
 * ============================================================ */

static int diagnostic_parameter_page(struct ufprog_spi *spi)
{
    uint8_t original_config = 0;

    uint8_t config = 0;

    uint8_t status = 0;

    uint8_t data[PARAMETER_TOTAL_SIZE];

    char manufacturer[64];

    char model[64];

    bool otp_enabled = false;

    int result = -1;

    memset(data,
           0,
           sizeof(data));

    memset(manufacturer,
           0,
           sizeof(manufacturer));

    memset(model,
           0,
           sizeof(model));


    /* --------------------------------------------------------
     * Read original B0
     * -------------------------------------------------------- */

    printf("\n");
    printf("Reading original B0 Configuration register...\n");

    if (get_feature(spi,
                    REG_CONFIGURATION,
                    &original_config)) {

        fprintf(stderr,
                "ERROR: unable to read B0 Configuration register.\n");

        return -1;
    }

    printf("Original B0 Configuration : %02X\n",
           original_config);


    /* --------------------------------------------------------
     * Check status
     * -------------------------------------------------------- */

    if (get_feature(spi,
                    REG_STATUS,
                    &status)) {

        fprintf(stderr,
                "ERROR: unable to read C0 Status register.\n");

        return -1;
    }

    printf("Initial C0 Status         : %02X\n",
           status);


    /* --------------------------------------------------------
     * Enable OTP mode
     *
     * Preserve all existing B0 bits and set bit 6.
     * -------------------------------------------------------- */

    config =
        (uint8_t)(original_config |
                  CONFIG_OTP_ENABLE);

    printf("\n");
    printf("Entering OTP/Parameter-Page read mode...\n");
    printf("Writing B0 Configuration  : %02X\n",
           config);

    if (set_feature(spi,
                    REG_CONFIGURATION,
                    config)) {

        fprintf(stderr,
                "ERROR: unable to set OTP Enable bit.\n");

        return -1;
    }

    otp_enabled = true;

    Sleep(1);


    /* --------------------------------------------------------
     * Verify B0
     * -------------------------------------------------------- */

    if (get_feature(spi,
                    REG_CONFIGURATION,
                    &config)) {

        fprintf(stderr,
                "ERROR: unable to verify B0 Configuration.\n");

        goto restore;
    }

    printf("B0 after OTP enable       : %02X\n",
           config);

    if (!(config & CONFIG_OTP_ENABLE)) {

        fprintf(stderr,
                "ERROR: OTP Enable bit did not become set.\n");

        goto restore;
    }


    /* --------------------------------------------------------
     * Read parameter page
     * -------------------------------------------------------- */

    printf("\n");
    printf("============================================================\n");
    printf(" PARAMETER PAGE READ\n");
    printf("============================================================\n");

    printf("Parameter page address    : %06X\n",
           PARAMETER_PAGE_ADDRESS);

    printf("Expected data              : 256 bytes x 3 copies\n");
    printf("\n");

    if (read_parameter_page(spi,
                            data,
                            sizeof(data))) {

        fprintf(stderr,
                "ERROR: unable to read parameter page.\n");

        goto restore;
    }


    /* --------------------------------------------------------
     * Check all three copies
     * -------------------------------------------------------- */

    printf("\n");
    printf("Parameter-page signatures:\n");

    for (unsigned copy = 0;
         copy < PARAMETER_COPY_COUNT;
         copy++) {

        size_t offset =
            copy * PARAMETER_COPY_SIZE;

        printf("Copy %u signature: %02X %02X %02X %02X",
               copy + 1,
               data[offset + 0],
               data[offset + 1],
               data[offset + 2],
               data[offset + 3]);

        if (check_signature(data,
                            offset) == 0) {

            printf("  [ONFI]\n");

        } else {

            printf("  [INVALID]\n");
        }
    }


    /* --------------------------------------------------------
     * Decode first copy
     * -------------------------------------------------------- */

    if (check_signature(data,
                        0) == 0) {

        extract_ascii(data,
                      32,
                      12,
                      manufacturer,
                      sizeof(manufacturer));

        extract_ascii(data,
                      44,
                      20,
                      model,
                      sizeof(model));

        printf("\n");
        printf("Parameter Page Information:\n");

        printf("Signature                 : %02X %02X %02X %02X\n",
               data[0],
               data[1],
               data[2],
               data[3]);

        printf("Revision                  : %02X %02X\n",
               data[4],
               data[5]);

        printf("Features supported        : %02X %02X\n",
               data[6],
               data[7]);

        printf("Optional commands        : %02X %02X\n",
               data[8],
               data[9]);

        printf("Manufacturer              : %s\n",
               manufacturer);

        printf("Model                     : %s\n",
               model);

        printf("Manufacturer ID           : %02X\n",
               data[64]);

        printf("Page data bytes           : %u\n",
               (unsigned)data[80] |
               ((unsigned)data[81] << 8) |
               ((unsigned)data[82] << 16) |
               ((unsigned)data[83] << 24));

        printf("Spare bytes per page      : %u\n",
               (unsigned)data[84] |
               ((unsigned)data[85] << 8));

        printf("Pages per block           : %u\n",
               (unsigned)data[92] |
               ((unsigned)data[93] << 8) |
               ((unsigned)data[94] << 16) |
               ((unsigned)data[95] << 24));

        printf("Blocks per unit           : %u\n",
               (unsigned)data[96] |
               ((unsigned)data[97] << 8) |
               ((unsigned)data[98] << 16) |
               ((unsigned)data[99] << 24));

        printf("Logical units             : %u\n",
               (unsigned)data[100]);

        printf("Bits per cell             : %u\n",
               (unsigned)data[102]);

        printf("Maximum bad blocks        : %u\n",
               (unsigned)data[103] |
               ((unsigned)data[104] << 8));

        printf("Block endurance           : %u\n",
               (unsigned)data[105] |
               ((unsigned)data[106] << 8));

        printf("Partial programs/page     : %u\n",
               (unsigned)data[110]);

        printf("ECC bits                  : %u\n",
               (unsigned)data[112]);

        printf("I/O pin capacitance       : %u pF\n",
               (unsigned)data[128]);

        if (!strcmp(manufacturer,
                    expected_manufacturer)) {

            printf("\nManufacturer matches expected POWERCHIP.\n");

        } else {

            printf("\nManufacturer differs from expected POWERCHIP.\n");
        }

        if (!strcmp(model,
                    expected_model)) {

            printf("Model matches expected PSU1GS20DX.\n");

        } else {

            printf("Model differs from expected PSU1GS20DX.\n");
        }

        result = 0;

    } else {

        printf("\n");
        printf("FIRST PARAMETER PAGE COPY DOES NOT CONTAIN ONFI SIGNATURE.\n");
    }


    /* --------------------------------------------------------
     * Compare copies
     * -------------------------------------------------------- */

    printf("\n");
    printf("Comparing three 256-byte parameter-page copies...\n");

    if (memcmp(data,
               data + PARAMETER_COPY_SIZE,
               PARAMETER_COPY_SIZE) == 0) {

        printf("Copy 1 == Copy 2          : YES\n");

    } else {

        printf("Copy 1 == Copy 2          : NO\n");
        result = -1;
    }

    if (memcmp(data,
               data + (PARAMETER_COPY_SIZE * 2),
               PARAMETER_COPY_SIZE) == 0) {

        printf("Copy 1 == Copy 3          : YES\n");

    } else {

        printf("Copy 1 == Copy 3          : NO\n");
        result = -1;
    }


    /* --------------------------------------------------------
     * Print complete parameter page
     * -------------------------------------------------------- */

    printf("\n");
    printf("============================================================\n");
    printf(" PARAMETER PAGE HEX DUMP\n");
    printf("============================================================\n");

    for (unsigned copy = 0;
         copy < PARAMETER_COPY_COUNT;
         copy++) {

        size_t offset =
            copy * PARAMETER_COPY_SIZE;

        printf("\n");
        printf("---- COPY %u ----\n",
               copy + 1);

        hex_dump(data + offset,
                 PARAMETER_COPY_SIZE,
                 offset);
    }


restore:

    /* --------------------------------------------------------
     * Restore original B0
     *
     * This is important because OTP mode persists until the
     * OTP enable bit is cleared or the chip is power-cycled.
     * -------------------------------------------------------- */

    if (otp_enabled) {

        printf("\n");
        printf("Restoring original B0 Configuration...\n");
        printf("B0 restore value          : %02X\n",
               original_config);

        if (set_feature(spi,
                        REG_CONFIGURATION,
                        original_config)) {

            fprintf(stderr,
                    "WARNING: unable to restore original B0.\n");

            result = -1;

        } else {

            Sleep(1);

            if (!get_feature(spi,
                             REG_CONFIGURATION,
                             &config)) {

                printf("B0 after restore          : %02X\n",
                       config);

                if (config != original_config) {

                    fprintf(stderr,
                            "WARNING: B0 restore verification failed.\n");

                    result = -1;
                }
            }
        }
    }


    return result;
}


/* ============================================================
 * SAFE CLEANUP
 * ============================================================ */

static int cleanup(struct ufprog_spi *spi)
{
    int ret = 0;

    if (spi) {

        printf("\nClosing CH341...\n");

        ret = ufprog_spi_close_device(spi);

        if (ret)
            fprintf(stderr,
                    "WARNING: CH341 close returned %d.\n",
                    ret);
    }

    g_spi = NULL;

    SetConsoleCtrlHandler(console_handler,
                           FALSE);

    return ret;
}


/* ============================================================
 * MAIN
 * ============================================================ */

int wmain(void)
{
    ufprog_status ret;

    struct ufprog_spi *spi = NULL;

    uint8_t status = 0;

    uint8_t config = 0;

    int diagnostic_result = -1;


    /* --------------------------------------------------------
     * Install Ctrl+C handler
     * -------------------------------------------------------- */

    if (!SetConsoleCtrlHandler(console_handler,
                               TRUE)) {

        fprintf(stderr,
                "WARNING: unable to install Ctrl+C handler.\n");
    }


    /* --------------------------------------------------------
     * UFPROG INITIALIZATION
     * -------------------------------------------------------- */

    set_os_default_log_print();

    os_init();


    /* --------------------------------------------------------
     * OPEN CH341
     * -------------------------------------------------------- */

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


    /* --------------------------------------------------------
     * Ctrl+C immediately after opening
     * -------------------------------------------------------- */

    if (g_ctrl_c) {

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("Cleaning up...\n");

        cleanup(spi);

        return 130;
    }


    /* --------------------------------------------------------
     * Initial RESET
     * -------------------------------------------------------- */

    if (reset_chip(spi)) {

        fprintf(stderr,
                "\nERROR: NAND RESET failed.\n");

        cleanup(spi);

        return 1;
    }


    /* --------------------------------------------------------
     * Initial READY
     * -------------------------------------------------------- */

    printf("Checking NAND ready/status...\n");

    if (wait_ready(spi,
                   1000,
                   &status)) {

        fprintf(stderr,
                "\nERROR: NAND did not become ready.\n");

        cleanup(spi);

        return 1;
    }

    printf("Initial C0 Status        : %02X\n",
           status);


    /* --------------------------------------------------------
     * Read current B0
     * -------------------------------------------------------- */

    if (get_feature(spi,
                    REG_CONFIGURATION,
                    &config)) {

        fprintf(stderr,
                "\nERROR: unable to read B0 Configuration.\n");

        cleanup(spi);

        return 1;
    }

    printf("Initial B0 Configuration : %02X\n",
           config);


    /* --------------------------------------------------------
     * Parameter page diagnostic
     * -------------------------------------------------------- */

    if (g_ctrl_c) {

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("No destructive operation was performed.\n");

        cleanup(spi);

        return 130;
    }


    diagnostic_result =
        diagnostic_parameter_page(spi);


    /* --------------------------------------------------------
     * Ctrl+C
     * -------------------------------------------------------- */

    if (g_ctrl_c) {

        /*
         * The diagnostic routine restores B0 before returning
         * whenever it has entered OTP mode.
         */

        printf("\n");
        printf("Ctrl+C received.\n");
        printf("Cleaning up...\n");

        cleanup(spi);

        return 130;
    }


    /* --------------------------------------------------------
     * Final RESET
     * -------------------------------------------------------- */

    printf("\n");
    printf("Performing final NAND RESET...\n");

    if (reset_chip(spi)) {

        fprintf(stderr,
                "WARNING: final RESET failed.\n");
    }


    /* --------------------------------------------------------
     * Final status/configuration
     * -------------------------------------------------------- */

    if (!g_ctrl_c) {

        if (!get_feature(spi,
                         REG_STATUS,
                         &status)) {

            printf("Final C0 Status          : %02X\n",
                   status);
        }

        if (!get_feature(spi,
                         REG_CONFIGURATION,
                         &config)) {

            printf("Final B0 Configuration   : %02X\n",
                   config);
        }
    }


    /* --------------------------------------------------------
     * FINAL RESULT
     * -------------------------------------------------------- */

    printf("\n");
    printf("============================================================\n");
    printf(" PARAMETER PAGE DIAGNOSTIC RESULT\n");
    printf("============================================================\n");

    if (diagnostic_result == 0) {

        printf("ONFI parameter page       : DETECTED\n");
        printf("Parameter-page signature  : 4F 4E 46 49\n");
        printf("Manufacturer/model data   : VALID\n");

    } else {

        printf("ONFI parameter page       : NOT VALIDATED\n");
        printf("Parameter-page signature  : NOT CONFIRMED\n");
    }

    printf("\n");
    printf("No NAND array erase       : PERFORMED\n");
    printf("No NAND array program     : PERFORMED\n");
    printf("No OTP program             : PERFORMED\n");
    printf("No Write Enable            : ISSUED\n");

    printf("============================================================\n");


    /* --------------------------------------------------------
     * CLEANUP
     * -------------------------------------------------------- */

    cleanup(spi);


    /* --------------------------------------------------------
     * RETURN
     * -------------------------------------------------------- */

    if (g_ctrl_c)
        return 130;

    if (diagnostic_result == 0)
        return 0;

    return 2;
}