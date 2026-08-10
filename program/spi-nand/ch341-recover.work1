#include <stdio.h>
#include <stdint.h>
#include <windows.h>

#include <ufprog/osdef.h>
#include <ufprog/log.h>
#include <ufprog/spi.h>

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

return ufprog_spi_generic_xfer(spi, xfer, n);

}

static int command(struct ufprog_spi *spi, uint8_t opcode)
{
return transfer(spi, &opcode, 1, NULL, 0);
}

static int read_feature(struct ufprog_spi *spi, uint8_t addr,
const char *name, uint8_t *value)
{
uint8_t tx[2] = {0x0F, addr};
uint8_t rx = 0;

if (transfer(spi, tx, sizeof(tx), &rx, 1)) {
    printf("%s: READ FAILED\n", name);
    return -1;
}

*value = rx;

printf("%-18s: %02X\n", name, rx);
return 0;

}

static int write_enable(struct ufprog_spi *spi)
{
uint8_t value;

if (command(spi, 0x06)) {
    printf("WRITE ENABLE failed\n");
    return -1;
}

if (read_feature(spi, 0xC0, "Status", &value))
    return -1;

printf("WEL = %u\n", (value >> 1) & 1);

return ((value & 0x02) != 0) ? 0 : -1;

}

static int set_feature(struct ufprog_spi *spi, uint8_t addr, uint8_t value)
{
uint8_t tx[3] = {0x1F, addr, value};

return transfer(spi, tx, sizeof(tx), NULL, 0);

}

static int wait_ready(struct ufprog_spi *spi)
{
unsigned i;
uint8_t status;

for (i = 0; i < 1000; i++) {
    if (read_feature(spi, 0xC0, "Status", &status))
        return -1;

    if (!(status & 0x01))
        return 0;

    Sleep(1);
}

printf("TIMEOUT waiting for ready\n");
return -1;

}

static int read_id(struct ufprog_spi *spi)
{
uint8_t tx = 0x9F;
uint8_t rx[8] = {0};

if (transfer(spi, &tx, 1, rx, sizeof(rx))) {
    printf("READ ID failed\n");
    return -1;
}

printf("ID:");
for (unsigned i = 0; i < sizeof(rx); i++)
    printf(" %02X", rx[i]);
printf("\n");

return 0;

}

static void dump_result(const char *label, uint8_t value)
{
printf("%s = %02X\n", label, value);
}

int wmain(void)
{
struct ufprog_spi *spi = NULL;
ufprog_status ret;
uint8_t original_b0 = 0;
uint8_t test_b0;
uint8_t status;

set_os_default_log_print();
os_init();

printf("=== ESMT F50L1G41LB FEATURE / DEVICE RESPONSE TEST ===\n");
printf("NO PROGRAM / NO ERASE\n\n");

ret = ufprog_spi_open_device("ch341-libusb", false, &spi);
if (ret) {
    fprintf(stderr, "open failed: %u\n", ret);
    return 1;
}

printf("RESET\n");

if (command(spi, 0xFF)) {
    printf("RESET FAILED\n");
    ufprog_spi_close_device(spi);
    return 1;
}

Sleep(2);

printf("RESET OK\n\n");

printf("FEATURES BEFORE TEST\n");

if (read_feature(spi, 0xA0, "Protection (A0)", &status))
    goto cleanup;

if (read_feature(spi, 0xB0, "Configuration (B0)", &original_b0))
    goto cleanup;

if (read_feature(spi, 0xC0, "Status (C0)", &status))
    goto cleanup;

printf("\n");

printf("READ ID BEFORE FEATURE TEST\n");
read_id(spi);

printf("\n");
printf("========================================\n");
printf("SAFE FEATURE WRITE / READBACK TEST\n");
printf("========================================\n\n");

/*
 * B0 is the configuration register.
 *
 * We first verify that the device accepts WREN and that the WEL
 * bit changes. Then we write the same B0 value back to the device.
 *
 * This deliberately does NOT touch NAND array data.
 */

test_b0 = original_b0;

printf("Original B0: %02X\n", original_b0);
printf("Test B0:     %02X\n\n", test_b0);

printf("WRITE ENABLE\n");

if (write_enable(spi)) {
    printf("WRITE ENABLE / WEL TEST FAILED\n");
    goto cleanup;
}

printf("\nSET FEATURE B0 = %02X\n", test_b0);

if (set_feature(spi, 0xB0, test_b0)) {
    printf("SET FEATURE failed\n");
    goto cleanup;
}

if (wait_ready(spi))
    goto cleanup;

printf("\nVERIFY B0\n");

if (read_feature(spi, 0xB0, "Configuration (B0)", &status))
    goto cleanup;

dump_result("B0 readback", status);

if (status == original_b0)
    printf("B0 WRITE/READBACK VERIFIED\n");
else
    printf("B0 WRITE/READBACK MISMATCH\n");

printf("\nREAD ID AFTER FEATURE WRITE\n");
read_id(spi);

printf("\nRESET\n");

if (command(spi, 0xFF)) {
    printf("RESET FAILED\n");
    goto cleanup;
}

Sleep(2);

printf("RESET OK\n\n");

printf("FINAL FEATURES\n");

read_feature(spi, 0xA0, "Protection (A0)", &status);
read_feature(spi, 0xB0, "Configuration (B0)", &status);
read_feature(spi, 0xC0, "Status (C0)", &status);

printf("\nFINAL READ ID\n");
read_id(spi);

cleanup:
ufprog_spi_close_device(spi);

printf("\n=== TEST COMPLETE ===\n");

return 0;

}
