#include <stdio.h>
#include <stdint.h>

#include <ufprog/osdef.h>
#include <ufprog/log.h>
#include <ufprog/spi.h>

static int cmd(struct ufprog_spi *spi, uint8_t opcode)
{
    struct ufprog_spi_transfer xfer = {0};

    xfer.tx = &opcode;
    xfer.len = 1;

    return ufprog_spi_transfer(spi, &xfer, 1);
}

static int read_feature(struct ufprog_spi *spi, uint8_t addr)
{
    uint8_t tx[2] = {0x0F, addr};
    uint8_t rx = 0;
    struct ufprog_spi_transfer xfer[2] = {0};

    xfer[0].tx = tx;
    xfer[0].len = 2;

    xfer[1].rx = &rx;
    xfer[1].len = 1;

    if (ufprog_spi_transfer(spi, xfer, 2)) {
        printf("READ FEATURE failed\n");
        return -1;
    }

    printf("FEATURE %02X = %02X\n", addr, rx);

    return 0;
}

static int read_id(struct ufprog_spi *spi)
{
    uint8_t tx = 0x9F;
    uint8_t rx[8] = {0};
    struct ufprog_spi_transfer xfer[2] = {0};

    xfer[0].tx = &tx;
    xfer[0].len = 1;

    xfer[1].rx = rx;
    xfer[1].len = sizeof(rx);

    if (ufprog_spi_transfer(spi, xfer, 2)) {
        printf("READ ID failed\n");
        return -1;
    }

    printf("ID:");
    for (unsigned i = 0; i < sizeof(rx); i++)
        printf(" %02X", rx[i]);
    printf("\n");

    return 0;
}

int main(void)
{
    struct ufprog_spi *spi = NULL;
    ufprog_status ret;

    set_os_default_log_print();
    os_init();

    ret = ufprog_spi_open_device("ch341-libusb", false, &spi);
    if (ret) {
        fprintf(stderr, "open failed: %u\n", ret);
        return 1;
    }

    printf("=== SPI-NAND diagnostic ===\n");

    ret = ufprog_spi_set_cs_pol(spi, false);
    if (ret)
        printf("set CS polarity failed: %u\n", ret);

    printf("\nRESET 1\n");
    cmd(spi, 0xFF);

    printf("RESET 2\n");
    cmd(spi, 0xFF);

    printf("\nSTATUS C0\n");
    read_feature(spi, 0xC0);

    printf("\nREAD ID\n");
    read_id(spi);

    printf("\nSTATUS C0 again\n");
    read_feature(spi, 0xC0);

    printf("\nRESET again\n");
    cmd(spi, 0xFF);

    printf("\nSTATUS C0 after reset\n");
    read_feature(spi, 0xC0);

    printf("\nREAD ID again\n");
    read_id(spi);

    ufprog_spi_free(spi);
    os_cleanup();

    return 0;
}