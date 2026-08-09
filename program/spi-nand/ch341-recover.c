#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ufprog/spi.h>

static ufprog_status xfer(struct ufprog_spi *spi,
                          const uint8_t *tx, uint8_t *rx, size_t len)
{
    struct ufprog_spi_transfer x = {0};

    x.buswidth = 1;
    x.dtr = false;
    x.len = len;

    if (tx) {
        x.dir = SPI_DATA_OUT;
        x.buf.tx = tx;
    } else {
        x.dir = SPI_DATA_IN;
        x.buf.rx = rx;
    }

    x.end = true;

    return ufprog_spi_generic_xfer(spi, &x, 1);
}

static ufprog_status cmd(struct ufprog_spi *spi, uint8_t opcode)
{
    return xfer(spi, &opcode, NULL, 1);
}

static ufprog_status read_status(struct ufprog_spi *spi, uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = { 0x0F, reg };
    uint8_t rx[2] = { 0 };

    struct ufprog_spi_transfer x[2] = {0};

    x[0].buswidth = 1;
    x[0].dtr = false;
    x[0].dir = SPI_DATA_OUT;
    x[0].buf.tx = tx;
    x[0].len = sizeof(tx);
    x[0].end = false;

    x[1].buswidth = 1;
    x[1].dtr = false;
    x[1].dir = SPI_DATA_IN;
    x[1].buf.rx = rx;
    x[1].len = 1;
    x[1].end = true;

    ufprog_status ret = ufprog_spi_generic_xfer(spi, x, 2);

    if (!ret)
        *val = rx[0];

    return ret;
}

int wmain(void)
{
    struct ufprog_spi *spi = NULL;
    ufprog_status ret;
    uint8_t sr = 0;
    uint8_t id[4] = {0};
    uint8_t tx[2] = {0x9F, 0};

    ret = ufprog_spi_open_device("ch341-libusb", false, &spi);
    if (ret) {
        fprintf(stderr, "open failed: %d\n", ret);
        return 1;
    }

    ufprog_spi_set_speed(spi, 500000, NULL);
    ufprog_spi_set_cs_pol(spi, false);

    printf("RESET...\n");
    cmd(spi, 0xFF);
    cmd(spi, 0xFF);

    printf("READ ID...\n");

    struct ufprog_spi_transfer x[2] = {0};

    x[0].buswidth = 1;
    x[0].dtr = false;
    x[0].dir = SPI_DATA_OUT;
    x[0].buf.tx = tx;
    x[0].len = 1;
    x[0].end = false;

    x[1].buswidth = 1;
    x[1].dtr = false;
    x[1].dir = SPI_DATA_IN;
    x[1].buf.rx = id;
    x[1].len = 4;
    x[1].end = true;

    ret = ufprog_spi_generic_xfer(spi, x, 2);

    if (!ret)
        printf("ID: %02X %02X %02X %02X\n",
               id[0], id[1], id[2], id[3]);

    printf("STATUS 0xC0...\n");
    if (!read_status(spi, 0xC0, &sr))
        printf("SR-C0: %02X\n", sr);

    printf("RESET...\n");
    cmd(spi, 0xFF);

    printf("WRITE ENABLE...\n");
    cmd(spi, 0x06);

    printf("RESET...\n");
    cmd(spi, 0xFF);

    printf("STATUS 0xC0...\n");
    if (!read_status(spi, 0xC0, &sr))
        printf("SR-C0: %02X\n", sr);

    ufprog_spi_close_device(spi);

    return ret ? 1 : 0;
}
