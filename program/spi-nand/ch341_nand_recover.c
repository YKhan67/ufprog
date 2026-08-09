#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ufprog/api_spi.h>
#include <ufprog/log.h>

static void dump(const char *name, const uint8_t *buf, size_t len)
{
    size_t i;

    printf("%s:", name);

    for (i = 0; i < len; i++)
        printf(" %02x", buf[i]);

    printf("\n");
}

static ufprog_status raw_xfer(struct ufprog_spi *spi,
                              const uint8_t *tx, size_t txlen,
                              uint8_t *rx, size_t rxlen)
{
    struct ufprog_spi_transfer xfers[2];

    memset(xfers, 0, sizeof(xfers));

    xfers[0].dir = SPI_DATA_OUT;
    xfers[0].buswidth = 1;
    xfers[0].dtr = 0;
    xfers[0].buf.tx = tx;
    xfers[0].len = txlen;
    xfers[0].end = false;

    xfers[1].dir = SPI_DATA_IN;
    xfers[1].buswidth = 1;
    xfers[1].dtr = 0;
    xfers[1].buf.rx = rx;
    xfers[1].len = rxlen;
    xfers[1].end = true;

    return ufprog_spi_generic_xfer(spi, xfers, 2);
}

int main(int argc, char **argv)
{
    struct ufprog_spi *spi = NULL;
    ufprog_status ret;
    uint8_t tx[4];
    uint8_t rx[16];

    const char *dev = "ch341";

    if (argc > 1)
        dev = argv[1];

    printf("CH341 SPI-NAND recovery test\n");
    printf("Device: %s\n\n", dev);

    ret = ufprog_spi_open_device(dev, false, &spi);
    if (ret) {
        printf("Failed to open SPI device: %d\n", ret);
        return 1;
    }

    /*
     * SPI-NAND RESET
     * FF
     */
    tx[0] = 0xFF;

    ret = ufprog_spi_set_cs_pol(spi, false);
    if (ret)
        goto out;

    ret = ufprog_spi_generic_xfer(spi,
        &(struct ufprog_spi_transfer) {
            .buf.tx = tx,
            .buswidth = 1,
            .dtr = 0,
            .dir = SPI_DATA_OUT,
            .len = 1,
            .end = true
        }, 1);

    printf("RESET FF: %s\n", ret ? "FAILED" : "OK");

    if (ret)
        goto out;

    /*
     * Give the NAND time to complete reset.
     */
    os_udelay(5000);

    /*
     * READ STATUS
     *
     * 0F = Get Feature
     * C0 = Status register
     */
    tx[0] = 0x0F;
    tx[1] = 0xC0;

    memset(rx, 0, sizeof(rx));

    ret = raw_xfer(spi, tx, 2, rx, 1);

    if (ret)
        printf("GET FEATURE C0: FAILED (%d)\n", ret);
    else
        dump("STATUS C0", rx, 1);

    /*
     * READ STATUS AGAIN
     */
    os_udelay(1000);

    memset(rx, 0, sizeof(rx));

    ret = raw_xfer(spi, tx, 2, rx, 1);

    if (ret)
        printf("GET FEATURE C0 #2: FAILED (%d)\n", ret);
    else
        dump("STATUS C0 #2", rx, 1);

    /*
     * RESET AGAIN
     */
    tx[0] = 0xFF;

    ret = ufprog_spi_generic_xfer(spi,
        &(struct ufprog_spi_transfer) {
            .buf.tx = tx,
            .buswidth = 1,
            .dtr = 0,
            .dir = SPI_DATA_OUT,
            .len = 1,
            .end = true
        }, 1);

    printf("RESET FF #2: %s\n", ret ? "FAILED" : "OK");

    os_udelay(5000);

    /*
     * JEDEC READ ID
     *
     * 9F followed by dummy clocks.
     */
    tx[0] = 0x9F;

    memset(rx, 0, sizeof(rx));

    ret = raw_xfer(spi, tx, 1, rx, 8);

    if (ret)
        printf("READ ID 9F: FAILED (%d)\n", ret);
    else
        dump("RAW 9F", rx, 8);

out:
    if (spi)
        ufprog_spi_close_device(spi);

    return ret ? 1 : 0;
}