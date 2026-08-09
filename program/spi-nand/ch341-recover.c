/* SPDX-License-Identifier: LGPL-2.1-only */

#include <stdio.h>
#include <stdint.h>

#include <ufprog/osdef.h>
#include <ufprog/log.h>
#include <ufprog/spi.h>

static ufprog_status spi_cmd(struct ufprog_spi *spi, uint8_t opcode)
{
	struct ufprog_spi_transfer xfer = {0};

	xfer.dir = SPI_DATA_OUT;
	xfer.buf.tx = &opcode;
	xfer.len = 1;
	xfer.end = true;

	return ufprog_spi_generic_xfer(spi, &xfer, 1);
}

static ufprog_status read_bytes(struct ufprog_spi *spi, const uint8_t *tx, size_t txlen,
				uint8_t *rx, size_t rxlen)
{
	struct ufprog_spi_transfer xfer[2] = {0};

	xfer[0].dir = SPI_DATA_OUT;
	xfer[0].buf.tx = tx;
	xfer[0].len = txlen;
	xfer[0].end = false;

	xfer[1].dir = SPI_DATA_IN;
	xfer[1].buf.rx = rx;
	xfer[1].len = rxlen;
	xfer[1].end = true;

	return ufprog_spi_generic_xfer(spi, xfer, 2);
}

static ufprog_status read_feature(struct ufprog_spi *spi, uint8_t addr, uint8_t *value)
{
	uint8_t tx[2] = {0x0F, addr};
	uint8_t rx = 0;
	ufprog_status ret;

	ret = read_bytes(spi, tx, sizeof(tx), &rx, 1);
	if (ret)
		return ret;

	*value = rx;
	return UFP_OK;
}

static ufprog_status read_id(struct ufprog_spi *spi)
{
	uint8_t tx = 0x9F;
	uint8_t rx[8] = {0};
	ufprog_status ret;

	ret = read_bytes(spi, &tx, 1, rx, sizeof(rx));
	if (ret)
		return ret;

	printf("ID:");
	for (unsigned i = 0; i < sizeof(rx); i++)
		printf(" %02X", rx[i]);
	printf("\n");

	return UFP_OK;
}

static void dump_feature(struct ufprog_spi *spi, uint8_t addr, const char *name)
{
	uint8_t value = 0;
	ufprog_status ret;

	ret = read_feature(spi, addr, &value);

	if (ret)
		printf("%s (%02X): ERROR %u\n", name, addr, ret);
	else
		printf("%s (%02X): %02X\n", name, addr, value);
}

int wmain(void)
{
	struct ufprog_spi *spi = NULL;
	ufprog_status ret;

	set_os_default_log_print();
	os_init();

	printf("=== ESMT F50L1G41LB READ-ONLY DIAGNOSTIC ===\n\n");

	ret = ufprog_spi_open_device("ch341-libusb", false, &spi);
	if (ret) {
		fprintf(stderr, "open failed: %u\n", ret);
		return 1;
	}

	ret = ufprog_spi_set_cs_pol(spi, false);
	if (ret)
		printf("CS polarity setup failed: %u\n", ret);

	printf("RESET\n");
	ret = spi_cmd(spi, 0xFF);
	printf("RESET: %s\n", ret ? "FAILED" : "OK");

	printf("\nREAD ID\n");
	ret = read_id(spi);
	if (ret)
		printf("READ ID failed: %u\n", ret);

	printf("\nREAD FEATURES\n");
	dump_feature(spi, 0xA0, "Protection");
	dump_feature(spi, 0xB0, "Configuration");
	dump_feature(spi, 0xC0, "Status");

	printf("\nRESET AGAIN\n");
	ret = spi_cmd(spi, 0xFF);
	printf("RESET: %s\n", ret ? "FAILED" : "OK");

	printf("\nREAD FEATURES AFTER RESET\n");
	dump_feature(spi, 0xA0, "Protection");
	dump_feature(spi, 0xB0, "Configuration");
	dump_feature(spi, 0xC0, "Status");

	printf("\nREAD ID AFTER RESET\n");
	ret = read_id(spi);
	if (ret)
		printf("READ ID failed: %u\n", ret);

	return 0;
}