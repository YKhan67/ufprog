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

static ufprog_status write_bytes(struct ufprog_spi *spi,
				 const uint8_t *buf, size_t len)
{
	struct ufprog_spi_transfer xfer = {0};

	xfer.dir = SPI_DATA_OUT;
	xfer.buf.tx = buf;
	xfer.len = len;
	xfer.end = true;

	return ufprog_spi_generic_xfer(spi, &xfer, 1);
}

static ufprog_status read_bytes(struct ufprog_spi *spi,
				const uint8_t *tx, size_t txlen,
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

static ufprog_status read_feature(struct ufprog_spi *spi,
				  uint8_t addr, uint8_t *value)
{
	uint8_t tx[2] = {0x0F, addr};

	return read_bytes(spi, tx, sizeof(tx), value, 1);
}

static ufprog_status set_feature(struct ufprog_spi *spi,
				 uint8_t addr, uint8_t value)
{
	uint8_t tx[3] = {0x1F, addr, value};

	return write_bytes(spi, tx, sizeof(tx));
}

static ufprog_status read_id(struct ufprog_spi *spi)
{
	uint8_t tx[2] = {0x9F, 0x00};
	uint8_t rx[5] = {0};
	ufprog_status ret;

	ret = read_bytes(spi, tx, sizeof(tx), rx, sizeof(rx));
	if (ret)
		return ret;

	printf("ID:");
	for (unsigned i = 0; i < sizeof(rx); i++)
		printf(" %02X", rx[i]);
	printf("\n");

	return UFP_OK;
}

static int get_feature(struct ufprog_spi *spi, uint8_t addr,
		       const char *name, uint8_t *value)
{
	ufprog_status ret;

	ret = read_feature(spi, addr, value);

	if (ret) {
		printf("%s (%02X): READ FAILED %u\n",
		       name, addr, ret);
		return -1;
	}

	printf("%s (%02X): %02X\n", name, addr, *value);

	return 0;
}

static int wait_ready(struct ufprog_spi *spi)
{
	uint8_t sr;
	ufprog_status ret;

	for (unsigned i = 0; i < 100; i++) {
		ret = read_feature(spi, 0xC0, &sr);
		if (ret)
			return -1;

		if (!(sr & 0x01))
			return 0;
	}

	printf("Timeout waiting for OIP=0\n");
	return -1;
}

int wmain(void)
{
	struct ufprog_spi *spi = NULL;
	ufprog_status ret;
	uint8_t original_b0;
	uint8_t value;
	uint8_t sr;

	set_os_default_log_print();
	os_init();

	printf("=== ESMT F50L1G41LB FEATURE WRITE TEST ===\n");
	printf("READ-ONLY ARRAY: NO PROGRAM / NO ERASE\n\n");

	ret = ufprog_spi_open_device("ch341-libusb", false, &spi);
	if (ret) {
		fprintf(stderr, "open failed: %u\n", ret);
		return 1;
	}

	ret = ufprog_spi_set_cs_pol(spi, false);
	if (ret)
		printf("CS polarity setup failed: %u\n", ret);

	/*
	 * Start from a known interface state.
	 */
	printf("RESET\n");

	ret = spi_cmd(spi, 0xFF);
	if (ret) {
		printf("RESET FAILED: %u\n", ret);
		return 1;
	}

	if (wait_ready(spi)) {
		printf("Device did not become ready\n");
		return 1;
	}

	printf("RESET OK\n\n");

	/*
	 * Read identification.
	 */
	printf("READ ID\n");

	ret = read_id(spi);
	if (ret)
		printf("READ ID FAILED: %u\n", ret);

	printf("\nINITIAL FEATURES\n");

	get_feature(spi, 0xA0, "Protection", &value);
	get_feature(spi, 0xB0, "Configuration", &original_b0);
	get_feature(spi, 0xC0, "Status", &sr);

	printf("\nOriginal B0 = %02X\n", original_b0);

	/*
	 * We expect B0 bit 4 (ECC-E) to be writable.
	 *
	 * Current value should normally be 0x10.
	 * Clear only bit 4.
	 */
	value = original_b0 & ~(uint8_t)0x10;

	printf("\nTEST VALUE\n");
	printf("B0: %02X -> %02X\n", original_b0, value);

	/*
	 * WRITE ENABLE
	 */
	printf("\nWRITE ENABLE\n");

	ret = spi_cmd(spi, 0x06);
	if (ret) {
		printf("WRITE ENABLE FAILED: %u\n", ret);
		return 1;
	}

	ret = read_feature(spi, 0xC0, &sr);
	if (ret) {
		printf("STATUS READ FAILED: %u\n", ret);
		return 1;
	}

	printf("STATUS: %02X\n", sr);

	if (!(sr & 0x02)) {
		printf("WEL did NOT become 1. Aborting.\n");
		return 1;
	}

	printf("WEL=1\n");

	/*
	 * SET FEATURE B0.
	 */
	printf("\nSET FEATURE B0 = %02X\n", value);

	ret = set_feature(spi, 0xB0, value);
	if (ret) {
		printf("SET FEATURE FAILED: %u\n", ret);
		return 1;
	}

	if (wait_ready(spi)) {
		printf("Device did not become ready after SET FEATURE\n");
		return 1;
	}

	/*
	 * Verify.
	 */
	printf("\nVERIFY TEST VALUE\n");

	ret = read_feature(spi, 0xB0, &sr);
	if (ret) {
		printf("B0 READ FAILED: %u\n", ret);
		return 1;
	}

	printf("B0 = %02X\n", sr);

	if (sr == value)
		printf("SUCCESS: B0 WRITE VERIFIED\n");
	else
		printf("FAIL: B0 WRITE NOT VERIFIED\n");

	/*
	 * Restore original value.
	 */
	printf("\nRESTORE ORIGINAL B0 = %02X\n", original_b0);

	ret = spi_cmd(spi, 0x06);
	if (ret) {
		printf("RESTORE WRITE ENABLE FAILED: %u\n", ret);
		return 1;
	}

	ret = read_feature(spi, 0xC0, &sr);
	if (ret) {
		printf("RESTORE STATUS READ FAILED: %u\n", ret);
		return 1;
	}

	printf("STATUS: %02X\n", sr);

	if (!(sr & 0x02)) {
		printf("RESTORE WEL DID NOT SET. Aborting restore.\n");
		return 1;
	}

	ret = set_feature(spi, 0xB0, original_b0);
	if (ret) {
		printf("RESTORE SET FEATURE FAILED: %u\n", ret);
		return 1;
	}

	if (wait_ready(spi)) {
		printf("Device did not become ready after restore\n");
		return 1;
	}

	/*
	 * Verify restoration.
	 */
	printf("\nVERIFY RESTORE\n");

	ret = read_feature(spi, 0xB0, &value);
	if (ret) {
		printf("RESTORE VERIFY FAILED: %u\n", ret);
		return 1;
	}

	printf("B0 = %02X\n", value);

	if (value == original_b0)
		printf("RESTORE SUCCESS\n");
	else
		printf("RESTORE FAILED: expected %02X, got %02X\n",
		       original_b0, value);

	/*
	 * Final status.
	 */
	printf("\nFINAL STATUS\n");

	get_feature(spi, 0xC0, "Status", &value);

	printf("\nFINAL RESET\n");

	ret = spi_cmd(spi, 0xFF);
	if (ret)
		printf("RESET FAILED: %u\n", ret);
	else
		printf("RESET OK\n");

	printf("\nFINAL B0\n");
	get_feature(spi, 0xB0, "Configuration", &value);

	printf("\n=== TEST COMPLETE ===\n");

	return 0;
}