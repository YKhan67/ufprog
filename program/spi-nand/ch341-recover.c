#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <ufprog/osdef.h>
#include <ufprog/log.h>
#include <ufprog/spi.h>

#define PAGE_READ       0x13
#define READ_FROM_CACHE 0x03
#define RESET           0xFF
#define GET_FEATURE     0x0F

#define REG_STATUS      0xC0

#define STATUS_OIP      0x01
#define STATUS_ECC_MASK 0x30

static ufprog_status xfer(struct ufprog_spi *spi,
			  struct ufprog_spi_transfer *xfer,
			  size_t count)
{
	return ufprog_spi_generic_xfer(spi, xfer, count);
}

static ufprog_status send_cmd(struct ufprog_spi *spi,
			      uint8_t cmd)
{
	struct ufprog_spi_transfer t = {0};

	t.dir = SPI_DATA_OUT;
	t.buf.tx = &cmd;
	t.len = 1;
	t.end = true;

	return xfer(spi, &t, 1);
}

static ufprog_status read_feature(struct ufprog_spi *spi,
				  uint8_t addr,
				  uint8_t *value)
{
	uint8_t tx[2] = {
		GET_FEATURE,
		addr
	};

	struct ufprog_spi_transfer t[2] = {0};

	t[0].dir = SPI_DATA_OUT;
	t[0].buf.tx = tx;
	t[0].len = sizeof(tx);
	t[0].end = false;

	t[1].dir = SPI_DATA_IN;
	t[1].buf.rx = value;
	t[1].len = 1;
	t[1].end = true;

	return xfer(spi, t, 2);
}

static ufprog_status wait_ready(struct ufprog_spi *spi,
				uint8_t *final_status)
{
	uint8_t status = 0;

	for (unsigned i = 0; i < 1000; i++) {
		ufprog_status ret;

		ret = read_feature(spi, REG_STATUS, &status);
		if (ret)
			return ret;

		if (!(status & STATUS_OIP)) {
			if (final_status)
				*final_status = status;

			return UFP_OK;
		}

		Sleep(1);
	}

	if (final_status)
		*final_status = status;

	return UFP_TIMEOUT;
}

static ufprog_status page_read(struct ufprog_spi *spi,
			       uint32_t page,
			       uint8_t *status)
{
	uint8_t tx[4];

	/*
	 * SPI-NAND PAGE READ:
	 *
	 *     13h
	 *     row address [23:0]
	 */
	tx[0] = PAGE_READ;
	tx[1] = (uint8_t)(page >> 16);
	tx[2] = (uint8_t)(page >> 8);
	tx[3] = (uint8_t)page;

	struct ufprog_spi_transfer t = {0};

	t.dir = SPI_DATA_OUT;
	t.buf.tx = tx;
	t.len = sizeof(tx);
	t.end = true;

	ufprog_status ret = xfer(spi, &t, 1);

	if (ret)
		return ret;

	return wait_ready(spi, status);
}

static ufprog_status read_cache(struct ufprog_spi *spi,
				uint16_t column,
				uint8_t *data,
				size_t len)
{
	/*
	 * READ FROM CACHE:
	 *
	 *     03h
	 *     column address [15:0]
	 *     data
	 *
	 * Keep command/address/read under one CS assertion.
	 */
	uint8_t tx[3];

	tx[0] = READ_FROM_CACHE;
	tx[1] = (uint8_t)(column >> 8);
	tx[2] = (uint8_t)column;

	struct ufprog_spi_transfer t[2] = {0};

	t[0].dir = SPI_DATA_OUT;
	t[0].buf.tx = tx;
	t[0].len = sizeof(tx);
	t[0].end = false;

	t[1].dir = SPI_DATA_IN;
	t[1].buf.rx = data;
	t[1].len = len;
	t[1].end = true;

	return xfer(spi, t, 2);
}

static void dump_data(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if ((i % 16) == 0)
			printf("%04X: ", (unsigned)i);

		printf("%02X ", data[i]);

		if ((i % 16) == 15)
			printf("\n");
	}

	if (len % 16)
		printf("\n");
}

static void analyse_data(const uint8_t *data, size_t len)
{
	size_t zero = 0;
	size_t ff = 0;

	for (size_t i = 0; i < len; i++) {
		if (data[i] == 0x00)
			zero++;

		if (data[i] == 0xFF)
			ff++;
	}

	printf("Statistics: %zu/%zu = 0x00, %zu/%zu = 0xFF\n",
	       zero, len, ff, len);

	if (zero == len)
		printf("RESULT: ALL ZEROES\n");
	else if (ff == len)
		printf("RESULT: ALL FF\n");
	else
		printf("RESULT: NON-UNIFORM DATA\n");
}

static void test_page(struct ufprog_spi *spi, uint32_t page)
{
	uint8_t data[64];
	uint8_t status = 0;
	ufprog_status ret;

	memset(data, 0, sizeof(data));

	printf("\n========================================\n");
	printf("PAGE %u\n", page);
	printf("========================================\n");

	printf("PAGE READ 13h...\n");

	ret = page_read(spi, page, &status);

	if (ret) {
		printf("PAGE READ FAILED: %u\n", ret);
		return;
	}

	printf("PAGE READ COMPLETE\n");
	printf("STATUS: %02X\n", status);

	printf("OIP: %s\n",
	       (status & STATUS_OIP) ? "BUSY" : "READY");

	printf("ECC bits: %02X\n",
	       status & STATUS_ECC_MASK);

	printf("\nREAD CACHE 03h...\n");

	ret = read_cache(spi, 0x0000, data, sizeof(data));

	if (ret) {
		printf("READ CACHE FAILED: %u\n", ret);
		return;
	}

	printf("FIRST 64 BYTES:\n");
	dump_data(data, sizeof(data));

	analyse_data(data, sizeof(data));
}

int wmain(void)
{
	struct ufprog_spi *spi = NULL;
	ufprog_status ret;
	uint8_t value;

	set_os_default_log_print();
	os_init();

	printf("=== ESMT F50L1G41LB NAND ARRAY READ TEST ===\n");
	printf("READ ONLY - NO PROGRAM / NO ERASE / NO WRITE ENABLE\n\n");

	ret = ufprog_spi_open_device("ch341-libusb", false, &spi);

	if (ret) {
		fprintf(stderr, "open failed: %u\n", ret);
		return 1;
	}

	ret = ufprog_spi_set_cs_pol(spi, false);

	if (ret)
		printf("CS polarity setup failed: %u\n", ret);

	printf("RESET\n");

	ret = send_cmd(spi, RESET);

	if (ret) {
		printf("RESET FAILED: %u\n", ret);
		return 1;
	}

	ret = wait_ready(spi, &value);

	if (ret) {
		printf("RESET WAIT FAILED: %u\n", ret);
		return 1;
	}

	printf("RESET OK\n");

	printf("\nINITIAL STATUS\n");

	ret = read_feature(spi, 0xA0, &value);
	if (!ret)
		printf("A0 Protection:    %02X\n", value);

	ret = read_feature(spi, 0xB0, &value);
	if (!ret)
		printf("B0 Configuration: %02X\n", value);

	ret = read_feature(spi, 0xC0, &value);
	if (!ret)
		printf("C0 Status:        %02X\n", value);

	/*
	 * Read several pages.
	 *
	 * These are PAGE READ operations only.
	 * They do not modify the NAND array.
	 */
	test_page(spi, 0);
	test_page(spi, 1);
	test_page(spi, 2);
	test_page(spi, 10);

	printf("\nFINAL STATUS\n");

	ret = read_feature(spi, 0xC0, &value);

	if (!ret)
		printf("C0 Status: %02X\n", value);

	printf("\n=== TEST COMPLETE ===\n");

	return 0;
}