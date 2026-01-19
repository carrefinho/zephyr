/*
 * Copyright (c) 2025 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/util.h"
#define DT_DRV_COMPAT solomon_ssd1362

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ssd1362, CONFIG_DISPLAY_LOG_LEVEL);

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/kernel.h>

#define SSD1362_SET_COLUMN_ADDR      0x15
#define SSD1362_SET_FADE_MODE        0x23
#define SSD1362_SET_ROW_ADDR         0x75
#define SSD1362_SET_CONTRAST         0x81
#define SSD1362_SET_REMAP            0xA0
#define SSD1362_SET_START_LINE       0xA1
#define SSD1362_SET_DISPLAY_OFFSET   0xA2
#define SSD1362_BLANKING_ON          0xA4
#define SSD1362_BLANKING_OFF         0xA6
#define SSD1362_BLANKING_OFF_INVERSE 0xA7
#define SSD1362_EXIT_PARTIAL         0xA9
#define SSD1362_SET_VDD              0xAB
#define SSD1362_SET_IREF             0xAD
#define SSD1362_DISPLAY_OFF          0xAE
#define SSD1362_DISPLAY_ON           0xAF
#define SSD1362_SET_PHASE_LENGTH     0xB1
#define SSD1362_SET_CLOCK_DIV        0xB3
#define SSD1362_SET_SECOND_PRECHARGE 0xB6
#define SSD1362_DEFAULT_GRAYSCALE    0xB9
#define SSD1362_SET_PRECHARGE        0xBC
#define SSD1362_SET_PRECHARGE_CAP    0xBD
#define SSD1362_SET_VCOMH            0xBE
#define SSD1362_SET_MUX_RATIO        0xCA
#define SSD1362_COMMAND_LOCK         0xFD

#define SSD1362_COMMAND_LOCK_UNLOCK 0x12

/* 2 pixels per byte */
#define SSD1362_2PPB 2

struct ssd1362_config {
	const struct device *mipi_dev;
	struct mipi_dbi_config dbi_config;
	uint16_t height;
	uint16_t width;
	uint16_t column_offset;
	uint8_t row_offset;
	uint8_t start_line;
	uint8_t mux_ratio;
	bool remap_row_first;
	bool remap_columns;
	bool remap_rows;
	bool remap_nibble;
	bool remap_com_odd_even_split;
	bool remap_com_dual;
	bool color_inversion;
	bool iref_external;
	uint8_t oscillator_freq;
	uint8_t precharge_voltage;
	uint8_t vcomh_voltage;
	uint8_t phase_length;
	uint8_t second_precharge_period;
	uint8_t *conversion_buf;
	size_t conversion_buf_size;
};

struct ssd1362_data {
	uint8_t contrast;
};

static inline int ssd1362_write_command(const struct device *dev, uint8_t cmd, const uint8_t *buf,
					size_t len)
{
	const struct ssd1362_config *config = dev->config;
	int ret;

	/* SSD1362 requires data bytes to be sent as separate commands (D/C# low) */
	ret = mipi_dbi_command_write(config->mipi_dev, &config->dbi_config, cmd, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	for (size_t i = 0; i < len; i++) {
		ret = mipi_dbi_command_write(config->mipi_dev, &config->dbi_config, buf[i], NULL,
					     0);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int ssd1362_blanking_on(const struct device *dev)
{
	return ssd1362_write_command(dev, SSD1362_BLANKING_ON, NULL, 0);
}

static int ssd1362_blanking_off(const struct device *dev)
{
	const struct ssd1362_config *config = dev->config;

	return ssd1362_write_command(
		dev, config->color_inversion ? SSD1362_BLANKING_OFF_INVERSE : SSD1362_BLANKING_OFF,
		NULL, 0);
}

/* Convert L8 (8-bit grayscale) to 4-bit packed format: pixel0 (7:4) | pixel1 (3:0) */
static int ssd1362_convert_L_8(const struct device *dev, const uint8_t *buf, int cur_offset,
			       uint32_t pixel_count)
{
	const struct ssd1362_config *config = dev->config;
	int i = 0;

	for (; i / 2 < config->conversion_buf_size && pixel_count > cur_offset + i; i += 2) {
		config->conversion_buf[i / 2] = (buf[cur_offset + i] >> 4) << 4;
		config->conversion_buf[i / 2] |= buf[cur_offset + i + 1] >> 4;
	}

	return i;
}

static int ssd1362_write_pixels(const struct device *dev, const uint8_t *buf, uint32_t pixel_count,
				const struct display_buffer_descriptor *desc)
{
	const struct ssd1362_config *config = dev->config;
	struct display_buffer_descriptor mipi_desc;
	int ret, i;
	int total = 0;

	mipi_desc.pitch = desc->pitch;

	while (pixel_count > total) {
		i = ssd1362_convert_L_8(dev, buf, total, pixel_count);

		mipi_desc.buf_size = i / SSD1362_2PPB;
		mipi_desc.width = mipi_desc.buf_size / desc->height;
		mipi_desc.height = mipi_desc.buf_size / desc->width;

		ret = mipi_dbi_write_display(config->mipi_dev, &config->dbi_config,
					     config->conversion_buf, &mipi_desc, PIXEL_FORMAT_L_8);
		if (ret < 0) {
			return ret;
		}
		total += i;
	}
	mipi_dbi_release(config->mipi_dev, &config->dbi_config);
	return 0;
}

static int ssd1362_write(const struct device *dev, const uint16_t x, const uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct ssd1362_config *config = dev->config;
	size_t buf_len;
	int ret;
	uint8_t cmd_data[2];

	if (desc->pitch != desc->width) {
		LOG_ERR("Pitch is different from width");
		return -EINVAL;
	}

	buf_len = MIN(desc->buf_size, desc->height * desc->width / SSD1362_2PPB);

	if (buf == NULL || buf_len == 0U) {
		LOG_ERR("Display buffer is not available");
		return -EINVAL;
	}

	if ((x & 1) != 0U) {
		LOG_ERR("Unsupported origin (x must be even)");
		return -EINVAL;
	}

	LOG_DBG("x %u, y %u, pitch %u, width %u, height %u, buf_len %u", x, y, desc->pitch,
		desc->width, desc->height, buf_len);

	/* SSD1362 addresses 2 pixels per column address */
	cmd_data[0] = config->column_offset + (x >> 1);
	cmd_data[1] = config->column_offset + ((x + desc->width) >> 1) - 1;
	ret = ssd1362_write_command(dev, SSD1362_SET_COLUMN_ADDR, cmd_data, 2);
	if (ret < 0) {
		return ret;
	}

	cmd_data[0] = config->row_offset + y;
	cmd_data[1] = config->row_offset + y + desc->height - 1;
	ret = ssd1362_write_command(dev, SSD1362_SET_ROW_ADDR, cmd_data, 2);
	if (ret < 0) {
		return ret;
	}

	return ssd1362_write_pixels(dev, buf, desc->width * desc->height, desc);
}

static int ssd1362_set_contrast(const struct device *dev, const uint8_t contrast)
{
	return ssd1362_write_command(dev, SSD1362_SET_CONTRAST, &contrast, 1);
}

static void ssd1362_get_capabilities(const struct device *dev, struct display_capabilities *caps)
{
	const struct ssd1362_config *config = dev->config;

	memset(caps, 0, sizeof(struct display_capabilities));
	caps->x_resolution = config->width;
	caps->y_resolution = config->height;
	caps->supported_pixel_formats = PIXEL_FORMAT_L_8;
	caps->current_pixel_format = PIXEL_FORMAT_L_8;
	caps->screen_info = 0;
}

static int ssd1362_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format pixel_format)
{
	if (pixel_format == PIXEL_FORMAT_L_8) {
		return 0;
	}
	LOG_ERR("Unsupported pixel format");
	return -ENOTSUP;
}

static int ssd1362_init_device(const struct device *dev)
{
	int ret;
	uint8_t data[2];
	const struct ssd1362_config *config = dev->config;

	ret = mipi_dbi_reset(config->mipi_dev, 1);
	if (ret < 0) {
		return ret;
	}
	k_usleep(100);

	/* Unlock display */
	data[0] = SSD1362_COMMAND_LOCK_UNLOCK;
	ret = ssd1362_write_command(dev, SSD1362_COMMAND_LOCK, data, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_DISPLAY_OFF, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	/* Disable fade mode */
	data[0] = 0x00;
	ret = ssd1362_write_command(dev, SSD1362_SET_FADE_MODE, data, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_SET_CLOCK_DIV, &config->oscillator_freq, 1);
	if (ret < 0) {
		return ret;
	}

	data[0] = config->mux_ratio - 1;
	ret = ssd1362_write_command(dev, SSD1362_SET_MUX_RATIO, data, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_SET_START_LINE, &config->start_line, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_SET_DISPLAY_OFFSET, &config->row_offset, 1);
	if (ret < 0) {
		return ret;
	}

	/* Build remap register value */
	data[0] = 0x00;
	WRITE_BIT(data[0], 0, config->remap_columns);
	WRITE_BIT(data[0], 1, config->remap_nibble);
	WRITE_BIT(data[0], 2, config->remap_row_first);
	WRITE_BIT(data[0], 4, config->remap_rows);
	WRITE_BIT(data[0], 6, !config->remap_com_odd_even_split);
	WRITE_BIT(data[0], 7, config->remap_com_dual);
	ret = ssd1362_write_command(dev, SSD1362_SET_REMAP, data, 1);
	if (ret < 0) {
		return ret;
	}

	/* Set VDD regulator */
	data[0] = 0x01;
	ret = ssd1362_write_command(dev, SSD1362_SET_VDD, data, 1);
	if (ret < 0) {
		return ret;
	}

	/* Set IREF: 0x8E = external, 0x9E = internal */
	data[0] = config->iref_external ? 0x8E : 0x9E;
	ret = ssd1362_write_command(dev, SSD1362_SET_IREF, data, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_DEFAULT_GRAYSCALE, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_SET_PHASE_LENGTH, &config->phase_length, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_SET_SECOND_PRECHARGE,
				    &config->second_precharge_period, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_SET_PRECHARGE, &config->precharge_voltage, 1);
	if (ret < 0) {
		return ret;
	}

	/* Pre-charge voltage capacitor selection: with Vp capacitor */
	data[0] = 0x01;
	ret = ssd1362_write_command(dev, SSD1362_SET_PRECHARGE_CAP, data, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_SET_VCOMH, &config->vcomh_voltage, 1);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_set_contrast(dev, CONFIG_SSD1362_DEFAULT_CONTRAST);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_blanking_on(dev);
	if (ret < 0) {
		return ret;
	}

	ret = ssd1362_write_command(dev, SSD1362_DISPLAY_ON, NULL, 0);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ssd1362_init(const struct device *dev)
{
	const struct ssd1362_config *config = dev->config;

	if (!device_is_ready(config->mipi_dev)) {
		LOG_ERR("MIPI not ready!");
		return -ENODEV;
	}

	int ret = ssd1362_init_device(dev);

	if (ret < 0) {
		LOG_ERR("Failed to initialize device, err = %d", ret);
		return -EIO;
	}

	return 0;
}

static DEVICE_API(display, ssd1362_driver_api) = {
	.blanking_on = ssd1362_blanking_on,
	.blanking_off = ssd1362_blanking_off,
	.write = ssd1362_write,
	.set_contrast = ssd1362_set_contrast,
	.get_capabilities = ssd1362_get_capabilities,
	.set_pixel_format = ssd1362_set_pixel_format,
};

#define SSD1362_WORD_SIZE(inst)                                                                    \
	((DT_STRING_UPPER_TOKEN(inst, mipi_mode) == MIPI_DBI_MODE_SPI_4WIRE) ? SPI_WORD_SET(8)     \
									     : SPI_WORD_SET(9))

#define SSD1362_CONV_BUFFER_SIZE(node_id)                                                          \
	DIV_ROUND_UP(DT_PROP(node_id, width) * CONFIG_SSD1362_CONV_BUFFER_LINES, SSD1362_2PPB)

#define SSD1362_DEFINE(node_id)                                                                    \
	static uint8_t conversion_buf##node_id[SSD1362_CONV_BUFFER_SIZE(node_id)];                 \
	static struct ssd1362_data data##node_id;                                                  \
	static const struct ssd1362_config config##node_id = {                                     \
		.height = DT_PROP(node_id, height),                                                \
		.width = DT_PROP(node_id, width),                                                  \
		.column_offset = DT_PROP(node_id, column_offset),                                  \
		.row_offset = DT_PROP(node_id, row_offset),                                        \
		.start_line = DT_PROP(node_id, start_line),                                        \
		.mux_ratio = DT_PROP(node_id, mux_ratio),                                          \
		.remap_row_first = DT_PROP(node_id, remap_row_first),                              \
		.remap_columns = DT_PROP(node_id, remap_columns),                                  \
		.remap_rows = DT_PROP(node_id, remap_rows),                                        \
		.remap_nibble = DT_PROP(node_id, remap_nibble),                                    \
		.remap_com_odd_even_split = DT_PROP(node_id, remap_com_odd_even_split),            \
		.remap_com_dual = DT_PROP(node_id, remap_com_dual),                                \
		.color_inversion = DT_PROP(node_id, inversion_on),                                 \
		.iref_external = DT_PROP(node_id, iref_external),                                  \
		.oscillator_freq = DT_PROP(node_id, oscillator_freq),                              \
		.precharge_voltage = DT_PROP(node_id, precharge_voltage),                          \
		.vcomh_voltage = DT_PROP(node_id, vcomh_voltage),                                  \
		.phase_length = DT_PROP(node_id, phase_length),                                    \
		.second_precharge_period = DT_PROP(node_id, second_precharge_period),              \
		.mipi_dev = DEVICE_DT_GET(DT_PARENT(node_id)),                                     \
		.dbi_config = MIPI_DBI_CONFIG_DT(                                                  \
			node_id, SSD1362_WORD_SIZE(node_id) | SPI_OP_MODE_MASTER, 0),              \
		.conversion_buf = conversion_buf##node_id,                                         \
		.conversion_buf_size = sizeof(conversion_buf##node_id),                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_DEFINE(node_id, ssd1362_init, NULL, &data##node_id, &config##node_id,            \
			 POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY, &ssd1362_driver_api);

DT_FOREACH_STATUS_OKAY(solomon_ssd1362, SSD1362_DEFINE)
