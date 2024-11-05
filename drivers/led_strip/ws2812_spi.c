/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2019, Nordic Semiconductor ASA
 * Copyright (c) 2021 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT worldsemi_ws2812_spi

#include <zephyr/drivers/led_strip.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ws2812_spi);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/led/led.h>

/* bits per color for WS2812, WS2813, WS2814, WS2815 but not WS2816 */
#define WS2812_BITS_PER_COLOR 8

/* spi-one-frame and spi-zero-frame in DT are for 8-bit frames. */
#define SPI_FRAME_BITS 8

/*
 * SPI master configuration:
 *
 * - mode 0 (the default), 8 bit, MSB first (arbitrary), one-line SPI
 * - no shenanigans (don't hold CS, don't hold the device lock, this
 *   isn't an EEPROM)
 */
#define SPI_OPER(idx) (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | \
		  SPI_WORD_SET(SPI_FRAME_BITS))

struct ws2812_spi_cfg {
	struct spi_dt_spec bus;
	uint8_t *px_buf;
	uint8_t frame_pattern[2];
	uint8_t frame_bits;
	uint8_t num_colors;
	const uint8_t *color_mapping;
	size_t length;
	uint16_t reset_delay;
};

static const struct ws2812_spi_cfg *dev_cfg(const struct device *dev)
{
	return dev->config;
}

/*
 * Serialize an 8-bit color channel value into an equivalent sequence
 * of SPI frames, MSbit first, where a one bit becomes SPI frame
 * one_frame, and zero bit becomes zero_frame.
 */
static inline void ws2812_spi_ser(uint8_t *buf, size_t *bit_offset, uint8_t color,
				  const uint8_t *frame_pattern, const uint8_t frame_bits)
{
	for (uint8_t mask = BIT(WS2812_BITS_PER_COLOR - 1); mask != 0; mask >>= 1) {
		uint8_t pattern = frame_pattern[(color & mask) ? 1 : 0];
		size_t byte_offset = *bit_offset / SPI_FRAME_BITS;
		size_t bits = *bit_offset % SPI_FRAME_BITS;

		if (bits == 0) {
			buf[byte_offset] = pattern;
		} else {
			buf[byte_offset] |= pattern >> bits;
			if (bits > (SPI_FRAME_BITS - frame_bits)) {
				buf[byte_offset + 1] = (pattern << (SPI_FRAME_BITS - bits));
			}
		}
		*bit_offset += frame_bits;
	}
}

/*
 * Latch current color values on strip and reset its state machines.
 */
static inline void ws2812_reset_delay(uint16_t delay)
{
	k_usleep(delay);
}

static int ws2812_strip_update_rgb(const struct device *dev,
				   struct led_rgb *pixels,
				   size_t num_pixels)
{
	const struct ws2812_spi_cfg *cfg = dev_cfg(dev);
	const uint8_t *frame_pattern = cfg->frame_pattern;
	const uint8_t frame_bits = cfg->frame_bits;

	struct spi_buf buf = {
		.buf = cfg->px_buf,
		.len = DIV_ROUND_UP(cfg->length * WS2812_BITS_PER_COLOR * cfg->num_colors *
					     frame_bits,
				     CHAR_BIT),
	};
	const struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};
	size_t bit_offset = 0;
	int rc;

	/*
	 * Convert pixel data into SPI frames. Each frame has pixel data
	 * in color mapping on-wire format (e.g. GRB, GRBW, RGB, etc).
	 */
	for (size_t i = 0; i < num_pixels; i++) {
		uint8_t j;

		for (j = 0; j < cfg->num_colors; j++) {
			uint8_t pixel;

			switch (cfg->color_mapping[j]) {
			/* White channel is not supported by LED strip API. */
			case LED_COLOR_ID_WHITE:
				pixel = 0;
				break;
			case LED_COLOR_ID_RED:
				pixel = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				pixel = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				pixel = pixels[i].b;
				break;
			default:
				return -EINVAL;
			}
			ws2812_spi_ser(cfg->px_buf, &bit_offset, pixel, frame_pattern, frame_bits);
		}
	}

	/*
	 * Display the pixel data.
	 */
	rc = spi_write_dt(&cfg->bus, &tx);
	ws2812_reset_delay(cfg->reset_delay);

	return rc;
}

static size_t ws2812_strip_length(const struct device *dev)
{
	const struct ws2812_spi_cfg *cfg = dev_cfg(dev);

	return cfg->length;
}

static int ws2812_spi_init(const struct device *dev)
{
	const struct ws2812_spi_cfg *cfg = dev_cfg(dev);
	uint8_t i;

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI device %s not ready", cfg->bus.bus->name);
		return -ENODEV;
	}

	for (i = 0; i < cfg->num_colors; i++) {
		switch (cfg->color_mapping[i]) {
		case LED_COLOR_ID_WHITE:
		case LED_COLOR_ID_RED:
		case LED_COLOR_ID_GREEN:
		case LED_COLOR_ID_BLUE:
			break;
		default:
			LOG_ERR("%s: invalid channel to color mapping."
				"Check the color-mapping DT property",
				dev->name);
			return -EINVAL;
		}
	}

	return 0;
}

static const struct led_strip_driver_api ws2812_spi_api = {
	.update_rgb = ws2812_strip_update_rgb,
	.length = ws2812_strip_length,
};

#define WS2812_SPI_NUM_PIXELS(idx) \
	(DT_INST_PROP(idx, chain_length))
#define WS2812_SPI_HAS_WHITE(idx) \
	(DT_INST_PROP(idx, has_white_channel) == 1)
#define WS2812_SPI_FRAME_BITS(idx) (DT_INST_PROP(idx, spi_frame_bits))
#define WS2812_SPI_ONE_FRAME(idx) \
	(DT_INST_PROP(idx, spi_one_frame) << (SPI_FRAME_BITS - WS2812_SPI_FRAME_BITS(idx)))
#define WS2812_SPI_ZERO_FRAME(idx) \
	(DT_INST_PROP(idx, spi_zero_frame) << (SPI_FRAME_BITS - WS2812_SPI_FRAME_BITS(idx)))
#define WS2812_SPI_BUFSZ(idx) \
	DIV_ROUND_UP(WS2812_NUM_COLORS(idx) * WS2812_BITS_PER_COLOR * WS2812_SPI_NUM_PIXELS(idx) * \
			     WS2812_SPI_FRAME_BITS(idx),                                           \
		     CHAR_BIT)

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)				  \
	static const uint8_t ws2812_spi_##idx##_color_mapping[] = \
		DT_INST_PROP(idx, color_mapping)

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))

/* Get the latch/reset delay from the "reset-delay" DT property. */
#define WS2812_RESET_DELAY(idx) DT_INST_PROP(idx, reset_delay)

#define WS2812_SPI_DEVICE(idx)						 \
									 \
	static uint8_t ws2812_spi_##idx##_px_buf[WS2812_SPI_BUFSZ(idx)]; \
									 \
	WS2812_COLOR_MAPPING(idx);					 \
									 \
	static const struct ws2812_spi_cfg ws2812_spi_##idx##_cfg = {	 \
		.bus = SPI_DT_SPEC_INST_GET(idx, SPI_OPER(idx), 0),	 \
		.px_buf = ws2812_spi_##idx##_px_buf,			 \
		.frame_pattern = {WS2812_SPI_ZERO_FRAME(idx), WS2812_SPI_ONE_FRAME(idx)},          \
		.frame_bits = WS2812_SPI_FRAME_BITS(idx),                                          \
		.num_colors = WS2812_NUM_COLORS(idx),			 \
		.color_mapping = ws2812_spi_##idx##_color_mapping,	 \
		.length = DT_INST_PROP(idx, chain_length),               \
		.reset_delay = WS2812_RESET_DELAY(idx),			 \
	};								 \
									 \
	DEVICE_DT_INST_DEFINE(idx,					 \
			      ws2812_spi_init,				 \
			      NULL,					 \
			      NULL,					 \
			      &ws2812_spi_##idx##_cfg,			 \
			      POST_KERNEL,				 \
			      CONFIG_LED_STRIP_INIT_PRIORITY,		 \
			      &ws2812_spi_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_SPI_DEVICE)
