/*
 * Copyright (C) 2013 ARCOS-Lab Universidad de Costa Rica
 * Author: Federico Ruiz Ugalde <memeruiz@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

void ad2s1210_init(void);
void ad2s1210_conf(void);

#define AD2S1210_WRPIN GPIO2
#define AD2S1210_WRPIN_PORT GPIOA
#define AD2S1210_SPI SPI2
#define AD2S1210_SAMPLEPIN GPIO2
#define AD2S1210_SAMPLEPIN_PORT GPIOC

#define AD2S1210_REG_EXC_FREQ 0x91
#define AD2S1210_REG_FAULT 0xff
#define AD2S1210_REG_POS_H 0x80
#define AD2S1210_REG_POS_L 0x81
#define AD2S1210_REG_VEL_H 0x82
#define AD2S1210_REG_VEL_L 0x83

#define AD2S1210_WR(reg, data) {\
  gpio_clear(AD2S1210_WRPIN_PORT, AD2S1210_WRPIN);\
  spi_write(AD2S1210_SPI, reg);			  \
  spi_read(AD2S1210_SPI); \
  gpio_set(AD2S1210_WRPIN_PORT, AD2S1210_WRPIN);\
  gpio_clear(AD2S1210_WRPIN_PORT, AD2S1210_WRPIN);\
  spi_write(AD2S1210_SPI, data);			  \
  spi_read(AD2S1210_SPI); \
  gpio_set(AD2S1210_WRPIN_PORT, AD2S1210_WRPIN);\
  }

#define AD2S1210_RD(reg) ({\
  uint8_t data;\
  gpio_clear(AD2S1210_WRPIN_PORT, AD2S1210_WRPIN);\
  spi_write(AD2S1210_SPI, reg);			  \
  spi_read(AD2S1210_SPI); \
  gpio_set(AD2S1210_WRPIN_PORT, AD2S1210_WRPIN);\
  gpio_clear(AD2S1210_WRPIN_PORT, AD2S1210_WRPIN);\
  spi_write(AD2S1210_SPI, 0x00); \
  data = spi_read(AD2S1210_SPI); \
  gpio_set(AD2S1210_WRPIN_PORT, AD2S1210_WRPIN);\
  data;\
    })

#define WAIT_100NANO(ns) {\
  int i;\
  for (i = 0; i < 7*ns; i++)    /* Wait a bit. */	\
    __asm__("nop");\
  }


/* #define AD2S1210_SAMPLE() {\ */
/*   gpio_set(AD2S1210_SAMPLEPIN_PORT, AD2S1210_SAMPLEPIN);\ */
/*   gpio_clear(AD2S1210_SAMPLEPIN_PORT, AD2S1210_SAMPLEPIN);\ */
/*   WAIT_100NANO(2); \ */
/*   gpio_set(AD2S1210_SAMPLEPIN_PORT, AD2S1210_SAMPLEPIN);\ */
/*   WAIT_100NANO(5); \ */
/*   } */

#define AD2S1210_SAMPLE() {\
  gpio_set(AD2S1210_SAMPLEPIN_PORT, AD2S1210_SAMPLEPIN);\
  gpio_clear(AD2S1210_SAMPLEPIN_PORT, AD2S1210_SAMPLEPIN);\
  WAIT_100NANO(4); \
  gpio_set(AD2S1210_SAMPLEPIN_PORT, AD2S1210_SAMPLEPIN);\
  WAIT_100NANO(8); \
  }

