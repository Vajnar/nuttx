/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_dma_v1.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_DMA_V1_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_DMA_V1_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is implementation for
 * STM32 DMA IP version 1 - F0, F1, F3, L0, L1, L4
 */

#define HAVE_IP_DMA_V1 1
#undef  HAVE_IP_DMA_V2

/* F0, L0, L4 have additional CSELR register */

#if defined(CONFIG_ARCH_CHIP_STM32L0)
#  define DMA_HAVE_CSELR 1
#endif

/* 2 DMA controllers */

#define DMA1                       (0)
#define DMA2                       (1)

/* 12 Channels Total: 7 DMA1 Channels(1-7) and 5 DMA2 channels (1-5) */

#define DMA_CHAN1                  (0)
#define DMA_CHAN2                  (1)
#define DMA_CHAN3                  (2)
#define DMA_CHAN4                  (3)
#define DMA_CHAN5                  (4)
#define DMA_CHAN6                  (5)
#define DMA_CHAN7                  (6)

/* Register Offsets *********************************************************/

#define STM32_DMA_ISR_OFFSET       0x0000 /* DMA interrupt status register */
#define STM32_DMA_IFCR_OFFSET      0x0004 /* DMA interrupt flag clear register */

#define STM32_DMACHAN_OFFSET(n)    (0x0014 * (n))
#define STM32_DMACHAN1_OFFSET      0x0000
#define STM32_DMACHAN2_OFFSET      0x0014
#define STM32_DMACHAN3_OFFSET      0x0028
#define STM32_DMACHAN4_OFFSET      0x003c
#define STM32_DMACHAN5_OFFSET      0x0050
#define STM32_DMACHAN6_OFFSET      0x0064
#define STM32_DMACHAN7_OFFSET      0x0078

#define STM32_DMACHAN_CCR_OFFSET   0x0008 /* DMA channel configuration register */
#define STM32_DMACHAN_CNDTR_OFFSET 0x000c /* DMA channel number of data register */
#define STM32_DMACHAN_CPAR_OFFSET  0x0010 /* DMA channel peripheral address register */
#define STM32_DMACHAN_CMAR_OFFSET  0x0014 /* DMA channel 1 memory address register */

#define STM32_DMA_CCR_OFFSET(n)   (STM32_DMACHAN_CCR_OFFSET + STM32_DMACHAN_OFFSET(n))
#define STM32_DMA_CNDTR_OFFSET(n) (STM32_DMACHAN_CNDTR_OFFSET + STM32_DMACHAN_OFFSET(n))
#define STM32_DMA_CPAR_OFFSET(n)  (STM32_DMACHAN_CPAR_OFFSET + STM32_DMACHAN_OFFSET(n))
#define STM32_DMA_CMAR_OFFSET(n)  (STM32_DMACHAN_CMAR_OFFSET + STM32_DMACHAN_OFFSET(n))

#define STM32_DMA_CCR1_OFFSET     0x0008 /* DMA channel 1 configuration register */
#define STM32_DMA_CCR2_OFFSET     0x001c /* DMA channel 2 configuration register */
#define STM32_DMA_CCR3_OFFSET     0x0030 /* DMA channel 3 configuration register */
#define STM32_DMA_CCR4_OFFSET     0x0044 /* DMA channel 4 configuration register */
#define STM32_DMA_CCR5_OFFSET     0x0058 /* DMA channel 5 configuration register */
#define STM32_DMA_CCR6_OFFSET     0x006c /* DMA channel 6 configuration register */
#define STM32_DMA_CCR7_OFFSET     0x0080 /* DMA channel 7 configuration register */

#define STM32_DMA_CNDTR1_OFFSET   0x000c /* DMA channel 1 number of data register */
#define STM32_DMA_CNDTR2_OFFSET   0x0020 /* DMA channel 2 number of data register */
#define STM32_DMA_CNDTR3_OFFSET   0x0034 /* DMA channel 3 number of data register */
#define STM32_DMA_CNDTR4_OFFSET   0x0048 /* DMA channel 4 number of data register */
#define STM32_DMA_CNDTR5_OFFSET   0x005c /* DMA channel 5 number of data register */
#define STM32_DMA_CNDTR6_OFFSET   0x0070 /* DMA channel 6 number of data register */
#define STM32_DMA_CNDTR7_OFFSET   0x0084 /* DMA channel 7 number of data register */

#define STM32_DMA_CPAR1_OFFSET    0x0010 /* DMA channel 1 peripheral address register */
#define STM32_DMA_CPAR2_OFFSET    0x0024 /* DMA channel 2 peripheral address register */
#define STM32_DMA_CPAR3_OFFSET    0x0038 /* DMA channel 3 peripheral address register */
#define STM32_DMA_CPAR4_OFFSET    0x004c /* DMA channel 4 peripheral address register */
#define STM32_DMA_CPAR5_OFFSET    0x0060 /* DMA channel 5 peripheral address register */
#define STM32_DMA_CPAR6_OFFSET    0x0074 /* DMA channel 6 peripheral address register */
#define STM32_DMA_CPAR7_OFFSET    0x0088 /* DMA channel 7 peripheral address register */

#define STM32_DMA_CMAR1_OFFSET    0x0014 /* DMA channel 1 memory address register */
#define STM32_DMA_CMAR2_OFFSET    0x0028 /* DMA channel 2 memory address register */
#define STM32_DMA_CMAR3_OFFSET    0x003c /* DMA channel 3 memory address register */
#define STM32_DMA_CMAR4_OFFSET    0x0050 /* DMA channel 4 memory address register */
#define STM32_DMA_CMAR5_OFFSET    0x0064 /* DMA channel 5 memory address register */
#define STM32_DMA_CMAR6_OFFSET    0x0078 /* DMA channel 6 memory address register */
#define STM32_DMA_CMAR7_OFFSET    0x008c /* DMA channel 7 memory address register */

#ifdef DMA_HAVE_CSELR
#  define STM32_DMA_CSELR_OFFSET  0x00a8 /* DMA channel selection register */
#endif

/* Register Addresses *******************************************************/

#define STM32_DMA1_ISRC           (STM32_DMA1_BASE + STM32_DMA_ISR_OFFSET)
#define STM32_DMA1_IFCR           (STM32_DMA1_BASE + STM32_DMA_IFCR_OFFSET)

#define STM32_DMA1_CCR(n)         (STM32_DMA1_BASE + STM32_DMA_CCR_OFFSET(n))
#define STM32_DMA1_CCR1           (STM32_DMA1_BASE + STM32_DMA_CCR1_OFFSET)
#define STM32_DMA1_CCR2           (STM32_DMA1_BASE + STM32_DMA_CCR2_OFFSET)
#define STM32_DMA1_CCR3           (STM32_DMA1_BASE + STM32_DMA_CCR3_OFFSET)
#define STM32_DMA1_CCR4           (STM32_DMA1_BASE + STM32_DMA_CCR4_OFFSET)
#define STM32_DMA1_CCR5           (STM32_DMA1_BASE + STM32_DMA_CCR5_OFFSET)
#define STM32_DMA1_CCR6           (STM32_DMA1_BASE + STM32_DMA_CCR6_OFFSET)
#define STM32_DMA1_CCR7           (STM32_DMA1_BASE + STM32_DMA_CCR7_OFFSET)

#define STM32_DMA1_CNDTR(n)       (STM32_DMA1_BASE + STM32_DMA_CNDTR_OFFSET(n))
#define STM32_DMA1_CNDTR1         (STM32_DMA1_BASE + STM32_DMA_CNDTR1_OFFSET)
#define STM32_DMA1_CNDTR2         (STM32_DMA1_BASE + STM32_DMA_CNDTR2_OFFSET)
#define STM32_DMA1_CNDTR3         (STM32_DMA1_BASE + STM32_DMA_CNDTR3_OFFSET)
#define STM32_DMA1_CNDTR4         (STM32_DMA1_BASE + STM32_DMA_CNDTR4_OFFSET)
#define STM32_DMA1_CNDTR5         (STM32_DMA1_BASE + STM32_DMA_CNDTR5_OFFSET)
#define STM32_DMA1_CNDTR6         (STM32_DMA1_BASE + STM32_DMA_CNDTR6_OFFSET)
#define STM32_DMA1_CNDTR7         (STM32_DMA1_BASE + STM32_DMA_CNDTR7_OFFSET)

#define STM32_DMA1_CPAR(n)        (STM32_DMA1_BASE + STM32_DMA_CPAR_OFFSET(n))
#define STM32_DMA1_CPAR1          (STM32_DMA1_BASE + STM32_DMA_CPAR1_OFFSET)
#define STM32_DMA1_CPAR2          (STM32_DMA1_BASE + STM32_DMA_CPAR2_OFFSET)
#define STM32_DMA1_CPAR3          (STM32_DMA1_BASE + STM32_DMA_CPAR3_OFFSET)
#define STM32_DMA1_CPAR4          (STM32_DMA1_BASE + STM32_DMA_CPAR4_OFFSET)
#define STM32_DMA1_CPAR5          (STM32_DMA1_BASE + STM32_DMA_CPAR5_OFFSET)
#define STM32_DMA1_CPAR6          (STM32_DMA1_BASE + STM32_DMA_CPAR6_OFFSET)
#define STM32_DMA1_CPAR7          (STM32_DMA1_BASE + STM32_DMA_CPAR7_OFFSET)

#define STM32_DMA1_CMAR(n)        (STM32_DMA1_BASE + STM32_DMA_CMAR_OFFSET(n))
#define STM32_DMA1_CMAR1          (STM32_DMA1_BASE + STM32_DMA_CMAR1_OFFSET)
#define STM32_DMA1_CMAR2          (STM32_DMA1_BASE + STM32_DMA_CMAR2_OFFSET)
#define STM32_DMA1_CMAR3          (STM32_DMA1_BASE + STM32_DMA_CMAR3_OFFSET)
#define STM32_DMA1_CMAR4          (STM32_DMA1_BASE + STM32_DMA_CMAR4_OFFSET)
#define STM32_DMA1_CMAR5          (STM32_DMA1_BASE + STM32_DMA_CMAR5_OFFSET)
#define STM32_DMA1_CMAR6          (STM32_DMA1_BASE + STM32_DMA_CMAR6_OFFSET)
#define STM32_DMA1_CMAR7          (STM32_DMA1_BASE + STM32_DMA_CMAR7_OFFSET)

#define STM32_DMA2_ISRC           (STM32_DMA2_BASE + STM32_DMA_ISR_OFFSET)
#define STM32_DMA2_IFCR           (STM32_DMA2_BASE + STM32_DMA_IFCR_OFFSET)

#define STM32_DMA2_CCR(n)         (STM32_DMA2_BASE + STM32_DMA_CCR_OFFSET(n))
#define STM32_DMA2_CCR1           (STM32_DMA2_BASE + STM32_DMA_CCR1_OFFSET)
#define STM32_DMA2_CCR2           (STM32_DMA2_BASE + STM32_DMA_CCR2_OFFSET)
#define STM32_DMA2_CCR3           (STM32_DMA2_BASE + STM32_DMA_CCR3_OFFSET)
#define STM32_DMA2_CCR4           (STM32_DMA2_BASE + STM32_DMA_CCR4_OFFSET)
#define STM32_DMA2_CCR5           (STM32_DMA2_BASE + STM32_DMA_CCR5_OFFSET)

#define STM32_DMA2_CNDTR(n)       (STM32_DMA2_BASE + STM32_DMA_CNDTR_OFFSET(n))
#define STM32_DMA2_CNDTR1         (STM32_DMA2_BASE + STM32_DMA_CNDTR1_OFFSET)
#define STM32_DMA2_CNDTR2         (STM32_DMA2_BASE + STM32_DMA_CNDTR2_OFFSET)
#define STM32_DMA2_CNDTR3         (STM32_DMA2_BASE + STM32_DMA_CNDTR3_OFFSET)
#define STM32_DMA2_CNDTR4         (STM32_DMA2_BASE + STM32_DMA_CNDTR4_OFFSET)
#define STM32_DMA2_CNDTR5         (STM32_DMA2_BASE + STM32_DMA_CNDTR5_OFFSET)

#define STM32_DMA2_CPAR(n)        (STM32_DMA2_BASE + STM32_DMA_CPAR_OFFSET(n))
#define STM32_DMA2_CPAR1          (STM32_DMA2_BASE + STM32_DMA_CPAR1_OFFSET)
#define STM32_DMA2_CPAR2          (STM32_DMA2_BASE + STM32_DMA_CPAR2_OFFSET)
#define STM32_DMA2_CPAR3          (STM32_DMA2_BASE + STM32_DMA_CPAR3_OFFSET)
#define STM32_DMA2_CPAR4          (STM32_DMA2_BASE + STM32_DMA_CPAR4_OFFSET)
#define STM32_DMA2_CPAR5          (STM32_DMA2_BASE + STM32_DMA_CPAR5_OFFSET)

#define STM32_DMA2_CMAR(n)        (STM32_DMA2_BASE + STM32_DMA_CMAR_OFFSET(n))
#define STM32_DMA2_CMAR1          (STM32_DMA2_BASE + STM32_DMA_CMAR1_OFFSET)
#define STM32_DMA2_CMAR2          (STM32_DMA2_BASE + STM32_DMA_CMAR2_OFFSET)
#define STM32_DMA2_CMAR3          (STM32_DMA2_BASE + STM32_DMA_CMAR3_OFFSET)
#define STM32_DMA2_CMAR4          (STM32_DMA2_BASE + STM32_DMA_CMAR4_OFFSET)
#define STM32_DMA2_CMAR5          (STM32_DMA2_BASE + STM32_DMA_CMAR5_OFFSET)

#ifdef DMA_HAVE_CSELR
#  define STM32_DMA_CSELR         (STM32_DMA2_BASE + STM32_DMA_CSELR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

#define DMA_CHAN_SHIFT(n)         ((n) << 2)
#define DMA_CHAN_MASK             0x0f
#define DMA_CHAN_GIF_BIT          (1 << 0)  /* Bit 0: Channel Global interrupt flag */
#define DMA_CHAN_TCIF_BIT         (1 << 1)  /* Bit 1: Channel Transfer Complete flag */
#define DMA_CHAN_HTIF_BIT         (1 << 2)  /* Bit 2: Channel Half Transfer flag */
#define DMA_CHAN_TEIF_BIT         (1 << 3)  /* Bit 3: Channel Transfer Error flag */

/* DMA interrupt status register */

#define DMA_ISR_CHAN_SHIFT(n)     DMA_CHAN_SHIFT(n)
#define DMA_ISR_CHAN_MASK(n)      (DMA_CHAN_MASK <<  DMA_ISR_CHAN_SHIFT(n))
#define DMA_ISR_CHAN1_SHIFT       (0)       /* Bits 3-0:  DMA Channel 1 interrupt status */
#define DMA_ISR_CHAN1_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN1_SHIFT)
#define DMA_ISR_CHAN2_SHIFT       (4)       /* Bits 7-4:  DMA Channel 2 interrupt status */
#define DMA_ISR_CHAN2_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN2_SHIFT)
#define DMA_ISR_CHAN3_SHIFT       (8)       /* Bits 11-8:  DMA Channel 3 interrupt status */
#define DMA_ISR_CHAN3_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN3_SHIFT)
#define DMA_ISR_CHAN4_SHIFT       (12)      /* Bits 15-12:  DMA Channel 4 interrupt status */
#define DMA_ISR_CHAN4_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN4_SHIFT)
#define DMA_ISR_CHAN5_SHIFT       (16)      /* Bits 19-16:  DMA Channel 5 interrupt status */
#define DMA_ISR_CHAN5_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN5_SHIFT)
#define DMA_ISR_CHAN6_SHIFT       (20)      /* Bits 23-20:  DMA Channel 6 interrupt status */
#define DMA_ISR_CHAN6_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN6_SHIFT)
#define DMA_ISR_CHAN7_SHIFT       (24)      /* Bits 27-24:  DMA Channel 7 interrupt status */
#define DMA_ISR_CHAN7_MASK        (DMA_CHAN_MASK <<  DMA_ISR_CHAN7_SHIFT)

#define DMA_ISR_GIF(n)            (DMA_CHAN_GIF_BIT << DMA_ISR_CHAN_SHIFT(n))
#define DMA_ISR_TCIF(n)           (DMA_CHAN_TCIF_BIT << DMA_ISR_CHAN_SHIFT(n))
#define DMA_ISR_HTIF(n)           (DMA_CHAN_HTIF_BIT << DMA_ISR_CHAN_SHIFT(n))
#define DMA_ISR_TEIF(n)           (DMA_CHAN_TEIF_BIT << DMA_ISR_CHAN_SHIFT(n))

/* DMA interrupt flag clear register */

#define DMA_IFCR_CHAN_SHIFT(n)    DMA_CHAN_SHIFT(n)
#define DMA_IFCR_CHAN_MASK(n)     (DMA_CHAN_MASK <<  DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CHAN1_SHIFT      (0)       /* Bits 3-0:  DMA Channel 1 interrupt flag clear */
#define DMA_IFCR_CHAN1_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN1_SHIFT)
#define DMA_IFCR_CHAN2_SHIFT      (4)       /* Bits 7-4:  DMA Channel 2 interrupt flag clear */
#define DMA_IFCR_CHAN2_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN2_SHIFT)
#define DMA_IFCR_CHAN3_SHIFT      (8)       /* Bits 11-8:  DMA Channel 3 interrupt flag clear */
#define DMA_IFCR_CHAN3_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN3_SHIFT)
#define DMA_IFCR_CHAN4_SHIFT      (12)      /* Bits 15-12:  DMA Channel 4 interrupt flag clear */
#define DMA_IFCR_CHAN4_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN4_SHIFT)
#define DMA_IFCR_CHAN5_SHIFT      (16)      /* Bits 19-16:  DMA Channel 5 interrupt flag clear */
#define DMA_IFCR_CHAN5_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN5_SHIFT)
#define DMA_IFCR_CHAN6_SHIFT      (20)      /* Bits 23-20:  DMA Channel 6 interrupt flag clear */
#define DMA_IFCR_CHAN6_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN6_SHIFT)
#define DMA_IFCR_CHAN7_SHIFT      (24)      /* Bits 27-24:  DMA Channel 7 interrupt flag clear */
#define DMA_IFCR_CHAN7_MASK       (DMA_CHAN_MASK <<  DMA_IFCR_CHAN7_SHIFT)
#define DMA_IFCR_ALLCHANNELS      (0x0fffffff)

#define DMA_IFCR_CGIF(n)          (DMA_CHAN_GIF_BIT << DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CTCIF(n)         (DMA_CHAN_TCIF_BIT << DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CHTIF(n)         (DMA_CHAN_HTIF_BIT << DMA_IFCR_CHAN_SHIFT(n))
#define DMA_IFCR_CTEIF(n)         (DMA_CHAN_TEIF_BIT << DMA_IFCR_CHAN_SHIFT(n))

/* DMA channel configuration register */

#define DMA_CCR_EN                (1 << 0)  /* Bit 0: Channel enable */
#define DMA_CCR_TCIE              (1 << 1)  /* Bit 1: Transfer complete interrupt enable */
#define DMA_CCR_HTIE              (1 << 2)  /* Bit 2: Half Transfer interrupt enable */
#define DMA_CCR_TEIE              (1 << 3)  /* Bit 3: Transfer error interrupt enable */
#define DMA_CCR_DIR               (1 << 4)  /* Bit 4: Data transfer direction */
#define DMA_CCR_CIRC              (1 << 5)  /* Bit 5: Circular mode */
#define DMA_CCR_PINC              (1 << 6)  /* Bit 6: Peripheral increment mode */
#define DMA_CCR_MINC              (1 << 7)  /* Bit 7: Memory increment mode */
#define DMA_CCR_PSIZE_SHIFT       (8)       /* Bits 8-9: Peripheral size */
#define DMA_CCR_PSIZE_MASK        (3 << DMA_CCR_PSIZE_SHIFT)
#  define DMA_CCR_PSIZE_8BITS     (0 << DMA_CCR_PSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_CCR_PSIZE_16BITS    (1 << DMA_CCR_PSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_CCR_PSIZE_32BITS    (2 << DMA_CCR_PSIZE_SHIFT) /* 10: 32-bits */

#define DMA_CCR_MSIZE_SHIFT       (10)      /* Bits 10-11: Memory size */
#define DMA_CCR_MSIZE_MASK        (3 << DMA_CCR_MSIZE_SHIFT)
#  define DMA_CCR_MSIZE_8BITS     (0 << DMA_CCR_MSIZE_SHIFT) /* 00: 8-bits */
#  define DMA_CCR_MSIZE_16BITS    (1 << DMA_CCR_MSIZE_SHIFT) /* 01: 16-bits */
#  define DMA_CCR_MSIZE_32BITS    (2 << DMA_CCR_MSIZE_SHIFT) /* 10: 32-bits */

#define DMA_CCR_PL_SHIFT          (12)      /* Bits 12-13: Channel Priority level */
#define DMA_CCR_PL_MASK           (3 << DMA_CCR_PL_SHIFT)
#  define DMA_CCR_PRILO           (0 << DMA_CCR_PL_SHIFT) /* 00: Low */
#  define DMA_CCR_PRIMED          (1 << DMA_CCR_PL_SHIFT) /* 01: Medium */
#  define DMA_CCR_PRIHI           (2 << DMA_CCR_PL_SHIFT) /* 10: High */
#  define DMA_CCR_PRIVERYHI       (3 << DMA_CCR_PL_SHIFT) /* 11: Very high */

#define DMA_CCR_MEM2MEM           (1 << 14) /* Bit 14: Memory to memory mode */

#define DMA_CCR_ALLINTS           (DMA_CCR_TEIE|DMA_CCR_HTIE|DMA_CCR_TCIE)

/* DMA channel number of data register */

#define DMA_CNDTR_NDT_SHIFT       (0)       /* Bits 15-0: Number of data to Transfer */
#define DMA_CNDTR_NDT_MASK        (0xffff << DMA_CNDTR_NDT_SHIFT)

/* DMA Channel mapping.  Each DMA channel has a mapping to several possible
 * sources/sinks of data.  The requests from peripherals assigned to a
 * channel are simply OR'ed together before entering the DMA block.  This
 * means that only one request on a given channel can be enabled at once.
 *
 * Alternative DMA channel selections are provided with a numeric suffix like
 * _1, _2, etc.  Drivers, however, will use the pin selection without the
 * numeric suffix. Additional definitions are required in the board.h file.
 */

#define STM32_DMA1_CHAN1          (0)
#define STM32_DMA1_CHAN2          (1)
#define STM32_DMA1_CHAN3          (2)
#define STM32_DMA1_CHAN4          (3)
#define STM32_DMA1_CHAN5          (4)
#define STM32_DMA1_CHAN6          (5)
#define STM32_DMA1_CHAN7          (6)

#define STM32_DMA2_CHAN1          (7)
#define STM32_DMA2_CHAN2          (8)
#define STM32_DMA2_CHAN3          (9)
#define STM32_DMA2_CHAN4          (10)
#define STM32_DMA2_CHAN5          (11)

#ifdef DMA_HAVE_CSELR
#  define DMACHAN_SETTING(chan, sel)     ((((sel) & 0xff) << 8) | ((chan) & 0xff))
#  define DMACHAN_SETTING_CHANNEL_MASK   0x00ff
#  define DMACHAN_SETTING_CHANNEL_SHIFT  (0)
#  define DMACHAN_SETTING_FUNCTION_MASK  0xff00
#  define DMACHAN_SETTING_FUNCTION_SHIFT (8)
#endif

#if defined(CONFIG_ARCH_CHIP_STM32F0)
/* REVISIT: based on STM32L4 DMA Header */

/* ADC */

#  define DMACHAN_ADC1_1          DMACHAN_SETTING(STM32_DMA1_CHAN1, 0)
#  define DMACHAN_ADC1_2          DMACHAN_SETTING(STM32_DMA2_CHAN3, 0)

#  define DMACHAN_ADC2_1          DMACHAN_SETTING(STM32_DMA1_CHAN1, 0)
#  define DMACHAN_ADC2_2          DMACHAN_SETTING(STM32_DMA2_CHAN4, 0)

#  define DMACHAN_ADC3_1          DMACHAN_SETTING(STM32_DMA1_CHAN1, 0)
#  define DMACHAN_ADC3_2          DMACHAN_SETTING(STM32_DMA2_CHAN5, 0)

/* AES */

#  define DMACHAN_AES_IN_1        DMACHAN_SETTING(STM32_DMA2_CHAN1, 6)
#  define DMACHAN_AES_IN_2        DMACHAN_SETTING(STM32_DMA2_CHAN5, 6)
#  define DMACHAN_AES_OUT_1       DMACHAN_SETTING(STM32_DMA2_CHAN2, 6)
#  define DMACHAN_AES_OUT_2       DMACHAN_SETTING(STM32_DMA2_CHAN3, 6)

/* DAC */

#  define DMACHAN_DAC1_1          DMACHAN_SETTING(STM32_DMA1_CHAN3, 6)
#  define DMACHAN_DAC1_2          DMACHAN_SETTING(STM32_DMA1_CHAN4, 5)
#  define DMACHAN_DAC1_3          DMACHAN_SETTING(STM32_DMA2_CHAN4, 3)

#  define DMACHAN_DAC2            DMACHAN_SETTING(STM32_DMA2_CHAN5, 3)

/* I2C */

#  define DMACHAN_I2C1_RX_1       DMACHAN_SETTING(STM32_DMA1_CHAN7, 3)
#  define DMACHAN_I2C1_RX_2       DMACHAN_SETTING(STM32_DMA2_CHAN6, 5)
#  define DMACHAN_I2C1_TX_1       DMACHAN_SETTING(STM32_DMA1_CHAN6, 3)
#  define DMACHAN_I2C1_TX_2       DMACHAN_SETTING(STM32_DMA2_CHAN7, 5)

#  define DMACHAN_I2C2_RX         DMACHAN_SETTING(STM32_DMA1_CHAN5, 3)
#  define DMACHAN_I2C2_TX         DMACHAN_SETTING(STM32_DMA1_CHAN4, 3)

#  define DMACHAN_I2C3_RX         DMACHAN_SETTING(STM32_DMA1_CHAN3, 2)
#  define DMACHAN_I2C3_TX         DMACHAN_SETTING(STM32_DMA1_CHAN2, 3)

/* QUADSPI */

#  define DMACHAN_QUADSPI_1       DMACHAN_SETTING(STM32_DMA1_CHAN5, 5)
#  define DMACHAN_QUADSPI_2       DMACHAN_SETTING(STM32_DMA2_CHAN7, 3)

/* SPI */

#  define DMACHAN_SPI1_RX_1       DMACHAN_SETTING(STM32_DMA1_CHAN2, 1)
#  define DMACHAN_SPI1_RX_2       DMACHAN_SETTING(STM32_DMA2_CHAN3, 4)
#  define DMACHAN_SPI1_TX_1       DMACHAN_SETTING(STM32_DMA1_CHAN3, 0)
#  define DMACHAN_SPI1_TX_2       DMACHAN_SETTING(STM32_DMA2_CHAN4, 4)

#  define DMACHAN_SPI2_RX         DMACHAN_SETTING(STM32_DMA1_CHAN4, 1)
#  define DMACHAN_SPI2_TX         DMACHAN_SETTING(STM32_DMA1_CHAN5, 1)

#  define DMACHAN_SPI3_RX         DMACHAN_SETTING(STM32_DMA2_CHAN1, 3)
#  define DMACHAN_SPI3_TX         DMACHAN_SETTING(STM32_DMA2_CHAN2, 3)

/* TIM */

#  define DMACHAN_TIM1_CH1        DMACHAN_SETTING(STM32_DMA1_CHAN2, 7)
#  define DMACHAN_TIM1_CH2        DMACHAN_SETTING(STM32_DMA1_CHAN3, 7)
#  define DMACHAN_TIM1_CH3        DMACHAN_SETTING(STM32_DMA1_CHAN7, 7)
#  define DMACHAN_TIM1_CH4        DMACHAN_SETTING(STM32_DMA1_CHAN4, 7)
#  define DMACHAN_TIM1_COM        DMACHAN_SETTING(STM32_DMA1_CHAN4, 7)
#  define DMACHAN_TIM1_TRIG       DMACHAN_SETTING(STM32_DMA1_CHAN4, 7)
#  define DMACHAN_TIM1_UP         DMACHAN_SETTING(STM32_DMA1_CHAN6, 7)

#  define DMACHAN_TIM2_CH1        DMACHAN_SETTING(STM32_DMA1_CHAN5, 4)
#  define DMACHAN_TIM2_CH2        DMACHAN_SETTING(STM32_DMA1_CHAN7, 4)
#  define DMACHAN_TIM2_CH3        DMACHAN_SETTING(STM32_DMA1_CHAN1, 4)
#  define DMACHAN_TIM2_CH4        DMACHAN_SETTING(STM32_DMA1_CHAN7, 4)
#  define DMACHAN_TIM2_UP         DMACHAN_SETTING(STM32_DMA1_CHAN2, 4)

#  define DMACHAN_TIM3_CH1        DMACHAN_SETTING(STM32_DMA1_CHAN6, 5)
#  define DMACHAN_TIM3_CH3        DMACHAN_SETTING(STM32_DMA1_CHAN2, 5)
#  define DMACHAN_TIM3_CH4        DMACHAN_SETTING(STM32_DMA1_CHAN3, 5)
#  define DMACHAN_TIM3_TRIG       DMACHAN_SETTING(STM32_DMA1_CHAN6, 5)
#  define DMACHAN_TIM3_UP         DMACHAN_SETTING(STM32_DMA1_CHAN3, 5)

#  define DMACHAN_TIM4_CH1        DMACHAN_SETTING(STM32_DMA1_CHAN1, 6)
#  define DMACHAN_TIM4_CH2        DMACHAN_SETTING(STM32_DMA1_CHAN4, 6)
#  define DMACHAN_TIM4_CH3        DMACHAN_SETTING(STM32_DMA1_CHAN5, 6)
#  define DMACHAN_TIM4_UP         DMACHAN_SETTING(STM32_DMA1_CHAN7, 6)

#  define DMACHAN_TIM5_CH1        DMACHAN_SETTING(STM32_DMA2_CHAN5, 5)
#  define DMACHAN_TIM5_CH2        DMACHAN_SETTING(STM32_DMA2_CHAN4, 5)
#  define DMACHAN_TIM5_CH3        DMACHAN_SETTING(STM32_DMA2_CHAN2, 5)
#  define DMACHAN_TIM5_CH4        DMACHAN_SETTING(STM32_DMA2_CHAN1, 5)
#  define DMACHAN_TIM5_COM        DMACHAN_SETTING(STM32_DMA2_CHAN1, 5)
#  define DMACHAN_TIM5_TRIG       DMACHAN_SETTING(STM32_DMA2_CHAN1, 5)
#  define DMACHAN_TIM5_UP         DMACHAN_SETTING(STM32_DMA2_CHAN2, 5)

#  define DMACHAN_TIM6_UP_1       DMACHAN_SETTING(STM32_DMA1_CHAN3, 6)
#  define DMACHAN_TIM6_UP_2       DMACHAN_SETTING(STM32_DMA2_CHAN4, 3)

#  define DMACHAN_TIM7_UP_1       DMACHAN_SETTING(STM32_DMA1_CHAN4, 5)
#  define DMACHAN_TIM7_UP_2       DMACHAN_SETTING(STM32_DMA2_CHAN5, 3)

#  define DMACHAN_TIM8_CH1        DMACHAN_SETTING(STM32_DMA2_CHAN6, 7)
#  define DMACHAN_TIM8_CH2        DMACHAN_SETTING(STM32_DMA2_CHAN7, 7)
#  define DMACHAN_TIM8_CH3        DMACHAN_SETTING(STM32_DMA2_CHAN1, 7)
#  define DMACHAN_TIM8_CH4        DMACHAN_SETTING(STM32_DMA2_CHAN2, 7)
#  define DMACHAN_TIM8_COM        DMACHAN_SETTING(STM32_DMA2_CHAN2, 7)
#  define DMACHAN_TIM8_TRIG       DMACHAN_SETTING(STM32_DMA2_CHAN2, 7)
#  define DMACHAN_TIM8_UP         DMACHAN_SETTING(STM32_DMA2_CHAN1, 7)

#  define DMACHAN_TIM15_CH1       DMACHAN_SETTING(STM32_DMA1_CHAN5, 7)
#  define DMACHAN_TIM15_COM       DMACHAN_SETTING(STM32_DMA1_CHAN5, 7)
#  define DMACHAN_TIM15_TRIG      DMACHAN_SETTING(STM32_DMA1_CHAN5, 7)
#  define DMACHAN_TIM15_UP        DMACHAN_SETTING(STM32_DMA1_CHAN5, 7)

#  define DMACHAN_TIM16_CH1_1     DMACHAN_SETTING(STM32_DMA1_CHAN3, 4)
#  define DMACHAN_TIM16_CH1_2     DMACHAN_SETTING(STM32_DMA1_CHAN6, 4)
#  define DMACHAN_TIM16_UP_1      DMACHAN_SETTING(STM32_DMA1_CHAN3, 4)
#  define DMACHAN_TIM16_UP_2      DMACHAN_SETTING(STM32_DMA1_CHAN6, 4)

#  define DMACHAN_TIM17_CH1_1     DMACHAN_SETTING(STM32_DMA1_CHAN1, 5)
#  define DMACHAN_TIM17_CH1_2     DMACHAN_SETTING(STM32_DMA1_CHAN7, 5)
#  define DMACHAN_TIM17_UP_1      DMACHAN_SETTING(STM32_DMA1_CHAN1, 5)
#  define DMACHAN_TIM17_UP_2      DMACHAN_SETTING(STM32_DMA1_CHAN7, 5)

/* USARTs */

#  define DMACHAN_USART1_RX_1     DMACHAN_SETTING(STM32_DMA1_CHAN5, 2)
#  define DMACHAN_USART1_RX_2     DMACHAN_SETTING(STM32_DMA2_CHAN7, 2)
#  define DMACHAN_USART1_TX_1     DMACHAN_SETTING(STM32_DMA1_CHAN4, 2)
#  define DMACHAN_USART1_TX_2     DMACHAN_SETTING(STM32_DMA2_CHAN6, 2)

#  define DMACHAN_USART2_RX       DMACHAN_SETTING(STM32_DMA1_CHAN6, 2)
#  define DMACHAN_USART2_TX       DMACHAN_SETTING(STM32_DMA1_CHAN7, 2)

#  define DMACHAN_USART3_RX       DMACHAN_SETTING(STM32_DMA1_CHAN3, 1)
#  define DMACHAN_USART3_TX       DMACHAN_SETTING(STM32_DMA1_CHAN2, 2)

#  define DMACHAN_USART4_RX       DMACHAN_SETTING(STM32_DMA2_CHAN5, 2)
#  define DMACHAN_USART4_TX       DMACHAN_SETTING(STM32_DMA2_CHAN3, 2)

#  define DMACHAN_USART5_RX       DMACHAN_SETTING(STM32_DMA2_CHAN2, 2)
#  define DMACHAN_USART5_TX       DMACHAN_SETTING(STM32_DMA2_CHAN1, 2)

#elif defined(CONFIG_ARCH_CHIP_STM32L0)

/* ADC */

#  define DMACHAN_ADC1_1          DMACHAN_SETTING(STM32_DMA1_CHAN1, 0)
#  define DMACHAN_ADC1_2          DMACHAN_SETTING(STM32_DMA1_CHAN2, 0)

/* AES */

#  define DMACHAN_AES_IN_1        DMACHAN_SETTING(STM32_DMA1_CHAN1, 11)
#  define DMACHAN_AES_IN_2        DMACHAN_SETTING(STM32_DMA1_CHAN5, 11)
#  define DMACHAN_AES_OUT_1       DMACHAN_SETTING(STM32_DMA1_CHAN2, 11)
#  define DMACHAN_AES_OUT_2       DMACHAN_SETTING(STM32_DMA1_CHAN3, 11)

/* DAC */

#  define DMACHAN_DAC1CH1         DMACHAN_SETTING(STM32_DMA1_CHAN2, 9)
#  define DMACHAN_DAC1CH2         DMACHAN_SETTING(STM32_DMA1_CHAN4, 15)

/* I2C */

#  define DMACHAN_I2C1_TX_1       DMACHAN_SETTING(STM32_DMA1_CHAN2, 6)
#  define DMACHAN_I2C1_TX_2       DMACHAN_SETTING(STM32_DMA1_CHAN6, 6)
#  define DMACHAN_I2C1_RX_1       DMACHAN_SETTING(STM32_DMA1_CHAN3, 6)
#  define DMACHAN_I2C1_RX_2       DMACHAN_SETTING(STM32_DMA1_CHAN7, 6)
#  define DMACHAN_I2C2_TX         DMACHAN_SETTING(STM32_DMA1_CHAN4, 7)
#  define DMACHAN_I2C2_RX         DMACHAN_SETTING(STM32_DMA1_CHAN7, 7)
#  define DMACHAN_I2C3_TX_1       DMACHAN_SETTING(STM32_DMA1_CHAN2, 14)
#  define DMACHAN_I2C3_TX_2       DMACHAN_SETTING(STM32_DMA1_CHAN4, 14)
#  define DMACHAN_I2C3_RX_1       DMACHAN_SETTING(STM32_DMA1_CHAN3, 14)
#  define DMACHAN_I2C3_RX_2       DMACHAN_SETTING(STM32_DMA1_CHAN5, 14)

/* SPI */

#  define DMACHAN_SPI1_RX         DMACHAN_SETTING(STM32_DMA1_CHAN2, 1)
#  define DMACHAN_SPI1_TX         DMACHAN_SETTING(STM32_DMA1_CHAN3, 1)
#  define DMACHAN_SPI2_RX_1       DMACHAN_SETTING(STM32_DMA1_CHAN4, 2)
#  define DMACHAN_SPI2_RX_2       DMACHAN_SETTING(STM32_DMA1_CHAN6, 2)
#  define DMACHAN_SPI2_TX_1       DMACHAN_SETTING(STM32_DMA1_CHAN5, 2)
#  define DMACHAN_SPI2_TX_2       DMACHAN_SETTING(STM32_DMA1_CHAN7, 2)

/* TIM */

#  define DMACHAN_TIM2_CH1        DMACHAN_SETTING(STM32_DMA1_CHAN5, 8)
#  define DMACHAN_TIM2_CH2_1      DMACHAN_SETTING(STM32_DMA1_CHAN3, 8)
#  define DMACHAN_TIM2_CH2_2      DMACHAN_SETTING(STM32_DMA1_CHAN7, 8)
#  define DMACHAN_TIM2_CH3        DMACHAN_SETTING(STM32_DMA1_CHAN1, 8)
#  define DMACHAN_TIM2_CH4_1      DMACHAN_SETTING(STM32_DMA1_CHAN4, 8)
#  define DMACHAN_TIM2_CH4_2      DMACHAN_SETTING(STM32_DMA1_CHAN7, 8)
#  define DMACHAN_TIM2_UP         DMACHAN_SETTING(STM32_DMA1_CHAN2, 8)
#  define DMACHAN_TIM6_UP         DMACHAN_SETTING(STM32_DMA1_CHAN2, 9)
#  define DMACHAN_TIM7_UP         DMACHAN_SETTING(STM32_DMA1_CHAN4, 15)

/* USART */

#  define DMACHAN_USART1_TX_1     DMACHAN_SETTING(STM32_DMA1_CHAN2, 3)
#  define DMACHAN_USART1_TX_2     DMACHAN_SETTING(STM32_DMA1_CHAN4, 3)
#  define DMACHAN_USART1_RX_1     DMACHAN_SETTING(STM32_DMA1_CHAN3, 3)
#  define DMACHAN_USART1_RX_2     DMACHAN_SETTING(STM32_DMA1_CHAN5, 3)
#  define DMACHAN_USART2_TX_1     DMACHAN_SETTING(STM32_DMA1_CHAN4, 4)
#  define DMACHAN_USART2_TX_2     DMACHAN_SETTING(STM32_DMA1_CHAN7, 4)
#  define DMACHAN_USART2_RX_1     DMACHAN_SETTING(STM32_DMA1_CHAN5, 4)
#  define DMACHAN_USART2_RX_2     DMACHAN_SETTING(STM32_DMA1_CHAN6, 4)
#  define DMACHAN_LPUART1_TX_1    DMACHAN_SETTING(STM32_DMA1_CHAN2, 5)
#  define DMACHAN_LPUART1_TX_2    DMACHAN_SETTING(STM32_DMA1_CHAN7, 5)
#  define DMACHAN_LPUART1_RX_1    DMACHAN_SETTING(STM32_DMA1_CHAN3, 5)
#  define DMACHAN_LPUART1_RX_2    DMACHAN_SETTING(STM32_DMA1_CHAN6, 5)
#  define DMACHAN_USART4_RX_1     DMACHAN_SETTING(STM32_DMA1_CHAN2, 12)
#  define DMACHAN_USART4_RX_2     DMACHAN_SETTING(STM32_DMA1_CHAN6, 12)
#  define DMACHAN_USART4_TX_1     DMACHAN_SETTING(STM32_DMA1_CHAN7, 12)
#  define DMACHAN_USART4_TX_2     DMACHAN_SETTING(STM32_DMA1_CHAN3, 12)
#  define DMACHAN_USART5_RX_1     DMACHAN_SETTING(STM32_DMA1_CHAN2, 13)
#  define DMACHAN_USART5_RX_2     DMACHAN_SETTING(STM32_DMA1_CHAN6, 13)
#  define DMACHAN_USART5_TX_1     DMACHAN_SETTING(STM32_DMA1_CHAN7, 13)
#  define DMACHAN_USART5_TX_2     DMACHAN_SETTING(STM32_DMA1_CHAN3, 13)

#elif defined(CONFIG_ARCH_CHIP_STM32G0) || defined(CONFIG_ARCH_CHIP_STM32C0)
/* This family uses a DMAMUX */

#else
#  error "Unknown DMA channel assignments"
#endif

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_DMA_V1_H */
