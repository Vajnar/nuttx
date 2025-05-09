/****************************************************************************
 * arch/sparc/src/bm3823/bm3823-memorymap.h
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

#ifndef __ARCH_SPARC_SRC_BM3823_BM3823_MEMORYMAP_H
#define __ARCH_SPARC_SRC_BM3823_BM3823_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#  define  BM3823_INTER_REG_BASE      0x80000000
#  define  BM3823_EXTER_REG_BASE      0x81000000

/* UART 1-3 Register Base Addresses */
#  define BM3823_UART1_BASE      (BM3823_EXTER_REG_BASE + 0x70)
#  define BM3823_UART2_BASE      (BM3823_EXTER_REG_BASE + 0x80)
#  define BM3823_UART3_BASE      (BM3823_EXTER_REG_BASE + 0x1C0)
#  define BM3823_UART4_BASE      (BM3823_EXTER_REG_BASE + 0x1E0)

#endif /* __ARCH_SPARC_SRC_BM3823_BM3823_MEMORYMAP_H */

