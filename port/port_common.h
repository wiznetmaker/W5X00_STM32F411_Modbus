/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PORT_COMMON_H_
#define _PORT_COMMON_H_

/**
  * ----------------------------------------------------------------------------------------------------
  * Includes
  * ----------------------------------------------------------------------------------------------------
  */
/* Common */
#include "main.h"
#include "stdbool.h"

#define MODBUS_RTU   0
#define MODBUS_ASCII 1

#define MODBUS_PROTOCOL MODBUS_RTU

#if !defined(MAX)
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#if !defined(MIN)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

enum {
    FALSE = 0,
    TRUE
};

#endif /* _PORT_COMMON_H_ */
