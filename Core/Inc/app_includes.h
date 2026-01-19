/*
 * app_includes.h
 *
 *  Created on: 16-Dec-2025
 *      Author: Rajeev
 */

#ifndef INC_APP_INCLUDES_H_
#define INC_APP_INCLUDES_H_

/* ============================================================================
 * Compilation control (feature flags, modes, etc.)
 * ==========================================================================*/
#include "app_compilation_macros.h"

/* ============================================================================
 * Standard C library
 * ==========================================================================*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /* NULL */
#include <string.h>
#include <stdio.h>
#include <errno.h>

/* ============================================================================
 * STM32 HAL + CubeMX project headers
 * ==========================================================================*/
#include <main.h>

/* ============================================================================
 * BlueNRG / BLE stack
 * ==========================================================================*/
#include <ble_status.h>
#include <bluenrg_conf.h>

#include <hci.h>
#include <hci_const.h>
#include <hci_tl.h>

#include <bluenrg1_types.h>
#include <bluenrg1_gap.h>
#include <bluenrg1_gatt_aci.h>
#include <bluenrg1_aci.h>
#include <bluenrg1_hci_le.h>

/* ============================================================================
 * Application infrastructure
 * ==========================================================================*/
#include "app_debug.h"
#include "app_LED_report_error.h"

/* ============================================================================
 * Application modules
 * ==========================================================================*/
#include <app_bluenrg.h>
#include <app_services.h>

#endif /* INC_APP_INCLUDES_H_ */
