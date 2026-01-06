/*
 * app_bluenrg.h
 *
 *  Created on: 17-Dec-2025
 *      Author: Rajeev
 */

#ifndef INC_APP_BLUENRG_H_
#define INC_APP_BLUENRG_H_

//#define HCI_RESET_OPCODE		cmd_opcode_pack(OGF_HOST_CTL, OCF_RESET)

#define INVALID_CONNECTION_HANDLE  ( 0xFFFF )

extern volatile uint16_t connection_handle;

extern tBleStatus bluenrg_init( void );
extern tBleStatus bluenrg_start_advertising( void );
extern void bluenrg_restart_advertising(void);

#endif /* INC_APP_BLUENRG_H_ */
