/*
 * app_bluenrg.c
 *
 *  Created on: 16-Dec-2025
 *      Author: Rajeev
 */

#include <app_includes.h>

/*
 * [ b0 ][ b1 ][ b2 ][ b3 ][ b4 ][ b5 ]
 *
 *    b0 is the Least Significant Byte (LSB)
 *
 *    b5 is the Most Significant Byte (MSB)
 *
 * The two least-significant bits of b0 define the address type:
 * b0 bit1:bit0	Meaning
 * 00	Public address (OUI-based)
 * 01	Random static
 * 10	Private resolvable
 * 11	Private non-resolvable
 */
uint8_t SERVER_BADDR[CONFIG_DATA_PUBADDR_LEN] =
{
  0xC0, 0xFF, 0xEE, 0xC0, 0xFF, 0xEE
};

tBleStatus bluenrg_init(void)
{
	tBleStatus ret = BLE_STATUS_SUCCESS;
	uint8_t bdaddr[CONFIG_DATA_PUBADDR_LEN];

	const char * BLE_Name = "BNRG";
	const uint8_t BLE_NameLength = strlen( BLE_Name );

  uint16_t service_handle;
  uint16_t device_name_char_handle;
  uint16_t appearance_char_handle;

	do
	{
		BLUENRG_memcpy(bdaddr, SERVER_BADDR, CONFIG_DATA_PUBADDR_LEN);

    /* ------------------------------------------------------------------
     * Initialise HCI
     *
     * App_UserEvtRx is REQUIRED only when at least one BLE service exists.
     * Otherwise pass NULL to avoid dangling symbol dependency.
     * ------------------------------------------------------------------ */
		/* Initialise HCI */
		extern void App_UserEvtRx(void *pData);
		hci_init(App_UserEvtRx, NULL);

		/* Reset HCI */
		ret = hci_reset();
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("hci_reset : FAILED (%d)", ret);
			break;
		}

		/* check reset complete */
		/* DO NOT REMOVE: allow controller firmware to settle */
    /*
     * Pump BLE stack for 100 ms
     *
     * Needed because:
     * - BLE controller firmware has just been reset
     * - Internal BlueNRG state machines are not ready yet
     * - Give the controller time to settle and drain mandatory HCI Events like
     *   reset complete, command complete, etc., must be drained
     * - This is intentional busy pumping
     */
		uint32_t start = HAL_GetTick();
    while (100U >= (HAL_GetTick() - start))
		{
			/* Keep BLE event processing alive. Do not exit delay based on the function's return value. */
		    (void)hci_user_evt_proc();
		}

		/* Configure device address */
		ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_hal_write_config_data : FAILED (%d)", ret);
			break;
		}

		/* Initialise GATT server */
		ret = aci_gatt_init();
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_init : FAILED (%d)", ret);
			break;
		}

		/* Initialise GAP server */
    ret = aci_gap_init(GAP_PERIPHERAL_ROLE, PRIVACY_DISABLED, DEVICE_NAME_LEN, &service_handle, &device_name_char_handle, &appearance_char_handle);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gap_init : FAILED (%d)", ret);
			break;
		}

		/* Update device name characteristic value */
		/* If this is set to 0 and the attribute value is of variable length. */
		const uint8_t CurrentOffset = 0;
		ret = aci_gatt_update_char_value(service_handle, device_name_char_handle, CurrentOffset, BLE_NameLength, (uint8_t *)BLE_Name);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_update_char_value : FAILED (%d)", ret);
			break;
		}
		/* Add custom service */
		ret = add_services();
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("add_services : FAILED (%d)", ret);
			break;
		}
	}while( false );

	return ret;
}

// Earlier name bluenrg_process
tBleStatus bluenrg_start_advertising( void )
{
	tBleStatus ret = BLE_STATUS_SUCCESS;

	do
	{
		const char LocalProjectName[] = "BNRG_ADV";
    const size_t LocalProjectNameLength = strlen(LocalProjectName);

		uint8_t LocalName[LocalProjectNameLength + 1];

    /* Checking max size of ADV type info (byte) and the ADV data length that can be safely tarnsmitted. */
    assert_param(18U >= (LocalProjectNameLength + 1));

		LocalName[0] = AD_TYPE_COMPLETE_LOCAL_NAME;
		BLUENRG_memcpy((LocalName + 1), LocalProjectName, LocalProjectNameLength);
		/* Length of the Service UUID List in octets. If there is no service to
		 * be advertised, set this field to 0x00. */
		const uint8_t ServiceUuidLength = 0;
		const uint8_t * const pServiceUuidList = (const uint8_t *)NULL;

		/* Set device in General Discoverable mode */
		ret = aci_gap_set_discoverable(ADV_DATA_TYPE, ADV_INTERV_MIN, ADV_INTERV_MAX, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(LocalName), LocalName, ServiceUuidLength, ( uint8_t *)pServiceUuidList, L2CAP_INTERV_MIN, L2CAP_INTERV_MAX);
//	ret = aci_gap_set_discoverable(ADV_DATA_TYPE, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE, sizeof(LocalName), LocalName, ServiceUuidLength, ( uint8_t *)pServiceUuidList, 0, 0);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gap_set_discoverable : FAILED (%d)", ret);
			break;
		}
	} while( false );
	return ret;
}
