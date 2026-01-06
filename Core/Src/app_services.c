/*
 * app_service.c
 *
 *  Created on: 20-Dec-2025
 *      Author: Rajeev
 */

#include "app_includes.h"

#define DEF_DATA_TX_CHAR_VALUE_LENGTH				( 20 )
#define DEF_CONTROL_RX_CHAR_VALUE_LENGTH		( 20 )

static const uint16_t DATA_TX_CHAR_VALUE_LENGTH = DEF_DATA_TX_CHAR_VALUE_LENGTH;
static const uint16_t CONTROL_RX_CHAR_VALUE_LENGTH = DEF_CONTROL_RX_CHAR_VALUE_LENGTH;

const uint16_t TEST_BPM_SENSOR_DATA					=	 80;
const uint16_t TEST_WEIGHT_SENSOR_DATA			=	 75;
const uint16_t TEST_TEMPERATURE_SENSOR_DATA	=	 17;
const uint16_t TEST_HUMIDITY_SENSOR_DATA		=	 48;

/* If this BLE is a part of a digital watch
 * that runs health service and weather service,
 * then number of services are 2.
 */

/* If this BLE's  has two characteristics:
 * 1. Heart rate (in Beats per Minute)
 * 2. Weight (in Kg)
 * then number of characteristics are 2 for Health Service.
 */

/* If this BLE's Weather Service has two characteristics:
 * 1. Temperature
 * 2. Humidity
 * then number of characteristics are 2 for Weather Service.
 */

/* 128-bit Health Service UUID in little-endian format (BlueNRG requirement) */
const uint8_t HEALTH_SERVICE_UUID[16] 					= { 0x39, 0xea, 0x83, 0x31, 0xa4, 0x1e, 0x4c, 0xbf, 0xa5, 0x99, 0x5a, 0xfc, 0x3e, 0xd2, 0x68, 0x51 };

/* 128-bit BPM Characteristic UUID (derived from Health Service UUID, little-endian, with byte[12] incremented by 1) */
const uint8_t HEALTH_BPM_CHAR_UUID[16]					= { 0x39, 0xea, 0x83, 0x31, 0xa4, 0x1e, 0x4c, 0xbf, 0xa5, 0x99, 0x5a, 0xfc, (HEALTH_SERVICE_UUID[12] + 1), 0xd2, 0x68, 0x51 };

/* 128-bit Weight Characteristic UUID (derived from Health Service UUID, little-endian, with byte[12] incremented by 2) */
const uint8_t HEALTH_WEIGHT_CHAR_UUID[16] 			= { 0x39, 0xea, 0x83, 0x31, 0xa4, 0x1e, 0x4c, 0xbf, 0xa5, 0x99, 0x5a, 0xfc, (HEALTH_SERVICE_UUID[12] + 2), 0xd2, 0x68, 0x51 };

/* 128-bit Health Data Tx Characteristic UUID (derived from Health Service UUID, little-endian, with byte[12] incremented by 2) */
const uint8_t HEALTH_DATA_TX_CHAR_UUID[16] 			= { 0x39, 0xea, 0x83, 0x31, 0xa4, 0x1e, 0x4c, 0xbf, 0xa5, 0x99, 0x5a, 0xfc, (HEALTH_SERVICE_UUID[12] + 3), 0xd2, 0x68, 0x51 };

/* 128-bit Health Control Rx Characteristic UUID (derived from Health Service UUID, little-endian, with byte[12] incremented by 2) */
const uint8_t HEALTH_CONTROL_RX_CHAR_UUID[16] 	= { 0x39, 0xea, 0x83, 0x31, 0xa4, 0x1e, 0x4c, 0xbf, 0xa5, 0x99, 0x5a, 0xfc, (HEALTH_SERVICE_UUID[12] + 4), 0xd2, 0x68, 0x51 };

/* 128-bit Weather Service UUID in little-endian format (BlueNRG requirement) */
const uint8_t WEATHER_SERVICE_UUID[16]					= { 0x8a, 0x7e, 0x84, 0xfd, 0x78, 0x6f, 0x43, 0x48, 0xa8, 0xc4, 0x46, 0x70, 0xf3, 0x39, 0xfb, 0x72 };

/* 128-bit Temperature Characteristic UUID (derived from Weather Service UUID, little-endian, with byte[12] incremented by 1) */
const uint8_t WEATHER_TEMPERATURE_CHAR_UUID[16]	= { 0x8a, 0x7e, 0x84, 0xfd, 0x78, 0x6f, 0x43, 0x48, 0xa8, 0xc4, 0x46, 0x70, (WEATHER_SERVICE_UUID[12] + 1), 0x39, 0xfb, 0x72 };

/* 128-bit Humidity Characteristic UUID (derived from Weather Service UUID, little-endian, with byte[12] incremented by 2) */
const uint8_t WEATHER_HUMIDITY_CHAR_UUID[16]		= { 0x8a, 0x7e, 0x84, 0xfd, 0x78, 0x6f, 0x43, 0x48, 0xa8, 0xc4, 0x46, 0x70, (WEATHER_SERVICE_UUID[12] + 2), 0x39, 0xfb, 0x72 };

uint16_t health_service_handle;
uint16_t health_bpm_char_handle;
uint16_t health_weight_char_handle;
uint16_t health_data_tx_char_handle;
uint16_t health_control_rx_char_handle;

uint16_t weather_service_handle;
uint16_t weather_temperature_char_handle;
uint16_t weather_humidity_char_handle;

/* ---- START: BLE connection tracking variables ---- */

/* Current BLE connection handle */
/* NOTE:
 * BlueNRG-2 supports only a single active BLE connection.
 * The application therefore tracks connection state globally
 * using connection_handle instead of per-connection context.
 */
volatile uint16_t connection_handle = INVALID_CONNECTION_HANDLE;  /* invalid when not connected */

/* ---- END:: BLE connection tracking variables ---- */

/* Assigned by the BLE controller. Valid only while a connection exists. */
bool notification_enabled = false;

static uint32_t g_last_btn_tick = (uint32_t)(0);
volatile bool g_btn_event = false;   /* One clean event */
/* Global scope flag */
volatile bool g_restart_adv = false;

/*
 * Validate parameters before calling aci_gatt_add_char().
 *
 * Notes:
 * - aci_gatt_add_char() returns generic error codes
 * - Caller-side validation is required to identify faulty parameters
 * - X-CUBE-NRG2 does not expose max Char_Value_Length as a macro
 */

/* As per X-CUBE-NRG2 documentation/comments:
 * Char_Value_Length valid range: 1 .. 512
 */
static const uint16_t g_max_char_value_length = 512U;

/* --------------------------------------------------------------------
 * Compile-time validation of characteristic value lengths
 *
 * These values are design-time constants.
 * Exceeding the maximum allowed GATT value length is a build-time error.
 * -------------------------------------------------------------------- */
#if (DATA_TX_CHAR_VALUE_LENGTH > g_max_char_value_length)
  #error "DATA_TX_CHAR_VALUE_LENGTH exceeds maximum allowed GATT value length (512)"
#endif

#if (CONTROL_RX_CHAR_VALUE_LENGTH > g_max_char_value_length)
  #error "CONTROL_RX_CHAR_VALUE_LENGTH exceeds maximum allowed GATT value length (512)"
#endif

/* Last successfully received control RX length (0 = no valid data) */
static uint16_t g_health_control_rx_len = 0;

static tBleStatus validate_add_char_params(
  uint8_t        uuid_type,
  const uint8_t *uuid,
  uint16_t       char_len,
  uint8_t        properties,
  uint8_t        permissions,
  uint8_t        min_key_size,
  bool           is_variable
)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  do
  {
    /* UUID type */
    if( (UUID_TYPE_16 != uuid_type) && (UUID_TYPE_128 != uuid_type) )
    {
      LOG_WARN("add_char: invalid UUID type (%u)", uuid_type);
      ret = BLE_STATUS_INVALID_PARAMS;
      break;
    }

    /* UUID pointer */
    if( NULL == uuid )
    {
      LOG_WARN("add_char: UUID pointer is NULL");
      ret = BLE_STATUS_NULL_PARAM;
      break;
    }

    /* Properties must not be zero */
    if( 0U == properties )
    {
      LOG_WARN("add_char: properties is zero");
      ret = BLE_STATUS_INVALID_PARAMS;
      break;
    }

		 /* --------------------------------------------------------------------
		 * Characteristic length vs variable flag validation
		 *
		 * Rules:
		 * - Fixed-length characteristic (is_variable == false):
		 *     char_len is typically sizeof(value)
		 *     MUST be non-zero and <= max allowed length
		 *
		 * - Variable-length characteristic (is_variable == true):
		 *     char_len represents MAX allowed value length
		 *     MUST be non-zero and <= max allowed length
		 * -------------------------------------------------------------------- */
    if( 0U == char_len )
    {
		  LOG_WARN("add_char: %s-length characteristic with zero length", ( false == is_variable ) ? "fixed" : "variable");
      ret = BLE_STATUS_INVALID_PARAMS;
      break;
    }

    if( g_max_char_value_length < char_len )
    {
      LOG_WARN("add_char: Char_Value_Length too large (%u > %u)", char_len, g_max_char_value_length);
      ret = BLE_STATUS_INVALID_PARAMS;
      break;
    }

    /*
     * IMPORTANT:
     * ATTR_PERMISSION_NONE is VALID and NORMAL.
     * Do NOT reject READ / WRITE characteristics because of it.
     */

    /* --------------------------------------------------------------------
     * Encryption key size validation
     *
     * Rules:
     * - If encryption IS required:
     *     min_key_size MUST be within [MIN_ENCRY_KEY_SIZE .. MAX_ENCRY_KEY_SIZE]
     *
     * - If encryption is NOT required (ATTR_PERMISSION_NONE):
     *     min_key_size is ignored by the stack
     *     0 is the recommended value
     * -------------------------------------------------------------------- */
    if( ATTR_PERMISSION_NONE != permissions )
    {
	    /* Minimum encryption key size — lower bound */
      if( MIN_ENCRY_KEY_SIZE > min_key_size )
      {
        LOG_WARN("add_char: min_key_size too small (%u < %u)", min_key_size, MIN_ENCRY_KEY_SIZE);
        ret = BLE_INSUFFICIENT_ENC_KEYSIZE;
        break;
      }

	    /* Minimum encryption key size — upper bound (BLE spec max = 16) */
      if( MAX_ENCRY_KEY_SIZE < min_key_size )
      {
        LOG_WARN("add_char: min_key_size too large (%u > %u)", min_key_size, MAX_ENCRY_KEY_SIZE);
        ret = BLE_STATUS_INVALID_PARAMS;
        break;
      }
    }
    else
    {
      if( 0U != min_key_size )
      {
        LOG_WARN("add_char: min_key_size ignored because ATTR_PERMISSION_NONE is set (%u)", min_key_size);
      }
    }
  } while( false );

  return ret;
}

/* Maximum reasonable attribute records per single service.
 * This is a sanity limit to catch configuration bugs, not a stack limit.
 */
static const uint8_t g_max_service_attribute_records = 20U;

static tBleStatus validate_add_service_params(
  uint8_t              uuid_type,
  const Service_UUID_t *uuid,
  uint8_t              service_type,
  uint8_t              max_attribute_records,
  const uint16_t       *service_handle
)
{
  /* UUID type */
  if( (UUID_TYPE_16 != uuid_type) && (UUID_TYPE_128 != uuid_type) )
  {
    LOG_WARN("add_service: invalid UUID type (%u)", uuid_type);
    return BLE_STATUS_INVALID_PARAMS;
  }

  /* UUID pointer */
  if( NULL == uuid )
  {
    LOG_WARN("add_service: UUID pointer is NULL");
    return BLE_STATUS_NULL_PARAM;
  }

  /* Service type */
  if( (PRIMARY_SERVICE != service_type) && (SECONDARY_SERVICE != service_type) )
  {
    LOG_WARN("add_service: invalid service type (%u)", service_type);
    return BLE_STATUS_INVALID_PARAMS;
  }

  /* Attribute record count */
  if( 0U == max_attribute_records )
  {
    LOG_WARN("add_service: Max_Attribute_Records is zero");
    return BLE_STATUS_INVALID_PARAMS;
  }

  if( (PRIMARY_SERVICE == service_type) && (max_attribute_records < 2U) )
  {
    LOG_WARN("add_service: PRIMARY_SERVICE requires at least 2 attribute records");
    return BLE_STATUS_INVALID_PARAMS;
  }

  if( max_attribute_records > g_max_service_attribute_records )
  {
    LOG_WARN("add_service: Max_Attribute_Records too large (%u > %u)",
             max_attribute_records,
             g_max_service_attribute_records);
    return BLE_STATUS_INVALID_PARAMS;
  }

  /* Service handle pointer */
  if( NULL == service_handle )
  {
    LOG_WARN("add_service: service_handle pointer is NULL");
    return BLE_STATUS_NULL_PARAM;
  }

  return BLE_STATUS_SUCCESS;
}

tBleStatus add_services(void)
{
	tBleStatus ret;
	do
	{
		Service_UUID_t	health_service_uuid;
		Char_UUID_t			health_bpm_char_uuid;
		Char_UUID_t			health_weight_char_uuid;
		Char_UUID_t			health_data_tx_char_uuid;
		Char_UUID_t			health_control_rx_char_uuid;

		Service_UUID_t	weather_service_uuid;
		Char_UUID_t			weather_temperature_char_uuid;
		Char_UUID_t			weather_humidity_char_uuid;

		/* Add health service */
		BLUENRG_memcpy(health_service_uuid.Service_UUID_128, HEALTH_SERVICE_UUID, sizeof(HEALTH_SERVICE_UUID));

		/*
		 * ============================================================================
		 * GATT Attribute Record Accounting (BlueNRG-2 / X-CUBE-BLE2)
		 * ============================================================================
		 *
		 * - Max_Attribute_Records is NOT a global pool for the entire GATT database.
		 * - It is specified PER SERVICE while calling aci_gatt_add_service().
		 * - Each service must reserve enough attribute records to cover
		 *   ALL attributes belonging to THAT service only.
		 *
		 * Attribute record usage (record COUNT, not byte size):
		 *
		 *   • Primary Service declaration                 : 1 record
		 *
		 *   • Characteristic Declaration                  : 1 record (always)
		 *   • Characteristic Value                        : 1 record (always)
		 *
		 *   • READ characteristic                         : 2 records
		 *       (Declaration + Value)
		 *
		 *   • WRITE / WRITE_NO_RESP characteristic         : 2 records
		 *       (Declaration + Value)
		 *
		 *   • NOTIFY or INDICATE characteristic            : 3 records
		 *       (Declaration + Value + CCCD)
		 *
		 *   • READ + NOTIFY / READ + INDICATE              : 3 records
		 *       (Declaration + Value + CCCD)
		 *
		 *   • Characteristic User Description (optional)   : +1 record
		 *
		 * Notes:
		 *
		 * - Char_Value_Length specifies the MAX size (in bytes) of the
		 *   Characteristic VALUE attribute only.
		 *
		 * - For NOTIFY / INDICATE, the BlueNRG-2 stack automatically creates a
		 *   Client Characteristic Configuration Descriptor (CCCD) during
		 *   aci_gatt_add_char().
		 *
		 * - CCCD is a SEPARATE attribute (fixed 2 bytes) and consumes
		 *   one additional attribute record; it is NOT part of the
		 *   characteristic value.
		 *
		 * - Although a NOTIFY characteristic involves:
		 *   Char_Value_Length bytes (value) + 2 bytes (CCCD),
		 *   these belong to DIFFERENT attributes and must NOT be combined
		 *   while selecting Char_Value_Length.
		 *
		 * - Example (for clarity):
		 *     1 service containing 1 NOTIFY characteristic
		 *       ⇒ 1 (service) + 3 (notify characteristic) = 4 attribute records total.
		 *
		 * - If Max_Attribute_Records for a service is undersized, GATT additions
		 *   may fail silently (for example, missing characteristics or an
		 *   "Unnamed" device shown on the client side).
		 * ============================================================================
		 */

		/* Health Service attribute record allocation:
		 *
		 * 1  Primary Service
		 * 2  BPM characteristic (READ)
		 * 2  Weight characteristic (READ)
		 * 3  Data TX characteristic (NOTIFY + CCCD)
		 * 2  Control RX characteristic (WRITE / WRITE_NO_RESP)
		 *   ----------------------------------------------
		 * = 12 attribute records (for this service only)
		 */
		#define HEALTH_SERVICE_ATTR_RECORDS     (12)

		uint8_t Max_Attribute_Records = HEALTH_SERVICE_ATTR_RECORDS;
		ret = validate_add_service_params(
		        UUID_TYPE_128,
		        &health_service_uuid,
		        PRIMARY_SERVICE,
		        Max_Attribute_Records,
		        &health_service_handle);
		if( BLE_STATUS_SUCCESS != ret )
		{
		  LOG_WARN("validate_add_service_params FAILED (%d) for health_service", ret);
		  break;
		}
		ret = aci_gatt_add_service(UUID_TYPE_128, &health_service_uuid, PRIMARY_SERVICE, Max_Attribute_Records, &health_service_handle);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_add_service : FAILED (%d) for health_service", ret);
			break;
		}

		/* Add BPM characteristic (READ) to health service */
		BLUENRG_memcpy(health_bpm_char_uuid.Char_UUID_128, HEALTH_BPM_CHAR_UUID, sizeof(HEALTH_BPM_CHAR_UUID));

		uint8_t Char_Properties = CHAR_PROP_READ; /* Earlier set to CHAR_PROP_NOTIFY */
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		uint16_t Char_Value_Length = ( CHAR_PROP_NOTIFY == Char_Properties ) ? 1 : 2;
		uint8_t GATT_Evt_Mask = ( CHAR_PROP_NOTIFY == Char_Properties ) ? GATT_DONT_NOTIFY_EVENTS : GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
		/* Attribute Permission impacts whether:
		 * Encryption required, LTK required, MITM required
		 */
		uint8_t Security_Permissions = ATTR_PERMISSION_NONE;
		/* Minimum required LTK size (in bytes) for encrypted access to this attribute.
		 * 0  = no encryption required
		 * 7-16 = minimum acceptable key size enforced during pairing
		 * Actual LTK is generated by the BLE stack during pairing and must be >= this value.
		 */
		uint8_t Enc_Key_Size = ( ATTR_PERMISSION_NONE == Security_Permissions ) ? 0 : 16;
		/* 0 means Fixed Length */
		uint8_t Is_Variable = 0;

		ret = validate_add_char_params(UUID_TYPE_128, health_bpm_char_uuid.Char_UUID_128, Char_Value_Length, Char_Properties, Security_Permissions, Enc_Key_Size, Is_Variable);
		if( BLE_STATUS_SUCCESS != ret )
		{
			LOG_DEBUG("validate_add_char_params FAILED (%d) for health_bpm_char_handle", ret);
			break;
		}

		ret = aci_gatt_add_char(health_service_handle, UUID_TYPE_128, &health_bpm_char_uuid, Char_Value_Length, Char_Properties, Security_Permissions, GATT_Evt_Mask, Enc_Key_Size, Is_Variable, &health_bpm_char_handle);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_add_char : FAILED (%d) for health_bpm_char_handle", ret);
			break;
		}

		/* Add Weight characteristic (READ) to health service */
		BLUENRG_memcpy(health_weight_char_uuid.Char_UUID_128, HEALTH_WEIGHT_CHAR_UUID, sizeof(HEALTH_WEIGHT_CHAR_UUID));

		/* Add characteristic */
		/* Char_Properties is set to CHAR_PROP_READ */
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		/* Char_Value_Length = ( CHAR_PROP_NOTIFY == Char_Properties ) ? 1 : 2 */
		/* GATT_Evt_Mask = ( CHAR_PROP_NOTIFY == Char_Properties ) ? GATT_DONT_NOTIFY_EVENTS : GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP*/
		/* Already set to ATTR_PERMISSION_NONE, hence commented: uint8_t Security_Permissions = ATTR_PERMISSION_NONE; */
		/* Already set to 16, hence commented: Enc_Key_Size = ( ATTR_PERMISSION_NONE == Security_Permissions ) ? 0 : 16; */
		/* 0 means Fixed Length. */
		/* Is_Variable = 0; */
		ret = validate_add_char_params(UUID_TYPE_128, health_weight_char_uuid.Char_UUID_128, Char_Value_Length, Char_Properties, Security_Permissions, Enc_Key_Size, Is_Variable);
		if( BLE_STATUS_SUCCESS != ret )
		{
			LOG_WARN("validate_add_char_params FAILED (%d) for health_weight_char_handle", ret);
			break;
		}

		ret = aci_gatt_add_char(health_service_handle, UUID_TYPE_128, &health_weight_char_uuid, Char_Value_Length, Char_Properties, Security_Permissions, GATT_Evt_Mask, Enc_Key_Size, Is_Variable, &health_weight_char_handle);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_add_char : FAILED (%d) for health_weight_char_handle", ret);
			break;
		}

		/* Add data tx characteristic (NOTIFY) to health service */
		BLUENRG_memcpy(health_data_tx_char_uuid.Char_UUID_128, HEALTH_DATA_TX_CHAR_UUID, sizeof(HEALTH_DATA_TX_CHAR_UUID));

		/* Add characteristic */
		Char_Properties = CHAR_PROP_NOTIFY;
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		Char_Value_Length = DATA_TX_CHAR_VALUE_LENGTH;
		GATT_Evt_Mask = GATT_DONT_NOTIFY_EVENTS;
		/* Already set to ATTR_PERMISSION_NONE, hence commented: uint8_t Security_Permissions = ATTR_PERMISSION_NONE; */
		Enc_Key_Size = ( ATTR_PERMISSION_NONE == Security_Permissions ) ? 0 : 16;
		/* 0 means Fixed Length, 1 means Variable length. */
		Is_Variable = 1;

		ret = validate_add_char_params(UUID_TYPE_128, health_data_tx_char_uuid.Char_UUID_128, Char_Value_Length, Char_Properties, Security_Permissions, Enc_Key_Size, Is_Variable);
		if( BLE_STATUS_SUCCESS != ret )
		{
			LOG_DEBUG("validate_add_char_params FAILED (%d) for health_data_tx_char_handle", ret);
			break;
		}

		ret = aci_gatt_add_char(health_service_handle, UUID_TYPE_128, &health_data_tx_char_uuid, Char_Value_Length, Char_Properties, Security_Permissions, GATT_Evt_Mask, Enc_Key_Size, Is_Variable, &health_data_tx_char_handle);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_add_char : FAILED (%d) for health_data_tx_char_handle", ret);
			break;
		}

		/* Add Control RX characteristic (WRITE / WRITE_NO_RESP) to health service */
		BLUENRG_memcpy(health_control_rx_char_uuid.Char_UUID_128, HEALTH_CONTROL_RX_CHAR_UUID, sizeof(HEALTH_CONTROL_RX_CHAR_UUID));

		/* Add characteristic */
		Char_Properties = CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP;
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		Char_Value_Length = CONTROL_RX_CHAR_VALUE_LENGTH;
		GATT_Evt_Mask = GATT_NOTIFY_ATTRIBUTE_WRITE;
		Security_Permissions = ATTR_PERMISSION_NONE;
		Enc_Key_Size = ( ATTR_PERMISSION_NONE == Security_Permissions ) ? 0 : 16;
		/* 0 means Fixed Length 1 means Variable length. */
		/* Variable-length control RX characteristic (reuses Is_Variable = 1 from Data TX) */
		ret = validate_add_char_params(UUID_TYPE_128, health_control_rx_char_uuid.Char_UUID_128, Char_Value_Length, Char_Properties, Security_Permissions, Enc_Key_Size, Is_Variable);
		if( BLE_STATUS_SUCCESS != ret )
		{
			LOG_DEBUG("validate_add_char_params FAILED (%d) for health_control_rx_char_handle", ret);
			break;
		}
		ret = aci_gatt_add_char(health_service_handle, UUID_TYPE_128, &health_control_rx_char_uuid, Char_Value_Length, Char_Properties, Security_Permissions, GATT_Evt_Mask, Enc_Key_Size, Is_Variable, &health_control_rx_char_handle);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_add_char : FAILED (%d) for health_control_rx_char_handle", ret);
			break;
		}

		/* Add weather service */
		BLUENRG_memcpy(weather_service_uuid.Service_UUID_128, WEATHER_SERVICE_UUID, sizeof(WEATHER_SERVICE_UUID));

		/* Weather Service attribute record allocation:
		 *
		 * 1  Primary Service
		 * 2  Temperature characteristic (READ)
		 * 2  Humidity characteristic (READ)
		 *   --------------------------------------
		 * = 5 attribute records (for this service only)
		 */
		#define WEATHER_SERVICE_ATTR_RECORDS    (5)

		Max_Attribute_Records = WEATHER_SERVICE_ATTR_RECORDS;
		ret = validate_add_service_params(
		        UUID_TYPE_128,
		        &weather_service_uuid,
		        PRIMARY_SERVICE,
		        Max_Attribute_Records,
		        &weather_service_handle);
		if( BLE_STATUS_SUCCESS != ret )
		{
		  LOG_DEBUG("validate_add_service_params FAILED (%d) for weather_service", ret);
		  break;
		}
		ret = aci_gatt_add_service(UUID_TYPE_128, &weather_service_uuid, PRIMARY_SERVICE, Max_Attribute_Records, &weather_service_handle);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_add_service : FAILED (%d) for weather_service", ret);
			break;
		}

		/* Add temperature characteristic (READ) to weather service */
		BLUENRG_memcpy(weather_temperature_char_uuid.Char_UUID_128, WEATHER_TEMPERATURE_CHAR_UUID, sizeof(WEATHER_TEMPERATURE_CHAR_UUID));

		/* Add characteristic */
		Char_Properties = CHAR_PROP_READ;
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		Char_Value_Length = ( CHAR_PROP_NOTIFY == Char_Properties ) ? 1 : 2;
		GATT_Evt_Mask = ( CHAR_PROP_NOTIFY == Char_Properties ) ? GATT_DONT_NOTIFY_EVENTS : GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
		Security_Permissions = ATTR_PERMISSION_NONE;
		Enc_Key_Size = ( ATTR_PERMISSION_NONE == Security_Permissions ) ? 0 : 16;
		/* 0 means Fixed Length. */
		Is_Variable = 0;
		ret = validate_add_char_params(UUID_TYPE_128, weather_temperature_char_uuid.Char_UUID_128, Char_Value_Length, Char_Properties, Security_Permissions, Enc_Key_Size, Is_Variable);
		if( BLE_STATUS_SUCCESS != ret )
		{
			LOG_DEBUG("validate_add_char_params FAILED (%d) for weather_temperature_char_handle", ret);
			break;
		}

		ret = aci_gatt_add_char(weather_service_handle, UUID_TYPE_128, &weather_temperature_char_uuid, Char_Value_Length, Char_Properties, Security_Permissions, GATT_Evt_Mask, Enc_Key_Size, Is_Variable, &weather_temperature_char_handle);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_add_char : FAILED (%d) for weather_temperature_char_handle", ret);
			break;
		}

		/* Add humidity characteristic (READ) to weather service */
		BLUENRG_memcpy(weather_humidity_char_uuid.Char_UUID_128, WEATHER_HUMIDITY_CHAR_UUID, sizeof(WEATHER_HUMIDITY_CHAR_UUID));

		/* Add characteristic */
		/* Char_Properties is set to CHAR_PROP_READ */
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		/* Char_Value_Length = ( CHAR_PROP_NOTIFY == Char_Properties ) ? 1 : 2 */
		/* GATT_Evt_Mask = ( CHAR_PROP_NOTIFY == Char_Properties ) ? GATT_DONT_NOTIFY_EVENTS : GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP*/
		/* Already set to ATTR_PERMISSION_NONE, hence commented: uint8_t Security_Permissions = ATTR_PERMISSION_NONE; */
		/* Already set to 16, hence commented: Enc_Key_Size = ( ATTR_PERMISSION_NONE == Security_Permissions ) ? 0 : 16; */
		/* 0 means Fixed Length. */
		/* Is_Variable = 0; */
		ret = validate_add_char_params(UUID_TYPE_128, weather_humidity_char_uuid.Char_UUID_128, Char_Value_Length, Char_Properties, Security_Permissions, Enc_Key_Size, Is_Variable);
		if( BLE_STATUS_SUCCESS != ret )
		{
			LOG_DEBUG("validate_add_char_params FAILED (%d) for weather_humidity_char_handle", ret);
			break;
		}

		ret = aci_gatt_add_char(weather_service_handle, UUID_TYPE_128, &weather_humidity_char_uuid, Char_Value_Length, Char_Properties, Security_Permissions, GATT_Evt_Mask, Enc_Key_Size, Is_Variable, &weather_humidity_char_handle);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_add_char : FAILED (%d) for weather_humidity_char_handle", ret);
			break;
		}

	} while( false );

	return ret;
}

uint8_t health_control_data_rx[DEF_CONTROL_RX_CHAR_VALUE_LENGTH];

typedef char STATIC_ASSERT_health_control_rx_size[ (sizeof(health_control_data_rx) == DEF_CONTROL_RX_CHAR_VALUE_LENGTH) ? 1 : -1 ];

tBleStatus health_control_rx(uint8_t *data_rx, uint16_t rx_bytes_len)
{
	tBleStatus ret = BLE_STATUS_SUCCESS;

	do
	{
		/* NULL pointer check */
		if( NULL == data_rx )
		{
			LOG_WARN("health_control_rx: data_rx is NULL");
			ret = BLE_STATUS_NULL_PARAM;
			break;
		}

		/* Zero-length writes are invalid for control commands */
		if( 0U == rx_bytes_len )
		{
			LOG_WARN("health_control_rx: zero-length write");
			ret = BLE_STATUS_INVALID_PARAMS;
			break;
		}

		/* Internal buffer size validation */
		const uint16_t buf_len = sizeof(health_control_data_rx);
		if( buf_len < rx_bytes_len )
		{
			LOG_WARN("health_control_rx: buffer too small (%u < %u)", buf_len, rx_bytes_len);
			ret = BLE_STATUS_INVALID_PARAMS;
			break;
		}

		/* Copy exactly what was written */
		BLUENRG_memcpy(health_control_data_rx, data_rx, rx_bytes_len);
		g_health_control_rx_len = rx_bytes_len;
		LOG_DEBUG("health_control_rx: received %u bytes", rx_bytes_len);

	} while(false);
	if(BLE_STATUS_SUCCESS != ret)
	{
		BLUENRG_memset(health_control_data_rx, 0, sizeof(health_control_data_rx));
		g_health_control_rx_len = 0;
	}
	return ret;
}

tBleStatus health_data_tx(const uint8_t * data_tx, uint16_t tx_bytes_len)
{
  tBleStatus ret = BLE_STATUS_SUCCESS;

  do
  {
    /* NULL pointer check */
    if( NULL == data_tx )
    {
      LOG_WARN("health_data_tx: data_tx is NULL");
      ret = BLE_STATUS_NULL_PARAM;
      break;
    }

    /* Zero-length notify is invalid at application level */
    if( 0U == tx_bytes_len )
    {
      LOG_WARN("health_data_tx: zero-length transmit");
      ret = BLE_STATUS_INVALID_PARAMS;
      break;
    }

    /* Validate against characteristic max length configured at add_char time */
    if( DATA_TX_CHAR_VALUE_LENGTH < tx_bytes_len )
    {
      LOG_WARN("health_data_tx: max allowed length %u, tx length %u", DATA_TX_CHAR_VALUE_LENGTH, tx_bytes_len);
      ret = BLE_STATUS_INVALID_PARAMS;
      break;
    }

    /* ---- PROTOCOL GUARD ---- */
    if( INVALID_CONNECTION_HANDLE == connection_handle )
    {
      LOG_WARN("health_data_tx: invalid connection handle");
      ret = BLE_STATUS_FAILED;
      break;
    }

    if( false == notification_enabled )
    {
      LOG_WARN("health_data_tx: notifications not enabled");
      ret = BLE_STATUS_FAILED;
      break;
    }

    /* Offset is always zero: full attribute value update */
    const uint8_t CurrentOffset = 0;

    /* NOTE:
     * aci_gatt_update_char_value() uses a uint8_t length parameter.
     * Therefore the absolute maximum supported by the API is 255 bytes.
     *
     * Actual transmittable payload over-the-air is limited by (ATT_MTU - 3).
     * Default ATT_MTU is 23 bytes (20-byte payload) unless MTU exchange
     * is implemented elsewhere.
     *
     * This check enforces ONLY the API limit, not the negotiated ATT MTU.
     */
 		#define BLUENRG_MAX_CHAR_VALUE_UPDATE_LEN   (255U)
    if(BLUENRG_MAX_CHAR_VALUE_UPDATE_LEN < tx_bytes_len)
    {
    	LOG_WARN("health_data_tx: tx length %u exceeds BlueNRG ATT update limit (uint8_t length, MTU-dependent)", tx_bytes_len);
    	ret = BLE_STATUS_INVALID_PARAMS;
    	break;
    }
    /* Actual number of bytes written to the characteristic value */
    const uint8_t Char_Value_Length = tx_bytes_len;

    ret = aci_gatt_update_char_value(health_service_handle, health_data_tx_char_handle, CurrentOffset, Char_Value_Length, (uint8_t *)data_tx);

    if( BLE_STATUS_SUCCESS != ret )
    {
      LOG_DEBUG("aci_gatt_update_char_value: health_data_tx FAILED (%d)", ret);
      break;
    }

    LOG_DEBUG("health_data_tx: transmitted %u bytes", tx_bytes_len);

  } while( false );

  return ret;
}

tBleStatus update_bpm_data(int16_t new_data)
{
	tBleStatus ret;
	do
	{
		/* Update BPM characteristic value in Health Service */
		/* If this is set to 0 and the attribute value is of variable length. */
		const uint8_t CurrentOffset = 0;
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		const uint8_t Char_Value_Length = 2;
		ret = aci_gatt_update_char_value(health_service_handle, health_bpm_char_handle, CurrentOffset, Char_Value_Length, (uint8_t *)&new_data);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_update_char_value: BPM Characteristic update FAILED (%d)", ret);
			break;
		}
	} while( false );

	return ret;
}

tBleStatus update_weight_data(int16_t new_data)
{
	tBleStatus ret;
	do
	{
		/* Update Weight characteristic value in Health Service */
		/* If this is set to 0 and the attribute value is of variable length. */
		const uint8_t CurrentOffset = 0;
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		const uint8_t Char_Value_Length = 2;
		ret = aci_gatt_update_char_value(health_service_handle, health_weight_char_handle, CurrentOffset, Char_Value_Length, (uint8_t *)&new_data);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_update_char_value: Weight Characteristic update FAILED (%d)", ret);
			break;
		}
	} while( false );

	return ret;
}

tBleStatus update_temperature_data(int16_t new_data)
{
	tBleStatus ret;
	do
	{
		/* Update Temperature characteristic value in Weather Service */
		/* If this is set to 0 and the attribute value is of variable length. */
		const uint8_t CurrentOffset = 0;
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		const uint8_t Char_Value_Length = 2;
		ret = aci_gatt_update_char_value(weather_service_handle, weather_temperature_char_handle, CurrentOffset, Char_Value_Length, (uint8_t *)&new_data);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_update_char_value: Temperature Characteristic update FAILED (%d)", ret);
			break;
		}
	} while( false );

	return ret;
}

tBleStatus update_humidity_data(int16_t new_data)
{
	tBleStatus ret;
	do
	{
		/* Update Humidity characteristic value in Weather Service */
		/* If this is set to 0 and the attribute value is of variable length. */
		const uint8_t CurrentOffset = 0;
		/* Char_Value_Length informs maximum size (in bytes) of the characteristic VALUE attribute stored in GATT DB. */
		const uint8_t Char_Value_Length = 2;
		ret = aci_gatt_update_char_value(weather_service_handle, weather_humidity_char_handle, CurrentOffset, Char_Value_Length, (uint8_t *)&new_data);
		if(BLE_STATUS_SUCCESS != ret)
		{
			LOG_DEBUG("aci_gatt_update_char_value: humidity Characteristic update FAILED (%d)", ret);
			break;
		}
	} while( false );

	return ret;
}

tBleStatus Attribute_Modify_CB(uint16_t handle, uint16_t offset, uint16_t data_length, uint8_t *att_data)
{
	tBleStatus ret = BLE_STATUS_SUCCESS;

	do
	{
		if( 0U != offset )
		{
			LOG_WARN("Attribute_Modify_CB passed non zero offset");
			ret = BLE_STATUS_INVALID_PARAMS;
			break;
		}

		if( INVALID_CONNECTION_HANDLE == connection_handle )
		{
			LOG_WARN("health_control_rx while disconnected");
			ret = BLE_STATUS_FAILED;
			break;
		}

		if( ( health_control_rx_char_handle + 1U ) == handle )
		{
			ret = health_control_rx(att_data, data_length);
			if( BLE_STATUS_SUCCESS != ret )
			{
				LOG_WARN("health_control_rx FAILED (%d)", ret);
				break;
			}
		}
		/*
		 * For enabling notification the configuration data with LSB first
		 * is { 0x01, 0x00 }, thus the handle of the attribute for data tx
		 * is offset by two from the characteristic handle.
		 *
		 * Using X-CUBE-NRG2, therefore Characteristic Handle layout:
		 *		H     : Characteristic Declaration
		 *		H+1   : Characteristic Value
		 *		H+2   : CCCD
		 */
		else if( ( health_data_tx_char_handle + 2U ) == handle )
		{
			if( 2U != data_length )
			{
				LOG_WARN("CCCD write with invalid length %u", data_length);
				ret = BLE_STATUS_INVALID_PARAMS;
				break;
			}

			/* CCCD write with LSB first: {0x01, 0x00} = notifications enabled */
			/* Add code for notify enable */
			if( 0U != att_data[1] )
			{
				LOG_WARN("CCCD write with invalid MSB 0x%02X", att_data[1]);
				ret = BLE_STATUS_INVALID_PARAMS;
				break;
			}
			/* Reject unsupported CCCD bits (only bit0 is valid for notifications) */
			if( 0U != ( att_data[0] & ~0x01U ) )
			{
				LOG_WARN("CCCD write with unsupported bits 0x%02X", att_data[0]);
				ret = BLE_STATUS_INVALID_PARAMS;
				break;
			}
			notification_enabled = ( 0U != ( att_data[0] & 0x01U ) ) ? true : false; /* Needed during Enable / Disable */
			LOG_DEBUG("Notify %s", notification_enabled ? "ENABLED" : "DISABLED");
		}
		else
		{
			LOG_WARN("Attribute_Modify_CB: unknown handle 0x%04X", handle);
			ret = BLE_STATUS_FAILED;
		}
	} while(false);

	return ret;
}

/*
 * Connection_Handle (i.e., WHO is accessing):
 *		Scope:				Link / connection level
 *		Meaning:			Identifies which BLE connection this request belongs to
 *		Assigned:			At LE connection complete
 *		Lifetime:			Valid until disconnect
 *		Who owns it:	Controller / Link Layer
 *
 * Attribute_Handle (i.e., WHAT is accessed):
 *		Scope:				GATT database
 *		Meaning:			Identifies which attribute is being accessed
 *		Assigned:			When you add services/characteristics
 *		Lifetime:			Static while the GATT DB exists
 *		Who owns it:	GATT server
 */
void Read_Request_CB(uint16_t conn_handle,
                     uint16_t attr_handle,
                     uint16_t offset)
{
	tBleStatus ret = BLE_STATUS_SUCCESS;

	do
	{
		/* This implementation supports only short, fixed-length attributes */
		if(0 != offset)
		{
			LOG_WARN("Read_Request_CB : NON-ZERO OFFSET (%u) handle=0x%04X", offset, attr_handle);
			aci_gatt_deny_read(conn_handle, BLE_STATUS_INVALID_PARAMS);
			break;
		}

		/* Decide WHAT attribute is being read */
		if( ( health_bpm_char_handle + 1 ) == attr_handle )
		{
			ret = update_bpm_data(TEST_BPM_SENSOR_DATA);
			if(BLE_STATUS_SUCCESS != ret)
			{
				LOG_WARN("update_bpm_data : FAILED (%d)", ret);
				aci_gatt_deny_read(conn_handle, BLE_STATUS_INVALID_PARAMS);
				break;
			}
		}
		else if( ( health_weight_char_handle + 1 ) == attr_handle )
		{
			ret = update_weight_data(TEST_WEIGHT_SENSOR_DATA);
			if(BLE_STATUS_SUCCESS != ret)
			{
				LOG_WARN("update_weight_data : FAILED (%d)", ret);
				aci_gatt_deny_read(conn_handle, BLE_STATUS_INVALID_PARAMS);
				break;
			}
		}
		else if( ( weather_temperature_char_handle + 1 ) == attr_handle )
		{
			ret = update_temperature_data(TEST_TEMPERATURE_SENSOR_DATA);
			if(BLE_STATUS_SUCCESS != ret)
			{
				LOG_WARN("update_temperature_data : FAILED (%d)", ret);
				aci_gatt_deny_read(conn_handle, BLE_STATUS_INVALID_PARAMS);
				break;
			}
		}
		else if( ( weather_humidity_char_handle + 1 ) == attr_handle )
		{
			ret = update_humidity_data(TEST_HUMIDITY_SENSOR_DATA);
			if(BLE_STATUS_SUCCESS != ret)
			{
				LOG_WARN("update_humidity_data : FAILED (%d)", ret);
				aci_gatt_deny_read(conn_handle, BLE_STATUS_INVALID_PARAMS);
				break;
			}
		}
		else
		{
			LOG_WARN("Read_Request_CB : UNKNOWN ATTRIBUTE handle=0x%04X", attr_handle);
			break;
		}

		/* Allow the read for THIS connection only */
		if(INVALID_CONNECTION_HANDLE != conn_handle)
		{
			ret = aci_gatt_allow_read(conn_handle);
			if(BLE_STATUS_SUCCESS != ret)
			{
				LOG_WARN("aci_gatt_allow_read : FAILED (%d) conn=0x%04X", ret, conn_handle);
				break;
			}
		}

	}while(false);
}

void aci_gatt_attribute_modified_event(uint16_t Connection_Handle, uint16_t Attr_Handle, uint16_t Offset, uint16_t Attr_Data_Length, uint8_t Attr_Data[])
{
	(void)Connection_Handle; /* handled via global connection_handle */

	/* BlueNRG-2 supports a single active connection only.
	 * Application tracks connection state via global connection_handle.
	 */
	tBleStatus ret = Attribute_Modify_CB(Attr_Handle, Offset, Attr_Data_Length, Attr_Data);

	if( BLE_STATUS_SUCCESS != ret )
	{
		LOG_WARN("Attribute_Modify_CB FAILED (%d) handle=0x%04X", ret, Attr_Handle);
	}
}

//void aci_gatt_notification_event(uint16_t Connection_Handle, uint16_t Attribute_Handle, uint8_t Attribute_Value_Length, uint8_t Attribute_Value[])
//{
//	if( Attribute_Handle == ( health_data_tx_char_handle + 2 ) )
//	{
//		health_control_rx(Attribute_Value, Attribute_Value_Length);
//	}
//}

void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
	Read_Request_CB(Connection_Handle, Attribute_Handle, Offset);
}

void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
	connection_handle = Connection_Handle;
	notification_enabled = false;
	LOG_DEBUG("Connected handle=0x%04X", Connection_Handle);
}

void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
	connection_handle = INVALID_CONNECTION_HANDLE;
	notification_enabled = false; /* Needed during disconnection */
	/* Global / file-scope flag */
	g_restart_adv = true;
	BLUENRG_memset(health_control_data_rx, 0, sizeof(health_control_data_rx));
	g_health_control_rx_len = 0;
	LOG_DEBUG("Disconnected handle=0x%04X", Connection_Handle);
}

void App_UserEvtRx(void *pData)
{
	do
	{
		if( NULL == pData )
		{
			break;
		}

		hci_spi_pckt * hci_pckt = (hci_spi_pckt *)pData;

		/* Process event packet */
		if( HCI_EVENT_PKT != hci_pckt->type)
		{
			break;
		}
		/* Get data from register */
		hci_event_pckt * event_pckt = (hci_event_pckt * )hci_pckt->data;
    switch(event_pckt->evt)
    {
    	case EVT_LE_META_EVENT:
  		{
  			bool handled = false;
				/* Need at least 1 byte for subevent */
				if (event_pckt->plen < EVT_LE_META_EVENT_SIZE)
				{
					LOG_WARN("LE Meta Event with invalid length %u", event_pckt->plen);
					break;
				}
  			/* Get meta data */
  			evt_le_meta_event *evt = (evt_le_meta_event*)event_pckt->data;

				if( EVT_LE_CONN_COMPLETE == evt->subevent)
				{
			    /* NOTE:
			     *   - Do NOT log here
			     *   - Do NOT update connection_handle here
			     *   - Ownership is with hci_le_connection_complete_event()
			     */
			    /* Validate connection-complete payload length. */
			    /* subevent (1 byte) + connection complete payload */
			    if (event_pckt->plen < (EVT_LE_META_EVENT_SIZE + sizeof(evt_le_connection_complete)))
			    {
						LOG_WARN("LE Conn Complete with invalid length %u", event_pckt->plen);
						break;
			    }
					// #NOTE: connection_handle is set inside hci_le_connection_complete_event
					// #NOTE: cast evt->data using (evt_le_connection_complete *) to get connection complete data structure values.
				}
        /* Process each meta data event */
  			const int MaxItemsInMetaEvtTable = sizeof(hci_le_meta_events_table) / sizeof(hci_le_meta_events_table_type);
  			uint32_t i;
  			for(i = 0; MaxItemsInMetaEvtTable > i; i++)
  			{
  				if(evt->subevent == hci_le_meta_events_table[i].evt_code)
  				{
  					hci_le_meta_events_table[i].process((void *)evt->data);
  					handled = true;
  					break;
  				}
  			}
  			if(false == handled)
  			{
  			    LOG_WARN("Unhandled LE Meta subevent=0x%02X", evt->subevent);
  			}
  			break;
  		}
    	case EVT_VENDOR:
    	{
    		bool handled = false;
    		/* Get Event Vendor */
    		evt_blue_aci *blue_evt = (evt_blue_aci *)event_pckt->data;
        /* Process each Event Vendor event */
  			const int MaxItemsInVendorSpecificEvtTable = sizeof(hci_vendor_specific_events_table) / sizeof(hci_vendor_specific_events_table_type);
  			uint32_t i;
  			for(i = 0; MaxItemsInVendorSpecificEvtTable > i; i++)
  			{
  				if(blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code)
  				{
  					hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
  					handled = true;
  					break;
  				}
  				}
  			if(false == handled)
				{
  				LOG_WARN("Unhandled Vendor event ecode=0x%04X", blue_evt->ecode);
  			}
    		break;
    	}
    	default:
    	{
    		bool handled = false;
  			const int MaxItemsInEventsTable = sizeof(hci_events_table) / sizeof(hci_events_table_type);
  			uint32_t i;
  			for(i = 0; MaxItemsInEventsTable > i; i++)
  			{
  				if(event_pckt->evt == hci_events_table[i].evt_code)
  				{
  					hci_events_table[i].process((void *)event_pckt->data);
  					handled = true;
  					break;
  				}
  			}
  			if(false == handled)
  			{
  				LOG_WARN("Unhandled HCI event evt=0x%02X", event_pckt->evt);
  			}
    		break;
    	}
    }
	}while( false );
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	#define BUTTON_DEBOUNCE_MS		( 100 )
	do
	{
//	if(GPIO_Pin == HCI_TL_SPI_EXTI_PIN)
//	{
//		LOG_DEBUG("IRQ");
//		hci_notify_asynch_evt(NULL);
//	}
		if ( B1_Pin == GPIO_Pin )
		{
			uint32_t now = HAL_GetTick();

			/* Debounce check */
			if ((now - g_last_btn_tick) < BUTTON_DEBOUNCE_MS)
			{
				return;   /* Ignore jitter */
			}
			g_last_btn_tick = now;
			g_btn_event = true;   /* One clean event */
		}
	}while(false);
}
