#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_log.h"
#include "nrf_delay.h"
#include "m_ucp_flash.h"
static bool write_to_flash_flag = false;
/* Array to map FDS return values to strings. */
static param_to_save_t param_to_save;
char const * fds_err_str[] =
{
    "FDS_SUCCESS",
    "FDS_ERR_OPERATION_TIMEOUT",
    "FDS_ERR_NOT_INITIALIZED",
    "FDS_ERR_UNALIGNED_ADDR",
    "FDS_ERR_INVALID_ARG",
    "FDS_ERR_NULL_ARG",
    "FDS_ERR_NO_OPEN_RECORDS",
    "FDS_ERR_NO_SPACE_IN_FLASH",
    "FDS_ERR_NO_SPACE_IN_QUEUES",
    "FDS_ERR_RECORD_TOO_LARGE",
    "FDS_ERR_NOT_FOUND",
    "FDS_ERR_NO_PAGES",
    "FDS_ERR_USER_LIMIT_REACHED",
    "FDS_ERR_CRC_CHECK_FAILED",
    "FDS_ERR_BUSY",
    "FDS_ERR_INTERNAL",
};

/* A record containing dummy configuration data. */
static fds_record_t const m_param_to_save_struct_record =
{
    .file_id           = CONFIG_FILE_ID,
    .key               = CONFIG_REC_KEY,
    .data.p_data       = &param_to_save,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(param_to_save_t) + 3) / sizeof(uint32_t),
};


void set_write_to_flash_flag(void)
{
    write_to_flash_flag = true;
}
/* Flag to check fds initialization. */
static bool volatile m_fds_initialized = false;

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == FDS_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;
		case FDS_EVT_UPDATE:
		if (p_evt->result == FDS_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
				NRF_LOG_INFO("UPDATE success");
            }
			break;

        default:
            break;
    }
}

//×¢²á  flashÇøÓò
void user_param_flash_register(void)
{
//	ret_code_t rc;
//	(void) fds_register(fds_evt_handler);
//	rc = fds_init();
//    APP_ERROR_CHECK(rc);
//	while(!m_fds_initialized);
//    fds_record_desc_t desc = {0};
//    fds_find_token_t  tok  = {0};
//	rc = fds_record_find(CONFIG_FILE_ID, CONFIG_REC_KEY, &desc, &tok);
//	if (rc == FDS_SUCCESS)
//    {
//        /* A config file is in flash. Let's update it. */
//        fds_flash_record_t config = {0};
//
//        /* Open the record and read its contents. */
//        rc = fds_record_open(&desc, &config);
//        APP_ERROR_CHECK(rc);
//
//        /* Copy the configuration from flash into bluetooth_info. */
//        memcpy(&bluetooth_info, config.p_data, sizeof(bluetooth_info));
//
//		rc = fds_record_close(&desc);
//        APP_ERROR_CHECK(rc);
//    }
//    else
//    {
//        /* System config not found; write a new one. */
//        NRF_LOG_INFO("Write default value");
//		write_default_param();
//       do
//		{
//			rc = fds_record_write(&desc, &m_bluetooth_info_struct_record);
//			if(rc==FDS_ERR_NO_SPACE_IN_FLASH)
//			{
//				fds_gc();
//			}
//		}while(rc!=FDS_SUCCESS);
//    }
//	//just for test param
}
//Ïò  flashÖÐÐ´ÈëÊý¾Ý

void param_write_to_flash(void)
{
//    ret_code_t rc;
//    fds_record_desc_t desc = {0};
//    fds_find_token_t  tok  = {0};
//    if(write_to_flash_flag)
//    {
//
//        rc = fds_record_find(CONFIG_FILE_ID, CONFIG_REC_KEY, &desc, &tok);
//        if (rc == FDS_SUCCESS)
//        {
//            do
//                {
//                    rc = fds_record_update(&desc, &m_bluetooth_info_struct_record);
//                    if(rc==FDS_ERR_NO_SPACE_IN_FLASH)
//                    {
//                        fds_gc();
//                    }
//                }while(rc!=FDS_SUCCESS);
//            
//        }
//        write_to_flash_flag = false;
//    }
}
