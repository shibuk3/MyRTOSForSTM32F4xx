#include "bootloader.h"
#include "gpio.h"
#define BL_RX_LEN  200
uint8_t bl_rx_buffer[BL_RX_LEN];

uint8_t supported_commands[] = {
                               BL_GET_VER ,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_GET_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
                               BL_READ_SECTOR_P_STATUS} ;



void  bootloader_uart_read_data(void)
{
    uint8_t rcv_len=0;

	while(1)
	{
		memset(bl_rx_buffer,0,200);
		//here we will read and decode the commands coming from host
		//first read only one byte from the host , which is the "length" field of the command packet
		usart_receive_bytes(bl_rx_buffer,2);
		rcv_len= bl_rx_buffer[0]-'0';
		usart_receive_bytes(&bl_rx_buffer[2],rcv_len+1);
		uint8_t payLoad[100];
		memset(payLoad,0,100);
		sprintf(payLoad,"BL_DEBUG_MSG:command code received from host:%d %d \r\n" , bl_rx_buffer[0], bl_rx_buffer[2]);
	    usart_send_bytes(payLoad,sizeof(payLoad));
		switch(bl_rx_buffer[2])
		{
            case BL_GET_VER:
                bootloader_handle_getver_cmd(bl_rx_buffer);
                break;
            case BL_GET_HELP:
                bootloader_handle_gethelp_cmd(bl_rx_buffer);
                break;
            case BL_GET_CID:
                bootloader_handle_getcid_cmd(bl_rx_buffer);
                break;
            case BL_GET_RDP_STATUS:
                bootloader_handle_getrdp_cmd(bl_rx_buffer);
                break;
            case BL_GO_TO_ADDR:
                bootloader_handle_go_cmd(bl_rx_buffer);
                break;
            case BL_FLASH_ERASE:
                bootloader_handle_flash_erase_cmd(bl_rx_buffer);
                break;
            case BL_MEM_WRITE:
                bootloader_handle_mem_write_cmd(bl_rx_buffer);
                break;
            case BL_EN_RW_PROTECT:
                bootloader_handle_en_rw_protect(bl_rx_buffer);
                break;
            case BL_MEM_READ:
                bootloader_handle_mem_read(bl_rx_buffer);
                break;
            case BL_READ_SECTOR_P_STATUS:
                bootloader_handle_read_sector_protection_status(bl_rx_buffer);
                break;
            case BL_OTP_READ:
                bootloader_handle_read_otp(bl_rx_buffer);
                break;
						case BL_DIS_R_W_PROTECT:
                bootloader_handle_dis_rw_protect(bl_rx_buffer);
                break;
             default:
            	sprintf(payLoad,"BL_DEBUG_MSG:Invalid command code received from host \r\n");
                usart_send_bytes(payLoad,sizeof(payLoad));
                break;


		}
		for(int i=0;i<900000;i++){}

	}

}

void bootloader_jump_to_user_app(void)
{

   //just a function pointer to hold the address of the reset handler of the user app.
    void (*app_reset_handler)(void);
    uint8_t payLoad[100];
    sprintf(payLoad ,"BL_DEBUG_MSG:bootloader_jump_to_user_app\n");
    usart_send_bytes(payLoad,sizeof(payLoad));

    // 1. configure the MSP by reading the value from the base address of the sector 2
    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
    sprintf(payLoad,"BL_DEBUG_MSG:MSP value : %#x\n",msp_value);
    usart_send_bytes(payLoad,sizeof(payLoad));

    //This function comes from CMSIS.
    __set_MSP(msp_value);

    //SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

    /* 2. Now fetch the reset handler address of the user application
     * from the location FLASH_SECTOR2_BASE_ADDRESS+4
     */
    uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

    app_reset_handler = (void*) resethandler_address;

    sprintf(payLoad,"BL_DEBUG_MSG: app reset handler addr : %#x\n",app_reset_handler);
    usart_send_bytes(payLoad,sizeof(payLoad));

    //3. jump to reset handler of the user application
    app_reset_handler();

}

/*Helper function to handle BL_GET_VER command */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;

    // 1) verify the checksum
    uint8_t payLoad[100];
    memset(payLoad,0,100);
      sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_getver_cmd\r\n");
      usart_send_bytes(payLoad,sizeof(payLoad));
	 //Total length of the command packet
	  uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

#if(CRC_ENABLED)
	  //extract the CRC32 sent by the Host
	  uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

    if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
    {
    	sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n");
        usart_send_bytes(payLoad,sizeof(payLoad));
        //checksum is wrong send nack
        bootloader_send_nack();

    }else
    {
    	sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n");
    	usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        // checksum is correct..
        bootloader_send_ack(bl_rx_buffer[0],1);
        bl_version=get_bootloader_version();
        sprintf(payLoad,"BL_DEBUG_MSG:BL_VER : %d %#x \r\n",bl_version,bl_version);
        usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_uart_write_data(&bl_version,1);
#if(CRC_ENABLED)
    }
#endif


}

/*This function sends ACK if CRC matches along with "len to follow"*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	 //here we send 2 byte.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
//	HAL_UART_Transmit(C_UART,ack_buf,2,HAL_MAX_DELAY);
	usart_send_bytes(ack_buf,sizeof(ack_buf));

}

void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
//	HAL_UART_Transmit(C_UART,pBuffer,len,HAL_MAX_DELAY);
	usart_send_bytes(pBuffer,len);
}

/*This function sends NACK */
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
//	HAL_UART_Transmit(C_UART,&nack,1,HAL_MAX_DELAY);
	usart_send_bytes(nack,sizeof(nack));
}


/*Helper function to handle BL_GET_HELP command
 * Bootloader sends out All supported Command codes
 */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
	uint8_t payLoad[100];
	sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\n");
    usart_send_bytes(payLoad,sizeof(payLoad));
	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

#if (CRC_ENABLED)
	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    	sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n");
        usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_send_nack();
	}else
	{
    	sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n");
        usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        bootloader_send_ack(pBuffer[0],sizeof(supported_commands));
        bootloader_uart_write_data(supported_commands,sizeof(supported_commands) );
#if (CRC_ENABLED)
	}
#endif
}

/*Helper function to handle BL_GET_CID command */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
	uint16_t bl_cid_num = 0;
	uint8_t payLoad[100];
    sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_getcid_cmd\n");
    usart_send_bytes(payLoad,sizeof(payLoad));

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
#if (CRC_ENABLED)
	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n");
	    usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        bootloader_send_ack(pBuffer[0],2);
        bl_cid_num = get_mcu_chip_id();
	    sprintf(payLoad,"BL_DEBUG_MSG:MCU id : %d %#x !!\n",bl_cid_num, bl_cid_num);
	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_uart_write_data((uint8_t *)&bl_cid_num,2);
#if (CRC_ENABLED)
	}else
	{
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n");
	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_send_nack();
	}
#endif
}

/*Helper function to handle BL_GET_RDP_STATUS command */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
    uint8_t rdp_level = 0x00;
    uint8_t payLoad[100];
    sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\n");
    usart_send_bytes(payLoad,sizeof(payLoad));

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
#if (CRC_ENABLED)
	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n");
	    usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        bootloader_send_ack(pBuffer[0],1);
        rdp_level = get_flash_rdp_level();
	    sprintf(payLoad,"BL_DEBUG_MSG:RDP level: %d %#x\n",rdp_level,rdp_level);
	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_uart_write_data(&rdp_level,1);
#if (CRC_ENABLED)
	}else
	{
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n");
	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_send_nack();
	}
#endif

}


/*Helper function to handle BL_GO_TO_ADDR command */
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
    uint32_t go_address=0;
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;
    uint8_t payLoad[100];
    sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_go_cmd\n");
    usart_send_bytes(payLoad,sizeof(payLoad));

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
#if (CRC_ENABLED)
	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if ( bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n");
	    usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        bootloader_send_ack(pBuffer[0],1);

        //extract the go address
        go_address = *((uint32_t *)&pBuffer[2] );
	    sprintf(payLoad,"BL_DEBUG_MSG:GO addr: %#x\n",go_address);
	    usart_send_bytes(payLoad,sizeof(payLoad));
        if( verify_address(go_address) == ADDR_VALID )
        {
            //tell host that address is fine
            bootloader_uart_write_data(&addr_valid,1);

            /*jump to "go" address.
            we dont care what is being done there.
            host must ensure that valid code is present over there
            Its not the duty of bootloader. so just trust and jump */

            /* Not doing the below line will result in hardfault exception for ARM cortex M */
            //watch : https://www.youtube.com/watch?v=VX_12SjnNhY

            go_address+=1; //make T bit =1

            void (*lets_jump)(void) = (void *)go_address;
    	    sprintf(payLoad,"BL_DEBUG_MSG: jumping to go address! \n");
    	    usart_send_bytes(payLoad,sizeof(payLoad));
            lets_jump();

		}else
		{
    	    sprintf(payLoad,"BL_DEBUG_MSG:GO addr invalid ! \n");
    	    usart_send_bytes(payLoad,sizeof(payLoad));
            //tell host that address is invalid
            bootloader_uart_write_data(&addr_invalid,1);
		}
#if (CRC_ENABLED)
	}else
	{
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n");
	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_send_nack();
	}
#endif

}


/*Helper function to handle BL_FLASH_ERASE command */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
#if FLASH_DRIVER_DEFINED
    uint8_t erase_status = 0x00;
    uint8_t payLoad[100];
    sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_flash_erase_cmd\n" );
    usart_send_bytes(payLoad,sizeof(payLoad));

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
#if (CRC_ENABLED)
	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n" );
	    usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        bootloader_send_ack(pBuffer[0],1);
	    sprintf(payLoad,"BL_DEBUG_MSG:initial_sector : %d  no_ofsectors: %d\n",pBuffer[2],pBuffer[3] );
	    usart_send_bytes(payLoad,sizeof(payLoad));

        GPIO_SetReset_Pin(GPIOA, GPIO_PIN5,1);
        erase_status = execute_flash_erase(pBuffer[2] , pBuffer[3]);
        GPIO_SetReset_Pin(GPIOA, GPIO_PIN5,0);

	    sprintf(payLoad,"BL_DEBUG_MSG: flash erase status: %#x\n",erase_status );
	    usart_send_bytes(payLoad,sizeof(payLoad));

        bootloader_uart_write_data(&erase_status,1);
#if (CRC_ENABLED)
	}else
	{
        printmsg();
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n");
	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_send_nack();
	}
#endif
#endif
}



/*Helper function to handle BL_MEM_WRITE command */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
#if FLASH_DRIVER_DEFINED
	uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	uint8_t chksum =0, len=0;
	len = pBuffer[0];
	uint8_t payload_len = pBuffer[6];

	uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );

	chksum = pBuffer[len];
	uint8_t payLoad[100];
    sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n" );
    usart_send_bytes(payLoad,sizeof(payLoad));

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
#if (CRC_ENABLED)
	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;


	if (bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n" );
	    usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        bootloader_send_ack(pBuffer[0],1);

	    sprintf(payLoad,"BL_DEBUG_MSG: mem write address : %#x\n",mem_address );
	    usart_send_bytes(payLoad,sizeof(payLoad));

		if( verify_address(mem_address) == ADDR_VALID )
		{

    	    sprintf(payLoad,"BL_DEBUG_MSG: valid mem write address\n" );
    	    usart_send_bytes(payLoad,sizeof(payLoad));

            //glow the led to indicate bootloader is currently writing to memory
            GPIO_SetReset_Pin(GPIOA,GPIO_PIN5, GPIO_PIN_SET);

            //execute mem write
            write_status = execute_mem_write(&pBuffer[7],mem_address, payload_len);

            //turn off the led to indicate memory write is over
            GPIO_SetReset_Pin(GPIOA, GPIO_PIN5, GPIO_PIN_RESET);

            //inform host about the status
            bootloader_uart_write_data(&write_status,1);

		}else
		{
    	    sprintf(payLoad,"BL_DEBUG_MSG: invalid mem write address\n" );
    	    usart_send_bytes(payLoad,sizeof(payLoad));
            write_status = ADDR_INVALID;
            //inform host that address is invalid
            bootloader_uart_write_data(&write_status,1);
		}

#if (CRC_ENABLED)
	}else
	{
	    sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n" );
	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_send_nack();
	}
#endif
#endif
}




/*Helper function to handle BL_EN_RW_PROTECT  command */
void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
#if FLASH_DRIVER_DEFINED
    uint8_t status = 0x00;
    uint8_t payLoad[100];
	    sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_endis_rw_protect\n" );
	    usart_send_bytes(payLoad,sizeof(payLoad));

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
#if (CRC_ENABLED)
	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
  	    sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n" );
  	    usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        bootloader_send_ack(pBuffer[0],1);

        status = configure_flash_sector_rw_protection(pBuffer[2] , pBuffer[3],0);

  	    sprintf(payLoad,"BL_DEBUG_MSG: flash erase status: %#x\n",status );
  	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_uart_write_data(&status,1);
#if (CRC_ENABLED)
	}else
	{
  	    sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n" );
  	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_send_nack();
	}

#endif
#endif
}




/*Helper function to handle BL_EN_RW_PROTECT  command */
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
#if FLASH_DRIVER_DEFINED
    uint8_t status = 0x00;
    uint8_t payLoad[100];
	  sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_dis_rw_protect\n" );
	  usart_send_bytes(payLoad,sizeof(payLoad));
    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
#if (CRC_ENABLED)
	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
  	    sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n" );
  	    usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        bootloader_send_ack(pBuffer[0],1);

        status = configure_flash_sector_rw_protection(0,0,1);
  	    sprintf(payLoad,"BL_DEBUG_MSG: flash erase status: %#x\n",status );
  	    usart_send_bytes(payLoad,sizeof(payLoad));

        bootloader_uart_write_data(&status,1);
#if (CRC_ENABLED)
	}else
	{
  	    sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n" );
  	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_send_nack();
	}
#endif
#endif
}



/*Helper function to handle BL_MEM_READ command */
void bootloader_handle_mem_read (uint8_t *pBuffer)
{


}


/*Helper function to handle _BL_READ_SECTOR_P_STATUS command */
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{
#if FLASH_DRIVER_DEFINED
	 uint16_t status;
	 uint8_t payLoad[100];
	  sprintf(payLoad,"BL_DEBUG_MSG:bootloader_handle_read_sector_protection_status\n" );
	  usart_send_bytes(payLoad,sizeof(payLoad));
    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;
#if (CRC_ENABLED)
	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
        printmsg();
  	    sprintf(payLoad,"BL_DEBUG_MSG:checksum success !!\n" );
  	    usart_send_bytes(payLoad,sizeof(payLoad));
#endif
        bootloader_send_ack(pBuffer[0],2);
        status=read_OB_rw_protection_status();
  	    sprintf(payLoad,"BL_DEBUG_MSG: nWRP status: %#x\n",status );
  	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_uart_write_data((uint8_t*)&status,2);
#if (CRC_ENABLED)
	}else
	{
  	    sprintf(payLoad,"BL_DEBUG_MSG:checksum fail !!\n" );
  	    usart_send_bytes(payLoad,sizeof(payLoad));
        bootloader_send_nack();
	}
#endif
#endif
}


/*Helper function to handle BL_OTP_READ command */
void bootloader_handle_read_otp(uint8_t *pBuffer)
{


}

//Just returns the macro value .
uint8_t get_bootloader_version()
{
  return (uint8_t)BL_VERSION;
}

//Read the chip identifier or device Identifier
uint16_t get_mcu_chip_id()
{
/*
	The STM32F446xx MCUs integrate an MCU ID code. This ID identifies the ST MCU partnumber
	and the die revision. It is part of the DBG_MCU component and is mapped on the
	external PPB bus (see Section 33.16 on page 1304). This code is accessible using the
	JTAG debug pCat.2ort (4 to 5 pins) or the SW debug port (two pins) or by the user software.
	It is even accessible while the MCU is under system reset. */
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return  cid;

}

//This verifies the CRC of the given buffer in pData .
#if (CRC_ENABLED)
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue=0xff;

    for (uint32_t i=0 ; i < len ; i++)
	{
        uint32_t i_data = pData[i];
        uwCRCValue = accumulate_crc(&i_data,1);
	}

	 /* Reset CRC Calculation Unit */
    crc_reset();

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}
#endif


/*This function reads the RDP ( Read protection option byte) value
 *For more info refer "Table 9. Description of the option bytes" in stm32f446xx RM
 */
uint8_t get_flash_rdp_level(void)
{

	uint8_t rdp_status=0;
#if 0
	FLASH_OBProgramInitTypeDef  ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
#else

	 volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
	 rdp_status =  (uint8_t)(*pOB_addr >> 8) ;
#endif

	return rdp_status;

}

//verify the address sent by the host .
uint8_t verify_address(uint32_t go_address)
{
	//so, what are the valid addresses to which we can jump ?
	//can we jump to system memory ? yes
	//can we jump to sram1 memory ?  yes
	//can we jump to sram2 memory ? yes
	//can we jump to backup sram memory ? yes
	//can we jump to peripheral memory ? its possible , but dont allow. so no
	//can we jump to external memory ? yes.

//incomplete -poorly written .. optimize it
	if ( go_address >= SRAM1_BASE && go_address <= SRAM1_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)
	{
		return ADDR_VALID;
	}
	else
		return ADDR_INVALID;
}

#if FLASH_DRIVER_DEFINED
 uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector)
{
    //we have totally 8 sectors in STM32F446RE mcu .. sector[0 to 7]
	//number_of_sector has to be in the range of 0 to 7
	// if sector_number = 0xff , that means mass erase !
	//Code needs to modified if your MCU supports more flash sectors
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;


	if( number_of_sector > 8 )
		return INVALID_SECTOR;

	if( (sector_number == 0xff ) || (sector_number <= 7) )
	{
		if(sector_number == (uint8_t) 0xff)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}else
		{
		    /*Here we are just calculating how many sectors needs to erased */
			uint8_t remanining_sector = 8 - sector_number;
            if( number_of_sector > remanining_sector)
            {
            	number_of_sector = remanining_sector;
            }
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number; // this is the initial sector
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;

		/*Get access to touch the flash registers */
		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // our mcu will work on this voltage range
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}


	return INVALID_SECTOR;
}

/*This function writes the contents of pBuffer to  "mem_address" byte by byte */
//Note1 : Currently this function supports writing to Flash only .
//Note2 : This functions does not check whether "mem_address" is a valid address of the flash range.
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status=HAL_OK;

    //We have to unlock flash module to get control of registers
    HAL_FLASH_Unlock();

    for(uint32_t i = 0 ; i <len ; i++)
    {
        //Here we program the flash byte by byte
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,pBuffer[i] );
    }

    HAL_FLASH_Lock();

    return status;
}


/*
Modifying user option bytes
To modify the user option value, follow the sequence below:
1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
FLASH_SR register
2. Write the desired option value in the FLASH_OPTCR register.
3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
4. Wait for the BSY bit to be cleared.
*/
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable)
{
    //First configure the protection mode
    //protection_mode =1 , means write protect of the user flash sectors
    //protection_mode =2, means read/write protect of the user flash sectors
    //According to RM of stm32f446xx TABLE 9, We have to modify the address 0x1FFF C008 bit 15(SPRMOD)

	 //Flash option control register (OPTCR)
    volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

	  if(disable)
		{

			//disable all r/w protection on sectors

			//Option byte configuration unlock
			HAL_FLASH_OB_Unlock();

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			//clear the 31st bit (default state)
			//please refer : Flash option control register (FLASH_OPTCR) in RM
			*pOPTCR &= ~(1 << 31);

			//clear the protection : make all bits belonging to sectors as 1
			*pOPTCR |= (0xFF << 16);

			//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
			*pOPTCR |= ( 1 << 1);

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			HAL_FLASH_OB_Lock();

			return 0;

		}

	   if(protection_mode == (uint8_t) 1)
    {
           //we are putting write protection on the sectors encoded in sector_details argument

			//Option byte configuration unlock
			HAL_FLASH_OB_Unlock();

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			//here we are setting just write protection for the sectors
			//clear the 31st bit
			//please refer : Flash option control register (FLASH_OPTCR) in RM
			*pOPTCR &= ~(1 << 31);

			//put write protection on sectors
			*pOPTCR &= ~ (sector_details << 16);

			//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
			*pOPTCR |= ( 1 << 1);

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			HAL_FLASH_OB_Lock();
		}

		else if (protection_mode == (uint8_t) 2)
    {
	  	//Option byte configuration unlock
			HAL_FLASH_OB_Unlock();

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			//here wer are setting read and write protection for the sectors
			//set the 31st bit
			//please refer : Flash option control register (FLASH_OPTCR) in RM
			*pOPTCR |= (1 << 31);

			//put read and write protection on sectors
            *pOPTCR &= ~(0xff << 16);
			*pOPTCR |= (sector_details << 16);

			//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
			*pOPTCR |= ( 1 << 1);

			//wait till no active operation on flash
			while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

			HAL_FLASH_OB_Lock();
    }

		return 0;
}

uint16_t read_OB_rw_protection_status(void)
{
    //This structure is given by ST Flash driver to hold the OB(Option Byte) contents .
	FLASH_OBProgramInitTypeDef OBInit;

	//First unlock the OB(Option Byte) memory access
	HAL_FLASH_OB_Unlock();
	//get the OB configuration details
	HAL_FLASHEx_OBGetConfig(&OBInit);
	//Lock back .
	HAL_FLASH_Lock();

	//We are just interested in r/w protection status of the sectors.
	return (uint16_t)OBInit.WRPSector;

}

#endif
