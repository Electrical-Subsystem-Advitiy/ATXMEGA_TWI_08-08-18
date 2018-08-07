/*
 * I2C_INTERRUPR_XMEGA.c
 *
 * Created: 13-05-2018 19:14:31
 * Author : PRASHANT KURREY
 */ 

#include <avr/io.h>




#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "usart_driver.h"
/*! Defining an example slave address. */
#define SLAVE_ADDRESS    0x55

/*! Defining number of bytes in buffer. */
#define NUM_BYTES        8

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED       2000000
#define BAUDRATE	9600
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)


/* Global variables */
TWI_Master_t twiMaster;    /*!< TWI master module. */


/*! Define that selects the Usart used in example. */
#define USART USARTC0

/*! USART data struct used in example. */
USART_data_t USART_data;




/*! Buffer with test data to send.*/
uint8_t sendBuffer[NUM_BYTES] = {'P','R','A', 'S','H','A', 'N','T'};


int main(void)
{
	
	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins are used. */
  	/* PC3 (TXD0) as output. */
	PORTC.DIRSET   = PIN3_bm;
	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR   = PIN2_bm;

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock frequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);
	
	// Enable internal pull-up on PC0, PC1.. Uncomment if you don't have external pullups
	PORTCFG.MPCMASK = 0x03; // Configure several PINxCTRL registers at the same time
	PORTC.PIN0CTRL = (PORTC.PIN0CTRL & ~PORT_OPC_gm) | PORT_OPC_PULLUP_gc; //Enable pull-up to get a defined level on the switches

	

	/* Initialize TWI master. */
	TWI_MasterInit(&twiMaster,
	               &TWIC,
	               TWI_MASTER_INTLVL_LO_gc,
	               TWI_BAUDSETTING);

	
	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	sei();
uint8_t BufPos = 0;
	
while(true){
	TWI_MasterInit(&twiMaster,
	&TWIC,
	TWI_MASTER_INTLVL_LO_gc,
	TWI_BAUDSETTING);
	TWI_MasterWriteRead(&twiMaster,                                      //added by me
	SLAVE_ADDRESS,
	&sendBuffer[BufPos],
	8,
	6);
	


	while (twiMaster.status != TWIM_STATUS_READY) {
		// Wait until transaction is complete.
	}


	//UART_TXBuffer_PutByte(&USART_data, 'A');	                     // send data
	//_delay_ms(500);
	};
	
	}



/*! TWIC Master Interrupt vector. */
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}

/*  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTC0_RXC_vect)
{
	int receive=0;
	USART_RXComplete(&USART_data);
	if (USART_RXBufferData_Available(&USART_data)) {                                               // modified by  me
	receive = USART_RXBuffer_GetByte(&USART_data);}                  // receive the data      // modified
	UART_TXBuffer_PutByte(&USART_data, receive);	                     // send data
	
}


/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}
