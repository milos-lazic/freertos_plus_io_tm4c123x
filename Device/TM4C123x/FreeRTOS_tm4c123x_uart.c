/*
 * FreeRTOS+IO V1.0.1 (C) 2012 Real Time Engineers ltd.
 *
 * FreeRTOS+IO is an add-on component to FreeRTOS.  It is not, in itself, part 
 * of the FreeRTOS kernel.  FreeRTOS+IO is licensed separately from FreeRTOS, 
 * and uses a different license to FreeRTOS.  FreeRTOS+IO uses a dual license
 * model, information on which is provided below:
 *
 * - Open source licensing -
 * FreeRTOS+IO is a free download and may be used, modified and distributed
 * without charge provided the user adheres to version two of the GNU General
 * Public license (GPL) and does not remove the copyright notice or this text.
 * The GPL V2 text is available on the gnu.org web site, and on the following
 * URL: http://www.FreeRTOS.org/gpl-2.0.txt
 *
 * - Commercial licensing -
 * Businesses and individuals who wish to incorporate FreeRTOS+IO into
 * proprietary software for redistribution in any form must first obtain a low
 * cost commercial license - and in-so-doing support the maintenance, support
 * and further development of the FreeRTOS+IO product.  Commercial licenses can
 * be obtained from http://shop.freertos.org and do not require any source files
 * to be changed.
 *
 * FreeRTOS+IO is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+IO unless you agree that you use the software 'as is'.
 * FreeRTOS+IO is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/FreeRTOS-Plus
 *
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* IO library includes. */
#include "FreeRTOS_IO.h"
#include "IOUtils_Common.h"
#include "FreeRTOS_uart.h"

/*-----------------------------------------------------------*/


/* Local function declarations. */
static size_t __attribute__ ((noinline)) prvUART_send( const void * pvBaseAddress, const char * const pcBuffer, const size_t xBytes );
static size_t __attribute__ ((noinline)) prvUART_recv( const void * pvBaseAddress, char * const pcBuffer, const size_t xBytes);

/*-----------------------------------------------------------*/

/* Stores the transfer control structures that are currently in use by the
supported UART ports. */
static Transfer_Control_t *pxTxTransferControlStructs[ boardNUM_UARTS ] = { NULL };
static Transfer_Control_t *pxRxTransferControlStructs[ boardNUM_UARTS ] = { NULL };

/* Stores the IRQ numbers of the supported UART ports. */
static const IRQn_Type xIRQ[] = { UART0_IRQn, UART1_IRQn, UART2_IRQn, UART3_IRQn, UART4_IRQn, UART5_IRQn, UART6_IRQn, UART7_IRQn };

/*-----------------------------------------------------------*/

portBASE_TYPE __attribute__((optimize("O0"))) FreeRTOS_UART_open( Peripheral_Control_t * const pxPeripheralControl )
{
	portBASE_TYPE xReturn;
	const uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );
	
	/* Sanity check the peripheral number. */
	if ( cPeripheralNumber < boardNUM_UARTS )
	{
		pxPeripheralControl->read = FreeRTOS_UART_read;
		pxPeripheralControl->write = FreeRTOS_UART_write;
		pxPeripheralControl->ioctl = FreeRTOS_UART_ioctl;

		taskENTER_CRITICAL();
		{
			/* Configure the pins for the UART being used. */
			boardCONFIGURE_UART_PINS( cPeripheralNumber);
			/* Configure the UART module being used. */
			boardCONFIGURE_UART( cPeripheralNumber);
		}
		taskEXIT_CRITICAL();

		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}


size_t __attribute__((optimize("O0"))) FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
	Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
	size_t xReturn = 0U;
	int8_t cPeripheralNumber;

		if( diGET_TX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
		{
			#if ioconfigUSE_UART_POLLED_TX == 1
			{
				/* No FreeRTOS objects exist to allow transmission without blocking
				the	task, so just send out by polling.  No semaphore or queue is
				used here, so the application must ensure only one task attempts to
				make a polling write at a time. */
				xReturn = prvUART_send( diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl), 
									(const char * const) pvBuffer, xBytes);
			}
			#endif /* ioconfigUSE_UART_POLLED_TX */
		}
		else
		{

		}

	return xReturn;
}


size_t __attribute__((optimize("O0"))) FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{
	Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
	int8_t cPeripheralNumber;
	size_t xReturn = 0U;

		if ( diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
		{
			#if ioconfigUSE_UART_POLLED_RX == 1
			{
				/* No FreeRTOS objects exist to allow reception without blocking
				the task, so just receive by polling.  No semaphore or queue is
				used here, so the application must ensure only one task attempts
				to make a polling read at a time. */
				xReturn = prvUART_recv( diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl),
									(char * const) pvBuffer, xBytes);
			}
			#endif
		}
		else
		{
			/* Sanity check the peripheral number. */
			configASSERT( diGET_PERIPHERAL_NUMBER( pxPeripheralControl ) < boardNUM_UARTS );

			switch( diGET_RX_TRANSFER_TYPE( pxPeripheralControl ) )
			{
				case ioctlUSE_CIRCULAR_BUFFER_RX:

					#if ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1
					{
						/* There is nothing to prevent multiple tasks attempting to
						read the circular buffer at any one time.  The implementation
						of the circular buffer uses a semaphore to indicate when new
						data is available, and the semaphore will ensure that only the
						highest priority task that is attempting a read will actually
						receive bytes. */
						ioutilsRECEIVE_CHARS_FROM_CIRCULAR_BUFFER(
							pxPeripheralControl,
							UARTIntDisable( diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl), UART_INT_RX | UART_INT_RT),
							UARTIntEnable( diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl), UART_INT_RX | UART_INT_RT),
							( ( uint8_t * ) pvBuffer ),
							xBytes,
							xReturn);
					}
					#endif
					break;

				case ioctlUSE_CHARACTER_QUEUE_RX:

					#if ioconfigUSE_UART_RX_CHAR_QUEUE == 1
					{

					}
					#endif
					break;

				default:
					/* Other RX methods can be implemented here. */
					break;
			}
		}

	return xReturn;
}


portBASE_TYPE __attribute__((optimize("O0"))) FreeRTOS_UART_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue )
{
	Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
	uint32_t ulValue = ( uint32_t ) pvValue;
	const int8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
	portBASE_TYPE xReturn = pdPASS;

		/* Sanity check the peripheral number. */
		configASSERT( cPeripheralNumber < boardNUM_UARTS );

		taskENTER_CRITICAL();
		{
			switch( ulRequest )
			{
				case ioctlUSE_INTERRUPTS :
					
					if( ulValue == pdFALSE )
					{
						/* Disable interrupts. */
						NVIC_DisableIRQ( xIRQ[ cPeripheralNumber ] );
					}
					else
					{
						/* Enable interrupts. */
						NVIC_EnableIRQ( xIRQ[ cPeripheralNumber ] );

						/* If the Rx is configured to use interrupts, remember the
						transfer control structure that should be used.  A reference
						to the Tx transfer control structure is taken when a write()
						operation is actually performed. */
						pxRxTransferControlStructs[ cPeripheralNumber ] = pxPeripheralControl->pxRxControl;
					}
					break;

				case ioctlSET_SPEED :
					break;

				case ioctlSET_INTERRUPT_PRIORITY :
					break;

				default :
					xReturn = pdFAIL;
					break;
			}
		}
		taskEXIT_CRITICAL();

	return xReturn;
}


/** prvUART_send: perfroms a blocking UART write */
/** TODO: move to BSP package */
static size_t __attribute__ ((noinline)) prvUART_send( const void * pvBaseAddress, const char * const pcBuffer, const size_t xBytes )
{
	const uint32_t ulBaseAddress = (const uint32_t) pvBaseAddress;
	size_t xReturn = 0U;
	uint32_t ulIdx = 0U;

	while( ulIdx < xBytes)
	{
		UARTCharPut( ulBaseAddress, pcBuffer[ulIdx]);
		ulIdx++;
		xReturn++;
	}

	return xReturn;
}


/** prvUART_recv: performs a blocking UART read */
/** TODO: move to BSP package */
static size_t __attribute__ ((noinline)) prvUART_recv( const void * pvBaseAddress, char * const pcBuffer, const size_t xBytes)
{
	const uint32_t ulBaseAddress = (const uint32_t) pvBaseAddress;
	size_t xReturn = 0U;
	uint32_t ulIdx = 0U;

	while( ulIdx < xBytes)
	{
		pcBuffer[ulIdx] = UARTCharGet( ulBaseAddress);
		ulIdx++;
		xReturn++;
	}

	return xReturn;
}


#if ioconfigINCLUDE_UART == 1
	/* If the UART driver is being used, include the interrupt handler. */
void  __attribute__ ((optimize("O0"))) UART0_Handler( void )
{
	uint32_t ulInterruptSource;
	volatile uint32_t ulReceived;
	const uint32_t ulRxInterrupts = ( UART_INT_RT | UART_INT_RX );
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	const unsigned portBASE_TYPE uxUARTNumber = 0UL;
	Transfer_Control_t * pxTransferStruct;

	/* Determine the interrupt source. */
	ulInterruptSource = UARTIntStatus( UART0_BASE, true);

	if ( ( ulInterruptSource & ulRxInterrupts ) != 0UL )
	{
		pxTransferStruct = pxRxTransferControlStructs[ uxUARTNumber ];
		if ( pxTransferStruct != NULL )
		{
			switch( diGET_TRANSFER_TYPE_FROM_CONTROL_STRUCT( pxTransferStruct ) )
			{
				case ioctlUSE_CIRCULAR_BUFFER_RX :

					#if ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1
					{
						ioutilsRX_CHARS_INTO_CIRCULAR_BUFFER_FROM_ISR(
							pxTransferStruct, /* The structure that contains the reference to the circular buffer. */
							UARTCharsAvail( UART0_BASE ),
							UARTCharGet( UART0_BASE ),
							ulReceived,
							xHigherPriorityTaskWoken);
					}
					#endif /* ioconfigUSE_UART_CIRCULAR_BUFFER_RX */
					break;


				case ioctlUSE_CHARACTER_QUEUE_RX :

					#if ioconfigUSE_UART_RX_CHAR_QUEUE == 1
					{

					}
					#endif /* ioconfigUSE_UART_RX_CHAR_QUEUE */
					break;


				default :

					/* This must be an error.  Force an assert. */
					configASSERT( xHigherPriorityTaskWoken );
					break;
			}
		}
	}

	/* The ulReceived parameter is not used by the UART ISR. */
	( void ) ulReceived;

	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	switch should be performed before the interrupt exists.  That ensures the
	unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}



void  __attribute__ ((optimize("O0"))) UART1_Handler( void )
{
	uint32_t ulInterruptSource;
	volatile uint32_t ulReceived;
	const uint32_t ulRxInterrupts = ( UART_INT_RT | UART_INT_RX );
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	const unsigned portBASE_TYPE uxUARTNumber = 1UL;
	Transfer_Control_t * pxTransferStruct;

	/* Determine the interrupt source. */
	ulInterruptSource = UARTIntStatus( UART1_BASE, true);

	if ( ( ulInterruptSource & ulRxInterrupts ) != 0UL )
	{
		pxTransferStruct = pxRxTransferControlStructs[ uxUARTNumber ];
		if ( pxTransferStruct != NULL )
		{
			switch( diGET_TRANSFER_TYPE_FROM_CONTROL_STRUCT( pxTransferStruct ) )
			{
				case ioctlUSE_CIRCULAR_BUFFER_RX :

					#if ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1
					{
						ioutilsRX_CHARS_INTO_CIRCULAR_BUFFER_FROM_ISR(
							pxTransferStruct, /* The structure that contains the reference to the circular buffer. */
							UARTCharsAvail( UART1_BASE ),
							UARTCharGet( UART1_BASE ),
							ulReceived,
							xHigherPriorityTaskWoken);
					}
					#endif /* ioconfigUSE_UART_CIRCULAR_BUFFER_RX */
					break;


				case ioctlUSE_CHARACTER_QUEUE_RX :

					#if ioconfigUSE_UART_RX_CHAR_QUEUE == 1
					{

					}
					#endif /* ioconfigUSE_UART_RX_CHAR_QUEUE */
					break;


				default :

					/* This must be an error.  Force an assert. */
					configASSERT( xHigherPriorityTaskWoken );
					break;
			}
		}
	}

	/* The ulReceived parameter is not used by the UART ISR. */
	( void ) ulReceived;

	/* If lHigherPriorityTaskWoken is now equal to pdTRUE, then a context
	switch should be performed before the interrupt exists.  That ensures the
	unblocked (higher priority) task is returned to immediately. */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
#endif
