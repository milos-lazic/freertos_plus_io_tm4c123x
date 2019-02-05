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

#ifndef TM4C123GH6PM_BASE_BOARD_H
#define TM4C123GH6PM_BASE_BOARD_H

#include <stdbool.h>
#include "FreeRTOS_DriverInterface.h"

/* Header files for all the peripheral driver libraries that can be used with this BSP */
#include "TM4C123GH6PM.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"


#define gpioPIN( x ) (0x00000001 << x)


/*******************************************************************************
 * Definitions used by FreeRTOS+IO to determine the peripherals that are
 * available on the board, and the functions used to interface with the target
 * specific peripheral drivers.
 ******************************************************************************/

/*******************************************************************************
 * Definitions used by the UART-interrupt-driven-command-console.c example file.
 *
 * See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_IO/Board_Support_Packages.shtml
 *
 ******************************************************************************/

#define boardAVAILABLE_DEVICES_LIST											\
{																			\
	{ ( const int8_t * const ) "/UART0/", eUART_TYPE, (void *) UART0 },		\
	{ ( const int8_t * const ) "/UART1/", eUART_TYPE, (void *) UART1 },		\
	{ ( const int8_t * const ) "/UART2/", eUART_TYPE, (void *) UART2 },		\
	{ ( const int8_t * const ) "/UART3/", eUART_TYPE, (void *) UART3 },		\
	{ ( const int8_t * const ) "/UART4/", eUART_TYPE, (void *) UART4 },		\
	{ ( const int8_t * const ) "/UART5/", eUART_TYPE, (void *) UART5 },		\
	{ ( const int8_t * const ) "/UART6/", eUART_TYPE, (void *) UART6 },		\
	{ ( const int8_t * const ) "/UART7/", eUART_TYPE, (void *) UART7 },		\
}

/*******************************************************************************
 * Map the FreeRTOS+IO interface to the LPC17xx specific functions.
 ******************************************************************************/
portBASE_TYPE vFreeRTOS_tm4c123x_PopulateFunctionPointers( const Peripheral_Types_t ePeripheralType, Peripheral_Control_t * const pxPeripheralControl );
#define boardFreeRTOS_PopulateFunctionPointers vFreeRTOS_tm4c123x_PopulateFunctionPointers


/*******************************************************************************
 * These define the number of peripherals available on the microcontroller -
 * not the number of peripherals that are supported by the software
 ******************************************************************************/
#define boardNUM_UARTS				8 /* UART0 to UART7 */


/*******************************************************************************
 * Configure port UART port pins to be correct for the wiring of the
 * TM4C123x base board.
 ******************************************************************************/
#define boardCONFIGURE_UART_PINS( cPeripheralNumber)								\
	switch( (cPeripheralNumber ) )													\
	{																				\
		case 0	:	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA);					\
					while( false == SysCtlPeripheralReady( SYSCTL_PERIPH_GPIOA));	\
					GPIOPinTypeUART( GPIOA_BASE, GPIO_PIN_0 | GPIO_PIN_1);			\
					GPIOPinConfigure( GPIO_PA0_U0RX);								\
					GPIOPinConfigure( GPIO_PA1_U0TX);								\
					break;															\
																					\
		case 1	:	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB);					\
					while( false == SysCtlPeripheralReady( SYSCTL_PERIPH_GPIOB));	\
					GPIOPinTypeUART( GPIOB_BASE, GPIO_PIN_0 | GPIO_PIN_1);			\
					GPIOPinConfigure( GPIO_PB0_U1RX);								\
					GPIOPinConfigure( GPIO_PB1_U1TX);								\
					break;															\
																					\
		default :	/* Not implemented yet or not available */						\
					configASSERT( ( cPeripheralNumber ) - (cPeripheralNumber ) );	\
					break;															\
	}

/*******************************************************************************
 * Set the default baud rate used by a UART.
 ******************************************************************************/
#define boardDEFAULT_UART_BAUD		115200
#define boardDEAFULT_UART_CONFIG	(UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE)	

/*******************************************************************************
 * Configure UART module.
 ******************************************************************************/
#define boardCONFIGURE_UART( cPeripheralNumber)											\
	switch( (cPeripheralNumber) )														\
	{																					\
																						\
		case 0	:	SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0);						\
					while( false == SysCtlPeripheralReady( SYSCTL_PERIPH_UART0));		\
					UARTConfigSetExpClk( UART0_BASE, SysCtlClockGet(),					\
						boardDEFAULT_UART_BAUD, boardDEAFULT_UART_CONFIG);				\
					UARTDisable( UART0_BASE);											\
					UARTFIFOEnable( UART0_BASE);										\
					UARTFIFOLevelSet( UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);	\
					UARTIntEnable( UART0_BASE, UART_INT_RX | UART_INT_RT);				\
					/* Priority must be at or below SYSCALL_MAX_PRIO for interrupts		\
					whose ISR use FreeRTOS APIs. */										\
					NVIC_SetPriority( UART0_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);\
					NVIC_DisableIRQ( UART0_IRQn);										\
					UARTEnable( UART0_BASE);											\
					break;																\
																						\
																						\
		case 1	:	SysCtlPeripheralEnable( SYSCTL_PERIPH_UART1);						\
					while( false == SysCtlPeripheralReady( SYSCTL_PERIPH_UART1));		\
					UARTConfigSetExpClk( UART1_BASE, SysCtlClockGet(),					\
						boardDEFAULT_UART_BAUD, boardDEAFULT_UART_CONFIG);				\
					UARTDisable( UART1_BASE);											\
					UARTFIFOEnable( UART1_BASE);										\
					UARTFIFOLevelSet( UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);	\
					UARTIntEnable( UART1_BASE, UART_INT_RX | UART_INT_RT);				\
					/* Priority must be at or less than SYSCALL_MAX_PRIO for ISRs		\
					that use FreeRTOS APIs. */											\
					NVIC_SetPriority( UART1_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);\
					NVIC_DisableIRQ( UART1_IRQn);										\
					UARTEnable( UART1_BASE);											\
					break;																\
																						\
		default	:	/* Not implemented yet or not available. */							\
					configASSERT( (cPeripheralNumber ) - (cPeripheralNumber ) );		\
					break;																\
	}


#endif