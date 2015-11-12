#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <conio.h>
#include <assert.h>

#include "main.h" /// Prototypes located here
#include "uart.h"

char uart_status;
/// INT_MASK Register - Only lower 2 bits are used (Rx/Tx)
char uart_mask;
/// UART Shift register for Rx
char uart_rd;
/// UART Shift register for Tx
char uart_td;

void main(void)
{
	/// Local Variables for testing
	UART_STATUS  status = UART_OK;
	uint8_t sampleTxBuffer[UART_TRANSMIT_QUEUE_SIZE_BYTES];
	uint8_t sampleRxBuffer[UART_RECEIVE_QUEUE_SIZE_BYTES];
	uint8_t *pSampleTxBuffer = &sampleTxBuffer[0];
	uint8_t *pSampleRxBuffer = &sampleRxBuffer[0];
	uint bytesReceivedTest;
	printf("Sample Test \n");
	/// Initialize Globals
	init_prototypes();

	///Call Test functions
	uart_init();
	/// Build the buffer
	for (int i = 0; i < UART_TRANSMIT_QUEUE_SIZE_BYTES; i++)
	{
		sampleTxBuffer[i] = (uint8_t) i;
	}
	uart_send(pSampleTxBuffer, 10);

	/// Test the TX ISR Handler
	uart_status = 0x02;
	uart_mask = 0x02;
	status = uart_tx_isr_handler();

	/// Test the uart_recv function
	uart_status = 0x01;
	uart_mask = 0x01;
	bytesReceivedTest = uart_recv(pSampleRxBuffer, 10);
	assert(bytesReceivedTest == 10);

	/// Test the UART interrupt function
	uart_interrupt();
/// OPTIONAL TESTS for TIME-OUTs
/*
	/// Test the uart_recv function with time-out
	uart_status = 0x01;
	uart_mask = 0x00;
	bytesReceivedTest = uart_recv(pSampleRxBuffer, 10);

	/// Test TX Time-out
	uart_status = 0x02;
	uart_mask = 0x00;
	status = uart_tx_isr_handler();
*/
	/// Stop to analyze the output
	while (1);
}


