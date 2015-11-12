///------------------------------------------------------------------------------
/// Challenge 2:   Please get back to us with the clear / compact / efficiently 
/// written code you can come up for the following requirements.I sincerely 
/// appreciate you taking time to program this.
/// UART driver Write a simple UART FIFO driver in C89(X3.159 - 1989).
/// Assume no library support and use any compiler specific directives or macros you are accustomed to.
/// The driver should provide callers with 2 function calls :
/// > void uart_send(uint8_t* data, uint length);
/// > uint uart_recv(uint8_t* buffer, uint bufSize);
/// The driver should supply an interrupt handler function as well :
/// > void uart_interrupt();
/// The driver should work with 4 hardware registers :
/// > extern char uart_status;
/// > extern char uart_mask;
/// > extern char uart_rd;
/// > extern char uart_td;
/// uart_status indicates the cause of interrupt.When bit 0 is true, 
/// then there is received data available in uart_rd.Bit 0 is cleared by reading uart_rd.
/// When bit 1 is true, then the uart is ready for an outbound byte to be written to uart_td, 
/// which will clear the status bit. uart_mask is an interrupt enable mask.
/// At any time when uart_status & uart_mask is not 0, the interrupt handler will be called.
/// The handler should clear the status bits by reading / writing to the uart, and / or 
/// it should disable interrupts(when there is no more data to be written, for example).
/// uart_rd and uart_td are uart shift registers.
/// The driver should supply the function uart_send, which writes a caller’s data into 
/// a transmit queue and returns immediately.The queue should drain out the uart on the interrupt handler.
/// When data is received from the uart by the interrupt handler, it should be written into a receive queue.
/// The driver should supply the function uart_recv to read from the queue.
///------------------------------------------------------------------------------

/// Global Includes
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <assert.h>
/// Local Includes
#include "uart.h"

#ifdef SW_SIMULATION_DEBUG_ONLY
#include "main.h"
#endif /// SW_SIMULATION_DEBUG_ONLY
/// External UART Registers
/// STATUS Register: Indicates the cause of interrupt
/// Bit 0 - Data available in uart_rd
/// Bit 1 - UART is ready for an outbound byte for uart_td
extern char uart_status; 
/// INT_MASK Register - Only lower 2 bits are used (Rx/Tx)
extern char uart_mask;
/// UART Shift register for Rx
extern char uart_rd;
/// UART Shift register for Tx
extern char uart_td;

#ifdef SW_SIMULATION_DEBUG_ONLY
/// Global variable prototypes
uint32_t uartFIFO[UART_TRANSMIT_QUEUE_SIZE_BYTES];
uart_config_reg_t uart_config;
transmit_queue_buffer_t transmit_queue_buffer;
#endif //SW_SIMULATION_DEBUG_ONLY
///------------------------------------------------------------------------------
///
/// \brief UART Initialization function. Initializes the UART HW (hypothetical)
///
/// @param N/A
///
/// @return N/A
///
///------------------------------------------------------------------------------
void uart_init(void)
{
   /// Initialize UART HW configuration - typical
   uart_config.clockRate = UART_CLOCK_RATE;
   uart_config.baudRate = UART_SELECTED_BAUD_RATE;
   uart_config.dataBits = UART_DATA_BITS;
   uart_config.parity = UART_SELECTED_PARITY;
   uart_config.stopBits = UART_STOP_BITS;
   
   /// Initialize the Transmit Queue Buffer
   transmit_queue_buffer.startAddress = UART_TRANSMIT_QUEUE_START;
   transmit_queue_buffer.endAddress = UART_TRANSMIT_QUEUE_START + UART_TRANSMIT_QUEUE_SIZE_BYTES;
   transmit_queue_buffer.bufferSizeBytes = UART_TRANSMIT_QUEUE_SIZE_BYTES;
   transmit_queue_buffer.txQueueEntriesBytes = 0;
   
   /// Complete any HW specific configuration
   WRITE_REGISTER(UART_RX_FIFO_FLOW_CONTROL, 1); /// Clear the RX Fifo
   WRITE_REGISTER(UART_TX_FIFO_FLOW_CONTROL, 1); /// Clear the TX Fifo
   
   /// Enable the FIFO flow
   WRITE_REGISTER(UART_CONTROL_REGISTER, 1); /// Enable FIFOs
}

///------------------------------------------------------------------------------
///
/// \brief Driver function used to send the data to UART.
///        UART send writes a caller's data into a transmit 
///        queue and returns immediately.
///
/// @param1 uint8_t* data - pointer to the data buffer to be sent.
/// @param2 uint length - length of the data buffer in bytes
///
/// @return N/A
///
///------------------------------------------------------------------------------
void uart_send(uint8_t* data, uint length)
{

   /// Sanity Checks on the inbound call
   assert(data != NULL || length != 0);
   /// Loop through the buffer to send out the characters one by one

   if (length + transmit_queue_buffer.txQueueEntriesBytes > UART_TRANSMIT_QUEUE_SIZE_BYTES)
   {
      /// Queue overflow, need to handle existing buffer items first.
      /// Ideally, we'd want to return some kind of error to the caller (UART_STATUS), asking to try again later
      /// However, the prototype of this function in the assignment doesn't allow that
      /// So, we're going to ASSERT for now.
      assert(0);
   }

   else
   {
       /// Handle the transfer
      while (length)
      {
         /// Place the character in the transmit queue
         uint32_t current_pointer = transmit_queue_buffer.startAddress + transmit_queue_buffer.txQueueEntriesBytes;
         /// Assuming that TX Queue is in HW
         WRITE_REGISTER(current_pointer, *data);
         transmit_queue_buffer.txQueueEntriesBytes++;
		 data++;
         length--;
      }
      /// Before we exit the send, enable the UART Tx Interrupt
      WRITE_REGISTER((uint32_t)uart_mask, BIT_SET(1));
   }
}

///------------------------------------------------------------------------------
///
/// \brief ISR Handler for UART Send (Tx) operation. Checks the transmit queue
///         and ensures that all the data in the transmit queue is sent.
///
/// @param1 uint8_t* data - pointer to the data buffer to be sent.
/// @param2 uint length - length of the data buffer
///
/// @return UART_ERROR - type of error returned
///
///------------------------------------------------------------------------------
UART_STATUS uart_tx_isr_handler(void)
{
	UART_STATUS status = UART_OK;
	uint32_t uart_status_contents;
	uint32_t timeout_count;
	uint32_t transmit_queue_xfer_size = transmit_queue_buffer.txQueueEntriesBytes;

	/// Check if the transmit buffer has entries
	if (transmit_queue_buffer.txQueueEntriesBytes != 0)
	{
		for (uint32_t currChar = 0; currChar < transmit_queue_xfer_size; currChar++)
		{
			timeout_count = 0;
			do
			{
				/// Poll for UART Tx Ready
				uart_status_contents = READ_REGISTER(uart_status);
				if (timeout_count > MAX_UART_SENT_TIMEOUT_COUNT)
				{
					/// Timed-out on the polling
					/// Make sure that the transmitted bytes are accounted for
					transmit_queue_buffer.txQueueEntriesBytes -= currChar;
					return status = UART_TX_TIMEOUT;
				}
				timeout_count++;
			} while ((uart_status_contents & uart_mask) != BIT_1_SET);
			/// Write the character to the register
			WRITE_REGISTER(uart_td, (transmit_queue_buffer.startAddress) + currChar);
			/// Decrement the Queue Size as it is sent - ideally, the HW would handle this.
			transmit_queue_buffer.txQueueEntriesBytes--;
		}
		/// Transfer should have completed here
		assert(transmit_queue_buffer.txQueueEntriesBytes == 0);
	}
	else
	{
		///Handle end-of-transmission (HW dependent)
	}
	return status;
}

///------------------------------------------------------------------------------
///
/// \brief Driver function used to receive the data from UART
///
/// @param1 uint8_t* buffer - pointer to the data buffer to be received.
/// @param2 uint bufSize - length of the data buffer
///
/// @return uint - number of bytes received in this call
///
///------------------------------------------------------------------------------
uint uart_recv(uint8_t* buffer, uint bufSize)
{
   uint received_bytes = 0;
   uint32_t timeout_count;
   uint32_t uart_status_contents;
   /// Read the first byte from the register
   for (uint currChar = 0; currChar < bufSize; currChar++)
   {
	   timeout_count = 0;
	   do
	   {
		   /// Poll for UART Rx Ready
		   uart_status_contents = READ_REGISTER(uart_status);
		   if (timeout_count > MAX_UART_SENT_TIMEOUT_COUNT)
		   {
			   /// Timed-out on the polling
			   /// Ideally, we would like to return a STATUS on a time-out
			   /// and allow the caller to try again, but the assignment
			   /// function prototype doesn't allow this
			   /// return status = UART_RX_TIMEOUT;
			   printf("Receive Timed Out. Bytes Transfered: %d \n", received_bytes);
		   }
		   timeout_count++;
	   } while ((uart_status_contents & uart_mask) != BIT_0_SET);
	   *buffer = READ_REGISTER(uart_rd);
	   /// Move the buffer pointer
	   *buffer++;
	   received_bytes++;
   }
   return received_bytes;
}

///------------------------------------------------------------------------------
///
/// \brief Main Interrupt handler for UART driver. Handles Rx/Tx and error interrupts
///
/// @param N/A
///
/// @return N/A
///
///------------------------------------------------------------------------------
void uart_interrupt(void)
{
	UART_STATUS uartStatus = UART_OK;
	uint32_t status = ((READ_REGISTER(uart_status)) & (READ_REGISTER(uart_mask)));
	/// Disable the interrupts while we're handling the current one:
	DISABLE_INTERRUPTS();
	/// Assuming the priority for the Rx is higher than Tx
	if (status & BIT_1_SET)
	{
		uint8_t receiveBuffer[UART_MAX_DATA_BYTES];
		uint8_t *pReceiveBuffer = &receiveBuffer[0];
		uint bytesReceived;
      
		bytesReceived = uart_recv(pReceiveBuffer, (uint)UART_MAX_DATA_BYTES);
		/// The buffer is received here, but we need to handle the data
		/// Either schedule a call to RTOS (if used), or call the appropriate handler
		/// Since this is not specificed, just display the data in the console
		printf("Receive Interrupt fired. ");
	}

	/// Check Tx Interrupt
	if (status & BIT_0_SET)
	{
		/// Go through the FIFO to send the entire buffer
		uartStatus = uart_tx_isr_handler();
		assert(uartStatus == UART_OK);
	}

	/// Re-enable interrupts
	ENABLE_INTERRUPTS();
}

/// Emulation functions - not needed when using HW
#ifndef SW_SIMULATION_DEBUG_ONLY
void init_prototypes(void)
{
	/// Initialize the extern registers to something
	/// that can be recognized during debug
	uart_status = 0x11;
	uart_mask = 0x22;
	uart_rd = 0x33;
	uart_td = 0x44;
}
uint32_t READ_REGISTER(uint32_t reg_address)
{
	printf("Simulated Read Register to 0x%x Current Contents %x\n", reg_address, (uint32_t)reg_address);
	return (uint32_t)reg_address;
}
void WRITE_REGISTER(uint32_t reg_address, uint32_t value)
{
	printf("Simulated Write Register to 0x%x, Value %x \n", reg_address, value);
}
#endif //SW_SIMULATION_DEBUG_ONLY