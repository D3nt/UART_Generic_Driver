#pragma once
/// Debugging purposes only
#define SW_SIMULATION_DEBUG_ONLY
/// Local H function to define the data types to be used in simulation

/// Defines used to configure the UART
#define UART_CLOCK_RATE         ( 0x123)
#define UART_SELECTED_BAUD_RATE ( 4096 )
#define UART_DATA_BITS          ( 8 )
#define UART_SELECTED_PARITY    ( 1 )
#define UART_STOP_BITS          ( 1 )
#define UART_MAX_DATA_BYTES     ( 8 ) /// Limitation that depends on HW

/// HW protocol limitations
#define MAX_UART_SENT_TIMEOUT_COUNT		( 100 )
#define MAX_UART_RECEIVE_TIMEOUT_COUNT	( 200 )
/// Defines for UART Transmit Queue Buffer
#ifdef REAL_HW
#define UART_TRANSMIT_QUEUE_START            (0x10000000) /// This would be the real address of the UART FIFO
#define UART_RX_FIFO_FLOW_CONTROL            (0x10000010) /// This would be the real address of the UART RX FIFO Flow Control
#define UART_TX_FIFO_FLOW_CONTROL            (0x10000020) /// This would be the real address of the UART TX FIFO Flow Control
#define UART_CONTROL_REGISTER                (0x10000030) /// This would be the real address of the UART Control
#define UART_TRANSMIT_QUEUE_SIZE_BYTES       (4096)
#define UART_RECEIVE_QUEUE_SIZE_BYTES		 (4096)
#define READ_REGISTER  (reg_address)         (*(HW_REG*)(void*)(reg_address))
#define WRITE_REGISTER (reg_address, value)  ((*(HW_REG*)(void*)(reg_address)) = (value))
#else ///--> SW_SIMULATION_DEBUG_ONLY
#define UART_RX_FIFO_FLOW_CONTROL          ((uint32_t)(0x10000010)) /// Simulated Value
#define UART_TX_FIFO_FLOW_CONTROL          ((uint32_t)(0x10000020)) /// Simulated Value
#define UART_CONTROL_REGISTER              ((uint32_t)(0x10000030)) /// Simulated Value
#define UART_TRANSMIT_QUEUE_SIZE_BYTES     (4096)
#define UART_TRANSMIT_QUEUE_START          (&uartFIFO[0])
#define UART_RECEIVE_QUEUE_SIZE_BYTES      (4096)
uint32_t READ_REGISTER(uint32_t reg_address);
void     WRITE_REGISTER(uint32_t reg_address, uint32_t value);
#endif //REAL_HW

#define BIT_SET(x)    (1<<x)
#define BIT_1_SET     (0x0002)
#define BIT_0_SET	  (0x0001)

/// Status of the UART messages
typedef enum //tagUART_STATUS
{
	UART_OK =			0x00,
	UART_TX_BUSY =		0x01,
	UART_RX_BUSY =		0x02,
	UART_TX_FULL =		0x03,
	UART_RX_FULL =		0x04,
	UART_TX_TIMEOUT =	0x05,
	UART_RX_TIMEOUT =   0x06,
	UART_OTHER_ERROR =	0xFF
} UART_STATUS;

typedef struct
{
   uint32_t* startAddress;
   uint32_t* endAddress;
   uint32_t  bufferSizeBytes;
   uint32_t  txQueueEntriesBytes;
} transmit_queue_buffer_t;

typedef struct
{
    uint32_t clockRate;
    uint32_t baudRate;
    uint32_t dataBits;
    uint32_t parity;
    uint32_t stopBits;
} uart_config_reg_t;
/// uint Declaration is not guaranteed in C89 C - redefined to make sure this works
typedef unsigned int uint;
/// Function prototypes - Internal testing only - to be called from main.c
void uart_init(void);
void uart_send(uint8_t* data, uint length);
uint uart_recv(uint8_t* buffer, uint bufSize);
void uart_interrupt(void);
UART_STATUS uart_tx_isr_handler(void);

#ifdef SW_SIMULATION_DEBUG_ONLY
void init_prototypes(void);
#endif ///SW_SIMULATION_DEBUG_ONLY