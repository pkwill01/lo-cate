/**
 * @file c_rfm95_driver.h
 * @brief A general c driver for the RFM95 LoRa Module
 *
 * @author pkwill01
 * @date 2024-07-13
 */
#ifndef C_RFM95_DRIVER
#define C_RFM95_DRIVER
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Enumerator for the Lora Registers
 *
 */
typedef enum {
	REG_FIFO = 0x00, /**< base-band FIFO data input/output */
	REG_OP_MODE = 0X01, /**< Operating mode & LoRaTM / FSK selection */
	REG_FR_MSB = 0X06, /**< MSB of RF carrier frequency */
	REG_FR_MIB = 0X07, /**< MIB of RF carrier frequency */
	REG_FR_LSB = 0X08, /**< LSB of RF carrier frequency */
	REG_PA_CONFIG = 0x09, /**< PA selection and Output Power control */
	REG_IRQ_FLAGS = 0x12, /**< Interrupt Flags describing system state */
	REG_RX_NB_BYTES = 0x13, /**< Number of payload bytes of latest packet received */
	REG_LNA   = 0x0c, /**< Low Noise Amplifier settings */
	REG_RX_PTR = 0X0d, /**< SPI interface address pointer in FIFO data buffer */
	REG_FIFO_TX_BASE_ADDR = 0X0e, /**< Write base address in FIFO data buffer for TX modulator*/
	REG_FIFO_RX_BASE_ADDR = 0X0f, /**< Read base address in FIFO data buffer for RX demodulator */
	REG_MODEM_CONFIG_1 = 0X1d, /**< Device header mode */
	REG_PAYLOAD_LENGTH = 0X22, /**< Payload length in bytes. The register needs to be set in implicit
header mode for the expected packet length. */
	REG_MODEM_CONFIG_3 = 0x26, /**< Modem Config settings 3, used for Mobile Node, AGC on */
	REG_VERSION = 0X42, /**< Version code of the chip*/

} LoraRegisters;


/**
 * @brief Masks for the IRQ register
 *
 */
typedef enum {
	IRQ_TX_DONE = 0x08, /**< FIFO Payload transmission complete*/
} IrqRegisterMasks;

/**
 * @brief Enumerator for the LoRa Operating Modes
 *
 */
typedef enum {
	MODE_SLEEP = 0X00, /**< Disable all LoRa functionality and put module to sleep*/
	MODE_STDBY = 0X01, /**< Turn on the top regulator and crystal oscillator */
	MODE_FSTX  = 0X02, /**< Turn on Frequency synthesizer at Tx frequency (Frf) */
	MODE_TX    = 0X03, /**< Turn on Frequency synthesizer and transmitter*/
	MODE_FSRX  = 0X04, /**< Turn on Frequency synthesizer at Rx frequency (Frf) */
	MODE_RX    = 0X05, /**< Turn on Frequency synthesizer and reciver*/
	MODE_LONG_RANGE = 0X80 /**< Enum for setting the long range bit in the operating mode register */
} LoraModes;

/**
 * @brief Function Pointer to read a register from the Lora module
 */
typedef int8_t (*read_register_funcntion)(uint8_t reg_addr, uint8_t reg_set,
		uint8_t *return_val);
/**
 * @brief Function Pointer to write to a register on Lora module
 */
typedef void (*write_register_funcntion)(uint8_t reg_addr, uint8_t reg_set,
		uint8_t *return_val);

/**
 * @brief Fucntion to initalise the Lora Module
 */
bool lora_initialise(long operating_frequency,
		read_register_funcntion p_read_funcntion,
		write_register_funcntion p_write_funcntion);
/**
 * @brief Send message to Lora Module
 */
bool lora_transmit(uint8_t output_packet[4], uint8_t size);
/**
 * @brief Read message from the Lora Module
 */
bool lora_recive(uint8_t recived_packet[4]);
#endif
