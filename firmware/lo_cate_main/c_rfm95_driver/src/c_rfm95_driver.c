#include "c_rfm95_driver.h"
#include <stddef.h>

read_register_funcntion read_register = NULL;
write_register_funcntion write_register = NULL;

uint8_t rx_lora_buff = { 0 };

bool lora_initialise(long operating_frequency,
		read_register_funcntion p_read_funcntion,
		write_register_funcntion p_write_funcntion) {
	read_register = p_read_funcntion;
	write_register = p_write_funcntion;

	// Check the Lora Hardware version
	read_register(REG_VERSION, 0x00, &rx_lora_buff);
	if (rx_lora_buff != 0x12) {
		return false;
	}
	// Put module to sleep so settings can be changed
	write_register(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP, &rx_lora_buff);
	read_register(REG_OP_MODE, 0x00, &rx_lora_buff);
	//Set the Operating Frequency
	uint64_t frf = ((uint64_t) operating_frequency << 19) / 32000000;
	uint8_t msb = (uint8_t)(frf >> 16);
	uint8_t mib = (uint8_t) (frf >> 8);
	uint8_t lsb = (uint8_t) (frf >> 0);
	write_register(REG_FR_MSB, msb, &rx_lora_buff);
	write_register(REG_FR_MIB, mib, &rx_lora_buff);
	write_register(REG_FR_LSB, lsb, &rx_lora_buff);

	read_register(REG_FR_MSB, 0x00, &rx_lora_buff);
	if (msb != rx_lora_buff)
	{
		return false;
	}

	// FIFO set base addresses
	write_register(REG_FIFO_TX_BASE_ADDR, 0, &rx_lora_buff);
	write_register(REG_FIFO_RX_BASE_ADDR, 0, &rx_lora_buff);

	// Set the LNA Boost
	uint8_t lna_setting = { 0 };
	read_register(REG_LNA, 0x00, &lna_setting);
	write_register(REG_LNA, lna_setting | 0x03, &rx_lora_buff);
	// Set the auto AGC
	write_register(REG_MODEM_CONFIG_3, 0x04, &rx_lora_buff);

	// Place it in standby
	write_register(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY, &rx_lora_buff);

	return true;
}

bool lora_transmit(uint8_t output_packet[4], uint8_t size)
{
	// Check if module is currently transmitting
	uint8_t isTransmitting = {0};
	read_register(REG_OP_MODE, 0x00, &isTransmitting);
	if ((isTransmitting & MODE_TX) == MODE_TX)
	{
		// Device is already transmitting
		return false;
	}

	// Place in explicit header mode
	uint8_t isExplicit = {0};
	read_register(REG_MODEM_CONFIG_1, 0x00, &isExplicit);
	write_register(REG_MODEM_CONFIG_1, isExplicit & 0xfe, &rx_lora_buff);

	// Reset the FIFO and Payload length Buffers
	write_register(REG_RX_PTR, 0, &rx_lora_buff);
	write_register(REG_PAYLOAD_LENGTH,0, &rx_lora_buff);

	// Write message to the FIFO register
	for(size_t i =0; i<size;i++)
	{
		write_register(REG_FIFO,output_packet[i], &rx_lora_buff);
	}

	// Place device in transmit mode
	write_register(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX, &rx_lora_buff);

	// Wait for device to finish transmitting
	uint8_t irqRegisterVal = {0};
	read_register(REG_IRQ_FLAGS, 0x00, &irqRegisterVal);

    while ((irqRegisterVal & IRQ_TX_DONE) == 0)
    {
    	read_register(REG_IRQ_FLAGS, 0x00, &irqRegisterVal);
    }
	// Reset the transmission interrupts
    write_register(REG_IRQ_FLAGS, IRQ_TX_DONE, &rx_lora_buff);
	return true;
}


bool lora_recive(uint8_t recived_packet[4])
{
	// Check if module is in Rx mode
	uint8_t isReciving = {0};
	read_register(REG_OP_MODE, 0x00, &isReciving);
	if ((isReciving & MODE_RX) == MODE_RX)
	{
		return false;
	}

	// Check if there is a recived packet length
	uint8_t bytesRecived = {0};
	read_register(REG_PAYLOAD_LENGTH, 0x00, &bytesRecived);
	if(bytesRecived == 0x00)
	{
		return false;
	}

	// Read from the FIFO buffer for packet length
	for(size_t i =0; i<4;i++)
	{
		read_register(REG_FIFO,0x00, &rx_lora_buff);
	}

	 return true;
}
