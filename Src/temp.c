/*
 * temp_adc.c
 *
 *  Created on: Feb 10, 2019
 *      Author: Matt Flanagan & Dawson Moore
 */

#include "temp.h"

// Globals
uint8_t temp_array[READ_MSG_SIZE];
uint16_t adc_val;
const uint8_t channel[NUM_CHANNELS] = {CHANNEL_0, CHANNEL_1,
                                       CHANNEL_2, CHANNEL_3, CHANNEL_4, CHANNEL_5, CHANNEL_6,
                                       CHANNEL_7, CHANNEL_8, CHANNEL_9, CHANNEL_10, CHANNEL_11,
                                       CHANNEL_12, CHANNEL_13, CHANNEL_14, CHANNEL_15
                                      };

const uint8_t chip[10] = {ID_TEMP_1A, ID_TEMP_1B, ID_TEMP_2A, ID_TEMP_2B, ID_TEMP_3A,
                          ID_TEMP_3B, ID_TEMP_4A, ID_TEMP_4B, ID_TEMP_5A, ID_TEMP_5B};
uint8_t read_byte = 0;
uint8_t write_data[WRITE_MSG_SIZE + 1];

// @funcname: adc2temp
//
// @brief: Converts ADC counts to engineering units
//
// @param: adc_value: Value read from LTC ADC chip
//
// @return: 16 bit temperature in engineering units
static uint16_t adc2temp(uint16_t adc_value)
{
    // Locals
	float voltage;                                                                                          	// Estimated thermistor voltage
	float thermistor_res;                                                                                   	// Estimated thermistor resistance
	double temperature;                                                                                     	// Estimated thermistor temperature
    
	//calculate the voltage from the adc_val
	voltage = VOLTAGE_REF * ((float) (adc_value)) / 0xFFFF;                                                 	// Calculate the voltage from the ADC value
	thermistor_res = (voltage * THERM_RESIST) / (VOLTAGE_TOP - voltage);                                    	// Calculate the resistance from the voltage
	temperature =  B_VALUE / log (thermistor_res / R_INF_3977) - KELVIN_2_CELSIUS;                          	// Calculate the temperature

	return (uint16_t)(10 * temperature);
}

// @funcname: adcExtract
//
// @brief: Extracts 16 bit ADC count value from returned I2C frame
//
// @param: arr: Pointer to array of data returned by LTC ADC
//
// @return: 16 bit temperature ADC count value
static inline uint16_t adcExtract(uint8_t* arr)
{
	return ((uint16_t) arr[0] << 10) | ((uint16_t) arr[1] << 2) | ((uint16_t) arr[2] >> 6);                 	// Returned extracted ADC value
}

// @funcname: readLTCValue
//
// @brief: Reads the value of the LTC ADC at the given channel and tap
//
// @param: current_channel: The channel on the ADC that is being read
// @param: tap: The current IC (tap) being read
//
// @return: 16 bit temperature in engineering units
static uint16_t readLTCValue(uint8_t current_channel, uint8_t tap)
{
	read_byte = set_address(chip[tap], READ_ENABLE);															// Set read byte
	HAL_I2C_Master_Receive(&hi2c1,(uint16_t) read_byte, &temp_array[0], READ_MSG_SIZE, 0xFFFF);					// Transmit read command
	while (hi2c1.State != HAL_I2C_STATE_READY);																	// Wait for the send to finish
	adc_val = adcExtract(temp_array);																			// Extract data

	return(adc2temp(adc_val));																					// Return in engineering unit
}

// @funcname: initLTC
//
// @brief: Initializes the LTC, sending a wake tone and ensuring a response
void initLTC()
{
	uint8_t timeout;                                                                                        	// Timeout counter
    uint8_t i;                                                                                              	// Loop counter variable

    for (i = 0; i < 2; i++)
    {
        timeout = 0;                                                                                        	// Set the timeout to 0 for this loop
        write_data[0] = set_address(chip[i], WRITE_ENABLE);                                                 	// Set address to chip i
        write_data[1] = channel_combine(channel[0]);                                                        	// Setup channel byte
        write_data[2] = 0xFF;                                                                               	// Write wakeup
        HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) write_data[0], &write_data[1], WRITE_MSG_SIZE, 0xFFFF);  	// Transmit wakeup

        while (hi2c1.State != HAL_I2C_STATE_READY && timeout++ < WRITE_TIMEOUT);                            	// Wait until I2C enters ready state
        if (timeout >= WRITE_TIMEOUT)                                                                       	// Check if we didn't write properly
        {
            // TODO: Use fault library for this
            //bms.temp_con = FAULTED;                                                                         	// Set temperature connection status to fault
			//signalFault(TEMP_CONNECTION_FAULT_NUM);
        }
        HAL_Delay(WRITE_REQ_WAIT);                                                                          	// Wait the required time period
    }
}

// @funcname: tim2Setup
//
// @brief: Configures and enables timer 2
void tim2Setup()
{
    // Targeting an interrupt every 2 ms
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;                           // Enable timer clock in RCC
    TIM2->PSC = 3200 - 1;                                           // Set prescalar (clock at 10000 Hz)
    TIM2->ARR = 10;                                                 // Set auto reload value
    TIM2->CR1 &= ~(TIM_CR1_DIR);                                    // Set to count down
    TIM2->DIER |= TIM_DIER_UIE;                                     // Enable update interrupt
    NVIC->ISER[0] |= 1 << TIM2_IRQn;                                // Unmask interrupt
    TIM2->CR1 |= TIM_CR1_CEN;                                       // Enable the timer
}

// @funcname: sendErrorFrame
//
// @brief: Signals to main that there's a fault in the system
void sendErrorFrame(DAQ_TypeDef* daq)
{
    // Locals
    uint32_t mailbox;                                               // Mailbox for send
    uint8_t data[1];                                                // Data for send
    CAN_TxHeaderTypeDef header;                                     // CAN frame

    header.StdId = ERROR_ADDR;                                      // Send with error ID
    header.IDE= CAN_ID_STD;                                         // Standard length frame
    header.RTR = CAN_RTR_DATA;                                      // Data frame
    header.DLC = 1;                                                 // Send 1 byte
    header.TransmitGlobalTime = DISABLE;                            // Don't tx time

    data[0] = 0x1;                                                  // Send error

    while (HAL_CAN_GetTxMailboxesFreeLevel(daq->hcan) == 0);        // I hate this line, but fair enough

    HAL_CAN_AddTxMessage(daq->hcan, &header, data, &mailbox);       // Send frame
}

// @funcname: checkTemp
//
// @brief: Checks given temperature values for rules compliance
//
// @exec_period: 10Hz or every 100 ms
void checkTemp(uint16_t temp[NUM_TEMP][NUM_CHANNELS], DAQ_TypeDef* daq)
{
    // Locals
    uint8_t i, j;                                                   // Generic loop counter vars

    /*
     * We have some cells that might be reading a bit too high all the time.
     * In order to combat this, we have a plausibility check in place.
     * The rules state we need to turn the car off if the temp of a cell
     * exceeds 60° C. If we check that the temp is between 60° and 70°, it
     * will be physically impossible for a cell to jump from < 60° and > 70°
     * between the span of one measurement. Therefore, this will catch all
     * errors while masking out bad measurements.
     */
    for (i = 0; i < NUM_TEMP; i++)                                  // Loop through each IC
    {
        for (j = 0; j < NUM_CHANNELS; j++)                          // Loop through each channel on each IC
        {
            if (temp[i][j] > 600 && temp[i][j] < 700)               // Check if 60 < temp < 70
            {
                sendErrorFrame(daq);                                // Send error
            }
        }
    }
}

// @funcname: acquireTemp
//
// @brief: Gathers temperature values for all ICs and all channels requested by user
//
// @exec_period: 1000Hz or every 1 ms
void acquireTemp(uint16_t temp[NUM_TEMP][NUM_CHANNELS])
{
	// Locals
	temp_state_t   		next_state;																				// Next state for FSM
	static uint8_t 		ic;																						// Current ic being read
	static uint8_t 		chan;																				    // Current channel being read
	static uint8_t		wait;																					// Current wait time
	static temp_state_t state;																					// Current FSM state

	/*
		The FSM is being used to hide a delay statement. If we don't do this, we need to ensure that a wait of
		~200 ms is taken between triggering a conversion and reading the value on the ADC. In this setup, without
		and RTOS, we can just skip 200 cycles of the FSM running at ~1ms per cycle to make up for this removed wait
		without the need to block.

		TL;DR: Be wary of editing the setup. I'm looking at you, future Dawson...
	*/

	next_state = state;																							// Default to same state

	switch (state)																								// Check current state
	{
		case CHANNEL_UPDATE:																					// Update channel
		{
		    chan = chan == NUM_CHANNELS ? 0 : chan;													            // Set channel back to 0 if number if channels is exceeded
			write_data[1] = channel_combine(channel[chan++]);											        // Set channel byte
			HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) write_data[0], &write_data[1], WRITE_MSG_SIZE, 0xFFFF);	// Send with max timeout
			while (hi2c1.State != HAL_I2C_STATE_READY);															// Wait for send to stop
			next_state = WAIT;																					// Move to wait state

			break;																								// Don't continue execution until next pass through
		}

		case WAIT:
		{
			if (wait++ == 199)																					// Increment wait and check if we've hit time
			{
				wait = 0;																						// We've hit the wait time, so reset counter
				next_state = TEMP_READ;																			// Move to read state
			}

			break;
		}

		case TEMP_READ:
		{
			temp[ic][chan - 1] = readLTCValue(chan - 1, ic);
			++ic;                                                                                               // Increment ic
			ic = ic == NUM_TEMP ? 0 : ic;																		// Set ic back to 0 if number of ICs is exceeded

			if (ic == 0)																						// Check if we just wrapped the ic variable
			{
				next_state = CHANNEL_UPDATE;																	// We did, so advance to next channel
			}
		}
	}

	state = next_state;
}

// @funcname: tempConnectionFaultSet
//
// @brief: Handles a temp connection fault
void tempConnectionFaultSet()
{
	// TODO: Handle temp connection fault (set)
}
