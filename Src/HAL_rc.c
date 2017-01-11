#include "HAL.h"
#include "nrf24l01.h"

rc_t rc;

void rc_init()
{
	NRF24L01_Init();
}
