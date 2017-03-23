#include "HAL.h"
#include "nrf24l01.h"
#include "delay.h"
#include "main.h"
#include "usart.h"

rc_t rc;
uint8_t rc_matched = 0x55;

void rc_init(void)
{
	NRF24L01_Init();
	if(HAL_GPIO_ReadPin(TXD_B_GPIO_Port,TXD_B_Pin) == 0){
		rc_matched = 0;
	}
}

void rc_match(void)
{
	uint8_t sta;
	uint8_t old_addr = RX_ADDRESS[4];
	static uint32_t dT,Ts;

	Ts = micros();
	do{
		dT = micros() - Ts;
		if((dT/100000)%2){
			HAL_GPIO_TogglePin(GPIOB, LED_SIGN_Pin);
		}

		NRF24L01_RX_Mode();
		SendChar("Finding ADDR: ");
		SendInt(RX_ADDRESS[4]);
		_n();
		delay_ms(4);
		sta = NRF24L01_Read_Reg(NRF_READ_REG + STATUS);

		if((sta & 0x0E) == 0x00){
			rc_matched = 1;

		}else{
			RX_ADDRESS[4] ++;
			if(RX_ADDRESS[4] == 0xff){
				RX_ADDRESS[4] = 0x00;
			}
		}
//		if(RX_ADDRESS[4] == old_addr){
//			rc_matched = 0;
//			RX_ADDRESS[4] ++;
//		}


	}while(!rc_matched);

}
