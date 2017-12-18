/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 * KNX dev + NCN 5120
 ******************************************************************************
 *
*/

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32l4xx_hal.h"
#include "user.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* -------------------------------------------------------------------------------------------------------------------------------- */

/* ***********************  MAKRA   *********************** */

#define sw_13 			GPIO_Pin == GPIO_PIN_13
#define sw_1 			GPIO_Pin == GPIO_PIN_1
#define sw_2 			GPIO_Pin == GPIO_PIN_2
#define sw_3 			GPIO_Pin == GPIO_PIN_3
#define sw_6 			GPIO_Pin == GPIO_PIN_6
#define sw_7 			GPIO_Pin == GPIO_PIN_7
#define sw_8 			GPIO_Pin == GPIO_PIN_8
#define led_1 			GPIOA, GPIO_PIN_5

/* *********************** VARIABLE *********************** */
variable transmit, receive; /* ZMIENNE STRUKTURALNE */
uint8_t NCN_state = 0; /* STAN POCZATKOWY UKLADU NCN */
uint8_t NCN_pres = 0; /* OBECNOSC NCN */
uint8_t U_Configure_rec; /* ZMIENNA KONFIGURACYJNA ZWRACANA [0,b,aa,ap,c,m,0,1] */
uint8_t U_Reset_rec; /* POTWIERDZENIE RESETU */
uint8_t KNXTransmit[15], NCNTransmit[50], KNXReceive[9], Info[4]; /* UART trans/rec TABLE */
int loop_i, loop_j; /* ZMIENNE DLA PETLI */
volatile uint8_t RxComplete; /* FLAGA ODBIORU, USTAWIANA PO ODBIORZE */
volatile uint8_t TxComplete; /* FLAGA NADAWCZA, USTAWIANA PO TRANSMISJI */
volatile uint8_t ADCStart; /* FLAGA POMIARU, USTAWIANA BY ZAINICJOWAC POMIAR PRZETWORNIKIEM */
volatile uint8_t Switch; /* FLAGI PRZERWANIA OD PRZYCISKÓW*/
volatile uint8_t EXTI_Butt; /* FLAGA USTAWIANA PO PRZERWANIU EXTI */
volatile uint8_t TimComplete; /* FLAGA USTAWIANA PO SKONCZENIU ZLICZANIA PRZEZ TIM 17 */
uint8_t TransmitInfo[15]; /* */
uint8_t ACK[] = { PACK, BACK, NACK }; /* TABLICA POTWIERDZEN */
uint8_t ACK_rec[1]; /* POTWIERDZNIE ODBIORU */
uint8_t Rep_counter = 0, Busy_counter = 0; /* LICZNIK POWTOREK */
uint8_t led_stat[1];/* STATUS LED 1 */
uint8_t stat_send = 0; /* FLAGA ODPOWIEDZI */
/* *********************** FUNCTION *********************** */

/* ********** USTAWIENIE ZMIENNYCH DO TRANSMISJI ********** */
void transmit_conf(void) {
	transmit.Area_s = 0x01;
	transmit.Line_s = 0x01;
	transmit.Device_s = 0x01;

	transmit.Area_r = 0x01;
	transmit.Line_r = 0x01;
	transmit.Device_r = 0x02;

	transmit.Main_gr = 0x01;
	transmit.Sub_gr_H = 0x00;
	transmit.Sub_gr_L = 0x01;

	transmit.Repeat = First;
	transmit.Priority = Priority_3;
	transmit.Info_length = 0x01;
	transmit.APCI = V_write;
	transmit.Routing_cnt = 0x00;

	receive.Area_r = 0x01;
	receive.Line_r = 0x01;
	receive.Device_r = 0x01;

}

/* *********************** INICJACJA *********************** */
uint8_t tableA[4], *point_a;

/* ********** USTAWIENIE ADRESU FILTRACJI ********** */
void SetAddress(void) {
	point_a = tableA;
	*(point_a++) = U_SetAddress;
	*(point_a++) = (transmit.Area_s << 4) | transmit.Line_s;
	*(point_a++) = transmit.Device_s;
	*(point_a++) = 0x11;
	point_a -= 4;
	HAL_UART_Transmit_IT(&huart1, point_a, 4);
	HAL_UART_Receive_IT(&huart1, &U_Configure_rec, 1);
}

/* ********** KONFIGURACJA POWTÓREK ********** */
void SetRepetition(void) {
	point_a = tableA;
	*(point_a++) = 0xF2;
	*(point_a++) = 0x33;
	*(point_a++) = 0x11;
	*(point_a++) = 0x11;
	point_a -= 4;
	HAL_UART_Transmit_IT(&huart1, point_a, 4);
}

/* ********** KONFIGURACJA INNYCH MOZLIWOSCI ********** */
void U_Configure(void) {
	point_a = tableA;
	*point_a = 0x19; /* USTAWIENIE MARKERA */
	HAL_UART_Transmit_IT(&huart1, point_a, 1);
	HAL_UART_Receive_IT(&huart1, &U_Configure_rec, 1);
}

/* ********** INICJACJA UKLADU NCN5120 ********** */
void Initial(void) {
	SetAddress();
	do {

	} while (RxComplete == 0);
	if (RxComplete == 1) {
		RxComplete = 0;
		TxComplete = 0;
		SetRepetition();
		do {

		} while (TxComplete == 0);
		if (TxComplete == 1) {
			TxComplete = 0;
			U_Configure();
			do {

			} while (RxComplete == 0);
			if (RxComplete == 1) {
				RxComplete = 0;
				TxComplete = 0;
				if (U_Configure_rec == 0x25) {
					HAL_GPIO_WritePin(led_1, GPIO_PIN_SET);
				} else {

				}
			}
		}
	}
}
/* *********************** INICJACJA *********************** */
uint8_t table[1];

/* ********** RESET NCN ********** */
void Reset(void) {
	RxComplete = 0;
	TxComplete = 0;
	table[0] = U_Reset; /* RESET NCN */
	HAL_UART_Transmit_IT(&huart1, table, 1);
	HAL_UART_Receive_IT(&huart1, &U_Reset_rec, 1);
	do {
		do {

		} while (RxComplete == 0);
	} while (TxComplete == 0);
	RxComplete = 0;
	TxComplete = 0;
}

/* ********** TRANSMISJA NCN ********** */
uint8_t Transmit() {
	/**/
	uint8_t fnc_state = 0;
	uint8_t enkapsulacja_state = 0;
	/**/

	if (transmit.Info_length == 1) {

		enkapsulacja_state = enkapsulacja(TransmitInfo, &transmit, KNXTransmit);

		//if(enkapsulacja_state == state_OK){
		HAL_GPIO_WritePin(led_1, GPIO_PIN_SET);
		NCN_enkapsulacja(KNXTransmit, &transmit, NCNTransmit);
		HAL_UART_Transmit_IT(&huart1, NCNTransmit, 18);
		HAL_UART_Receive_IT(&huart1, KNXReceive, 9);
		do {

		} while (RxComplete == 0);
		RxComplete = 0;
		if (KNXReceive[8] == NCNTransmit[17]) {
			HAL_UART_Receive_IT(&huart1, KNXReceive, 2);
			do {

			} while (RxComplete == 0);
			RxComplete = 0;
			if (KNXReceive[0] == U_FrameStateMap) {
				fnc_state = state_OK;
			} else {
				fnc_state = KNXReceive[0];
			}
		}
		//}
	} else {

		enkapsulacja_state = enkapsulacja(TransmitInfo, &transmit, KNXTransmit);
		if (enkapsulacja_state == state_OK) {
			NCN_enkapsulacja(KNXTransmit, &transmit, NCNTransmit);
			HAL_UART_Transmit_IT(&huart1, NCNTransmit,
					(18 + (2 * transmit.Info_length)));
			HAL_UART_Receive_IT(&huart1, KNXReceive,
					(9 + transmit.Info_length));
			do {

			} while (RxComplete == 0);
			RxComplete = 0;
			if (KNXReceive[8 + transmit.Info_length]
					== NCNTransmit[17 + (2 * transmit.Info_length)]) {
				HAL_UART_Receive_IT(&huart1, KNXReceive, 2);
				do {

				} while (RxComplete == 0);
				RxComplete = 0;
				if (KNXReceive[0] == U_FrameStateMap) {
					fnc_state = state_OK;
				} else {
					fnc_state = KNXReceive[0];
				}
			}
		}
	}
	return (fnc_state);
}

void KNXtransmitIT(void) {
	Repeat: for (loop_i = 0; loop_i < 9; loop_i++) {
		/* WYSLANIE POJEDYNCZEGO ZNAKU */
		HAL_UART_Transmit_IT(&huart1, KNXTransmit + loop_i, 1);
		do {

		} while (TxComplete == 0);
		TxComplete = 0;
		/* PAUZA */
		HAL_TIM_Base_Start_IT(&htim17);
		do {

		} while (TimComplete == 0);
		TimComplete = 0;
		HAL_TIM_Base_Stop_IT(&htim17);
	}

	/* ********** CZEKANIE NA POTWIERDZENIE ********** */
	do {

	} while (RxComplete == 0);
	RxComplete = 0;
	if (ACK_rec[0] != 0) {
		switch (ACK_rec[0]) {
		case NACK: /* BLAD TRANSMISJI */
			transmit.Repeat = Repeated;
			Rep_counter++;
			if (Rep_counter <= 3) {
				goto Repeat;
			}
			break;

		case PACK: /* TRANSMISJA POMYSLNA */
			HAL_UART_Receive_IT(&huart1, KNXReceive, 9);
			break;

		case BACK: /* ODBIORCA ZAJETY */
			HAL_Delay(10);
			transmit.Repeat = Repeated;
			Busy_counter++;
			if (Busy_counter <= 3) {
				goto Repeat;
			}
			break;
		}
	}
	Rep_counter = 0;
	Busy_counter = 0;
}
/* *********************** PRZERWANIA *********************** */

/* ********** ODBIOR ********** */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	RxComplete = 1;
}

/* ********** TRANSMISJA ********** */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	TxComplete = 1;
}

/* ********** PRZYCISKI ********** */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	EXTI_Butt = 1;
	if (sw_13) {
		Switch = 13;
	}
	 if(sw_1){
	 Switch = 1;
	 }
	 if(sw_2){
	 Switch = 2;
	 }
	 if(sw_3){
	 Switch = 3;
	 }
	 if(sw_6){
	 Switch = 6;
	 }
	 if(sw_7){
	 Switch = 7;
	 }
	 if(sw_8){
	 Switch = 8;
	 }
}

/* ********** TIMER ********** */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	if (htim->Instance == TIM17) {
		TimComplete = 1;
	}
}

/* -------------------------------------------------------------------------------------------------------------------------------- */

/*
 * 		MAIN:
 * INICJACJA SYSTEMU
 * OPOZNIENIE
 * SPRAWDZENIE NCN
 * INICJACJA NCN
 * ZEROWANIE FLAG
 *
 * */

/* *********************** MAIN *********************** */
int main(void) {

	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	MX_TIM17_Init();
	MX_USART1_UART_Init();

	/* ********** INICJACJA NCN ********** */
	HAL_Delay(1000);
	transmit_conf();

	if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)
			&& HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)) == GPIO_PIN_SET) {
		NCN_pres = 1;
		Initial();
	} else {
		//HAL_GPIO_WritePin(led_1, GPIO_PIN_SET);
		NCN_pres = 0;
	}

	EXTI_Butt = 0;
	HAL_UART_Receive_IT(&huart1, KNXReceive, 9);
	uint8_t dekapsulacja_state;
	/* ********** PETLA GLOWNA ********** */
	while (1) {

		/* ********** PRZERWANIE OD PRZYCISKU ********** */
		if (EXTI_Butt == 1) {
			EXTI_Butt = 0;
			uint8_t Butt = Switch;
			Switch = 0;

			/* ********** STAN PRZYSISKOW ********** */
			switch (Butt) {
			case 13: /* RESET NCN */
				HAL_GPIO_TogglePin(led_1);
				HAL_Delay(10);
				HAL_GPIO_TogglePin(led_1);

				Reset();
				RxComplete = 0;
				Initial();

				break;

			case 1: /* LAMP 1 ON */
				TransmitInfo[1] = DPT_Switch_ON;
				break;

			case 2: /* LAMP 1 OFF */
				TransmitInfo[2] = DPT_Switch_OFF;
				break;

			case 3: /* LAMP 1 DIMMER UP */
				TransmitInfo[3] = DPT_Dimm_UP;
				break;

			case 6: /* LAMP 1 DIMMER DOWN */
				TransmitInfo[6] = DPT_Dimm_DOWN;
				break;

			case 7: /* ALL LAMP ON */
				TransmitInfo[7] = DPT_Switch_ON;
				break;

			case 8: /* ALL LAMP OFF */
				TransmitInfo[8] = DPT_Switch_OFF;
				break;
			}
			/* ********** TRANSMISJA WIADOMOSCI ********** */
			enkapsulacja(&TransmitInfo[Butt], &transmit, KNXTransmit);
			if (Butt != 13) {
				if ((Butt == 7) || (Butt == 8)) {
					transmit.Add_type = Group_add;
				} else {
					transmit.Add_type = Indywidual_add;
				}

				RxComplete = 0;
				HAL_UART_Receive_IT(&huart1, ACK_rec, 1);
				if (NCN_pres == 1) {
					HAL_UART_Transmit_IT(&huart1, NCNTransmit, 18);
				} else {
					KNXtransmitIT();
				}
			}
		}


		/* ************************** REC ************************** */
		/* ********** PRZERWANIE PO ODBIORZE WIADOMOSCI ********** */
		if (RxComplete == 1) {
			RxComplete = 0;
			dekapsulacja_state = dekapsulacja(KNXReceive, &receive, Info);
			HAL_UART_Transmit_IT(&huart1, &dekapsulacja_state, 1);

			/* BLAD PARZYSTOSC */
			if ((HAL_UART_GetError(&huart1) == HAL_UART_ERROR_PE)) {
			 for (loop_i = 0; loop_i < 36; loop_i++) {
			 HAL_TIM_Base_Start_IT(&htim17);
			 do {

			 } while (TimComplete == 0);
			 TimComplete = 0;
			 HAL_TIM_Base_Stop_IT(&htim17);
			 }
			 HAL_UART_Transmit_IT(&huart1, &ACK[2], 1);
			 do {

			 } while (TxComplete == 0);
			 TxComplete = 0;
			 }

			/* STAN OK */

			if (dekapsulacja_state == state_OK) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				if(receive.APCI == V_write){
				switch (Info[0]) {
				case 0x01:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					break;
				case 0x00:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					break;
				}

				}

				 if(receive.APCI == V_read){
				 led_stat[0] = HAL_GPIO_ReadPin(led_1);
				 enkapsulacja(led_stat, &transmit, KNXTransmit);
				 stat_send = 1;
				 }
				 if(receive.APCI == V_response){
				 if(Info[0] == 1){
				 transmit.APCI = V_write;
				 EXTI_Butt = 1;
				 Switch = 2;
				 }else if(Info[0] == 0){
				 transmit.APCI = V_write;
				 EXTI_Butt = 1;
				 Switch = 1;
				 }
				 }

				for (loop_i = 0; loop_i < 36; loop_i++) {
					HAL_TIM_Base_Start_IT(&htim17);
					do {

					} while (TimComplete == 0);
					TimComplete = 0;
					HAL_TIM_Base_Stop_IT(&htim17);
				}

				HAL_UART_Transmit_IT(&huart1, ACK, 1);

				do {

				} while (TxComplete == 0);
				TxComplete = 0;

				/* PARZYSTOSC OD POLA SPR */
			}
			if (dekapsulacja_state == state_Parity) {
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				for (loop_i = 0; loop_i < 36; loop_i++) {
					HAL_TIM_Base_Start_IT(&htim17);
					do {

					} while (TimComplete == 0);
					TimComplete = 0;
					HAL_TIM_Base_Stop_IT(&htim17);
				}
				HAL_UART_Transmit_IT(&huart1, &ACK[2], 1);
				do {

				} while (TxComplete == 0);
				TxComplete = 0;
			}
			HAL_UART_Receive_IT(&huart1, KNXReceive, 9);
			if (stat_send == 1) {
				KNXtransmitIT();
				stat_send = 0;
			}

		}
	}
}
/* *********************** MAIN *********************** */

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
			| RCC_PERIPHCLK_ADC;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

	ADC_MultiModeTypeDef multimode;
	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 47999;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim3);

}

/* TIM17 init function */
static void MX_TIM17_Init(void) {

	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 1;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 630;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B; /*8 b + parity*/
	huart1.Init.StopBits = UART_STOPBITS_1; /*2*/
	huart1.Init.Parity = UART_PARITY_NONE; /*EVEN*/
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;

	/*Configure GPIO pins : PC13 PC1 PC2 PC3
	 PC6 PC7 PC8 */
	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
	|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void Error_Handler(void) {
	while (1) {
	}

}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */
