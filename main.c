
/* USER CODE BEGIN Header */
/*eger bu kodu CubeMX uzerinden guncellediyseniz usart.c icerisindeki usart4 icin
baudrate degerini "baudrate" degiskeni ile degistirmeniz gerekir.*/
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE   				((uint32_t)  200)
#define VERIFIED							((uint8_t)	0x20)
#define RECEIVE_TYPE					((uint8_t)  0x90)
#define TRANSMIT_STATUS_TYPE	((uint8_t)  0x8B)
#define AT_RESPONSE_TYPE			((uint8_t)  0x88)
#define ND_RESPONSE_TYPE1			((uint8_t)  0x4E)
#define ND_RESPONSE_TYPE2			((uint8_t)  0x44)
#define TRANSMIT_TYPE					((uint8_t)  0x10)
#define RAISED								((uint8_t)  0xAA)
#define CLEARED								((uint8_t)  0xFF)
#define RECEIVE_DELIMITER			((uint8_t)  0xEE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static uint8_t		xBeeFrame[RXBUFFERSIZE];
static uint8_t		myData[RXBUFFERSIZE] = {RECEIVE_DELIMITER, 0x02, 0x03, 0x01, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t 		myMAC[8] = {0x00, 0x13, 0xA2, 0x00, 0x40, 0x99, 0x59, 0xDA};
static uint8_t		myShort[2] = {0xFF, 0xFE};
static uint8_t		rxLongAddress[8];
static uint8_t		rxShortAddress[2];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int baudrate = 300;
 uint8_t myRxData [1], buff[RXBUFFERSIZE] ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void handshake (void);
void send_command(char* data1, char* data2);
void commEnd (void);
uint8_t buildFrameTransmitRequest		(uint8_t*,uint8_t*,uint8_t*,uint8_t);
void 		clearFrame						(void);
void 		setTransmitCommons		(void);
void 		setDestAddress				(uint8_t*, uint8_t*);
uint8_t setData								(uint8_t*,uint8_t);
void 		setChecksum						(uint8_t);
void 		buildFrameATCommand		(void);	//7E 00 04 08 01 4E 44 64

void 		sendxBee			(void)
{
	uint8_t data_length = 0;

	for (uint8_t i = 0x00; i < 0x04; i++)
		myData[i+data_length+0x05] = buff[i];
	data_length	+=	0x04;

	//-------------------------------------------------------


	for (uint8_t i = 0x00; i < 0x04; i++)
		myData[i+data_length+0x05] = buff[i];
	data_length	+=	0x04;

	//-------------------------------------------------------
	//	3	-	Other Sensors
	/* NOT: bu ikinci blok da ayni "buff" kaynagini kopyaliyor; henuz baglanmamis
	   ikinci bir olcum/sensor icin birakilmis tamamlanmamis bir yer tutucu (stub).
	   Islevsel bir "duzeltme" yapmadan, oldugu gibi birakildi. */
	//-------------------------------------------------------
	uint8_t length_of_frame = buildFrameTransmitRequest(rxLongAddress,rxShortAddress,myData, data_length+0x05);
	HAL_UART_Transmit(&huart1,xBeeFrame,length_of_frame,0x1000);
}

void 		getOnOff					(void){}
void 		getPower					(void){}
void 		setOnOff					(void){}
void 		setPower					(void){}

uint8_t buildFrameTransmitRequest (uint8_t * mac_address, uint8_t * short_address, uint8_t * data, uint8_t data_buffer_length)
{
	/*
	I need to:
			++	Clear whole frame
			++	Set0				ZigBee Start Byte as 7E
			++	Set1:2			Length to 				Length of data + 14
			++	Set3				Frame Type to 10 for Transmit Request
			++	Set4				Frame IDs set to 01 as Default
			++	Set5:12 		Destination Adress	- 64 bit
			++	Set13:14		Short Adress				- 16 bit
			++	Set15:16		00
			++	Set17:			Data
			--	Set					Checksum
	*/
	//--------------------------------------------------------
	uint8_t data_length;
	clearFrame();
	setTransmitCommons();
	setDestAddress(mac_address, short_address);
	data_length = setData(data, data_buffer_length);
	setChecksum(data_length);
	return data_length + 18;
}

void 		clearFrame					(void)
{
	for (int i=0; i<RXBUFFERSIZE; i++){
		xBeeFrame[i] = 0x00;
	}
}

void 		setTransmitCommons	(void)
{
	xBeeFrame[0] 	= 0x7E;
	xBeeFrame[3] 	= TRANSMIT_TYPE;
	xBeeFrame[4] 	= 0x01;
	xBeeFrame[15]	= 0x00;
	xBeeFrame[16]	= 0x00;
}

void 		setDestAddress			(uint8_t * mac_address, uint8_t * short_address)
{
	for (int i=0; i<8; i++){
		xBeeFrame[i+5] = *(mac_address+i);
	}
	for (int i=0; i<2; i++){
		xBeeFrame[i+13] = *(short_address+i);
	}
}

uint8_t setData							(uint8_t* data, uint8_t data_buffer_length)
{
	uint8_t real_data_length = 0;
	for (int i=0; i<data_buffer_length && (*(data+i+0) || *(data+i+1)|| *(data+i+2)|| *(data+i+3)|| *(data+i+4)|| *(data+i+5)|| *(data+i+6)|| *(data+i+7)|| *(data+i+8)|| *(data+i+9)) ; i++){
		xBeeFrame[i+17] = *(data+i);
		real_data_length++;
	}

	xBeeFrame[1] = 0x00;
	xBeeFrame[2] = real_data_length + 14;
	return real_data_length;
}

void 		setChecksum					(uint8_t data_length)
{
	uint8_t checksum = 0xFF;
	for (int i=3; i<data_length+17; i++)	{
		checksum -= xBeeFrame[i];
	}
	xBeeFrame[data_length+17]	= checksum;
}

void 		buildFrameATCommand	(void)
{
	clearFrame();
	xBeeFrame[0] = 0x7E;
	xBeeFrame[1] = 0x00;
	xBeeFrame[2] = 0x04;
	xBeeFrame[3] = 0x08;
	xBeeFrame[4] = 0x01;
	xBeeFrame[5] = 0x4E;
	xBeeFrame[6] = 0x44;
	xBeeFrame[7] = 0x64;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 char *txData,*txData1, *txData2, *txData3, *rxData;
 int rtime, temp =0, j = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART4_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


while (1){

	handshake(); // sayac ile haberlesmeyi baslatma fonksiyonu

	/// kohler sayac okumasi
	send_command("0.0.0()"	, "P\r");		// Seri no
	send_command("32.7.0()"	, "f\r");		// Voltaj R
	send_command("52.7.0()"	, "`\r");   // Voltaj S
	send_command("72.7.0()"	, "b\r");   // Voltaj T
	send_command("31.7.0()"	, "e\r");  // Akim R
	send_command("51.7.0()"	, "c\r");   // Akim S
	send_command("71.7.0()"	, "a\r");   // Akim T
	send_command("33.7.0()"	, "g\r");  // Cos Q (R)
	send_command("53.7.0()"	, "a\r");   // Cos Q (S)
	send_command("73.7.0()"	, "c\r");   // Cos Q (T)

	commEnd(); // haberlesme biter


			HAL_Delay(5000); //dongu basa saracagindan bir bekleme konuldu. proje devaminda bu silinebilir
 }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */

	if(j >= (int)RXBUFFERSIZE){
		/* tampon dolu: tasmayi engellemek icin yeni veri "buff" disina yazilmaz,
		   alim dongusu sonlandirilir */
		temp++;
		return;
	}

	buff[j] = myRxData[0];

	if(buff[j] == 0x0a || buff[j] == 0x06 ||	(j > 0 && buff[j-1] == 0x03 &&	buff[j] != 0x00 )|| buff[j] == 0x00){
		temp++;
	}
	j++;
}


void handshake (void) {

		baudrate = 300;
		MX_USART2_UART_Init();
		MX_USART4_UART_Init();	     //baudrate degisiminden etkilenen uartlar tekrar baslatilir
		txData = "/?!\r\n";						// handshake ilk komut


j = 0;	// callBack icinde surekli sayim yapacak deger baslangicta sifirlanir

	HAL_UART_Transmit(&huart2, (uint8_t*)txData, strlen(txData), (baudrate/2)*strlen(txData)); // optik port
		HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_SET); //RS485 gpio pini ile kontrol edilir, iletim aktif
	HAL_UART_Transmit(&huart4, (uint8_t*)txData, strlen(txData), (baudrate/2)*strlen(txData));	// enson sayac portuna gonderilir ki hemen diger koda gecikmesiz halde gecsin
		HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_RESET);// gonderim aktif

while(temp == 0){HAL_UART_Receive_DMA(&huart4, myRxData, 1);} //gelen veriyi tek tek yakalar, bu islem bitmeden dongudan cikmaz
//transmiting data //yakalanan veriyi iletim kismi, tek tek gonderilir
for(int i = 0; i<RXBUFFERSIZE ; i++){

			uint8_t abuff[1] ;
			abuff[0]= buff[i];
			HAL_UART_Transmit(&huart2, abuff,1,50);
				//detecting end of data
				// LF or ACK or (ETX and not null	) tespitinde mesajin sonu oldugu anlasilir ve dongu sona erer
			if(buff[i] == 0x0a || buff[i] == 0x06 ||	(i >= 2 && buff[i-2] == 0x03 &&	buff[i] == 0x13 )){
				break;
				}
			}
			temp =0;//sonraki alim dongusune girmesi icin bu deger sifirlanir
		memset(buff, 0, RXBUFFERSIZE); // uzerine yazma olma olmamasi icin ana dizi sifirlanir

		uint8_t buffer = 0x06; //<ACK>        //yazdirilamaz karakter ile normal karakterler ayri gonderilir
		txData= "051\r\n";                    //yazdirilamaz karakter ile normal karakterler ayri gonderilir

	j = 0;
		HAL_UART_Transmit(&huart2, &buffer, 1, (baudrate/10)*1);
		HAL_UART_Transmit(&huart2, (uint8_t*)txData, strlen(txData), 300*4);
	HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart4, &buffer, 1, (baudrate/10)*1);
		HAL_UART_Transmit(&huart4, (uint8_t*)txData, strlen(txData), 300*4);
	HAL_GPIO_WritePin(GPIOA, DE_RE_Pin,GPIO_PIN_RESET);

while(temp == 0){HAL_UART_Receive_DMA(&huart4, myRxData, 1);}
//transmiting data
for(int i = 0; i<RXBUFFERSIZE ; i++){

			uint8_t abuff[1] ;
			abuff[0]= buff[i];
			HAL_UART_Transmit(&huart2, abuff,1,50);
				//detecting end of data
				// LF or ACK or (ETX and not null	)
			if(buff[i] == 0x0a || buff[i] == 0x06 ||	(i >= 2 && buff[i-2] == 0x03 &&	buff[i] == 0x13 )){
				break;
				}
			}
			temp =0;
		memset(buff, 0, RXBUFFERSIZE);

		baudrate = 9600;                  //baudrate degisimi
		MX_USART2_UART_Init();          //baudrate degisimi
		MX_USART4_UART_Init();          //baudrate degisimi
}
void send_command (char* data1, char* data2){

	uint8_t buffer = 0x01; //<SOH>       //standart kalip
	txData1 = "R2";					             //standart kalip
	uint8_t buffer1 = 0x02; //<STX>      //standart kalip
	txData2 = data1;                     //standart kalip
	uint8_t buffer2 = 0x03; //<ETX>      //standart kalip
	txData3 = data2;                     //standart kalip

	j = 0;
			//control, optik
	HAL_UART_Transmit(&huart2, &buffer, 1, 1*baudrate/10);
	HAL_UART_Transmit(&huart2, (uint8_t*)txData1, strlen(txData1), 2*baudrate/10);
	HAL_UART_Transmit(&huart2, &buffer1, 1, 1*baudrate/10);//STX
	HAL_UART_Transmit(&huart2, (uint8_t*)txData2, strlen(txData2), 7*baudrate/10);
	HAL_UART_Transmit(&huart2, &buffer2, 1, 1*baudrate/10);//ETX
	HAL_UART_Transmit(&huart2, (uint8_t*)txData3, strlen(txData3), 1*baudrate/10);//

//	sayac portu
	HAL_GPIO_WritePin(GPIOA, DE_RE_Pin,GPIO_PIN_SET); //usart4 sending
	HAL_UART_Transmit(&huart4, &buffer, 1, 1*baudrate/10); //SOH
	HAL_UART_Transmit(&huart4, (uint8_t*)txData1, strlen(txData1), 2*baudrate/10);//R2
	HAL_UART_Transmit(&huart4, &buffer1, 1, 1*baudrate/10);//STX
	HAL_UART_Transmit(&huart4, (uint8_t*)txData2, strlen(txData2), 7*baudrate/10);//
	HAL_UART_Transmit(&huart4, &buffer2, 1, 1*baudrate/10);//ETX
	HAL_UART_Transmit(&huart4, (uint8_t*)txData3, strlen(txData3), 1*baudrate/10);// no cr lf
	HAL_GPIO_WritePin(GPIOA, DE_RE_Pin,GPIO_PIN_RESET);

//alim dongusu
while(temp == 0){HAL_UART_Receive_DMA(&huart4, myRxData, 1);}
//gonderim dongusu
for(int i = 0; i<RXBUFFERSIZE-2 ; i++){

			uint8_t abuff[1] ;
			abuff[0]= buff[i];
			HAL_UART_Transmit(&huart2, abuff,1,50);
				//detecting end of data
				// LF or ACK or (ETX and not null	)
			if(buff[i] == 0x0a || buff[i] == 0x06 ||	(i >= 2 && buff[i-2] == 0x03 &&	buff[i] == 0x13) || buff[i] == 0x00){
				HAL_UART_Transmit(&huart2, (uint8_t*)("\r\nSayac cevabi\r\n"),16,50);
				break;
				}
			}
	sendxBee();
			temp =0;
		memset(buff, 0, RXBUFFERSIZE);
}
void commEnd (void){
	uint8_t buffer = 0x01; //<SOH>
	txData1 = "B0";
	uint8_t buffer1 = 0x03; //<ETX>
	txData2 = "q\r";
	HAL_GPIO_WritePin(GPIOA, DE_RE_Pin,GPIO_PIN_SET); //usart4 sending
	HAL_UART_Transmit(&huart4, &buffer, 1, 1*baudrate/10); //SOH
	HAL_UART_Transmit(&huart4, (uint8_t*)txData1, strlen(txData1), 2*baudrate/10);//
	HAL_UART_Transmit(&huart4, &buffer1, 1, 1*baudrate/10);//ETX
	HAL_UART_Transmit(&huart4, (uint8_t*)txData2, strlen(txData2), 7*baudrate/10);//
	HAL_GPIO_WritePin(GPIOA, DE_RE_Pin,GPIO_PIN_RESET);
	j = 0;
	while(temp == 0){HAL_UART_Receive_DMA(&huart4, myRxData, 1);}
	//gonderim dongusu
for(int i = 0; i<RXBUFFERSIZE-2 ; i++){

			uint8_t abuff[1] ;
			abuff[0]= buff[i];
			HAL_UART_Transmit(&huart2, abuff,1,50);
				//detecting end of data
				// LF or ACK or (ETX and not null	)
			if(buff[i] == 0x0a || buff[i] == 0x06 ||	(i >= 2 && buff[i-2] == 0x03 &&	buff[i] == 0x13) || buff[i] == 0x00){
					HAL_UART_Transmit(&huart2, (uint8_t*)("Finished\r\n"), 10, 50);
					break;
				}
			}
			temp =0;
		memset(buff, 0, RXBUFFERSIZE);

}
void 		addressParser				(uint8_t type)
{
	if(type == RECEIVE_TYPE){
		for(int i = 0; i < 8; i++){
			rxLongAddress[i] = buff[i + 4];					/* it is for Xbee S1.	*/
			//rxLongAddress[i] = myRxBuffer[i + 5];				/* it is for Xbee S2.	*/
		}
		rxShortAddress[0] = buff[12];
		rxShortAddress[1] = buff[13];/*
		HAL_UART_Transmit(&huart3,(uint8_t*)"\n\tLong: ",8,0x1000);
		HAL_UART_Transmit(&huart3,rxLongAddress,sizeof(rxLongAddress),0x1000);
		HAL_UART_Transmit(&huart3,(uint8_t*)"\n\tShort: ",9,0x1000);
		HAL_UART_Transmit(&huart3,rxShortAddress,sizeof(rxShortAddress),0x1000);*/
	}
	else{
		rxShortAddress[0] = buff[5];
		rxShortAddress[1] = buff[6];/*
		HAL_UART_Transmit(&huart1,(uint8_t*)"\n\tShort: ",8,0x1000);
		HAL_UART_Transmit(&huart1,rxShortAddress,sizeof(rxShortAddress),0x1000);*/
	}
	return;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
