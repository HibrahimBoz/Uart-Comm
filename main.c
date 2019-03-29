
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

 uint8_t myRxData [1], buff[RXBUFFERSIZE] ;
/* USER CODE END PV */



/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

 char *txData,*txData1, *txData2, *txData3, *rxData, *sData;
 int rtime, temp =0, j = 0;
 int baudrate =300;
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

	txData = "/?!\r\n";	
		
			
j = 0;	// callBack içinde sürekli sayim yapacak deger baslangiçta sifirlanir

	HAL_UART_Transmit(&huart2, (uint8_t*)txData, strlen(txData), (baudrate/2)*strlen(sData)); // kontrol port
		HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_SET); //RS485 gpio pini ile kontrol edilir, iletim aktif
	HAL_UART_Transmit(&huart4, (uint8_t*)txData, strlen(txData), (baudrate/2)*strlen(sData));	// ana portna gönderilir ki hemen diger koda gecikmesiz halde gecsin
		HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_RESET);// gönderim aktif
			
while(temp == 0){HAL_UART_Receive_DMA(&huart4, myRxData, 1);} //gelen veriyi tek tek yakalar, bu islem bitmeden dongüden cikmaz
//transmiting data //yakalanan veriyi iletim kismi, tek tek gönderilir

for(int i = 0; i<RXBUFFERSIZE ; i++){
			
			uint8_t abuff[1] ;
			abuff[0]= buff[i];
			HAL_UART_Transmit(&huart2, abuff,1,50);
				//detecting end of data
				// LF or ACK or (ETX and not null	) tespitinde mesajin sonu oldugu anlasilir ve döngü sona erer
			if(buff[i] == 0x0a || buff[i] == 0x06 ||	(buff[i-2] == 0x03 &&	buff[i] == 0x13 )){		
				break;
				}
			}
			temp =0;//sonraki alim döngüsüne girmesi icin bu deger sifirlanir
		memset(buff, NULL, 50); // uzerine yazma olma olmamasi icin ana dizi sifirlanir

		uint8_t buffer = 0x06; //<ACK>        //yazdirilamaz karakter ile normal yarakterler ayri gönderilir
		txData= "051\r\n";                    //yazdirilamaz karakter ile normal yarakterler ayri gönderilir
					
	j = 0;
   
		HAL_UART_Transmit(&huart2, &buffer, 1, (baudrate/10)*1);                  
		HAL_UART_Transmit(&huart2, (uint8_t*)txData, strlen(txData), 300*4);      
	HAL_GPIO_WritePin(DE_RE_GPIO_Port, DE_RE_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart4, &buffer, 1, (baudrate/10)*1);
		HAL_UART_Transmit(&huart4, (uint8_t*)txData, strlen(txData), 300*4);
	HAL_GPIO_WritePin(GPIOA, DE_RE_Pin,GPIO_PIN_RESET);

while(temp == 0){HAL_UART_Receive_DMA(&huart4, myRxData, 1);}
//transmiting data:
for(int i = 0; i<49 ; i++){
			
	uint8_t abuff[1] ;
	abuff[0]= buff[i];
	HAL_UART_Transmit(&huart2, abuff,1,50);
		//detecting end of data
		// LF or ACK or (ETX and not null	)
	if(buff[i] == 0x0a || buff[i] == 0x06 || (buff[i-2] == 0x03 &&	buff[i] == 0x13 )){		
		break;
		}
	}
temp =0;
memset(buff, NULL, 50);
				

 }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 
  /* USER CODE END 3 */
}
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
	buff[j] = myRxData[0];
	
	if(buff[j] == 0x0a || buff[j] == 0x06 || (buff[j-1] == 0x03 &&	buff[j] != 0x00 )|| buff[j] == 0x00){		
		temp++;
		}
	j++;
} 
