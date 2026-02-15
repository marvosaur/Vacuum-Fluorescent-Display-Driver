/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MN12832JC Driver - High Refresh (400Hz)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "u8g2.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    ANIM_WAIT,          //filament warmup
    ANIM_DOT_TO_LINE,   //expand width
    ANIM_LINE_TO_BORDER,//expand height
    ANIM_INFO_HOLD,     //hold text
    ANIM_TEXT_COLLAPSE, //squeeze text vertically
    ANIM_SHRINK_HEIGHT, //reverse border
    ANIM_SHRINK_WIDTH,  //reverse line
    ANIM_DONE           //exit
} AnimationState;

typedef enum {
    UART_WAIT_SYNC1,
    UART_WAIT_SYNC2,
    UART_WAIT_BRIGHTNESS,
    UART_RECEIVE_DATA
} UartRxState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//hardware mappings
#define PORT_SIG    GPIOA
#define PIN_SIG     GPIO_PIN_0

#define PORT_CLKG   GPIOA
#define PIN_CLKG    GPIO_PIN_1

#define PORT_BLANK1 GPIOA
#define PIN_BLANK1  GPIO_PIN_8

#define PORT_BKG    GPIOB
#define PIN_BKG     GPIO_PIN_6

#define PORT_LATG   GPIOB
#define PIN_LATG    GPIO_PIN_7

#define PORT_LAT    GPIOB
#define PIN_LAT     GPIO_PIN_10

#define PORT_BLANK2 GPIOB
#define PIN_BLANK2  GPIO_PIN_11

#define PORT_TOUCH  GPIOA
#define PIN_TOUCH   GPIO_PIN_9

//fast gpio macros
#define FAST_PIN_HIGH(port, pin) (port)->BSRR = (pin)
#define FAST_PIN_LOW(port, pin)  (port)->BSRR = ((uint32_t)(pin) << 16)

//delay macros (72mhz)
#define DELAY_LAT() { \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
}

#define DELAY_CLK() { \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//dwt cycle counter macros
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004)
#define SCB_DEMCR  (*(volatile uint32_t *)0xE000EDFC)

//helper to enable the counter
#define ENABLE_CYCLES() { \
    SCB_DEMCR |= 0x01000000; \
    DWT_CYCCNT = 0; \
    DWT_CTRL |= 1; \
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//global u8g2 instance
u8g2_t u8g2;

//double buffered physical buffer
uint8_t vfdDmaBuffer[2][64][8];

//buffer control
volatile uint8_t activeBufferIdx = 0;
volatile uint8_t backBufferIdx = 1;
volatile uint8_t frameReady = 0;

volatile uint32_t vsyncCounter = 0;

//current grid step for muxing
volatile uint8_t timingStep = 0;

//uart reception vars
#define RX_BUFFER_SIZE 512
uint8_t rxBuffers[2][RX_BUFFER_SIZE]; //double rx buffers
volatile uint8_t rxWriteBank = 0;     //uart write buffer
volatile uint8_t rxReadBank = 1;      //main loop read buffer

uint8_t rxByte;
volatile uint8_t rxState = 0;
volatile uint16_t rxIndex = 0;
volatile uint8_t newFrameReceived = 0;

//animation config
const uint32_t animFrameWait = 2; //ticks between updates
const int animWarmupTime = 60;    //warmup duration
const int animHoldTime = 120;     //text hold duration
const float animCollapseSpeed = 0.5f;

//pid const for animation
const float Kp_W = 0.03f;
const float Kp_H = 0.05f;

//dimensions
const float dispW = 128.0f;
const float dispH = 32.0f;

volatile uint8_t globalBrightness = 255;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void updateDmaBuffer(void);
uint8_t u8x8_d_vfd_128x32(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);
uint8_t u8x8GpioAndDelayStm32(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr);

void startSequenceAnim(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        //unblank display
        FAST_PIN_LOW(PORT_BKG, PIN_BKG);
    }
}

uint8_t u8x8GpioAndDelayStm32(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr) {
    return 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        switch (rxState) {
            case UART_WAIT_SYNC1:
                if (rxByte == 0xAA) rxState = UART_WAIT_SYNC2;
                break;

            case UART_WAIT_SYNC2:
                if (rxByte == 0x55) {
                    rxState = UART_WAIT_BRIGHTNESS; //go to brightness
                } else {
                    rxState = UART_WAIT_SYNC1;
                }
                break;

            case UART_WAIT_BRIGHTNESS:
                globalBrightness = rxByte; //store brightness
                rxIndex = 0;
                rxState = UART_RECEIVE_DATA; //get data
                break;

            case UART_RECEIVE_DATA:
                rxBuffers[rxWriteBank][rxIndex++] = rxByte;
                if (rxIndex >= RX_BUFFER_SIZE) {
                    rxReadBank = rxWriteBank;
                    newFrameReceived = 1;
                    rxWriteBank ^= 1;
                    rxState = UART_WAIT_SYNC1;
                }
                break;
        }
        HAL_UART_Receive_IT(&huart2, &rxByte, 1);
    }
}

void updateDmaBuffer(void) {
    uint8_t targetBuf = backBufferIdx;
    uint8_t(*pDest)[8] = vfdDmaBuffer[targetBuf];

    //get raw u8g2 buffer
    const uint8_t* src = u8g2_GetBufferPtr(&u8g2);

    for (int t = 0; t < 64; t++) {
        //physical grid calc
        uint8_t colFirst = (t + 1) << 1;
        if (colFirst >= 128) colFirst = 0;
        uint8_t colLast = (colFirst == 0) ? 127 : colFirst - 1;

        //flip x logic
        uint8_t srcColF = 127 - colFirst;
        uint8_t srcColL = 127 - colLast;

        //fetch flipped vertical data
        uint32_t colFData =
            (src[srcColF]) |               //rows 0-7
            (src[srcColF + 128] << 8) |    //rows 8-15
            (src[srcColF + 256] << 16) |   //rows 16-23
            (src[srcColF + 384] << 24);    //rows 24-31

        uint32_t colLData =
            (src[srcColL]) |
            (src[srcColL + 128] << 8) |
            (src[srcColL + 256] << 16) |
            (src[srcColL + 384] << 24);

        //unpack into vfd format
        for (int byte = 0; byte < 8; byte++) {
            //vfd map logic
            int shift = 28 - (byte * 4);
            uint8_t packed = 0;

            //flip y because i like it upside down

            //logic row 31->phys 0
            if ((colFData >> (28 - shift)) & 1) packed |= (1 << 6);
            if ((colLData >> (28 - shift)) & 1) packed |= (1 << 7);

            //logic row 30->phys 1
            if ((colFData >> (29 - shift)) & 1) packed |= (1 << 4);
            if ((colLData >> (29 - shift)) & 1) packed |= (1 << 5);

            //logic row 29->phys 2
            if ((colFData >> (30 - shift)) & 1) packed |= (1 << 2);
            if ((colLData >> (30 - shift)) & 1) packed |= (1 << 3);

            //logic row 28->phys 3
            if ((colFData >> (31 - shift)) & 1) packed |= (1 << 0);
            if ((colLData >> (31 - shift)) & 1) packed |= (1 << 1);

            pDest[t][byte] = packed;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM2) {
        //blank display start of cycle
        FAST_PIN_HIGH(PORT_BKG, PIN_BKG);

        //calc brightness on-time
        uint32_t onTime = 75 - (globalBrightness * 70 / 255);
        if (onTime < 6) onTime = 6;

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, onTime);

        //vsync and swap
        if (timingStep == 64) {
            vsyncCounter++;
            if (frameReady == 1) {
                activeBufferIdx = backBufferIdx;
                backBufferIdx ^= 1;
                frameReady = 0;
            }
        }

        //load data during blank
        FAST_PIN_HIGH(PORT_LAT, PIN_LAT);
        DELAY_LAT();
        FAST_PIN_LOW(PORT_LAT, PIN_LAT);

        timingStep++;
        if (timingStep > 64) timingStep = 1;

        if (timingStep == 1 || timingStep == 2) FAST_PIN_HIGH(PORT_SIG, PIN_SIG);
        else                                    FAST_PIN_LOW(PORT_SIG, PIN_SIG);

        FAST_PIN_LOW(PORT_CLKG, PIN_CLKG);
        DELAY_CLK();
        FAST_PIN_HIGH(PORT_CLKG, PIN_CLKG);

        //mux scan line
        FAST_PIN_HIGH(PORT_BLANK1, PIN_BLANK1);
        FAST_PIN_HIGH(PORT_BLANK2, PIN_BLANK2);
        if (timingStep % 2 != 0) FAST_PIN_LOW(PORT_BLANK1, PIN_BLANK1);
        else                     FAST_PIN_LOW(PORT_BLANK2, PIN_BLANK2);

        //trigger dma
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = DMA_IFCR_CGIF3;
        DMA1_Channel3->CMAR = (uint32_t)vfdDmaBuffer[activeBufferIdx][timingStep - 1];
        DMA1_Channel3->CNDTR = 8;
        DMA1_Channel3->CCR |= DMA_CCR_EN;
    }
}

void startSequenceAnim(void) {
    AnimationState animState = ANIM_WAIT;
    uint16_t timer = 0;

    //animation physics
    float animWidth = 2.0f;
    float animHeight = 2.0f;
    float textYOffset = 0.0f; //text collapse

    while (animState != ANIM_DONE) {
        //sync lock
        uint32_t targetSync = vsyncCounter + animFrameWait;

        u8g2_ClearBuffer(&u8g2);

        //warmup
        if (animState == ANIM_WAIT) {
            timer++;
            if (timer > animWarmupTime) animState = ANIM_DOT_TO_LINE;

            const char* warmupStr = "- - warming up - -";
            int strWidth = u8g2_GetStrWidth(&u8g2, warmupStr);
            u8g2_DrawStr(&u8g2, (128 - strWidth) / 2, 19, warmupStr);
        }

        //opening sequence
        else if (animState == ANIM_DOT_TO_LINE) {
            float error = dispW - animWidth;
            animWidth += error * Kp_W;

            int w = (int)animWidth;
            u8g2_DrawBox(&u8g2, 64 - (w / 2), 15, w, 2);

            if (animWidth > (dispW - 2.0f)) {
                animWidth = dispW;
                animState = ANIM_LINE_TO_BORDER;
            }
        }
        else if (animState == ANIM_LINE_TO_BORDER) {
            float error = dispH - animHeight;
            animHeight += error * Kp_H;

            int h = (int)animHeight;
            u8g2_DrawFrame(&u8g2, 0, 16 - (h / 2), (int)dispW, h);

            if (animHeight > (dispH - 1.0f)) {
                animHeight = dispH;
                timer = 0;
                animState = ANIM_INFO_HOLD;
            }
        }

        //show info
        else if (animState == ANIM_INFO_HOLD) {
            timer++;
            u8g2_DrawFrame(&u8g2, 0, 0, (int)dispW, (int)dispH);

            const char* line1 = "MN12832JC VFD Driver";
            const char* line2 = "V. 1.5.0 - Marvin Tian";

            int w1 = u8g2_GetStrWidth(&u8g2, line1);
            int w2 = u8g2_GetStrWidth(&u8g2, line2);

            u8g2_DrawStr(&u8g2, (128 - w1) / 2, 12, line1);
            u8g2_DrawStr(&u8g2, (128 - w2) / 2, 22, line2);

            if (timer > animHoldTime) {
                animState = ANIM_TEXT_COLLAPSE;
            }
        }

        //text collapse
        else if (animState == ANIM_TEXT_COLLAPSE) {
            u8g2_DrawFrame(&u8g2, 0, 0, (int)dispW, (int)dispH);

            textYOffset += animCollapseSpeed;

            int y1 = 12 + (int)textYOffset;
            int y2 = 22 - (int)textYOffset;

            const char* line1 = "MN12832JC VFD Driver";
            const char* line2 = "V. 1.5.0 - Marvin Tian";

            int w1 = u8g2_GetStrWidth(&u8g2, line1);
            int w2 = u8g2_GetStrWidth(&u8g2, line2);

            if (y1 < 18) u8g2_DrawStr(&u8g2, (128 - w1) / 2, y1, line1);
            if (y2 > 14) u8g2_DrawStr(&u8g2, (128 - w2) / 2, y2, line2);

            if (y1 >= 22) {
                animState = ANIM_SHRINK_HEIGHT;
            }
        }

        //closing sequence
        else if (animState == ANIM_SHRINK_HEIGHT) {
            float error = 2.0f - animHeight;
            animHeight += error * Kp_H;

            int h = (int)animHeight;
            if (h < 2) h = 2;

            u8g2_DrawFrame(&u8g2, 0, 16 - (h / 2), (int)dispW, h);

            if (animHeight < 3.0f) {
                animHeight = 2.0f;
                animState = ANIM_SHRINK_WIDTH;
            }
        }
        else if (animState == ANIM_SHRINK_WIDTH) {
            float error = 0.0f - animWidth;
            animWidth += error * Kp_W;

            int w = (int)animWidth;
            if (w < 0) w = 0;

            if (w > 0) {
                u8g2_DrawBox(&u8g2, 64 - (w / 2), 15, w, 2);
            }

            if (animWidth < 1.0f) {
                animState = ANIM_DONE;
            }
        }

        updateDmaBuffer();

        //request buffer swap
        frameReady = 1;

        //wait for sync
        while (vsyncCounter < targetSync) { __NOP(); }
    }

    //final cleanup
    u8g2_ClearBuffer(&u8g2);
    updateDmaBuffer();
}
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
      //startup blink
      for(int i = 0; i < 6; i++) {
          HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
          HAL_Delay(50);
      }

      __HAL_SPI_ENABLE(&hspi1);
      SPI1->CR2 |= SPI_CR2_TXDMAEN;

      //dma config
      DMA1_Channel3->CCR = 0;
      DMA1_Channel3->CPAR = (uint32_t) & (SPI1->DR);
      DMA1_Channel3->CMAR = 0;
      DMA1_Channel3->CNDTR = 8;

      //minc enable
      uint32_t config = DMA_CCR_MINC | DMA_CCR_DIR | (0x2 << 12);
      DMA1_Channel3->CCR = config;

      HAL_GPIO_WritePin(PORT_LATG, PIN_LATG, GPIO_PIN_SET);
      HAL_GPIO_WritePin(PORT_CLKG, PIN_CLKG, GPIO_PIN_SET);
      HAL_GPIO_WritePin(PORT_BLANK1, PIN_BLANK1, GPIO_PIN_SET);
      HAL_GPIO_WritePin(PORT_BLANK2, PIN_BLANK2, GPIO_PIN_SET);
      HAL_GPIO_WritePin(PORT_BKG, PIN_BKG, GPIO_PIN_RESET);

      ENABLE_CYCLES();

      //u8g2 init
      u8g2_Setup_ssd1306_128x32_univision_f(
          &u8g2,
          U8G2_R0,
          u8x8_byte_empty,
          u8x8GpioAndDelayStm32
      );

      //wake up
      u8g2_InitDisplay(&u8g2);
      u8g2_SetPowerSave(&u8g2, 0);

      //set font
      u8g2_SetFont(&u8g2, u8g2_font_4x6_tr);

      //start hardware
      updateDmaBuffer();
      HAL_TIM_Base_Start_IT(&htim2);              // Starts PeriodElapsed (Turn OFF)
      HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

      //enable hv boost
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

      //play startup sequence
      startSequenceAnim();

      //start rx interrupt
      HAL_UART_Receive_IT(&huart2, &rxByte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
        {
            //copy new packet
            if (newFrameReceived) {
                 uint8_t* u8g2Ptr = u8g2_GetBufferPtr(&u8g2);
                 __disable_irq();
                 uint8_t targetBank = rxReadBank;
                 __enable_irq();
                 memcpy(u8g2Ptr, rxBuffers[targetBank], RX_BUFFER_SIZE);
                 newFrameReceived = 0;
            }

            //update driver buffer
            updateDmaBuffer();

            //wait for vsync
            frameReady = 1;
            while (frameReady == 1) { __NOP(); }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 75-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SIG_Pin|CLKG_Pin|BLANK1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LAT_Pin|BLANK2_Pin|BKG_Pin|LATG_Pin
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SIG_Pin CLKG_Pin BLANK1_Pin */
  GPIO_InitStruct.Pin = SIG_Pin|CLKG_Pin|BLANK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LAT_Pin BKG_Pin LATG_Pin */
  GPIO_InitStruct.Pin = LAT_Pin|BKG_Pin|LATG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BLANK2_Pin PB8 */
  GPIO_InitStruct.Pin = BLANK2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
