/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : exemple_aht20_async_it.c
  * @brief          : Exemple AHT20 async non-bloquant (I2C IT)
  ******************************************************************************
  * Pré-requis CubeMX:
  * - I2C3 activé (I2C, addressing 7-bit) + broches SCL/SDA configurées par CubeMX
  *   (pull-ups externes requis sur le bus I2C, typiquement 4.7 kΩ)
  * - NVIC: activer "I2C3 event interrupt" et "I2C3 error interrupt"
  * - USART2 optionnel (Asynchronous, 115200 8N1) si tu utilises printf via __io_putchar()
  *
  * Notes:
  * - Fichier d'exemple (non compilé par défaut)
  * - La mesure est déclenchée toutes les 2 s.
  * - La boucle principale ne bloque pas: progression via callbacks I2C + AHT20_Async_Tick().
  * - AHT20_MAX_CONSECUTIVE_ERRORS : seuil de détection panne prolongée (défaut: 3).
  *
  * Compatibilité FreeRTOS :
  * - HAL_Delay() suspend la tâche courante → pas de spin-wait, CPU libéré ✅
  * - async_busy / async_state sont volatile → lecture safe depuis plusieurs tâches ✅
  * - Pour plusieurs tâches accédant au bus I2C : utiliser un mutex I2C (osMutexAcquire)
  * - Init() uniquement depuis une tâche d'init (jamais depuis une IRQ) ✅
  * - AHT20_Async_Tick() à appeler depuis UNE SEULE tâche (pas thread-safe multi-tick) ⚠️
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM32_AHT20.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME "exemple_aht20_async_it"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static AHT20_Handle haht20;                         ///< Handle principal AHT20
static AHT20_Async  aht20_ctx;                      ///< Contexte asynchrone IT
static uint32_t last_trigger_time = 0;              ///< Timestamp dernière mesure (TriggerEvery)
static uint32_t measure_count = 0;                  ///< Compteur mesures valides
static uint8_t preflight_error_count = 0;           ///< Compteur erreurs préliminaires
static bool first_valid_sample = false;             ///< true dès la première mesure valide
/* Intervalle entre mesures : contrôlé via haht20.sample_interval_ms (défaut : AHT20_DEFAULT_SAMPLE_INTERVAL_MS = 2000 ms) */
static uint8_t async_error_count = 0;               ///< Suivi erreurs async — recovery via DeInit/Init
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  Redirige stdout vers l'UART pour printf() (retarget newlib).
 * @param  ch  Caractère à transmettre (int pour compatibilité stdio).
 * @retval ch  Caractère transmis (convention stdio).
 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF); // Envoie 1 octet via UART — timeout 0xFFFF ms
    // ITM_SendChar(ch);                 // Alternative : sortie via SWO (debug ITM)
    return ch;
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
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  /* 1) Bannière exemple */
  printf("\r\n========================================\r\n");
  printf("  Fichier: " LOG_NAME "\r\n");
  printf("  AHT20 - Async IT (non-bloquant)\r\n");
  printf("========================================\r\n\r\n");

  /* 2) Initialisation capteur */
  printf("INFO  Initialisation AHT20...\r\n");
  AHT20_Status status = AHT20_Init(&haht20, &hi2c3);   // Soft reset + lecture statut + cmd 0xBE si CAL=0
  if (status != AHT20_OK) {
    printf("ERREUR  Init: %s\r\n", AHT20_StatusToString(status));
    Error_Handler();
  }
  printf("OK  AHT20 initialisé\r\n\r\n");

  /* 3) Initialisation contexte async */
  AHT20_Async_Init(&aht20_ctx, &haht20);              // Réinitialise FSM, state=IDLE
  printf("OK  Contexte async initialisé\r\n");
  printf("   Mode : Interruptions I2C (IT)\r\n");
  printf("   Intervalle : %lu ms (AHT20_DEFAULT_SAMPLE_INTERVAL_MS)\r\n",
         (unsigned long)haht20.sample_interval_ms);
  printf("   Base de temps : SysTick (HAL_GetTick)\r\n");
  printf("   Callbacks HAL I2C dans USER CODE BEGIN 4\r\n\r\n");

  /* 4) Premier déclenchement / validation MX */
  AHT20_Status trig_init = AHT20_ReadAll_IT(&aht20_ctx); // Déclenche 1ère mesure — valide config I2C + IRQ
  if (trig_init != AHT20_OK) {
    printf("ERREUR  Configuration MX incomplète (I2C/IRQ) pour mode IT\r\n");
    printf("   Action : .ioc -> Connectivity -> I2Cx -> Configuration -> NVIC Settings\r\n");
    printf("            cocher I2C event interrupt + I2C error interrupt\r\n");
    printf("   Code   : %s\r\n", AHT20_StatusToString(trig_init));
    Error_Handler();
  }
  last_trigger_time = HAL_GetTick();
  printf("INFO  Première mesure lancée...\r\n");
  printf("   Validation MX (I2C/IRQ) effectuée par la librairie\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t now = HAL_GetTick();
        AHT20_Data data;

        /* Déclenchement mesure périodique — TriggerEvery = IsIdle + timer + ReadAll_IT en une seule ligne */
        /* L'intervalle est haht20.sample_interval_ms (AHT20_DEFAULT_SAMPLE_INTERVAL_MS = 2000ms par défaut) */
        status = AHT20_Async_TriggerEvery(&aht20_ctx, now, &last_trigger_time);
        if (status == AHT20_OK) {
          if (first_valid_sample) {
            printf("INFO  Nouvelle mesure...\r\n");
          }
        } else if (status != AHT20_ERR_BUSY) {
          /* Erreur déclenchement réelle (hors ERR_BUSY = FSM occupée ou intervalle non atteint) */
          if (first_valid_sample) {
          printf("INFO  Trigger échoué: %s\r\n", AHT20_StatusToString(status));
          }
          if (!first_valid_sample &&
              (status == AHT20_ERR_TIMEOUT || status == AHT20_ERR_I2C)) {
            preflight_error_count++;
            if (preflight_error_count >= 2U) {
              printf("ERREUR  Configuration MX incomplète (I2C/IRQ) détectée\r\n");
              printf("   Action : .ioc -> Connectivity -> I2Cx -> Configuration -> NVIC Settings\r\n");
              printf("            cocher I2C event interrupt + I2C error interrupt\r\n");
              printf("   Dernier code : %s\r\n", AHT20_StatusToString(status));
              Error_Handler();
            }
          }
        }

        AHT20_TickResult tick = AHT20_Async_Tick(&aht20_ctx, now, &data);
        if (tick == AHT20_TICK_DATA_READY) {
          first_valid_sample = true;
          preflight_error_count = 0U;
          measure_count++;
          printf("[#%lu] OK  T=%.2f C | RH=%.1f %%\r\n",
                 measure_count, data.temperature, data.humidity);
        } else if (tick == AHT20_TICK_ERROR) {
          AHT20_Status err = haht20.last_error;                                 // Champ public du handle
          if (!first_valid_sample &&
              (err == AHT20_ERR_TIMEOUT || err == AHT20_ERR_BUSY || err == AHT20_ERR_I2C)) {
            preflight_error_count++;
            if (preflight_error_count >= 2U) {
              printf("ERREUR  Configuration MX incomplète (I2C/IRQ) détectée\r\n");
              printf("   Action : .ioc -> Connectivity -> I2Cx -> Configuration -> NVIC Settings\r\n");
              printf("            cocher I2C event interrupt + I2C error interrupt\r\n");
              printf("   Dernier code : %s\r\n", AHT20_StatusToString(err));
              Error_Handler();
            }
          }
          if (first_valid_sample) {
            async_error_count++;
            printf("ERREUR  Erreur async [%d/%d]: %s\r\n",
                   async_error_count, AHT20_MAX_CONSECUTIVE_ERRORS,
                   AHT20_StatusToString(err));

            /* Recovery après AHT20_MAX_CONSECUTIVE_ERRORS erreurs async consécutives */
            if (async_error_count >= AHT20_MAX_CONSECUTIVE_ERRORS) {
              printf("INFO  Recovery DeInit + Init...\r\n");
              AHT20_DeInit(&haht20);
              if (AHT20_Init(&haht20, &hi2c3) == AHT20_OK) {
                AHT20_Async_Init(&aht20_ctx, &haht20);
                async_error_count = 0;
                last_trigger_time = HAL_GetTick();
                printf("OK  Recovery OK\r\n");
              }
            }
          }
        }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */
  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */
  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */
  /* USER CODE END I2C3_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Callback HAL I2C de fin de transmission (IT).
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  AHT20_Async_OnI2CMasterTxCplt(&aht20_ctx, hi2c);   /* La lib filtre hi2c en interne */
}

/**
 * @brief  Callback HAL I2C de fin de réception (IT).
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  AHT20_Async_OnI2CMasterRxCplt(&aht20_ctx, hi2c);   /* La lib filtre hi2c en interne */
}

/**
 * @brief  Callback HAL I2C d'erreur.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  AHT20_Async_OnI2CError(&aht20_ctx, hi2c);           /* La lib filtre hi2c en interne */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {
        // Blink LED ou autre indication
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
