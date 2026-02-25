/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : exemple_aht20_async_it_coverage.c
  * @brief          : Couverture API async AHT20 — callbacks, reset, GetData, ClearFlags, IRQ-safe
  ******************************************************************************
  *
  * Couvre les fonctions non présentes dans les exemples polling/async_it :
  *   - AHT20_Async_SetCallbacks()    — callbacks data-ready/erreur (main loop)
  *   - AHT20_Async_SetIrqCallbacks() — callbacks IRQ-safe (signal flag seulement)
  *   - AHT20_Async_Reset()           — reset contexte async (préserve callbacks)
  *   - AHT20_Async_Process()         — avance la FSM manuellement (sans Tick)
  *   - AHT20_Async_HasData()         — vrai après Process, avant GetData
  *   - AHT20_Async_GetData()         — récupère la donnée si HasData=true
  *   - AHT20_Async_DataReadyFlag()   — drapeau data-ready (vrai après Process)
  *   - AHT20_Async_ErrorFlag()       — drapeau erreur (vrai après Process)
  *   - AHT20_Async_ClearFlags()      — acquittement manuel des drapeaux
  *
  * CubeMX requis (Nucleo L476RG) :
  *   - I2C3 : SCL=PC0, SDA=PC1, Standard mode, Global interrupt activée
  *   - USART2 : 115200 bauds (printf)
  *
  * Compatibilité FreeRTOS :
  * - HAL_Delay() suspend la tâche courante → pas de spin-wait, CPU libéré ✅
  * - async_busy / state sont volatile → lecture safe depuis plusieurs tâches ✅
  * - Pour plusieurs tâches accédant au bus I2C : utiliser un mutex I2C (osMutexAcquire)
  * - AHT20_Async_Tick() à appeler depuis UNE SEULE tâche (⚠️)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "STM32_AHT20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME            "exemple_aht20_async_it_coverage"     ///< Identifiant log série
#define MEASUREMENT_INTERVAL_MS  2000u                            ///< Intervalle mesure async (ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static AHT20_Handle_t haht20;                        ///< Handle principal AHT20
static AHT20_Async   aht20_ctx;                      ///< Contexte asynchrone IT

/* Flags posés par callbacks IRQ (volatile), consommés dans la boucle principale */
static volatile bool irq_data_flag  = false;         ///< Signal data-ready depuis IRQ
static volatile bool irq_error_flag = false;         ///< Signal erreur depuis IRQ
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
/* Callback main-loop : données disponibles (appelé depuis AHT20_Async_Process) */
static void OnAHT20DataReady(void *user_ctx, const AHT20_Data *data,
                             AHT20_Status st);
/* Callback main-loop : erreur (appelé depuis AHT20_Async_Process) */
static void OnAHT20Error(void *user_ctx, AHT20_Status st);
/* Callback IRQ-safe : signal seulement — PAS d'accès capteur ici ! */
static void OnAHT20IrqDataReady(void *user_ctx);
/* Callback IRQ-safe : erreur signalée */
static void OnAHT20IrqError(void *user_ctx);
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

/* --- Callbacks main-loop -------------------------------------------------- */
/**
 * @brief  Callback main-loop : données disponibles (appelé depuis AHT20_Async_Process).
 * @param  user_ctx  Contexte utilisateur (NULL ici).
 * @param  data      Pointeur vers les données prises (non NULL si st==OK).
 * @param  st        Statut de la mesure.
 */
static void OnAHT20DataReady(void *user_ctx, const AHT20_Data *data,
                             AHT20_Status st)
{
  (void)user_ctx;
  if (st == AHT20_OK) {
    printf("OK  CB-DATA T=%.2f C | H=%.2f %%\r\n",
           data->temperature, data->humidity);
  }
}

/**
 * @brief  Callback main-loop : erreur async (appelé depuis AHT20_Async_Process).
 * @param  user_ctx  Contexte utilisateur (NULL ici).
 * @param  st        Code d'erreur.
 */
static void OnAHT20Error(void *user_ctx, AHT20_Status st)
{
  (void)user_ctx;
  printf("ERREUR  CB-ERR: %s\r\n", AHT20_StatusToString(st));
}

/* --- Callbacks IRQ-safe (← depuis l'interruption I2C) -------------------- */
/* RÈGLE : ne faire ici qu'un set de flag volatile — pas de printf, pas HAL ! */
static void OnAHT20IrqDataReady(void *user_ctx)
{
  (void)user_ctx;
  irq_data_flag = true;
}

static void OnAHT20IrqError(void *user_ctx)
{
  (void)user_ctx;
  irq_error_flag = true;
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
  printf("  AHT20 - Coverage async IT (callbacks, GetData, ClearFlags, IRQ-safe)\r\n");
  printf("========================================\r\n\r\n");

  /* 2) Initialisation capteur */
  printf("INFO  Initialisation AHT20...\r\n");
  AHT20_Status st = AHT20_Init(&haht20, &hi2c3);   // Soft reset + lecture statut + cmd 0xBE si CAL=0
  if (st != AHT20_OK) {
    printf("ERREUR  Init: %s\r\n", AHT20_StatusToString(st));
    Error_Handler();
  }
  printf("OK  AHT20 initialisé\r\n\r\n");

  /* 3) Initialisation contexte async + enregistrement callbacks */
  AHT20_Async_Init(&aht20_ctx, &haht20);                   // Réinitialise FSM, state=IDLE

  printf("INFO  Enregistrement callbacks main-loop (data-ready + erreur)...\r\n");
  AHT20_Async_SetCallbacks(&aht20_ctx,
                            OnAHT20DataReady,
                            OnAHT20Error,
                            NULL);                           // user_ctx = NULL (non utilisé ici)
  printf("OK  Callbacks main-loop enregistrés\r\n");

  printf("INFO  Enregistrement callbacks IRQ-safe (signal flag)...\r\n");
  AHT20_Async_SetIrqCallbacks(&aht20_ctx,
                               OnAHT20IrqDataReady,
                               OnAHT20IrqError,
                               NULL);                       // Callbacks IRQ-safe : set flag uniquement
  printf("OK  Callbacks IRQ enregistrés\r\n");

  printf("INFO  AHT20_Async_Reset : simule récupération d'erreur (callbacks conservés)...\r\n");
  AHT20_Async_Reset(&aht20_ctx);                           // Remet state=IDLE, préserve callbacks
  printf("OK  Reset contexte async OK\r\n\r\n");

  /* 4) Premier déclenchement / validation MX */
  st = AHT20_ReadAll_IT(&aht20_ctx);                       // Déclenche 1ère mesure — valide config I2C + IRQ
  if (st != AHT20_OK) {
    printf("ERREUR  Configuration MX incomplète (I2C/IRQ) pour mode IT\r\n");
    printf("   Action: vérifier I2C choisi + IRQ EV/ER puis régénérer\r\n");
    Error_Handler();
  }
  printf("INFO  Première mesure lancée...\r\n");
  printf("   Validation MX (I2C/IRQ) effectuée par la librairie\r\n\r\n");

  /* 5) Paramètres runtime de la boucle principale */
  uint32_t last_trigger = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* Flags IRQ : consommés en main-loop, jamais longtemps pending */
    if (irq_data_flag) {
      irq_data_flag = false;
      printf("INFO  IRQ data-ready signal reçu\r\n");
    }
    if (irq_error_flag) {
      irq_error_flag = false;
      printf("INFO  IRQ erreur signal reçu\r\n");
    }

    /* Avance la FSM (TX→wait→RX→done) — appelle OnDataReady/OnError via callbacks en interne */
    AHT20_Async_Process(&aht20_ctx, now);

    /* DataReadyFlag : vrai après Process quand state=DONE, avant consommation par GetData */
    if (AHT20_Async_DataReadyFlag(&aht20_ctx)) {
      printf("INFO  DataReadyFlag actif\r\n");
    }

    /* HasData + GetData : consomme la donnée, remet state=IDLE, data_ready_flag=false */
    if (AHT20_Async_HasData(&aht20_ctx)) {
      AHT20_Data sample;
      if (AHT20_Async_GetData(&aht20_ctx, &sample) == AHT20_OK) {
        printf("OK  Process+HasData+GetData T=%.2f C | H=%.2f %%\r\n",
               sample.temperature, sample.humidity);
      }
      /* ClearFlags : acquittement explicite des drapeaux après GetData */
      AHT20_Async_ClearFlags(&aht20_ctx);
      last_trigger = now;
    }

    /* ErrorFlag : levé par Process sur timeout ou erreur HAL_I2C */
    if (AHT20_Async_ErrorFlag(&aht20_ctx)) {
      printf("INFO  ErrorFlag actif\r\n");
      printf("ERREUR  Async: %s\r\n", AHT20_StatusToString(AHT20_GetLastError(&haht20)));
      AHT20_Async_ClearFlags(&aht20_ctx);
    }

    /* Nouveau déclenchement périodique */
    if (AHT20_Async_IsIdle(&aht20_ctx) &&
        (now - last_trigger >= MEASUREMENT_INTERVAL_MS))
    {
      AHT20_Async_ClearFlags(&aht20_ctx);
      AHT20_ReadAll_IT(&aht20_ctx);
      last_trigger = now;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* AHT20_DeInit(&haht20); */
  /* À appeler en sortie de boucle (reset logiciel, bootloader, tests unitaires)
     pour remettre le handle à zéro avant une ré-initialisation éventuelle. */
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
  /* ⚠️ CubeMX : cocher "I2C3 event interrupt" + "I2C3 error interrupt" dans NVIC Settings */
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
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  AHT20_Async_OnI2CMasterTxCplt(&aht20_ctx, hi2c);   /* La lib filtre hi2c en interne */
}
/**
 * @brief  Callback HAL I2C de fin de réception (IT).
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  AHT20_Async_OnI2CMasterRxCplt(&aht20_ctx, hi2c);   /* La lib filtre hi2c en interne */
}
/**
 * @brief  Callback HAL I2C d'erreur.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
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
    /* Handler d'erreur bloquant: LED clignotante pour diagnostic visuel. */
    __disable_irq();
    while (1)  // Boucle d'erreur bloquante
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);                           // Fait clignoter la LED d'erreur
      for (volatile uint32_t wait = 0U; wait < 250000U; ++wait) {          // Temporisation locale 250ms sans HAL_Delay
        __NOP();                                                            // Occupation CPU minimale pour espacer le clignotement
      }
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
