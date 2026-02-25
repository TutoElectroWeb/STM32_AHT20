/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : 'exemple_aht20_polling_reset.c'
 * @brief          : Exemple soft reset et récupération d'erreurs AHT20
 * @date           : 2026-02-12
 ******************************************************************************
 * @description
 * Démontre l'utilisation du soft reset pour récupérer d'erreurs :
 * - Utilisation de AHT20_SoftReset() pour réinitialiser le capteur
 * - Récupération automatique après erreur de calibration
 * - Test de robustesse avec reset périodique
 * - Comparaison état avant/après reset
 *
 * Use case :
 * - Récupération après erreur de communication prolongée
 * - Réinitialisation capteur sans coupure alimentation
 * - Test de stabilité après perturbation I2C
 *
 * Configuration CubeMX requise :
 * - I2C3 : Mode I2C, Speed Mode Standard (100 kHz)
 * - USART2 : Asynchronous, 115200 baud, 8N1
 * - SysTick activé
 *
 * Hardware :
 * - AHT20 sur I2C3 (adresse 0x38)
 * - Pull-up 4.7kΩ sur SDA/SCL recommandées
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
#define LOG_NAME "exemple_aht20_polling_reset"              ///< Identifiant log série
#define RESET_TEST_INTERVAL_MS 30000              ///< Période du reset périodique (ms) — valeur élevée pour test de stabilité
#define MEASUREMENT_INTERVAL_MS 2000              ///< Intervalle entre mesures (ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static AHT20_Handle_t haht20;            ///< Handle principal AHT20
static uint32_t reset_count = 0;         ///< Compteur de resets effectués (diagnostic)
static uint32_t last_reset_tick = 0;     ///< Timestamp dernier reset périodique (HAL_GetTick)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
static AHT20_Status PerformSoftResetAndReinit(void);
static void CheckAndPrintStatus(const char* label);
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

/**
 * @brief Effectue un soft reset puis réinitialise le capteur
 * @return AHT20_OK si succès, code d'erreur sinon
 */
static AHT20_Status PerformSoftResetAndReinit(void)
{
    AHT20_Status status;

  printf("INFO  Soft Reset...\r\n");

    status = AHT20_SoftReset(&haht20);                // Envoie commande 0xBA, attend 20ms
    if (status != AHT20_OK) {
      printf("ERREUR  SoftReset échoué: %s\r\n", AHT20_StatusToString(status));
        return status;
    }

    printf("OK  SoftReset OK (délai 20ms intégré)\r\n");

    HAL_Delay(50);  /* Marge supplémentaire post-reset (optionnel, non exigé datasheet) */

    /* Note : AHT20_Init appelle AHT20_SoftReset en interne (20ms supplémentaires).
     * Ce second soft reset est redondant ici, mais inoffensif. Dans un vrai produit,
     * remplacer cet appel par AHT20_Init seul suffit à reset + réinit le capteur. */
    status = AHT20_Init(&haht20, &hi2c3);             /* Ré-initialise (inclut un SoftReset) */
    if (status != AHT20_OK) {
      printf("ERREUR  Ré-init échouée: %s\r\n", AHT20_StatusToString(status));
        return status;
    }

    printf("OK  Ré-init OK\r\n");
    reset_count++;
    last_reset_tick = HAL_GetTick();

    return AHT20_OK;
}

/**
 * @brief Vérifie et affiche le statut du capteur
 * @param label Texte descriptif pour identifier le contexte
 */
static void CheckAndPrintStatus(const char* label)
{
    uint8_t status_byte;
    AHT20_Status status = AHT20_GetStatus(&haht20, &status_byte);

    if (status == AHT20_OK) {
      printf("INFO  %s - Status: 0x%02X [Busy:%d, Cal:%d]\r\n",
               label,
               status_byte,
               (status_byte & 0x80) ? 1 : 0,
               (status_byte & 0x08) ? 1 : 0);
    } else {
      printf("ERREUR  %s - Erreur GetStatus: %s\r\n",
               label, AHT20_StatusToString(status));
    }
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
    printf("  AHT20 - Soft reset et récupération\r\n");
    printf("========================================\r\n\r\n");

  /* 2) Initialisation capteur */
    printf("INFO  Initialisation AHT20...\r\n");
  AHT20_Status status = AHT20_Init(&haht20, &hi2c3);   // Soft reset + lecture statut + cmd 0xBE si CAL=0
  if (status != AHT20_OK) {
      printf("ERREUR  Init échouée: %s\r\n", AHT20_StatusToString(status));
      printf("INFO  Tentative de récupération avec SoftReset...\r\n");

      if (PerformSoftResetAndReinit() != AHT20_OK) {
        printf("ERREUR  Récupération impossible: %s\r\n", AHT20_StatusToString(status));
          Error_Handler();
      }
      printf("OK  Récupération réussie\r\n\r\n");
  } else {
      printf("OK  Init OK\r\n\r\n");
  }

  /* 5) Paramètres runtime de la boucle principale */
  CheckAndPrintStatus("État initial");       // Lecture registre statut (bit Busy + bit CAL)
  printf("INFO  Robustesse : reset périodique toutes les %u ms\r\n\r\n", RESET_TEST_INTERVAL_MS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Reset périodique : intervalle contrôlé via RESET_TEST_INTERVAL_MS */
    if (HAL_GetTick() - last_reset_tick >= RESET_TEST_INTERVAL_MS) {
      printf("\r\n========================================\r\n");
      printf("  Test reset périodique #%lu\r\n", reset_count + 1);
      printf("========================================\r\n");
        CheckAndPrintStatus("Avant reset");

        if (PerformSoftResetAndReinit() == AHT20_OK) {
            CheckAndPrintStatus("Après reset");
        printf("========================================\r\n\r\n");
        } else {
        printf("ERREUR  Reset échoué: %s, poursuite du programme\r\n\r\n", AHT20_StatusToString(status));
        }
    }

    AHT20_Data sensor_data;     // Mesure bloquante ~80ms avec vérification CRC
    status = AHT20_ReadMeasurements(&haht20, &sensor_data);

    if (status == AHT20_OK) {
      printf("Température : %.1f°C | Humidité : %.1f%%RH | Reset: %lu | Uptime: %lus\r\n",
               sensor_data.temperature,
               sensor_data.humidity,
               reset_count,
               HAL_GetTick() / 1000);
    } else {
      printf("ERREUR  Erreur: %s\r\n", AHT20_StatusToString(status));

        // Si erreur de calibration, tenter récupération immédiate
        if (status == AHT20_ERR_CALIBRATION) {
        printf("INFO  Erreur calibration : tentative récupération immédiate...\r\n");

            if (PerformSoftResetAndReinit() == AHT20_OK) {
          printf("OK  Capteur récupéré\r\n\r\n");
                continue;  // Reprendre mesure immédiatement
            } else {
          printf("ERREUR  Récupération impossible: %s\r\n", AHT20_StatusToString(status));
                Error_Handler();
            }
        }
    }

    HAL_Delay(MEASUREMENT_INTERVAL_MS);
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
