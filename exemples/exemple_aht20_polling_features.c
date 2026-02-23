/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : exemple_aht20_polling_features.c
  * @brief          : Couverture API publique AHT20 — callbacks, reset async, GetData
  ******************************************************************************
  *
  * Couvre les fonctions non présentes dans les exemples polling/async_it :
  *   - AHT20_Async_Reset()          — reset contexte async (préserve callbacks)
  *   - AHT20_Async_SetCallbacks()   — callbacks data-ready/erreur (main loop)
  *   - AHT20_Async_SetIrqCallbacks()— callbacks IRQ-safe (signal flag seulement)
  *   - AHT20_Async_GetData()        — récupération donnée si flag levé
  *   - AHT20_Async_ClearFlags()     — acquittement manuel des drapeaux
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

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME            "exemple_aht20_polling_features"  ///< Identifiant log série
#define MEASURE_INTERVAL_MS  2000u                            ///< Intervalle mesure async (ms)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static AHT20_Handle  haht20;                         ///< Handle principal AHT20
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

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  /* 1) Bannière exemple */
  printf("\r\n========================================\r\n");
  printf("  Fichier: " LOG_NAME "\r\n");
  printf("  AHT20 - Couverture API async (callbacks, GetData, ClearFlags)\r\n");
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
    printf("ERREUR  Première mesure IT: %s\r\n", AHT20_StatusToString(st));
    printf("   Action : .ioc -> Connectivity -> I2C3 -> Configuration -> NVIC Settings\r\n");
    printf("            cocher Global interrupt activée\r\n");
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

    /* Avance la machine d'état async (appelle OnDataReady / OnError si besoin) */
    AHT20_Data sample;
    AHT20_TickResult tick = AHT20_Async_Tick(&aht20_ctx, now, &sample);

    if (tick == AHT20_TICK_DATA_READY) {
      printf("OK  Tick T=%.2f C | H=%.2f %%\r\n",
             sample.temperature, sample.humidity);

      /* AHT20_Async_GetData : pull explicite après TICK_DATA_READY si flag encore levé */
      if (AHT20_Async_DataReadyFlag(&aht20_ctx)) {
        AHT20_Data out;
        st = AHT20_Async_GetData(&aht20_ctx, &out);     // Consomme flag — autorise mesure suivante
        if (st == AHT20_OK) {
          printf("   GetData : T=%.2f C | H=%.2f %%\r\n",
                 out.temperature, out.humidity);
        }
      }

      /* AHT20_Async_ClearFlags : acquittement manuel des drapeaux data_ready et error */
      AHT20_Async_ClearFlags(&aht20_ctx);               // Optionnel si GetData déjà appelé
      last_trigger = now;
    }
    else if (tick == AHT20_TICK_ERROR) {
      printf("ERREUR  Async: %s\r\n", AHT20_StatusToString(haht20.last_error));  // last_error = champ public
      AHT20_Async_ClearFlags(&aht20_ctx);
    }

    /* Nouveau déclenchement périodique */
    if (AHT20_Async_IsIdle(&aht20_ctx) &&
        (now - last_trigger >= MEASURE_INTERVAL_MS))
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

void SystemClock_Config(void) { /* généré par CubeMX */ }
static void MX_GPIO_Init(void) { /* généré par CubeMX */ }
static void MX_USART2_UART_Init(void) { /* généré par CubeMX — 115200 bauds */ }
static void MX_I2C3_Init(void) { /* généré par CubeMX — Standard mode, NVIC activé */ }
void Error_Handler(void) { while (1) {} }
