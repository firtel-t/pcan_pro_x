#if defined(STM32G431xx)
/* ================================================================
 * STM32G431 (MKS CANable V2.0) - pcan_pro_x port
 * System clock: 160MHz from HSI via PLL
 * USB clock: 48MHz from HSI48 + CRS
 * FDCAN clock: 80MHz from PLLQ
 * ================================================================ */
#include "stm32g4xx_hal.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_led.h"
#include "pcanpro_protocol.h"
#include "usb_device.h"

void Error_Handler(void)
{
  HAL_Delay( 250 );
  HAL_NVIC_SystemReset();
  for(;;);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

static void pcan_io_config(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
}

void pcan_clock_config( void )
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* Configure the main internal regulator output voltage */
  HAL_PWREx_ControlVoltageScaling( PWR_REGULATOR_VOLTAGE_SCALE1_BOOST );

  /* Enable HSI and HSI48 oscillators, configure PLL from HSI */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;   /* 16MHz / 4 = 4MHz */
  RCC_OscInitStruct.PLL.PLLN = 80;               /* 4MHz * 80 = 320MHz VCO */
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;   /* 160MHz */
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;   /* 320/4 = 80MHz for FDCAN */
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;   /* 320/2 = 160MHz SYSCLK */

  if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure bus clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_4 ) != HAL_OK )
  {
    Error_Handler();
  }

  /* USB clock from HSI48 */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
  {
    Error_Handler();
  }

  /* Enable CRS for HSI48 calibration from USB SOF */
  __HAL_RCC_CRS_CLK_ENABLE();

  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE( 48000000, 1000 );
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig( &RCC_CRSInitStruct );

  /* FDCAN clock from PLLQ (80MHz) */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;

  if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
  {
    Error_Handler();
  }
}

int main(void)
{
  HAL_Init();

  pcan_clock_config();
  pcan_io_config();
  pcan_timestamp_init();
  
  pcan_led_init();
  pcan_led_set_mode( LED_CH0_RX, LED_MODE_ON, 0 );
  pcan_led_set_mode( LED_CH0_TX, LED_MODE_ON, 0 );
  pcan_protocol_init();
  pcan_usb_device_init();
  
  for(;;)
  {
    pcan_usb_device_poll();
    pcan_can_poll();
    pcan_protocol_poll();
    pcan_led_poll();
  }
}

#else /* STM32F4xx */
#if defined(STM32G431xx)
#include <stm32g4xx_hal.h>
#else
#include <stm32f4xx_hal.h>
#endif
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_led.h"
#include "pcanpro_protocol.h"
#include "usb_device.h"

void Error_Handler(void)
{
  /* reboot */
  HAL_Delay( 250 );
  HAL_NVIC_SystemReset();
  for(;;);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

static void pcan_io_config(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
#if !defined(STM32G431xx)
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
}

#if defined(STM32G431xx)
void pcan_clock_config( void )
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /* HSI + HSI48 + PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;  /* 80MHz FDCAN */
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;  /* 160MHz SYSCLK */
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { Error_Handler(); }

  /* USB from HSI48 */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }

  /* CRS for HSI48 */
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef crs = {0};
  crs.Prescaler = RCC_CRS_SYNC_DIV1;
  crs.Source = RCC_CRS_SYNC_SOURCE_USB;
  crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  crs.ErrorLimitValue = 34;
  crs.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&crs);

  /* FDCAN from PLLQ (80MHz) */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}
#else
void pcan_clock_config( void )
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#if (HSE_VALUE == 8000000 )
  RCC_OscInitStruct.PLL.PLLM = 8;
#elif (HSE_VALUE == 25000000 )
  RCC_OscInitStruct.PLL.PLLM = 25;
#else
  #error Invalid HSE_VALUE
#endif
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if( HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK )
  {
    Error_Handler();
  }
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if( HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3 ) != HAL_OK )
  {
    Error_Handler();
  }
}
#endif /* STM32G431xx */

int main(void)
{
  HAL_Init();

  pcan_clock_config();
  pcan_io_config();
  pcan_timestamp_init();
  
  pcan_led_init();
  pcan_led_set_mode( LED_STAT, LED_MODE_BLINK_FAST, 0xFFFFFFFF );
  pcan_protocol_init();
  pcan_usb_device_init();
  
  for(;;)
  {
    pcan_usb_device_poll();
    pcan_can_poll();
    pcan_protocol_poll();
    pcan_led_poll();
  }
}

#endif /* STM32G431xx */
