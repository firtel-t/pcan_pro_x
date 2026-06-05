#include <assert.h>
#include "io_macro.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_variant.h"

#if defined(STM32G431xx)
/* ================================================================
 * FDCAN implementation for STM32G431
 * ================================================================ */

#define CAN_TX_FIFO_SIZE (256)

static FDCAN_HandleTypeDef g_hfdcan;

static struct t_can_dev
{
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;
  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  struct t_can_msg tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;

  int (*rx_isr)( uint8_t, struct t_can_msg* );
  int (*tx_isr)( uint8_t, struct t_can_msg* );
  void (*err_handler)( int bus, uint32_t esr );
}
can_dev = { 0 };

uint32_t pcan_can_msg_time( const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt )
{
  const uint32_t data_bits = pmsg->size<<3;
  const uint32_t control_bits = ( pmsg->flags & MSG_FLAG_EXT ) ? 67:47;
 
  if( pmsg->flags & MSG_FLAG_BRS )
    return (control_bits*nt) + (data_bits*dt);
  else
    return (control_bits+data_bits)*nt;
}

static uint32_t canfd_size_to_fdcan_dlc(uint8_t size)
{
  if(size <= 8)
  {
    switch(size)
    {
      case 0: return FDCAN_DLC_BYTES_0;
      case 1: return FDCAN_DLC_BYTES_1;
      case 2: return FDCAN_DLC_BYTES_2;
      case 3: return FDCAN_DLC_BYTES_3;
      case 4: return FDCAN_DLC_BYTES_4;
      case 5: return FDCAN_DLC_BYTES_5;
      case 6: return FDCAN_DLC_BYTES_6;
      case 7: return FDCAN_DLC_BYTES_7;
      default: return FDCAN_DLC_BYTES_8;
    }
  }
  if(size <= 12) return FDCAN_DLC_BYTES_12;
  if(size <= 16) return FDCAN_DLC_BYTES_16;
  if(size <= 20) return FDCAN_DLC_BYTES_20;
  if(size <= 24) return FDCAN_DLC_BYTES_24;
  if(size <= 32) return FDCAN_DLC_BYTES_32;
  if(size <= 48) return FDCAN_DLC_BYTES_48;
  return FDCAN_DLC_BYTES_64;
}

static uint8_t fdcan_dlc_to_size(uint32_t dlc)
{
  switch(dlc)
  {
    case FDCAN_DLC_BYTES_0:  return 0;
    case FDCAN_DLC_BYTES_1:  return 1;
    case FDCAN_DLC_BYTES_2:  return 2;
    case FDCAN_DLC_BYTES_3:  return 3;
    case FDCAN_DLC_BYTES_4:  return 4;
    case FDCAN_DLC_BYTES_5:  return 5;
    case FDCAN_DLC_BYTES_6:  return 6;
    case FDCAN_DLC_BYTES_7:  return 7;
    case FDCAN_DLC_BYTES_8:  return 8;
    case FDCAN_DLC_BYTES_12: return 12;
    case FDCAN_DLC_BYTES_16: return 16;
    case FDCAN_DLC_BYTES_20: return 20;
    case FDCAN_DLC_BYTES_24: return 24;
    case FDCAN_DLC_BYTES_32: return 32;
    case FDCAN_DLC_BYTES_48: return 48;
    case FDCAN_DLC_BYTES_64: return 64;
    default: return 0;
  }
}

int pcan_can_set_filter_mask( int bus, int num, int format, uint32_t id, uint32_t mask )
{
  (void)bus; (void)num; (void)format; (void)id; (void)mask;
  /* FDCAN global filter accepts all - configured in init */
  return 0;
}

int pcan_can_filter_init_stdid_list( int bus, const uint16_t *id_list, int id_len )
{
  (void)bus; (void)id_list; (void)id_len;
  return 0;
}

int pcan_can_init_ex( int bus, uint32_t bitrate )
{
  (void)bus;
  (void)bitrate;

  __HAL_RCC_FDCAN_CLK_ENABLE();

  PORT_ENABLE_CLOCK( PIN_PORT( CAN1_RX ), PIN_PORT( CAN1_TX ) );
  PIN_INIT( CAN1_RX );
  PIN_INIT( CAN1_TX );

  g_hfdcan.Instance = FDCAN1;
  g_hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  g_hfdcan.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  g_hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
  g_hfdcan.Init.AutoRetransmission = ENABLE;
  g_hfdcan.Init.TransmitPause = DISABLE;
  g_hfdcan.Init.ProtocolException = DISABLE;
  /* Default 500kbps nominal @ 80MHz FDCAN clock */
  g_hfdcan.Init.NominalPrescaler = 10;
  g_hfdcan.Init.NominalSyncJumpWidth = 1;
  g_hfdcan.Init.NominalTimeSeg1 = 13;
  g_hfdcan.Init.NominalTimeSeg2 = 2;
  /* Default 2Mbps data @ 80MHz */
  g_hfdcan.Init.DataPrescaler = 4;
  g_hfdcan.Init.DataSyncJumpWidth = 1;
  g_hfdcan.Init.DataTimeSeg1 = 15;
  g_hfdcan.Init.DataTimeSeg2 = 4;
  g_hfdcan.Init.StdFiltersNbr = 1;
  g_hfdcan.Init.ExtFiltersNbr = 1;
  g_hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if( HAL_FDCAN_Init(&g_hfdcan) != HAL_OK )
    return -1;

  /* Accept all frames */
  HAL_FDCAN_ConfigGlobalFilter(&g_hfdcan,
    FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
    FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  FDCAN_FilterTypeDef filter = { 0 };
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x000;
  HAL_FDCAN_ConfigFilter(&g_hfdcan, &filter);

  filter.IdType = FDCAN_EXTENDED_ID;
  filter.FilterIndex = 0;
  filter.FilterID1 = 0x00000000;
  filter.FilterID2 = 0x00000000;
  HAL_FDCAN_ConfigFilter(&g_hfdcan, &filter);

  HAL_FDCAN_ActivateNotification(&g_hfdcan,
    FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL |
    FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY |
    FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_WARNING |
    FDCAN_IT_ERROR_PASSIVE |
    FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR,
    0xFFFFFFFF);

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

  return 0;
}

void pcan_can_set_silent( int bus, uint8_t silent_mode )
{
  (void)bus;
  HAL_FDCAN_Stop(&g_hfdcan);
  g_hfdcan.Init.Mode = silent_mode ? FDCAN_MODE_BUS_MONITORING : FDCAN_MODE_NORMAL;
  HAL_FDCAN_Init(&g_hfdcan);
}

void pcan_can_set_iso_mode( int bus, uint8_t iso_mode )
{
  (void)bus;
  HAL_FDCAN_Stop(&g_hfdcan);
  if(iso_mode)
    HAL_FDCAN_EnableISOMode(&g_hfdcan);
  else
    HAL_FDCAN_DisableISOMode(&g_hfdcan);
}

void pcan_can_set_loopback( int bus, uint8_t loopback )
{
  (void)bus;
  HAL_FDCAN_Stop(&g_hfdcan);
  g_hfdcan.Init.Mode = loopback ? FDCAN_MODE_INTERNAL_LOOPBACK : FDCAN_MODE_NORMAL;
  HAL_FDCAN_Init(&g_hfdcan);
}

void pcan_can_set_bus_active( int bus, uint16_t mode )
{
  (void)bus;
  if(mode)
    HAL_FDCAN_Start(&g_hfdcan);
  else
    HAL_FDCAN_Stop(&g_hfdcan);
}

void pcan_can_set_bitrate( int bus, uint32_t bitrate, int is_data_bitrate )
{
  (void)bus;

  /* Use precalculated values for common bitrates @ 80MHz FDCAN clock */
  HAL_FDCAN_Stop(&g_hfdcan);

  if(is_data_bitrate)
  {
    switch(bitrate)
    {
      case 8000000u:
        g_hfdcan.Init.DataPrescaler = 1;
        g_hfdcan.Init.DataTimeSeg1 = 7;
        g_hfdcan.Init.DataTimeSeg2 = 2;
        g_hfdcan.Init.DataSyncJumpWidth = 2;
        break;
      case 5000000u:
        g_hfdcan.Init.DataPrescaler = 1;
        g_hfdcan.Init.DataTimeSeg1 = 13;
        g_hfdcan.Init.DataTimeSeg2 = 2;
        g_hfdcan.Init.DataSyncJumpWidth = 2;
        break;
      case 4000000u:
        g_hfdcan.Init.DataPrescaler = 2;
        g_hfdcan.Init.DataTimeSeg1 = 7;
        g_hfdcan.Init.DataTimeSeg2 = 2;
        g_hfdcan.Init.DataSyncJumpWidth = 2;
        break;
      default:
      case 2000000u:
        g_hfdcan.Init.DataPrescaler = 4;
        g_hfdcan.Init.DataTimeSeg1 = 15;
        g_hfdcan.Init.DataTimeSeg2 = 4;
        g_hfdcan.Init.DataSyncJumpWidth = 1;
        break;
      case 1000000u:
        g_hfdcan.Init.DataPrescaler = 10;
        g_hfdcan.Init.DataTimeSeg1 = 6;
        g_hfdcan.Init.DataTimeSeg2 = 1;
        g_hfdcan.Init.DataSyncJumpWidth = 1;
        break;
    }
  }
  else
  {
    switch(bitrate)
    {
      case 1000000u:
        g_hfdcan.Init.NominalPrescaler = 5;
        g_hfdcan.Init.NominalTimeSeg1 = 13;
        g_hfdcan.Init.NominalTimeSeg2 = 2;
        g_hfdcan.Init.NominalSyncJumpWidth = 1;
        break;
      default:
      case 500000u:
        g_hfdcan.Init.NominalPrescaler = 10;
        g_hfdcan.Init.NominalTimeSeg1 = 13;
        g_hfdcan.Init.NominalTimeSeg2 = 2;
        g_hfdcan.Init.NominalSyncJumpWidth = 1;
        break;
      case 250000u:
        g_hfdcan.Init.NominalPrescaler = 20;
        g_hfdcan.Init.NominalTimeSeg1 = 13;
        g_hfdcan.Init.NominalTimeSeg2 = 2;
        g_hfdcan.Init.NominalSyncJumpWidth = 1;
        break;
      case 125000u:
        g_hfdcan.Init.NominalPrescaler = 40;
        g_hfdcan.Init.NominalTimeSeg1 = 13;
        g_hfdcan.Init.NominalTimeSeg2 = 2;
        g_hfdcan.Init.NominalSyncJumpWidth = 1;
        break;
      case 100000u:
        g_hfdcan.Init.NominalPrescaler = 50;
        g_hfdcan.Init.NominalTimeSeg1 = 13;
        g_hfdcan.Init.NominalTimeSeg2 = 2;
        g_hfdcan.Init.NominalSyncJumpWidth = 1;
        break;
      case 50000u:
        g_hfdcan.Init.NominalPrescaler = 100;
        g_hfdcan.Init.NominalTimeSeg1 = 13;
        g_hfdcan.Init.NominalTimeSeg2 = 2;
        g_hfdcan.Init.NominalSyncJumpWidth = 1;
        break;
      case 20000u:
        g_hfdcan.Init.NominalPrescaler = 250;
        g_hfdcan.Init.NominalTimeSeg1 = 13;
        g_hfdcan.Init.NominalTimeSeg2 = 2;
        g_hfdcan.Init.NominalSyncJumpWidth = 1;
        break;
      case 10000u:
        g_hfdcan.Init.NominalPrescaler = 500;
        g_hfdcan.Init.NominalTimeSeg1 = 13;
        g_hfdcan.Init.NominalTimeSeg2 = 2;
        g_hfdcan.Init.NominalSyncJumpWidth = 1;
        break;
    }
  }

  HAL_FDCAN_Init(&g_hfdcan);
}

void pcan_can_set_bitrate_ex( int bus, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw )
{
  (void)bus;

  HAL_FDCAN_Stop(&g_hfdcan);

  g_hfdcan.Init.NominalPrescaler = brp;
  g_hfdcan.Init.NominalTimeSeg1 = tseg1;
  g_hfdcan.Init.NominalTimeSeg2 = tseg2;
  g_hfdcan.Init.NominalSyncJumpWidth = sjw;

  HAL_FDCAN_Init(&g_hfdcan);
}

static int _fdcan_try_send(struct t_can_msg *p_msg)
{
  FDCAN_TxHeaderTypeDef tx_hdr = { 0 };

  if(p_msg->flags & MSG_FLAG_EXT)
  {
    tx_hdr.Identifier = p_msg->id & 0x1FFFFFFF;
    tx_hdr.IdType = FDCAN_EXTENDED_ID;
  }
  else
  {
    tx_hdr.Identifier = p_msg->id & 0x7FF;
    tx_hdr.IdType = FDCAN_STANDARD_ID;
  }

  tx_hdr.TxFrameType = (p_msg->flags & MSG_FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
  tx_hdr.ErrorStateIndicator = (p_msg->flags & MSG_FLAG_ESI) ? FDCAN_ESI_PASSIVE : FDCAN_ESI_ACTIVE;
  tx_hdr.BitRateSwitch = (p_msg->flags & MSG_FLAG_BRS) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  tx_hdr.FDFormat = (p_msg->flags & MSG_FLAG_FD) ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_hdr.MessageMarker = 0;
  tx_hdr.DataLength = canfd_size_to_fdcan_dlc(p_msg->size);

  if(HAL_FDCAN_AddMessageToTxFifoQ(&g_hfdcan, &tx_hdr, p_msg->data) != HAL_OK)
    return -1;

  return 0;
}

static void pcan_can_flush_tx( void )
{
  struct t_can_msg *p_msg;

  if( can_dev.tx_head == can_dev.tx_tail )
    return;

  p_msg = &can_dev.tx_fifo[can_dev.tx_tail];
  if( _fdcan_try_send( p_msg ) < 0 )
    return;

  if( can_dev.tx_isr )
  {
    (void)can_dev.tx_isr( CAN_BUS_1, p_msg );
  }

  can_dev.tx_tail = (can_dev.tx_tail+1)&(CAN_TX_FIFO_SIZE-1);
  ++can_dev.tx_msgs;
}

int pcan_can_write( int bus, struct t_can_msg *p_msg )
{
  (void)bus;

  if( !p_msg )
    return 0;

  uint32_t tx_head_next = (can_dev.tx_head+1)&(CAN_TX_FIFO_SIZE-1);
  if( tx_head_next == can_dev.tx_tail )
  {
    ++can_dev.tx_ovfs;
    return -1;
  }

  can_dev.tx_fifo[can_dev.tx_head] = *p_msg;
  can_dev.tx_head = tx_head_next;

  return 0;
}

void pcan_can_install_rx_callback( int bus, int (*cb)( uint8_t, struct t_can_msg* ) )
{
  (void)bus;
  can_dev.rx_isr = cb;
}

void pcan_can_install_tx_callback( int bus, int (*cb)( uint8_t, struct t_can_msg* ) )
{
  (void)bus;
  can_dev.tx_isr = cb;
}

void pcan_can_install_err_callback( int bus, void (*cb)( int, uint32_t ) )
{
  (void)bus;
  can_dev.err_handler = cb;
}

int pcan_can_stats( int bus, struct t_can_stats *p_stats )
{
  (void)bus;
  p_stats->tx_msgs = can_dev.tx_msgs;
  p_stats->tx_errs = can_dev.tx_errs;
  p_stats->rx_msgs = can_dev.rx_msgs;
  p_stats->rx_errs = can_dev.rx_errs;
  p_stats->rx_ovfs = can_dev.rx_ovfs;
  return sizeof( struct t_can_stats );
}

static void pcan_fdcan_rx_frame(void)
{
  FDCAN_RxHeaderTypeDef hdr = { 0 };
  struct t_can_msg msg = { 0 };

  if(HAL_FDCAN_GetRxMessage(&g_hfdcan, FDCAN_RX_FIFO0, &hdr, msg.data) != HAL_OK)
    return;

  if(hdr.IdType == FDCAN_STANDARD_ID)
    msg.id = hdr.Identifier;
  else
  {
    msg.id = hdr.Identifier;
    msg.flags |= MSG_FLAG_EXT;
  }

  if(hdr.RxFrameType == FDCAN_REMOTE_FRAME)
    msg.flags |= MSG_FLAG_RTR;

  if(hdr.FDFormat == FDCAN_FD_CAN)
  {
    msg.flags |= MSG_FLAG_FD;
    if(hdr.BitRateSwitch == FDCAN_BRS_ON)
      msg.flags |= MSG_FLAG_BRS;
    if(hdr.ErrorStateIndicator == FDCAN_ESI_PASSIVE)
      msg.flags |= MSG_FLAG_ESI;
  }

  msg.size = fdcan_dlc_to_size(hdr.DataLength);
  msg.timestamp = pcan_timestamp_us();

  if( can_dev.rx_isr )
  {
    if( can_dev.rx_isr( CAN_BUS_1, &msg ) < 0 )
    {
      ++can_dev.rx_ovfs;
      return;
    }
  }
  ++can_dev.rx_msgs;
}

void pcan_can_poll( void )
{
  /* Poll RX FIFO */
  while(HAL_FDCAN_GetRxFifoFillLevel(&g_hfdcan, FDCAN_RX_FIFO0) > 0)
    pcan_fdcan_rx_frame();

  /* Flush TX */
  pcan_can_flush_tx();
}

void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&g_hfdcan); }
void FDCAN1_IT1_IRQHandler(void) { HAL_FDCAN_IRQHandler(&g_hfdcan); }

/**
 * Bus-Off recovery callback.
 * FDCAN (unlike bxCAN) does not have automatic bus-off recovery.
 * When bus-off occurs the hardware sets CCCR.INIT=1 and stays there.
 * We must manually re-start the peripheral to recover.
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if( ErrorStatusITs & FDCAN_IR_BO )
  {
    /* Bus-Off detected: stop, re-init, and restart FDCAN */
    HAL_FDCAN_Stop(hfdcan);
    HAL_FDCAN_Init(hfdcan);

    /* Re-apply global accept-all filters */
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,
      FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
      FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    FDCAN_FilterTypeDef filter = { 0 };
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0;
    filter.FilterID1 = 0x00000000;
    filter.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(hfdcan, &filter);

    /* Re-activate notifications */
    HAL_FDCAN_ActivateNotification(hfdcan,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL |
      FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY |
      FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_WARNING |
      FDCAN_IT_ERROR_PASSIVE |
      FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR,
      0xFFFFFFFF);

    /* Restart the bus */
    HAL_FDCAN_Start(hfdcan);

    /* Notify upper layer if error handler is registered */
    if( can_dev.err_handler )
    {
      can_dev.err_handler( CAN_BUS_1, FDCAN_IR_BO );
    }
  }
}

#else /* STM32F4xx bxCAN */

static CAN_HandleTypeDef hcan[CAN_BUS_TOTAL] = 
{ 
  [CAN_BUS_1].Instance = CAN1,
  [CAN_BUS_2].Instance = CAN2
};

#define CAN_TX_FIFO_SIZE (256)
static struct t_can_dev
{
  void *dev;
  uint32_t tx_msgs;
  uint32_t tx_errs;
  uint32_t tx_ovfs;

  uint32_t rx_msgs;
  uint32_t rx_errs;
  uint32_t rx_ovfs;

  struct t_can_msg tx_fifo[CAN_TX_FIFO_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;
  uint32_t esr_reg;
  int (*rx_isr)( uint8_t, struct  t_can_msg* );
  int (*tx_isr)( uint8_t, struct  t_can_msg* );
  void (*err_handler)( int bus, uint32_t esr );
}
can_dev_array[CAN_BUS_TOTAL] = 
{
#ifdef CAN1_RX
  [CAN_BUS_1] = { .dev = &hcan[CAN_BUS_1] },
#else
  [CAN_BUS_1] = { .dev = 0 },
#endif
#ifdef CAN2_RX
  [CAN_BUS_2] = { .dev = &hcan[CAN_BUS_2] },
#else
  [CAN_BUS_2] = { .dev = 0 },
#endif
};

#define INTERNAL_CAN_IT_FLAGS          (  CAN_IT_TX_MAILBOX_EMPTY |\
                                          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING |\
                                          CAN_IT_ERROR_WARNING |\
                                          CAN_IT_ERROR_PASSIVE |\
                                          CAN_IT_LAST_ERROR_CODE |\
                                          CAN_IT_ERROR )

#define CAN2_FILTER_START (14u)

#define CAN_WITHOUT_ISR 1

uint32_t pcan_can_msg_time( const struct t_can_msg *pmsg, uint32_t nt, uint32_t dt )
{
  const uint32_t data_bits = pmsg->size<<3;
  const uint32_t control_bits = ( pmsg->flags & MSG_FLAG_EXT ) ? 67:47;
 
  if( pmsg->flags & MSG_FLAG_BRS )
    return (control_bits*nt) + (data_bits*dt);
  else
    return (control_bits+data_bits)*nt;
}

int pcan_can_set_filter_mask( int bus, int num, int format, uint32_t id, uint32_t mask )
{
  CAN_FilterTypeDef filter = { 0 };
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return 0;
  
  if( num >= CAN_INT_FILTER_MAX )
    return -1;
  
  /* CAN1 & CAN2 filter shared 28 filters, we use 14 for each one */
  if( p_can->Instance == CAN2 )
  {
    num += CAN2_FILTER_START;
  }
  
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  if( format == MSG_FLAG_EXT )
  {
    id &= 0x1FFFFFFF;
    
    /* EXTID[28:13] */
    filter.FilterIdHigh = id >> 13;
    /* EXTID[12:0] + IDE */
    filter.FilterIdLow = ((id << 3)&0xFFFF) | CAN_ID_EXT;
    filter.FilterMaskIdHigh = mask >> 13;
    filter.FilterMaskIdLow = ((mask << 3)&0xFFFF) | CAN_ID_EXT;
  }
  else
  {
    id &= 0x7FF;
    filter.FilterIdHigh = id << 5;
    filter.FilterIdLow =  0x0;
    filter.FilterMaskIdHigh = mask << 5;
    filter.FilterMaskIdLow = 0x0;
  }
  
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.FilterBank = num;
  filter.SlaveStartFilterBank = CAN2_FILTER_START;
  
  if( HAL_CAN_ConfigFilter( p_can, &filter ) != HAL_OK )
    return -1;
  
  return 0;
}

int pcan_can_filter_init_stdid_list( int bus, const uint16_t *id_list, int id_len  )
{
  CAN_FilterTypeDef filter = { 0 };
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return 0;
 
  int i, offset;

  offset = ( p_can->Instance == CAN2 ) ? CAN2_FILTER_START: 0;  
  
  /* STDID[10:3] | STDID[2:0] RTR IDE EXID[17:15] */
  /* IDE = 0, RTR = 0, EXID[17:15] = { 0 } */
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  
  i = 0;
  while( i < id_len )
  {
    filter.FilterBank = ((i)>>2)+offset;
    filter.SlaveStartFilterBank = CAN2_FILTER_START;
    
    filter.FilterMaskIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterIdHigh = 0;
    
    switch( (id_len-i) )
    {
    /* fallthrough */
    default:
      filter.FilterMaskIdLow = id_list[i++]<<5;
    case 3:
      filter.FilterMaskIdHigh = id_list[i++]<<5;
    case 2:
      filter.FilterIdLow = id_list[i++]<<5;
    case 1:
      filter.FilterIdHigh = id_list[i++]<<5;
    }
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    
    if( HAL_CAN_ConfigFilter( p_can, &filter ) != HAL_OK )
      return -1;
  }
  
  return id_len;
}

static int _can_send( CAN_HandleTypeDef *p_can, struct t_can_msg *p_msg )
{
  CAN_TxHeaderTypeDef msg = { .TransmitGlobalTime = DISABLE };
  uint32_t txMailbox = 0;

  if( HAL_CAN_IsTxMessagePending( p_can, CAN_TX_MAILBOX0 ) )
    return -1;

  if( p_msg->flags & MSG_FLAG_EXT )
  {
    msg.ExtId = p_msg->id & 0x1FFFFFFF;
    msg.IDE = CAN_ID_EXT;
  }
  else
  {
    msg.StdId = p_msg->id & 0x7FF;
    msg.IDE = CAN_ID_STD;
  }
  
  msg.DLC = p_msg->size;
  msg.RTR = (p_msg->flags & MSG_FLAG_RTR)?CAN_RTR_REMOTE:CAN_RTR_DATA;
  
  if( HAL_CAN_AddTxMessage( p_can, &msg, (void*)p_msg->data, &txMailbox ) != HAL_OK )
    return -1;

  return txMailbox;
}

static void pcan_can_flush_tx( int bus )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  struct t_can_msg *p_msg;

  /* empty fifo */
  if( p_dev->tx_head == p_dev->tx_tail )
    return;

  if( !p_dev->dev )
    return;
  
  p_msg = &p_dev->tx_fifo[p_dev->tx_tail];
  if( _can_send( p_dev->dev, p_msg ) < 0 )
    return;

  if( p_dev->tx_isr )
  {
    (void)p_dev->tx_isr( bus, p_msg );
  }

  /* update fifo index */
  p_dev->tx_tail = (p_dev->tx_tail+1)&(CAN_TX_FIFO_SIZE-1);
}

int pcan_can_write( int bus, struct t_can_msg *p_msg )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];

  if( !p_dev )
    return 0;

  if( !p_msg )
    return 0;

  uint32_t  tx_head_next = (p_dev->tx_head+1)&(CAN_TX_FIFO_SIZE-1);
  /* overflow ? just skip it */
  if( tx_head_next == p_dev->tx_tail )
  {
    ++p_dev->tx_ovfs;
    return -1;
  }

  p_dev->tx_fifo[p_dev->tx_head] = *p_msg;
  p_dev->tx_head = tx_head_next;

  return 0;
}

void pcan_can_install_rx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->rx_isr = cb;
}

void pcan_can_install_tx_callback( int bus, int (*cb)( uint8_t, struct  t_can_msg* ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->tx_isr = cb;
}

void pcan_can_install_err_callback( int bus, void (*cb)( int , uint32_t ) )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  p_dev->err_handler = cb;
}

/* all internal CANs on APB1 */
static int _get_precalculated_bitrate( uint32_t bitrate, uint32_t *brp, uint32_t *tseg1, uint32_t *tseg2, uint32_t *sjw )
{
  *sjw = CAN_SJW_1TQ;

  switch( bitrate )
  {
    case 1000000u:
      *brp = 3;
      *tseg1 = CAN_BS1_6TQ;
      *tseg2 = CAN_BS2_1TQ;
    break;
    case 800000u:
      *brp = 2;
      *tseg1 = CAN_BS1_12TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    default:
    case 500000u:
      *brp = 3;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 250000u:
      *brp = 6;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 125000u:
      *brp = 12;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 100000u:
      *brp = 15;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 50000u:
      *brp = 30;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 20000u:
      *brp = 75;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
    case 10000u:
      *brp = 150;
      *tseg1 = CAN_BS1_13TQ;
      *tseg2 = CAN_BS2_2TQ;
    break;
  }

  return 0;
}

int pcan_can_init_ex( int bus, uint32_t bitrate )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;
  uint32_t brp;
  uint32_t tseg1, tseg2, sjw;

  if( !p_can )
    return 0;

  p_can->Init.Mode = CAN_MODE_NORMAL;//CAN_MODE_NORMAL;// CAN_MODE_LOOPBACK;
    
  p_can->Init.TimeTriggeredMode = DISABLE;
  p_can->Init.AutoBusOff = ENABLE;
  p_can->Init.AutoWakeUp = ENABLE;

  p_can->Init.AutoRetransmission = DISABLE;
  p_can->Init.ReceiveFifoLocked = DISABLE;
  p_can->Init.TransmitFifoPriority = ENABLE;
  
  /* APB1 bus ref clock = 24MHz, best sp is 87.5% */
  _get_precalculated_bitrate( bitrate, &brp, &tseg1, &tseg2, &sjw );

  p_can->Init.SyncJumpWidth = sjw;
  p_can->Init.Prescaler = brp;
  p_can->Init.TimeSeg1 = tseg1;
  p_can->Init.TimeSeg2 = tseg2;
  
  //(void)HAL_CAN_AbortTxRequest( p_can, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
  //(void)HAL_CAN_Stop( p_can );
  //(void)HAL_CAN_DeInit( p_can );
  
  if( HAL_CAN_Init( p_can ) != HAL_OK )
    return -1;
  
  if( HAL_CAN_ActivateNotification( p_can, INTERNAL_CAN_IT_FLAGS ) != HAL_OK )
    return -1;
  
  if( HAL_CAN_Start( p_can ) != HAL_OK )
    return -1;
  
  return 0;
}

void pcan_can_set_silent( int bus, uint8_t silent_mode )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;

  p_can->Init.Mode = silent_mode ? CAN_MODE_SILENT: CAN_MODE_NORMAL;
  if( HAL_CAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

/* bxCAN does not support FDCAN ISO mode switch */
void pcan_can_set_iso_mode( int bus, uint8_t iso_mode )
{
  (void)bus;
  (void)iso_mode;
}

void pcan_can_set_loopback( int bus, uint8_t loopback )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;
  
  p_can->Init.Mode = loopback ? CAN_MODE_LOOPBACK: CAN_MODE_NORMAL;
  if( HAL_CAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_set_bus_active( int bus, uint16_t mode )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;

  if( mode )
  {
    HAL_CAN_Start( p_can );
    HAL_CAN_AbortTxRequest( p_can, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
  }
  else
  {
    HAL_CAN_AbortTxRequest( p_can, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2 );
    HAL_CAN_Stop( p_can );
  }
}

/* set predefined best values */
void pcan_can_set_bitrate( int bus, uint32_t bitrate, int is_data_bitrate )
{
  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;
  uint32_t brp;
  uint32_t tseg1, tseg2, sjw;

  if( !p_can )
    return;

  _get_precalculated_bitrate( bitrate, &brp, &tseg1, &tseg2, &sjw );

  if( is_data_bitrate )
  {
    return;
  }
  else
  {
    p_can->Init.Prescaler = brp;
    p_can->Init.SyncJumpWidth = sjw;
    p_can->Init.TimeSeg1 = tseg1;
    p_can->Init.TimeSeg2 = tseg2;
  }
  
  if( HAL_CAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

void pcan_can_set_bitrate_ex( int bus, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw )
{
  static const uint32_t sjw_table[] = 
  { 
    CAN_SJW_1TQ, CAN_SJW_2TQ, CAN_SJW_3TQ, CAN_SJW_4TQ 
  };
  static const uint32_t tseg1_table[] = 
  { 
    CAN_BS1_1TQ, CAN_BS1_2TQ, CAN_BS1_3TQ, CAN_BS1_4TQ, 
    CAN_BS1_5TQ, CAN_BS1_6TQ, CAN_BS1_7TQ, CAN_BS1_8TQ, 
    CAN_BS1_9TQ, CAN_BS1_10TQ, CAN_BS1_11TQ, CAN_BS1_12TQ,
    CAN_BS1_13TQ, CAN_BS1_14TQ, CAN_BS1_15TQ, CAN_BS1_16TQ
  };
  static const uint32_t tseg2_table[] =
  { 
    CAN_BS2_1TQ, CAN_BS2_2TQ, CAN_BS2_3TQ, CAN_BS2_4TQ,
    CAN_BS2_5TQ, CAN_BS2_6TQ, CAN_BS2_7TQ, CAN_BS2_8TQ
  };

  if( sjw > 4 )
    sjw = 4;
  if( tseg1 > 16 )
    tseg1 = 16;
  if( tseg2 > 8 )
    tseg2 = 8;

  CAN_HandleTypeDef *p_can = can_dev_array[bus].dev;

  if( !p_can )
    return;

  p_can->Init.Prescaler = brp;

  p_can->Init.SyncJumpWidth = sjw_table[sjw - 1];
  p_can->Init.TimeSeg1 = tseg1_table[tseg1 - 1];
  p_can->Init.TimeSeg2 = tseg2_table[tseg2 - 1];

  if( HAL_CAN_Init( p_can ) != HAL_OK )
  {
    assert( 0 );
  }
}

static void pcan_can_tx_complete( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_msgs;
}

static void pcan_can_tx_err( int bus, int mail_box )
{
  ++can_dev_array[bus].tx_errs;
}

int pcan_can_stats( int bus, struct t_can_stats *p_stats )
{
  struct t_can_dev *p_dev = &can_dev_array[bus];
  
  p_stats->tx_msgs = p_dev->tx_msgs;
  p_stats->tx_errs = p_dev->tx_errs;
  p_stats->rx_msgs = p_dev->rx_msgs;
  p_stats->rx_errs = p_dev->rx_errs;
  p_stats->rx_ovfs = p_dev->rx_ovfs;

  return sizeof( struct t_can_stats );
}

void pcan_can_poll( void )
{
  static uint32_t err_last_check = 0;
  uint32_t ts_ms;
  
  ts_ms = pcan_timestamp_millis();
#if ( CAN_WITHOUT_ISR == 1 )
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
#endif
  
  pcan_can_flush_tx( CAN_BUS_1 );
  pcan_can_flush_tx( CAN_BUS_2 );

  if( (uint32_t)( err_last_check - ts_ms ) > 250 )
  {
    err_last_check = ts_ms;
    for( int i = 0; i < CAN_BUS_TOTAL; i++ )
    {
      if( !can_dev_array[i].err_handler )
        continue;
      CAN_HandleTypeDef *pcan = can_dev_array[i].dev;
      if( !pcan )
        continue;
      if( can_dev_array[i].esr_reg != pcan->Instance->ESR )
      {
        can_dev_array[i].esr_reg = pcan->Instance->ESR;
        can_dev_array[i].err_handler( i, can_dev_array[i].esr_reg );
      }
    }
  }
}

/* --------------- HAL PART ------------- */
static int _bus_from_int_dev( CAN_TypeDef *can )
{
  if( can == CAN1 )
    return CAN_BUS_1;
  else if( can == CAN2 )
    return CAN_BUS_2;
  /* abnormal! */
  return CAN_BUS_1;
}

static void pcan_can_isr_frame( CAN_HandleTypeDef *hcan, uint32_t fifo )
{
  CAN_RxHeaderTypeDef hdr;
  const int bus = _bus_from_int_dev( hcan->Instance );
  struct t_can_dev * const p_dev = &can_dev_array[bus];
  struct t_can_msg  msg = { 0 };
  
  if( HAL_CAN_GetRxMessage( hcan, fifo, &hdr, msg.data ) != HAL_OK )
    return;

  /* oversize frame ? */
  if( hdr.DLC > CAN_PAYLOAD_MAX_SIZE )
    return;

  if( hdr.IDE == CAN_ID_STD )
  {
    msg.id = hdr.StdId;
  }
  else
  {
    msg.id = hdr.ExtId;
    msg.flags |= MSG_FLAG_EXT;
  }

  if( hdr.RTR != CAN_RTR_DATA )
  {
    msg.flags |= MSG_FLAG_RTR;
  }

  msg.size = hdr.DLC;
  msg.timestamp = pcan_timestamp_us();
  
  if( p_dev->rx_isr )
  {
    if( p_dev->rx_isr( bus, &msg ) < 0 )
    {
      ++p_dev->rx_ovfs;
      return;
    }
  }
  ++p_dev->rx_msgs;
}

/* WARN: CAN1 & CAN2 use filter registers of CAN1 */
void HAL_CAN_MspInit( CAN_HandleTypeDef *hcan )
{
  if( hcan->Instance == CAN1 )
  {
    if( !__HAL_RCC_CAN1_IS_CLK_ENABLED() )
    {
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

#ifdef CAN1_RX
    PORT_ENABLE_CLOCK( PIN_PORT( CAN1_RX ), PIN_PORT( CAN1_TX ) );

    PIN_INIT( CAN1_RX );
    PIN_INIT( CAN1_TX );
#endif
    
#if ( CAN_WITHOUT_ISR == 0 ) 
    HAL_NVIC_SetPriority( CAN1_TX_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_TX_IRQn );
    
    HAL_NVIC_SetPriority( CAN1_RX0_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_RX0_IRQn );
    
    HAL_NVIC_SetPriority( CAN1_RX1_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_RX1_IRQn );

    HAL_NVIC_SetPriority( CAN1_SCE_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN1_SCE_IRQn );
#endif
  }
  else if( hcan->Instance == CAN2 )
  {
    if( !__HAL_RCC_CAN1_IS_CLK_ENABLED() )
    {
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
    if( !__HAL_RCC_CAN2_IS_CLK_ENABLED() )
    {
      __HAL_RCC_CAN2_CLK_ENABLE();
    }

#ifdef CAN2_RX
    PORT_ENABLE_CLOCK( PIN_PORT( CAN2_RX ), PIN_PORT( CAN2_TX ) );

    PIN_INIT( CAN2_RX );
    PIN_INIT( CAN2_TX );
#endif

#if ( CAN_WITHOUT_ISR == 0 ) 
    HAL_NVIC_SetPriority( CAN2_TX_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN2_TX_IRQn );
    
    HAL_NVIC_SetPriority( CAN2_RX0_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN2_RX0_IRQn );
    
    HAL_NVIC_SetPriority( CAN2_RX1_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN2_RX1_IRQn );
    
    HAL_NVIC_SetPriority( CAN2_SCE_IRQn, 6, 0 );
    HAL_NVIC_EnableIRQ( CAN2_SCE_IRQn );
#endif
  }
}

void HAL_CAN_MspDeInit( CAN_HandleTypeDef *hcan )
{
  if( hcan->Instance == CAN1 )
  {
    /* if CAN2 not used anymore */
    if( !__HAL_RCC_CAN2_IS_CLK_ENABLED() )
    {
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
#ifdef CAN1_RX
    PIN_DEINIT( CAN1_RX );
    PIN_DEINIT( CAN1_TX );
#endif
    
    HAL_NVIC_DisableIRQ( CAN1_TX_IRQn );
    HAL_NVIC_DisableIRQ( CAN1_RX0_IRQn );
    HAL_NVIC_DisableIRQ( CAN1_RX1_IRQn );
    HAL_NVIC_DisableIRQ( CAN1_SCE_IRQn );
  }
  else if( hcan->Instance == CAN2 )
  {
    __HAL_RCC_CAN2_CLK_DISABLE();

#ifdef CAN2_RX 
    PIN_DEINIT( CAN2_RX );
    PIN_DEINIT( CAN2_TX );
#endif
    
    HAL_NVIC_DisableIRQ( CAN2_TX_IRQn );
    HAL_NVIC_DisableIRQ( CAN2_RX0_IRQn );
    HAL_NVIC_DisableIRQ( CAN2_RX1_IRQn );
    HAL_NVIC_DisableIRQ( CAN2_SCE_IRQn );
  }
}


/* CAN HAL subsystem callbacks */
void HAL_CAN_TxMailbox0CompleteCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_tx_complete( _bus_from_int_dev( hcan->Instance ), 0 );
}

void HAL_CAN_TxMailbox1CompleteCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_tx_complete( _bus_from_int_dev( hcan->Instance ), 1 ); 
}

void HAL_CAN_TxMailbox2CompleteCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_tx_complete( _bus_from_int_dev( hcan->Instance ), 2 );
}

void HAL_CAN_TxMailbox0AbortCallback( CAN_HandleTypeDef *hcan ){}
void HAL_CAN_TxMailbox1AbortCallback( CAN_HandleTypeDef *hcan ){}
void HAL_CAN_TxMailbox2AbortCallback( CAN_HandleTypeDef *hcan ){}

void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_isr_frame( hcan, CAN_RX_FIFO0 );
}

void HAL_CAN_RxFifo1MsgPendingCallback( CAN_HandleTypeDef *hcan )
{
  pcan_can_isr_frame( hcan, CAN_RX_FIFO1 );
}

void HAL_CAN_RxFifo0FullCallback( CAN_HandleTypeDef *hcan )
{
}

void HAL_CAN_RxFifo1FullCallback( CAN_HandleTypeDef *hcan )
{
}

void HAL_CAN_SleepCallback( CAN_HandleTypeDef *hcan ){}
void HAL_CAN_WakeUpFromRxMsgCallback( CAN_HandleTypeDef *hcan ){}

void HAL_CAN_ErrorCallback( CAN_HandleTypeDef *hcan )
{
  /* handle errors */
  uint32_t err = HAL_CAN_GetError( hcan );
  int bus = _bus_from_int_dev( hcan->Instance );
  
  if ( err & HAL_CAN_ERROR_TX_TERR0 ) 
  {
    pcan_can_tx_err( bus, 0 );
  }
  
  if ( err & HAL_CAN_ERROR_TX_TERR1 ) 
  {
    pcan_can_tx_err( bus, 1 );
  }
  
  if ( err & HAL_CAN_ERROR_TX_TERR2 ) 
  {
    pcan_can_tx_err( bus, 2 );
  }
  
  HAL_CAN_ResetError( hcan );
}


/* ISR handlers */
#if ( CAN_WITHOUT_ISR == 0 ) 
/* CAN1 */
void CAN1_TX_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
}

void CAN1_RX0_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
}

void CAN1_RX1_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
}

void CAN1_SCE_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_1] );
}
/* CAN2 */
void CAN2_TX_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
}

void CAN2_RX0_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
}

void CAN2_RX1_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
}

void CAN2_SCE_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &hcan[CAN_BUS_2] );
}
#endif

#endif /* STM32G431xx */
