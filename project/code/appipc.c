#include "appipc.h"

#include "cy_project.h"
#include "cy_device_headers.h"
#include "ipc/cy_ipc_drv.h"
#include "ipc/cy_ipc_config.h"
#include "zf_common_interrupt.h"

// 通道0：核心1发送跳跃标志或视觉数据，核心0接收。
#define APPIPC_CHANNEL              (CY_IPC_CHAN_PIPE_EP0)
#define APPIPC_RX_INTR_STRUCT       (CY_IPC_INTR_PIPE_EP0)
#define APPIPC_RX_CPU_INT           (CPUIntIdx4_IRQn)
#define APPIPC_RX_IRQ_PRIORITY      (3u)

#define APPIPC_CHANNEL_MASK         (1ul << APPIPC_CHANNEL)
#define APPIPC_RX_INTR_MASK         (1ul << APPIPC_RX_INTR_STRUCT)

// 通道1：核心0发送速度，核心1接收。
#define APPIPC_SPEED_CHANNEL              (CY_IPC_CHAN_PIPE_EP1)
#define APPIPC_SPEED_RX_INTR_STRUCT       (CY_IPC_INTR_PIPE_EP1)
#define APPIPC_SPEED_RX_CPU_INT           (CPUIntIdx4_IRQn)
#define APPIPC_SPEED_RX_IRQ_PRIORITY      (3u)

#define APPIPC_SPEED_CHANNEL_MASK         (1ul << APPIPC_SPEED_CHANNEL)
#define APPIPC_SPEED_RX_INTR_MASK         (1ul << APPIPC_SPEED_RX_INTR_STRUCT)

#define APPIPC_BRIDGE_VALID_MASK          (0x80000000ul)
#define APPIPC_BRIDGE_DISTANCE_MASK       (0x00FF0000ul)
#define APPIPC_BRIDGE_DISTANCE_SHIFT      (16u)
#define APPIPC_BRIDGE_ANGLE_MASK          (0x0000FFFFul)
#define APPIPC_BRIDGE_DISTANCE_MAX        (127)
#define APPIPC_BRIDGE_DISTANCE_MIN        (-127)

static appipc_callback_t appipc_user_callback = (appipc_callback_t)0;
static volatile stc_IPC_STRUCT_t      *appipc_channel_ptr = (volatile stc_IPC_STRUCT_t *)0;
static volatile stc_IPC_INTR_STRUCT_t *appipc_intr_ptr    = (volatile stc_IPC_INTR_STRUCT_t *)0;

static appipc_callback_t appipc_speed_user_callback = (appipc_callback_t)0;
static volatile stc_IPC_STRUCT_t      *appipc_speed_channel_ptr = (volatile stc_IPC_STRUCT_t *)0;
static volatile stc_IPC_INTR_STRUCT_t *appipc_speed_intr_ptr    = (volatile stc_IPC_INTR_STRUCT_t *)0;

static void appipc_default_callback(uint32 data)
{
    (void)data;
}

static uint32 appipc_pack_bridge_data(uint8 valid, int16 distance_px, int16 angle_d10)
{
    if(!valid)
    {
        return 0u;
    }

    if(distance_px > APPIPC_BRIDGE_DISTANCE_MAX)
    {
        distance_px = APPIPC_BRIDGE_DISTANCE_MAX;
    }
    else if(distance_px < APPIPC_BRIDGE_DISTANCE_MIN)
    {
        distance_px = APPIPC_BRIDGE_DISTANCE_MIN;
    }

    return APPIPC_BRIDGE_VALID_MASK |
           ((uint32)(uint8)(int8)distance_px << APPIPC_BRIDGE_DISTANCE_SHIFT) |
           (uint32)(uint16)angle_d10;
}

static void appipc_read_and_release(void)
{
    uint32 data;

    if(CY_IPC_DRV_SUCCESS == Cy_IPC_Drv_ReadMsgWord(appipc_channel_ptr, &data))
    {
        __DSB();
        (void)Cy_IPC_Drv_LockRelease(appipc_channel_ptr, CY_IPC_NO_NOTIFICATION);
        __DSB();

        appipc_user_callback(data);
    }
}

static void appipc_rx_isr(void)
{
    uint32 intr_status;

    intr_status = Cy_IPC_Drv_GetInterruptStatusMasked(appipc_intr_ptr);

    if(0u != (intr_status & IPC_INTR_STRUCT_INTR_MASK_NOTIFY_Msk))
    {
        Cy_IPC_Drv_ClearInterrupt(appipc_intr_ptr, 0u, APPIPC_CHANNEL_MASK);
        appipc_read_and_release();
    }
}

void appipc_rx_init(appipc_callback_t callback)
{
    cy_stc_sysint_irq_t irq_cfg;

    appipc_user_callback = (callback == (appipc_callback_t)0) ? appipc_default_callback : callback;
    appipc_channel_ptr   = Cy_IPC_Drv_GetIpcBaseAddress(APPIPC_CHANNEL);
    appipc_intr_ptr      = Cy_IPC_Drv_GetIntrBaseAddr(APPIPC_RX_INTR_STRUCT);

    Cy_IPC_Drv_ClearInterrupt(appipc_intr_ptr, 0u, APPIPC_CHANNEL_MASK);
    Cy_IPC_Drv_SetInterruptMask(appipc_intr_ptr, 0u, APPIPC_CHANNEL_MASK);

    irq_cfg.sysIntSrc = (cy_en_intr_t)CY_IPC_INTR_NUM_TO_VECT(APPIPC_RX_INTR_STRUCT);
    irq_cfg.intIdx    = APPIPC_RX_CPU_INT;
    irq_cfg.isEnabled = true;

    interrupt_init(&irq_cfg, appipc_rx_isr, APPIPC_RX_IRQ_PRIORITY);

    if(Cy_IPC_Drv_IsLockAcquired(appipc_channel_ptr))
    {
        appipc_read_and_release();
    }
}

uint8 appipc_send_u32(uint32 data)
{
    volatile stc_IPC_STRUCT_t *ipc_ptr;

    ipc_ptr = Cy_IPC_Drv_GetIpcBaseAddress(APPIPC_CHANNEL);

    if(CY_IPC_DRV_SUCCESS == Cy_IPC_Drv_SendMsgWord(ipc_ptr, APPIPC_RX_INTR_MASK, data))
    {
        return APPIPC_OK;
    }

    return APPIPC_BUSY;
}

uint8 appipc_send_bridge_data(uint8 valid, int16 distance_px, int16 angle_d10)
{
    return appipc_send_u32(appipc_pack_bridge_data(valid, distance_px, angle_d10));
}

uint8 appipc_decode_bridge_data(uint32 data, appipc_bridge_data_t *bridge_data)
{
    if((appipc_bridge_data_t *)0 == bridge_data)
    {
        return 0;
    }

    bridge_data->valid = (uint8)(0u != (data & APPIPC_BRIDGE_VALID_MASK));

    if(!bridge_data->valid)
    {
        bridge_data->distance_px = 0;
        bridge_data->angle_d10 = 0;
        return 0;
    }

    bridge_data->distance_px =
        (int8)((data & APPIPC_BRIDGE_DISTANCE_MASK) >> APPIPC_BRIDGE_DISTANCE_SHIFT);
    bridge_data->angle_d10 = (int16)(data & APPIPC_BRIDGE_ANGLE_MASK);

    return 1;
}

static void appipc_speed_read_and_release(void)
{
    uint32 data;

    if(CY_IPC_DRV_SUCCESS == Cy_IPC_Drv_ReadMsgWord(appipc_speed_channel_ptr, &data))
    {
        __DSB();
        (void)Cy_IPC_Drv_LockRelease(appipc_speed_channel_ptr, CY_IPC_NO_NOTIFICATION);
        __DSB();

        appipc_speed_user_callback(data);
    }
}

static void appipc_speed_rx_isr(void)
{
    uint32 intr_status;

    intr_status = Cy_IPC_Drv_GetInterruptStatusMasked(appipc_speed_intr_ptr);

    if(0u != (intr_status & IPC_INTR_STRUCT_INTR_MASK_NOTIFY_Msk))
    {
        Cy_IPC_Drv_ClearInterrupt(appipc_speed_intr_ptr, 0u, APPIPC_SPEED_CHANNEL_MASK);
        appipc_speed_read_and_release();
    }
}

void appipc_speed_rx_init(appipc_callback_t callback)
{
    cy_stc_sysint_irq_t irq_cfg;

    appipc_speed_user_callback = (callback == (appipc_callback_t)0) ? appipc_default_callback : callback;
    appipc_speed_channel_ptr   = Cy_IPC_Drv_GetIpcBaseAddress(APPIPC_SPEED_CHANNEL);
    appipc_speed_intr_ptr      = Cy_IPC_Drv_GetIntrBaseAddr(APPIPC_SPEED_RX_INTR_STRUCT);

    Cy_IPC_Drv_ClearInterrupt(appipc_speed_intr_ptr, 0u, APPIPC_SPEED_CHANNEL_MASK);
    Cy_IPC_Drv_SetInterruptMask(appipc_speed_intr_ptr, 0u, APPIPC_SPEED_CHANNEL_MASK);

    irq_cfg.sysIntSrc = (cy_en_intr_t)CY_IPC_INTR_NUM_TO_VECT(APPIPC_SPEED_RX_INTR_STRUCT);
    irq_cfg.intIdx    = APPIPC_SPEED_RX_CPU_INT;
    irq_cfg.isEnabled = true;

    interrupt_init(&irq_cfg, appipc_speed_rx_isr, APPIPC_SPEED_RX_IRQ_PRIORITY);

    if(Cy_IPC_Drv_IsLockAcquired(appipc_speed_channel_ptr))
    {
        appipc_speed_read_and_release();
    }
}

uint8 appipc_send_speed_u32(uint32 data)
{
    volatile stc_IPC_STRUCT_t *ipc_ptr;

    ipc_ptr = Cy_IPC_Drv_GetIpcBaseAddress(APPIPC_SPEED_CHANNEL);

    if(CY_IPC_DRV_SUCCESS == Cy_IPC_Drv_SendMsgWord(ipc_ptr, APPIPC_SPEED_RX_INTR_MASK, data))
    {
        return APPIPC_OK;
    }

    return APPIPC_BUSY;
}
