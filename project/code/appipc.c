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

// 通道1：核心0发送车速、遥控器通道 9、视觉工作模式和 BAB 子状态，核心1接收。
#define APPIPC_SPEED_CHANNEL              (CY_IPC_CHAN_PIPE_EP1)
#define APPIPC_SPEED_RX_INTR_STRUCT       (CY_IPC_INTR_PIPE_EP1)
#define APPIPC_SPEED_RX_CPU_INT           (CPUIntIdx4_IRQn)
#define APPIPC_SPEED_RX_IRQ_PRIORITY      (3u)

#define APPIPC_SPEED_CHANNEL_MASK         (1ul << APPIPC_SPEED_CHANNEL)
#define APPIPC_SPEED_RX_INTR_MASK         (1ul << APPIPC_SPEED_RX_INTR_STRUCT)

#define APPIPC_BRIDGE_VALID_MASK          (0x80000000ul)
#define APPIPC_BRIDGE_ALIGNED_MASK        (0x40000000ul)
#define APPIPC_BRIDGE_EXITED_MASK         (0x20000000ul)
#define APPIPC_BRIDGE_FORCE_BLIND_MASK     (0x10000000ul)
#define APPIPC_BRIDGE_BLIND_RELEASE_MASK   (0x08000000ul)
#define APPIPC_BRIDGE_FRESH_TARGET_MASK    (0x04000000ul)
#define APPIPC_BRIDGE_BUMP_START_MASK      (0x02000000ul)
#define APPIPC_BRIDGE_BOTTOM_MASK         (0x00FF0000ul)
#define APPIPC_BRIDGE_BOTTOM_SHIFT        (16u)
#define APPIPC_BRIDGE_CONTROL_MASK        (0x0000FFFFul)

#define APPIPC_CORE0_BUMP_FINISH_MASK      (0x80000000ul)
#define APPIPC_CORE0_SPEED_MASK            (0x0000FFFFul)
#define APPIPC_CORE0_REMOTE_CH9_MASK       (0x00FF0000ul)
#define APPIPC_CORE0_REMOTE_CH9_SHIFT      (16u)
#define APPIPC_CORE0_MODE_MASK             (0x0F000000ul)
#define APPIPC_CORE0_MODE_SHIFT            (24u)
#define APPIPC_CORE0_BAB_PHASE_MASK        (0x70000000ul)
#define APPIPC_CORE0_BAB_PHASE_SHIFT       (28u)

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

static uint32 appipc_pack_bridge_data(uint8 valid, uint8 aligned, uint8 exited,
                                     uint8 bottom_y, int16 control_value,
                                     uint8 force_blind, uint8 blind_release,
                                     uint8 fresh_target, uint8 vision_bump_start)
{
    uint32 data =
        (exited ? APPIPC_BRIDGE_EXITED_MASK : 0u) |
        (force_blind ? APPIPC_BRIDGE_FORCE_BLIND_MASK : 0u) |
        (blind_release ? APPIPC_BRIDGE_BLIND_RELEASE_MASK : 0u) |
        (fresh_target ? APPIPC_BRIDGE_FRESH_TARGET_MASK : 0u) |
        (vision_bump_start ? APPIPC_BRIDGE_BUMP_START_MASK : 0u);

    if(valid)
    {
        data |= APPIPC_BRIDGE_VALID_MASK |
                (aligned ? APPIPC_BRIDGE_ALIGNED_MASK : 0u) |
                ((uint32)bottom_y << APPIPC_BRIDGE_BOTTOM_SHIFT) |
                (uint32)(uint16)control_value;
    }

    return data;
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

uint8 appipc_send_bridge_data(uint8 valid, uint8 aligned, uint8 exited,
                              uint8 bottom_y, int16 control_value,
                              uint8 force_blind, uint8 blind_release,
                              uint8 fresh_target, uint8 vision_bump_start)
{
    return appipc_send_u32(appipc_pack_bridge_data(
        valid, aligned, exited, bottom_y, control_value,
        force_blind, blind_release, fresh_target, vision_bump_start));
}

uint8 appipc_decode_bridge_data(uint32 data, appipc_bridge_data_t *bridge_data)
{
    if((appipc_bridge_data_t *)0 == bridge_data)
    {
        return 0;
    }

    bridge_data->valid = (uint8)(0u != (data & APPIPC_BRIDGE_VALID_MASK));
    bridge_data->aligned = (uint8)(bridge_data->valid && (0u != (data & APPIPC_BRIDGE_ALIGNED_MASK)));
    bridge_data->exited = (uint8)(0u != (data & APPIPC_BRIDGE_EXITED_MASK));
    bridge_data->force_blind = (uint8)(0u != (data & APPIPC_BRIDGE_FORCE_BLIND_MASK));
    bridge_data->blind_release = (uint8)(0u != (data & APPIPC_BRIDGE_BLIND_RELEASE_MASK));
    bridge_data->fresh_target = (uint8)(0u != (data & APPIPC_BRIDGE_FRESH_TARGET_MASK));
    bridge_data->vision_bump_start = (uint8)(0u != (data & APPIPC_BRIDGE_BUMP_START_MASK));

    if(!bridge_data->valid)
    {
        bridge_data->bottom_y = 0;
        bridge_data->control_value = 0;
        return 1;
    }

    bridge_data->bottom_y =
        (uint8)((data & APPIPC_BRIDGE_BOTTOM_MASK) >> APPIPC_BRIDGE_BOTTOM_SHIFT);
    bridge_data->control_value = (int16)(data & APPIPC_BRIDGE_CONTROL_MASK);

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

uint8 appipc_send_core0_data(uint16 car_speed, uint8 vision_detect_mode, uint8 vision_phase_bab,
                             uint8 remote_ch9_value, uint8 vision_bump_finish)
{
    uint32 data;

    data = (vision_bump_finish ? APPIPC_CORE0_BUMP_FINISH_MASK : 0u) |
           (((uint32)vision_phase_bab << APPIPC_CORE0_BAB_PHASE_SHIFT) & APPIPC_CORE0_BAB_PHASE_MASK) |
           (((uint32)vision_detect_mode << APPIPC_CORE0_MODE_SHIFT) & APPIPC_CORE0_MODE_MASK) |
           ((uint32)remote_ch9_value << APPIPC_CORE0_REMOTE_CH9_SHIFT) |
           (uint32)car_speed;

    return appipc_send_speed_u32(data);
}

uint8 appipc_decode_core0_data(uint32 data, appipc_core0_data_t *core0_data)
{
    uint8 vision_detect_mode;
    uint8 vision_phase_bab;

    if((appipc_core0_data_t *)0 == core0_data)
    {
        return 0;
    }

    vision_detect_mode = (uint8)((data & APPIPC_CORE0_MODE_MASK) >> APPIPC_CORE0_MODE_SHIFT);
    if(vision_detect_mode > VISION_BACK)
    {
        return 0;
    }

    vision_phase_bab = (uint8)((data & APPIPC_CORE0_BAB_PHASE_MASK) >> APPIPC_CORE0_BAB_PHASE_SHIFT);
    if(vision_phase_bab > VISION_PHASE_BAB_COMPLETE)
    {
        return 0;
    }

    core0_data->car_speed = (uint16)(data & APPIPC_CORE0_SPEED_MASK);
    core0_data->vision_detect_mode = vision_detect_mode;
    core0_data->vision_phase_bab = vision_phase_bab;
    core0_data->remote_ch9_value = (uint8)((data & APPIPC_CORE0_REMOTE_CH9_MASK) >> APPIPC_CORE0_REMOTE_CH9_SHIFT);
    core0_data->vision_bump_finish = (uint8)(0u != (data & APPIPC_CORE0_BUMP_FINISH_MASK));

    return 1;
}
