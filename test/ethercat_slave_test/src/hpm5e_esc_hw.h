/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WBR_HPM5E_ESC_HW_H_
#define WBR_HPM5E_ESC_HW_H_

#include <stddef.h>
#include <stdint.h>

#include <drivers/ethercat/ethercat_hpmicro.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t hpm5e_ssc_hw_init(void);
void hpm5e_ssc_hw_release(void);
void hpm5e_ssc_esc_read(volatile void *data, uint16_t address, uint16_t length);
void hpm5e_ssc_esc_write(const volatile void *data, uint16_t address, uint16_t length);
uint16_t hpm5e_ssc_al_event(void);
uint32_t hpm5e_ssc_timer_get(void);
void hpm5e_ssc_timer_clear(void);
uint16_t hpm5e_ssc_eeprom_reload(void);
void hpm5e_ssc_irq_enable(void);
void hpm5e_ssc_irq_disable(void);
void hpm5e_ssc_enter_critical(void);
void hpm5e_ssc_leave_critical(void);

#define ECAT_TIMER_INC_P_MS 1U
#define HW_Init() hpm5e_ssc_hw_init()
#define HW_Release() hpm5e_ssc_hw_release()
#define HW_GetTimer() hpm5e_ssc_timer_get()
#define HW_ClearTimer() hpm5e_ssc_timer_clear()
#define HW_GetALEventRegister() hpm5e_ssc_al_event()
#define HW_GetALEventRegister_Isr() hpm5e_ssc_al_event()

#define HW_EscRead(pData, Address, Len) \
	hpm5e_ssc_esc_read((pData), (uint16_t)(Address), (uint16_t)(Len))
#define HW_EscReadIsr(pData, Address, Len) HW_EscRead((pData), (Address), (Len))
#define HW_EscReadMbxMem(pData, Address, Len) HW_EscRead((pData), (Address), (Len))
#define HW_EscWrite(pData, Address, Len) \
	hpm5e_ssc_esc_write((pData), (uint16_t)(Address), (uint16_t)(Len))
#define HW_EscWriteIsr(pData, Address, Len) HW_EscWrite((pData), (Address), (Len))
#define HW_EscWriteMbxMem(pData, Address, Len) HW_EscWrite((pData), (Address), (Len))

#define HW_EscReadByte(value, address) hpm5e_ssc_esc_read(&(value), (address), 1U)
#define HW_EscReadByteIsr(value, address) HW_EscReadByte((value), (address))
#define HW_EscReadWord(value, address) hpm5e_ssc_esc_read(&(value), (address), 2U)
#define HW_EscReadWordIsr(value, address) HW_EscReadWord((value), (address))
#define HW_EscReadDWord(value, address) hpm5e_ssc_esc_read(&(value), (address), 4U)
#define HW_EscReadDWordIsr(value, address) HW_EscReadDWord((value), (address))
#define HW_EscWriteByte(value, address) hpm5e_ssc_esc_write(&(value), (address), 1U)
#define HW_EscWriteByteIsr(value, address) HW_EscWriteByte((value), (address))
#define HW_EscWriteWord(value, address) hpm5e_ssc_esc_write(&(value), (address), 2U)
#define HW_EscWriteWordIsr(value, address) HW_EscWriteWord((value), (address))
#define HW_EscWriteDWord(value, address) hpm5e_ssc_esc_write(&(value), (address), 4U)
#define HW_EscWriteDWordIsr(value, address) HW_EscWriteDWord((value), (address))

#define HW_EepromReload() hpm5e_ssc_eeprom_reload()
#define ENABLE_ESC_INT() hpm5e_ssc_irq_enable()
#define DISABLE_ESC_INT() hpm5e_ssc_irq_disable()

#ifdef __cplusplus
}
#endif

#endif /* WBR_HPM5E_ESC_HW_H_ */
