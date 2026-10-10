/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include <drivers/ethercat/ethercat_hpmicro.h>
#include <hpm_esc_drv.h>
#include <hpm_gpio_drv.h>
#include <hpm_iomux.h>

#include "applInterface.h"
#include "ecatappl.h"

#define SII_SIZE 2048U

extern unsigned char aEepromData[SII_SIZE];

static const struct device *const esc = DEVICE_DT_GET(DT_NODELABEL(esc0));
static uint8_t sii_image[SII_SIZE];
static atomic_t stack_ready;
static atomic_t reset_requested;
static uint32_t phy_present;
static uint16_t previous_bmsr[32];

static void esc_sm_dump(const char *stage)
{
	volatile uint8_t *bytes = (volatile uint8_t *)DT_REG_ADDR(DT_NODELABEL(esc0));
	ESC_Type *base = (ESC_Type *)bytes;
	printk("ESC state %s: GPR_STATUS=%08x DL_CTRL=%08x DL_STAT=%04x AL=%04x code=%04x event=%08x\n",
	       stage, base->GPR_STATUS, base->ESC_DL_CTRL, base->ESC_DL_STAT,
	       base->AL_STAT, base->AL_STAT_CODE, base->AL_EVT_REQ);
	for (unsigned int i = 0; i < MIN(base->SYNCM_NUM, 8U); i++) {
		volatile uint32_t *sm = (volatile uint32_t *)&bytes[0x800 + i * 8];
		const uint32_t first = sm[0];
		const uint32_t second = sm[1];
		printk("SM%u: raw=%08x:%08x start=%04x len=%u ctrl=%02x status=%02x activate=%02x pdi=%02x\n",
		       i, first, second, first & 0xffffU, first >> 16,
		       second & 0xffU, (second >> 8) & 0xffU,
		       (second >> 16) & 0xffU, second >> 24);
	}
}

static bool esc_access_probe(void)
{
	volatile uint8_t *bytes = (volatile uint8_t *)DT_REG_ADDR(DT_NODELABEL(esc0));
	ESC_Type *base = (ESC_Type *)bytes;
	/* Standalone startup only: 0x1800 is outside this sample's mailbox/PDO
	 * regions. Do not run this probe while a master owns process data. */
	if (base->RAM_SIZE >= 3U) {
		volatile uint32_t *ram = (volatile uint32_t *)&bytes[0x1800];
		const uint32_t original = *ram;
		const uint32_t patterns[] = { 0x55aa33ccU, 0xaa55cc33U };
		for (size_t i = 0; i < ARRAY_SIZE(patterns); i++) {
			*ram = patterns[i];
			const uint32_t actual = *ram;
			printk("ESC RAM32: offset=1800 expected=%08x actual=%08x pass=%u\n",
			       patterns[i], actual, actual == patterns[i]);
		}
		*ram = original;
		printk("ESC RAM32 restore: expected=%08x actual=%08x\n", original, *ram);
	} else {
		printk("ESC RAM probe skipped: RAM_SIZE=%u KiB\n", base->RAM_SIZE);
	}
	const uint32_t mask_saved = base->PDI_AL_EVT_MSK;
	/* Only toggle AL-control event bit; CPU ESC IRQs are masked here. */
	const uint32_t mask_test = mask_saved ^ BIT(0);
	base->PDI_AL_EVT_MSK = mask_test;
	const uint32_t mask_actual = base->PDI_AL_EVT_MSK;
	base->PDI_AL_EVT_MSK = mask_saved;
	printk("ESC AL mask: expected=%08x actual=%08x pass=%u restored=%08x\n",
	       mask_test, mask_actual, mask_actual == mask_test, base->PDI_AL_EVT_MSK);
	volatile uint32_t *sm0 = (volatile uint32_t *)&bytes[0x804];
	const uint32_t saved = *sm0;
	esc_sm_dump("before SM0 request");
	/* 0x804..0x806 are PDI read-only; bit 24 is SM0 PDI disable. */
	bytes[0x807] = 1;
	uint8_t byte_after = 0;
	int64_t deadline = k_uptime_get() + 100;
	do {
		byte_after = bytes[0x807];
		if (byte_after & 1U) { break; }
		k_msleep(1);
	} while (k_uptime_get() < deadline);
	printk("ESC access: byte disable SM0 before32=%08x after8=%02x after32=%08x\n",
	       saved, byte_after, *sm0);
	*sm0 = 0x01000000U;
	uint32_t word_after = 0;
	deadline = k_uptime_get() + 100;
	do {
		word_after = *sm0;
		if (word_after & BIT(24)) { break; }
		k_msleep(1);
	} while (k_uptime_get() < deadline);
	printk("ESC access: aligned32 disable SM0 after32=%08x after8=%02x\n",
	       word_after, bytes[0x807]);
	esc_sm_dump("after SM0 request (100ms per width)");
	/* Restore only the writable PDI byte using the aligned bus transaction. */
	*sm0 = saved & 0xff000000U;
	printk("ESC access: restored SM0=%08x; byte_write=%u aligned32_write=%u\n",
	       *sm0, !!(byte_after & 1U), !!(word_after & BIT(24)));
	if (!(byte_after & 1U) && bytes[0x802] == 0 && bytes[0x803] == 0 &&
	    !(bytes[0x806] & 1U)) {
		printk("ESC experiment: SM0 unconfigured and inactive; allow SSC startup experiment\n");
		return true;
	}
	return (byte_after & 1U) != 0U;
}

static void phy_config_dump(void)
{
	ESC_Type *base = (ESC_Type *)DT_REG_ADDR(DT_NODELABEL(esc0));
	printk("ESC PDI: type=%02x info=%04x cfg=%02x ext=%04x DL=%04x EEPROM=%04x active=%u invalid=%u\n",
	       base->PDI_CTRL, base->PDI_INFO, base->PDI_CFG, base->PDI_EXT_CFG,
	       base->ESC_DL_STAT, base->EEPROM_CTRL_STAT,
	       ESC_PDI_INFO_PDIA_GET(base->PDI_INFO),
	       ESC_PDI_INFO_PDICN_GET(base->PDI_INFO));
	const uint16_t pads[] = { IOC_PAD_PW21, IOC_PAD_PV12, IOC_PAD_PW17,
		IOC_PAD_PA27, IOC_PAD_PA28, IOC_PAD_PA29, IOC_PAD_PA30,
		IOC_PAD_PA31, IOC_PAD_PA20, IOC_PAD_PA21, IOC_PAD_PA22, IOC_PAD_PA23 };
	const char *names[] = { "PW21", "PV12", "PW17", "PA27", "PA28",
		"PA29", "PA30", "PA31", "PA20", "PA21", "PA22", "PA23" };
	printk("ESC cfg: GPR0=%08x GPR1=%08x GPR2=%08x PHY0=%08x PHY1=%08x\n",
	       base->GPR_CFG0, base->GPR_CFG1, base->GPR_CFG2,
	       base->PHY_CFG0, base->PHY_CFG1);
	printk("PHY reset: PV12 output=%u input=%u; PA28 level=%u\n",
	       gpio_get_pin_output_status(HPM_GPIO0, GPIO_DO_GPIOV, 12),
	       gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOV, 12),
	       gpio_read_pin(HPM_GPIO0, GPIO_DI_GPIOA, 28));
	for (size_t i = 0; i < ARRAY_SIZE(pads); i++) {
		printk("IOC %s FUNC=%08x PAD=%08x\n", names[i],
		       HPM_IOC->PAD[pads[i]].FUNC_CTL, HPM_IOC->PAD[pads[i]].PAD_CTL);
	}
}

static void sip_phy_prepare(void)
{
	/* HPM5E00 SIP PHY guide v1.1: PHY0 MII is connected to PV internally. */
	const uint16_t mii_pads[] = {
		IOC_PAD_PV00, IOC_PAD_PV01, IOC_PAD_PV02, IOC_PAD_PV03,
		IOC_PAD_PV04, IOC_PAD_PV05, IOC_PAD_PV06, IOC_PAD_PV07,
		IOC_PAD_PV08, IOC_PAD_PV09, IOC_PAD_PV10, IOC_PAD_PV11,
		IOC_PAD_PV15,
	};
	for (size_t i = 0; i < ARRAY_SIZE(mii_pads); i++) {
		HPM_IOC->PAD[mii_pads[i]].FUNC_CTL = IOC_PAD_FUNC_CTL_ALT_SELECT_SET(11);
	}
	HPM_IOC->PAD[IOC_PAD_PW21].FUNC_CTL = IOC_PW21_FUNC_CTL_ESC0_REFCK;
	HPM_IOC->PAD[IOC_PAD_PV12].FUNC_CTL = IOC_PV12_FUNC_CTL_GPIO_V_12;
	gpio_set_pin_output_with_initial(HPM_GPIO0, GPIO_DO_GPIOV, 12, 0);
	/* Unused PHY1 stays in reset; its analog pins are not available for UART. */
	HPM_IOC->PAD[IOC_PAD_PW12].FUNC_CTL = IOC_PW12_FUNC_CTL_GPIO_W_12;
	gpio_set_pin_output_with_initial(HPM_GPIO0, GPIO_DO_GPIOW, 12, 0);
	const uint16_t analog_pads[] = {
		IOC_PAD_PA16, IOC_PAD_PA17, IOC_PAD_PA18, IOC_PAD_PA19,
		IOC_PAD_PA20, IOC_PAD_PA21, IOC_PAD_PA22, IOC_PAD_PA23,
		IOC_PAD_PA24, IOC_PAD_PA25, IOC_PAD_PA26, IOC_PAD_PA27,
		IOC_PAD_PA29, IOC_PAD_PW16, IOC_PAD_PW17,
	};
	for (size_t i = 0; i < ARRAY_SIZE(analog_pads); i++) {
		HPM_IOC->PAD[analog_pads[i]].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
	}
	/* Address straps must be set by board resistors while reset is held.
	 * Defer digital link mux and internal pulls until after reset settling. */
	HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
	HPM_IOC->PAD[IOC_PAD_PA28].PAD_CTL &= ~IOC_PAD_PAD_CTL_PE_MASK;
	HPM_IOC->PAD[IOC_PAD_PA29].PAD_CTL &= ~IOC_PAD_PAD_CTL_PE_MASK;
}

/* The SDK takes PDI ownership for each MDIO read. Always return it to ESC,
 * including failed reads, so autonomous link monitoring can continue. */
static hpm_stat_t phy_read(uint8_t address, uint8_t reg, uint16_t *value)
{
	ESC_Type *base = (ESC_Type *)DT_REG_ADDR(DT_NODELABEL(esc0));
	hpm_stat_t rc = esc_mdio_read(base, address, reg, value);
	base->MIIM_PDI_ACC_STAT = 0;
	return rc;
}

static void phy_scan(void)
{
	printk("PHY diagnostic: offset=%u; scanning MDIO 0..31\n",
	       DT_PROP(DT_NODELABEL(esc0), phy_address_offset));
	phy_present = 0;
	for (uint8_t address = 0; address < 32; address++) {
		uint16_t id1 = 0xffff, id2 = 0xffff;
		hpm_stat_t rc1 = phy_read(address, 2, &id1);
		hpm_stat_t rc2 = phy_read(address, 3, &id2);
		printk("PHY addr=%u ID=%04x:%04x rc=%ld/%ld\n", address,
		       id1, id2, (long)rc1, (long)rc2);
		uint16_t bmcr = 0xffff, bmsr = 0xffff;
		hpm_stat_t rc0 = phy_read(address, 0, &bmcr);
		hpm_stat_t rcs = phy_read(address, 1, &bmsr);
		printk("PHY addr=%u BMCR=%04x BMSR=%04x rc=%ld/%ld\n",
		       address, bmcr, bmsr, (long)rc0, (long)rcs);
		if (rc0 != status_success || rcs != status_success ||
		    bmcr == 0xffff || bmsr == 0xffff || (bmcr == 0 && bmsr == 0)) {
			continue;
		}
		phy_present |= BIT(address);
		previous_bmsr[address] = 0xffff;
		for (uint8_t reg = 0; reg <= 6; reg++) {
			uint16_t value = 0xffff;
			hpm_stat_t rc = phy_read(address, reg, &value);
			printk("PHY addr=%u reg=%u value=%04x rc=%ld\n",
			       address, reg, value, (long)rc);
		}
		/* Give the RTT reader time to consume the startup dump. */
		k_msleep(20);
	}
	printk("PHY scan complete mask=%08x (multiple addresses may be aliases)\n",
	       phy_present);
}

static void phy_poll(bool heartbeat)
{
	for (uint8_t address = 0; address < 32; address++) {
		if (!(phy_present & BIT(address))) {
			continue;
		}
		uint16_t bmcr = 0, bmsr = 0;
		/* BMSR link-down is latched; read twice for the current state. */
		hpm_stat_t rc = phy_read(address, 1, &bmsr);
		if (rc == status_success) {
			rc = phy_read(address, 1, &bmsr);
		}
		if (rc != status_success ||
		    phy_read(address, 0, &bmcr) != status_success) {
			printk("PHY addr=%u read failed\n", address);
			continue;
		}
		if (heartbeat || bmsr != previous_bmsr[address]) {
			printk("PHY t=%lld addr=%u BMCR=%04x BMSR=%04x link=%u "
			       "reset=%u powerdown=%u isolate=%u AN=%u ANdone=%u "
			       "speed_bit100=%u duplex_bit=%u\n",
			       (long long)k_uptime_get(), address, bmcr, bmsr,
			       !!(bmsr & BIT(2)), !!(bmcr & BIT(15)),
			       !!(bmcr & BIT(11)), !!(bmcr & BIT(10)),
			       !!(bmcr & BIT(12)), !!(bmsr & BIT(5)),
			       !!(bmcr & BIT(13)), !!(bmcr & BIT(8)));
		}
		previous_bmsr[address] = bmsr;
	}
}

static int sii_read(uint32_t word_addr, uint16_t *data, size_t word_count, void *user_data)
{
	uint8_t *image = user_data;
	const size_t offset = (size_t)word_addr * sizeof(uint16_t);
	const size_t length = word_count * sizeof(uint16_t);

	if (offset + length > SII_SIZE) {
		return -EINVAL;
	}
	memcpy(data, image + offset, length);
	return 0;
}

static int sii_write(uint32_t word_addr, uint16_t data, void *user_data)
{
	uint8_t *image = user_data;
	const size_t offset = (size_t)word_addr * sizeof(uint16_t);

	if (offset + sizeof(data) > SII_SIZE) {
		return -EINVAL;
	}
	memcpy(image + offset, &data, sizeof(data));
	return 0;
}

static int sii_reload(uint32_t *data, void *user_data)
{
	uint8_t *image = user_data;
	/* HPM ESC reload response: alias in bits 15:0, enhanced link
	 * detection and port enables in bits 20:16 (not raw SII word 0/1). */
	const uint8_t cfg = image[1];
	*data = (uint32_t)image[8] | ((uint32_t)image[9] << 8) |
		(ESC_ESC_CFG_ELDAP_GET(cfg) << 16) |
		(ESC_ESC_CFG_ELP0_GET(cfg) << 17) |
		(ESC_ESC_CFG_ELP1_GET(cfg) << 18) |
		(ESC_ESC_CFG_ELP2_GET(cfg) << 19) |
		(ESC_ESC_CFG_ELP3_GET(cfg) << 20);
	printk("SII reload: config=%02x%02x response=%08x\n", image[1], image[0], *data);
	return 0;
}

static void pdi_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	(void)hpm_ethercat_process_eeprom(esc);
	if (atomic_get(&stack_ready)) {
		PDI_Isr();
	}
}

static void reset_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	atomic_set(&reset_requested, 1);
}

int main(void)
{
	const struct hpm_ethercat_eeprom_ops eeprom_ops = {
		.read = sii_read,
		.write = sii_write,
		.reload = sii_reload,
		.user_data = sii_image,
	};
	const struct hpm_ethercat_callbacks callbacks = {
		.pdi = pdi_callback,
		.reset = reset_callback,
	};

	if (!device_is_ready(esc)) {
		printk("EtherCAT ESC device is not ready\n");
		return 0;
	}

	memcpy(sii_image, aEepromData, sizeof(sii_image));
	sip_phy_prepare();
	if (hpm_ethercat_set_eeprom_ops(esc, &eeprom_ops) != 0 ||
	    hpm_ethercat_set_callbacks(esc, &callbacks) != 0 ||
	    hpm_ethercat_start(esc) != 0) {
		printk("EtherCAT ESC start failed\n");
		return 0;
	}
	/* Prevent AL-event storms before SSC has installed its masks/state. */
	for (enum hpm_ethercat_irq irq = HPM_ETHERCAT_IRQ_PDI;
	     irq <= HPM_ETHERCAT_IRQ_RESET; irq++) {
		hpm_ethercat_irq_enable(esc, irq, false);
	}
	/* ESC start enabled the shared 25 MHz reference; release SIP reset now. */
	k_msleep(100);
	printk("SIP strap experiment: PA28/PA29 analog, internal pulls off; release PV12\n");
	gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOV, 12, 1);
	k_msleep(100);
	HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_ESC0_CTR_1;
	HPM_IOC->PAD[IOC_PAD_PA28].PAD_CTL =
		IOC_PAD_PAD_CTL_PE_SET(1) | IOC_PAD_PAD_CTL_PS_SET(1);
	printk("SIP PHY0: PV12 released, PW21 REFCK, PA28 CTR1; PHY1 held reset\n");
	phy_config_dump();
	phy_scan();
	phy_poll(true);
	if (!esc_access_probe()) {
		printk("SSC skipped: byte SM0 write failed; PHY diagnostic remains running\n");
		for (;;) {
			phy_poll(true);
			k_msleep(5000);
		}
	}
	printk("SSC stage: entering MainInit (ESC IRQs masked)\n");

	if (MainInit() != 0U) {
		printk("SSC MainInit failed\n");
		return 0;
	}
	atomic_set(&stack_ready, 1);
	hpm_ethercat_irq_enable(esc, HPM_ETHERCAT_IRQ_PDI, true);
	hpm_ethercat_irq_enable(esc, HPM_ETHERCAT_IRQ_RESET, true);
	printk("WBR HPM5E SSC FreeRun slave ready: VID=0 PID=0x57425231\n");
	int64_t next_phy_poll = 0;
	unsigned int poll_count = 0;

	for (;;) {
		if (atomic_cas(&reset_requested, 1, 0)) {
			atomic_clear(&stack_ready);
			(void)MainInit();
			atomic_set(&stack_ready, 1);
			printk("SSC reset handled\n");
		}
		MainLoop();
		if (k_uptime_get() >= next_phy_poll) {
			phy_poll((poll_count++ % 10U) == 0U);
			next_phy_poll = k_uptime_get() + 1000;
		}
		k_usleep(100);
	}
	return 0;
}
