/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include <app/services/chassis/chassis_tuning_service.h>

namespace {

constexpr size_t kMaxProviders = 8U;

struct ProviderSlot {
	rm_test::app::services::chassis_tuning::SpeedPidTuningProvider *provider;
	const char *name;
	int priority;
	uint32_t order;
	bool in_use;
};

ProviderSlot g_slots[kMaxProviders] = {};
size_t g_provider_count = 0U;
uint32_t g_next_order = 1U;
K_MUTEX_DEFINE(g_provider_mutex);

size_t FindSlotByProviderLocked(
	rm_test::app::services::chassis_tuning::SpeedPidTuningProvider *provider)
{
	for (size_t i = 0U; i < kMaxProviders; ++i) {
		if (g_slots[i].in_use && (g_slots[i].provider == provider)) {
			return i;
		}
	}

	return kMaxProviders;
}

size_t FindFreeSlotLocked()
{
	for (size_t i = 0U; i < kMaxProviders; ++i) {
		if (!g_slots[i].in_use) {
			return i;
		}
	}

	return kMaxProviders;
}

const ProviderSlot *FindActiveSlotLocked()
{
	const ProviderSlot *active = nullptr;
	for (size_t i = 0U; i < kMaxProviders; ++i) {
		if (!g_slots[i].in_use) {
			continue;
		}

		if ((active == nullptr) ||
		    (g_slots[i].priority > active->priority) ||
		    ((g_slots[i].priority == active->priority) && (g_slots[i].order < active->order))) {
			active = &g_slots[i];
		}
	}

	return active;
}

}  // namespace

namespace rm_test::app::services::chassis_tuning {

int RegisterProvider(SpeedPidTuningProvider *provider)
{
	return RegisterProvider(provider, "provider", 0);
}

int RegisterProvider(SpeedPidTuningProvider *provider, const char *name, int priority)
{
	if (provider == nullptr) {
		return -EINVAL;
	}

	if (name == nullptr) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&g_provider_mutex, K_FOREVER);
	size_t slot = FindSlotByProviderLocked(provider);
	if (slot == kMaxProviders) {
		slot = FindFreeSlotLocked();
		if (slot == kMaxProviders) {
			k_mutex_unlock(&g_provider_mutex);
			return -ENOMEM;
		}

		g_slots[slot] = {
			provider,
			name,
			priority,
			g_next_order++,
			true,
		};
		++g_provider_count;
	} else {
		g_slots[slot].name = name;
		g_slots[slot].priority = priority;
	}
	k_mutex_unlock(&g_provider_mutex);

	return 0;
}

int UnregisterProvider(SpeedPidTuningProvider *provider)
{
	if (provider == nullptr) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&g_provider_mutex, K_FOREVER);
	const size_t slot = FindSlotByProviderLocked(provider);
	if (slot == kMaxProviders) {
		k_mutex_unlock(&g_provider_mutex);
		return -ENOENT;
	}

	g_slots[slot] = {};
	if (g_provider_count > 0U) {
		--g_provider_count;
	}
	k_mutex_unlock(&g_provider_mutex);
	return 0;
}

bool HasProvider()
{
	return ProviderCount() > 0U;
}

size_t ProviderCount()
{
	(void)k_mutex_lock(&g_provider_mutex, K_FOREVER);
	const size_t count = g_provider_count;
	k_mutex_unlock(&g_provider_mutex);
	return count;
}

int GetProviderStatus(const char **active_name, int *active_priority, size_t *provider_count)
{
	if ((active_name == nullptr) || (active_priority == nullptr) || (provider_count == nullptr)) {
		return -EINVAL;
	}

	(void)k_mutex_lock(&g_provider_mutex, K_FOREVER);
	*provider_count = g_provider_count;
	const ProviderSlot *active = FindActiveSlotLocked();
	if (active == nullptr) {
		*active_name = nullptr;
		*active_priority = 0;
		k_mutex_unlock(&g_provider_mutex);
		return -ENOENT;
	}

	*active_name = active->name;
	*active_priority = active->priority;
	k_mutex_unlock(&g_provider_mutex);
	return 0;
}

int SetSpeedPidTuning(float kp, float ki, float kd, float i_limit, float out_limit)
{
	SpeedPidTuningProvider *provider = nullptr;
	(void)k_mutex_lock(&g_provider_mutex, K_FOREVER);
	const ProviderSlot *active = FindActiveSlotLocked();
	if (active != nullptr) {
		provider = active->provider;
	}
	k_mutex_unlock(&g_provider_mutex);

	if (provider == nullptr) {
		return -EAGAIN;
	}

	return provider->SetSpeedPidTuning(kp, ki, kd, i_limit, out_limit);
}

int GetSpeedPidTuning(float *kp, float *ki, float *kd, float *i_limit, float *out_limit)
{
	SpeedPidTuningProvider *provider = nullptr;
	(void)k_mutex_lock(&g_provider_mutex, K_FOREVER);
	const ProviderSlot *active = FindActiveSlotLocked();
	if (active != nullptr) {
		provider = active->provider;
	}
	k_mutex_unlock(&g_provider_mutex);

	if (provider == nullptr) {
		return -EAGAIN;
	}

	return provider->GetSpeedPidTuning(kp, ki, kd, i_limit, out_limit);
}

int ResetSpeedPidIntegrator()
{
	SpeedPidTuningProvider *provider = nullptr;
	(void)k_mutex_lock(&g_provider_mutex, K_FOREVER);
	const ProviderSlot *active = FindActiveSlotLocked();
	if (active != nullptr) {
		provider = active->provider;
	}
	k_mutex_unlock(&g_provider_mutex);

	if (provider == nullptr) {
		return -EAGAIN;
	}

	return provider->ResetSpeedPidIntegrator();
}

}  // namespace rm_test::app::services::chassis_tuning
