/* SPDX-License-Identifier: Apache-2.0 */
#pragma once
#include <protocols/usb_bridge/bridge_protocol.hpp>
namespace gateway
{
uint32_t Session();
void PublishPeripheral(const wbr::bridge::Packet &);
// Capability bits: CAN=1, Remote UART=2, HI91 UART=4, onboard raw IMU=8, referee UART=16.
uint32_t StartPeripherals();
void AppendUartProbeStatus(wbr::bridge::Packet &);
} // namespace gateway
