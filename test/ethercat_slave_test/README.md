# HPM5E EtherCAT slave bring-up

This directory now contains a buildable SSC 5.13 FreeRun slave for the
`dust-hpm5e3y` board.  It is the first-link bring-up image, not the final robot
process-data image:

- Vendor ID: `0x00000000` (experimental only)
- Product code: `0x57425231`
- Revision: `0x00010000`
- CoE mailbox: 128 bytes in each direction
- SM2/RxPDO: one 32-bit `OutputCounter` at object `0x7010`
- SM3/TxPDO: one 32-bit `InputCounter` at object `0x6000`
- FreeRun with AL-event interrupt; DC/SYNC0 is disabled
- the generated 2048-byte SII image is served from RAM

The generated files retain their original SSC licence headers and must only be
used under the licence that accompanied SSC Tool.

## Build and flash

From the `wbr_control` directory:

```sh
CCACHE_DISABLE=1 /Users/panpoming/Documents/zephyr_projects/.venv/bin/west \
  build -p always -b dust-hpm5e3y \
  -d build-hpm5e3y-ethercat-ssc test/ethercat_slave_test

/Users/panpoming/Documents/zephyr_projects/.venv/bin/west \
  flash -d build-hpm5e3y-ethercat-ssc
```

The firmware files are generated as:

- `build-hpm5e3y-ethercat-ssc/zephyr/zephyr.elf`
- `build-hpm5e3y-ethercat-ssc/zephyr/zephyr.bin`
- `build-hpm5e3y-ethercat-ssc/zephyr/zephyr.hex`

RTT prints `WBR HPM5E SSC FreeRun slave ready` after the ESC and SSC have
started.

## First IgH master check

Connect the master's EtherCAT port directly to one port of the slave, power the
board, and run on the NUC:

```sh
sudo systemctl restart ethercat.service
ethercat master
ethercat slaves
ethercat slaves -v
ethercat pdos
```

The slave should progress through `INIT`, `PREOP`, `SAFEOP`, and `OP`.  For an
explicit request:

```sh
sudo ethercat states OP
ethercat slaves
```

If it does not reach OP, preserve these diagnostics before changing the SII or
PDO configuration:

```sh
ethercat slaves -v
ethercat pdos
ethercat sii_read -p 0 > hpm5e-sii.bin
sudo journalctl -k -b --no-pager | grep -i ethercat | tail -n 200
```

The matching ESI is `esi/WBR_HPM5E_FreeRun.xml`. IgH does not require the ESI
for basic discovery, but the identity and PDO layout reported by the slave must
match it.

## Production integration status

The repository contains all license-independent parts of the slave test:

- HPM5E ESC device driver and interrupt callbacks;
- 72-byte SM2 / 96-byte SM3 process-data ABI;
- benchmark echo, CRC and timing fields;
- MPC command validation, force limiting and local fallback;
- an alignment-safe SSC process-data bridge;
- host-native and Zephyr protocol tests.

The sections below describe the later 72-byte/96-byte robot integration. They
do not describe the small counter PDOs in the current bring-up firmware. Before
the production hardware integration build:

1. obtain SSC Tool under the applicable Beckhoff/ETG license;
2. create the objects and PDOs listed in
   `docs/ethercat/WBR_ECAT_OBJECT_DICTIONARY_V1.csv`;
3. select two RxPDOs on SM2 and two TxPDOs on SM3 in the documented order;
4. generate SSC sources, ESI XML and `eeprom.h` using the final Vendor ID,
   Product Code and Revision;
5. copy `ssc_application_hooks.cpp.template` into the generated application and
   replace its generated application mapping hooks;
6. use the provided `modules::EthercatChassisPort`; wire an explicit local UI
   action to `SetOperatorEnabled(true/false)` (it defaults false after boot);
7. register `PDI_Isr`, `Sync0_Isr`, `Sync1_Isr` and reset callbacks with
   `hpm_ethercat_set_callbacks()` before starting the ESC;
8. register EEPROM callbacks before `hpm_ethercat_start()`.

## Flash-backed SII setup

`EthercatSiiStore` keeps the live SII in RAM so ESC reads/writes never erase
flash in the realtime path. `EthercatSiiFlashBackend` uses the board's
`ethercat_eeprom_partition` as two power-fail-safe slots. Each slot has a CRC,
generation and final commit marker.

Application setup outline:

```cpp
platform::storage::EthercatSiiFlashBackend flash;
flash.Open();
protocols::ethercat::EthercatSiiStore sii(flash);
sii.Load(generated_eeprom_bytes, generated_eeprom_size);

hpm_ethercat_eeprom_ops ops{};
ops.read = protocols::ethercat::EthercatSiiStore::ReadCallback;
ops.write = protocols::ethercat::EthercatSiiStore::WriteCallback;
ops.reload = protocols::ethercat::EthercatSiiStore::ReloadCallback;
ops.user_data = &sii;
hpm_ethercat_set_eeprom_ops(esc, &ops);
```

Call `sii.Commit(true)` only from INIT/PREOP maintenance logic after outputs are
disabled. Never call it from OP, SYNC0, PDI ISR or the chassis control thread.
`Commit(false)` deliberately returns `-EPERM`, making the state check explicit.

The first firmware should use FreeRun and Shadow mode. Enable DC/SYNC0 only
after FreeRun reaches OP with WKC 3 and stable sequence echo. Active mode must
remain separately gated in the chassis state machine.

The concrete chassis bridge is already connected to the 1 kHz chassis loop.
It publishes a coherent IMU/leg/support-force sample, records Shadow commands,
and substitutes Active left/right support feed-forward at the existing VMC
boundary. Local leg-length PD/I and all joint torque limits remain active. An
Active command is applied only while the explicit operator gate is true and
the chassis is in an IMU-fresh, feedback-valid Balance state. Either gate
dropping atomically restores the local roll compensation path.

The template assumes SSC 5.13-style hook names used by the HPM SDK examples.
If a later SSC version changes a signature, keep the process-data sizes and
call order unchanged and adjust only the hook declarations.
