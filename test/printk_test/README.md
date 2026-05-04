# printk_test

Minimal continuous printk validation app.

## Build

From workspace root:

```bash
source .venv/bin/activate
west build -p always -b hpm6e00evk -d build-printk-test applications/rm_test/test/printk_test
```

## Flash

```bash
source .venv/bin/activate
west flash -d build-printk-test --skip-rebuild
```

## Capture log

```bash
./applications/rm_test/tools/serial_log.sh /dev/cu.usbserial-11301
```
