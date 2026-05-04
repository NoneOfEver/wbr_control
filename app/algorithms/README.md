# Algorithms

This directory mirrors the algorithm layer from the legacy FreeRTOS project.
Migrate implementation here gradually, keeping algorithm code independent from
Zephyr device details whenever possible.

Current cleanup status:
- Mahony AHRS canonical implementation: `app/algorithms/MahonyAHRS.{h,cpp}`.
- Placeholder duplicates under `app/algorithms/orientation/` have been removed.

Audit guard:
- Run `bash applications/rm_test/tools/algorithms_dedupe_audit.sh` to check duplicate
	filenames, exact duplicate content, and leftover placeholder markers.
