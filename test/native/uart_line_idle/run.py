#!/usr/bin/env python3
"""Compile current driver functions against a deterministic DMA/callback backend."""
import pathlib
import re
import subprocess
import tempfile
import os

here = pathlib.Path(__file__).resolve().parent
# The repository is normally a sibling of hpm_support in the Zephyr workspace.
source = here.parents[3] / 'hpm_support/drivers/serial/uart_hpmicro.c'
text = source.read_text()

def function(name):
    match = re.search(r'^(?:static )?(?:void|int) ' + name + r'\([^;]*?\)\n\{', text, re.M)
    if not match:
        raise RuntimeError('Driver function not found: ' + name)
    start = match.start()
    depth = 1
    position = match.end()
    # These selected function bodies have no braces in comments/string literals.
    while depth:
        depth += (text[position] == '{') - (text[position] == '}')
        position += 1
    return text[start:position]

functions = function('async_evt_rx_rdy') + '\n' + function('uart_hpm_rx_line_idle_flush')
# Native CPUs cannot execute a RISC-V MMIO fence. This does not model bus/DMA drain.
functions = functions.replace('__asm__ volatile ("fence iorw, iorw" ::: "memory");',
                              'atomic_thread_fence(memory_order_seq_cst);')
with tempfile.TemporaryDirectory(prefix='uart-line-idle-test-') as directory:
    directory = pathlib.Path(directory)
    (directory / 'driver_functions.inc').write_text(functions)
    executable = directory / 'contracts'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I', str(directory), str(here / 'contracts.c'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
