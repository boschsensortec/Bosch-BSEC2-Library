Import('env')
from os.path import join, realpath, exists

# We need the equivalent of build.mcu in Arduino board definitions. 
# For ESP this is BOARD_MCU
cpu = env.get("BOARD_MCU")

#All ESP32 except the original and S-series use riscv cores. Can use ESP32-C3 build.
if len(cpu) > 5 and cpu[:5] == "esp32" and cpu[5] != "s":
    cpu = "esp32c3"

# For ARM cores, BOARD_MCU is the chip (e.g. rp2040), not the cpu (e.g. cortex-m0plus).
# To find the correct binary, we check the linkflags.
linkflags = env.get("LINKFLAGS", [])
for flag in linkflags:
    if flag.startswith("-mcpu="):
        prefix, divider, cpu = flag.partition("=")
        continue

path = realpath(join("src", cpu))
if exists(path):
    env.Append(
        LIBPATH=[realpath(join("src", cpu))],
        LIBS=["algobsec"]
    )
else:
    print(f"BSEC2 is not supported for CPU '{cpu}', path '{path}' doesn't exist")
    exit(1)
