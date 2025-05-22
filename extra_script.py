Import('env')
from os.path import join, realpath, exists

# We need the equivalent of build.mcu in Arduino board definitions. 
# For ESP this is BOARD_MCU
cpu = env.get("BOARD_MCU")
fpu = ""
floatabi = ""

# For ESP32, all variants except original ESP32 and "S" series are riscv
# The 'esp32c3' target can be used on all riscv ESP32 variants.
if cpu.startswith("esp32") and not (cpu.startswith("esp32s") or cpu == "esp32"):
    cpu = "esp32c3"

# For ARM cores, BOARD_MCU is the chip (e.g. rp2040), not the cpu (e.g. cortex-m0plus).
# To find the correct binary, we check the linkflags.
linkflags = env.get("LINKFLAGS", [])
for flag in linkflags:
    if flag.startswith("-mcpu="):
        prefix, divider, cpu = flag.partition("=")
        continue
    elif flag.startswith("-mfpu="):
        prefix, divider, fpu = flag.partition("=")
        continue
    elif flag.startswith("-mfloat-abi="):
        prefix, divider, floatabi = flag.partition("=")
        continue

# For M4 / M33 -- use hardfloat binaries when appropriate
if cpu in ['cortex-m4', 'cortex-m33'] and floatabi == "hard":
    path = realpath(join("src", cpu, f"{fpu}-{floatabi}"))
else:
    path = realpath(join("src", cpu))

if exists(path):
    env.Append(
        LIBPATH=[path],
        LIBS=["algobsec"]
    )
else:
    print(f"BSEC2 is not supported for CPU '{cpu} {floatabi}', path '{path}' doesn't exist")
    exit(1)
