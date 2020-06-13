# 3.18 CAF kernel base

Minimal kernel base with everything working.

## About phone

![phone](rosy.jpg)

##### Basic specifications:
```
Codename - rosy
SoC - Snapdragon 450
CPU - Octa-core 1.8 GHz Cortex-A53
GPU - 600 MHz Adreno 506
```

Full specifications - [GSM Arena](https://www.gsmarena.com/xiaomi_redmi_5-8768.php)

## Features & information

##### Main features:
```
* Merged linux-stable (upstream to Linux 3.18.140).
* Merged f2fs-stable (upstreamed F2FS).
* Merged latest compatible prima drivers (upstreamed Wi-Fi drivers).
* Optimized prima drivers (reduced debugging and unnecessary wakelocks).
* Clean, optimized, and minimal imports of code for rosy (based on riva-o-oss branch).
* Optimized touchscreen drivers (fixed freezing and reduced debugging).
* Reduced unnecessary debug and removed useless hard-coded debug.
* Can be compiled with GCC 9.2.1 & -O3, all code warnings are fixed.
* Compiler tuned for Cortex-A53 (CPU optimization).
* Memory optimizations.
* Full F2FS support.
* Miscellaneous config improvements that are not listed here.
```

##### Information:
```
- CAF tag kernel: LA.UM.8.6.r1-04400-89xx.0
- CAF tag prima: LA.UM.8.6.r1-04400-89xx.0
- CPU governors: [interactive], conservative, ondemand, userspace, powersave, performance.
- I/O schedulers: [cfq], noop, deadline.
- TCP algorithms: [westwood], cubic, reno.
- Compression algorithms: [lz4], lzo.
- INITRD compression: [gzip].
```

## Branches

##### master:
```
Contains all 3.18* branches merged.
```

##### 3.18-fixes:
```
Contains commits that fix various things.
```

##### 3.18-gcc-fixes:
```
Contains commits that fix various things by/for GCC.
```

##### 3.18-no-debug:
```
Contains commits that remove hard-coded debug code.
```

##### 3.18-optimizations:
```
Contains commits for optimization.
```

##### 3.18-rosy-o-oss:
```
Contains drivers and code changes for rosy.
```

##### 3.18-upstream:
```
Contains 3.18 upstream commits, backports from Linus' tree, and occasionally commits that didn't make it to Linus' tree.
```

##### LA.UM.8.6.r1-04400-89xx.0:
```
CAF tag LA.UM.8.6.r1-04400-89xx.0 (May 21, 2020; Linux 3.18.124).
```

Any other branches (if any at the moment) contain experimental stuff or stuff under testing.

## Cloning & building

For ROSY:

This kernel contains submodules. To build it successfully, you have to either clone it with `--recursive` option or do `git submodule update --init --recursive` in the kernel directory after cloning.

Make sure you are building the kernel with `rosy_defconfig`. In case you want to base on another defconfig, you can easily enable ROSY's drivers in any config using the kernel's `menuconfig`. You can find a special section with drivers for ROSY in `menuconfig > Device Drivers > Drivers for ROSY`. Do not forget to build the Wi-Fi (prima) drivers though, they are located in `menuconfig > Device Drivers > Staging drivers`.

##### Steps (normal defconfig):
```
1 - make ARCH=arm64 <defconfig>
2 - make ARCH=arm64 menuconfig
3 - make ARCH=arm64 savedefconfig
4 - mv defconfig arch/arm64/configs/menuconfig_defconfig
```

##### Steps (raw defconfig):
```
1 - make ARCH=arm64 <defconfig>
2 - make ARCH=arm64 menuconfig
3 - mv .config arch/arm64/configs/menuconfig_rawconfig
```

![rosymenu](menu.png)
