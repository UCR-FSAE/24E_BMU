# STM32F405 Architecture for 24E BMU

## Documentation
Documentation for this project can be found here (request edit access in BMS Discord GC): https://docs.google.com/document/d/1UyszmjpuPbkYDdnsG2s5ma4l2FgmAOwvlrdJy_fe6Pc/edit?usp=sharing

## Renode
Basic Instructions on how to use renode can be found above. Two ELF files have been provided here, along with a demo script utilizing this project's architecture. The demo script does **not** require you to download the elf files, as it pulls the files directly from github. Further renode documentation available at https://renode.readthedocs.io/en/latest/

### *Important note for pulling/cloning*
Please note that the most updated code is in the F4_BMU folder. This only contains the Core files (no auto generated code from STM32CubeIDE) therefore you have to create your project using the included ioc file, and then copy over the files that you want to update/ work on. This may be changed later depending on if there is an easier way to work with git on Eclipse. (Changes to the ioc are discussed in person and shouldn't affect user code too much.)

*Note for C++ projects:* The STM32CubeIDE auto generates main as a .c file. You need to manually change this to .cpp after generating. If you need to regenerate the code, change the suffix to .c, regenerate, and then change back to .cpp in order to prevent loss of work.

BMU-F7(old) contains an older project utilizing the F7 chip. This is not active and merely used as a reference.
