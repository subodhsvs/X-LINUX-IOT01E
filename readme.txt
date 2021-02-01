How to build on Ubuntu 118.04 (and above) for STM32MP157C-DK2

1. Install cmake for Linux (use below command)

    $ sudo apt-get install cmake

2. Install cross-compilation toolchain & set environment to Enable the SDK (follow below link)
  https://wiki.st.com/stm32mpu/wiki/Getting_started/STM32MP1_boards/STM32MP157C-DK2/Develop_on_Arm%C2%AE_Cortex%C2%AE-A7/Install_the_SDK

3. Enter the build directory in the Demo application source code
  $ cd "Project-Folder"/linux_demo/build

4. RUN below command to generate makefiles from CMakeLists
  $ cmake ..
OUTPUT:
-- Toolchain file defaulted to '/home/subodh/STM32MPU_workspace/STM32MP15-Ecosystem-v2.1.0/Developer-Package/SDK/sysroots/x86_64-ostl_sdk-linux/usr/share/cmake/OEToolchainConfig.cmake'
-- The C compiler identification is GNU 9.3.0
-- The CXX compiler identification is GNU 9.3.0
-- Check for working C compiler: /home/subodh/STM32MPU_workspace/STM32MP15-Ecosystem-v2.1.0/Developer-Package/SDK/sysroots/x86_64-ostl_sdk-linux/usr/bin/arm-ostl-linux-gnueabi/arm-ostl-linux-gnueabi-gcc
-- Check for working C compiler: /home/subodh/STM32MPU_workspace/STM32MP15-Ecosystem-v2.1.0/Developer-Package/SDK/sysroots/x86_64-ostl_sdk-linux/usr/bin/arm-ostl-linux-gnueabi/arm-ostl-linux-gnueabi-gcc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /home/subodh/STM32MPU_workspace/STM32MP15-Ecosystem-v2.1.0/Developer-Package/SDK/sysroots/x86_64-ostl_sdk-linux/usr/bin/arm-ostl-linux-gnueabi/arm-ostl-linux-gnueabi-g++
-- Check for working CXX compiler: /home/subodh/STM32MPU_workspace/STM32MP15-Ecosystem-v2.1.0/Developer-Package/SDK/sysroots/x86_64-ostl_sdk-linux/usr/bin/arm-ostl-linux-gnueabi/arm-ostl-linux-gnueabi-g++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
GNU
-Wl,-O1 -Wl,--hash-style=gnu -Wl,--as-needed
-- Looking for pthread.h
-- Looking for pthread.h - found
-- Performing Test CMAKE_HAVE_LIBC_PTHREAD
-- Performing Test CMAKE_HAVE_LIBC_PTHREAD - Failed
-- Check if compiler accepts -pthread
-- Check if compiler accepts -pthread - yes
-- Found Threads: TRUE
-- Configuring done
-- Generating done
-- Build files have been written to: /home/subodh/My_Projects/X-LINUX-IOT1A1_Project/X-LINUX-IOT01A1/linux_demo/build

4. RUN below command to build
  $ make
OUTPUT:
Scanning dependencies of target spirit_application_binary
[  3%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT1_Util.c.o
[  7%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Aes.c.o
[ 11%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Calibration.c.o
[ 14%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Commands.c.o
[ 18%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Csma.c.o
[ 22%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_DirectRF.c.o
[ 25%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_General.c.o
[ 29%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Gpio.c.o
[ 33%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Irq.c.o
[ 37%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_LinearFifo.c.o
[ 40%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Management.c.o
[ 44%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_PktBasic.c.o
[ 48%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_PktCommon.c.o
[ 51%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_PktMbus.c.o
[ 55%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_PktStack.c.o
[ 59%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Qi.c.o
[ 62%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Radio.c.o
[ 66%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Timer.c.o
[ 70%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/SPIRIT_Types.c.o
[ 74%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/main.c.o
[ 77%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/radio_appli.c.o
[ 81%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/radio_gpio.c.o
[ 85%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/Src/radio_spi.c.o
[ 88%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/__/platform/Src/pltf_gpio.c.o
[ 92%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/__/platform/Src/pltf_spi.c.o
[ 96%] Building C object spirit_application/CMakeFiles/spirit_application_binary.dir/__/platform/Src/pltf_timer.c.o
[100%] Linking C executable spirit_application_binary
[100%] Built target spirit_application_binary


-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


How to RUN the demo application on STM32MP157C-DK2 board.

1. Follow above mentioned steps to build the Package.

2. Copy the stm32mp157c-dk2.dtb file to /boot folder of DK2 board and run below commands -
   Board$> /sbin/depmod -a
   Board$> sync
   Board$> reboot

3. Copy the below mentioned executable file to /usr/local folder on STM32MP157C-DK2 board.
    linux_demo/build/spirit_application/spirit_application_binary

4. On STM32MP157C-DK2 command line, execute below commands -

 $ cd /usr/local
 $ ./spirit_application_binary
