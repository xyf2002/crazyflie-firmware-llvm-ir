set -e

make -j32


stem="sensors_bmi088_bmp388"
obj_file_name="${stem}.o"
c_file_name="${stem}.c"

obj_file=$(find ./build -type f -name "${obj_file_name}")
c_file=$(find ./src -type f -name "${c_file_name}")

flags="-Wp,-MD,build/src/hal/src/.${obj_file_name}.d \
  -Isrc/hal/src -D__firmware__ -fno-exceptions \
  -Qunused-arguments -Wno-unknown-warning-option \
  -Wall -Wmissing-braces -fno-strict-aliasing \
  -ffunction-sections -fdata-sections -Wdouble-promotion \
  -std=gnu11 -DCRAZYFLIE_FW -Ivendor/CMSIS/CMSIS/Core/Include \
  -Ivendor/CMSIS/CMSIS/DSP/Include -Ivendor/libdw1000/inc \
  -Ivendor/FreeRTOS/include -Ivendor/FreeRTOS/portable/GCC/ARM_CM4F \
  -Isrc/config -Isrc/platform/interface -Isrc/deck/interface \
  -Isrc/deck/drivers/interface -Isrc/drivers/interface \
  -Isrc/drivers/bosch/interface -Isrc/drivers/esp32/interface \
  -Isrc/hal/interface -Isrc/modules/interface \
  -Isrc/modules/interface/kalman_core -Isrc/modules/interface/lighthouse \
  -Isrc/modules/interface/cpx -Isrc/modules/interface/p2pDTR \
  -Isrc/utils/interface -Isrc/utils/interface/kve \
  -Isrc/utils/interface/lighthouse -Isrc/utils/interface/tdoa \
  -Isrc/lib/FatFS -Isrc/lib/CMSIS/STM32F4xx/Include \
  -Isrc/lib/STM32_USB_Device_Library/Core/inc \
  -Isrc/lib/STM32_USB_OTG_Driver/inc -Isrc/lib/STM32F4xx_StdPeriph_Driver/inc \
  -Isrc/lib/vl53l1 -Isrc/lib/vl53l1/core/inc -Ibuild/include/generated \
  -I/usr/arm-linux-gnueabihf/include -I/usr/arm-linux-gnueabi/include \
  -fno-delete-null-pointer-checks --param=allow-store-data-races=0 \
  -Wno-unused-variable -Wno-format-invalid-specifier -Wno-gnu \
  -Wno-tautological-compare -mno-global-merge -fomit-frame-pointer \
  -Wno-pointer-sign -fno-strict-overflow -DCC_HAVE_ASM_GOTO \
  -Wno-initializer-overrides -Wno-unused-value -Wno-format \
  -Wno-unknown-warning-option -Wno-sign-compare \
  -Wno-format-zero-length -Wno-uninitialized \
  --rtlib=compiler-rt --rtlib=libgcc --stdlib=libc++ \
  --stdlib=libstdc++ -fuse-ld=lld --target=arm-none-eabi \
  -fshort-enums -v -fno-verbose-asm -D__GCC_HAVE_DWARF2_CFI_ASM=1 \
  -march=armv7e-m+fp -mthumb -mcpu=cortex-m4 -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 -specs=nosys.specs -specs=nano.specs \
  -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 \
  -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow \
  -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 \
  -DUSE_STDPERIPH_DRIVER --sysroot=/usr/lib/arm-none-eabi"

opt_config=${OPT_CFG:-none}

if [ -e "tmp.ll" ]; then
  rm "tmp.ll"
fi

if [ -e "${stem}.ll" ]; then
  rm "${stem}.ll"
fi

if [ -e "${stem}_output.ll" ]; then
  rm "${stem}_output.ll"
fi

if [ -z "${obj_file}" ]; then
  echo "no such objective file"
else
  echo "deleting objective file ${obj_file} ......"
  rm "${obj_file}"
fi

if [ -z "${c_file}" ]; then
  echo "no such c file"
  exit 1
fi

echo "compling with ${c_file}"
cmd="clang -g -O0 -Xclang -disable-O0-optnone -emit-llvm -S ${flags} -o tmp.ll -c ${c_file}"
echo "${cmd}"
bash -c "${cmd}"
cmd="opt tmp.ll --mem2reg --instsimplify -S -o ${stem}.ll"
echo "${cmd}"
bash -c "${cmd}"

if [ "${opt_config}" = "none" ]; then
  echo "without compiler optimization"
  cmd="cp ${stem}.ll ${stem}_output.ll"
  echo "${cmd}"
  bash -c "${cmd}"
elif [ "${opt_config}" = "opt" ]; then
  echo "optimize by the CoSense Compiler"
  # change to your compiler path
  compiler_path="/home/xyf/Noisy-lang-compiler"
  current_path=$(pwd)
  cmd="cp ${stem}.ll ${compiler_path}/applications/newton/llvm-ir/${stem}.ll"
  echo "${cmd}"
  bash -c "${cmd}"
  cmd="cd ${compiler_path}/src/newton && ./newton-linux-EN \
  --llvm-ir=../../applications/newton/llvm-ir/${stem}.ll \
  --llvm-ir-liveness-check ../../applications/newton/sensors/BMX055.nt"
  echo "${cmd}"
  bash -c "${cmd}"
  cmd="llvm-dis ${compiler_path}/applications/newton/llvm-ir/${stem}_output.bc \
  -o ${compiler_path}/applications/newton/llvm-ir/${stem}_output.ll"
  echo "${cmd}"
  bash -c "${cmd}"
  cmd="cd ${current_path} && cp ${compiler_path}/applications/newton/llvm-ir/${stem}_output.ll ${stem}_output.ll"
  echo "${cmd}"
  bash -c "${cmd}"
fi
cmd="opt ${stem}_output.ll -o ${c_file}.bc"
echo "${cmd}"
bash -c "${cmd}"
#cmd="clang ${flags} -o ${obj_file} -c ${c_file}.bc"
cmd="clang ${flags} -O3 -o ${obj_file} -c ${c_file}.bc"
echo "${cmd}"
bash -c "${cmd}"

make -j32
