# SPDX-License-Identifier: Apache-2.0

# Configures binary toos as llvm binary tool set

if(DEFINED TOOLCHAIN_HOME)
  set(find_program_clang_args PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
  set(find_program_binutils_args PATHS ${TOOLCHAIN_HOME})
endif()

find_program(CMAKE_AR      llvm-ar      ${find_program_clang_args}   )
find_program(CMAKE_NM      llvm-nm      ${find_program_clang_args}   )
find_program(CMAKE_OBJDUMP riscv32-unknown-elf-objdump ${find_program_clang_args}   )
find_program(CMAKE_RANLIB  llvm-ranlib  ${find_program_clang_args}   )
find_program(CMAKE_STRIP   llvm-strip   ${find_program_clang_args}   )
find_program(CMAKE_OBJCOPY riscv32-unknown-elf-objcopy      ${find_program_binutils_args})
find_program(CMAKE_READELF riscv32-unknown-elf-readelf      ${find_program_binutils_args})

# Use the gnu binutil abstraction
include(${ZEPHYR_BASE}/cmake/bintools/llvm/target_bintools.cmake)
