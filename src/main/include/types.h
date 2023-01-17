#pragma once

#include <stddef.h>
#include <stdint.h>

// I think roborio is only 32 bit, so no need for 64 bit integers
typedef uint8_t u8;
typedef int8_t i8;
typedef uint16_t u16;
typedef int16_t i16;
typedef uint32_t u32;
typedef int32_t i32;
typedef size_t usize;
// TODO: figure out if ptrdiff_t is actually the equivalent of isize, I think it is but documentation online is unclear for sure
typedef ptrdiff_t isize;

// type used by the code for math
typedef float num;