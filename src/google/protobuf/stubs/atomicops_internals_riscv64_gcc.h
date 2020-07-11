// Protocol Buffers - Google's data interchange format
// Copyright 2012 Google Inc.  All rights reserved.
// https://developers.google.com/protocol-buffers/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// This file is an internal atomic implementation, use atomicops.h instead.

#ifndef GOOGLE_PROTOBUF_ATOMICOPS_INTERNALS_RISCV_GCC_H_
#define GOOGLE_PROTOBUF_ATOMICOPS_INTERNALS_RISCV_GCC_H_

namespace google {
namespace protobuf {
namespace internal {

// This struct is not part of the public API of this module; clients may not
// use it.

#define ATOMICOPS_COMPILER_BARRIER() __asm__ __volatile__("" : : : "memory")

// 32-bit low-level operations on any platform.

inline  Atomic32 NoBarrier_CompareAndSwap(volatile Atomic32* ptr,
                                         Atomic32 old_value,
                                         Atomic32 new_value) {
  // C code:
  // prev = *ptr
  // if (*ptr == old_value) {
  //   *ptr = new_value;
  // }
  // return prev;
  //
  Atomic32 prev = 0;
  Atomic32 temp = 0;
  __asm__ __volatile__ (
    "0:\n\t"
    "lr.w %0, %5\n\t"       // prev = *ptr
    "bne %0, %3, 1f\n\t"       // if prev != old_value, goto 1:
    "sc.w %2, %4, %1\n\t"   // prev == old_value, replace *ptr with new value
    "bnez %2, 0b\n\t"       // result == 0, go out; else go to 0 and retry.
    "1:\n\t"
    : "=&r"(prev), "=m" (*ptr), "=&r" (temp)
    : "r" (old_value), "r" (new_value), "m" (*ptr)
    : "memory"
  );
  return prev;
}

inline  Atomic32 NoBarrier_AtomicExchange(volatile Atomic32* ptr,
                                         Atomic32 new_value) {
  // swap ptr and new value
  // prev = *ptr
  // *ptr = new_value
  // return prev;
  Atomic32 prev = 0;
  Atomic32 temp = 0;
  __asm__ __volatile__ (
    "0:\n\t"
    "lr.w %0, %4\n\t"       // prev = *ptr
    "sc.w %2, %3, %1\n\t"   // prev == old_value, replace *ptr with new value
    "bnez %2, 0b\n\t"
    "1:\n\t"
    : "=&r"(prev), "=m" (*ptr), "=&r" (temp)
    : "r" (new_value), "m" (*ptr)
    : "memory"
  );
  return prev;  // Now it's the previous value.
}

inline  Atomic32 NoBarrier_AtomicIncrement(volatile Atomic32* ptr,
                                          Atomic32 increment) {
  Atomic32 prev =0;
  __asm__ __volatile__ (
    "amoadd.w %0, %2, %1\n\t"
    "add %0, %0, %2"
    : "=&r"(prev), "=m" (*ptr)
    : "r" (increment)
    : "memory"
  );

  return prev;
}

inline  Atomic32 Barrier_AtomicIncrement(volatile Atomic32* ptr,
                                        Atomic32 increment) {
  ATOMICOPS_COMPILER_BARRIER();
  Atomic32 res = NoBarrier_AtomicIncrement (ptr, increment);
  ATOMICOPS_COMPILER_BARRIER();
  return res;
}

inline  Atomic32 Acquire_CompareAndSwap(volatile Atomic32* ptr,
                                       Atomic32 old_value,
                                       Atomic32 new_value) {
  ATOMICOPS_COMPILER_BARRIER();
  Atomic32 x = NoBarrier_CompareAndSwap(ptr, old_value, new_value);
  ATOMICOPS_COMPILER_BARRIER();
  return x;
}

inline  Atomic32 Release_CompareAndSwap(volatile Atomic32* ptr,
                                       Atomic32 old_value,
                                       Atomic32 new_value) {
  ATOMICOPS_COMPILER_BARRIER();
  Atomic32 res = NoBarrier_CompareAndSwap(ptr, old_value, new_value);
  ATOMICOPS_COMPILER_BARRIER();
  return res;
}
inline  void NoBarrier_Store(volatile Atomic32* ptr, Atomic32 value) {
  *ptr = value;
}


// 64-bit implementations of memory barrier can be simpler, because it
// "mfence" is guaranteed to exist.

inline  void Acquire_Store(volatile Atomic32* ptr, Atomic32 value) {
  *ptr = value;
  ATOMICOPS_COMPILER_BARRIER();
}


inline  void Release_Store(volatile Atomic32* ptr, Atomic32 value) {
  ATOMICOPS_COMPILER_BARRIER();
  *ptr = value;  // An x86 store acts as a release barrier.
  // See comments in Atomic64 version of Release_Store(), below.
}

inline  Atomic32 NoBarrier_Load(volatile const Atomic32* ptr) {
  return *ptr;
}

inline  Atomic32 Acquire_Load(volatile const Atomic32* ptr) {
  Atomic32 value = *ptr;  // An x86 load acts as a acquire barrier.
  // See comments in Atomic64 version of Release_Store(), below.
  ATOMICOPS_COMPILER_BARRIER();
  return value;
}

inline  Atomic32 Release_Load(volatile const Atomic32* ptr) {
  ATOMICOPS_COMPILER_BARRIER();
  return *ptr;
}

#if (__riscv_xlen == 64)

// 64-bit low-level operations on 64-bit platform.

inline  Atomic64 NoBarrier_CompareAndSwap(volatile Atomic64* ptr,
                                         Atomic64 old_value,
                                         Atomic64 new_value) {
  Atomic64 prev = 0;
  Atomic64 temp = 0;
  __asm__ __volatile__ (
    "0:\n\t"
    "lr.d %0, %5\n\t"       // prev = *ptr
    "bne %0, %3, 1f\n\t"       // if prev != old_value, goto 1:
    "sc.d %2, %4, %1\n\t"   // prev == old_value, replace *ptr with new value
    "bnez %2, 0b\n\t"       // result == 0, go out; else go to 0 and retry.
    "1:\n\t"
    : "=&r"(prev), "=m" (*ptr), "=&r" (temp)
    : "r" (old_value), "r" (new_value), "m" (*ptr)
    : "memory"
  );
  return prev;
}

inline  Atomic64 NoBarrier_AtomicExchange(volatile Atomic64* ptr,
                                         Atomic64 new_value) {
  Atomic64 prev = 0;
  Atomic64 temp = 0;
  __asm__ __volatile__ (
    "0:\n\t"
    "lr.d %0, %4\n\t"       // prev = *ptr
    "sc.d %2, %3, %1\n\t"   // prev == old_value, replace *ptr with new value
    "bnez %2, 0b\n\t"
    : "=&r"(prev), "=m" (*ptr), "=&r" (temp)
    : "r" (new_value), "m" (*ptr)
    : "memory"
  );
  return prev;  // Now it's the previous value.
}

inline  Atomic64 NoBarrier_AtomicIncrement(volatile Atomic64* ptr,
                                          Atomic64 increment) {
  Atomic64 prev =0;
  __asm__ __volatile__ (
    "amoadd.d %0, %2, %1\n\t"
    "add %0, %0, %2"
    : "=&r"(prev), "=m" (*ptr)
    : "r" (increment)
    : "memory"
  );

  return prev;
}

inline  Atomic64 Barrier_AtomicIncrement(volatile Atomic64* ptr,
                                        Atomic64 increment) {
  ATOMICOPS_COMPILER_BARRIER();
  Atomic64 res = NoBarrier_AtomicIncrement (ptr, increment);
  ATOMICOPS_COMPILER_BARRIER();
  return res;
}

inline  void NoBarrier_Store(volatile Atomic64* ptr, Atomic64 value) {
  *ptr = value;
}

inline  void Acquire_Store(volatile Atomic64* ptr, Atomic64 value) {
  *ptr = value;
  ATOMICOPS_COMPILER_BARRIER();
}

inline  void Release_Store(volatile Atomic64* ptr, Atomic64 value) {
  ATOMICOPS_COMPILER_BARRIER();
  *ptr = value;
}

inline  Atomic64 NoBarrier_Load(volatile const Atomic64* ptr) {
  return *ptr;
}

inline  Atomic64 Acquire_Load(volatile const Atomic64* ptr) {
  Atomic64 value = *ptr;
  ATOMICOPS_COMPILER_BARRIER();
  return value;
}

inline  Atomic64 Release_Load(volatile const Atomic64* ptr) {
  ATOMICOPS_COMPILER_BARRIER();
  return *ptr;
}

inline  Atomic64 Acquire_CompareAndSwap(volatile Atomic64* ptr,
                                       Atomic64 old_value,
                                       Atomic64 new_value) {
  Atomic64 res = NoBarrier_CompareAndSwap (ptr, old_value, new_value);
  ATOMICOPS_COMPILER_BARRIER();
  return res;
}

inline Atomic64 Release_CompareAndSwap(volatile Atomic64* ptr,
                                       Atomic64 old_value,
                                       Atomic64 new_value) {
  ATOMICOPS_COMPILER_BARRIER();
  return NoBarrier_CompareAndSwap(ptr, old_value, new_value);
}

#endif  // (__riscv_xlen == 64)

}  // namespace internal
}  // namespace protobuf
}  // namespace google

#undef ATOMICOPS_COMPILER_BARRIER

#endif  // GOOGLE_PROTOBUF_ATOMICOPS_INTERNALS_RISCV_GCC_H_
