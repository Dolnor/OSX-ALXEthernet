/* alx_defs.h -- ALX defines for Linux to Mac compatibility.
 *
 * Copyright (c) 2012 Hayley <tranquil.reticence@gmail.com>.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Driver for Atheros(R) AR81(31/32/51/52/61/62/71/72) PCI-E ethernet.
 *
 */

#ifndef __ALX_DEFS_H__
#define __ALX_DEFS_H__

/******************************************************************************/
#pragma mark -
#pragma mark Debugging
#pragma mark -
/******************************************************************************/

#if defined(DEBUG)

// Levels 1 to 6 are used, in order of verbosity.
// 5 or above will usually produce a LOT of log output for every packet.
#define DEBUGLEVEL 3
#define DbgPrint(dbglvl,format,args...)                                        \
do                                                                             \
{                                                                              \
  if (dbglvl<=DEBUGLEVEL)                                                      \
  {                                                                            \
    IOLog("[ALXEthernet] " format, ##args);                             \
  }                                                                            \
} while (0)

#define dev_dbg(dev,format,args...)                                            \
    IOLog("[ALXEthernet] " format, ##args);

#else // Disable debugging.

#define DbgPrint(...)

#endif // Disable debugging.

#define ErrPrint(format,args...)                                               \
    IOLog("[ALXEthernet] Error: " format, ##args)


/******************************************************************************/
#pragma mark -
#pragma mark Bits and Bytes
#pragma mark -
/******************************************************************************/

#define HZ 1000 // Milliseconds.

OS_INLINE int
num_online_cpus()
{
  int cpus;
  size_t len = sizeof(cpus);
  
  // Get the number of CPUs from the system.
  sysctlbyname("hw.activecpu", &cpus, &len, NULL, 0);
  
  if (cpus < 1)
  {
    sysctlbyname("hw.physicalcpu", &cpus, &len, NULL, 0);
    if (cpus < 1)
    {
      cpus = 1;
    }
  }
  
  return cpus;
}

//------------------------------------------------------------------------------

#if defined(__LITTLE_ENDIAN__)

#define __LITTLE_ENDIAN         1234
#define __LITTLE_ENDIAN_BITFIELD

#elif defined(__BIG_ENDIAN__)

#define __BIG_ENDIAN            4321
#define __BIG_ENDIAN_BITFIELD

#endif // ENDIAN

//------------------------------------------------------------------------------

#define u8                      UInt8
#define u16                     UInt16
#define u32                     UInt32
#define u64                     UInt64

#define s8                      SInt8
#define s16                     SInt16
#define s32                     SInt32
#define s64                     SInt64

#define __be16                  SInt16
#define __be32                  SInt32
#define __be64                  SInt64

#define __le16                  SInt16
#define __le32                  SInt32
#define __le64                  SInt64

#define __u8                    UInt8
#define __u16                   UInt16
#define __u32                   UInt32
#define __u64                   UInt64

#define __s8                    SInt8
#define __s16                   SInt16
#define __s32                   SInt32
#define __s64                   SInt64

#define dma_addr_t              UInt64

//------------------------------------------------------------------------------

#define ALIGN_MASK(x, mask)     (((x) + (mask)) & ~(mask))
#define ALIGN(x, a)             ALIGN_MASK(x, (typeof(x))(a) - 1)

#define cpu_to_le16(x)          OSSwapHostToLittleInt16(x)
#define cpu_to_le32(x)          OSSwapHostToLittleInt32(x)
#define cpu_to_le64(x)          OSSwapHostToLittleInt64(x)
#define le16_to_cpu(x)          OSSwapLittleToHostInt16(x)
#define le32_to_cpu(x)          OSSwapLittleToHostInt32(x)
#define le64_to_cpu(x)          OSSwapLittleToHostInt64(x)

#define cpu_to_be16(x)          OSSwapHostToBigInt16(x)
#define cpu_to_be32(x)          OSSwapHostToBigInt32(x)
#define cpu_to_be64(x)          OSSwapHostToBigInt64(x)
#define be16_to_cpu(x)          OSSwapBigToHostInt16(x)
#define be32_to_cpu(x)          OSSwapBigToHostInt32(x)
#define be64_to_cpu(x)          OSSwapBigToHostInt64(x)

#define le16_to_cpus(x)         ((*x) = OSSwapLittleToHostInt16((*x)))
#define le32_to_cpus(x)         ((*x) = OSSwapLittleToHostInt32((*x)))
#define le64_to_cpus(x)         ((*x) = OSSwapLittleToHostInt64((*x)))

#define BITS_PER_LONG           LONG_BIT
#define BIT(nr)                 (1UL << (nr))
#define BIT_MASK(nr)            (1UL << ((nr) % BITS_PER_LONG))
#define BIT_WORD(nr)            ((nr) / BITS_PER_LONG)
#define BITS_PER_BYTE           8
#define BITS_TO_LONGS(bits)     (((bits)+BITS_PER_LONG-1)/BITS_PER_LONG)

#define ARRAY_SIZE(x)           (sizeof(x) / sizeof((x)[0]))

#define min_t(type, x, y) \
    ({ type __x = (x); type __y = (y); __x < __y ? __x: __y; })

#define max_t(type, x, y) \
    ({ type __x = (x); type __y = (y); __x > __y ? __x: __y; })

#define likely(x)               __builtin_expect(!!(x), 1)
#define unlikely(x)             __builtin_expect(!!(x), 0)

OS_INLINE int
test_bit(int nr, const volatile unsigned long *addr)
{
  return (OSAddAtomic(0, addr) & (1 << nr)) != 0;
}

OS_INLINE void
set_bit(unsigned int nr, volatile unsigned long *addr)
{
  OSTestAndSet(nr, (volatile UInt8 *)addr);
}

OS_INLINE void
clear_bit(unsigned int nr, volatile unsigned long *addr)
{
  OSTestAndClear(nr, (volatile UInt8 *)addr);
}

OS_INLINE int
test_and_clear_bit(unsigned int nr, volatile unsigned long *addr)
{
  return !OSTestAndClear(nr, (volatile UInt8 *)addr);
}

OS_INLINE int
test_and_set_bit(unsigned int nr, volatile unsigned long *addr)
{
  return OSTestAndSet(nr, (volatile UInt8 *)addr);
}

/******************************************************************************/
#pragma mark -
#pragma mark Memory Allocation
#pragma mark -
/******************************************************************************/

enum
{
  GFP_KERNEL,
  GFP_ATOMIC,
}; // For kmalloc flags compatibility; ignored.

OS_INLINE void *
kmalloc(size_t size, int flags)
{
  size_t *memory = (size_t *)IOMalloc(size + sizeof(size));
  if (memory == 0)
  {
    return NULL;
  }
  memory[0] = size;
  return ((void *)++memory);
}

OS_INLINE void *
kzalloc(size_t size, int flags)
{
  void *memory = kmalloc(size, flags);
  if (memory == NULL)
  {
    return memory;
  }
  memset(memory, 0, size);
  return memory;
}

OS_INLINE void *
kcalloc(size_t n, size_t size, int flags)
{
  return kzalloc(n * size, flags);
}

OS_INLINE void
kfree(void *p)
{
  if (p == NULL)
  {
    return;
  }
  size_t *memory = (size_t *)p;
  memory--;
  IOFree((void *)memory, *memory);
}

/******************************************************************************/
#pragma mark -
#pragma mark Read/Write Registers
#pragma mark -
/******************************************************************************/

OS_INLINE void
_OSWriteInt8(volatile void *base, uintptr_t byteOffset, uint16_t data)
{
  *(volatile uint8_t *)((uintptr_t)base + byteOffset) = data;
}

OS_INLINE uint8_t
_OSReadInt8(const volatile void *base, uintptr_t byteOffset)
{
  return *(volatile uint8_t *)((uintptr_t)base + byteOffset);
}

#define OSWriteLittleInt8(base, byteOffset, data) \
    _OSWriteInt8((base), (byteOffset), (data))
#define OSReadLittleInt8(base, byteOffset) \
    _OSReadInt8((base), (byteOffset))

//------------------------------------------------------------------------------

#define	writeb(val, reg)    OSWriteLittleInt8((reg), 0, (val))
#define	writew(val, reg)    OSWriteLittleInt16((reg), 0, (val))
#define	writel(val, reg)    OSWriteLittleInt32((reg), 0, (val))

#define	readb(reg)          OSReadLittleInt8((reg), 0)
#define	readw(reg)          OSReadLittleInt16((reg), 0)
#define	readl(reg)          OSReadLittleInt32((reg), 0)

#define wmb()               OSSynchronizeIO()

/******************************************************************************/
#pragma mark -
#pragma mark Locks and Atomics
#pragma mark -
/******************************************************************************/

#define spinlock_t              IOSimpleLock *
#define atomic_t                volatile SInt32

#define atomic_inc(v)           OSIncrementAtomic(v)
#define atomic_dec(v)           OSDecrementAtomic(v)

OS_INLINE void
atomic_set(atomic_t *v, int i)
{
  OSCompareAndSwap(*v, i, v);
}

OS_INLINE SInt32
atomic_read(atomic_t *v)
{
  return OSAddAtomic(0, v);
}

OS_INLINE int
atomic_inc_and_test(volatile SInt32 * addr)
{
  return ((OSIncrementAtomic(addr) == -1) ? 1 : 0);
}

OS_INLINE int
atomic_dec_and_test(volatile SInt32 * addr)
{
  return ((OSDecrementAtomic(addr) == 1) ? 1 : 0);
}

//------------------------------------------------------------------------------

#define spin_lock_init(slock)                           \
do                                                      \
{                                                       \
  if (*slock == NULL)                                   \
  {                                                     \
    *(slock) = IOSimpleLockAlloc();                     \
  }                                                     \
} while (0)

#define spin_lock(lock)   (IOSimpleLockLock(*(lock)))

#define spin_unlock(lock) (IOSimpleLockUnlock(*(lock)))

#define spin_lock_irqsave(lock,flags)                   \
    ((flags) = IOSimpleLockLockDisableInterrupt(*(lock)))

#define spin_trylock_irqsave(lock,flags)                \
    ((flags) = IOSimpleLockLockDisableInterrupt(*(lock)))

#define spin_unlock_irqrestore(lock,flags)              \
    (IOSimpleLockUnlockEnableInterrupt(*(lock), (flags)))

/******************************************************************************/
#pragma mark -
#pragma mark Time
#pragma mark -
/******************************************************************************/

#define usec_delay(x)           IODelay(x)
#define msec_delay(x)           IOSleep(x)
#define udelay(x)               IODelay(x)
#define mdelay(x)               IODelay(1000*(x))
#define msleep(x)               IOSleep(x)

//------------------------------------------------------------------------------

#endif // __ALX_DEFS_H__
