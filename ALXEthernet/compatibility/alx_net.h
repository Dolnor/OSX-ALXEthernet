/* alx_net.h -- ALX net-related defines for Linux to Mac compatibility.
 * Much of the checksum code (e.g. assembly) is heavily derived from
 * Linux code and is copyright the respective authors.
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

/*
 * ether_crc_le function derived from FreeBSD code.
 * Copyright (c) 1982, 1989, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef __ALX_NET_H__
#define __ALX_NET_H__

//------------------------------------------------------------------------------

#define ETH_ALEN              ETHER_ADDR_LEN  // Octets in one ethernet addr.
#define ETH_HLEN              ETHER_HDR_LEN   // Total octets in header.
#define ETH_FCS_LEN           ETHER_CRC_LEN		// Octets in the FCS.
#define VLAN_HLEN             4               // Additional VLAN bytes.

//==============================================================================
#pragma mark -
#pragma mark Ethernet Addresses
#pragma mark -
//==============================================================================

/**
 * eth_random_addr - Generate software assigned random Ethernet address
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Generate a random Ethernet address (MAC) that is not multicast
 * and has the local assigned bit set.
 */
 
#define __APPLE_API_UNSTABLE

#ifdef __APPLE_API_UNSTABLE
#include <sys/random.h>
#endif

OS_INLINE
void eth_random_addr(UInt8 *addr)
{
#ifdef __APPLE_API_UNSTABLE
  read_random(addr, ETH_ALEN);
  addr[0] &= 0xfe;		/* clear multicast bit */
  addr[0] |= 0x02;		/* set local assignment bit (IEEE802) */
#else
  addr[0] = 0x00;
  addr[1] = 0x13;
  addr[2] = 0x74;
  addr[3] = 0x00;
  addr[4] = 0x5c;
  addr[5] = 0x38;
#endif
}

//------------------------------------------------------------------------------

OS_INLINE bool
is_zero_ether_addr(const UInt8 * addr)
{
  return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}

OS_INLINE bool
is_multicast_ether_addr(const UInt8 * addr)
{
  return (0x01 & addr[0]);
}

OS_INLINE bool
is_broadcast_ether_addr(const UInt8 *addr)
{
  return (addr[0] & addr[1] & addr[2] & addr[3] & addr[4] & addr[5]) == 0xff;
}

OS_INLINE int
is_valid_ether_addr(const UInt8 * addr)
{
  /* FF:FF:FF:FF:FF:FF is a multicast address so we don't need to
   * explicitly check for it here. */
  return !is_multicast_ether_addr(addr) && !is_zero_ether_addr(addr);
}

//==============================================================================
#pragma mark -
#pragma mark Checksums
#pragma mark -
//==============================================================================

OS_INLINE UInt32
ether_crc_le(size_t length, const UInt8 *data)
{
	static const UInt32 crctab[] =
  {
		0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
		0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
		0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
		0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
	};
	size_t i;
	UInt32 crc;
  
	crc = 0xffffffff;
  
	for (i = 0; i < length; i++)
  {
		crc ^= data[i];
		crc = (crc >> 4) ^ crctab[crc & 0xf];
		crc = (crc >> 4) ^ crctab[crc & 0xf];
	}
  
	return (crc);
}

OS_INLINE UInt32
ether_crc(size_t len, const UInt8 *mc_addr)
{
  UInt32 crc32;
  UInt32 value = 0;
  int i;

  crc32 = ether_crc_le(len, mc_addr);
  for (i = 0; i < 32; i++)
    value |= (((crc32 >> i) & 1) << (31 - i));

  return value;
}

//------------------------------------------------------------------------------

OS_INLINE UInt32
csum_tcpudp_nofold(UInt32 saddr, UInt32 daddr,
    UInt16 len, UInt16 proto, UInt32 sum)
{
#if defined(__i386__) || defined(__x86_64__)

  asm("addl %1, %0	;\n"
	    "adcl %2, %0	;\n"
	    "adcl %3, %0	;\n"
	    "adcl $0, %0	;\n"
	    : "=r" (sum)
	    : "g" (daddr), "g"(saddr),
      "g" ((len + proto) << 8), "0" (sum));
  
  return sum;
    
#else // Generic

  UInt64 s = sum;
  
  s += saddr;
	s += daddr;
#if BYTE_ORDER == BIG_ENDIAN
	s += proto + len;
#else // BYTE_ORDER == LITTLE_ENDIAN
	s += (proto + len) << 8;
#endif // BYTE_ORDER == LITTLE_ENDIAN
	s += (s >> 32);
  
  return s;

#endif // Generic
}

/*
 * Fold a partial checksum.
 */
OS_INLINE UInt16
csum_fold(UInt32 csum)
{  
#if defined(__i386__) || defined(__x86_64__)

  asm("addl %1,%0\n"
	    "adcl $0xffff,%0"
	    : "=r" (csum)
	    : "r" ((UInt32)csum << 16),
      "0" ((UInt32)csum & 0xffff0000));
  
	return (UInt16)(~(UInt32)csum >> 16);
    
#else // Generic

	UInt32 sum = csum;
  
	sum = (sum & 0xffff) + (sum >> 16);
	sum = (sum & 0xffff) + (sum >> 16);
  
	return (UInt16)~sum;
  
#endif // Generic
}

/*
 * Computes the checksum of the TCP/UDP pseudo-header
 * returns a 16-bit checksum, already complemented.
 */
OS_INLINE UInt16
csum_tcpudp_magic(UInt32 saddr, UInt32 daddr, UInt16 len, UInt16 proto, UInt32 sum)
{
	return csum_fold(csum_tcpudp_nofold(saddr, daddr, len, proto, sum));
}

OS_INLINE UInt16
csum_ipv6_magic(const struct in6_addr *saddr,
                const struct in6_addr *daddr,
                UInt32 len, UInt16 proto, UInt32 csum)
{
#if defined(__i386__) || defined(__x86_64__)

	asm("addl 0(%1), %0	;\n"
	    "adcl 4(%1), %0	;\n"
	    "adcl 8(%1), %0	;\n"
	    "adcl 12(%1), %0	;\n"
	    "adcl 0(%2), %0	;\n"
	    "adcl 4(%2), %0	;\n"
	    "adcl 8(%2), %0	;\n"
	    "adcl 12(%2), %0	;\n"
	    "adcl %3, %0	;\n"
	    "adcl %4, %0	;\n"
	    "adcl $0, %0	;\n"
	    : "=&r" (csum)
	    : "r" (saddr), "r" (daddr),
      "r" (htonl(len)), "r" (htonl(proto)), "0" (csum)
	    : "memory");
  
	return csum_fold(csum);

#else // Generic
  
  int carry;
	UInt32 ulen;
	UInt32 uproto;
	UInt32 sum = csum;

	sum += saddr->s6_addr32[0];
	carry = (sum < saddr->s6_addr32[0]);
	sum += carry;

	sum += saddr->s6_addr32[1];
	carry = (sum < saddr->s6_addr32[1]);
	sum += carry;

	sum += saddr->s6_addr32[2];
	carry = (sum < saddr->s6_addr32[2]);
	sum += carry;

	sum += saddr->s6_addr32[3];
	carry = (sum < saddr->s6_addr32[3]);
	sum += carry;

	sum += daddr->s6_addr32[0];
	carry = (sum < daddr->s6_addr32[0]);
	sum += carry;

	sum += daddr->s6_addr32[1];
	carry = (sum < daddr->s6_addr32[1]);
	sum += carry;

	sum += daddr->s6_addr32[2];
	carry = (sum < daddr->s6_addr32[2]);
	sum += carry;

	sum += daddr->s6_addr32[3];
	carry = (sum < daddr->s6_addr32[3]);
	sum += carry;

	ulen = htonl(len);
	sum += ulen;
	carry = (sum < ulen);
	sum += carry;

	uproto = htonl(proto);
	sum += uproto;
	carry = (sum < uproto);
	sum += carry;

	return csum_fold(sum);

#endif // Generic
}

//==============================================================================
#pragma mark -
#pragma mark VLAN and TSO
#pragma mark -
//==============================================================================

OS_INLINE bool
vlan_tx_tag_present(mbuf_t m)
{
  UInt16 vlan = 0;
  
  if (mbuf_get_vlan_tag(m, &vlan) == ENXIO)
  {
    return false;
  }
  
  return true;
}

OS_INLINE UInt16
vlan_tx_tag_get(mbuf_t m)
{
  UInt16 vlan = 0;
  
  mbuf_get_vlan_tag(m, &vlan);
  
  return vlan;
}

#define __vlan_hwaccel_put_tag(m,vlan) mbuf_set_vlan_tag((m),(vlan))

//------------------------------------------------------------------------------

#define SKB_GSO_TCPV4           MBUF_TSO_IPV4
#define SKB_GSO_TCPV6           MBUF_TSO_IPV6

OS_INLINE int
mbuf_is_gso(mbuf_t m)
{
  mbuf_tso_request_flags_t tsoreq;
  u_int32_t value;
  
  mbuf_get_tso_requested(m, &tsoreq, &value);
  
  return !!tsoreq; // Return exactly 1 or 0; is or is not GSO.
}

OS_INLINE int
gso_size(mbuf_t m)
{
  mbuf_tso_request_flags_t tsoreq;
  u_int32_t value;
  
  mbuf_get_tso_requested(m, &tsoreq, &value);
  
  return value;
}

OS_INLINE mbuf_tso_request_flags_t
gso_type(mbuf_t m)
{
  mbuf_tso_request_flags_t tsoreq;
  u_int32_t value;
  
  mbuf_get_tso_requested(m, &tsoreq, &value);
  
  return tsoreq;
}

//==============================================================================
#pragma mark -
#pragma mark Packet Headers
#pragma mark -
//==============================================================================

#define skb_headlen(m)      ((int)mbuf_len(m))

#define eth_hdr(skb)        ((struct ether_header *)mbuf_data(skb))

#define ip_hdr(skb)         ((struct ip *)(eth_hdr(skb) + 1))
#define ipv6_hdr(skb)       ((struct ip6_hdr *)(eth_hdr(skb) + 1))

OS_INLINE struct tcphdr
*tcp_hdr(const mbuf_t m)
{
  char *hdr = NULL;
  struct ip6_ext *ext = NULL;
  
  if (ip_hdr(m)->ip_v == 4)
  {
    hdr = (char *)ip_hdr(m) + (ip_hdr(m)->ip_hl << 2);
  }
  else if (ip_hdr(m)->ip_v == 6)
  {
    hdr = (char *)ipv6_hdr(m) + sizeof(struct ip6_hdr);
    
    if (unlikely(ipv6_hdr(m)->ip6_nxt == IPPROTO_NONE))
    {
      ErrPrint("No TCP header found in IPv6 TSO packet!\n");
    }
    else if (ipv6_hdr(m)->ip6_nxt != IPPROTO_TCP) // Extension headers.
    {
      do
      {
        ext = (struct ip6_ext *)hdr;
        hdr += 8 + (ext->ip6e_len << 3);
        if (unlikely(ext->ip6e_nxt == IPPROTO_NONE))
        {
          ErrPrint("No TCP header found in IPv6 TSO packet!\n");
          break;
        }
      } while (ext->ip6e_nxt != IPPROTO_TCP);
    }
  }
  
  return (struct tcphdr *)hdr;
}

#define tcp_hdrlen(m)     ((int)(tcp_hdr(m)->th_off << 2))

#define skb_network_offset(m) \
    ((int)((char *)ip_hdr(m) - (char *)mbuf_data(m)))

#define skb_transport_offset(m) \
    ((int)((char *)tcp_hdr(m) - (char *)mbuf_data(m)))

//------------------------------------------------------------------------------

#endif // __ALX_NET_H__
