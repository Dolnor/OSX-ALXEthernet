/* ALXEthernet.h -- ALX driver class definition.
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
 * This driver is heavily based on the Atheros ALX Linux driver.
 */

#ifndef __ALXETHERNET_H__
#define __ALXETHERNET_H__

#include "alx.h"
#include "alx_hwcom.h"

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Definitions & Helper Functions
#pragma mark -
//------------------------------------------------------------------------------

#define CLASS       ALXEthernet
#define super       IOEthernetController

//------------------------------------------------------------------------------

#define RELEASE(x)                                                             \
do                                                                             \
{                                                                              \
  if (x)                                                                       \
  {                                                                            \
    (x)->release();                                                            \
    (x) = NULL;                                                                \
  }                                                                            \
} while(0)

#define ALX_MULTICAST_CAPACITY      32
#define ALX_RX_DESCS                1024
#define ALX_TX_DESCS                512

// Buffer address mask, 32 bit addressable and 8-byte boundry aligned.
#define ALX_BufferAddressMask	(0x00000000ffffffffULL & ~(sizeof(UInt64) - 1))
#define ALX_32BITMASK 0x00000000ffffffffULL
#define ALX_64BITMASK 0xfffffffffffff000ULL


#define NumPMStates                 2
#define PMStateOff                  0
#define PMStateOn                   kIOPMPowerOn

enum
{
  kPowerStateOff = 0,
  kPowerStateOn = 1,
  kPowerStateCount = 2
};

#define Mbit                        1000000
#define ALX_TransmitQueueCapacity   512

enum
{
  MEDIUM_INDEX_AUTO = 0,
  MEDIUM_INDEX_NONE = 1,
  MEDIUM_INDEX_10HD = 2,
  MEDIUM_INDEX_10FD = 3,
  MEDIUM_INDEX_100HD = 4,
  MEDIUM_INDEX_100FD = 5,
  MEDIUM_INDEX_COUNT_FAST = 6,
  MEDIUM_INDEX_1000FD = 6,
  MEDIUM_INDEX_COUNT_GIGABIT = 7
};

enum ALX_speedType
{
  TYPE_GIGABIT = 0,
  TYPE_FAST = 1
};

#define ALX_VENDOR                  "Qualcomm Atheros"
#define ALX_VENDOR_SHORT            "QCA"

#define ALX_NAME_UNKNOWN            "Unknown Ethernet"
#define ALX_NAME_AR8131             "AR8131 Gigabit Ethernet"
#define ALX_NAME_AR8132             "AR8132/L1c Fast Ethernet"
#define ALX_NAME_AR8151_V1          "AR8151 v1.0 Gigabit Ethernet"
#define ALX_NAME_AR8151_V2          "AR8151 v2.0 Gigabit Ethernet"
#define ALX_NAME_AR8152_V1          "AR8152 v1.1 Fast Ethernet"
#define ALX_NAME_AR8152_V2          "AR8152 v2.0 Fast Ethernet"
#define ALX_NAME_AR8161             "AR8161 Gigabit Ethernet"
#define ALX_NAME_AR8162             "AR8162 Fast Ethernet"
#define ALX_NAME_AR8171             "AR8171 Gigabit Ethernet"
#define ALX_NAME_AR8172             "AR8172 Fast Ethernet"

enum                            // Indices of ALX_deviceTable[] below.
{
  ALX_INDEX_UNKNOWN = 0,
  ALX_INDEX_AR8131 = 1,
  ALX_INDEX_AR8132 = 2,
  ALX_INDEX_AR8151_V1 = 3,
  ALX_INDEX_AR8151_V2 = 4,
  ALX_INDEX_AR8152_V1 = 5,
  ALX_INDEX_AR8152_V2 = 6,
  ALX_INDEX_AR8161 = 7,
  ALX_INDEX_AR8162 = 8,
  ALX_INDEX_AR8171 = 9,
  ALX_INDEX_AR8172 = 10
};

struct
{
  const char *name;
  const char *model;
  enum ALX_speedType type;
} const ALX_deviceTable[] = {
  {
   ALX_VENDOR " " ALX_NAME_UNKNOWN,
   ALX_NAME_UNKNOWN,
   TYPE_FAST},
  {
   ALX_VENDOR " " ALX_NAME_AR8131,
   ALX_NAME_AR8131,
   TYPE_GIGABIT},
  {
   ALX_VENDOR " " ALX_NAME_AR8132,
   ALX_NAME_AR8132,
   TYPE_FAST},
  {
   ALX_VENDOR " " ALX_NAME_AR8151_V1,
   ALX_NAME_AR8151_V1,
   TYPE_GIGABIT},
  {
   ALX_VENDOR " " ALX_NAME_AR8151_V2,
   ALX_NAME_AR8151_V2,
   TYPE_GIGABIT},
  {
   ALX_VENDOR " " ALX_NAME_AR8152_V1,
   ALX_NAME_AR8152_V1,
   TYPE_FAST},
  {
   ALX_VENDOR " " ALX_NAME_AR8152_V2,
   ALX_NAME_AR8152_V2,
   TYPE_FAST},
  {
   ALX_VENDOR " " ALX_NAME_AR8161,
   ALX_NAME_AR8161,
   TYPE_GIGABIT},
  {
   ALX_VENDOR " " ALX_NAME_AR8162,
   ALX_NAME_AR8162,
   TYPE_FAST},
  {
   ALX_VENDOR " " ALX_NAME_AR8171,
   ALX_NAME_AR8171,
   TYPE_GIGABIT},
  {
   ALX_VENDOR " " ALX_NAME_AR8172,
   ALX_NAME_AR8172,
   TYPE_FAST},
};

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark ALXEthernet
#pragma mark -
//------------------------------------------------------------------------------

class ALXEthernet : public IOEthernetController
{
  OSDeclareDefaultStructors(ALXEthernet)
  
//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Public Methods
#pragma mark -
//------------------------------------------------------------------------------

public:
  bool init(OSDictionary * properties);
  void free();

  bool start(IOService * provider);
  void stop(IOService * provider);

//------------------------------------------------------------------------------

  IOReturn enable(IONetworkInterface * interface);
  IOReturn disable(IONetworkInterface * interface);

  const OSString *newVendorString() const;
  const OSString *newModelString() const;

  bool configureInterface(IONetworkInterface * interface);
  IOReturn selectMedium(const IONetworkMedium * medium);
  IOOutputQueue *createOutputQueue();
  bool createWorkLoop();
  IOWorkLoop *getWorkLoop() const;
  UInt32 getFeatures() const;

  IOReturn getHardwareAddress(IOEthernetAddress * addr);
  IOReturn setHardwareAddress(const IOEthernetAddress * addr);

  IOReturn setPromiscuousMode(bool enabled);
  IOReturn setMulticastMode(bool enabled);
  IOReturn setMulticastAllMode(bool enabled);
  IOReturn setMulticastList(IOEthernetAddress * addrs, UInt32 count);

  IOReturn registerWithPolicyMaker(IOService * policyMaker);
  IOReturn setPowerState
    (unsigned long powerStateOrdinal, IOService * policyMaker);
  IOReturn setWakeOnMagicPacket(bool active);
  void systemWillShutdown(IOOptionBits specifier);

  IOReturn getMaxPacketSize(UInt32 * maxSize) const;
  IOReturn getMinPacketSize(UInt32 * minSize) const;
  IOReturn setMaxPacketSize(UInt32 maxSize);
  IOReturn getPacketFilters(const OSSymbol * group, UInt32 * filters) const;
  IOReturn enablePacketFilter
    (const OSSymbol * group,
     UInt32 aFilter, UInt32 enabledFilters, IOOptionBits options = 0);
  IOReturn disablePacketFilter
    (const OSSymbol * group,
     UInt32 aFilter, UInt32 enabledFilters, IOOptionBits options = 0);
  void getPacketBufferConstraints
    (IOPacketBufferConstraints * constraints) const;
  UInt32 outputPacket(mbuf_t m, void *param);

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Private Methods
#pragma mark -
//------------------------------------------------------------------------------

private:
  static IOReturn setPowerStateSleepAction
    (OSObject * owner, void *arg1, void *arg2, void *arg3, void *arg4);
  static IOReturn setPowerStateWakeAction
    (OSObject * owner, void *arg1, void *arg2, void *arg3, void *arg4);
  IOReturn setPowerStateSleep();
  IOReturn setPowerStateWake();

//------------------------------------------------------------------------------

  bool ALX_addNetworkMedium
    (UInt32 type, UInt32 bps, UInt32 index, const char *name = 0);
  IOReturn ALX_allocateAllDescriptors();
  IOReturn ALX_allocateAllQueues();
  IOReturn ALX_allocateAllRxDescriptors();
  IOReturn ALX_allocateAllTxDescriptors();
  IOReturn ALX_allocateRxDescriptor(struct alx_rx_queue *rxque);
  IOReturn ALX_allocateTxDescriptor(struct alx_tx_queue *txque);
  void ALX_checkLinkStatusChange();
  bool ALX_checkNumberTxDescriptors
    (struct alx_tx_queue *txque, const mbuf_t m);
  void ALX_cleanAllRxQueues();
  void ALX_cleanAllTxQueues();
  void ALX_cleanBuffer(struct alx_buffer *buf);
  void ALX_cleanRFDescriptor
    (struct alx_rx_queue *rxque, union alx_sw_rrdesc *srrd);
  void ALX_cleanRxQueue(struct alx_rx_queue *rxque);
  void ALX_cleanTxQueue(struct alx_tx_queue *txque);
  void ALX_configureRSS();
  IOReturn ALX_disable(UInt32 ctrl);
  void ALX_disableInterrupt();
  IOReturn ALX_enable(UInt32 ctrl);
  void ALX_enableInterrupt();
  void ALX_freeAllDescriptors();
  void ALX_freeAllQueues();
  void ALX_freeRxDescriptor(struct alx_rx_queue *rxque);
  void ALX_freeAllRxDescriptors();
  void ALX_freeTxDescriptor(struct alx_tx_queue *txque);
  void ALX_freeAllTxDescriptors();
  IOReturn ALX_getInterruptSource();
  enum ALX_speedType ALX_getNICType();
  bool ALX_getRRDescriptor
    (struct alx_rx_queue *rxque, union alx_sw_rrdesc *srrd);
  IOReturn ALX_handleRxIRQ(struct alx_rx_queue *rxque);
  IOReturn ALX_handleTxIRQ(struct alx_tx_queue *txque);
  IOReturn ALX_init();
  IOReturn ALX_initAdapter();
  IOReturn ALX_initAdapterSpecial();
  static IOReturn ALX_initBuffer(struct alx_buffer *buf);
  IOReturn ALX_initHWCallbacks();
  void ALX_initPCIPowerManagement();
  IOReturn ALX_initRingPointers();
  void ALX_interrupt
    (OSObject * client, IOInterruptEventSource * src, int count);
  void ALX_linkTaskRoutine();
  void ALX_patchAssign();
  IOReturn ALX_publishMediumDictionary();
  void ALX_receiveMbuf(mbuf_t m, u16 vlan_tag, bool vlan_flag);
  int ALX_refreshRxBuffer(struct alx_rx_queue *rxque);
  void ALX_reinitLocked();
  void ALX_reinitTaskRoutine();
  void ALX_restoreVLAN();
  IOReturn ALX_selectMedium(const IONetworkMedium * medium);
  void ALX_setDeviceInfo();
  IOReturn ALX_setMACAddress(const IOEthernetAddress * addr);
  IOReturn ALX_setMACType();
  void ALX_setMulticastList();
  void ALX_setQueueNumber();
  IOReturn ALX_setRegisterInfoSpecial();
  IOReturn ALX_setRFDescriptor
    (struct alx_rx_queue *rxque, union alx_sw_rfdesc *srfd);
  void ALX_setRxQueueNumber();
  bool ALX_setTPDescriptor
    (struct alx_tx_queue *txque, union alx_sw_tpdesc *stpd);
  void ALX_setTPDescriptorLastFragment(struct alx_tx_queue *txque);
  void ALX_setTxQueueNumber();
  IOReturn ALX_sleep(bool * wakeup);
  IOReturn ALX_startTransmit(mbuf_t m);
  IOReturn ALX_startTransmitFrame(struct alx_tx_queue *txque, mbuf_t m);
  void ALX_taskRoutine();
  void ALX_taskSchedule();
  static void ALX_timerFired(OSObject * owner, IOTimerEventSource * sender);
  void ALX_timerRoutine();
  void ALX_TxMap
    (struct alx_tx_queue *txque, mbuf_t m, union alx_sw_tpdesc *stpd);
  IOReturn ALX_validateMACAddress(const UInt8 * mac_addr);
  void ALX_VLANMode();
  IOReturn ALX_wake();

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Private Variables
#pragma mark -
//------------------------------------------------------------------------------

private:
  struct alx_adapter ALX_Adapter;

  IOPCIDevice *ALX_PCIDevice;
  IOMemoryMap *ALX_PCIRegMap;
  UInt16 ALX_VendorID;
  UInt16 ALX_DeviceID;
  UInt8 ALX_DeviceTableIndex;

  IOEthernetInterface *ALX_NetIf;
  IONetworkStats *ALX_NetStats;
  IOEthernetStats *ALX_EtherStats;

  IOWorkLoop *ALX_WorkLoop;
  IOInterruptEventSource *ALX_InterruptSource;
  IOCommandGate *ALX_CommandGate;
  IOTimerEventSource *ALX_TimerEventSource;

  bool ALX_Enabled;

  IOOutputQueue *ALX_TransmitQueue;
  bool ALX_TransmitQueueEnabled;

  bool ALX_LinkEnabled;
  UInt64 ALX_LinkTimeout;

  OSDictionary *ALX_MediumDict;
  const IONetworkMedium *ALX_MediumTable[MEDIUM_INDEX_COUNT_GIGABIT];

  IOEthernetAddress *ALX_MulticastList;
  UInt8 ALX_MulticastListLength;
  UInt32 ALX_MulticastFlags;
  bool ALX_MulticastAllRequested; // Client requested.

  bool ALX_MagicPacketSupported;
  bool ALX_MagicPacketEnabled;
  UInt32 ALX_CurrentPowerState;
};

//------------------------------------------------------------------------------

#endif // __ALXETHERNET_H__
