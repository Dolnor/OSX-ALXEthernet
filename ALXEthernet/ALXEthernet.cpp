/* ALXEthernet.cpp --  Driver class implementation.
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

#include "ALXEthernet.h"

int alx_cfg_r32(const struct alx_hw *hw, int reg, u32 *pval)
{
  *pval = hw->adpt->pdev->extendedConfigRead32(reg);

  return 0;
}

int alx_cfg_w32(const struct alx_hw *hw, int reg, u32 val)
{
  hw->adpt->pdev->extendedConfigWrite32(reg, val);

  return 0;
}

int alx_cfg_r16(const struct alx_hw *hw, int reg, u16 *pval)
{
  *pval = hw->adpt->pdev->extendedConfigRead16(reg);

  return 0;
}

int alx_cfg_w16(const struct alx_hw *hw, int reg, u16 val)
{
  hw->adpt->pdev->extendedConfigWrite16(reg, val);

  return 0;
}

void alx_mem_flush(const struct alx_hw *hw)
{
  readl(hw->hw_addr);
}


void alx_mem_r32(const struct alx_hw *hw, int reg, u32 *val)
{
  if (unlikely(!hw->link_up))
  {
    readl(hw->hw_addr + reg);
  }

  *val = readl(hw->hw_addr + reg);
}


void alx_mem_w32(const struct alx_hw *hw, int reg, u32 val)
{
  if (hw->mac_type == alx_mac_l2cb_v20 && reg < 0x1400)
  {
    readl(hw->hw_addr + reg);
  }

  writel(val, hw->hw_addr + reg);
}


void alx_mem_r16(const struct alx_hw *hw, int reg, u16 *val)
{
  if (unlikely(!hw->link_up))
  {
    readl(hw->hw_addr + reg);
  }

  *val = readw(hw->hw_addr + reg);
}


void alx_mem_w16(const struct alx_hw *hw, int reg, u16 val)
{
  if (hw->mac_type == alx_mac_l2cb_v20 && reg < 0x1400)
  {
    readl(hw->hw_addr + reg);
  }

  writew(val, hw->hw_addr + reg);
}


void alx_mem_w8(const struct alx_hw *hw, int reg, u8 val)
{
  if (hw->mac_type == alx_mac_l2cb_v20 && reg < 0x1400)
  {
    readl(hw->hw_addr + reg);
  }

  writeb(val, hw->hw_addr + reg);
}

#define ALX_checkAdapterFlag(_flag) \
  CHK_FLAG(&ALX_Adapter, ADPT, _flag)
#define ALX_setAdapterFlag(_flag) \
  SET_FLAG(&ALX_Adapter, ADPT, _flag)
#define ALX_clearAdapterFlag(_flag) \
  CLI_FLAG(&ALX_Adapter, ADPT, _flag)

//==============================================================================

OSDefineMetaClassAndStructors(ALXEthernet, IOEthernetController)

//==============================================================================
#pragma mark -
#pragma mark IOService Overrides
#pragma mark -
//==============================================================================

bool
CLASS::init(OSDictionary *properties)
{
  DbgPrint(4, "init()\n");

  if (!super::init(properties))
  {
    ErrPrint("Unable to initialise ethernet controller superclass.\n");
    return false;
  }

  memset(&ALX_Adapter, 0, sizeof(alx_adapter));
  ALX_Adapter.pdev = NULL;
  ALX_Adapter.hw.hw_addr = NULL;
  ALX_Adapter.hw.adpt = &ALX_Adapter;

  /**********************************************************/

  ALX_MulticastList = (IOEthernetAddress *)kzalloc(sizeof(IOEthernetAddress) *
                      ALX_MULTICAST_CAPACITY, 0);

  if (ALX_MulticastList == NULL)
  {
    ErrPrint("Unable to allocate multicast list array!\n");
    return false;
  }

  DbgPrint(5, "Multicast list array allocated successfully.\n");
  ALX_MulticastListLength = 0;
  ALX_MulticastFlags = 0;

  /**********************************************************/

  ALX_NetIf = NULL;
  ALX_PCIRegMap = NULL;
  ALX_WorkLoop = NULL;
  ALX_InterruptSource = NULL;
  ALX_CommandGate = NULL;
  ALX_TimerEventSource = NULL;
  ALX_Enabled = false;
  ALX_TransmitQueueEnabled = false;
  ALX_LinkEnabled = false;
  ALX_LinkTimeout = 0;
  ALX_MulticastAllRequested = false;

  return true;
}

//==============================================================================

void
CLASS::free()
{
  DbgPrint(4, "free()\n");

  if (ALX_Adapter.hw.mdio_lock)
  {
    IOSimpleLockFree(ALX_Adapter.hw.mdio_lock);
    ALX_Adapter.hw.mdio_lock = NULL;
  }

  if (ALX_Adapter.rx_lock)
  {
    IOSimpleLockFree(ALX_Adapter.rx_lock);
    ALX_Adapter.rx_lock = NULL;
  }

  if (ALX_Adapter.tx_lock)
  {
    IOSimpleLockFree(ALX_Adapter.tx_lock);
    ALX_Adapter.tx_lock = NULL;
  }

  if (ALX_MulticastList)
  {
    kfree(ALX_MulticastList);
    ALX_MulticastList = NULL;
  }

  RELEASE(ALX_InterruptSource);
  RELEASE(ALX_CommandGate);
  RELEASE(ALX_TimerEventSource);
  RELEASE(ALX_NetIf);
  RELEASE(ALX_PCIRegMap);

  super::free();
}

//==============================================================================

bool
CLASS::start(IOService *provider)
{
  DbgPrint (4, "start()\n");

  if (!super::start(provider))
  {
    ErrPrint ("Unable to start ethernet controller superclass.\n");
    return false;
  }

  PMinit();
  provider->joinPMtree(this);
  registerWithPolicyMaker(provider);

  /**********************************************************/
  /* Open the PCI provider.                                 */
  /**********************************************************/

  ALX_PCIDevice = OSDynamicCast(IOPCIDevice, provider);

  if (!ALX_PCIDevice)
  {
    ErrPrint ("Unable to cast provider to PCI device.\n");
    return false;
  }

  ALX_PCIDevice->retain();

  if (ALX_PCIDevice->open(this) == false)
  {
    ErrPrint("Unable to open PCI device.\n");
    ALX_PCIDevice->close(this);
    return false;
  }

  /**********************************************************/

  if (ALX_init() != kIOReturnSuccess)
  {
    ErrPrint("Unable to initialise adapter.\n");
    stop(provider);
    return false;
  }

  ALX_setDeviceInfo();

  if (ALX_publishMediumDictionary() != kIOReturnSuccess)
  {
    ErrPrint("Unable to publish medium dictionary.\n");
    return false;
  }

  /**********************************************************/
  /* Get command gate already attached to workloop.         */
  /**********************************************************/

  ALX_CommandGate = getCommandGate();

  if (!ALX_CommandGate)
  {
    ErrPrint("Unable to create command gate.\n");
    stop(provider);
    return false;
  }

  ALX_CommandGate->retain();

  /**********************************************************/
  /* Attach watchdog timer to workloop.                     */
  /**********************************************************/

  ALX_TimerEventSource = IOTimerEventSource::timerEventSource(this,
                         ALX_timerFired);

  if (!ALX_TimerEventSource)
  {
    ErrPrint("Unable to create timer event source.\n");
    stop(provider);
    return false;
  }

  if (ALX_WorkLoop->addEventSource(ALX_TimerEventSource) != kIOReturnSuccess)
  {
    ErrPrint("Unable to add timer event source to workloop.\n");
    stop(provider);
    return false;
  }

  ALX_timerRoutine();

  /**********************************************************/
  /* Attach MSI interrupts to workloop.                     */
  /**********************************************************/

  ALX_getInterruptSource();

  if (!ALX_InterruptSource ||
      ALX_WorkLoop->addEventSource(ALX_InterruptSource) != kIOReturnSuccess)
  {
    if (!ALX_InterruptSource)
    {
      ErrPrint("Unable to create interrupt source.\n");
    }
    else
    {
      ErrPrint("Unable to attach interrupt source.\n");
    }

    stop(provider);
    return false;
  }

  ALX_setAdapterFlag(MSI_EN);
  ALX_InterruptSource->enable();

  /**********************************************************/
  /* Create controller output queue.                        */
  /**********************************************************/

  ALX_TransmitQueue = getOutputQueue();

  if (!ALX_TransmitQueue)
  {
    ErrPrint("Unable to get output queue.\n");
    stop(provider);
    return false;
  }

  ALX_TransmitQueue->retain();

  /**********************************************************/
  // Attach dynamic link layer.
  /**********************************************************/

  if (!attachInterface(reinterpret_cast<IONetworkInterface **>(&ALX_NetIf)))
  {
    ErrPrint("Unable to attach data link layer.\n");
    return false;
  }

  ALX_NetIf->setProperty(kIOBuiltin, true);

  if (ALX_allocateAllDescriptors() != kIOReturnSuccess)
  {
    ErrPrint("Unable to allocate descriptors.\n");
    ALX_freeAllDescriptors();
    ALX_Adapter.hw.cbs.reset_mac(&ALX_Adapter.hw);
    return false;
  }

  /**********************************************************/
  /* Close PCI, re-open on demand when controller enabled.  */
  /**********************************************************/

  ALX_PCIDevice->close(this);

  return true;
}

//==============================================================================

void
CLASS::stop(IOService *provider)
{
  DbgPrint (4, "stop()\n");

  if (ALX_NetIf)
  {
    detachInterface(ALX_NetIf);
  }

  if (ALX_InterruptSource && ALX_WorkLoop)
  {
    ALX_WorkLoop->removeEventSource(ALX_InterruptSource);
  }

  if (ALX_TimerEventSource && ALX_WorkLoop)
  {
    ALX_TimerEventSource->cancelTimeout();
    ALX_WorkLoop->removeEventSource(ALX_TimerEventSource);
  }

  ALX_freeAllDescriptors();

  ALX_freeAllQueues();

  PMstop();
  super::stop(provider);
}

//==============================================================================
#pragma mark -
#pragma mark IOEthernetController Overrides
#pragma mark -
//==============================================================================

IOReturn
CLASS::enable(IONetworkInterface *netif)
{
  DbgPrint(5, "enable()\n");

  struct alx_hw *hw = &ALX_Adapter.hw;

  if (ALX_PCIDevice == NULL)
  {
    ErrPrint("Unable to open null PCI device.\n");
    return kIOReturnError;
  }

  if ((ALX_PCIDevice->isOpen() == false) &&
      (ALX_PCIDevice->open(this) == false))
  {
    ErrPrint("Unable to open PCI device.\n");
    return kIOReturnError;
  }

  // Disallow enable during testing.
  if (ALX_checkAdapterFlag(STATE_TESTING))
  {
    DbgPrint(3, "Disallowed enable during testing.\n");
    ALX_PCIDevice->close(this);
    return kIOReturnError;
  }

  setLinkStatus(kIONetworkLinkValid);
  ALX_LinkEnabled = false;

  if (ALX_enable(ALX_OPEN_CTRL_IRQ_EN) != kIOReturnSuccess)
  {
    ErrPrint("Failed to enable adapter.\n");
    ALX_disable(ALX_OPEN_CTRL_IRQ_EN);
    hw->cbs.reset_mac(hw);
    ALX_PCIDevice->close(this);
    return kIOReturnError;
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::disable(IONetworkInterface *netif)
{
  DbgPrint (5, "disable()\n");

  if (ALX_checkAdapterFlag(STATE_RESETTING))
  {
    DbgPrint(2, "Flag STATE_RESETTING has already been set.\n");
  }

  ALX_disable(ALX_OPEN_CTRL_IRQ_EN | ALX_OPEN_CTRL_RESET_MAC);

  if ((ALX_PCIDevice != NULL) &&
      (ALX_PCIDevice->isOpen() == true))
  {
    ALX_PCIDevice->close(this);
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOOutputQueue *
CLASS::createOutputQueue()
{
  DbgPrint(4, "createOutputQueue()\n");

  // Sharing one event source with transmit/receive handles.
  return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

//==============================================================================

// Configure a newly instantiated IONetworkInterface object.
// Fairly stock-standard implementation for IOEthernetController
// implementations.

bool
CLASS::configureInterface(IONetworkInterface *netif)
{
  DbgPrint(4, "configureInterface()\n");

  IONetworkData *data;

  if (!super::configureInterface(netif))
  {
    return false;
  }

  // Get the generic network statistics structure.
  data = netif->getParameter(kIONetworkStatsKey);

  if (!data || !(ALX_NetStats = (IONetworkStats *)data->getBuffer()))
  {
    return false;
  }

  // Get the ethernet statistics structure.
  data = netif->getParameter(kIOEthernetStatsKey);

  if (!data || !(ALX_EtherStats = (IOEthernetStats *)data->getBuffer()))
  {
    return false;
  }

  return true;
}

//==============================================================================

const OSString *
CLASS::newVendorString() const
{
  return OSString::withCString(ALX_VENDOR);
}

//==============================================================================

const OSString *
CLASS::newModelString() const
{
  return OSString::withCString(ALX_deviceTable[ALX_DeviceTableIndex].model);
}

//==============================================================================

IOReturn
CLASS::selectMedium(const IONetworkMedium *medium)
{
  DbgPrint (4, "selectMedium()\n");

  if (medium == 0)
  {
    medium = IONetworkMedium::getMediumWithIndex(
               ALX_MediumDict, MEDIUM_INDEX_AUTO);

    if (medium == 0)
    {
      return kIOReturnUnsupported;
    }
  }

  if (ALX_selectMedium(medium) == kIOReturnSuccess)
  {
    DbgPrint(3, "Selected medium: %s\n", medium->getName()->getCStringNoCopy());

    setSelectedMedium(medium);
  }
  else
  {
    ErrPrint("Unable to select medium: %s\n",
             medium->getName()->getCStringNoCopy());
    return kIOReturnError;
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::getHardwareAddress(IOEthernetAddress *addr)
{
  DbgPrint(4, "getHardwareAddress()\n");

  struct alx_hw *hw = &ALX_Adapter.hw;

  if (ALX_validateMACAddress(ALX_Adapter.hw.mac_addr) == kIOReturnSuccess)
  {
    memcpy(addr->bytes, ALX_Adapter.hw.mac_addr, ETHER_ADDR_LEN);
    return kIOReturnSuccess;
  }
  else if (hw->cbs.get_mac_addr(hw, addr->bytes) == 0)
  {
    return kIOReturnSuccess;
  }

  return kIOReturnError;
}

//==============================================================================

IOReturn
CLASS::setHardwareAddress(const IOEthernetAddress *addr)
{
  DbgPrint(4, "setHardwareAddress()\n");

  return ALX_setMACAddress(addr);
}

//==============================================================================

void
CLASS::getPacketBufferConstraints(IOPacketBufferConstraints *constraints)
const
{
  DbgPrint(4, "getPacketBufferConstraints()\n");

  constraints->alignStart = kIOPacketBufferAlign8;
  constraints->alignLength = kIOPacketBufferAlign8;
}

//==============================================================================

UInt32
CLASS::outputPacket(mbuf_t m, void *param)
{
  DbgPrint(6, "outputPacket()\n");

  IOReturn ret_val;

  ret_val = ALX_startTransmit(m);
  OSSynchronizeIO();

  return ret_val;
}

//==============================================================================

IOReturn
CLASS::registerWithPolicyMaker(IOService *policyMaker)
{
  DbgPrint (4, "registerWithPolicyMaker()\n");

  static IOPMPowerState powerStateArray[kPowerStateCount] =
  {
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
  };

  ALX_CurrentPowerState = kPowerStateOn;

  return policyMaker->registerPowerDriver(this, powerStateArray,
                                          kPowerStateCount);
}

//==============================================================================

IOReturn
CLASS::setPowerState(unsigned long powerStateOrdinal,
                     IOService *policyMaker)
{
  if (!ALX_PCIDevice)
  {
    ErrPrint("Null PCI device.\n");

    return IOPMAckImplied;
  }

  if (powerStateOrdinal == ALX_CurrentPowerState)
  {
    DbgPrint(4, "setPowerState(ALX_CurrentPowerState)\n");
    return IOPMAckImplied;
  }

  retain();

  switch (powerStateOrdinal)
  {
  case kPowerStateOff:
    DbgPrint(3, "Powering ethernet off.\n");

    if (ALX_CommandGate)
    {
      ALX_CommandGate->runAction(setPowerStateSleepAction);
    }

    break;

  case kPowerStateOn:
    DbgPrint(3, "Powering ethernet on.\n");

    if (ALX_CommandGate)
    {
      ALX_CommandGate->runAction(setPowerStateWakeAction);
    }

    break;
  }

  release();

  ALX_CurrentPowerState = powerStateOrdinal;

  return IOPMAckImplied;
}

//==============================================================================

IOReturn
CLASS::setPowerStateSleepAction(OSObject *owner,
                                void *arg1, void *arg2, void *arg3, void *arg4)
{
  if (owner)
  {
    ALXEthernet *alx_eth = OSDynamicCast(ALXEthernet, owner);

    if (alx_eth)
    {
      alx_eth->setPowerStateSleep();
    }
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::setPowerStateWakeAction(OSObject *owner,
                               void *arg1, void *arg2, void *arg3, void *arg4)
{
  if (owner)
  {
    ALXEthernet *alx_eth = OSDynamicCast(ALXEthernet, owner);

    if (alx_eth)
    {
      alx_eth->setPowerStateWake();
    }
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::setPowerStateSleep()
{
  DbgPrint(4, "setPowerStateSleep()\n");

  bool wakeup;

  // This is already called automatically before setPowerState().
  // disable(ALX_NetIf);

  // Turn off wake on PHY status change. Sleep seems to wake back up
  // immediately with it turned on.
  ALX_Adapter.wol &= ~(ALX_WOL_PHY);

  if (ALX_sleep(&wakeup) != kIOReturnSuccess)
  {
    ErrPrint("Unable to properly sleep device.\n");
  }

  DbgPrint(2, "Sleeping ethernet %s wake support.\n",
           wakeup ? "with" : "without");

  if (wakeup)
  {
    if (ALX_PCIDevice->hasPCIPowerManagement(kPCIPMCPMESupportFromD3Cold))
    {
      DbgPrint (3, "Enabling PME from D3cold.\n");
      ALX_PCIDevice->enablePCIPowerManagement(kPCIPMCSPMEStatus |
                                              kPCIPMCSPMEEnable |
                                              kPCIPMCSPowerStateD3);
    }
  }
  else
  {
    if (ALX_PCIDevice->hasPCIPowerManagement(kPCIPMCD3Support))
    {
      ALX_PCIDevice->enablePCIPowerManagement(kPCIPMCSPowerStateD3);
    }
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::setPowerStateWake()
{
  DbgPrint(4, "setPowerStateWake()\n");

  if (ALX_CurrentPowerState == kPowerStateOff)
  {
    ALX_initPCIPowerManagement();
    ALX_wake();
  }

  // This is already called automatically after setPowerState().
  // enable(ALX_NetIf);

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::enablePacketFilter(const OSSymbol *group, UInt32 aFilter,
                          UInt32 enabledFilters, IOOptionBits options)
{
  if (group == gIONetworkFilterGroup)
  {
    switch (aFilter)
    {
    case kIOPacketFilterMulticastAll:
      return setMulticastAllMode(true);
      break;

    case kIOPacketFilterPromiscuousAll:
      return kIOReturnUnsupported;
      break;

    default:
      break;
    }
  }

  return IOEthernetController::enablePacketFilter(group, aFilter,
         enabledFilters, options);
}

//==============================================================================

IOReturn
CLASS::disablePacketFilter(const OSSymbol *group, UInt32 aFilter,
                           UInt32 enabledFilters, IOOptionBits options)
{
  if (group == gIONetworkFilterGroup)
  {
    switch (aFilter)
    {
    case kIOPacketFilterMulticastAll:
      return setMulticastAllMode(false);
      break;

    case kIOPacketFilterPromiscuousAll:
      return kIOReturnUnsupported;
      break;

    default:
      break;
    }
  }

  return IOEthernetController::disablePacketFilter(group, aFilter,
         enabledFilters, options);
}

//==============================================================================

IOReturn
CLASS::getPacketFilters(const OSSymbol *group, UInt32 *filters) const
{
  if ((group == gIOEthernetWakeOnLANFilterGroup) && ALX_MagicPacketSupported)
  {
    *filters = kIOEthernetWakeOnMagicPacket;
    DbgPrint(3, "kIOEthernetWakeOnMagicPacket added to filters.\n");

    return kIOReturnSuccess;
  }
  else if (group == gIONetworkFilterGroup)
  {
    IOEthernetController::getPacketFilters(group, filters);
    *filters |= kIOPacketFilterMulticastAll;
    DbgPrint(2, "Packet filters: %#x\n", (int)*filters);

    return kIOReturnSuccess;
  }

  return IOEthernetController::getPacketFilters(group, filters);
}

//==============================================================================

IOReturn
CLASS::setWakeOnMagicPacket(bool active)
{
  // if (ALX_MagicPacketSupported)
  {
    ALX_MagicPacketEnabled = active;
    DbgPrint (2, "Wake on magic packet %s.\n",
              active ? "enabled" : "disabled");

    return kIOReturnSuccess;
  }

  return kIOReturnUnsupported;
}

//==============================================================================

// Stop active DMA to allow proper shutdown.
void
CLASS::systemWillShutdown(IOOptionBits specifier)
{
  DbgPrint(4, "systemWillShutdown(%#x)\n", (uint)specifier);

  struct alx_hw *hw = &ALX_Adapter.hw;

  if ((kIOMessageSystemWillPowerOff | kIOMessageSystemWillRestart) & specifier)
  {
    disable(ALX_NetIf);
    hw->cbs.config_pow_save(hw, 0, false, false, false, false);

    // Resume permanent MAC address.
    hw->cbs.set_mac_addr(hw, hw->mac_perm_addr);
  }

  // Must call super shutdown or system will stall.
  super::systemWillShutdown(specifier);
}

//==============================================================================

/**
 * Promiscuous mode set.
 **/

IOReturn
CLASS::setPromiscuousMode(bool enabled)
{
  DbgPrint(2, "%s promiscuous mode.\n", enabled ? "Enabling" : "Disabling");

  if (enabled)
  {
    ALX_MulticastFlags |= IFF_PROMISC;
  }
  else
  {
    ALX_MulticastFlags &= ~(IFF_PROMISC);
  }

  ALX_setMulticastList();

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::setMulticastMode(bool enabled)
{
  DbgPrint(2, "%s multicast mode.\n", enabled ? "Enabling" : "Disabling");

  if (!enabled)
  {
    // Clear the multicast list.
    ALX_MulticastListLength = 0;
  }

  ALX_setMulticastList();

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::setMulticastAllMode(bool enabled)
{
  DbgPrint(2, "%s multicast-all mode.\n", enabled ? "Enabling" : "Disabling");

  if (enabled)
  {
    ALX_MulticastAllRequested = true;
    ALX_MulticastFlags |= IFF_ALLMULTI;
  }
  else
  {
    ALX_MulticastAllRequested = false;
    ALX_MulticastFlags &= ~(IFF_ALLMULTI);
  }

  ALX_setMulticastList();

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::setMulticastList(IOEthernetAddress *addrs, UInt32 count)
{
  DbgPrint(6, "setMulticastList()\n");

  if (count <= ALX_MULTICAST_CAPACITY)
  {
    memcpy(&ALX_MulticastList[0], addrs, count * sizeof(IOEthernetAddress));
    ALX_MulticastListLength = count;
    if ((ALX_MulticastFlags & IFF_ALLMULTI) && (!ALX_MulticastAllRequested))
    {
      ALX_MulticastFlags &= ~IFF_ALLMULTI;
    }
  }
  else // Hash collisions probable; just enable multicast-all instead.
  {
    memcpy(&ALX_MulticastList[0], addrs,
           ALX_MULTICAST_CAPACITY * sizeof(IOEthernetAddress));
    ALX_MulticastListLength = ALX_MULTICAST_CAPACITY;
    ALX_MulticastFlags |= IFF_ALLMULTI;
    DbgPrint(1, "Excessively large multicast list; enabling multicast-all.\n");
  }

  ALX_setMulticastList();

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::getMaxPacketSize(UInt32 *maxSize) const
{
  DbgPrint(4, "getMaxPacketSize()\n");

  if (!maxSize)
  {
    return kIOReturnBadArgument;
  }

  *maxSize = ALX_Adapter.hw.mtu + ETHER_HDR_LEN + ETHER_CRC_LEN + VLAN_HLEN;

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::getMinPacketSize (UInt32 *minSize) const
{
  return IOEthernetController::getMinPacketSize(minSize);
}


//==============================================================================

IOReturn
CLASS::setMaxPacketSize(UInt32 maxSize)
{
  DbgPrint(4, "setMaxPacketSize(%u)\n", (unsigned int)maxSize);
  // TODO: Set MTU.
#if 0
  int err = alx_change_mtu(maxSize);

  if (err == -EINVAL)
  {
    return kIOReturnBadArgument;
  }
  else if (err == -ENOMEM)
  {
    return kIOReturnNoMemory;
  }

  return kIOReturnSuccess;
#endif
  return kIOReturnUnsupported;
}

//==============================================================================

UInt32 CLASS::getFeatures() const
{
  return kIONetworkFeatureHardwareVlan;
#if 0
  return
  (kIONetworkFeatureHardwareVlan | kIONetworkFeatureTSOIPv4 |
  kIONetworkFeatureTSOIPv6);
#endif
}

//==============================================================================

bool
CLASS::createWorkLoop()
{
  ALX_WorkLoop = IOWorkLoop::workLoop();

  return (ALX_WorkLoop != 0);
}

//==============================================================================

IOWorkLoop *
CLASS::getWorkLoop() const
{
  return ALX_WorkLoop;
}

//==============================================================================

// TODO: Checksum support.
#if 0
IOReturn
CLASS::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily,
                          bool isOutput)
{
  if (checksumFamily != kChecksumFamilyTCPIP)
  {
    return kIOReturnUnsupported;
  }

  if (isOutput)
  {
    *checksumMask = kChecksumTCPSum16;
    DbgPrint(2, "Enabled partial TX checksum offloading to hardware.\n");
  }

  return kIOReturnSuccess;
}
#endif

//==============================================================================

bool
CLASS::ALX_addNetworkMedium(UInt32 type, UInt32 bps, UInt32 index,
                            const char *name)
{
  DbgPrint(4, "ALX_addNetworkMedium()\n");

  IONetworkMedium *medium;

  medium = IONetworkMedium::medium(type, bps, 0, index, name);

  if (!medium)
  {
    ErrPrint("Unable to allocate network medium.\n");
    return false;
  }

  if (!IONetworkMedium::addMedium(ALX_MediumDict, medium))
  {
    ErrPrint("Unable to add network medium.\n");
    return false;
  }

  ALX_MediumTable[index] = medium;

  return true;
}

//==============================================================================

IOReturn
CLASS::ALX_allocateAllDescriptors()
{
  struct alx_ring_header *ring_header = &ALX_Adapter.ring_header;
  int num_tques = ALX_Adapter.num_txques;
  int num_rques = ALX_Adapter.num_hw_rxques;
  unsigned int num_tx_descs = ALX_Adapter.num_txdescs;
  unsigned int num_rx_descs = ALX_Adapter.num_rxdescs;

  // Real ring DMA buffer.
  // Each ring/block may need up to 8 bytes for alignment, hence the
  // additional bytes tacked onto the end.
  ring_header->size =
    num_tques * num_tx_descs * sizeof(union alx_hw_tpdesc) +
    num_rques * num_rx_descs * sizeof(union alx_hw_rfdesc) +
    num_rques * num_rx_descs * sizeof(union alx_hw_rrdesc) +
    num_tques * 8 + num_rques * 2 * 8;
  DbgPrint(3, "num_tques = %d, num_tx_descs = %d\n",
           num_tques, num_tx_descs);
  DbgPrint(3, "num_rques = %d, num_rx_descs = %d\n",
           num_rques, num_rx_descs);

  ring_header->used = 0;
  ring_header->dbuf =
    IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task,
    (kIODirectionInOut | kIOMemoryPhysicallyContiguous),
    ring_header->size, ALX_32BITMASK);

  if (!ring_header->dbuf)
  {
    ErrPrint("Cannot allocate memory for descriptor ring, size=%d\n",
      ring_header->size);
    return kIOReturnNoMemory;
  }

  if (ring_header->dbuf->prepare() != kIOReturnSuccess)
  {
    ErrPrint("Unable to prepare memory for descriptor ring, size=%d\n",
      ring_header->size);
    return kIOReturnCannotWire;
  }

  ring_header->desc = ring_header->dbuf->getBytesNoCopy();
#ifdef __LP64__
  ring_header->dma =
    ring_header->dbuf->getPhysicalSegment(0, 0);
#else // __ILP32__
  ring_header->dma =
    ring_header->dbuf->getPhysicalSegment(0, 0, kIOMemoryMapperNone);
#endif // __ILP32__
  memset(ring_header->desc, 0, ring_header->size);
  ring_header->used = ALIGN(ring_header->dma, 8) - ring_header->dma;

  DbgPrint(3, "ring header: size = %d, used= %d\n",
           ring_header->size, ring_header->used);

  // Allocate transmit descriptors.
  if (ALX_allocateAllTxDescriptors() != kIOReturnSuccess)
  {
    ALX_freeAllTxDescriptors();
    return kIOReturnNoMemory;
  }

  // Allocate receive descriptors.
  if (ALX_allocateAllRxDescriptors() != kIOReturnSuccess)
  {
    ALX_freeAllTxDescriptors();
    ALX_freeAllRxDescriptors();
    return kIOReturnNoMemory;
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::ALX_allocateAllQueues()
{
  int que_idx;

  for (que_idx = 0; que_idx < ALX_Adapter.num_txques; que_idx++)
  {
    struct alx_tx_queue *txque = ALX_Adapter.tx_queue[que_idx];

    txque = (struct alx_tx_queue *)kzalloc(sizeof(struct alx_tx_queue),
                                             GFP_KERNEL);

    if (!txque)
    {
      ErrPrint("Error allocating Tx queues.\n");

      for (que_idx = 0; que_idx < ALX_Adapter.num_txques; que_idx++)
      {
        kfree(ALX_Adapter.tx_queue[que_idx]);
      }

      return kIOReturnNoMemory;
    }

    txque->tpq.count = ALX_Adapter.num_txdescs;
    txque->que_idx = que_idx;

    ALX_Adapter.tx_queue[que_idx] = txque;
  }

  for (que_idx = 0; que_idx < ALX_Adapter.num_rxques; que_idx++)
  {
    struct alx_rx_queue *rxque = ALX_Adapter.rx_queue[que_idx];

    rxque = (struct alx_rx_queue *)kzalloc(sizeof(struct alx_rx_queue),
                                             GFP_KERNEL);

    if (!rxque)
    {
      ErrPrint("Error allocating Rx queues.\n");

      for (que_idx = 0; que_idx < ALX_Adapter.num_txques; que_idx++)
      {
        kfree(ALX_Adapter.tx_queue[que_idx]);
      }

      for (que_idx = 0; que_idx < ALX_Adapter.num_rxques; que_idx++)
      {
        kfree(ALX_Adapter.rx_queue[que_idx]);
      }

      return kIOReturnNoMemory;
    }

    rxque->rrq.count = ALX_Adapter.num_rxdescs;
    rxque->rfq.count = ALX_Adapter.num_rxdescs;
    rxque->swq.count = ALX_Adapter.num_rxdescs;
    rxque->que_idx = que_idx;

    if (ALX_checkAdapterFlag(SRSS_EN))
    {
      if (que_idx < ALX_Adapter.num_hw_rxques)
      {
        SET_RX_FLAG(HW_QUE);
      }

      if (que_idx < ALX_Adapter.num_sw_rxques)
      {
        SET_RX_FLAG(SW_QUE);
      }
    }
    else
    {
      SET_RX_FLAG(HW_QUE);
    }

    ALX_Adapter.rx_queue[que_idx] = rxque;
  }

  DbgPrint(3, "num_tx_descs = %d, num_rx_descs = %d\n",
           ALX_Adapter.num_txdescs, ALX_Adapter.num_rxdescs);

  return kIOReturnSuccess;
}

//==============================================================================

// Allocate all Rx descriptors.
IOReturn
CLASS::ALX_allocateAllRxDescriptors()
{
  uint i;

  for (i = 0; i < ALX_Adapter.num_rxques; i++)
  {
    if (ALX_allocateRxDescriptor(ALX_Adapter.rx_queue[i]) != kIOReturnSuccess)
    {
      ErrPrint("Allocation of Rx queue #%u failed.\n", i);
      return kIOReturnNoMemory;
    }
  }

  return kIOReturnSuccess;
}

//==============================================================================

// Allocate all Tx descriptors.
IOReturn
CLASS::ALX_allocateAllTxDescriptors()
{
  uint i;
  DbgPrint(3, "num_tques = %d\n", ALX_Adapter.num_txques);

  for (i = 0; i < ALX_Adapter.num_txques; i++)
  {
    if (ALX_allocateTxDescriptor(ALX_Adapter.tx_queue[i]) != kIOReturnSuccess)
    {
      ErrPrint("Allocation of Tx queue #%u failed.\n", i);
      return kIOReturnNoMemory;
    }
  }

  return kIOReturnSuccess;
}

//==============================================================================

// Allocate individual Rx descriptors
IOReturn
CLASS::ALX_allocateRxDescriptor(struct alx_rx_queue *rxque)
{
  struct alx_ring_header *ring_header = &ALX_Adapter.ring_header;
  struct alx_hw *hw = &ALX_Adapter.hw;
  u16 que_idx = rxque->que_idx;
  int size;

  DbgPrint(3, "RRD.count = %d, RFD.count = %d, SWD.count = %d\n",
           rxque->rrq.count, rxque->rfq.count, rxque->swq.count);

  if (CHK_RX_FLAG(HW_QUE))
  {
    // Allocate buffer info.
    size = sizeof(struct alx_buffer) * rxque->rfq.count;
    rxque->rfq.rfbuff = (struct alx_buffer *)kzalloc(size, GFP_KERNEL);

    if (!rxque->rfq.rfbuff)
    {
      ErrPrint("Unable to allocate memory for HW Rx descriptor.\n");
      return kIOReturnNoMemory;
    }

    // Set DMA point of RRQ and RFQ.
    // Round up to nearest 4K.
    rxque->rrq.size =
      rxque->rrq.count * sizeof(union alx_hw_rrdesc);
    rxque->rfq.size =
      rxque->rfq.count * sizeof(union alx_hw_rfdesc);

    rxque->rrq.rrdma = ring_header->dma + ring_header->used;
    rxque->rrq.rrdesc = (union alx_hw_rrdesc *)((char *)ring_header->desc +
                        ring_header->used);
    ring_header->used += ALIGN(rxque->rrq.size, 8);

    rxque->rfq.rfdma = (dma_addr_t)((char *)ring_header->dma +
                                    ring_header->used);
    rxque->rfq.rfdesc = (union alx_hw_rfdesc *)((char *)ring_header->desc +
                        ring_header->used);
    ring_header->used += ALIGN(rxque->rfq.size, 8);

    hw->dma.rrdmem_hi[que_idx] = ALX_DMA_ADDR_HI(rxque->rrq.rrdma);
    hw->dma.rrdmem_lo[que_idx] = ALX_DMA_ADDR_LO(rxque->rrq.rrdma);
    hw->dma.rfdmem_hi[que_idx] = ALX_DMA_ADDR_HI(rxque->rfq.rfdma);
    hw->dma.rfdmem_lo[que_idx] = ALX_DMA_ADDR_LO(rxque->rfq.rfdma);

    // Clean all counts within rxque.
    rxque->rrq.produce_idx = 0;
    rxque->rrq.consume_idx = 0;

    rxque->rfq.produce_idx = 0;
    rxque->rfq.consume_idx = 0;
  }

  if (CHK_RX_FLAG(SW_QUE))
  {
    size = sizeof(struct alx_sw_buffer) * rxque->swq.count;
    rxque->swq.swbuff = (struct alx_sw_buffer *)kzalloc(size, GFP_KERNEL);

    if (!rxque->swq.swbuff)
    {
      kfree(rxque->rfq.rfbuff);
      rxque->rfq.rfbuff = NULL;
      ErrPrint("Unable to allocate memory for SW Rx descriptor.\n");
      return kIOReturnNoMemory;
    }

    rxque->swq.consume_idx = 0;
    rxque->swq.produce_idx = 0;
  }

  rxque->max_packets = rxque->rrq.count / 2;

  return kIOReturnSuccess;
}

//==============================================================================

// Allocate individual Tx descriptors.
IOReturn
CLASS::ALX_allocateTxDescriptor(struct alx_tx_queue *txque)
{
  struct alx_ring_header *ring_header = &ALX_Adapter.ring_header;
  struct alx_hw *hw = &ALX_Adapter.hw;
  u16 que_idx = txque->que_idx;
  int size;

  DbgPrint(3, "tpq.count = %d\n", txque->tpq.count);

  size = sizeof(struct alx_buffer) * txque->tpq.count;
  txque->tpq.tpbuff = (struct alx_buffer *)kzalloc(size, GFP_KERNEL);

  if (!txque->tpq.tpbuff)
  {
    ErrPrint("Unable to allocate memory for the Tx descriptor.\n");
    return kIOReturnNoMemory;
  }

  // Round up to nearest 4K.
  txque->tpq.size = txque->tpq.count * sizeof(union alx_hw_tpdesc);

  txque->tpq.tpdma = ring_header->dma + ring_header->used;
  txque->tpq.tpdesc = (union alx_hw_tpdesc *)((char *)ring_header->desc +
                      ring_header->used);
  ring_header->used += ALIGN(txque->tpq.size, 8);

  hw->dma.tpdmem_hi[que_idx] = ALX_DMA_ADDR_HI(txque->tpq.tpdma);
  hw->dma.tpdmem_lo[que_idx] = ALX_DMA_ADDR_LO(txque->tpq.tpdma);

  txque->tpq.produce_idx = 0;
  txque->tpq.consume_idx = 0;
  txque->max_packets = txque->tpq.count;

  return kIOReturnSuccess;
}

//==============================================================================

void
CLASS::ALX_checkLinkStatusChange()
{
  ALX_setAdapterFlag(TASK_LSC_REQ);
  clock_interval_to_deadline(
    ALX_TRY_LINK_TIMEOUT, kMillisecondScale, &ALX_LinkTimeout);

  if (!ALX_checkAdapterFlag(STATE_DOWN))
  {
    ALX_taskSchedule();
  }
}

//==============================================================================

// Calculate the transmit packet descriptors needed.
bool
CLASS::ALX_checkNumberTxDescriptors(struct alx_tx_queue *txque, const mbuf_t m)
{
  u16 num_required = 1;
  u16 num_available = 0;
  u16 produce_idx = txque->tpq.produce_idx;
  u16 consume_idx = txque->tpq.consume_idx;
  mbuf_t next_m = m;
  // TODO: TSO.
#if 0
  u16 proto_hdr_len = 0;

  if (mbuf_is_gso(m))
  {
    proto_hdr_len = skb_transport_offset(m) + tcp_hdrlen(m);

    if (proto_hdr_len < skb_headlen(m))
    {
      num_required++;
    }

    if (skb_shinfo(m)->gso_type & SKB_GSO_TCPV6)
    {
      num_required++;
    }
  }

#endif

  while (next_m != NULL)
  {
    num_required++;
    next_m = mbuf_next(next_m);
  }

  num_available = (u16)(consume_idx > produce_idx) ?
                  (consume_idx - produce_idx - 1) :
                  (txque->tpq.count + consume_idx - produce_idx - 1);

  return num_required < num_available;
}

//==============================================================================

void
CLASS::ALX_cleanAllRxQueues()
{
  int i;

  for (i = 0; i < ALX_Adapter.num_rxques; i++)
  {
    ALX_cleanRxQueue(ALX_Adapter.rx_queue[i]);
  }
}

//==============================================================================

void
CLASS::ALX_cleanAllTxQueues()
{
  int i;

  for (i = 0; i < ALX_Adapter.num_txques; i++)
  {
    ALX_cleanTxQueue(ALX_Adapter.tx_queue[i]);
  }
}

//==============================================================================

void
CLASS::ALX_cleanBuffer(struct alx_buffer *buf)
{
  if (buf->dseg != NULL)
  {
    kfree(buf->dseg);
    buf->dseg = NULL;
  }

  if (buf->dcom != NULL)
  {
    buf->dcom->clearMemoryDescriptor();
    buf->dcom->release();
    buf->dcom = NULL;
  }

  if (buf->dbuf != NULL)
  {
    buf->dbuf->complete();
    buf->dbuf->release();
    buf->dbuf = NULL;
  }

  if (buf->dma)
  {
    buf->dma = NULL;
  }
}

//==============================================================================

void
CLASS::ALX_cleanRFDescriptor(struct alx_rx_queue *rxque,
                               union alx_sw_rrdesc *srrd)
{
  u32 consume_idx = srrd->genr.si;
  u32 i;

  for (i = 0; i < srrd->genr.nor; i++)
  {
    if (++consume_idx == rxque->rfq.count)
    {
      consume_idx = 0;
    }
  }

  rxque->rfq.consume_idx = consume_idx;
}

//==============================================================================

void
CLASS::ALX_cleanRxQueue(struct alx_rx_queue *rxque)
{
  unsigned long size;
  int i;

  if (CHK_RX_FLAG(HW_QUE))
  {
    // Ring already cleared, nothing to do.
    if (!rxque->rfq.rfbuff)
    {
      goto clean_sw_queue;
    }

    for (i = 0; i < rxque->rfq.count; i++)
    {
      struct alx_buffer *rfbuf;
      rfbuf = GET_RF_BUFFER(rxque, i);
      ALX_cleanBuffer(rfbuf);
    }

    size =  sizeof(struct alx_buffer) * rxque->rfq.count;
    memset(rxque->rfq.rfbuff, 0, size);

    // Zero out the descriptor ring.
    memset(rxque->rrq.rrdesc, 0, rxque->rrq.size);
    rxque->rrq.produce_idx = 0;
    rxque->rrq.consume_idx = 0;

    memset(rxque->rfq.rfdesc, 0, rxque->rfq.size);
    rxque->rfq.produce_idx = 0;
    rxque->rfq.consume_idx = 0;
  }

  clean_sw_queue:

  if (CHK_RX_FLAG(SW_QUE))
  {
    // Ring already cleared, nothing to do.
    if (!rxque->swq.swbuff)
    {
      return;
    }

    for (i = 0; i < rxque->swq.count; i++)
    {
      struct alx_sw_buffer *swbuf;
      swbuf = GET_SW_BUFFER(rxque, i);

      // SWQ doesn't map DMA.

      if (swbuf->packet)
      {
        mbuf_freem_list(swbuf->packet);
        swbuf->packet = NULL;
      }
    }

    size =  sizeof(struct alx_buffer) * rxque->swq.count;
    memset(rxque->swq.swbuff, 0, size);

    // SWQ doesn't have any descriptor rings.
    rxque->swq.produce_idx = 0;
    rxque->swq.consume_idx = 0;
  }
}

//==============================================================================

void
CLASS::ALX_cleanTxQueue(struct alx_tx_queue *txque)
{
  unsigned long size;
  UInt16 i;

  // Ring already cleared, nothing to do.
  if (!txque->tpq.tpbuff)
  {
    return;
  }

  for (i = 0; i < txque->tpq.count; i++)
  {
    struct alx_buffer *tpbuf;
    tpbuf = GET_TP_BUFFER(txque, i);
    ALX_cleanBuffer(tpbuf);
  }

  size = sizeof(struct alx_buffer) * txque->tpq.count;
  memset(txque->tpq.tpbuff, 0, size);

  // Zero out the Tx buffers.
  memset(txque->tpq.tpdesc, 0, txque->tpq.size);

  txque->tpq.consume_idx = 0;
  txque->tpq.produce_idx = 0;
}

//==============================================================================

void
CLASS::ALX_configureRSS()
{
  static const u8 key[40] =
  {
    0xE2, 0x91, 0xD7, 0x3D, 0x18, 0x05, 0xEC, 0x6C,
    0x2A, 0x94, 0xB3, 0x0D, 0xA5, 0x4F, 0x2B, 0xEC,
    0xEA, 0x49, 0xAF, 0x7C, 0xE2, 0x14, 0xAD, 0x3D,
    0xB8, 0x55, 0xAA, 0xBE, 0x6A, 0x3E, 0x67, 0xEA,
    0x14, 0x36, 0x4D, 0x17, 0x3B, 0xED, 0x20, 0x0D
  };

  struct alx_hw *hw = &ALX_Adapter.hw;
  u32 reta = 0;
  int i, j;

  // Initialise RSS hash type and IDT table size.
  hw->rss_hstype = ALX_RSS_HSTYP_ALL_EN;
  hw->rss_idt_size = 0x100;

  // Fill out redirection table.
  memcpy(hw->rss_key, key, sizeof(hw->rss_key));

  // Fill out redirection table.
  memset(hw->rss_idt, 0x0, sizeof(hw->rss_idt));

  for (i = 0, j = 0; i < 256; i++, j++)
  {
    if (j == ALX_Adapter.max_rxques)
    {
      j = 0;
    }

    reta |= (j << ((i & 7) * 4));

    if ((i & 7) == 7)
    {
      hw->rss_idt[i >> 3] = reta;
      reta = 0;
    }
  }

  if (hw->cbs.config_rss)
  {
    hw->cbs.config_rss(hw, ALX_checkAdapterFlag(SRSS_EN));
  }
}

//==============================================================================

IOReturn
CLASS::ALX_disable(u32 ctrl)
{
  struct alx_hw *hw = &ALX_Adapter.hw;

  ALX_setAdapterFlag(STATE_DOWN);
  ALX_Enabled = false;

  ALX_TransmitQueue->stop();
  ALX_TransmitQueueEnabled = false;
  setLinkStatus(0);
  ALX_LinkEnabled = false;
  ALX_TransmitQueue->setCapacity(0);
  ALX_NetStats->outputErrors = ALX_TransmitQueue->flush();

  ALX_disableInterrupt();

  // TODO: MSI-X if possible.
#if 0

  if (ctrl & ALX_OPEN_CTRL_IRQ_EN)
  {
    ALX_freeIRQ();
  }

#endif

  ALX_clearAdapterFlag(TASK_LSC_REQ);
  ALX_clearAdapterFlag(TASK_REINIT_REQ);
  ALX_clearAdapterFlag(STATE_WATCH_DOG);

  if (ctrl & ALX_OPEN_CTRL_RESET_PHY)
  {
    hw->cbs.reset_phy(hw);
  }

  if (ctrl & ALX_OPEN_CTRL_RESET_MAC)
  {
    hw->cbs.reset_mac(hw);
  }

  ALX_Adapter.hw.link_speed = 0;

  ALX_cleanAllTxQueues();
  ALX_cleanAllRxQueues();

  return kIOReturnSuccess;
}

//==============================================================================

// Mask off interrupt generation on the NIC.
void
CLASS::ALX_disableInterrupt()
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  int i;

  atomic_inc(&ALX_Adapter.irq_sem);

  if (hw->cbs.disable_legacy_intr)
  {
    hw->cbs.disable_legacy_intr(hw);
  }

  if (ALX_checkAdapterFlag(MSIX_EN))
  {
    for (i = 0; i < ALX_Adapter.num_msix_intrs; i++)
    {
      // FIXME: synchronize_irq equivalent.
      // synchronize_irq(ALX_Adapter.msix_entries[i].vector);
      hw->cbs.disable_msix_intr(hw, i);
    }
  }
  else
  {
    // synchronize_irq(ALX_Adapter.pdev->irq);
  }
}

//==============================================================================

IOReturn
CLASS::ALX_enable(UInt32 ctrl)
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  int retval = 0;
  int i;

  if (ALX_initRingPointers() != kIOReturnSuccess)
  {
    ErrPrint("Unable to initialise ring pointers.\n");
    return kIOReturnError;
  }

  ALX_setMulticastList();
  ALX_restoreVLAN();

  if (hw->cbs.init_mac)
  {
    retval = hw->cbs.init_mac(hw, ALX_Adapter.rxbuf_size,
                              ALX_Adapter.num_hw_rxques, ALX_Adapter.num_rxdescs,
                              ALX_Adapter.num_txques, ALX_Adapter.num_txdescs);
  }

  if (hw->cbs.config_tx)
  {
    hw->cbs.config_tx(hw);
  }

  if (hw->cbs.config_rx)
  {
    retval = hw->cbs.config_rx(hw);
  }

  ALX_configureRSS();

  for (i = 0; i < ALX_Adapter.num_hw_rxques; i++)
  {
    ALX_refreshRxBuffer(ALX_Adapter.rx_queue[i]);
  }

  // Configure HW registers of MSIX.
  if (hw->cbs.config_msix)
  {
    retval = hw->cbs.config_msix(hw, ALX_Adapter.num_msix_intrs,
                                 ALX_checkAdapterFlag(MSIX_EN), ALX_checkAdapterFlag(MSI_EN));
  }

  // TODO: MSI-X if possible.
#if 0

  if (ctrl & ALX_OPEN_CTRL_IRQ_EN)
  {
    retval = ALX_requestIRQ();

    if (retval)
    {
      ALX_cleanAllRxQueues();
      return kIOReturnNoInterrupt;
    }
  }

#endif

  // Enable INTR and TX.
  ALX_enableInterrupt();

  ALX_TransmitQueue->setCapacity(ALX_TransmitQueueCapacity);
  ALX_TransmitQueue->start();
  ALX_TransmitQueueEnabled = true;

  ALX_clearAdapterFlag(STATE_DOWN);
  ALX_Enabled = true;

  ALX_setAdapterFlag(TASK_LSC_REQ);
  clock_interval_to_deadline(
    ALX_TRY_LINK_TIMEOUT, kMillisecondScale, &ALX_LinkTimeout);

  return kIOReturnSuccess;
}

//==============================================================================

// Enable default interrupt generation settings.
void
CLASS::ALX_enableInterrupt()
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  int i;

  if (!atomic_dec_and_test(&ALX_Adapter.irq_sem))
  {
    return;
  }

  if (hw->cbs.enable_legacy_intr)
  {
    hw->cbs.enable_legacy_intr(hw);
  }

  // Enable all MSIX IRQs.
  if (ALX_checkAdapterFlag(MSIX_EN))
  {
    for (i = 0; i < ALX_Adapter.num_msix_intrs; i++)
    {
      if (hw->cbs.disable_msix_intr)
      {
        hw->cbs.disable_msix_intr(hw, i);
      }

      if (hw->cbs.enable_msix_intr)
      {
        hw->cbs.enable_msix_intr(hw, i);
      }
    }
  }
}

//==============================================================================

void
CLASS::ALX_freeAllDescriptors()
{
  struct alx_ring_header *ring_header = &ALX_Adapter.ring_header;

  ALX_freeAllTxDescriptors();
  ALX_freeAllRxDescriptors();

  ring_header->dma = NULL;
  ring_header->desc = NULL;
  ring_header->size = ring_header->used = 0;

  if (ring_header->dbuf != NULL)
  {
    ring_header->dbuf->complete();
    ring_header->dbuf->release();
    ring_header->dbuf = NULL;
  }
}

//==============================================================================

void
CLASS::ALX_freeAllQueues()
{
  int que_idx;

  for (que_idx = 0; que_idx < ALX_Adapter.num_txques; que_idx++)
  {
    kfree(ALX_Adapter.tx_queue[que_idx]);
    ALX_Adapter.tx_queue[que_idx] = NULL;
  }

  for (que_idx = 0; que_idx < ALX_Adapter.num_rxques; que_idx++)
  {
    kfree(ALX_Adapter.rx_queue[que_idx]);
    ALX_Adapter.rx_queue[que_idx] = NULL;
  }
}

//==============================================================================

void
CLASS::ALX_freeAllRxDescriptors()
{
  int i;

  for (i = 0; i < ALX_Adapter.num_rxques; i++)
  {
    ALX_freeRxDescriptor(ALX_Adapter.rx_queue[i]);
  }
}

//==============================================================================

void
CLASS::ALX_freeAllTxDescriptors()
{
  int i;

  for (i = 0; i < ALX_Adapter.num_txques; i++)
  {
    ALX_freeTxDescriptor(ALX_Adapter.tx_queue[i]);
  }
}

//==============================================================================

void
CLASS::ALX_freeRxDescriptor(struct alx_rx_queue *rxque)
{
  ALX_cleanRxQueue(rxque);

  if (CHK_RX_FLAG(HW_QUE))
  {
    kfree(rxque->rfq.rfbuff);
    rxque->rfq.rfbuff = NULL;

    // If not set, then don't free.
    if (!rxque->rrq.rrdesc)
    {
      return;
    }

    rxque->rrq.rrdesc = NULL;

    if (!rxque->rfq.rfdesc)
    {
      return;
    }

    rxque->rfq.rfdesc = NULL;
  }

  if (CHK_RX_FLAG(SW_QUE))
  {
    kfree(rxque->swq.swbuff);
    rxque->swq.swbuff = NULL;
  }
}

//==============================================================================

void
CLASS::ALX_freeTxDescriptor(struct alx_tx_queue *txque)
{
  ALX_cleanTxQueue(txque);

  kfree(txque->tpq.tpbuff);
  txque->tpq.tpbuff = NULL;

  // If not set, then don't free.
  if (!txque->tpq.tpdesc)
  {
    return;
  }

  txque->tpq.tpdesc = NULL;
}

//==============================================================================

IOReturn
CLASS::ALX_getInterruptSource()
{
  // Search for interrupt index of MSI type.
  int msi_index = -1;
  int intr_index = 0, intr_type = 0;
  IOReturn intr_ret;

  if (ALX_PCIDevice == NULL)
  {
    return kIOReturnError;
  }

  while (true)
  {
    intr_ret = ALX_PCIDevice->getInterruptType(intr_index, &intr_type);

    if (intr_ret != kIOReturnSuccess)
    {
      break;
    }

    if (intr_type & kIOInterruptTypePCIMessaged)
    {
      msi_index = intr_index;
      break;
    }

    intr_index++;
  }

  if (msi_index != -1)
  {
    DbgPrint(3, "MSI interrupt index: %d\n", msi_index);
    ALX_InterruptSource = IOInterruptEventSource::interruptEventSource(this,
                          OSMemberFunctionCast(IOInterruptEventSource::Action, this,
                              &CLASS::ALX_interrupt), ALX_PCIDevice, msi_index);
  }

  if (msi_index == -1 || ALX_InterruptSource == NULL)
  {
    DbgPrint(1, "Warning: MSI index was not found or MSI "
             "interrupt could not be enabled.\n");
    ALX_InterruptSource = IOInterruptEventSource::interruptEventSource(this,
                          OSMemberFunctionCast(IOInterruptEventSource::Action, this,
                              &CLASS::ALX_interrupt), ALX_PCIDevice);
  }

  return kIOReturnSuccess;
}

//==============================================================================

//  Driver  Model-name    vendor:device           Type
//  alc     AR8131        1969:1063     l1c       Gigabit Ethernet
//  alc     AR8132        1969:1062     l2c       Fast Ethernet
//  alc     AR8151(v1.0)  1969:1073     l1d_v1    Gigabit Ethernet
//  alc     AR8151(v2.0)  1969:1083     l1d_v2    Gigabit Ethernet
//  alc     AR8152(v1.1)  1969:2060     l2cb_v1   Fast Ethernet
//  alc     AR8152(v2.0)  1969:2062     l2cb_v2   Fast Ethernet
//  alf     AR8161        1969:1091     l1f       Gigabit Ethernet
//  alf     AR8162        1969:1090     l2f       Fast Ethernet
enum ALX_speedType
CLASS::ALX_getNICType()
{
  return ALX_deviceTable[ALX_DeviceTableIndex].type;
}

//==============================================================================

bool
CLASS::ALX_getRRDescriptor(struct alx_rx_queue *rxque,
                             union alx_sw_rrdesc *srrd)
{
  union alx_hw_rrdesc *hrrd = ALX_RRD(rxque, rxque->rrq.consume_idx);

  srrd->dfmt.dw3 = le32_to_cpu(hrrd->dfmt.dw3);

  if (!srrd->genr.update)
  {
    return false;
  }

  srrd->dfmt.dw0 = le32_to_cpu(hrrd->dfmt.dw0);
  srrd->dfmt.dw1 = le32_to_cpu(hrrd->dfmt.dw1);
  srrd->dfmt.dw2 = le32_to_cpu(hrrd->dfmt.dw2);

  srrd->genr.update = 0;
  hrrd->dfmt.dw3 = cpu_to_le32(srrd->dfmt.dw3);

  if (++rxque->rrq.consume_idx == rxque->rrq.count)
  {
    rxque->rrq.consume_idx = 0;
  }

  return true;
}

//==============================================================================

IOReturn
CLASS::ALX_handleRxIRQ(struct alx_rx_queue *rxque)
{
  struct alx_hw *hw = &ALX_Adapter.hw;

  union alx_sw_rrdesc srrd;
  struct alx_buffer *rfbuf;
  mbuf_t m;

  u16 consume_idx, hw_consume_idx, num_consume_pkts;
  u16 count = 0;

  alx_mem_r16(hw, rxque->consume_reg, &hw_consume_idx);
  num_consume_pkts = (hw_consume_idx >= rxque->rrq.consume_idx) ?
                     (hw_consume_idx -  rxque->rrq.consume_idx) :
                     (hw_consume_idx + rxque->rrq.count - rxque->rrq.consume_idx);

  while (1)
  {
    if (!num_consume_pkts)
    {
      break;
    }

    consume_idx = rxque->rrq.consume_idx;

    if (!ALX_getRRDescriptor(rxque, &srrd))
    {
      break;
    }

    if (srrd.genr.res || srrd.genr.lene)
    {
      ALX_cleanRFDescriptor(rxque, &srrd);
      DbgPrint(1, "Wrong packet! rrd->word3 is 0x%08x\n", (uint)srrd.dfmt.dw3);
      continue;
    }

    // Bad Receive.
    if (srrd.genr.nor != 1)
    {
      // TODO: Multi-RFD.
      ErrPrint("Multi-RFD not supported yet.\n");
      break;
    }

    rfbuf = GET_RF_BUFFER(rxque, srrd.genr.si);
    rfbuf->dma = 0;
    ALX_cleanRFDescriptor(rxque, &srrd);

    m = allocatePacket(srrd.genr.pkt_len - ETHER_CRC_LEN + ETHER_ALIGN);
    mbuf_adj(m, ETHER_ALIGN);
    rfbuf->dcom->synchronize(kIODirectionIn);
    mbuf_copyback(m, 0, srrd.genr.pkt_len - ETHER_CRC_LEN,
                  rfbuf->dbuf->getBytesNoCopy(), MBUF_DONTWAIT);
    mbuf_pkthdr_setlen(m, srrd.genr.pkt_len - ETHER_CRC_LEN);

    ALX_receiveMbuf(m, (u16)srrd.genr.vlan_tag, (bool)srrd.genr.vlan_flag);

    num_consume_pkts--;
    count++;
  }

  if (count)
  {
    ALX_NetStats->inputPackets += ALX_NetIf->flushInputQueue();

    ALX_refreshRxBuffer(rxque);
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::ALX_handleTxIRQ(struct alx_tx_queue *txque)
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  struct alx_buffer *tpbuf;
  u16 consume_data;

  alx_mem_r16(hw, txque->consume_reg, &consume_data);
  DbgPrint(5, "TX[%d]: consume_reg[0x%x] = 0x%x, tpq.consume_idx = 0x%x\n",
           txque->que_idx, txque->consume_reg, consume_data, txque->tpq.consume_idx);

  while (txque->tpq.consume_idx != consume_data)
  {
    tpbuf = GET_TP_BUFFER(txque, txque->tpq.consume_idx);

    if (tpbuf->dma)
    {
      tpbuf->dma = NULL;
    }

    if (++txque->tpq.consume_idx == txque->tpq.count)
    {
      txque->tpq.consume_idx = 0;
    }
  }

  if ((ALX_TransmitQueueEnabled == false) && (ALX_Enabled == true))
  {
    ALX_TransmitQueue->start();
    ALX_TransmitQueueEnabled = true;
  }

  return true;
}

//==============================================================================

IOReturn
CLASS::ALX_init()
{
  DbgPrint(4, "ALX_init()\n");

  struct alx_hw *hw = &ALX_Adapter.hw;
  UInt8 eth_addr[ETHER_ADDR_LEN];
  int retval;

  ALX_PCIDevice->setBusMasterEnable(true);
  ALX_PCIDevice->setMemoryEnable(true);
  ALX_PCIDevice->setIOEnable(false);
  ALX_VendorID = ALX_PCIDevice->configRead16(kIOPCIConfigVendorID);
  ALX_DeviceID = ALX_PCIDevice->configRead16(kIOPCIConfigDeviceID);

  DbgPrint(2, "Vendor ID: %#x Device ID: %#x\n", ALX_VendorID, ALX_DeviceID);

  ALX_PCIRegMap = ALX_PCIDevice->mapDeviceMemoryWithRegister(
                    kIOPCIConfigBaseAddress0);

  if (ALX_PCIRegMap == NULL)
  {
    ErrPrint("Unable to map physical PCI memory!\n");
    return kIOReturnDeviceError;
  }

#ifdef __LP64__
  DbgPrint(2, "PCI memory mapped to bus address %#018llx, "
           "virtual address %#018llx, length %llu bytes.\n",
           ALX_PCIRegMap->getPhysicalAddress(),
           ALX_PCIRegMap->getVirtualAddress(),
           ALX_PCIRegMap->getLength());
#else // __ILP32__
  DbgPrint(2, "PCI memory mapped to bus address %#010lx, "
           "virtual address %#010x, length %lu bytes.\n",
           ALX_PCIRegMap->getPhysicalAddress(),
           ALX_PCIRegMap->getVirtualAddress(),
           ALX_PCIRegMap->getLength());
#endif // __ILP32__

  ALX_Adapter.msg_enable = ALX_MSG_DEFAULT;
  ALX_Adapter.pdev = ALX_PCIDevice;
  ALX_Adapter.hw.hw_addr =
    reinterpret_cast<UInt8 *>(ALX_PCIRegMap->getAddress());

  // Setup the empty adapter structure.
  if (ALX_initAdapter() != kIOReturnSuccess)
  {
    ErrPrint("Unable to initialise ALX adapter structure.\n");
    return kIOReturnError;
  }

  // Reset PCI-E.
  retval = hw->cbs.reset_pcie(hw, true, true);

  if (retval != 0)
  {
    ErrPrint("PCI-E reset failed, error = %d.\n", retval);
    return kIOReturnError;
  }

  ALX_initPCIPowerManagement();

  // Init GPHY as early as possible due to power saving issue.
  retval = hw->cbs.reset_phy(hw);

  if (retval != 0)
  {
    ErrPrint("PHY reset failed, error = %d.\n", retval);
    return kIOReturnError;
  }

  // Reset MAC.
  retval = hw->cbs.reset_mac(hw);

  if (retval)
  {
    ErrPrint("MAC reset failed, error = %d.\n", retval);
    return kIOReturnError;
  }

  // Setup link to put it in a known good starting state.
  retval = hw->cbs.setup_phy_link(hw, hw->autoneg_advertised, true,
                                  !hw->disable_fc_autoneg);

  // Get user settings.
  // FIXME: Figure out how many CPUs are present.
  // CPU number is then used for second parameter of the two min_t calls
  // below. Otherwise leave at 1 and let the user set more.
  ALX_Adapter.num_txdescs = ALX_TX_DESCS;
  ALX_Adapter.num_rxdescs = ALX_RX_DESCS;
  ALX_Adapter.max_rxques = min_t(int, ALX_MAX_RX_QUEUES, num_online_cpus());
  ALX_Adapter.max_txques = min_t(int, ALX_MAX_TX_QUEUES, num_online_cpus());

  // Get MAC address and permanent MAC address, set to register.
  if (hw->cbs.get_mac_addr)
  {
    retval = hw->cbs.get_mac_addr(hw, hw->mac_perm_addr);
  }
  else
  {
    retval = -1;
  }

  if (retval != 0)
  {
    eth_random_addr(eth_addr);
    memcpy(hw->mac_perm_addr, eth_addr, ETHER_ADDR_LEN);
  }

  memcpy(hw->mac_addr, hw->mac_perm_addr, ETHER_ADDR_LEN);

  if (hw->cbs.set_mac_addr)
  {
    hw->cbs.set_mac_addr(hw, hw->mac_addr);
  }

  retval = ALX_validateMACAddress(hw->mac_perm_addr);

  if (retval != 0)
  {
    ErrPrint("Invalid MAC address.\n");
    return kIOReturnError;
  }

  ALX_setQueueNumber();
  ALX_Adapter.num_msix_intrs = 0;

  if (ALX_allocateAllQueues() != kIOReturnSuccess)
  {
    ErrPrint("Unable to allocate Rx/Tx queues.\n");
  }

  ALX_setRegisterInfoSpecial();

  switch (ALX_DeviceID)
  {
  case ALX_DEV_ID_AR8131:
  case ALX_DEV_ID_AR8132:
  case ALX_DEV_ID_AR8151_V1:
  case ALX_DEV_ID_AR8151_V2:
  case ALX_DEV_ID_AR8152_V1:
  case ALX_DEV_ID_AR8152_V2:
    ALX_Adapter.wol = (ALX_WOL_MAGIC | ALX_WOL_PHY);
    break;

  case ALX_DEV_ID_AR8161:
  case ALX_DEV_ID_AR8162:
  case ALX_DEV_ID_AR8171:
  case ALX_DEV_ID_AR8172:
    ALX_Adapter.wol = (ALX_WOL_MAGIC | ALX_WOL_PHY);
    break;

  default:
    ALX_Adapter.wol = 0;
    break;
  }

  ALX_setAdapterFlag(STATE_DOWN);

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::ALX_initAdapter()
{
  int max_frame;
  struct alx_hw *hw = &ALX_Adapter.hw;

  /* PCI config space info */
  ALX_Adapter.hw.pci_venid = ALX_VendorID;
  ALX_Adapter.hw.pci_devid = ALX_DeviceID;
  ALX_Adapter.hw.pci_revid = ALX_PCIDevice->configRead8(kIOPCIConfigRevisionID);
  ALX_Adapter.hw.pci_sub_venid = ALX_PCIDevice->configRead16(
                                   kIOPCIConfigSubSystemVendorID);
  ALX_Adapter.hw.pci_sub_devid = ALX_PCIDevice->configRead16(
                                   kIOPCIConfigSubSystemID);

  ALX_patchAssign();

  if (ALX_initHWCallbacks() != kIOReturnSuccess)
  {
    ErrPrint("Unable to set HW function pointers.\n");
    return kIOReturnError;
  }

  if (hw->cbs.identify_nic(hw) != 0)
  {
    ErrPrint("HW is disabled.\n");
    return kIOReturnError;
  }

  // Set adapter flags.
  switch (hw->mac_type)
  {
  case alx_mac_l1f:
  case alx_mac_l2f:
  case alx_mac_l1h:
  case alx_mac_l2h:
#if 0
#ifdef CONFIG_ALX_MSI
    ALX_setAdapterFlag(MSI_CAP);
#endif
#ifdef CONFIG_ALX_MSIX
    ALX_setAdapterFlag(MSIX_CAP);
#endif

    if (ALX_checkAdapterFlag(MSIX_CAP))
    {
      ALX_setAdapterFlag(FIXED_MSIX);
      ALX_setAdapterFlag(MRQ_CAP);
#ifdef CONFIG_ALX_RSS
      ALX_setAdapterFlag(SRSS_CAP);
#endif
    }

    break;
#endif

  case alx_mac_l1c:
  case alx_mac_l1d_v1:
  case alx_mac_l1d_v2:
  case alx_mac_l2c:
  case alx_mac_l2cb_v1:
  case alx_mac_l2cb_v20:
  case alx_mac_l2cb_v21:
#ifdef CONFIG_ALX_MSI
    ALX_setAdapterFlag(MSI_CAP);
#endif
    break;

  default:
    break;
  }

  // Set defaults for alx_adapter.
  ALX_Adapter.max_msix_intrs = 0;
  ALX_Adapter.min_msix_intrs = 0;
  max_frame = ETHER_MAX_LEN + VLAN_HLEN;
  ALX_Adapter.rxbuf_size = ETHERMTU > ALX_DEF_RX_BUF_SIZE ?
                           ALIGN(max_frame, 8) : ALX_DEF_RX_BUF_SIZE;
  ALX_Adapter.wol = 0;

  // Set defaults for alx_hw.
  hw->msi_lnkpatch = false;
  hw->link_up = false;
  hw->link_speed = 0;
  hw->intr_mask = ALX_IMR_NORMAL_MASK;
  hw->smb_timer = 400;  // 400ms
  hw->mtu = ETHERMTU;   // 1500
  hw->imt_mod = 100;        // Set to 200us.

  // Set defaults for WRR.
  hw->wrr_prio0 = 4;
  hw->wrr_prio1 = 4;
  hw->wrr_prio2 = 4;
  hw->wrr_prio3 = 4;
  hw->wrr_mode = alx_wrr_mode_none;

  // Set default flow control settings.
  hw->req_fc_mode = alx_fc_full;
  hw->cur_fc_mode = alx_fc_full;  // Init for ethtool output.
  hw->disable_fc_autoneg = false;
  hw->fc_was_autonegged = false;
  hw->fc_single_pause = true;

  // Set defaults for RSS info.
  hw->rss_hstype = 0;
  hw->rss_mode = alx_rss_mode_disable;
  hw->rss_idt_size = 0;
  hw->rss_base_cpu = 0;
  memset(hw->rss_idt, 0x0, sizeof(hw->rss_idt));
  memset(hw->rss_key, 0x0, sizeof(hw->rss_key));

  atomic_set(&ALX_Adapter.irq_sem, 1);
  spin_lock_init(&ALX_Adapter.tx_lock);
  spin_lock_init(&ALX_Adapter.rx_lock);

  ALX_initAdapterSpecial();

  if (hw->cbs.init_phy)
  {
    if (hw->cbs.init_phy(hw))
    {
      return kIOReturnError;
    }
  }

  ALX_setAdapterFlag(STATE_DOWN);

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::ALX_initAdapterSpecial()
{
  switch (ALX_Adapter.hw.mac_type)
  {
  case alx_mac_l1f:
  case alx_mac_l2f:
  case alx_mac_l1h:
  case alx_mac_l2h:
    goto init_alf_adapter;
    break;

  case alx_mac_l1c:
  case alx_mac_l1d_v1:
  case alx_mac_l1d_v2:
  case alx_mac_l2c:
  case alx_mac_l2cb_v1:
  case alx_mac_l2cb_v20:
  case alx_mac_l2cb_v21:
    goto init_alc_adapter;
    break;

  default:
    break;
  }

  return kIOReturnError;

  init_alc_adapter:

  if (ALX_checkAdapterFlag(MSIX_CAP))
  {
    ErrPrint("ALC doesn't support MSI-X.\n");
  }

  // MSI for Tx, Rx and none queues.
  ALX_Adapter.num_msix_txques = 0;
  ALX_Adapter.num_msix_rxques = 0;
  ALX_Adapter.num_msix_noques = 0;

  return kIOReturnSuccess;

  init_alf_adapter:

  if (ALX_checkAdapterFlag(MSIX_CAP))
  {
    // MSI-X for Tx, Rx and none queues.
    ALX_Adapter.num_msix_txques = 4;
    ALX_Adapter.num_msix_rxques = 8;
    ALX_Adapter.num_msix_noques = ALF_MAX_MSIX_NOQUE_INTRS;

    // MSI-X vector range.
    ALX_Adapter.max_msix_intrs = ALF_MAX_MSIX_INTRS;
    ALX_Adapter.min_msix_intrs = ALF_MIN_MSIX_INTRS;
  }
  else
  {
    // MSI for Tx, Rx and none queues.
    ALX_Adapter.num_msix_txques = 0;
    ALX_Adapter.num_msix_rxques = 0;
    ALX_Adapter.num_msix_noques = 0;
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::ALX_initBuffer(struct alx_buffer *buf)
{
  UInt32 numSeg = 1;
  UInt64 offset = 0;

  buf->dbuf = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
                kernel_task, (kIODirectionOut | kIOMemoryPhysicallyContiguous),
                0x4000UL, 0xfffffffffffff000ULL);

  if (!buf->dbuf)
  {
    return kIOReturnNoMemory;
  }

  if (buf->dbuf->prepare() != kIOReturnSuccess)
  {
    return kIOReturnCannotWire;
  }

  buf->dcom = IODMACommand::withSpecification(kIODMACommandOutputHost64,
              32, 0x4000, IODMACommand::kMapped, 0, 8, 0, 0);

  if (!buf->dcom)
  {
    return kIOReturnNoMemory;
  }

  if (buf->dcom->setMemoryDescriptor(buf->dbuf) != kIOReturnSuccess)
  {
    return kIOReturnError;
  }

  buf->dseg = (IODMACommand::Segment64 *)
              kzalloc(sizeof(IODMACommand::Segment64), 0);

  if (!buf->dseg)
  {
    return kIOReturnNoMemory;
  }

  if (buf->dcom->gen64IOVMSegments(&offset, buf->dseg, &numSeg) !=
      kIOReturnSuccess)
  {
    return kIOReturnError;
  }

  buf->dma = 0;

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::ALX_initHWCallbacks()
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  int retval = 0;

  ALX_setMACType();

  switch (hw->mac_type)
  {
  case alx_mac_l1f:
  case alx_mac_l2f:
  case alx_mac_l1h:
  case alx_mac_l2h:
    retval = alf_init_hw_callbacks(hw);
    break;

  case alx_mac_l1c:
  case alx_mac_l2c:
  case alx_mac_l2cb_v1:
  case alx_mac_l2cb_v20:
  case alx_mac_l2cb_v21:
  case alx_mac_l1d_v1:
  case alx_mac_l1d_v2:
    retval = alc_init_hw_callbacks(hw);
    break;

  default:
    retval = -1;
    break;
  }

  if (retval != 0)
  {
    return kIOReturnError;
  }

  return kIOReturnSuccess;
}

//==============================================================================

void
CLASS::ALX_initPCIPowerManagement()
{
  DbgPrint(4, "ALX_initPCIPowerManagement()\n");

  UInt8 pmCapOffset;

  // PCI power management (wake from D3cold for WoL magic packet).
  if (ALX_PCIDevice->findPCICapability(kIOPCIPowerManagementCapability,
                                       &pmCapOffset))
  {
    UInt16 pmc = ALX_PCIDevice->configRead16(pmCapOffset + 2);
    DbgPrint(3, "PCI register PMC = %#04x\n", pmc);

    if (pmc & kPCIPMCPMESupportFromD3Cold)
    {
      ALX_MagicPacketSupported = true;
      DbgPrint(3, "PME# from D3cold state is supported.\n");
    }
  }

  if (ALX_PCIDevice->hasPCIPowerManagement(kPCIPMCD3Support))
  {
    ALX_PCIDevice->enablePCIPowerManagement(kPCIPMCSPowerStateD3);
  }
}

//==============================================================================

IOReturn
CLASS::ALX_initRingPointers()
{
  int i, j;

  for (i = 0; i < ALX_Adapter.num_txques; i++)
  {
    struct alx_tx_queue *txque = ALX_Adapter.tx_queue[i];
    struct alx_buffer *tpbuf = txque->tpq.tpbuff;
    txque->tpq.produce_idx = 0;
    txque->tpq.consume_idx = 0;

    for (j = 0; j < txque->tpq.count; j++)
    {
      if (ALX_initBuffer(&tpbuf[j]) != kIOReturnSuccess)
      {
        return kIOReturnError;
      }
    }
  }

  for (i = 0; i < ALX_Adapter.num_hw_rxques; i++)
  {
    struct alx_rx_queue *rxque = ALX_Adapter.rx_queue[i];
    struct alx_buffer *rfbuf = rxque->rfq.rfbuff;
    rxque->rrq.produce_idx = 0;
    rxque->rrq.consume_idx = 0;
    rxque->rfq.produce_idx = 0;
    rxque->rfq.consume_idx = 0;

    for (j = 0; j < rxque->rfq.count; j++)
    {
      if (ALX_initBuffer(&rfbuf[j]) != kIOReturnSuccess)
      {
        return kIOReturnError;
      }
    }
  }

  if (ALX_checkAdapterFlag(SRSS_EN))
  {
    for (i = 0; i < ALX_Adapter.num_sw_rxques; i++)
    {
      struct alx_rx_queue *rxque = ALX_Adapter.rx_queue[i];
      rxque->swq.produce_idx = 0;
      rxque->swq.consume_idx = 0;
    }
  }

  return kIOReturnSuccess;
}

//==============================================================================

// Interrupt handler.
void
CLASS::ALX_interrupt(OSObject *client, IOInterruptEventSource *src, int count)
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  int max_intrs = ALX_MAX_HANDLED_INTRS;
  u32 isr, status;

  do
  {
    alx_mem_r32(hw, ALX_ISR, &isr);
    status = isr & hw->intr_mask;

    if (status == 0)
    {
      alx_mem_w32(hw, ALX_ISR, 0);
      return;
    }

    // ACK ISR to PHY register.
    if (status & ALX_ISR_PHY)
    {
      hw->cbs.ack_phy_intr(hw);
    }

    // ACK ISR to MAC register.
    alx_mem_w32(hw, ALX_ISR, status | ALX_ISR_DIS);

    if (status & ALX_ISR_ERROR)
    {
      DbgPrint(1, "ISR error (status = 0x%x)\n", (uint)status & ALX_ISR_ERROR);

      if (status & ALX_ISR_PCIE_FERR)
      {
        alx_mem_w16(hw, ALX_PCI_DEV_STAT,
                    ALX_PCI_DEV_STAT_FERR | ALX_PCI_DEV_STAT_NFERR |
                    ALX_PCI_DEV_STAT_CERR);
      }

      // Reset MAC.
      ALX_setAdapterFlag(TASK_REINIT_REQ);
      ALX_taskSchedule();
      return;
    }

    if (status & ALX_ISR_RXQ)
    {
      ALX_handleRxIRQ(ALX_Adapter.rx_queue[0]);
    }

    if (status & ALX_ISR_TXQ)
    {
      ALX_handleTxIRQ(ALX_Adapter.tx_queue[0]);
    }

    if (status & ALX_ISR_OVER)
    {
      DbgPrint(1, "TX/RX overflow (status = 0x%x)\n",
               (uint)status & ALX_ISR_OVER);
    }

    // Link event.
    if (status & (ALX_ISR_PHY | ALX_ISR_MANU))
    {
      DbgPrint(3, "Link event interrupt.\n");
      ALX_checkLinkStatusChange();
      break;
    }
  }
  while (--max_intrs > 0);

  // Re-enable interrupts.
  alx_mem_w32(hw, ALX_ISR, 0);

  return;
}

//==============================================================================

void
CLASS::ALX_linkTaskRoutine()
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  const char *link_desc;
  UInt link_speed_mb, medium_index;
  UInt64 uptime = 0;

  if (!ALX_checkAdapterFlag(TASK_LSC_REQ))
  {
    return;
  }

  ALX_clearAdapterFlag(TASK_LSC_REQ);

  if (ALX_checkAdapterFlag(STATE_DOWN))
  {
    return;
  }

  if (hw->cbs.check_phy_link)
  {
    hw->cbs.check_phy_link(hw, &hw->link_speed, &hw->link_up);
  }
  else
  {
    // Always assume link is up if no check link function is available.
    hw->link_speed = LX_LC_1000F;
    hw->link_up = true;
  }

  DbgPrint(5, "link_speed = %d, link_up = %d\n",
           hw->link_speed, hw->link_up);

  clock_get_uptime(&uptime);

  if (!hw->link_up && (uptime < ALX_LinkTimeout))
  {
    ALX_setAdapterFlag(TASK_LSC_REQ);
  }

  if (hw->link_up)
  {
    if (ALX_LinkEnabled)
    {
      return;
    }

    switch (hw->link_speed)
    {
    case LX_LC_1000F:
      link_desc = "1000Mb/s Full Duplex";
      link_speed_mb = 1000;
      medium_index = MEDIUM_INDEX_1000FD;
      break;

    case LX_LC_100F:
      link_desc = "100Mb/s Full Duplex";
      link_speed_mb = 100;
      medium_index = MEDIUM_INDEX_100FD;
      break;

    case LX_LC_100H:
      link_desc = "100Mb/s Half Duplex";
      link_speed_mb = 100;
      medium_index = MEDIUM_INDEX_100HD;
      break;

    case LX_LC_10F:
      link_desc = "10Mb/s Full Duplex";
      link_speed_mb = 10;
      medium_index = MEDIUM_INDEX_10FD;
      break;

    case LX_LC_10H:
      link_desc = "10Mb/s Half Duplex";
      link_speed_mb = 10;
      medium_index = MEDIUM_INDEX_10HD;
      break;

    default:
      link_desc = "Unknown Speed";
      link_speed_mb = 0;
      medium_index = MEDIUM_INDEX_NONE;
      break;
    }

    DbgPrint(2, "NIC link is up: %s.\n", link_desc);

    hw->cbs.post_phy_link(hw, CHK_HW_FLAG(AZ_EN), hw->link_up, hw->link_speed);
    hw->cbs.config_aspm(hw, true, true);
    hw->cbs.start_mac(hw);
    setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid,
                  ALX_MediumTable[medium_index], link_speed_mb * Mbit);
    ALX_LinkEnabled = true;
  }
  else // Link down.
  {
    // Only continue if link was up previously.
    if (!ALX_LinkEnabled)
    {
      return;
    }

    hw->link_speed = 0;
    DbgPrint(2, "NIC link is down.\n");
    setLinkStatus(kIONetworkLinkValid);
    ALX_LinkEnabled = false;

    hw->cbs.post_phy_link(hw, CHK_HW_FLAG(AZ_EN), hw->link_up, hw->link_speed);
    hw->cbs.stop_mac(hw);
    hw->cbs.config_aspm(hw, false, true);
    hw->cbs.setup_phy_link(hw, hw->autoneg_advertised, true,
                           !hw->disable_fc_autoneg);
  }
}

//==============================================================================

void
CLASS::ALX_patchAssign()
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  u32 misc_ctrl;

  if (hw->pci_devid == ALX_DEV_ID_AR8152_V2 &&
      hw->pci_revid == ALX_REV_ID_AR8152_V2_1)
  {
    // Configure access mode.
    alx_cfg_w32(hw, ALX_PCI_IND_ACC_ADDR, ALX_PCI_DEV_MISC_CTRL);
    alx_cfg_r32(hw, ALX_PCI_IND_ACC_DATA, &misc_ctrl);
    misc_ctrl &= ~0x100;
    alx_cfg_w32(hw, ALX_PCI_IND_ACC_ADDR, ALX_PCI_DEV_MISC_CTRL);
    alx_cfg_w32(hw, ALX_PCI_IND_ACC_DATA, misc_ctrl);
  }
}

//==============================================================================

IOReturn
CLASS::ALX_publishMediumDictionary()
{
  UInt32 fc_en = kIOMediumOptionFlowControl;

  // Publish the ethernet medium dictionary items.
  if (ALX_getNICType() == TYPE_GIGABIT)
  {
    ALX_MediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT_GIGABIT + 1);

    ALX_addNetworkMedium(
      kIOMediumEthernetAuto | fc_en,
      0, MEDIUM_INDEX_AUTO, "Autonegotiate Gigabit");
    ALX_addNetworkMedium(
      kIOMediumEthernet1000BaseTX | kIOMediumOptionFullDuplex | fc_en,
      1000 * Mbit, MEDIUM_INDEX_1000FD, "1000Mb/s Full Duplex");
  }
  else // TYPE_FAST
  {
    ALX_MediumDict = OSDictionary::withCapacity(MEDIUM_INDEX_COUNT_FAST + 1);

    ALX_addNetworkMedium(
      kIOMediumEthernetAuto,
      0, MEDIUM_INDEX_AUTO, "Autonegotiate Fast");
  }

  ALX_addNetworkMedium(kIOMediumEthernetNone, 0, MEDIUM_INDEX_NONE, "None");
  ALX_addNetworkMedium(
    kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex | fc_en,
    10 * Mbit, MEDIUM_INDEX_10HD, "10Mb/s Half Duplex");
  ALX_addNetworkMedium(
    kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex | fc_en,
    10 * Mbit, MEDIUM_INDEX_10FD, "10Mb/s Full Duplex");
  ALX_addNetworkMedium(
    kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex | fc_en,
    100 * Mbit, MEDIUM_INDEX_100HD, "100Mb/s Half Duplex");
  ALX_addNetworkMedium(
    kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | fc_en,
    100 * Mbit, MEDIUM_INDEX_100FD, "100Mb/s Full Duplex");

  if (!publishMediumDictionary(ALX_MediumDict))
  {
    return kIOReturnError;
  }

  return kIOReturnSuccess;
}

//==============================================================================

void
CLASS::ALX_receiveMbuf(mbuf_t m, u16 vlan_tag, bool vlan_flag)
{
	if (vlan_flag)
  {
		u16 vlan;
		ALX_TAG_TO_VLAN(vlan_tag, vlan);
		mbuf_set_vlan_tag((m),(vlan));
	}

	ALX_NetIf->inputPacket(m, 0, IONetworkInterface::kInputOptionQueuePacket);
}

//==============================================================================

int
CLASS::ALX_refreshRxBuffer(struct alx_rx_queue *rxque)
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  struct alx_buffer *curr_rxbuf;
  struct alx_buffer *next_rxbuf;
  union alx_sw_rfdesc srfd;
  UInt16 count = 0;
  UInt16 next_produce_idx;

  next_produce_idx = rxque->rfq.produce_idx;

  if (++next_produce_idx == rxque->rfq.count)
  {
    next_produce_idx = 0;
  }

  curr_rxbuf = GET_RF_BUFFER(rxque, rxque->rfq.produce_idx);
  next_rxbuf = GET_RF_BUFFER(rxque, next_produce_idx);

  // This always has a blank rx_buffer.
  while (next_rxbuf->dma == 0)
  {
    // Make buffer alignment 2 beyond a 16 byte boundary.
    // This will result in a 16 byte aligned IP header after
    // the 14 byte MAC header is removed.
    curr_rxbuf->length = ALX_Adapter.rxbuf_size;
    curr_rxbuf->dma = curr_rxbuf->dseg->fIOVMAddr;
    srfd.genr.addr = curr_rxbuf->dma;
    ALX_setRFDescriptor(rxque, &srfd);

    next_produce_idx = rxque->rfq.produce_idx;

    if (++next_produce_idx == rxque->rfq.count)
    {
      next_produce_idx = 0;
    }

    curr_rxbuf = GET_RF_BUFFER(rxque, rxque->rfq.produce_idx);
    next_rxbuf = GET_RF_BUFFER(rxque, next_produce_idx);
    count++;
  }

  if (count)
  {
    OSSynchronizeIO();
    alx_mem_w16(hw, rxque->produce_reg, rxque->rfq.produce_idx);
    DbgPrint(5, "RX[%d]: prod_reg[%x] = 0x%x, rfq.prod_idx = 0x%x\n",
             rxque->que_idx, rxque->produce_reg,
             rxque->rfq.produce_idx, rxque->rfq.produce_idx);
  }

  return count;
}

//==============================================================================

void
CLASS::ALX_reinitLocked()
{
  while (ALX_checkAdapterFlag(STATE_RESETTING))
  {
    IOSleep(20);
  }

  ALX_setAdapterFlag(STATE_RESETTING);

  ALX_disable(ALX_OPEN_CTRL_RESET_MAC);
  ALX_enable(ALX_OPEN_CTRL_RESET_MAC);

  ALX_clearAdapterFlag(STATE_RESETTING);
}

//==============================================================================
void
CLASS::ALX_reinitTaskRoutine()
{
  if (!ALX_checkAdapterFlag(TASK_REINIT_REQ))
  {
    return;
  }

  ALX_clearAdapterFlag(TASK_REINIT_REQ);

  if (ALX_checkAdapterFlag(STATE_DOWN) ||
      ALX_checkAdapterFlag(STATE_RESETTING))
  {
    return;
  }

  ALX_reinitLocked();
}

//==============================================================================

void
CLASS::ALX_restoreVLAN()
{
  ALX_VLANMode();
}

//==============================================================================

IOReturn
CLASS::ALX_selectMedium(const IONetworkMedium *medium)
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  u32 advertised, old;
  int error = 0;
  UInt32 med_idx = medium->getIndex();

  while (ALX_checkAdapterFlag(STATE_RESETTING))
  {
    IOSleep(20);
  }

  ALX_setAdapterFlag(STATE_RESETTING);

  old = hw->autoneg_advertised;
  advertised = 0;

  switch (med_idx)
  {
  case MEDIUM_INDEX_AUTO:
    advertised = LX_LC_ALL;
    break;

  case MEDIUM_INDEX_10HD:
    advertised = LX_LC_10H;
    break;

  case MEDIUM_INDEX_10FD:
    advertised = LX_LC_10F;
    break;

  case MEDIUM_INDEX_100HD:
    advertised = LX_LC_100H;
    break;

  case MEDIUM_INDEX_100FD:
    advertised = LX_LC_100F;
    break;

  case MEDIUM_INDEX_1000FD:
    advertised = LX_LC_1000F;
    break;

  default:
    break;
  }

  if (hw->autoneg_advertised == advertised) // No change in medium.
  {
    ALX_clearAdapterFlag(STATE_RESETTING);
    return kIOReturnSuccess;
  }

  hw->autoneg_advertised = advertised;
  error = hw->cbs.setup_phy_link(hw, hw->autoneg_advertised, true,
                                 !hw->disable_fc_autoneg);

  if (error)
  {
    ErrPrint("Setup link failed with code %d.\n", error);
    hw->autoneg_advertised = old;
    hw->cbs.setup_phy_link(hw, hw->autoneg_advertised, true,
                           !hw->disable_fc_autoneg);
  }

  ALX_clearAdapterFlag(STATE_RESETTING);

  return error ? kIOReturnError : kIOReturnSuccess;
}

//==============================================================================

// Setup some miscellaneous device information.
void
CLASS::ALX_setDeviceInfo()
{
  // Get the device information table index.
  switch (ALX_DeviceID)
  {
  case ALX_DEV_ID_AR8131:
    ALX_DeviceTableIndex = ALX_INDEX_AR8131;
    break;

  case ALX_DEV_ID_AR8132:
    ALX_DeviceTableIndex = ALX_INDEX_AR8132;
    break;

  case ALX_DEV_ID_AR8152_V1:
    ALX_DeviceTableIndex = ALX_INDEX_AR8152_V1;
    break;

  case ALX_DEV_ID_AR8152_V2:
    ALX_DeviceTableIndex = ALX_INDEX_AR8152_V2;
    break;

  case ALX_DEV_ID_AR8151_V1:
    ALX_DeviceTableIndex = ALX_INDEX_AR8151_V1;
    break;

  case ALX_DEV_ID_AR8151_V2:
    ALX_DeviceTableIndex = ALX_INDEX_AR8151_V2;
    break;

  case ALX_DEV_ID_AR8161:
    ALX_DeviceTableIndex = ALX_INDEX_AR8161;
    break;

  case ALX_DEV_ID_AR8162:
    ALX_DeviceTableIndex = ALX_INDEX_AR8162;
    break;

  case ALX_DEV_ID_AR8171:
    ALX_DeviceTableIndex = ALX_INDEX_AR8171;
    break;

  case ALX_DEV_ID_AR8172:
    ALX_DeviceTableIndex = ALX_INDEX_AR8172;
    break;

  default:
    ALX_DeviceTableIndex = ALX_INDEX_UNKNOWN;
    break;
  }

  ALX_PCIDevice->setProperty("model", ALX_deviceTable[ALX_DeviceTableIndex].name);
  setProperty("name", "ethernet");
}

//==============================================================================

// Change the Ethernet Address of the NIC.
IOReturn
CLASS::ALX_setMACAddress(const IOEthernetAddress *addr)
{
  struct alx_hw *hw = &ALX_Adapter.hw;

  if (ALX_validateMACAddress(addr->bytes) != kIOReturnSuccess)
  {
    return kIOReturnBadArgument;
  }

  if (ALX_Enabled)
  {
    return kIOReturnBusy;
  }

  memcpy(hw->mac_addr, addr->bytes, ETHER_ADDR_LEN);

  if (hw->cbs.set_mac_addr)
  {
    hw->cbs.set_mac_addr(hw, hw->mac_addr);
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::ALX_setMACType()
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  int retval = 0;

  if (hw->pci_venid == ALX_VENDOR_ID)
  {
    switch (hw->pci_devid)
    {
    case ALX_DEV_ID_AR8131:
      hw->mac_type = alx_mac_l1c;
      break;

    case ALX_DEV_ID_AR8132:
      hw->mac_type = alx_mac_l2c;
      break;

    case ALX_DEV_ID_AR8151_V1:
      hw->mac_type = alx_mac_l1d_v1;
      break;

    case ALX_DEV_ID_AR8151_V2:
      // Just use l1d configure.
      hw->mac_type = alx_mac_l1d_v2;
      break;

    case ALX_DEV_ID_AR8152_V1:
      hw->mac_type = alx_mac_l2cb_v1;
      break;

    case ALX_DEV_ID_AR8152_V2:
      if (hw->pci_revid == ALX_REV_ID_AR8152_V2_0)
      {
        hw->mac_type = alx_mac_l2cb_v20;
      }
      else
      {
        hw->mac_type = alx_mac_l2cb_v21;
      }

      break;

    case ALX_DEV_ID_AR8161:
      hw->mac_type = alx_mac_l1f;
      break;

    case ALX_DEV_ID_AR8162:
      hw->mac_type = alx_mac_l2f;
      break;

    case ALX_DEV_ID_AR8171:
      hw->mac_type = alx_mac_l1h;
      break;

    case ALX_DEV_ID_AR8172:
      hw->mac_type = alx_mac_l2h;
      break;

    default:
      retval = -EINVAL;
      break;
    }
  }
  else
  {
    retval = -EINVAL;
  }

  DbgPrint(3, "Found MAC: %d, retval: %d\n", hw->mac_type, retval);

  if (retval != 0)
  {
    return kIOReturnError;
  }

  return kIOReturnSuccess;
}

//==============================================================================

void
CLASS::ALX_setMulticastList()
{
  struct alx_hw *hw = &ALX_Adapter.hw;

  // Check for Promiscuous and All Multicast modes.
  if (ALX_MulticastFlags & IFF_PROMISC)
  {
    SET_HW_FLAG(PROMISC_EN);
  }
  else if (ALX_MulticastFlags & IFF_ALLMULTI)
  {
    SET_HW_FLAG(MULTIALL_EN);
    CLI_HW_FLAG(PROMISC_EN);
  }
  else
  {
    CLI_HW_FLAG(MULTIALL_EN);
    CLI_HW_FLAG(PROMISC_EN);
  }

  hw->cbs.update_mac_filter(hw);

  // Clear the old settings from the multicast hash table.
  hw->cbs.clear_mc_addr(hw);

  // Compute MAC address hash value and put it into the hash table.
  for (int i = 0; i < ALX_MulticastListLength; i++)
  {
    hw->cbs.set_mc_addr(hw, ALX_MulticastList[i].bytes);
  }
}

//==============================================================================

// Allocate queues for device, feature dependent.
void
CLASS::ALX_setQueueNumber()
{
  // Set the default case.
  ALX_Adapter.num_txques = 2;
  ALX_Adapter.num_rxques = 1;
  ALX_Adapter.num_hw_rxques = 1;
  ALX_Adapter.num_sw_rxques = 0;

  // ALX_setRxQueueNumber();
  // ALX_setTxQueueNumber();
}

//==============================================================================

IOReturn
CLASS::ALX_setRegisterInfoSpecial()
// FIXME: They could've just iterated num_txques.
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  int num_txques = ALX_Adapter.num_txques;

  switch (ALX_Adapter.hw.mac_type)
  {
  case alx_mac_l1f:
  case alx_mac_l2f:
  case alx_mac_l1h:
  case alx_mac_l2h:
    goto cache_alf_register;
    break;

  case alx_mac_l1c:
  case alx_mac_l1d_v1:
  case alx_mac_l1d_v2:
  case alx_mac_l2c:
  case alx_mac_l2cb_v1:
  case alx_mac_l2cb_v20:
  case alx_mac_l2cb_v21:
    goto cache_alc_register;
    break;

  default:
    break;
  }

  return kIOReturnError;

  cache_alc_register:
  // Setting for Produce Index and Consume Index.
  ALX_Adapter.rx_queue[0]->produce_reg = hw->rx_prod_reg[0];
  ALX_Adapter.rx_queue[0]->consume_reg = hw->rx_cons_reg[0];

  switch (num_txques)
  {
  case 2:
    ALX_Adapter.tx_queue[1]->produce_reg = hw->tx_prod_reg[1];
    ALX_Adapter.tx_queue[1]->consume_reg = hw->tx_cons_reg[1];

  case 1:
    ALX_Adapter.tx_queue[0]->produce_reg = hw->tx_prod_reg[0];
    ALX_Adapter.tx_queue[0]->consume_reg = hw->tx_cons_reg[0];
    break;
  }

  return kIOReturnSuccess;

  cache_alf_register:
  // Setting for Produce Index and Consume Index.
  ALX_Adapter.rx_queue[0]->produce_reg = hw->rx_prod_reg[0];
  ALX_Adapter.rx_queue[0]->consume_reg = hw->rx_cons_reg[0];

  switch (num_txques)
  {
  case 4:
    ALX_Adapter.tx_queue[3]->produce_reg = hw->tx_prod_reg[3];
    ALX_Adapter.tx_queue[3]->consume_reg = hw->tx_cons_reg[3];

  case 3:
    ALX_Adapter.tx_queue[2]->produce_reg = hw->tx_prod_reg[2];
    ALX_Adapter.tx_queue[2]->consume_reg = hw->tx_cons_reg[2];

  case 2:
    ALX_Adapter.tx_queue[1]->produce_reg = hw->tx_prod_reg[1];
    ALX_Adapter.tx_queue[1]->consume_reg = hw->tx_cons_reg[1];

  case 1:
    ALX_Adapter.tx_queue[0]->produce_reg = hw->tx_prod_reg[0];
    ALX_Adapter.tx_queue[0]->consume_reg = hw->tx_cons_reg[0];
  }

  return kIOReturnSuccess;
}

//==============================================================================

IOReturn
CLASS::ALX_setRFDescriptor(struct alx_rx_queue *rxque,
                             union alx_sw_rfdesc *srfd)
{
  union alx_hw_rfdesc *hrfd =
    ALX_RFD(rxque, rxque->rfq.produce_idx);

  hrfd->qfmt.qw0 = cpu_to_le64(srfd->qfmt.qw0);

  if (++rxque->rfq.produce_idx == rxque->rfq.count)
  {
    rxque->rfq.produce_idx = 0;
  }

  return kIOReturnSuccess;
}

//==============================================================================

void
CLASS::ALX_setRxQueueNumber()
{
  if (ALX_checkAdapterFlag(SRSS_CAP))
  {
    ALX_Adapter.num_hw_rxques = 1;
    ALX_Adapter.num_sw_rxques = ALX_Adapter.max_rxques;
    ALX_Adapter.num_rxques = max_t(u16,
                                   ALX_Adapter.num_hw_rxques, ALX_Adapter.num_sw_rxques);
  }
}

//==============================================================================

bool
CLASS::ALX_setTPDescriptor(struct alx_tx_queue *txque,
                             union alx_sw_tpdesc *stpd)
{
  union alx_hw_tpdesc *htpd;

  txque->tpq.last_produce_idx = txque->tpq.produce_idx;
  htpd = ALX_TPD(txque, txque->tpq.produce_idx);

  if (++txque->tpq.produce_idx == txque->tpq.count)
  {
    txque->tpq.produce_idx = 0;
  }

  htpd->dfmt.dw0 = cpu_to_le32(stpd->dfmt.dw0);
  htpd->dfmt.dw1 = cpu_to_le32(stpd->dfmt.dw1);
  htpd->qfmt.qw1 = cpu_to_le64(stpd->qfmt.qw1);

  return true;
}

//==============================================================================

void
CLASS::ALX_setTPDescriptorLastFragment(struct alx_tx_queue *txque)
{
  union alx_hw_tpdesc *htpd;
#define ALX_TPD_LAST_FRAGMENT  0x80000000
  htpd = ALX_TPD(txque, txque->tpq.last_produce_idx);
  htpd->dfmt.dw1 |= cpu_to_le32(ALX_TPD_LAST_FRAGMENT);
}

//==============================================================================

void
CLASS::ALX_setTxQueueNumber()
{
  struct alx_hw *hw = &ALX_Adapter.hw;

  if (hw->mac_type == alx_mac_l1f || hw->mac_type == alx_mac_l2f ||
      hw->mac_type == alx_mac_l1h || hw->mac_type == alx_mac_l2h)
  {
    ALX_Adapter.num_txques = 4;
  }
  else
  {
    ALX_Adapter.num_txques = 2;
  }
}

//==============================================================================

IOReturn
CLASS::ALX_sleep(bool *wakeup)
{
  DbgPrint(1, "ALX_sleep()\n");

  struct alx_hw *hw = &ALX_Adapter.hw;
  u32 misc, wufc = ALX_Adapter.wol;
  u16 lpa;
  u8 speed, adv_speed;
  bool link_up;
  bool wol_en, tx_en, rx_en;
  int i;
  int retval = 0;

  hw->cbs.config_aspm(hw, false, false);

  hw->cbs.check_phy_link(hw, &speed, &link_up);

  if (link_up)
  {
    if (hw->mac_type == alx_mac_l1f ||
        hw->mac_type == alx_mac_l2f ||
        hw->mac_type == alx_mac_l1h ||
        hw->mac_type == alx_mac_l2h)
    {
      alx_mem_r32(hw, ALX_MISC, &misc);
      misc |= ALX_MISC_INTNLOSC_OPEN;
      alx_mem_w32(hw, ALX_MISC, misc);
    }

    retval = hw->cbs.read_phy_reg(hw, MII_LPA, &lpa);

    if (retval)
    {
      return kIOReturnError;
    }

    adv_speed = LX_LC_10H;

    if (lpa & LPA_10FULL)
    {
      adv_speed = LX_LC_10F;
    }
    else if (lpa & LPA_10HALF)
    {
      adv_speed = LX_LC_10H;
    }
    else if (lpa & LPA_100FULL)
    {
      adv_speed = LX_LC_100F;
    }
    else if (lpa & LPA_100HALF)
    {
      adv_speed = LX_LC_100H;
    }

    retval = hw->cbs.setup_phy_link(hw, adv_speed, true,
                                    !hw->disable_fc_autoneg);

    if (retval)
    {
      return kIOReturnError;
    }

    for (i = 0; i < ALX_MAX_SETUP_LNK_CYCLE; i++)
    {
      IODelay(100);
      retval = hw->cbs.check_phy_link(hw, &speed, &link_up);

      if (retval)
      {
        continue;
      }

      if (link_up)
      {
        break;
      }
    }
  }
  else
  {
    speed = LX_LC_10H;
    link_up = false;
  }

  hw->link_speed = speed;
  hw->link_up = link_up;

  retval = hw->cbs.config_wol(hw, wufc);

  if (retval)
  {
    return kIOReturnError;
  }

  // Clear PHY interrupt.
  retval = hw->cbs.ack_phy_intr(hw);

  if (retval)
  {
    return kIOReturnError;
  }

  wol_en = wufc ? true : false;
  tx_en = false;
  rx_en = ((wufc & ALX_WOL_MAGIC) && ALX_MagicPacketEnabled) ? true : false;
  DbgPrint(3, "wol_en=%u tx_en=%u rx_en=%u\n", wol_en, tx_en, rx_en);

  retval = hw->cbs.config_pow_save(hw, ALX_Adapter.hw.link_speed,
                                   wol_en, tx_en, rx_en, true);

  if (retval)
  {
    return kIOReturnError;
  }

  *wakeup = wol_en;

  return 0;
}

//==============================================================================

IOReturn
CLASS::ALX_startTransmit(mbuf_t m)
{
  struct alx_tx_queue *txque;

  txque = ALX_Adapter.tx_queue[0];

  return ALX_startTransmitFrame(txque, m);
}

//==============================================================================

IOReturn
CLASS::ALX_startTransmitFrame(struct alx_tx_queue *txque, mbuf_t m)
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  unsigned long flags = 0;
  union alx_sw_tpdesc stpd;

  if (ALX_checkAdapterFlag(STATE_DOWN) ||
      ALX_checkAdapterFlag(STATE_TESTING))
  {
    DbgPrint(1, "Tx attempted while STATE_DOWN.\n");
    freePacket(m);
    return kIOReturnOutputSuccess;
  }

  if (!spin_trylock_irqsave(&ALX_Adapter.tx_lock, flags))
  {
    ErrPrint("Tx locked, cannot send packet.\n");
    return kIOReturnOutputStall;
  }

  if (!ALX_checkNumberTxDescriptors(txque, m))
  {
    // Not enough descriptors, just stop queue.
    DbgPrint(1, "Tx attempted with insufficient descriptors available.\n");
    ALX_TransmitQueue->stop();
    ALX_TransmitQueueEnabled = false;
    spin_unlock_irqrestore(&ALX_Adapter.tx_lock, flags);
    return kIOReturnOutputStall;
  }

  memset(&stpd, 0, sizeof(union alx_sw_tpdesc));
  // TODO: Do TSO and checksum.
#if 0

  if (alx_tso_csum(adpt, txque, m, &stpd) != 0)
  {
    spin_unlock_irqrestore(&ALX_Adapter.tx_lock, flags);
    dev_kfree_skb_any(m);
    return NETDEV_TX_OK;
  }

#endif

  if (unlikely(vlan_tx_tag_present(m)))
  {
    u16 vlan = vlan_tx_tag_get(m);
    u16 tag;
    ALX_VLAN_TO_TAG(vlan, tag);
    stpd.genr.vlan_tag = tag;
    stpd.genr.instag = 0x1;
  }

  if (((ether_header *)mbuf_data(m))->ether_type >= 0x0600)
  {
    stpd.genr.type = 0x1; // Ethernet frame.
  }

  ALX_TxMap(txque, m, &stpd);

  // Update produce index.
  OSSynchronizeIO();
  alx_mem_w16(hw, txque->produce_reg, txque->tpq.produce_idx);
  DbgPrint(5, "TX[%d]: tpq.consume_idx = 0x%x, tpq.produce_idx = 0x%x\n",
           txque->que_idx, txque->tpq.consume_idx,
           txque->tpq.produce_idx);
  DbgPrint(5, "TX[%d]: Produce Reg[%x] = 0x%x\n",
           txque->que_idx, txque->produce_reg, txque->tpq.produce_idx);

  spin_unlock_irqrestore(&ALX_Adapter.tx_lock, flags);

  return kIOReturnOutputSuccess;
}

//==============================================================================

// Manages and runs subtasks.
void
CLASS::ALX_taskRoutine()
{
  DbgPrint(5, "ALX_taskRoutine()\n");

  // Test state of adapter.
  if (!ALX_checkAdapterFlag(STATE_WATCH_DOG))
  {
    DbgPrint(5, "Flag STATE_WATCH_DOG not set.\n");
    return;
  }

  ALX_reinitTaskRoutine();

  ALX_linkTaskRoutine();

  ALX_clearAdapterFlag(STATE_WATCH_DOG);
}

//==============================================================================

void
CLASS::ALX_taskSchedule()
{
  if (!ALX_checkAdapterFlag(STATE_DOWN) &&
      !ALX_checkAdapterFlag(STATE_WATCH_DOG))
  {
    ALX_setAdapterFlag(STATE_WATCH_DOG);
  }
}

//==============================================================================

void
CLASS::ALX_timerFired(OSObject *owner, IOTimerEventSource *sender)
{
  ALXEthernet *alx_eth = OSDynamicCast(ALXEthernet, owner);

  alx_eth->ALX_taskRoutine();
  alx_eth->ALX_timerRoutine();
}

//==============================================================================

void
CLASS::ALX_timerRoutine()
{
  // Poll faster when waiting for link.
  if (ALX_checkAdapterFlag(TASK_LSC_REQ))
  {
    ALX_TimerEventSource->setTimeoutMS(HZ / 10);
  }
  else
  {
    ALX_TimerEventSource->setTimeoutMS(HZ * 2);
  }

  ALX_taskSchedule();
}

//==============================================================================

void
CLASS::ALX_TxMap(struct alx_tx_queue *txque, mbuf_t m,
                   union alx_sw_tpdesc *stpd)
{
  struct alx_buffer *tpbuf = NULL;
  // uint nr_frags = skb_shinfo(skb)->nr_frags;
  uint len = mbuf_len(m);

  // UInt16 map_len = 0;
  UInt16 mapped_len = 0;
  // UInt16 hdr_len = 0;
  // UInt32 tso = stpd->tso.lso;
  mbuf_t frag = NULL;
  // TODO: TSO.
#if 0

  if (tso)
  {
    map_len = hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);

    tpbuf = GET_TP_BUFFER(txque, txque->tpq.produce_idx);
    tpbuf->length = map_len;
    tpbuf->dma = dma_map_single(txque->dev,
                                skb->data, hdr_len, DMA_TO_DEVICE);
    mapped_len += map_len;
    stpd->genr.addr = tpbuf->dma;
    stpd->genr.buffer_len = tpbuf->length;

    alx_set_tpdesc(txque, stpd);
  }

#endif

  if (mapped_len < len)
  {
    tpbuf = GET_TP_BUFFER(txque, txque->tpq.produce_idx);

    if (!tpbuf->dbuf || !tpbuf->dcom || !tpbuf->dseg)
    {
      ErrPrint("Improperly prepared Tx buffer.");
    }

    tpbuf->length = len - mapped_len;

    mbuf_copydata(m, mapped_len, tpbuf->length,
                  tpbuf->dbuf->getBytesNoCopy(0, tpbuf->length));
    tpbuf->dcom->synchronize(kIODirectionOut);
    tpbuf->dma = tpbuf->dseg->fIOVMAddr;

    stpd->genr.addr = tpbuf->dma;
    stpd->genr.buffer_len  = tpbuf->length;
    ALX_setTPDescriptor(txque, stpd);
  }

  for (frag = mbuf_next(m); frag != NULL; frag = mbuf_next(frag))
  {
    tpbuf = GET_TP_BUFFER(txque, txque->tpq.produce_idx);
    tpbuf->length = mbuf_len(frag);
    mbuf_copydata(frag, 0, tpbuf->length,
                  tpbuf->dbuf->getBytesNoCopy(0, tpbuf->length));
    tpbuf->dcom->synchronize(kIODirectionOut);
    tpbuf->dma = tpbuf->dseg->fIOVMAddr;

    stpd->genr.addr = tpbuf->dma;
    stpd->genr.buffer_len  = tpbuf->length;
    ALX_setTPDescriptor(txque, stpd);
  }

  // The last TPD.
  ALX_setTPDescriptorLastFragment(txque);

  freePacket(m);
}

//==============================================================================

IOReturn
CLASS::ALX_validateMACAddress(const UInt8 *mac_addr)
{
  int retval = 0;

  if (is_broadcast_ether_addr(mac_addr))
  {
    DbgPrint(1, "MAC address is broadcast.\n");
    retval = -1;
  }
  else if (is_multicast_ether_addr(mac_addr))
  {
    DbgPrint(1, "MAC address is multicast.\n");
    retval = -1;
  }
  else if (is_zero_ether_addr(mac_addr))
  {
    DbgPrint(1, "MAC address is all zeros.\n");
    retval = -1;
  }

  if (retval != 0)
  {
    return kIOReturnError;
  }

  return kIOReturnSuccess;
}

//==============================================================================

void
CLASS::ALX_VLANMode()
{
  struct alx_hw *hw = &ALX_Adapter.hw;
  UInt32 features = getFeatures();

  if (!ALX_checkAdapterFlag(STATE_DOWN))
  {
    ALX_disableInterrupt();
  }

  if (features & kIONetworkFeatureHardwareVlan)
  {
    // Enable VLAN tag insert/strip.
    SET_HW_FLAG(VLANSTRIP_EN);
  }
  else
  {
    // Disable VLAN tag insert/strip.
    CLI_HW_FLAG(VLANSTRIP_EN);
  }

  hw->cbs.update_mac_filter(hw);

  if (!ALX_checkAdapterFlag(STATE_DOWN))
  {
    ALX_enableInterrupt();
  }
}

//==============================================================================

IOReturn
CLASS::ALX_wake()
{
  DbgPrint(1, "ALX_wake()\n");

  struct alx_hw *hw = &ALX_Adapter.hw;
  u32 retval;

  retval = hw->cbs.reset_pcie(hw, true, true);
  retval = hw->cbs.reset_phy(hw);
  retval = hw->cbs.reset_mac(hw);
  retval = hw->cbs.setup_phy_link(hw, hw->autoneg_advertised, true,
                                  !hw->disable_fc_autoneg);

  retval = hw->cbs.config_wol(hw, 0);

  if (retval)
  {
    return kIOReturnError;
  }

  return kIOReturnSuccess;
}
