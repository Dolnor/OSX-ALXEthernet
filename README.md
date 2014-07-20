
## Atheros(R) AR81(31/32/51/52/61/62/71/72) PCI-E ethernet

Atheros OSX driver originally coded by Shailua based on “Unified alx driver strategy attempt for Linux and FreeBSD”, 
at https://github.com/erikarn/alx with additions by Zephiris. 

Original discussion thread: 

http://www.insanelymac.com/forum/topic/284119-experimental-atheros-ar813132515261627172-driver-for-107108/


### Changelog:

v1.0.3 

- Displaying actual adapter model in System Profiler instead of “ALX Ethernet Driver”

- Displaying ‘ethernet” in name field instead of adapter model

- Fixed memory allocation and which lead to freezing

- Number of RX/TX descriptors reduced due to latency reasons


v1.0.2 

- Bug fixes that should help clean up memory allocation problems at boot time 

- VLAN is now possible for anyone that needs it, but I've only tested it minimally


v1.0.1 

- Bug fixes and refinements. 

- Manually setting the link speed should work properly now and might be an option if auto negotiation is causing issues. 

- The link watchdog timer should generally be working better now. 

- Changed some Linux code that was automatically enabling MSI-X interrupts on newer chips. 


v1.0.0 

- The initial release. Don't be fooled, it's likely very unstable!


### Bugs:

Using Apple's "Logic Remote" from Wi-Fi (then to ethernet) will kill the driver immediately with "incorrect zfree from zone kalloc.16 to zone kalloc.32"


### To-Do:

Backport newer Linux/FreeBSD code into the project.
