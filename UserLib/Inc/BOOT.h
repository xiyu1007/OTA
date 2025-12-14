#ifndef __BOOT_H__
#define __BOOT_H__


void BootLoader(void); 
void BootJumpToApp(uint32_t addr);

void BootLoader_CommandLine(void);
uint8_t BootLoader_Enter(uint8_t timeout);
void BootLoader_Event(uint8_t *data, uint32_t len);
void BootLoader_EchoRxBuffer(uint8_t echo);
void BootClearAppEntry(void);

typedef void (*AppEntryFunc)(void);



#endif /* __BOOT_H__ */








