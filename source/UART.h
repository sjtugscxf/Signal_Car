#ifndef UART_H
#define UART_H


// ====== APIs ======

// ---- Ayano's Format ----
void UART_SendDataHead();
void UART_SendData(s16 data);


// ---- Basic ----

void UART_Init(u32);
int8 UART_GetChar();
void UART_SendChar(uint8 data);
void UART_SendString(const char *p);

#endif