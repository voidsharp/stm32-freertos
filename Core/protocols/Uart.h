#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>

typedef enum {
    PacketType_Unknown = 0,
    PacketType_StringMessage = 1,     // змінна довжина
    PacketType_GetTime,              // довжина = 0
    PacketType_SetTime,              // довжина = sizeof(TimeStruct)
    PacketType_GetTemp,              // довжина = 0
    PacketType_GetHumidity,          // довжина = 0
    PacketType_RadioSetChannel,      // довжина = 1
    PacketType_RadioGetChannel,      // довжина = 0
    PacketType_RadioSetAddress,      // довжина = 5
    PacketType_RadioGetAddress,      // довжина = 0
    PacketType_RadioSetPowerLevel,   // довжина = 1
    PacketType_RadioGetPowerLevel,   // довжина = 0
} PacketType;

// Загальна структура без CRC
#pragma pack(push, 1)
typedef struct {
    uint8_t type;
    uint8_t length;
    uint8_t value[]; // payload, залежить від type
    // CRC йде одразу після value[length]
} Packet;

#pragma pack(pop)

uint16_t crc;

#pragma pack(push, 1)

typedef struct {
    char message[]; // NULL-terminated або просто байти — на твій розсуд
} Value_StringMessage;

typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} Value_SetTime;

typedef struct {
    uint8_t channel; // Наприклад, 0–125
} Value_RadioSetChannel;

typedef struct {
    uint8_t address[5];
} Value_RadioSetAddress;

typedef struct {
    uint8_t powerLevel; // Наприклад, 0 (min) – 3 (max)
} Value_RadioSetPowerLevel;

#pragma pack(pop)






#endif // UART_PROTOCOL_H
