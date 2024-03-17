/*
 * Board Config
 *
 *  Created on: 3 Feb 2019
 *      Author: sdavi
 */

#ifndef BOARDCONFIG_H_
#define BOARDCONFIG_H_

#include "Core.h"
#include "RepRapFirmware.h"
#include "Platform/MessageType.h"
#include "ff.h"
#include "Platform/RepRap.h"


enum configValueType{
    cvPinType = 0,
    cvBoolType,
    cvUint8Type,
    cvUint16Type,
    cvUint32Type,
    cvFloatType,
    cvStringType,
    cvDriverType
};

const uint32_t CVTLengths[] = {sizeof(Pin), sizeof(bool), sizeof(uint8_t), sizeof(uint16_t), sizeof(uint32_t), sizeof(float), MaxBoardNameLength, sizeof(DriverType)};

struct boardConfigEntry_t
{
    const char* key;
    void *variable;
    const size_t numItems;
    configValueType type;
};


class Platform; //fwd decl

class BoardConfig {
public:
    static void Init() noexcept;

    static void Diagnostics(MessageType mtype) noexcept;
    static Pin StringToPin(const char *strvalue) noexcept;
    static Pin LookupPin(char *strvalue) noexcept;
    static bool LoadBoardConfigFromFile(const char *filePath, bool restricted = true) noexcept;

#if HAS_SBC_INTERFACE
    static bool LoadBoardConfigFromSBC() noexcept;
    static void InvalidateBoardConfiguration() noexcept;
#endif
    
private:
    BoardConfig()  noexcept;
    static bool GetConfigKeys(FileStore * const configFile) noexcept;
    static void SetValueFromString(configValueType type, void *variable, char *valuePtr) noexcept;
    static void PrintValue(MessageType mtype, configValueType configType, void *variable) noexcept;
};

#endif /* BOARDCONFIG_H_ */
