/*
    Most STM32 boards use a Bootloader to flash new firmware. So firmware updates is just
    moving firmware.bin to / and rebooting.
*/

extern uint32_t _estack;			// defined in the linker script


// Update the firmware from SD. Prerequisites should be checked before calling this.
void RepRap::RunSdIap(c_string _ecv_null  filename) noexcept
{
    debugPrintf("Update firmware from SD card based file\n");
#if  HAS_MASS_STORAGE
    //DWC will upload firmware to 0:/sys/ we need to move to 0:/firmware.bin and reboot
    
    String<MaxFilenameLength> fileName;
    MassStorage::CombineName(fileName.GetRef(), FIRMWARE_DIRECTORY, filename[0] == 0 ? IAP_FIRMWARE_FILE : filename);
    FileStore * const fin = MassStorage::OpenFile(fileName.c_str(), OpenMode::read, 0);
    if (fin == nullptr)
    {
        platform->MessageF(FirmwareUpdateMessage, "Failed to open firmware input file.\n");
        return;
    }

    FileStore * const fout = MassStorage::OpenFile(FIRMWARE_FILE, OpenMode::write, 0);
    if (fout == nullptr)
    {
        platform->MessageF(FirmwareUpdateMessage, "Failed to open firmware output file.\n");
        return;
    }
    uint8_t buffer[1024];
    uint32_t ret;
    for(;;)
    {
        ret = fin->Read(buffer, sizeof(buffer));
        if (ret <= 0) break;
        fout->Write(buffer, ret);
    }
    fin->Close();
    fout->Close();
    if (!stopped)
    {
        debugPrintf("Sending estop\n");
        EmergencyStop();			// turn off heaters etc.
    }
    debugPrintf("Restarting....\n");
    delay(1000);    
    SoftwareReset(SoftwareResetReason::user); // Reboot
#endif
    
}

#if SUPPORT_REMOTE_COMMANDS

#if 0
int32_t RequestFirmwareBlock(FirmwareModule modType, uint32_t fileOffset, uint32_t numBytes, uint8_t *buffer, uint32_t *fileSize)
{
    CanMessageBuffer *buf = CanMessageBuffer::Allocate();
    if (buf == nullptr)
    {
        debugPrintf("No Message buffer available\n");
        return -1;
    }
    digitalWrite(DiagPin, XNor(DiagOnPolarity, millis() & 32) != 0);
    CanMessageFirmwareUpdateRequest * const msg = buf->SetupRequestMessageNoRid<CanMessageFirmwareUpdateRequest>(CanInterface::GetCanAddress(), CanId::MasterAddress);
    SafeStrncpy(msg->boardType, BOARD_SHORT_NAME, sizeof(msg->boardType));
    msg->boardVersion = 0;
    msg->bootloaderVersion = CanMessageFirmwareUpdateRequest::BootloaderVersion0;
    msg->fileOffset = fileOffset;
    msg->lengthRequested = (numBytes > sizeof(CanMessageFirmwareUpdateResponse::data)) ? sizeof(CanMessageFirmwareUpdateResponse::data) : numBytes;
    msg->fileWanted = (uint32_t)modType;
    msg->uf2Format = 0;
    buf->dataLength = msg->GetActualDataLength();
    String<1> dummy;
    int32_t ret = -5;
    CanInterface::SendRequestAndGetCustomReply(buf, fileOffset & 0xfff, dummy.GetRef(), nullptr, CanMessageType::firmwareBlockResponse,
                                                            [buffer, &ret, fileSize](const CanMessageBuffer *buf)
                                                                {
                                                                    auto response = buf->msg.firmwareUpdateResponse;
                                                                    if (response.err != CanMessageFirmwareUpdateResponse::ErrNone )
                                                                        ret = - response.err;
                                                                    else
                                                                    {
                                                                        ret = response.dataLength;
                                                                        for (int32_t i = 0; i < ret; ++i)
                                                                        {
                                                                            buffer[i] = response.data[i];
                                                                        }
                                                                    }
                                                                    *fileSize = response.fileLength;
                                                                });
    if (ret < 0)
        debugPrintf("FirmwareUpdateRequest error %d\n", (int)-ret);
    return ret;
}
#endif

static void RequestFirmwareBlock(FirmwareModule modType, uint32_t fileOffset, uint32_t numBytes, CanMessageBuffer& buf)
{
    CanMessageFirmwareUpdateRequest * const msg = buf.SetupRequestMessageNoRid<CanMessageFirmwareUpdateRequest>(CanInterface::GetCanAddress(), CanId::MasterAddress);
    SafeStrncpy(msg->boardType, BOARD_SHORT_NAME, sizeof(msg->boardType));
    msg->boardVersion = 0;
    msg->bootloaderVersion = CanMessageFirmwareUpdateRequest::BootloaderVersion0;
    msg->uf2Format = false;
    msg->fileWanted = (uint32_t)modType;
    msg->fileOffset = fileOffset;
    msg->lengthRequested = numBytes;
    buf.dataLength = msg->GetActualDataLength();
    CanInterface::SendMessageNoReplyNoFree(&buf);
}


// Get a buffer of data from the host
static int32_t GetBlock(FirmwareModule modType, uint32_t startingOffset, uint32_t& fileSize, uint32_t numBytes, uint8_t *buffer)
{
    CanMessageBuffer buf;
    RequestFirmwareBlock(modType, startingOffset, numBytes, buf);	// ask for 16K or 64K from the starting offset

    uint32_t whenStartedWaiting = millis();
    uint32_t bytesReceived = 0;
    bool done = false;
    do
    {
        digitalWrite(DiagPin, XNor(DiagOnPolarity, millis() & 32) != 0);
        const bool ok = CanInterface::GetFirmwareUpdateResponse(&buf);
        if (ok)
        {
            if (buf.id.MsgType() == CanMessageType::firmwareBlockResponse)
            {
                const CanMessageFirmwareUpdateResponse& response = buf.msg.firmwareUpdateResponse;
                switch (response.err)
                {
                case CanMessageFirmwareUpdateResponse::ErrNoFile:
                case CanMessageFirmwareUpdateResponse::ErrBadOffset:
                case CanMessageFirmwareUpdateResponse::ErrOther:
                    return -response.err;

                case CanMessageFirmwareUpdateResponse::ErrNone:
                    if (response.fileOffset >= startingOffset && response.fileOffset <= startingOffset + bytesReceived)
                    {
                        const uint32_t bufferOffset = response.fileOffset - startingOffset;
                        const uint32_t bytesToCopy = min<uint32_t>(numBytes - bufferOffset, response.dataLength);
                        memcpy(buffer + bufferOffset, response.data, bytesToCopy);
                        if (response.fileOffset + bytesToCopy > startingOffset + bytesReceived)
                        {
                            bytesReceived = response.fileOffset - startingOffset + bytesToCopy;
                        }
                        if (bytesReceived == numBytes || bytesReceived >= response.fileLength - startingOffset)
                        {
                            // Reached the end of the file
                            memset(buffer + bytesReceived, 0xFF, numBytes - bytesReceived);
                            fileSize = response.fileLength;
                            done = true;
                        }
                    }
                    whenStartedWaiting = millis();
                }
            }
        }
        else if (millis() - whenStartedWaiting > 2000)
        {
            if (bytesReceived == 0)
            {
                return -5;
            }
            RequestFirmwareBlock(modType, startingOffset + bytesReceived, numBytes - bytesReceived, buf);		// ask for 16K or 64K from the starting offset
            whenStartedWaiting = millis();
        }
    } while (!done);
    return bytesReceived;
}

// Update the firmware over CAN. Prerequisites should be checked before calling this.
void RepRap::RunCanIap(c_string _ecv_null  filenameRef) noexcept
{
    debugPrintf("Update firmware over CAN\n");
    uint32_t start = millis();
    EmergencyStop();			// turn off heaters etc.
    String<MaxFilenameLength> fileName;
    MassStorage::CombineName(fileName.GetRef(), FIRMWARE_DIRECTORY, IAP_FIRMWARE_FILE);
    FileStore * const f = MassStorage::OpenFile(fileName.c_str(), OpenMode::write, 0);
    if (f == nullptr)
    {
        debugPrintf("Failed to create firmware file %s\n", fileName.c_str());
        return;
    }
    debugPrintf("Save firmware to %s\n", fileName.c_str());
    uint32_t offset = 0;
    int32_t ret;
    uint32_t fileSize;
    do {
        uint8_t buf[1024];
        ret = GetBlock(FirmwareModule::main, offset, fileSize, sizeof(buf), buf);
        if (ret > 0)
            if (!f->Write(buf, ret))
            {
                debugPrintf("Failed to write to firmware file offset %u len %u", (unsigned)offset, (unsigned)ret);
                return;
            }
        offset += ret;
    } while (ret > 0 && offset < fileSize);
    f->Close();
    debugPrintf("Update time %ums\n", (unsigned)(millis() - start));
    RunSdIap(IAP_FIRMWARE_FILE);
    debugPrintf("Restarting....\n");
    delay(1000);    
    SoftwareReset(SoftwareResetReason::user); // Reboot

}

#if STM32H7
#include <Flash.h>
constexpr size_t BootloaderFlashStart = 0x8000000;

// Update the Bootloader. Prerequisites should be checked before calling this.
void RepRap::RunCanBootloaderIap(c_string _ecv_null  filenameRef) noexcept
{
    debugPrintf("Update bootloader over CAN\n");
    uint32_t start = millis();
    EmergencyStop();			// turn off heaters etc.
    uint32_t offset = 0;
    int32_t ret;
    uint32_t fileSize;
    do {
        uint8_t buf[1024];
        ret = GetBlock(FirmwareModule::bootloader, offset, fileSize, sizeof(buf), buf);
        if (ret > 0)
        {
            debugPrintf("Got block size %d\n", ret);
            if (offset == 0)
            {
                Flash::FlashEraseSector(0);
            }
            Flash::FlashWrite(BootloaderFlashStart+offset, buf, sizeof(buf));
            offset += ret;
        }
    } while (ret > 0 && offset < fileSize);
    if (ret < 0)
        debugPrintf("got error %d\n", ret);
    debugPrintf("Update time %ums\n", (unsigned)(millis() - start));
    debugPrintf("Restarting....\n");
    delay(1000);
    SoftwareReset(SoftwareResetReason::user); // Reboot
}
#endif
#endif

