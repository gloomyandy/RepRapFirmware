/*
    Most STM32 boards use a Bootloader to flash new firmware. So firmware updates is just
    moving firmware.bin to / and rebooting.
*/

extern uint32_t _estack;			// defined in the linker script


// Update the firmware. Prerequisites should be checked before calling this.
void RepRap::RunSdIap(const char * filename) noexcept
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

int32_t RequestFirmwareBlock(uint32_t fileOffset, uint32_t numBytes, uint8_t *buffer, uint32_t *fileSize)
{
	CanMessageBuffer *buf = CanMessageBuffer::Allocate();
	if (buf == nullptr)
	{
		debugPrintf("No Message buffer available\n");
        return -1;
	}
    CanMessageFirmwareUpdateRequest * const msg = buf->SetupRequestMessage<CanMessageFirmwareUpdateRequest>(0, CanInterface::GetCanAddress(), CanId::MasterAddress);
	SafeStrncpy(msg->boardType, BOARD_SHORT_NAME, sizeof(msg->boardType));
	msg->boardVersion = 0;
	msg->bootloaderVersion = CanMessageFirmwareUpdateRequest::BootloaderVersion0;
	msg->fileOffset = fileOffset;
	msg->lengthRequested = numBytes;
    msg->fileWanted = (uint32_t)FirmwareModule::main;
    msg->uf2Format = 0;
	buf->dataLength = msg->GetActualDataLength();
    String<1> dummy;
    uint32_t ret = -5;
	CanInterface::SendRequestAndGetCustomReply(buf, fileOffset & 0xfff, dummy.GetRef(), nullptr, CanMessageType::firmwareBlockResponse,
															[buffer, &ret, fileSize](const CanMessageBuffer *buf)
																{
																	auto response = buf->msg.firmwareUpdateResponse;
                                                                    if (response.err != CanMessageFirmwareUpdateResponse::ErrNone )
                                                                        ret = - response.err;
                                                                    else
                                                                    {
                                                                        ret = response.dataLength;
                                                                        for (unsigned int i = 0; i < ret; ++i)
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

// Update the firmware. Prerequisites should be checked before calling this.
void RepRap::RunCanIap(const char * filenameRef) noexcept
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
        uint8_t buf[56];
        ret = RequestFirmwareBlock(offset, sizeof(buf), buf, &fileSize);
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
#endif

