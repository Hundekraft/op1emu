#pragma once

#include <uvw.hpp>
#include <memory>
#include <vector>
#include <cqueue/cqueue.h>
#include <functional>
#include <mutex>
#include "usbip-internal.h"

enum class USBIPState {
    WaitCommand,
    WaitCommandImport,
    WaitHeader,
    WaitURB,
    WaitUnlink,
    WaitTransferBuffer,
};

class USBIPServer {
public:
    USBIPServer(uvw::loop& loop, USBDevice& device);
    void Start();
    // A system reset replaces the USB device (nullptr while the machine is rebuilt): the next requests go to
    // the new one, like a re-plug.
    void Rebind(USBDevice* d);

private:
    void OnClientDataEvent(const uvw::data_event& event, uvw::tcp_handle& client);
    void HandleURBRequest(USBIP_CMD_SUBMIT& req, uint8_t* data, std::size_t length, uvw::tcp_handle& client);
    void Reply(uvw::tcp_handle& client, const USBIP_CMD_SUBMIT& req, const void* data, std::size_t length, const USBIP_ISOC_DESC* isoc);
    void ReplyImport(OP_REQ_IMPORT& req, uvw::tcp_handle& client);

private:
    cqueue<uint8_t> buffer;
    USBIPState state = USBIPState::WaitCommand;

    std::mutex mutex;
    uvw::loop& loop;
    std::shared_ptr<uvw::tcp_handle> server;

    USBDevice* device;
    std::recursive_mutex deviceMutex;   // held while a request is in the device and by Rebind
};