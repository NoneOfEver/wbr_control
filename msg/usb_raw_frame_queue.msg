# @struct UsbRawFrameMessage
# @queue usb_raw_frames 16
# @constant size_t kUsbRawChunkSize 128
uint32 receive_timestamp_us
uint16 len
uint8[kUsbRawChunkSize] data
