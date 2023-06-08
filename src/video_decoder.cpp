#include "ros2_bebop_driver/video_decoder.hpp"

namespace bebop_driver {
VideoDecoder::VideoDecoder() {}
VideoDecoder::~VideoDecoder() {}

void VideoDecoder::init() {}
void VideoDecoder::allocateBuffers() {}

void VideoDecoder::decode(const uint8_t* data, uint32_t size) {}

const uint8_t* VideoDecoder::getFrame() const { return frame; }
}  // namespace bebop_driver
