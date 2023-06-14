#include "ros2_bebop_driver/video_decoder.hpp"

namespace bebop_driver {
VideoDecoder::VideoDecoder() { init(); }
VideoDecoder::~VideoDecoder() {}

bool VideoDecoder::init() {
    av_log_set_level(AV_LOG_WARNING);

    p_codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!p_codec_) {
	std::cerr << "Could not find ffmpeg h264 codec" << std::endl;
	return false;
    }

    p_codec_context_ = avcodec_alloc_context3(p_codec_);
    if (p_codec_context_ == nullptr) {
	std::cerr << "Could not alloc codec context" << std::endl;
	return false;
    }

    if (avcodec_open2(p_codec_context_, p_codec_, nullptr) < 0) {
	std::cerr << "Could not open ffmpeg h264 codec" << std::endl;
	return false;
    }

    p_packet_ = av_packet_alloc();
    if (p_packet_ == nullptr) {
	std::cerr << "Could not alloc packet" << std::endl;
	return false;
    }

    p_frame_ = av_frame_alloc();
    if (p_frame_ == nullptr) {
	std::cerr << "Could not alloc frame" << std::endl;
	return false;
    }
    return true;
}
void VideoDecoder::allocateBuffers() {
    delete[] frame;

    frame = new uint8_t[3 * width * height];
}

bool VideoDecoder::decode(uint8_t* data, uint32_t size) {
    p_packet_->size = size;
    p_packet_->data = data;

    // Send the packet to the decoder
    if (avcodec_send_packet(p_codec_context_, p_packet_) < 0) {
	std::cerr << "Could not send packet to the decoder" << std::endl;
	return false;
    }
    // Get decoded frame
    // We may not yet get a frame. The avcodec_receive_frame output can be any
    // of
    //  0	success, a frame was returned
    //  AVERROR(EAGAIN)	output is not available in this state - user
    //         must try to send new input
    //  AVERROR_EOF	the codec has been fully flushed, and there will
    //         be no more output frames
    //  AVERROR(EINVAL)	codec not opened, or it is an encoder without
    //          the AV_CODEC_FLAG_RECON_FRAME flag enabled
    //  AVERROR_INPUT_CHANGED current decoded frame has changed
    //          parameters with respect to first decoded frame.
    //          Applicable when flag AV_CODEC_FLAG_DROPCHANGED is set.
    //          other negative error code	legitimate decoding errors
    if (avcodec_receive_frame(p_codec_context_, p_frame_) < 0) {
	return false;
    }

    // If the size of the image changed, we need to reallocate the
    // buffers
    if ((int)width != p_frame_->width || (int)height != p_frame_->height) {
	width = p_frame_->width;
	height = p_frame_->height;
	allocateBuffers();
    }

    p_sws_context_ = sws_getCachedContext(
	p_sws_context_, p_frame_->width, p_frame_->height,
	p_codec_context_->pix_fmt, p_frame_->width, p_frame_->height,
	AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
    int stride = 3 * p_frame_->width;
    sws_scale(p_sws_context_, (const uint8_t* const*)p_frame_->data,
	      p_frame_->linesize, 0, p_frame_->height, &frame, &stride);

    //
    {
	std::ostringstream filename;
	filename << "toto-" << std::setw(6) << std::setfill('0') << id++
		 << ".ppm";
	std::ofstream outfile(filename.str());
	outfile << "P6\n856 480\n255\n";
	outfile.write(reinterpret_cast<char*>(frame), 856 * 480 * 3);
    }

    return true;
}

const uint8_t* VideoDecoder::getFrame() const { return frame; }

unsigned int VideoDecoder::getWidth() const { return width; }
unsigned int VideoDecoder::getHeight() const { return height; }

}  // namespace bebop_driver

