#include <LaneDetectorPublisher.hpp>

LaneDetectorPublisher::LaneDetectorPublisher(
    std::shared_ptr<zenoh::Session> session)
    : session_(session),
      provider_(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})))
{
    cameraError_pub.emplace(session_->declare_publisher(
        zenoh::KeyExpr("Vehicle/1/LaneDetection/CameraError")));
}

void LaneDetectorPublisher::publishCameraError(float error)
{
    std::cout << "Error send: " << error << std::endl;
    std::string value_str = to_string(error);
    const auto len        = value_str.size() + 1;
    auto alloc_result =
        provider_.alloc_gc_defrag_blocking(len, zenoh::AllocAlignment({0}));
    zenoh::ZShmMut&& buf = std::get<zenoh::ZShmMut>(std::move(alloc_result));
    memcpy(buf.data(), value_str.c_str(), len);
    cameraError_pub->put(std::move(buf));
}