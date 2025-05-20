#include "LaneDetector.hpp"
#include <sys/time.h>
#include <iostream>
#include <signal.h>
#include <atomic>

using namespace cv;
using namespace std;
using namespace zenoh;

LaneDetector::LaneDetector(const std::string& enginePath, std::shared_ptr<zenoh::Session> session)
{
    session_ = session;
    provider_.emplace(zenoh::MemoryLayout(65536, zenoh::AllocAlignment({2})));

    createExecutionContext(enginePath);

    // Set highest stream priority
    int leastPriority, greatestPriority;
    cudaDeviceGetStreamPriorityRange(&leastPriority, &greatestPriority);
    cudaStreamCreateWithPriority(&stream, cudaStreamNonBlocking,
                                 greatestPriority);

    // Create an OpenCV CUDA stream (with default flags)
    cv_stream = cv::cuda::Stream();

    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // Pin memory for faster transfers
    void* input_ptr;
    void* output_ptr;
    cudaHostAlloc(&input_ptr, 3 * HEIGHT * WIDTH * sizeof(float),
                  cudaHostAllocMapped);
    cudaHostAlloc(&output_ptr, 1 * HEIGHT * WIDTH * sizeof(float),
                  cudaHostAllocMapped);
    inputData  = static_cast<float*>(input_ptr);
    outputData = static_cast<float*>(output_ptr);

    // Allocate GPU memory
    size_t pitch;
    cudaMallocPitch(&inputDevice, &pitch, WIDTH * sizeof(float),
                    HEIGHT * 3);
    cudaMallocPitch(&outputDevice, &pitch, WIDTH * sizeof(float),
                    HEIGHT * 1);

    // Calibrate IPM

    float cameraHeight = 0.13f;       // meters
    float cameraPitch = 20.0f;       // degrees down from horizontal
    float horizontalFOV = 120.0f;     // degrees
    float img_height = static_cast<float>(HEIGHT);
    float img_width = static_cast<float>(WIDTH);
    float h_fov_rad = horizontalFOV * CV_PI / 180.0f;
    float verticalFOV = 2.0f * std::atan((img_height/img_width) * std::tan(h_fov_rad/2.0f)) * 180.0f / CV_PI;
    float nearDistance = 0.2f;       // meters
    float farDistance = 1.5f;       // meters
    float laneWidth = 1.0f;          // meters
    bevSize = cv::Size(WIDTH, WIDTH);
    cv::Size origSize = cv::Size(WIDTH, HEIGHT);
    ipm.initialize(origSize, bevSize);
    ipm.calibrateFromCamera(cameraHeight, cameraPitch, horizontalFOV, verticalFOV,
                            nearDistance, farDistance, laneWidth);

    try
    {
        this->canBus     = new CAN();
        this->canBus->init("/dev/spidev0.0");
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error initializing CAN on Object Detector " << e.what() << std::endl;
    }

}

LaneDetector::~LaneDetector()
{
    cudaFreeHost(inputData);
    cudaFreeHost(outputData);
    cudaFree(inputDevice);
    cudaFree(outputDevice);
    cudaStreamDestroy(stream);
}

void LaneDetector::createExecutionContext(const std::string& enginePath)
{
    Logger logger;

    std::ifstream file(enginePath, std::ios::binary);
    if (!file)
    {
        throw std::runtime_error("Failed to open engine file");
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> engineData(size);
    file.read(engineData.data(), size);

    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(logger);
    nvinfer1::ICudaEngine* engine =
        runtime->deserializeCudaEngine(engineData.data(), size);
    context.reset(engine->createExecutionContext());
}

void LaneDetector::detect(cv::Mat& frame)
{
    float milliseconds = 0;

    // Preprocess
    preProcess(frame);

    cudaEventRecord(start, stream);

    // Copy to GPU
    cudaMemcpyAsync(inputDevice, inputData,
                    3 * HEIGHT * WIDTH * sizeof(float),
                    cudaMemcpyHostToDevice, stream);

    // Run inference with optimization flags
    void* bindings[] = {inputDevice, outputDevice};
    context->enqueueV2(bindings, stream, nullptr);

    // Copy back to CPU
    cudaMemcpyAsync(outputData, outputDevice,
                    1 * HEIGHT * WIDTH * sizeof(float),
                    cudaMemcpyDeviceToHost, stream);

    cudaStreamSynchronize(stream);

    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);
    
    // Postprocess
    postProcess(frame);

    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Inference time: " << milliseconds << "ms\n";
}

void LaneDetector::preProcess(const cv::Mat& frame)
{
    // Create static GPU matrices
    static cv::cuda::GpuMat d_frame, d_resized, d_rgb_image;
    static cv::Mat cpu_rgb_image(HEIGHT, WIDTH, CV_8UC3);

    // Upload input frame to GPU
    d_frame.upload(frame);

    // Resize on GPU with CUDA stream
    cv::cuda::resize(d_frame, d_resized, cv::Size(WIDTH, HEIGHT), 0, 0,
                     cv::INTER_NEAREST, cv_stream);

    // Convert BGR to RGB on GPU
    cv::cuda::cvtColor(d_resized, d_rgb_image, cv::COLOR_BGR2RGB, 0, cv_stream);

    // Download the result back to CPU
    d_rgb_image.download(cpu_rgb_image, cv_stream);

    // Wait for CUDA operations to complete
    cv_stream.waitForCompletion();

    // Continue with existing channel reordering code
    const int plane_size      = HEIGHT * WIDTH;
    const uint8_t* frame_data = cpu_rgb_image.data;

    for (int c = 0; c < 3; c++)
    {
        for (int i = 0; i < plane_size; i++)
        {
            inputData[c * plane_size + i] = frame_data[i * 3 + c] / 255.0f;
        }
    }
}

void LaneDetector::postProcess(cv::Mat& frame)
{
    static cv::Mat colored_mask(HEIGHT, WIDTH, CV_8UC3);
    const int total_pixels = HEIGHT * WIDTH;

    for (int i = 0; i < total_pixels; i++) {
        int y = i / WIDTH;
        int x = i % WIDTH;
        uchar value = (outputData[i] > 0.5) ? 255 : 0;
        colored_mask.at<cv::Vec3b>(y, x) = cv::Vec3b(0, value, 0);
    }

    // Apply IPM to mask and frame
    cv::Mat ipm_mask = ipm.applyIPM(colored_mask);
    cv::Mat ipm_frame = ipm.applyIPM(frame);
    
    cv::Mat resized_ipm_mask;
    cv::resize(ipm_mask, resized_ipm_mask, frame.size(), 0, 0, cv::INTER_NEAREST);

    cv::Mat resized_ipm_frame;
    cv::resize(ipm_frame, resized_ipm_frame, frame.size(), 0, 0, cv::INTER_NEAREST);
    
    // Create a temporary result image for the bird's eye view
    cv::Mat bev_result;
    cv::addWeighted(resized_ipm_frame, 0.7, resized_ipm_mask, 0.3, 0, bev_result);
    
    // Replace the original frame with the bird's eye view result
    bev_result.copyTo(frame);
}

