#include "GPUInference.hpp"

GPUInference::GPUInference(const std::string &enginePath) {
    enginePath_ = enginePath;
}

GPUInference::~GPUInference() {
    cudaFreeHost(inputData);
    cudaFreeHost(outputData);
    cudaFree(inputDevice);
    cudaFree(outputDevice);
    cudaStreamDestroy(stream);
}

void GPUInference::init() {
    createExecutionContext(enginePath_);

    // Set highest stream priority
    int leastPriority, greatestPriority;
    cudaDeviceGetStreamPriorityRange(&leastPriority, &greatestPriority);
    cudaStreamCreateWithPriority(&stream, cudaStreamNonBlocking,
                                 greatestPriority);

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
}

void GPUInference::createExecutionContext(const std::string& enginePath)
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

void GPUInference::inference(cv::Mat& frame, cv::Mat& binary_mask) {
    float milliseconds = 0;

    // Continue with existing channel reordering code
    const int plane_size      = HEIGHT * WIDTH;
    const uint8_t* frame_data = frame.data;

    for (int c = 0; c < 3; c++)
    {
        for (int i = 0; i < plane_size; i++)
        {
            inputData[c * plane_size + i] = frame_data[i * 3 + c] / 255.0f;
        }
    }

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

    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Inference time in lane detection: " << milliseconds << "ms\n";

    const int total_pixels = HEIGHT * WIDTH;

    for (int i = 0; i < total_pixels; i++) {
        int y = i / WIDTH;
        int x = i % WIDTH;
        uchar value = (outputData[i] > 0.5) ? 127 : 0;
        binary_mask.at<uchar>(y, x) = value;
    }
}