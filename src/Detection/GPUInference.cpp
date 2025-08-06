#include "GPUInference.hpp"

GPUInference::GPUInference(const std::string& enginePath, int inputChannels,
                           int outputChannels, int height, int width)
    : height_(height), width_(width)
{
    enginePath_     = enginePath;
    inputChannels_  = inputChannels;
    outputChannels_ = outputChannels;
}

GPUInference::~GPUInference()
{
    cudaFreeHost(inputData);
    cudaFreeHost(outputData);
    cudaFree(inputDevice);
    cudaFree(outputDevice);
    cudaStreamDestroy(stream);
}

void GPUInference::init()
{
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
    cudaHostAlloc(&input_ptr, inputChannels_ * height_ * width_ * sizeof(float),
                  cudaHostAllocMapped);
    cudaHostAlloc(&output_ptr,
                  outputChannels_ * height_ * width_ * sizeof(float),
                  cudaHostAllocMapped);
    inputData  = static_cast<float*>(input_ptr);
    outputData = static_cast<float*>(output_ptr);

    // Allocate GPU memory
    size_t pitch;
    cudaMallocPitch(&inputDevice, &pitch, width_ * sizeof(float),
                    height_ * inputChannels_);
    cudaMallocPitch(&outputDevice, &pitch, width_ * sizeof(float),
                    height_ * outputChannels_);
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

void GPUInference::inference()
{
    float milliseconds = 0;

    cudaEventRecord(start, stream);

    // Run inference with optimization flags
    void* bindings[] = {inputDevice, outputDevice};
    context->enqueueV2(bindings, stream, nullptr);

    // try {
    //     for (int i = 0; i < 2; i++) {
    //         if (i == 0) {
    //             context->setTensorAddress("input", bindings[0]);
    //         } else {
    //             context->setTensorAddress("output", bindings[1]);
    //         }
    //     }
    //     context->enqueueV3(stream);
    // } catch (const std::exception& e) {
    //     std::cout << "Error infering!" << e.what() << std::endl;
    // }

    cudaStreamSynchronize(stream);

    cudaEventRecord(stop, stream);
    cudaEventSynchronize(stop);

    cudaEventElapsedTime(&milliseconds, start, stop);

    // if (outputChannels_ == 1)
    // {
    //     std::cout << "\033[32mInference time in lane detection: " <<
    //     milliseconds
    //             << "ms\033[0m\n"; // Green
    // }
    // else if (outputChannels_ == 8)
    // {
    //     std::cout << "\033[34mInference time in object detection: " <<
    //     milliseconds
    //             << "ms\033[0m\n"; // Blue
    // }
    // else if (outputChannels_ == 9)
    // {
    //     std::cout << "\033[33mInference time in traffic classification: " <<
    //     milliseconds
    //             << "ms\033[0m\n"; // Yellow
    // }
}

void GPUInference::copyToGPU(cv::Mat& preprocessedFrame)
{
    const int plane_size            = height_ * width_;
    const uint8_t* preprocessedData = preprocessedFrame.data;

    float means[3] = {0.485f, 0.456f, 0.406f};
    float stds[3]  = {0.229f, 0.224f, 0.225f};

    for (int c = 0; c < inputChannels_; c++)
    {
        for (int i = 0; i < plane_size; i++)
        {
            float pixelValue =
                preprocessedData[i * inputChannels_ + (inputChannels_ + c)] /
                255.0f;

            inputData[c * plane_size + i] = (pixelValue - means[c]) / stds[c];
        }
    }

    cudaMemcpyAsync(inputDevice, inputData,
                    inputChannels_ * height_ * width_ * sizeof(float),
                    cudaMemcpyHostToDevice, stream);
}

float sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

void GPUInference::copyToCPUBinaryOutput(cv::Mat& outputMask)
{
    const int total_pixels = height_ * width_;

    cudaMemcpyAsync(outputData, outputDevice,
                    outputChannels_ * height_ * width_ * sizeof(float),
                    cudaMemcpyDeviceToHost, stream);

    cudaStreamSynchronize(stream);

    for (int i = 0; i < total_pixels; i++)
    {
        int y                      = i / width_;
        int x                      = i % width_;
        float prob                 = sigmoid(outputData[i]);
        uchar value                = (prob > 0.5f) ? 255 : 0;
        outputMask.at<uchar>(y, x) = value;
    }
}

void GPUInference::copyToCPUClassOutput(cv::Mat& outputMask)
{
    const int total_pixels       = height_ * width_;
    const cv::Scalar color_map[] = {
        cv::Scalar(0, 0, 10),     // Background
        cv::Scalar(128, 64, 128), // Road
        cv::Scalar(0, 0, 142),    // Car
        cv::Scalar(250, 0, 0),    // Traffic Light
        cv::Scalar(220, 220, 0),  // Traffic Sign
        cv::Scalar(220, 20, 60),  // Person
        cv::Scalar(244, 35, 232), // Sidewalks
        cv::Scalar(0, 0, 70),     // Truck
        cv::Scalar(0, 60, 100),   // Bus
        cv::Scalar(0, 0, 230)     // Motorcycle
    };

    // Copy back to CPU
    cudaMemcpyAsync(outputData, outputDevice,
                    outputChannels_ * height_ * width_ * sizeof(float),
                    cudaMemcpyDeviceToHost, stream);

    cudaStreamSynchronize(stream);

    // For each pixel, find the class with highest probability
    for (int i = 0; i < total_pixels; i++)
    {
        // Get probability for each class
        float probs[outputChannels_];
        for (int c = 0; c < outputChannels_; c++)
        {
            probs[c] = outputData[total_pixels * c + i];
        }

        int best_class = 0;
        float max_prob = probs[0];

        for (int c = 1; c < outputChannels_; c++)
        {
            if (probs[c] > max_prob)
            {
                max_prob   = probs[c];
                best_class = c;
            }
        }

        // Map pixel coordinates (i) back to x,y
        int y = i / width_;
        int x = i % width_;

        // Set pixel color based on class
        outputMask.at<cv::Vec3b>(y, x) =
            cv::Vec3b(color_map[best_class][0], color_map[best_class][1],
                      color_map[best_class][2]);
    }
}

int GPUInference::copyToCPUTrafficOutput()
{
    cudaMemcpyAsync(outputData, outputDevice,
                    outputChannels_ * height_ * width_ * sizeof(float),
                    cudaMemcpyDeviceToHost, stream);

    cudaStreamSynchronize(stream);

    const std::string classes[10] = {
        "Speed 50km/h",   "Speed 80km/h", "Yield",         "Stop",
        "Danger",         "Crosswalk",    "Traffic Green", "Traffic Red",
        "Traffic Yellow", "Unknown"};

    // Apply softmax to outputData
    float sum = 0.0f;
    std::vector<float> probs(outputChannels_);
    for (int c = 0; c < outputChannels_; ++c)
    {
        probs[c] = expf(outputData[c]);
        sum += probs[c];
    }
    for (int c = 0; c < outputChannels_; ++c)
    {
        probs[c] /= sum;
    }

    // Print probabilities and predicted class
    int best_class = 0;
    float max_prob = probs[0];
    std::cout << "Traffic sign class probabilities: ";
    for (int c = 0; c < outputChannels_; ++c)
    {
        std::cout << classes[c] << "(" << probs[c] << "), ";
        if (probs[c] > max_prob)
        {
            max_prob   = probs[c];
            best_class = c;
        }
    }

    if (probs[best_class] > 0.85)
    {
        std::cout << "\nPredicted class: " << classes[best_class]
                  << " (prob=" << max_prob << ")" << std::endl;

        return best_class;
    }
    return (-1);
}