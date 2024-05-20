//
// Created by xinyang on 2021/4/8.
//
#ifdef ARTINX_CUDA
// NOLINTBEGIN
#include "TRTModule.hpp"

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, char const* msg) noexcept override {
        // 过滤掉不需要的日志级别
        if(severity <= nvinfer1::ILogger::Severity::kWARNING) {
            std::cout << msg << std::endl;
        }
    }
} gLogger;

#define TRT_ASSERT(expr)                                                      \
    do {                                                                      \
        if(!(expr)) {                                                         \
            fmt::print(fmt::fg(fmt::color::red), "assert fail: '" #expr "'"); \
            exit(-1);                                                         \
        }                                                                     \
    } while(0)

using namespace nvinfer1;
// using namespace sample;

static inline size_t get_dims_size(const Dims& dims) {
    size_t sz = 1;
    for(int i = 0; i < dims.nbDims; i++)
        sz *= dims.d[i];
    return sz;
}

template <class F, class T, class... Ts>
T reduce(F&& func, T x, Ts... xs) {
    if constexpr(sizeof...(Ts) > 0) {
        return func(x, reduce(std::forward<F>(func), xs...));
    } else {
        return x;
    }
}

template <class T, class... Ts>
T reduce_max(T x, Ts... xs) {
    return reduce([](auto&& a, auto&& b) { return std::max(a, b); }, x, xs...);
}

template <class T, class... Ts>
T reduce_min(T x, Ts... xs) {
    return reduce([](auto&& a, auto&& b) { return std::min(a, b); }, x, xs...);
}

static inline bool is_overlap(const float pts1[8], const float pts2[8]) {
    cv::Rect2f bbox1, bbox2;
    bbox1.x = reduce_min(pts1[0], pts1[2], pts1[4], pts1[6]);
    bbox1.y = reduce_min(pts1[1], pts1[3], pts1[5], pts1[7]);
    bbox1.width = reduce_max(pts1[0], pts1[2], pts1[4], pts1[6]) - bbox1.x;
    bbox1.height = reduce_max(pts1[1], pts1[3], pts1[5], pts1[7]) - bbox1.y;
    bbox2.x = reduce_min(pts2[0], pts2[2], pts2[4], pts2[6]);
    bbox2.y = reduce_min(pts2[1], pts2[3], pts2[5], pts2[7]);
    bbox2.width = reduce_max(pts2[0], pts2[2], pts2[4], pts2[6]) - bbox2.x;
    bbox2.height = reduce_max(pts2[1], pts2[3], pts2[5], pts2[7]) - bbox2.y;
    return (bbox1 & bbox2).area() > 0;
}

static inline int argmax(const float* ptr, int len) {
    int max_arg = 0;
    for(int i = 1; i < len; i++) {
        if(ptr[i] > ptr[max_arg])
            max_arg = i;
    }
    return max_arg;
}

constexpr float inv_sigmoid(float x) {
    return -std::log(1 / x - 1);
}

constexpr float sigmoid(float x) {
    return 1 / (1 + std::exp(-x));
}

TRTModule::TRTModule(const std::string& onnx_file, int kptNum, int classNum, int anchorNum, bool rune = false) {
    this->kptNum = kptNum;
    this->classNum = classNum;
    this->anchorNum = anchorNum;

    std::filesystem::path onnx_file_path(onnx_file);
    auto cache_file_path = onnx_file_path;
    cache_file_path.replace_extension("cache");
    if(std::filesystem::exists(cache_file_path)) {
        build_engine_from_cache(cache_file_path.c_str());
    } else {
        build_engine_from_onnx(onnx_file_path.c_str());
        cache_engine(cache_file_path.c_str());
    }
    TRT_ASSERT((context = engine->createExecutionContext()) != nullptr);
    TRT_ASSERT((input_idx = engine->getBindingIndex("input")) == 0);
    TRT_ASSERT((output_idx = engine->getBindingIndex("output-topk")) == 1);
    auto input_dims = engine->getBindingDimensions(input_idx);
    auto output_dims = engine->getBindingDimensions(output_idx);
    input_sz = get_dims_size(input_dims);
    output_sz = get_dims_size(output_dims);
    TRT_ASSERT(cudaMalloc(&device_buffer[input_idx], input_sz * sizeof(float)) == 0);
    TRT_ASSERT(cudaMalloc(&device_buffer[output_idx], output_sz * sizeof(float)) == 0);
    TRT_ASSERT(cudaStreamCreate(&stream) == 0);
    output_buffer = new float[output_sz];
    TRT_ASSERT(output_buffer != nullptr);
}

TRTModule::~TRTModule() {
    delete[] output_buffer;
    cudaStreamDestroy(stream);
    cudaFree(device_buffer[output_idx]);
    cudaFree(device_buffer[input_idx]);
    engine->destroy();
}

void TRTModule::build_engine_from_onnx(const std::string& onnx_file) {
    std::cout << "[INFO]: build engine from onnx" << std::endl;
    auto builder = createInferBuilder(gLogger);
    TRT_ASSERT(builder != nullptr);
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);
    TRT_ASSERT(network != nullptr);
    auto parser = nvonnxparser::createParser(*network, gLogger);
    TRT_ASSERT(parser != nullptr);
    parser->parseFromFile(onnx_file.c_str(), static_cast<int>(ILogger::Severity::kINFO));
    auto yolov5_output = network->getOutput(0);
    /*
        在这里的TensorRT网络中，输出尺寸为 1 * 15120 * 22，这意味着网络的输出是一个三维数组，其中有1个批次（batch），15120个元素，每个元素包含22个特征。这种格式通常用于对象检测任务中，其中15120可能代表了不同的锚框（anchor boxes），而22个特征包含了位置坐标、置信度和类别概率等信息。
        这里创建的切片层auto slice_layer = network->addSlice(*yolov5_output, Dims3{ 0, 0, 8 }, Dims3{ 1, 15120, 1 }, Dims3{ 1, 1, 1 });的作用是从整个输出中提取特定的数据子集。这里的参数解释如下：
        Dims3{ 0, 0, 8 } 表示切片的起始点，分别是批次维度的第0位、元素维度的第0位、特征维度的第8位。因此，切片是从每个元素的第8个特征开始的。
        Dims3{ 1, 15120, 1 } 表示切片的尺寸，即在批次维度取1（整个批次），在元素维度取15120（全部元素），在特征维度取1（仅取一个特征）。这意味着你正在从每个元素中仅提取第8个特征，这个特征很可能是置信度（confidence score）。
        Dims3{ 1, 1, 1 } 是步长，指明在每个维度上移动的步长，这里都是1，意味着连续提取，没有跳过。
        总结来说，这个切片层的创建是为了从每个预测元素的22个特征中提取第8个特征（可能是置信度），并且对全部15120个预测元素进行这样的操作。这种操作通常在需要单独处理某些特定数据（如置信度）以进行进一步操作（如TopK筛选）时使用。
    */
    
    auto slice_layer = network->addSlice(*yolov5_output, Dims3{ 0, 0, 8 }, Dims3{ 1, 15120, 1 }, Dims3{ 1, 1, 1 });
    auto yolov5_conf = slice_layer->getOutput(0);
    auto shuffle_layer = network->addShuffle(*yolov5_conf);
    shuffle_layer->setReshapeDimensions(Dims2{ 1, 15120 });

    if (rune){
        auto feature_3 = network->addSlice(*yolov5_output, Dims3{ 0, 4, 0 }, Dims3{ 1, 1, 3549 }, Dims3{ 1, 1, 1 })->getOutput(0);
        auto feature_4 = network->addSlice(*yolov5_output, Dims3{ 0, 5, 0 }, Dims3{ 1, 1, 3549 }, Dims3{ 1, 1, 1 })->getOutput(0);
        auto feature_5 = network->addSlice(*yolov5_output, Dims3{ 0, 6, 0 }, Dims3{ 1, 1, 3549 }, Dims3{ 1, 1, 1 })->getOutput(0);
        auto feature_6 = network->addSlice(*yolov5_output, Dims3{ 0, 7, 0 }, Dims3{ 1, 1, 3549 }, Dims3{ 1, 1, 1 })->getOutput(0);
        auto max_layer_1 = network->addElementWise(*feature_3, *feature_4, ElementWiseOperation::kMAX)->getOutput(0);
        auto max_layer_2 = network->addElementWise(*feature_5, *feature_6, ElementWiseOperation::kMAX)->getOutput(0);
        yolov5_conf = network->addElementWise(*max_layer_1, *max_layer_2, ElementWiseOperation::kMAX)->getOutput(0);
        shuffle_layer = network->addShuffle(*yolov5_conf);
        shuffle_layer->setReshapeDimensions(Dims2{ 1, 3549 });
    }



    yolov5_conf = shuffle_layer->getOutput(0);
    auto topk_layer = network->addTopK(*yolov5_conf, TopKOperation::kMAX, TOPK_NUM, 1 << 1);
    auto topk_idx = topk_layer->getOutput(1);
    auto gather_layer = network->addGather(*yolov5_output, *topk_idx, 1);
    gather_layer->setNbElementWiseDims(1);
    auto yolov5_output_topk = gather_layer->getOutput(0);
    yolov5_output_topk->setName("output-topk");
    network->getInput(0)->setName("input");
    network->markOutput(*yolov5_output_topk);
    network->unmarkOutput(*yolov5_output);
    auto config = builder->createBuilderConfig();
    // if (builder->platformHasFastFp16()) {
    //     std::cout << "[INFO]: platform support fp16, enable fp16" << std::endl;
    //     config->setFlag(BuilderFlag::kFP16);
    // } else {
    //     std::cout << "[INFO]: platform do not support fp16, enable fp32" << std::endl;
    // }
    size_t free, total;
    cuMemGetInfo(&free, &total);
    std::cout << "[INFO]: total gpu mem: " << (total >> 20) << "MB, free gpu mem: " << (free >> 20) << "MB" << std::endl;
    std::cout << "[INFO]: max workspace size will use all of free gpu mem" << std::endl;
    config->setMaxWorkspaceSize(free);
    TRT_ASSERT((engine = builder->buildEngineWithConfig(*network, *config)) != nullptr);
    config->destroy();
    parser->destroy();
    network->destroy();
    builder->destroy();
}

void TRTModule::build_engine_from_cache(const std::string& cache_file) {
    std::cout << "[INFO]: build engine from cache" << std::endl;
    std::ifstream ifs(cache_file, std::ios::binary);
    ifs.seekg(0, std::ios::end);
    size_t sz = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    auto buffer = std::make_unique<char[]>(sz);
    ifs.read(buffer.get(), sz);
    auto runtime = createInferRuntime(gLogger);
    TRT_ASSERT(runtime != nullptr);
    TRT_ASSERT((engine = runtime->deserializeCudaEngine(buffer.get(), sz)) != nullptr);
    // runtime->destroy();
}

void TRTModule::cache_engine(const std::string& cache_file) {
    auto engine_buffer = engine->serialize();
    TRT_ASSERT(engine_buffer != nullptr);
    std::ofstream ofs(cache_file, std::ios::binary);
    ofs.write(static_cast<const char*>(engine_buffer->data()), engine_buffer->size());
    engine_buffer->destroy();
}

std::vector<bbox_t> TRTModule::operator()(const cv::Mat& src) const {
    // pre-process [bgr2rgb & resize]

    auto img_width = rune ? 416.f : 640.f;
    auto img_height = rune ? 416.f : 512.f;
    cv::Mat x;
    float fx = (float)src.cols / img_width, fy = (float)src.rows / img_height;
    cv::cvtColor(src, x, cv::COLOR_BGR2RGB);
    if(src.cols != static_cast<int>(img_width) || src.rows != static_cast<int>(img_height)) {
        cv::resize(x, x, { static_cast<int>(img_width),  static_cast<int>(img_height) });
    }
    x.convertTo(x, CV_32F);

    // run model
    cudaMemcpyAsync(device_buffer[input_idx], x.data, input_sz * sizeof(float), cudaMemcpyHostToDevice, stream);
    context->enqueueV2(device_buffer, stream, nullptr);
    cudaMemcpyAsync(output_buffer, device_buffer[output_idx], output_sz * sizeof(float), cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);

    // post-process [nms]
    std::vector<bbox_t> rst;
    rst.reserve(TOPK_NUM);
    std::vector<uint8_t> removed(TOPK_NUM);
    if (rune){
        for(int i = 0; i < TOPK_NUM; i++) {
            auto* box_buffer = output_buffer + i * 8;
            if(argmax(box_buffer + 4, 4) < inv_sigmoid(KEEP_THRES))
                break;
            if(removed[i])
                continue;
            rst.emplace_back();
            auto& box = rst.back();
            memcpy(&box.pts, box_buffer, 4 * sizeof(float));
            for(auto& pt : box.pts)
                pt.x *= fx, pt.y *= fy;
            box.confidence = sigmoid(argmax(box_buffer + 4, 4));
            box.color_id = 0;
            box.tag_id = argmax(box_buffer + 13, 9);
            for(int j = i + 1; j < TOPK_NUM; j++) {
                auto* box2_buffer = output_buffer + j * 22;
                if(box2_buffer[8] < inv_sigmoid(KEEP_THRES))
                    break;
                if(removed[j])
                    continue;
                if(is_overlap(box_buffer, box2_buffer))
                    removed[j] = true;
            }
        }
        return rst;
    }else{
        for(int i = 0; i < TOPK_NUM; i++) {
            auto* box_buffer = output_buffer + i * 22;  // 20->23
            if(box_buffer[8] < inv_sigmoid(KEEP_THRES))
                break;
            if(removed[i])
                continue;
            rst.emplace_back();
            auto& box = rst.back();
            memcpy(&box.pts, box_buffer, 8 * sizeof(float));
            for(auto& pt : box.pts)
                pt.x *= fx, pt.y *= fy;
            box.confidence = sigmoid(box_buffer[8]);
            box.color_id = argmax(box_buffer + 9, 4);
            box.tag_id = argmax(box_buffer + 13, 9);
            for(int j = i + 1; j < TOPK_NUM; j++) {
                auto* box2_buffer = output_buffer + j * 22;
                if(box2_buffer[8] < inv_sigmoid(KEEP_THRES))
                    break;
                if(removed[j])
                    continue;
                if(is_overlap(box_buffer, box2_buffer))
                    removed[j] = true;
            }
        }
        return rst;
    }
}
// NOLINTEND
#endif
