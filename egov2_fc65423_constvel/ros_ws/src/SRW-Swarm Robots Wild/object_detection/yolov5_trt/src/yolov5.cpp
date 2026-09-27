#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <object_detection_msgs/BoundingBox.h>
#include <object_detection_msgs/BoundingBoxes.h>
#include <object_detection_msgs/ObjectCount.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include "yolov5_trt/cuda_utils.h"
#include "yolov5_trt/logging.h"
#include "yolov5_trt/common.hpp"
#include "yolov5_trt/utils.h"
#include "yolov5_trt/calibrator.h"

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1

#define NET s  // s m l x
#define NETSTRUCT(str) createEngine_##str
#define CREATENET(net) NETSTRUCT(net)
#define STR1(x) #x
#define STR2(x) STR1(x)

// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;

// Creat the engine using only the API and not any parser.
ICudaEngine* createEngine_s(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt) {
    INetworkDefinition* network = builder->createNetworkV2(0U);

    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor* data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{ 3, INPUT_H, INPUT_W });
    assert(data);

    std::map<std::string, Weights> weightMap = loadWeights("../yolov5s.wts");
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    // yolov5 backbone
    auto focus0 = focus(network, weightMap, *data, 3, 32, 3, "model.0");
    auto conv1 = convBlock(network, weightMap, *focus0->getOutput(0), 64, 3, 2, 1, "model.1");
    auto bottleneck_CSP2 = C3(network, weightMap, *conv1->getOutput(0), 64, 64, 1, true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *bottleneck_CSP2->getOutput(0), 128, 3, 2, 1, "model.3");
    auto bottleneck_csp4 = C3(network, weightMap, *conv3->getOutput(0), 128, 128, 3, true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *bottleneck_csp4->getOutput(0), 256, 3, 2, 1, "model.5");
    auto bottleneck_csp6 = C3(network, weightMap, *conv5->getOutput(0), 256, 256, 3, true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *bottleneck_csp6->getOutput(0), 512, 3, 2, 1, "model.7");
    auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 512, 512, 5, 9, 13, "model.8");

    // yolov5 head
    auto bottleneck_csp9 = C3(network, weightMap, *spp8->getOutput(0), 512, 512, 1, false, 1, 0.5, "model.9");
    auto conv10 = convBlock(network, weightMap, *bottleneck_csp9->getOutput(0), 256, 1, 1, 1, "model.10");

    float *deval = reinterpret_cast<float*>(malloc(sizeof(float) * 256 * 2 * 2));
    for (int i = 0; i < 256 * 2 * 2; i++) {
        deval[i] = 1.0;
    }
    Weights deconvwts11{ DataType::kFLOAT, deval, 256 * 2 * 2 };
    IDeconvolutionLayer* deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 256, DimsHW{ 2, 2 }, deconvwts11, emptywts);
    deconv11->setStrideNd(DimsHW{ 2, 2 });
    deconv11->setNbGroups(256);
    weightMap["deconv11"] = deconvwts11;

    ITensor* inputTensors12[] = { deconv11->getOutput(0), bottleneck_csp6->getOutput(0) };
    auto cat12 = network->addConcatenation(inputTensors12, 2);
    auto bottleneck_csp13 = C3(network, weightMap, *cat12->getOutput(0), 512, 256, 1, false, 1, 0.5, "model.13");
    auto conv14 = convBlock(network, weightMap, *bottleneck_csp13->getOutput(0), 128, 1, 1, 1, "model.14");

    Weights deconvwts15{ DataType::kFLOAT, deval, 128 * 2 * 2 };
    IDeconvolutionLayer* deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 128, DimsHW{ 2, 2 }, deconvwts15, emptywts);
    deconv15->setStrideNd(DimsHW{ 2, 2 });
    deconv15->setNbGroups(128);

    ITensor* inputTensors16[] = { deconv15->getOutput(0), bottleneck_csp4->getOutput(0) };
    auto cat16 = network->addConcatenation(inputTensors16, 2);
    auto bottleneck_csp17 = C3(network, weightMap, *cat16->getOutput(0), 256, 128, 1, false, 1, 0.5, "model.17");
    IConvolutionLayer* det0 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.0.weight"], weightMap["model.24.m.0.bias"]);

    auto conv18 = convBlock(network, weightMap, *bottleneck_csp17->getOutput(0), 128, 3, 2, 1, "model.18");
    ITensor* inputTensors19[] = { conv18->getOutput(0), conv14->getOutput(0) };
    auto cat19 = network->addConcatenation(inputTensors19, 2);
    auto bottleneck_csp20 = C3(network, weightMap, *cat19->getOutput(0), 256, 256, 1, false, 1, 0.5, "model.20");
    IConvolutionLayer* det1 = network->addConvolutionNd(*bottleneck_csp20->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.1.weight"], weightMap["model.24.m.1.bias"]);

    auto conv21 = convBlock(network, weightMap, *bottleneck_csp20->getOutput(0), 256, 3, 2, 1, "model.21");
    ITensor* inputTensors22[] = { conv21->getOutput(0), conv10->getOutput(0) };
    auto cat22 = network->addConcatenation(inputTensors22, 2);
    auto bottleneck_csp23 = C3(network, weightMap, *cat22->getOutput(0), 512, 512, 1, false, 1, 0.5, "model.23");
    IConvolutionLayer* det2 = network->addConvolutionNd(*bottleneck_csp23->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.2.weight"], weightMap["model.24.m.2.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, det0, det1, det2);
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));

    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2 *calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto& mem : weightMap)
    {
        free((void*)(mem.second.values));

    }
    return engine;
}

ICudaEngine* createEngine_m(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt) {
    INetworkDefinition* network = builder->createNetworkV2(0U);

    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor* data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{ 3, INPUT_H, INPUT_W });
    assert(data);

    std::map<std::string, Weights> weightMap = loadWeights("../yolov5m.wts");
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    /* ------ yolov5 backbone------ */
    auto focus0 = focus(network, weightMap, *data, 3, 48, 3, "model.0");
    auto conv1 = convBlock(network, weightMap, *focus0->getOutput(0), 96, 3, 2, 1, "model.1");
    auto bottleneck_CSP2 = C3(network, weightMap, *conv1->getOutput(0), 96, 96, 2, true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *bottleneck_CSP2->getOutput(0), 192, 3, 2, 1, "model.3");
    auto bottleneck_csp4 = C3(network, weightMap, *conv3->getOutput(0), 192, 192, 6, true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *bottleneck_csp4->getOutput(0), 384, 3, 2, 1, "model.5");
    auto bottleneck_csp6 = C3(network, weightMap, *conv5->getOutput(0), 384, 384, 6, true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *bottleneck_csp6->getOutput(0), 768, 3, 2, 1, "model.7");
    auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 768, 768, 5, 9, 13, "model.8");
    /* ------ yolov5 head ------ */
    auto bottleneck_csp9 = C3(network, weightMap, *spp8->getOutput(0), 768, 768, 2, false, 1, 0.5, "model.9");
    auto conv10 = convBlock(network, weightMap, *bottleneck_csp9->getOutput(0), 384, 1, 1, 1, "model.10");

    float *deval = reinterpret_cast<float*>(malloc(sizeof(float) * 384 * 2 * 2));
    for (int i = 0; i < 384 * 2 * 2; i++) {
        deval[i] = 1.0;
    }
    Weights deconvwts11{ DataType::kFLOAT, deval, 384 * 2 * 2 };
    IDeconvolutionLayer* deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 384, DimsHW{ 2, 2 }, deconvwts11, emptywts);
    deconv11->setStrideNd(DimsHW{ 2, 2 });
    deconv11->setNbGroups(384);
    weightMap["deconv11"] = deconvwts11;
    ITensor* inputTensors12[] = { deconv11->getOutput(0), bottleneck_csp6->getOutput(0) };
    auto cat12 = network->addConcatenation(inputTensors12, 2);

    auto bottleneck_csp13 = C3(network, weightMap, *cat12->getOutput(0), 768, 384, 2, false, 1, 0.5, "model.13");

    auto conv14 = convBlock(network, weightMap, *bottleneck_csp13->getOutput(0), 192, 1, 1, 1, "model.14");

    Weights deconvwts15{ DataType::kFLOAT, deval, 192 * 2 * 2 };
    IDeconvolutionLayer* deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 192, DimsHW{ 2, 2 }, deconvwts15, emptywts);
    deconv15->setStrideNd(DimsHW{ 2, 2 });
    deconv15->setNbGroups(192);

    ITensor* inputTensors16[] = { deconv15->getOutput(0), bottleneck_csp4->getOutput(0) };
    auto cat16 = network->addConcatenation(inputTensors16, 2);
    auto bottleneck_csp17 = C3(network, weightMap, *cat16->getOutput(0), 384, 192, 2, false, 1, 0.5, "model.17");

    //yolo layer 0
    IConvolutionLayer* det0 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.0.weight"], weightMap["model.24.m.0.bias"]);
    auto conv18 = convBlock(network, weightMap, *bottleneck_csp17->getOutput(0), 192, 3, 2, 1, "model.18");
    ITensor* inputTensors19[] = { conv18->getOutput(0), conv14->getOutput(0) };
    auto cat19 = network->addConcatenation(inputTensors19, 2);
    auto bottleneck_csp20 = C3(network, weightMap, *cat19->getOutput(0), 384, 384, 2, false, 1, 0.5, "model.20");

    //yolo layer 1
    IConvolutionLayer* det1 = network->addConvolutionNd(*bottleneck_csp20->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.1.weight"], weightMap["model.24.m.1.bias"]);
    auto conv21 = convBlock(network, weightMap, *bottleneck_csp20->getOutput(0), 384, 3, 2, 1, "model.21");
    ITensor* inputTensors22[] = { conv21->getOutput(0), conv10->getOutput(0) };
    auto cat22 = network->addConcatenation(inputTensors22, 2);
    auto bottleneck_csp23 = C3(network, weightMap, *cat22->getOutput(0), 768, 768, 2, false, 1, 0.5, "model.23");
    // yolo layer 2
    IConvolutionLayer* det2 = network->addConvolutionNd(*bottleneck_csp23->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.2.weight"], weightMap["model.24.m.2.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, det0, det1, det2);
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));

    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2 *calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto& mem : weightMap)
    {
        free((void*)(mem.second.values));
    }

    return engine;
}

ICudaEngine* createEngine_l(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt) {
    INetworkDefinition* network = builder->createNetworkV2(0U);

    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor* data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{ 3, INPUT_H, INPUT_W });
    assert(data);

    std::map<std::string, Weights> weightMap = loadWeights("../yolov5l.wts");
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    /* ------ yolov5 backbone------ */
    auto focus0 = focus(network, weightMap, *data, 3, 64, 3, "model.0");
    auto conv1 = convBlock(network, weightMap, *focus0->getOutput(0), 128, 3, 2, 1, "model.1");
    auto bottleneck_CSP2 = C3(network, weightMap, *conv1->getOutput(0), 128, 128, 3, true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *bottleneck_CSP2->getOutput(0), 256, 3, 2, 1, "model.3");
    auto bottleneck_csp4 = C3(network, weightMap, *conv3->getOutput(0), 256, 256, 9, true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *bottleneck_csp4->getOutput(0), 512, 3, 2, 1, "model.5");
    auto bottleneck_csp6 = C3(network, weightMap, *conv5->getOutput(0), 512, 512, 9, true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *bottleneck_csp6->getOutput(0), 1024, 3, 2, 1, "model.7");
    auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 1024, 1024, 5, 9, 13, "model.8");

    /* ------ yolov5 head ------ */
    auto bottleneck_csp9 = C3(network, weightMap, *spp8->getOutput(0), 1024, 1024, 3, false, 1, 0.5, "model.9");
    auto conv10 = convBlock(network, weightMap, *bottleneck_csp9->getOutput(0), 512, 1, 1, 1, "model.10");

    float *deval = reinterpret_cast<float*>(malloc(sizeof(float) * 512 * 2 * 2));
    for (int i = 0; i < 512 * 2 * 2; i++) {
        deval[i] = 1.0;
    }
    Weights deconvwts11{ DataType::kFLOAT, deval, 512 * 2 * 2 };
    IDeconvolutionLayer* deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 512, DimsHW{ 2, 2 }, deconvwts11, emptywts);
    deconv11->setStrideNd(DimsHW{ 2, 2 });
    deconv11->setNbGroups(512);
    weightMap["deconv11"] = deconvwts11;

    ITensor* inputTensors12[] = { deconv11->getOutput(0), bottleneck_csp6->getOutput(0) };
    auto cat12 = network->addConcatenation(inputTensors12, 2);
    auto bottleneck_csp13 = C3(network, weightMap, *cat12->getOutput(0), 1024, 512, 3, false, 1, 0.5, "model.13");
    auto conv14 = convBlock(network, weightMap, *bottleneck_csp13->getOutput(0), 256, 1, 1, 1, "model.14");

    Weights deconvwts15{ DataType::kFLOAT, deval, 256 * 2 * 2 };
    IDeconvolutionLayer* deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 256, DimsHW{ 2, 2 }, deconvwts15, emptywts);
    deconv15->setStrideNd(DimsHW{ 2, 2 });
    deconv15->setNbGroups(256);
    ITensor* inputTensors16[] = { deconv15->getOutput(0), bottleneck_csp4->getOutput(0) };
    auto cat16 = network->addConcatenation(inputTensors16, 2);

    auto bottleneck_csp17 = C3(network, weightMap, *cat16->getOutput(0), 512, 256, 3, false, 1, 0.5, "model.17");

    // yolo layer 0
    IConvolutionLayer* det0 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.0.weight"], weightMap["model.24.m.0.bias"]);
    auto conv18 = convBlock(network, weightMap, *bottleneck_csp17->getOutput(0), 256, 3, 2, 1, "model.18");
    ITensor* inputTensors19[] = { conv18->getOutput(0), conv14->getOutput(0) };
    auto cat19 = network->addConcatenation(inputTensors19, 2);
    auto bottleneck_csp20 = C3(network, weightMap, *cat19->getOutput(0), 512, 512, 3, false, 1, 0.5, "model.20");
    //yolo layer 1
    IConvolutionLayer* det1 = network->addConvolutionNd(*bottleneck_csp20->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.1.weight"], weightMap["model.24.m.1.bias"]);
    auto conv21 = convBlock(network, weightMap, *bottleneck_csp20->getOutput(0), 512, 3, 2, 1, "model.21");
    ITensor* inputTensors22[] = { conv21->getOutput(0), conv10->getOutput(0) };
    auto cat22 = network->addConcatenation(inputTensors22, 2);
    auto bottleneck_csp23 = C3(network, weightMap, *cat22->getOutput(0), 1024, 1024, 3, false, 1, 0.5, "model.23");
    IConvolutionLayer* det2 = network->addConvolutionNd(*bottleneck_csp23->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.2.weight"], weightMap["model.24.m.2.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, det0, det1, det2);
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));

    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2 *calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto& mem : weightMap)
    {
        free((void*)(mem.second.values));
    }

    return engine;
}

ICudaEngine* createEngine_x(unsigned int maxBatchSize, IBuilder* builder, IBuilderConfig* config, DataType dt) {
    INetworkDefinition* network = builder->createNetworkV2(0U);

    // Create input tensor of shape {3, INPUT_H, INPUT_W} with name INPUT_BLOB_NAME
    ITensor* data = network->addInput(INPUT_BLOB_NAME, dt, Dims3{ 3, INPUT_H, INPUT_W });
    assert(data);

    std::map<std::string, Weights> weightMap = loadWeights("../yolov5x.wts");
    Weights emptywts{ DataType::kFLOAT, nullptr, 0 };

    /* ------ yolov5 backbone------ */
    auto focus0 = focus(network, weightMap, *data, 3, 80, 3, "model.0");
    auto conv1 = convBlock(network, weightMap, *focus0->getOutput(0), 160, 3, 2, 1, "model.1");
    auto bottleneck_CSP2 = C3(network, weightMap, *conv1->getOutput(0), 160, 160, 4, true, 1, 0.5, "model.2");
    auto conv3 = convBlock(network, weightMap, *bottleneck_CSP2->getOutput(0), 320, 3, 2, 1, "model.3");
    auto bottleneck_csp4 = C3(network, weightMap, *conv3->getOutput(0), 320, 320, 12, true, 1, 0.5, "model.4");
    auto conv5 = convBlock(network, weightMap, *bottleneck_csp4->getOutput(0), 640, 3, 2, 1, "model.5");
    auto bottleneck_csp6 = C3(network, weightMap, *conv5->getOutput(0), 640, 640, 12, true, 1, 0.5, "model.6");
    auto conv7 = convBlock(network, weightMap, *bottleneck_csp6->getOutput(0), 1280, 3, 2, 1, "model.7");
    auto spp8 = SPP(network, weightMap, *conv7->getOutput(0), 1280, 1280, 5, 9, 13, "model.8");

    /* ------- yolov5 head ------- */
    auto bottleneck_csp9 = C3(network, weightMap, *spp8->getOutput(0), 1280, 1280, 4, false, 1, 0.5, "model.9");
    auto conv10 = convBlock(network, weightMap, *bottleneck_csp9->getOutput(0), 640, 1, 1, 1, "model.10");

    float *deval = reinterpret_cast<float*>(malloc(sizeof(float) * 640 * 2 * 2));
    for (int i = 0; i < 640 * 2 * 2; i++) {
        deval[i] = 1.0;
    }
    Weights deconvwts11{ DataType::kFLOAT, deval, 640 * 2 * 2 };
    IDeconvolutionLayer* deconv11 = network->addDeconvolutionNd(*conv10->getOutput(0), 640, DimsHW{ 2, 2 }, deconvwts11, emptywts);
    deconv11->setStrideNd(DimsHW{ 2, 2 });
    deconv11->setNbGroups(640);
    weightMap["deconv11"] = deconvwts11;

    ITensor* inputTensors12[] = { deconv11->getOutput(0), bottleneck_csp6->getOutput(0) };
    auto cat12 = network->addConcatenation(inputTensors12, 2);

    auto bottleneck_csp13 = C3(network, weightMap, *cat12->getOutput(0), 1280, 640, 4, false, 1, 0.5, "model.13");
    auto conv14 = convBlock(network, weightMap, *bottleneck_csp13->getOutput(0), 320, 1, 1, 1, "model.14");

    Weights deconvwts15{ DataType::kFLOAT, deval, 320 * 2 * 2 };
    IDeconvolutionLayer* deconv15 = network->addDeconvolutionNd(*conv14->getOutput(0), 320, DimsHW{ 2, 2 }, deconvwts15, emptywts);
    deconv15->setStrideNd(DimsHW{ 2, 2 });
    deconv15->setNbGroups(320);
    ITensor* inputTensors16[] = { deconv15->getOutput(0), bottleneck_csp4->getOutput(0) };
    auto cat16 = network->addConcatenation(inputTensors16, 2);

    auto bottleneck_csp17 = C3(network, weightMap, *cat16->getOutput(0), 640, 320, 4, false, 1, 0.5, "model.17");

    // yolo layer 0
    IConvolutionLayer* det0 = network->addConvolutionNd(*bottleneck_csp17->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.0.weight"], weightMap["model.24.m.0.bias"]);
    auto conv18 = convBlock(network, weightMap, *bottleneck_csp17->getOutput(0), 320, 3, 2, 1, "model.18");
    ITensor* inputTensors19[] = { conv18->getOutput(0), conv14->getOutput(0) };
    auto cat19 = network->addConcatenation(inputTensors19, 2);
    auto bottleneck_csp20 = C3(network, weightMap, *cat19->getOutput(0), 640, 640, 4, false, 1, 0.5, "model.20");
    // yolo layer 1
    IConvolutionLayer* det1 = network->addConvolutionNd(*bottleneck_csp20->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.1.weight"], weightMap["model.24.m.1.bias"]);
    auto conv21 = convBlock(network, weightMap, *bottleneck_csp20->getOutput(0), 640, 3, 2, 1, "model.21");
    ITensor* inputTensors22[] = { conv21->getOutput(0), conv10->getOutput(0) };
    auto cat22 = network->addConcatenation(inputTensors22, 2);
    auto bottleneck_csp23 = C3(network, weightMap, *cat22->getOutput(0), 1280, 1280, 4, false, 1, 0.5, "model.23");
    // yolo layer 2
    IConvolutionLayer* det2 = network->addConvolutionNd(*bottleneck_csp23->getOutput(0), 3 * (Yolo::CLASS_NUM + 5), DimsHW{ 1, 1 }, weightMap["model.24.m.2.weight"], weightMap["model.24.m.2.bias"]);

    auto yolo = addYoLoLayer(network, weightMap, det0, det1, det2);
    yolo->getOutput(0)->setName(OUTPUT_BLOB_NAME);
    network->markOutput(*yolo->getOutput(0));

    // Build engine
    builder->setMaxBatchSize(maxBatchSize);
    config->setMaxWorkspaceSize(16 * (1 << 20));  // 16MB
#if defined(USE_FP16)
    config->setFlag(BuilderFlag::kFP16);
#elif defined(USE_INT8)
    std::cout << "Your platform support int8: " << (builder->platformHasFastInt8() ? "true" : "false") << std::endl;
    assert(builder->platformHasFastInt8());
    config->setFlag(BuilderFlag::kINT8);
    Int8EntropyCalibrator2 *calibrator = new Int8EntropyCalibrator2(1, INPUT_W, INPUT_H, "./coco_calib/", "int8calib.table", INPUT_BLOB_NAME);
    config->setInt8Calibrator(calibrator);
#endif

    std::cout << "Building engine, please wait for a while..." << std::endl;
    ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);
    std::cout << "Build engine successfully!" << std::endl;

    // Don't need the network any more
    network->destroy();

    // Release host memory
    for (auto& mem : weightMap)
    {
        free((void*)(mem.second.values));
    }

    return engine;
}

void APIToModel(unsigned int maxBatchSize, IHostMemory** modelStream) {
    // Create builder
    IBuilder* builder = createInferBuilder(gLogger);
    IBuilderConfig* config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine* engine = (CREATENET(NET))(maxBatchSize, builder, config, DataType::kFLOAT);
    //ICudaEngine* engine = createEngine(maxBatchSize, builder, config, DataType::kFLOAT);
    assert(engine != nullptr);

    // Serialize the engine
    (*modelStream) = engine->serialize();

    // Close everything down
    engine->destroy();
    builder->destroy();
}

void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize) {
    // const ICudaEngine& engine = context.getEngine();

    // assert(engine.getNbBindings() == 2);
    // void* buffers[2];
    // // In order to bind the buffers, we need to know the names of the input and output tensors.
    // // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    // const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);
    // const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);
    // assert(inputIndex == 0);
    // assert(outputIndex == 1);
    // // Create GPU buffers on device
    // CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
    // CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));
    // // Create stream
    // cudaStream_t stream;
    // CUDA_CHECK(cudaStreamCreate(&stream));
    
    
    
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);

    // cudaStreamDestroy(stream);
    // CUDA_CHECK(cudaFree(buffers[inputIndex]));
    // CUDA_CHECK(cudaFree(buffers[outputIndex]));
}

//ros subscriber
ros::Subscriber img_sub;

//ros publisher
ros::Publisher img_pub;
ros::Publisher bboxes_pub;
ros::Publisher objectcnt_pub;

bool isdetect = false;
bool if_pub_image = false;
ros::Time last_detect_time;
int image_num = 0; 

static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
static float prob[BATCH_SIZE * OUTPUT_SIZE];
// ICudaEngine* engine;
// IRuntime* runtime;
IExecutionContext* context;
// static const std::string OPENCV_WINDOW = "Image window";
// std::vector<Yolo::Detection> last_res;
cudaStream_t stream;
void* buffers[2];

//image calibrator Matrix
//double arr[3][3] = {{886.9746218077141, 0, 956.3277141856945},{ 0, 887.145203462317, 604.9874601334124},{ 0, 0, 1}};
//cv::Mat cameraMatrix (3,3, CV_64FC1,arr);
//cv::Mat inv_cameraMatrix;

//矫正畸变
// double arr1[5] = {-0.0002713973318024921, 0.001030391492847567, 0.0004310183175245195, 0.0002994399191058541, -0.01240890576907068};
// cv::Mat distCoeffs(1,5,CV_64FC1,arr1);

void imageCallback(const sensor_msgs::ImageConstPtr& img){
    // std::cout << "****************************imageCallback: " << image_num << "****************************" << std::endl;
    ++image_num;

    object_detection_msgs::BoundingBoxes bboxes_msg;
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    } 
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge execption: %s", e.what());
        return;
    }


    //image calibrator
    cv::Mat origin_img, in_img, out_img;
    cv_ptr->image.copyTo(origin_img);

    in_img = origin_img;
    out_img = in_img;

    if(!in_img.empty()){
        cv::Mat pr_img = preprocess_img(in_img, INPUT_W, INPUT_H); // letterbox BGR to RGB
        int i = 0;
        for (int row = 0; row < INPUT_H; ++row) {
            uchar* uc_pixel = pr_img.data + row * pr_img.step;
            for (int col = 0; col < INPUT_W; ++col) {
                data[i] = (float)uc_pixel[2] / 255.0;
                data[i + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
                data[i + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[0] / 255.0;
                uc_pixel += 3;
                ++i;
            }
        }

        // Run inference
        doInference(*context, stream, buffers, data, prob, BATCH_SIZE);

        std::vector<Yolo::Detection> res;
        nms(res, prob, CONF_THRESH, NMS_THRESH);
        object_detection_msgs::BoundingBox bbox_msg;
        object_detection_msgs::ObjectCount cnt_msg;

        last_detect_time = img -> header.stamp;
        for (size_t j = 0; j < res.size(); j++) {
            bbox_msg.id = (int)res[j].class_id;
            cv::Rect r = get_rect(out_img, res[j].bbox);
            bbox_msg.probability = res[j].conf;
            auto top_left = r.tl();
            auto bottom_right = r.br();
            bbox_msg.xmin = top_left.x;
            bbox_msg.ymin = top_left.y;
            bbox_msg.xmax = bottom_right.x;
            bbox_msg.ymax = bottom_right.y;
            bboxes_msg.bounding_boxes.push_back(bbox_msg);

            if ( if_pub_image )
            {
                cv::rectangle(out_img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
                cv::putText(out_img, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
            }
        }

        if(res.size() > 0){
            cnt_msg.header.stamp  = ros::Time::now() - ros::Duration(0.15);
            cnt_msg.header.frame_id = "detection";
            cnt_msg.count = res.size();
            objectcnt_pub.publish(cnt_msg);

            bboxes_msg.header.stamp = ros::Time::now() - ros::Duration(0.15);
            bboxes_msg.header.frame_id = "detection";
            bboxes_pub.publish(bboxes_msg);
            
            if ( if_pub_image )
            {
                cv_ptr->image = out_img.clone();
                img_pub.publish(cv_ptr->toImageMsg());
            }
        }
    }
    else{
        ROS_WARN("in_img empty!");
    }
}

void compressedimageCallback(const sensor_msgs::CompressedImageConstPtr& img){
    object_detection_msgs::BoundingBoxes bboxes_msg;
    //std::cout << "image callback successful!!!\n" << std::endl;
    ++image_num;
    //std::cout << image_num << std::endl;
    auto start = std::chrono::system_clock::now();
    
    cv::Mat origin_img;
    try {
        //cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::RGB8);
        //cv::Mat origin_img = cv_ptr_compressed->image;
        origin_img  = cv::imdecode(cv::Mat(img->data),1);
    } 
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge execption: %s", e.what());
        return;
    }   

    //image calibrator
    cv::Mat in_img, out_img;

    //cv::undistort(origin_img, in_img, cameraMatrix, distCoeffs);
  
    in_img = origin_img;
    out_img = in_img;

    int fcount = 0;
    if(!in_img.empty()){
        fcount++;
        if (fcount < BATCH_SIZE ) return;
        for (int b = 0; b < fcount; b++) {
            // cv::Mat img = cv::imread(std::string(pict_dir) + "/" + file_names[f - fcount + 1 + b]);
            if (in_img.empty()) return;
            cv::Mat pr_img = preprocess_img(in_img, INPUT_W, INPUT_H); // letterbox BGR to RGB
            int i = 0;
            for (int row = 0; row < INPUT_H; ++row) {
                uchar* uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col) {
                    data[b * 3 * INPUT_H * INPUT_W + i] = (float)uc_pixel[2] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++i;
                }
            }
        }
        // Run inference
        // auto start = std::chrono::system_clock::now();
        doInference(*context,  stream, buffers,data, prob, BATCH_SIZE);
        // auto end = std::chrono::system_clock::now();
        // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        std::vector<std::vector<Yolo::Detection>> batch_res(fcount);
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
        }
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            object_detection_msgs::BoundingBox bbox_msg;
            object_detection_msgs::ObjectCount  cnt_msg;
            //std::cout << res.size() << std::endl;
            if(res.size() == 0 ){
                isdetect = false;
                cnt_msg.header.stamp = ros::Time::now();
                cnt_msg.header.frame_id = "detection";
                cnt_msg.count = 0;
                objectcnt_pub.publish(cnt_msg);
            }
            else{
                isdetect =  true;
                last_detect_time = img -> header.stamp;
                 for (size_t j = 0; j < res.size(); j++) {
                    bbox_msg.id = (int)res[j].class_id;
                    if (bbox_msg.id == 0)
                        bbox_msg.Class = std::string("drone");
                    
                    cv::Rect r = get_rect(out_img, res[j].bbox);

                    /////////
                    cv::rectangle(out_img, r, CV_RGB(255,0,255),5);

                    bbox_msg.probability = res[j].conf;
                    auto top_left = r.tl();
                    auto bottom_right = r.br();
                    bbox_msg.xmin = top_left.x;

                    auto t1 = std::chrono::system_clock::now();
                    cv::Mat coor_imageplane = (cv::Mat_<double>(3,1)<< top_left.x, top_left.y, 1);

                    //auto top_left_x_cam = inv_cameraMatrix * coor_imageplane;
                    auto t2 = std::chrono::system_clock::now();
                    std::cout << "t2-t1 = " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms" << std::endl;
                    //std::cout << top_left << std::endl;
                    //std::cout << top_left_x_cam << std::endl;
                    
                    bbox_msg.ymin = top_left.y;
                    bbox_msg.xmax = bottom_right.x;
                    bbox_msg.ymax = bottom_right.y;
                    bboxes_msg.bounding_boxes.push_back(bbox_msg);
            
                }

                cnt_msg.header.stamp  = ros::Time::now();
                cnt_msg.header.frame_id = "detection";
                cnt_msg.count = res.size();
                objectcnt_pub.publish(cnt_msg);

                bboxes_msg.header.stamp = ros::Time::now();
                bboxes_msg.header.frame_id = "detection";
                bboxes_pub.publish(bboxes_msg);
            }

        }
        fcount = 0;
        //cv_bridge::CvImagePtr cv_ptr;
        if ( if_pub_image )
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img,sensor_msgs::image_encodings::BGR8);
            cv_ptr->image = out_img.clone();
            img_pub.publish(cv_ptr->toImageMsg());
        }
    }
    else{
                ROS_WARN("can't get valid image");
    }
    auto end = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv,"yolov5trt");
    ros::NodeHandle nh("~");

    nh.param("if_pub_image", if_pub_image, false);

    //cameraMtrix inv
    //cv::invert(cameraMatrix,inv_cameraMatrix);

    //subscribe image topic
    // img_sub = nh.subscribe("/video_source/raw/compressed", 10, compressedimageCallback, ros::TransportHints().tcpNoDelay());
    img_sub = nh.subscribe("/video_source/raw", 10, imageCallback, ros::TransportHints().tcpNoDelay());

    //publish msgs
    img_pub = nh.advertise<sensor_msgs::Image>("img_pub",1);
    objectcnt_pub = nh.advertise<object_detection_msgs::ObjectCount>("objectcnt_pub",1);
    bboxes_pub = nh.advertise<object_detection_msgs:: BoundingBoxes>("bboxes_pub",1);

    cudaSetDevice(DEVICE);
    // create a model using the API directly and serialize it to a stream
    char *trtModelStream{ nullptr };
    size_t size{ 0 };
    std::string engine_name = STR2(NET);
    engine_name = "/home/nv/ego-swarm-v2_tracking/src/realflight_modules/object_detection/yolov5_trt/src/best.engine";

    std::ifstream file(engine_name, std::ios::binary);
    if (file.good()) {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    }
    else{
        printf("can't load model/n");
    }


    IRuntime* runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;


    assert(engine->getNbBindings() == 2);
    // void* buffers[2];
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));
    // Create stream
    // cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));

    ros::spin();
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));

    ROS_WARN("end");

    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
    //printf("xunzhiren");

    return 0;
}
