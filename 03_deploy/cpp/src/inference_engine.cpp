#include "inference_engine.h"
#include <algorithm> <cstring> <iostream> <chrono>
InferenceEngine::InferenceEngine(){
    input_size_bytes_=1*3*input_size_*input_size_*sizeof(float);
    input_buf_=new float[1*3*input_size_*input_size_];
    output_size_bytes_=4096*8*sizeof(float);output_buf_=new float[4096*8];
}
InferenceEngine::~InferenceEngine(){delete[] input_buf_;delete[] output_buf_;}
bool InferenceEngine::load(const std::string& model_path){
    // TODO: 调用地平线hrt API加载模型
    std::cout<<"[Inference] 加载模型: "<<model_path<<std::endl;loaded_=true;return true;
}
float InferenceEngine::infer(const float* input_tensor){
    if(!loaded_)return -1;
    memcpy(input_buf_,input_tensor,input_size_bytes_);
    auto t0=std::chrono::high_resolution_clock::now();
    // TODO: 调用地平线hrt推理API: hrt_forward(model_handle_,input_buf_,output_buf_);
    usleep(38000); // 模拟38ms推理延迟
    auto t1=std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float,std::milli>(t1-t0).count();
}
std::vector<Detection> InferenceEngine::getDetections(float conf_thres,float iou_thres){
    // TODO: 从output_buf_解析YOLOv8输出 + NMS
    return {};
}
float InferenceEngine::iou(const Detection& a,const Detection& b){
    float x1=std::max(a.x1,b.x1),y1=std::max(a.y1,b.y1);
    float x2=std::min(a.x2,b.x2),y2=std::min(a.y2,b.y2);
    float inter=std::max(0.0f,x2-x1)*std::max(0.0f,y2-y1);
    float aa=(a.x2-a.x1)*(a.y2-a.y1),bb=(b.x2-b.x1)*(b.y2-b.y1);
    return inter/(aa+bb-inter+1e-6f);
}
