/**
 * 主程序：多线程流水线 AEB 系统（C++优化版）
 * 流水线：采集→预处理→推理→后处理+决策+CAN
 * 每个环节一个线程，用环形缓冲区连接
 */
#include <iostream> <thread> <atomic> <chrono>
#include "v4l2_capture.h" "preprocess.h" "inference_engine.h" "ring_buffer.h" "logger.h"
struct FrameData{cv::Mat image;uint64_t timestamp;};
struct TensorData{float tensor[1*3*416*416];uint64_t timestamp;int width,height;};
struct ResultData{std::vector<Detection> detections;float inference_latency;uint64_t timestamp;};
const int INPUT_SIZE=416;
RingBuffer<FrameData,2> g_cap_buf;
RingBuffer<TensorData,2> g_pre_buf;
RingBuffer<ResultData,2> g_inf_buf;
std::atomic<bool> g_running{true};
DataLogger g_logger;
void capture_thread(const std::string& device){
    V4L2Capture cap;
    if(!cap.open(device,640,480,30)){std::cerr<<"[采集] 摄像头打开失败"<<std::endl;g_running=false;return;}
    while(g_running){cv::Mat frame=cap.read();if(frame.empty())continue;
        FrameData data;data.image=frame.clone();data.timestamp=DataLogger::getTimeNs();
        g_cap_buf.push(std::move(data));}
    cap.close();
}
void preprocess_thread(){
    Preprocessor pre(INPUT_SIZE);TensorData tensor_data;
    while(g_running){FrameData frame=g_cap_buf.pop();pre.process(frame.image,tensor_data.tensor);
        tensor_data.timestamp=frame.timestamp;tensor_data.width=frame.image.cols;tensor_data.height=frame.image.rows;
        g_pre_buf.push(std::move(tensor_data));}
}
void inference_thread(InferenceEngine& engine){
    while(g_running){TensorData tensor=g_pre_buf.pop();float latency=engine.infer(tensor.tensor);
        ResultData result;result.detections=engine.getDetections();result.inference_latency=latency;result.timestamp=tensor.timestamp;
        g_inf_buf.push(std::move(result));}
}
void postprocess_thread(){
    int frame_count=0;auto last_time=std::chrono::steady_clock::now();
    while(g_running){
        ResultData result=g_inf_buf.pop();
        // TODO: 距离估计 + AEB决策 + CAN发送
        g_logger.logDecision(result.detections.size(),-1,0,result.timestamp);
        frame_count++;
        if(frame_count%30==0){
            auto now=std::chrono::steady_clock::now();
            float fps=30.0f/std::chrono::duration<float>(now-last_time).count();last_time=now;
            std::cout<<"帧#"<<frame_count<<" 推理="<<result.inference_latency<<"ms FPS="<<fps<<std::endl;
        }
    }
}
int main(int argc,char* argv[]){
    std::string camera="/dev/video0";std::string model="yolov8n_car.bin";
    if(argc>1)camera=argv[1];if(argc>2)model=argv[2];
    std::cout<<"=== AEB系统启动(C++优化版) ==="<<std::endl;
    InferenceEngine engine;if(!engine.load(model)){std::cerr<<"模型加载失败"<<std::endl;return 1;}
    g_logger.start("aeb_log.bin");
    std::thread t1(capture_thread,camera);
    std::thread t2(preprocess_thread);
    std::thread t3(inference_thread,std::ref(engine));
    std::thread t4(postprocess_thread);
    std::cout<<"按Enter停止..."<<std::endl;std::cin.get();
    g_running=false;t1.join();t2.join();t3.join();t4.join();
    g_logger.stop();std::cout<<"=== 系统已停止 ==="<<std::endl;return 0;
}
