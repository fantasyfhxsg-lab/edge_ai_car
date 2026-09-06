#include "logger.h"
#include <chrono> <cstring> <iostream>
DataLogger::DataLogger(){}
DataLogger::~DataLogger(){stop();}
uint64_t DataLogger::getTimeNs(){
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool DataLogger::start(const std::string& filename){
    file_.open(filename,std::ios::binary|std::ios::trunc);
    if(!file_.is_open()){std::cerr<<"[Logger] 无法打开: "<<filename<<std::endl;return false;}
    char magic[4]={'C','A','R','L'};file_.write(magic,4);
    uint32_t version=1;file_.write(reinterpret_cast<char*>(&version),4);
    uint64_t start_time=getTimeNs();file_.write(reinterpret_cast<char*>(&start_time),8);
    running_=true;write_thread_=std::thread(&DataLogger::writeLoop,this);
    std::cout<<"[Logger] 开始记录: "<<filename<<std::endl;return true;
}
void DataLogger::stop(){
    running_=false;
    if(write_thread_.joinable())write_thread_.join();
    if(file_.is_open())file_.close();
}
void DataLogger::logImage(const cv::Mat& frame,uint64_t timestamp){
    std::vector<uchar> jpeg;cv::imencode(".jpg",frame,jpeg,{cv::IMWRITE_JPEG_QUALITY,70});
    LogEntry entry;entry.type=TYPE_IMAGE;entry.timestamp=timestamp;entry.data=std::vector<uint8_t>(jpeg.begin(),jpeg.end());
    std::lock_guard<std::mutex> lock(mtx_);queue_.push(std::move(entry));
}
void DataLogger::logCAN(uint32_t can_id,const uint8_t* data,uint8_t len,uint64_t timestamp){
    LogEntry entry;entry.type=TYPE_CAN;entry.timestamp=timestamp;entry.data.resize(4+len);
    memcpy(entry.data.data(),&can_id,4);memcpy(entry.data.data()+4,data,len);
    std::lock_guard<std::mutex> lock(mtx_);queue_.push(std::move(entry));
}
void DataLogger::logDecision(int num_objects,float min_distance,int aeb_state,uint64_t timestamp){
    LogEntry entry;entry.type=TYPE_DECISION;entry.timestamp=timestamp;entry.data.resize(12);
    int32_t n=num_objects;float dist=min_distance;int32_t state=aeb_state;
    memcpy(entry.data.data(),&n,4);memcpy(entry.data.data()+4,&dist,4);memcpy(entry.data.data()+8,&state,4);
    std::lock_guard<std::mutex> lock(mtx_);queue_.push(std::move(entry));
}
void DataLogger::writeLoop(){
    while(running_||!queue_.empty()){
        LogEntry entry;{std::lock_guard<std::mutex> lock(mtx_);
            if(queue_.empty()){std::this_thread::sleep_for(std::chrono::milliseconds(1));continue;}
            entry=std::move(queue_.front());queue_.pop();}
        writeEntry(entry);
    }
}
void DataLogger::writeEntry(const LogEntry& entry){
    uint8_t type=static_cast<uint8_t>(entry.type);uint32_t length=entry.data.size();
    file_.write(reinterpret_cast<const char*>(&type),1);
    file_.write(reinterpret_cast<const char*>(&entry.timestamp),8);
    file_.write(reinterpret_cast<const char*>(&length),4);
    file_.write(reinterpret_cast<const char*>(entry.data.data()),length);
}
