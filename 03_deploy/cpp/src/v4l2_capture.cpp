#include "v4l2_capture.h"
#include <fcntl.h> <unistd.h> <sys/ioctl.h> <sys/mman.h> <linux/videodev2.h> <cstring> <iostream>
V4L2Capture::V4L2Capture():fd_(-1){for(int i=0;i<NUM_BUFFERS;i++){buffers_[i].start=nullptr;buffers_[i].length=0;}}
V4L2Capture::~V4L2Capture(){close();}
bool V4L2Capture::open(const std::string& device,int width,int height,int fps){
    width_=width;height_=height;
    fd_=::open(device.c_str(),O_RDWR|O_NONBLOCK);
    if(fd_<0){std::cerr<<"[V4L2] 无法打开: "<<device<<std::endl;return false;}
    struct v4l2_format fmt;memset(&fmt,0,sizeof(fmt));
    fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;fmt.fmt.pix.width=width;fmt.fmt.pix.height=height;
    fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;fmt.fmt.pix.field=V4L2_FIELD_NONE;
    if(ioctl(fd_,VIDIOC_S_FMT,&fmt)<0){fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_YUYV;ioctl(fd_,VIDIOC_S_FMT,&fmt);}
    struct v4l2_streamparm parm;memset(&parm,0,sizeof(parm));
    parm.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;parm.parm.capture.timeperframe.numerator=1;parm.parm.capture.timeperframe.denominator=fps;
    ioctl(fd_,VIDIOC_S_PARM,&parm);
    if(!init_mmap()||!start_streaming())return false;
    std::cout<<"[V4L2] 打开成功: "<<width<<"x"<<height<<"@"<<fps<<"fps"<<std::endl;return true;
}
bool V4L2Capture::init_mmap(){
    struct v4l2_requestbuffers req;memset(&req,0,sizeof(req));
    req.count=NUM_BUFFERS;req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;req.memory=V4L2_MEMORY_MMAP;
    if(ioctl(fd_,VIDIOC_REQBUFS,&req)<0)return false;
    for(int i=0;i<NUM_BUFFERS;i++){
        struct v4l2_buffer buf;memset(&buf,0,sizeof(buf));
        buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;buf.memory=V4L2_MEMORY_MMAP;buf.index=i;
        if(ioctl(fd_,VIDIOC_QUERYBUF,&buf)<0)return false;
        buffers_[i].length=buf.length;
        buffers_[i].start=mmap(nullptr,buf.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd_,buf.m.offset);
        if(buffers_[i].start==MAP_FAILED)return false;
        ioctl(fd_,VIDIOC_QBUF,&buf);
    }
    return true;
}
bool V4L2Capture::start_streaming(){enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;return ioctl(fd_,VIDIOC_STREAMON,&type)>=0;}
void V4L2Capture::stop_streaming(){if(fd_>=0){enum v4l2_buf_type type=V4L2_BUF_TYPE_VIDEO_CAPTURE;ioctl(fd_,VIDIOC_STREAMOFF,&type);}}
cv::Mat V4L2Capture::read(){
    struct v4l2_buffer buf;memset(&buf,0,sizeof(buf));
    buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;buf.memory=V4L2_MEMORY_MMAP;
    while(ioctl(fd_,VIDIOC_DQBUF,&buf)<0){if(errno==EAGAIN)continue;return cv::Mat();}
    cv::Mat frame(height_,width_,CV_8UC2,buffers_[buf.index].start);
    cv::Mat bgr;cv::cvtColor(frame,bgr,cv::COLOR_YUV2BGR_YUYV);
    ioctl(fd_,VIDIOC_QBUF,&buf);return bgr;
}
void V4L2Capture::close(){
    stop_streaming();
    for(int i=0;i<NUM_BUFFERS;i++){if(buffers_[i].start&&buffers_[i].start!=MAP_FAILED)munmap(buffers_[i].start,buffers_[i].length);}
    if(fd_>=0){::close(fd_);fd_=-1;}
}
