#include "preprocess.h"
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif
Preprocessor::Preprocessor(int input_size):input_size_(input_size){resized_.create(input_size,input_size,CV_8UC3);}
Preprocessor::~Preprocessor(){}
void Preprocessor::process(const cv::Mat& bgr,float* output){
    cv::resize(bgr,resized_,cv::Size(input_size_,input_size_));
    hwc2chw_neon(resized_.data,input_size_,input_size_,output);
}
void Preprocessor::hwc2chw_neon(const uint8_t* bgr,int h,int w,float* chw){
#ifdef __ARM_NEON
    const float scale=1.0f/255.0f;const float32x4_t vscale=vdupq_n_f32(scale);
    for(int c=0;c<3;c++){
        float* out=chw+c*h*w;int src_offset=2-c;int total=h*w;int i=0;
        for(;i+8<=total;i+=8){
            uint8x8_t pixels={bgr[(i+0)*3+src_offset],bgr[(i+1)*3+src_offset],bgr[(i+2)*3+src_offset],bgr[(i+3)*3+src_offset],bgr[(i+4)*3+src_offset],bgr[(i+5)*3+src_offset],bgr[(i+6)*3+src_offset],bgr[(i+7)*3+src_offset]};
            uint16x8_t u16=vmovl_u8(pixels);
            float32x4_t f0=vcvtq_f32_u32(vmovl_u16(vget_low_u16(u16)));
            float32x4_t f1=vcvtq_f32_u32(vmovl_u16(vget_high_u16(u16)));
            f0=vmulq_f32(f0,vscale);f1=vmulq_f32(f1,vscale);
            vst1q_f32(out+i,f0);vst1q_f32(out+i+4,f1);
        }
        for(;i<total;i++)out[i]=bgr[i*3+src_offset]*scale;
    }
#else
    const float scale=1.0f/255.0f;
    for(int c=0;c<3;c++){float* out=chw+c*h*w;int src_offset=2-c;for(int i=0;i<h*w;i++)out[i]=bgr[i*3+src_offset]*scale;}
#endif
}
