#include <cuda_runtime.h>
#include <string>
#include <stdexcept>
#include <algorithm>

static thread_local std::string error;
static void check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
struct Context {
    cudaStream_t stream=nullptr; cudaEvent_t done=nullptr;
    float *xyz=nullptr; double *boxes=nullptr; unsigned char *mask=nullptr;
    size_t point_capacity=0,box_capacity=0;
    ~Context(){if(stream)cudaStreamSynchronize(stream);cudaFree(xyz);cudaFree(boxes);cudaFree(mask);if(done)cudaEventDestroy(done);if(stream)cudaStreamDestroy(stream);}
    void reserve(size_t n,size_t b){
        if(n>point_capacity){
            size_t capacity=std::max(n,point_capacity*2);float* p=nullptr;unsigned char* m=nullptr;
            check(cudaMalloc(&p,capacity*3*sizeof(float)));
            auto e=cudaMalloc(&m,capacity);if(e!=cudaSuccess){cudaFree(p);check(e);}
            cudaFree(xyz);cudaFree(mask);xyz=p;mask=m;point_capacity=capacity;
        }
        if(b>box_capacity){double* p=nullptr;check(cudaMalloc(&p,b*24*sizeof(double)));cudaFree(boxes);boxes=p;box_capacity=b;}
    }
};
__global__ void crop(const float* xyz,int n,const double* boxes,int count,unsigned char* out){
    int i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=n)return;
    double x=xyz[i*3],y=xyz[i*3+1],z=xyz[i*3+2];out[i]=0;
    if(!isfinite(x)||!isfinite(y)||!isfinite(z))return;
    float fx=xyz[i*3],fy=xyz[i*3+1],fz=xyz[i*3+2];
    if(fx*fx+fy*fy+fz*fz<0.0025f)return;
    for(int j=0;j<count;++j){
        const double* b=boxes+j*24;
        if(x<b[18]||y<b[19]||z<b[20]||x>b[21]||y>b[22]||z>b[23])continue;
        double dx=x-b[9],dy=y-b[10],dz=z-b[11];bool hit=true;
        for(int a=0;a<3;++a){double v=dx*b[a]+dy*b[3+a]+dz*b[6+a];if(v<b[12+a]-1e-7||v>b[15+a]+1e-7){hit=false;break;}}
        if(hit){out[i]=1;return;}
    }
}
extern "C" const char* crop_error(){return error.c_str();}
extern "C" void* crop_create(){
    Context* c=new Context;
    try{int low,high;check(cudaDeviceGetStreamPriorityRange(&low,&high));check(cudaStreamCreateWithPriority(&c->stream,cudaStreamNonBlocking,low));
        check(cudaEventCreateWithFlags(&c->done,cudaEventBlockingSync|cudaEventDisableTiming));return c;
    }catch(const std::exception& e){error=e.what();delete c;return nullptr;}
}
extern "C" void crop_destroy(void* p){delete static_cast<Context*>(p);}
extern "C" int crop_run(void* p,const float* xyz,int n,const double* boxes,int count,unsigned char* out){
    Context* c=static_cast<Context*>(p);
    try{
        if(!c||n<0||count<0)throw std::runtime_error("Invalid crop arguments");
        if(n==0)return 0;
        c->reserve(n,count);
        check(cudaMemcpyAsync(c->xyz,xyz,n*3*sizeof(float),cudaMemcpyHostToDevice,c->stream));
        if(count)check(cudaMemcpyAsync(c->boxes,boxes,count*24*sizeof(double),cudaMemcpyHostToDevice,c->stream));
        crop<<<(n+127)/128,128,0,c->stream>>>(c->xyz,n,c->boxes,count,c->mask);check(cudaGetLastError());
        check(cudaMemcpyAsync(out,c->mask,n,cudaMemcpyDeviceToHost,c->stream));
        check(cudaEventRecord(c->done,c->stream));check(cudaEventSynchronize(c->done));return 0;
    }catch(const std::exception& e){error=e.what();if(c)cudaStreamSynchronize(c->stream);return -1;}
}
