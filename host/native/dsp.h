#pragma once
#include <array>
#include <complex>
#include <cmath>
#include <algorithm>
#include <memory>

static constexpr double pi=3.14159265358979323846;
static void fft(std::vector<std::complex<double>> &a) {
    const size_t n=a.size();
    for(size_t i=1,j=0;i<n;i++) {size_t bit=n>>1;for(;j&bit;bit>>=1)j^=bit;j^=bit;if(i<j)std::swap(a[i],a[j]);}
    for(size_t len=2;len<=n;len<<=1) {
        const std::complex<double> step=std::polar(1.0,-2*pi/len);
        for(size_t i=0;i<n;i+=len) {std::complex<double> w=1;
            for(size_t j=0;j<len/2;j++){auto u=a[i+j],v=a[i+j+len/2]*w;a[i+j]=u+v;a[i+j+len/2]=u-v;w*=step;}
        }
    }
}
struct DspBlock {Header header;std::array<int8_t,4032> iq;};
class Processor {
    static constexpr size_t capacity=4096;
    std::unique_ptr<DspBlock[]> queue{new DspBlock[capacity]{}};
    alignas(64) std::atomic<size_t> head{0};
    alignas(64) std::atomic<size_t> tail{0};
    std::atomic<bool> closed{false};
    std::thread thread;
    std::mutex frame_mutex;
    std::string frame="null";
public:
    std::atomic<unsigned> fft_size{4096},average{8};std::atomic<bool> remove_dc{true};
    std::atomic<uint64_t> skipped{0},frames{0};
    Processor():thread(&Processor::work,this) {}
    ~Processor(){closed=true;thread.join();}
    void push(const Header &h,const uint8_t *data) {
        size_t pos=head.load(std::memory_order_relaxed);
        if(pos-tail.load(std::memory_order_acquire)>=capacity){skipped++;return;}
        auto &slot=queue[pos%capacity];slot.header=h;memcpy(slot.iq.data(),data,h.samples*2);
        head.store(pos+1,std::memory_order_release);
    }
    std::string snapshot(){std::lock_guard<std::mutex> lock(frame_mutex);return frame;}
    void work() {
        std::array<std::complex<double>,16384> iq{};
        size_t written=0,available=0;uint64_t next=0;uint32_t session=0,rate=0,frequency=0;
        unsigned old_size=0,old_average=0;bool old_dc=false;
        std::vector<double> power;
        double last=seconds();
        while(!closed) {
            size_t pos=tail.load(std::memory_order_relaxed),end=head.load(std::memory_order_acquire);
            bool received=false;
            while(pos<end) {
                const auto &block=queue[pos%capacity];const auto &h=block.header;
                if(session!=h.session || next!=h.first_sample){available=written=0;power.clear();}
                session=h.session;next=h.first_sample+h.samples;rate=h.rate;frequency=h.frequency;
                for(unsigned j=0;j<h.samples;j++) {iq[written%iq.size()]={block.iq[j*2]/128.0,block.iq[j*2+1]/128.0};written++;}
                available=std::min(available+h.samples,iq.size());received=true;
                tail.store(++pos,std::memory_order_release);
            }
            unsigned n=fft_size.load(),avg=average.load();bool dc=remove_dc.load();
            if(received && available>=n && seconds()-last>=1.0/15) {
                std::vector<std::complex<double>> x(n);
                std::complex<double> mean=0;double rms=0,window_sum=0;unsigned clipped=0;
                for(unsigned j=0;j<n;j++){x[j]=iq[(written-n+j)%iq.size()];mean+=x[j];if(x[j].real()<=-1 || x[j].imag()<=-1 || x[j].real()>=127.0/128 || x[j].imag()>=127.0/128)clipped++;}
                mean/=n;
                for(unsigned j=0;j<n;j++){if(dc)x[j]-=mean;rms+=std::norm(x[j]);double w=.5-.5*std::cos(2*pi*j/(n-1));window_sum+=w;x[j]*=w;}
                fft(x);
                bool reset=power.size()!=n || old_size!=n || old_average!=avg || old_dc!=dc;
                if(reset)power.assign(n,0);
                old_size=n;old_average=avg;old_dc=dc;
                double peak=-160;unsigned peak_bin=0;std::array<double,1024> bins;bins.fill(-160);
                for(unsigned j=0;j<n;j++) {
                    double p=std::norm(x[(j+n/2)%n])/(window_sum*window_sum);
                    if(reset)power[j]=p;else power[j]+=(p-power[j])/avg;
                    double db=10*std::log10(std::max(power[j],1e-16));
                    if(db>peak){peak=db;peak_bin=j;}bins[j/(n/1024)]=std::max(bins[j/(n/1024)],db);
                }
                std::ostringstream out;out<<std::fixed<<std::setprecision(3);
                out<<"{\"rate\":"<<rate<<",\"frequency\":"<<frequency<<",\"fft_size\":"<<n
                   <<",\"first_sample\":"<<(next-n)<<",\"i_dc\":"<<mean.real()*128<<",\"q_dc\":"<<mean.imag()*128
                   <<",\"rms_dbfs\":"<<10*std::log10(std::max(rms/n,1e-16))<<",\"clipping\":"<<double(clipped)/n
                   <<",\"peak_dbfs\":"<<peak<<",\"peak_offset_hz\":"<<(double(peak_bin)-n/2)*rate/n<<",\"db\":[";
                for(unsigned j=0;j<1024;j++){if(j)out<<',';out<<bins[j];}out<<"]}";
                {std::lock_guard<std::mutex> lock(frame_mutex);frame=out.str();}frames++;last=seconds();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};
