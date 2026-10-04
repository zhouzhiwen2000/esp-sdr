// Native Windows/Linux S31Q receiver. No allocation, logging, FFT or file I/O on IQ receive path.
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <avrt.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <csignal>
#include <cerrno>
using SOCKET=int;
constexpr SOCKET INVALID_SOCKET=-1;
inline int closesocket(SOCKET s){return close(s);}
#endif
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#ifdef _WIN32
#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib,"avrt.lib")
using socket_length=int;
static int socket_error(){return WSAGetLastError();}
#else
using socket_length=socklen_t;
static int socket_error(){return errno;}
#define WSAETIMEDOUT EAGAIN
#define WSAECONNRESET ECONNRESET
#endif
static void socket_timeout(SOCKET s,int option,unsigned ms){
#ifdef _WIN32
    DWORD value=ms;
#else
    timeval value{static_cast<long>(ms/1000),static_cast<long>((ms%1000)*1000)};
#endif
    setsockopt(s,SOL_SOCKET,option,reinterpret_cast<const char*>(&value),sizeof(value));
}

using Clock=std::chrono::steady_clock;
static double seconds() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
static std::atomic<bool> exiting{false};
#ifdef _WIN32
static BOOL WINAPI console_handler(DWORD type) {
    if(type==CTRL_C_EVENT || type==CTRL_BREAK_EVENT || type==CTRL_CLOSE_EVENT) {exiting=true;return TRUE;}
    return FALSE;
}
#else
static void console_signal(int){exiting=true;}
#endif
static uint64_t json_u64(const std::string &json,const std::string &key) {
    size_t p=json.find('"'+key+'"');if(p==std::string::npos)return 0;
    p=json.find(':',p);if(p==std::string::npos)return 0;
    try{return std::stoull(json.substr(p+1));}catch(...){return 0;}
}
#pragma pack(push,1)
struct Header {
    char magic[4];uint8_t version,flags;uint16_t header_bytes;
    uint32_t session,sequence;uint64_t first_sample;uint32_t rate,frequency;
    uint16_t samples,format;uint64_t dma_drops;
};
#pragma pack(pop)
static_assert(sizeof(Header)==44,"wire header");
#include "dsp.h"
struct Options {
    std::string board="169.254.9.36",bind="169.254.9.35",report;
    unsigned rate=4000000,frequency=2412,gain=40;
    int core=-1,buffer_mb=64;double duration=180;
    bool realtime=true,mmcss=true,dsp=false,serve=false,tcp=false;
    unsigned port=8765;std::string page="host/s31_receiver/index.html";
};
struct alignas(64) Counts {
    std::atomic<uint64_t> packets{0},samples{0},missing_packets{0},missing_samples{0},bad_packets{0},late_packets{0},foreign_packets{0},dma_drops{0};
    std::atomic<uint64_t> max_receive_gap_us{0};
    std::atomic<double> last_packet{0};
};
class Receiver {
    SOCKET sock=INVALID_SOCKET;
    std::atomic<SOCKET> tcp_socket{INVALID_SOCKET};
    std::atomic<bool> tcp_eof{true};
    std::atomic<uint64_t> control_send_errors{0},socket_receive_errors{0};
    std::atomic<int> last_socket_error{0};
    sockaddr_in peer{};
    std::thread thread;
    std::mutex control_mutex,reply_mutex;
    std::condition_variable reply_ready;
    std::string waiting,reply,device="{}";
    std::atomic<uint32_t> session{0};uint32_t next_sequence=0;uint64_t next_sample=0;
    std::atomic<bool> closed{false},running{false},ready{false};
    std::atomic<double> started{0};
public:
    Options opt;Counts count;std::unique_ptr<Processor> processor;
    std::atomic<unsigned> current_rate{0},current_frequency{0},current_gain{0};
    int receive_buffer=0,chosen_core=-1,actual_priority=0;
    bool mmcss_enabled=false,affinity_set=false,priority_set=false;
    unsigned mmcss_error=0;
    bool memory_locked=false;
    std::atomic<uint64_t> kernel_drops{0};
    explicit Receiver(Options options):opt(options) {
        if(opt.dsp)processor=std::make_unique<Processor>();
        sock=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);if(sock==INVALID_SOCKET)throw std::runtime_error("socket failed");
        int size=opt.buffer_mb*1024*1024;
        if(setsockopt(sock,SOL_SOCKET,SO_RCVBUF,reinterpret_cast<char*>(&size),sizeof(size)))throw std::runtime_error("SO_RCVBUF failed");
        socket_length n=sizeof(receive_buffer);getsockopt(sock,SOL_SOCKET,SO_RCVBUF,reinterpret_cast<char*>(&receive_buffer),&n);
#ifndef _WIN32
        // Per-socket privilege override; no global sysctl is changed.
        setsockopt(sock,SOL_SOCKET,SO_RCVBUFFORCE,&size,sizeof(size));
        n=sizeof(receive_buffer);getsockopt(sock,SOL_SOCKET,SO_RCVBUF,&receive_buffer,&n);
        int overflow=1;setsockopt(sock,SOL_SOCKET,SO_RXQ_OVFL,&overflow,sizeof(overflow));
#endif
        socket_timeout(sock,SO_RCVTIMEO,100);
        sockaddr_in local{};local.sin_family=AF_INET;inet_pton(AF_INET,opt.bind.c_str(),&local.sin_addr);
        if(::bind(sock,reinterpret_cast<sockaddr*>(&local),sizeof(local)))throw std::runtime_error("bind failed: "+std::to_string(socket_error()));
        peer.sin_family=AF_INET;peer.sin_port=htons(9875);inet_pton(AF_INET,opt.board.c_str(),&peer.sin_addr);
        thread=std::thread(&Receiver::receive,this);
        while(!ready)std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ~Receiver() {closed=true;if(thread.joinable())thread.join();if(sock!=INVALID_SOCKET)closesocket(sock);SOCKET tcp=tcp_socket.exchange(INVALID_SOCKET);if(tcp!=INVALID_SOCKET)closesocket(tcp);}
    void send_text(const std::string &text) {
        std::string line=text+"\n";
        if(sendto(sock,line.data(),static_cast<int>(line.size()),0,reinterpret_cast<sockaddr*>(&peer),sizeof(peer))<0)throw std::runtime_error("sendto failed: "+std::to_string(socket_error()));
    }
    std::string command(const std::string &text,const std::string &prefix="OK") {
        std::lock_guard<std::mutex> serial(control_mutex);
        std::unique_lock<std::mutex> lock(reply_mutex);reply.clear();waiting=prefix;
        send_text(text);
        bool got=reply_ready.wait_for(lock,std::chrono::seconds(3),[&]{return !reply.empty();});
        waiting.clear();
        if(!got)throw std::runtime_error("timeout: "+text);
        if(reply.rfind("ERR",0)==0)throw std::runtime_error(reply);
        return reply;
    }
    void start(unsigned rate=0,unsigned frequency=0,unsigned gain=0) {
        if(running)stop();
        if(rate){opt.rate=rate;opt.frequency=frequency;opt.gain=gain;}
        command("NET?","NET ");
        command("FREQ "+std::to_string(opt.frequency));
        command("GAIN MANUAL "+std::to_string(opt.gain));
        session=std::random_device{}();if(!session)session=1;
        current_rate=opt.rate;current_frequency=opt.frequency;current_gain=opt.gain;
        count.packets=0;count.samples=0;count.missing_packets=0;count.missing_samples=0;
        count.bad_packets=0;count.late_packets=0;count.foreign_packets=0;count.dma_drops=0;
        count.last_packet=0;count.max_receive_gap_us=0;
        if(opt.tcp) {
            SOCKET connection=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
            if(connection==INVALID_SOCKET)throw std::runtime_error("TCP socket failed");
            int size=opt.buffer_mb*1024*1024;
            setsockopt(connection,SOL_SOCKET,SO_RCVBUF,reinterpret_cast<char*>(&size),sizeof(size));
            sockaddr_in remote=peer;remote.sin_port=htons(9876);
            if(connect(connection,reinterpret_cast<sockaddr*>(&remote),sizeof(remote))) {
                closesocket(connection);throw std::runtime_error("TCP connection failed");
            }
            tcp_eof=false;tcp_socket=connection;
        }
        started=seconds();running=true;
        try {command(std::string(opt.tcp?"NETSTARTTCP ":"NETSTART ")+std::to_string(opt.rate)+" "+std::to_string(session));}
        catch(...) {running=false;throw;}
    }
    void heartbeat() {
        if(!running)return;
        try {send_text("NETPING "+std::to_string(session));send_text("NET?");}
        catch(const std::exception &) {last_socket_error=socket_error();control_send_errors++;}
    }
    void stop() {
        if(running) {
            command("NETSTOP");
            if(opt.tcp) {double deadline=seconds()+3;while(!tcp_eof && seconds()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(1));}
            else std::this_thread::sleep_for(std::chrono::milliseconds(150));
            running=false;command("NET?","NET ");
        }
    }
    std::string snapshot(bool include_spectrum=false) {
        std::string dev;{std::lock_guard<std::mutex> lock(reply_mutex);dev=device;}
        double age=count.last_packet?seconds()-count.last_packet.load():-1;
        double elapsed=count.last_packet?count.last_packet.load()-started:0;
        uint64_t final_missing=!running && json_u64(dev,"session")==session && json_u64(dev,"packets")>count.packets?json_u64(dev,"packets")-count.packets:0;
        bool stalled=running && ((age<0 && seconds()-started.load()>2) || age>2);
        bool incomplete_tcp=opt.tcp && !running && !tcp_eof;
        bool final_bytes_mismatch=!running && json_u64(dev,"session")==session && json_u64(dev,"bytes")!=count.samples*2;
        bool good=!incomplete_tcp && !final_bytes_mismatch && !kernel_drops && !stalled && !final_missing && !json_u64(dev,"parlio_overflow") && !json_u64(dev,"dma_drops") && !json_u64(dev,"buffer_drops") && !json_u64(dev,"tx_errors") && !json_u64(dev,"error") && count.packets && !count.missing_packets && !count.missing_samples && !count.bad_packets && !count.late_packets && !count.dma_drops;
        std::ostringstream out;out<<std::fixed<<std::setprecision(3);
        out<<"{\"protocol_version\":2,\"payload_checksum\":\"none\",\"session\":"<<session<<",\"packets\":"<<count.packets<<",\"samples\":"<<count.samples
           <<",\"missing_packets\":"<<count.missing_packets<<",\"missing_samples\":"<<count.missing_samples
           <<",\"bad_packets\":"<<count.bad_packets<<",\"late_packets\":"<<count.late_packets
           <<",\"foreign_packets\":"<<count.foreign_packets<<",\"dma_drops\":"<<count.dma_drops
           <<",\"elapsed\":"<<elapsed<<",\"mbps\":"<<(elapsed>0?count.samples*16.0/elapsed/1e6:0)
           <<",\"packet_age\":"<<age<<",\"running\":"<<(running?"true":"false")
           <<",\"transport_continuous\":"<<(good?"true":"false")<<",\"socket_buffer\":"<<receive_buffer
           <<",\"control_send_errors\":"<<control_send_errors<<",\"socket_receive_errors\":"<<socket_receive_errors<<",\"last_socket_error\":"<<last_socket_error
           <<",\"kernel_drops\":"<<kernel_drops<<",\"memory_locked\":"<<(memory_locked?"true":"false")
           <<",\"rx_core\":"<<chosen_core<<",\"affinity_set\":"<<(affinity_set?"true":"false")
           <<",\"priority_set\":"<<(priority_set?"true":"false")<<",\"thread_priority\":"<<actual_priority
           <<",\"mmcss\":"<<(mmcss_enabled?"true":"false")<<",\"mmcss_error\":"<<mmcss_error
           <<",\"max_receive_gap_us\":"<<count.max_receive_gap_us<<",\"device\":"<<dev
           <<",\"record_supported\":false,\"recording\":null,\"last_recording\":null,\"error\":\""<<(stalled?"IQ stream stalled":"")<<"\""
           <<",\"final_missing_packets\":"<<final_missing
           <<",\"settings\":{\"rate\":"<<current_rate<<",\"frequency\":"<<current_frequency<<",\"gain\":"<<current_gain<<"}"
           <<",\"transport\":\""<<(opt.tcp?"tcp":"udp")<<"\""
           <<",\"frame_number\":"<<(processor?processor->frames.load():0)
           <<",\"display_skipped_packets\":"<<(processor?processor->skipped.load():0);
        if(include_spectrum)out<<",\"spectrum\":"<<(processor?processor->snapshot():"null");
        out<<"}";
        return out.str();
    }
    void consume_frame(const uint8_t *data,int length,double &previous,uint32_t &last_session) {
            if(length<static_cast<int>(sizeof(Header))) {count.bad_packets++;return;}
            Header h;memcpy(&h,data,sizeof(h));
            if(h.version!=2 || h.flags!=0 || h.header_bytes!=sizeof(Header) || h.format!=1 || !h.samples || h.samples>(opt.tcp?2016:672) || length!=static_cast<int>(sizeof(Header)+h.samples*2)) {count.bad_packets++;return;}
            if(h.session!=session){count.foreign_packets++;return;}
            if(last_session!=h.session){next_sequence=0;next_sample=0;previous=0;last_session=h.session;}
            uint32_t delta=h.sequence-next_sequence;
            if(delta>=0x80000000u || h.first_sample<next_sample){count.late_packets++;return;}
            count.missing_packets+=delta;count.missing_samples+=h.first_sample-next_sample;
            next_sequence=h.sequence+1;next_sample=h.first_sample+h.samples;
            count.dma_drops=h.dma_drops;count.samples+=h.samples;count.packets++;
            if(processor)processor->push(h,data+sizeof(Header));
            double now=seconds();count.last_packet=now;
            if(previous) {uint64_t gap=static_cast<uint64_t>((now-previous)*1e6);if(gap>count.max_receive_gap_us)count.max_receive_gap_us=gap;}
            previous=now;
    }
    void receive() {
        #ifdef _WIN32
        HANDLE mmcss_handle=nullptr;
        if(opt.realtime) {
            DWORD_PTR available=0,system=0;GetProcessAffinityMask(GetCurrentProcess(),&available,&system);
            chosen_core=opt.core;
            if(chosen_core<0)for(int i=0;i<static_cast<int>(sizeof(DWORD_PTR)*8);i++)if(available&(DWORD_PTR(1)<<i))chosen_core=i;
            if(chosen_core>=0 && chosen_core<static_cast<int>(sizeof(DWORD_PTR)*8))affinity_set=SetThreadAffinityMask(GetCurrentThread(),DWORD_PTR(1)<<chosen_core)!=0;
            priority_set=SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_HIGHEST)!=0;
            if(opt.mmcss) {
                DWORD index=0;mmcss_handle=AvSetMmThreadCharacteristicsW(L"Pro Audio",&index);
                if(mmcss_handle)mmcss_enabled=AvSetMmThreadPriority(mmcss_handle,AVRT_PRIORITY_CRITICAL)!=0;
                else mmcss_error=GetLastError();
            }
        }
        actual_priority=GetThreadPriority(GetCurrentThread());
#else
        if(opt.realtime) {
            cpu_set_t available;CPU_ZERO(&available);
            if(!pthread_getaffinity_np(pthread_self(),sizeof(available),&available)) {
                chosen_core=opt.core;
                if(chosen_core<0)for(int i=0;i<CPU_SETSIZE;i++)if(CPU_ISSET(i,&available))chosen_core=i;
                if(chosen_core>=0 && chosen_core<CPU_SETSIZE && CPU_ISSET(chosen_core,&available)) {
                    cpu_set_t chosen;CPU_ZERO(&chosen);CPU_SET(chosen_core,&chosen);
                    affinity_set=!pthread_setaffinity_np(pthread_self(),sizeof(chosen),&chosen);
                }
            }
            sched_param priority{};priority.sched_priority=20;
            priority_set=!pthread_setschedparam(pthread_self(),SCHED_FIFO,&priority);
        }
        int policy=0;sched_param priority{};pthread_getschedparam(pthread_self(),&policy,&priority);
        actual_priority=priority.sched_priority;
#endif
        ready=true;
        uint8_t data[4096],stream[65536];size_t have=0;double previous=0;uint32_t last_session=0;
        SOCKET previous_socket=INVALID_SOCKET;
        while(!closed) {
            SOCKET tcp=tcp_socket.load();
            if(tcp!=previous_socket){have=0;previous_socket=tcp;}
            fd_set ready_set;FD_ZERO(&ready_set);FD_SET(sock,&ready_set);if(tcp!=INVALID_SOCKET)FD_SET(tcp,&ready_set);
            timeval timeout{0,100000};
            if(select(std::max(sock,tcp)+1,&ready_set,nullptr,nullptr,&timeout)<=0)continue;
            if(tcp!=INVALID_SOCKET && FD_ISSET(tcp,&ready_set)) {
                int received=recv(tcp,reinterpret_cast<char*>(stream+have),static_cast<int>(sizeof(stream)-have),0);
                bool failed=received<=0;
                if(received>0) {
                    have+=received;size_t offset=0;
                    while(have-offset>=sizeof(Header)) {
                        Header h;memcpy(&h,stream+offset,sizeof(h));
                        size_t size=sizeof(Header)+size_t(h.samples)*2;
                        if(memcmp(h.magic,"S31Q",4) || h.header_bytes!=sizeof(Header) || !h.samples || h.samples>2016) {count.bad_packets++;failed=true;break;}
                        if(have-offset<size)break;
                        consume_frame(stream+offset,static_cast<int>(size),previous,last_session);offset+=size;
                    }
                    if(offset){have-=offset;memmove(stream,stream+offset,have);}
                }
                if(failed) {
                    if(have)count.bad_packets++;
                    closesocket(tcp);tcp_socket=INVALID_SOCKET;tcp_eof=true;have=0;
                }
            }
            if(!FD_ISSET(sock,&ready_set))continue;
            sockaddr_in from{};socket_length from_size=sizeof(from);
#ifdef _WIN32
            int length=recvfrom(sock,reinterpret_cast<char*>(data),sizeof(data),0,reinterpret_cast<sockaddr*>(&from),&from_size);
#else
            (void)from_size;
            iovec iov{data,sizeof(data)};alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(uint32_t))]{};
            msghdr message{};message.msg_name=&from;message.msg_namelen=sizeof(from);
            message.msg_iov=&iov;message.msg_iovlen=1;message.msg_control=ancillary;message.msg_controllen=sizeof(ancillary);
            int length=static_cast<int>(recvmsg(sock,&message,0));
            if(length>=0)for(cmsghdr *c=CMSG_FIRSTHDR(&message);c;c=CMSG_NXTHDR(&message,c)) {
                if(c->cmsg_level==SOL_SOCKET && c->cmsg_type==SO_RXQ_OVFL && c->cmsg_len>=CMSG_LEN(sizeof(uint32_t))) {
                    uint32_t drops;memcpy(&drops,CMSG_DATA(c),sizeof(drops));kernel_drops=drops;
                }
            }
            if(message.msg_flags&MSG_TRUNC){length=-1;errno=EMSGSIZE;}
#endif
            if(length<0) {int e=socket_error();if(e==WSAETIMEDOUT || e==WSAECONNRESET)continue;
                last_socket_error=e;socket_receive_errors++;std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;}
            if(from.sin_addr.s_addr!=peer.sin_addr.s_addr || from.sin_port!=peer.sin_port)continue;
            if(length<4 || memcmp(data,"S31Q",4)) {
                std::string text(reinterpret_cast<char*>(data),length);
                while(!text.empty() && (text.back()=='\r'||text.back()=='\n'))text.pop_back();
                std::lock_guard<std::mutex> lock(reply_mutex);
                if(text.rfind("NET ",0)==0)device=text.substr(4);
                if(!waiting.empty() && (text.rfind(waiting,0)==0 || text.rfind("ERR",0)==0)) {reply=text;reply_ready.notify_one();}
                continue;
            }
            consume_frame(data,length,previous,last_session);

        }
#ifdef _WIN32
        if(mmcss_handle)AvRevertMmThreadCharacteristics(mmcss_handle);
#endif
    }
};
#include "http.h"
int main(int argc,char **argv) {
    try {
        Options opt;
        for(int j=1;j<argc;j++) {
            std::string flag=argv[j];
            if(flag=="--help") {
                std::cout<<"S31 native receiver (Windows / Linux)\n"
                    "  --serve                   Local spectrum UI on http://localhost:8765\n"
                    "  --seconds N               Headless capture (default 180 seconds)\n"
                    "  --rate HZ                 250000, 1000000, 2000000, 4000000, 8000000, 16000000, 20000000, 32000000, 40000000, 53333333\n"
                    "  --frequency MHZ --gain N  Receiver settings (default 2412 / 40)\n"
                    "  --board IP --bind IP      Board / local Ethernet address\n"
                    "  --dsp                     Enable FFT during headless capture\n"
                    "  --tcp                     Use buffered TCP stream on port 9876\n"
                    "  --core N                  Pin receive thread to logical CPU (default last available)\n"
                    "  --buffer-mb N             Socket receive buffer (default 64 MiB)\n"
                    "  --normal --no-mmcss       Scheduling comparison options\n"
                    "  --report FILE             Final JSON report\n"
                    "  --page FILE --port N      Viewer HTML path / local HTTP port\n";
                return 0;
            }
            if(flag=="--tcp"){opt.tcp=true;continue;}
            if(flag=="--serve"){opt.serve=true;opt.dsp=true;continue;}
            if(flag=="--dsp"){opt.dsp=true;continue;}
            if(flag=="--normal"){opt.realtime=false;continue;}
            if(flag=="--no-mmcss"){opt.mmcss=false;continue;}
            if(j+1>=argc)throw std::runtime_error("option needs a value: "+flag);
            std::string value=argv[++j];
            if(flag=="--board")opt.board=value;else if(flag=="--bind")opt.bind=value;
            else if(flag=="--rate")opt.rate=std::stoul(value);else if(flag=="--frequency")opt.frequency=std::stoul(value);
            else if(flag=="--gain")opt.gain=std::stoul(value);else if(flag=="--seconds")opt.duration=std::stod(value);
            else if(flag=="--core")opt.core=std::stoi(value);else if(flag=="--buffer-mb")opt.buffer_mb=std::stoi(value);
            else if(flag=="--port")opt.port=std::stoul(value);else if(flag=="--page")opt.page=value;
            else if(flag=="--report")opt.report=value;else throw std::runtime_error("unknown option: "+flag);
        }
        if(opt.buffer_mb<1 || opt.buffer_mb>256 || opt.core< -1 || opt.core>=64 || opt.duration<=0 || opt.port<1 || opt.port>65535)
            throw std::runtime_error("invalid buffer, CPU, duration or HTTP port");
#ifdef _WIN32
        WSADATA ws{};if(WSAStartup(MAKEWORD(2,2),&ws))throw std::runtime_error("WSAStartup failed");
        SetConsoleCtrlHandler(console_handler,TRUE);
        if(opt.realtime)SetPriorityClass(GetCurrentProcess(),ABOVE_NORMAL_PRIORITY_CLASS);
#else
        std::signal(SIGINT,console_signal);std::signal(SIGTERM,console_signal);std::signal(SIGPIPE,SIG_IGN);
#endif
        Receiver receiver(opt);
#ifndef _WIN32
        if(opt.realtime)receiver.memory_locked=mlockall(MCL_CURRENT)==0;
#endif
        receiver.start();
        std::unique_ptr<HttpServer> web;
        if(opt.serve){web=std::make_unique<HttpServer>(receiver,opt.port,opt.page);std::cout<<"Spectrum viewer: http://localhost:"<<opt.port<<std::endl;}
        double start=seconds(),last=start;
        while(!exiting && (opt.serve || seconds()-start<opt.duration)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if(seconds()-last>=1){receiver.heartbeat();std::cout<<receiver.snapshot()<<std::endl;last=seconds();}
        }
        web.reset();receiver.stop();std::string report=receiver.snapshot();std::cout<<report<<std::endl;
        if(!opt.report.empty()){std::ofstream file(opt.report);file<<report<<"\n";}
        return report.find("\"transport_continuous\":true")!=std::string::npos?0:2;
    }catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}
}
