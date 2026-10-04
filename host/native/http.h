#pragma once
#include <iterator>
static unsigned json_number(const std::string &body,const std::string &key) {
    size_t pos=body.find('"'+key+'"');if(pos==std::string::npos)throw std::runtime_error("missing "+key);
    pos=body.find(':',pos);if(pos==std::string::npos)throw std::runtime_error("invalid JSON");
    size_t consumed=0;unsigned long value=std::stoul(body.substr(pos+1),&consumed);
    if(value>100000000)throw std::runtime_error("value too large");
    return static_cast<unsigned>(value);
}
static std::string json_escape(const std::string &s) {
    std::string result;for(char c:s){if(c=='"'||c=='\\')result+='\\';if(c=='\n')result+="\\n";else if(c!='\r')result+=c;}return result;
}
class HttpServer {
    Receiver &receiver;SOCKET server=INVALID_SOCKET;std::thread thread;
    std::atomic<bool> closed{false};std::string page;
public:
    HttpServer(Receiver &rx,unsigned port,const std::string &path):receiver(rx) {
        std::ifstream input(path,std::ios::binary);if(!input)throw std::runtime_error("cannot read page: "+path);
        page.assign(std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>());
        server=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
#ifndef _WIN32
        int reuse=1;setsockopt(server,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(reuse));
#endif
        sockaddr_in local{};local.sin_family=AF_INET;local.sin_addr.s_addr=htonl(INADDR_LOOPBACK);local.sin_port=htons(static_cast<u_short>(port));
        if(bind(server,reinterpret_cast<sockaddr*>(&local),sizeof(local)) || listen(server,8))throw std::runtime_error("HTTP listen failed");
        thread=std::thread(&HttpServer::work,this);
    }
    ~HttpServer(){closed=true;if(thread.joinable())thread.join();closesocket(server);}
    void respond(SOCKET socket,int status,const std::string &body,const std::string &type="application/json") {
        std::string out="HTTP/1.1 "+std::to_string(status)+(status==200?" OK\r\n":" Error\r\n")+
            "Content-Type: "+type+"; charset=utf-8\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: "+std::to_string(body.size())+"\r\n\r\n"+body;
        size_t sent=0;while(sent<out.size()){int n=send(socket,out.data()+sent,static_cast<int>(out.size()-sent),0);if(n<=0)break;sent+=n;}
    }
    void work() {
        while(!closed) {
            fd_set set;FD_ZERO(&set);FD_SET(server,&set);timeval timeout{0,100000};
            if(select(server+1,&set,nullptr,nullptr,&timeout)<=0)continue;
            SOCKET client=accept(server,nullptr,nullptr);if(client==INVALID_SOCKET)continue;
            socket_timeout(client,SO_RCVTIMEO,1000);
            socket_timeout(client,SO_SNDTIMEO,1000);
            bool received_any=false;
            try {
                std::string request;char buffer[4096];size_t split=std::string::npos,total=0;
                for(;;) {
                    int n=recv(client,buffer,sizeof(buffer),0);if(n<=0)throw std::runtime_error("incomplete HTTP request");received_any=true;request.append(buffer,n);
                    if(request.size()>8192)throw std::runtime_error("request too large");
                    split=request.find("\r\n\r\n");if(split==std::string::npos)continue;
                    std::string headers=request.substr(0,split);std::string lower=headers;
                    std::transform(lower.begin(),lower.end(),lower.begin(),[](unsigned char c){return char(std::tolower(c));});
                    size_t position=lower.find("content-length:");
                    total=position==std::string::npos?0:std::stoul(headers.substr(position+15));
                    if(total>4096)throw std::runtime_error("body too large");
                    if(request.size()>=split+4+total)break;
                }
                std::istringstream first(request);std::string method,path;first>>method>>path;
                std::string body=request.substr(split+4,total);
                if(method=="GET" && path=="/")respond(client,200,page,"text/html");
                else if(method=="GET" && path=="/api/state")respond(client,200,receiver.snapshot(true));
                else if(method=="POST") {
                    std::string headers=request.substr(0,split);std::string lower=headers;
                    std::transform(lower.begin(),lower.end(),lower.begin(),[](unsigned char c){return char(std::tolower(c));});
                    // A JSON content type makes cross-origin browser form submissions fail.
                    if(lower.find("content-type: application/json")==std::string::npos)throw std::runtime_error("JSON content type required");
                    size_t origin=lower.find("\r\norigin: ");
                    if(origin!=std::string::npos) {
                        std::string value=lower.substr(origin+10);value=value.substr(0,value.find("\r\n"));
                        size_t host=lower.find("\r\nhost: ");if(host==std::string::npos)throw std::runtime_error("host required");
                        std::string name=lower.substr(host+8);name=name.substr(0,name.find("\r\n"));
                        if(value!="http://"+name)throw std::runtime_error("origin mismatch");
                    }
                    if(path=="/api/start") {
                        unsigned rate=json_number(body,"rate"),frequency=json_number(body,"frequency"),gain=json_number(body,"gain");
                        if((rate!=250000 && rate!=1000000 && rate!=2000000 && rate!=4000000 && rate!=8000000 && rate!=16000000 && rate!=20000000 && rate!=32000000 && rate!=40000000 && rate!=53333333) || frequency>6000 || frequency<100 || gain>127)throw std::runtime_error("invalid receiver settings");
                        receiver.start(rate,frequency,gain);
                    } else if(path=="/api/stop")receiver.stop();
                    else if(path=="/api/dsp") {
                        unsigned n=json_number(body,"fft_size"),avg=json_number(body,"average");
                        if(n<1024 || n>16384 || (n&(n-1)) || avg<1 || avg>100)throw std::runtime_error("invalid FFT settings");
                        if(receiver.processor){receiver.processor->fft_size=n;receiver.processor->average=avg;
                            size_t dc=body.find("\"remove_dc\"");if(dc==std::string::npos)throw std::runtime_error("missing remove_dc");
                            size_t colon=body.find(':',dc),v=body.find_first_not_of(" \t",colon+1);
                            receiver.processor->remove_dc=body.compare(v,4,"true")==0;}
                    } else throw std::runtime_error("unknown endpoint");
                    respond(client,200,receiver.snapshot());
                } else respond(client,404,"{\"error\":\"not found\"}");
            }catch(const std::exception &e){if(received_any)respond(client,400,"{\"error\":\""+json_escape(e.what())+"\"}");}
            closesocket(client);
        }
    }
};
