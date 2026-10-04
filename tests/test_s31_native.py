"""End-to-end Windows/Linux native receive/FFT/UI test against a local synthetic board.
Windows example: python tests/test_s31_native.py --exe artifacts/native/s31_rx.exe
"""
import argparse
import cmath
import json
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time
import urllib.request


def main():
    p=argparse.ArgumentParser();p.add_argument('--exe',required=True,type=Path);p.add_argument('--tcp',action='store_true');p.add_argument('--port',type=int,default=8766);a=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',9875));sock.settimeout(.02)
    listener=None;connection=None
    if a.tcp:
        listener=socket.socket(socket.AF_INET,socket.SOCK_STREAM);listener.bind(('127.0.0.1',9876));listener.listen(1);listener.settimeout(.001)
    done=threading.Event();state={'active':False,'rate':1000000,'session':0,'packets':0,'bytes':0,'dma_drops':0,'tx_errors':0,'error':0,'mbps':1000}
    peer=None;sequence=first=0;inject=False
    cycle=bytes(v%256 for i in range(4096) for v in (round(64*cmath.exp(2j*cmath.pi*137*i/4096).real)+20,round(64*cmath.exp(2j*cmath.pi*137*i/4096).imag)-10))
    def board():
        nonlocal peer,sequence,first,inject,connection
        last=0
        while not done.is_set():
            if listener and connection is None:
                try:connection,_=listener.accept()
                except socket.timeout:pass
            try:
                raw,address=sock.recvfrom(1024);cmd=raw.decode().strip()
                if cmd=='NET?':reply='NET '+json.dumps(state)
                elif cmd.startswith(('NETSTART ','NETSTARTTCP ')):
                    _,rate,session=cmd.split();state.update(active=True,rate=int(rate),session=int(session),packets=0,bytes=0);sequence=first=0;peer=address;reply='OK'
                elif cmd=='NETSTOP':
                    state['active']=False;reply='OK'
                    if connection:
                        connection.shutdown(socket.SHUT_WR);connection.close();connection=None
                elif cmd.startswith('NETPING '):reply='PONG '+cmd.split()[1]
                else:reply='OK'
                sock.sendto((reply+'\n').encode(),address)
            except socket.timeout:pass
            if state['active'] and time.monotonic()-last>.001:
                if inject:sequence+=1;first+=672;state['packets']+=1;state['bytes']+=1344;inject=False
                payload=(cycle*2)[(first%4096)*2:(first%4096)*2+1344]
                header=struct.pack('<4sBBHIIQIIHHQ',b'S31Q',2,0,44,state['session'],sequence,first,state['rate'],2412,672,1,0)
                frame=header+payload
                if a.tcp:
                    for part in (frame[:13],frame[13:57],frame[57:]):connection.sendall(part)
                else:sock.sendto(frame,peer)
                sequence+=1;first+=672;state['packets']+=1;state['bytes']+=1344;last=time.monotonic()
    worker=threading.Thread(target=board);worker.start()
    def get():return json.load(urllib.request.urlopen(f'http://localhost:{a.port}/api/state',timeout=2))
    def post(path,body):
        req=urllib.request.Request(f'http://localhost:{a.port}/api/'+path,data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
        return json.load(urllib.request.urlopen(req,timeout=5))
    try:
        with tempfile.TemporaryDirectory() as temp:
            with (Path(temp)/'process.log').open('w') as log:
                proc=subprocess.Popen([str(a.exe.resolve()),'--board','127.0.0.1','--bind','127.0.0.1','--rate','1000000','--serve','--port',str(a.port),'--page',str(root/'host/s31_receiver/index.html')]+(['--tcp'] if a.tcp else []),stdout=log,stderr=log)
                try:
                    limit=time.monotonic()+10
                    while True:
                        try:
                            result=get()
                            if result.get('spectrum'):break
                        except OSError:pass
                        if time.monotonic()>limit:raise TimeoutError('native synthetic FFT did not start')
                        time.sleep(.1)
                    f=result['spectrum']
                    assert abs(f['peak_offset_hz']-137*1_000_000/4096)<1, f
                    assert abs(f['peak_dbfs']+6.0206)<.06, f['peak_dbfs']
                    assert abs(f['i_dc']-20)<.1 and abs(f['q_dc']+10)<.1
                    assert result['transport_continuous']
                    # A capture FIFO overflow invalidates continuity even if
                    # every UDP packet and software sample index arrived.
                    state['parlio_overflow']=1
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline:
                        result=get()
                        if result['device'].get('parlio_overflow'):break
                        time.sleep(.02)
                    assert not result['transport_continuous'] and result['device']['parlio_overflow']==1,result
                    assert result['missing_samples']==0,result
                    state['parlio_overflow']=0
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline:
                        result=get()
                        if result['transport_continuous']:break
                        time.sleep(.02)
                    assert result['transport_continuous'],result
                    # Browser preconnects may sit idle. Close them silently instead
                    # of sending a stale HTTP 400 that a later request could read.
                    with socket.create_connection(('127.0.0.1',a.port),timeout=3) as idle:
                        idle.settimeout(3)
                        assert idle.recv(256)==b'', 'idle HTTP connection received an unsolicited response'
                    assert get()['transport_continuous']
                    # WSAEMSGSIZE on the UDP control socket must not kill the
                    # shared UDP/TCP receive thread.
                    before=result['packets']
                    sock.sendto(b'X'*5000,peer)
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline:
                        result=get()
                        if result.get('socket_receive_errors') and result['packets']>before:break
                        time.sleep(.02)
                    assert result['socket_receive_errors']==1 and result['packets']>before, result
                    assert result['transport_continuous'], result
                    inject=True;time.sleep(.1)
                    result=get();assert result['missing_packets']==1 and result['missing_samples']==672,result
                    assert not result['transport_continuous']
                    result=post('stop',{});assert result['final_missing_packets']==1,result
                    detected_missing=result['missing_samples']
                    result=post('start',{'rate':53333333,'frequency':2412,'gain':40})
                    assert result['settings']['rate']==53333333,result
                    time.sleep(.15)
                    result=post('stop',{});assert result['transport_continuous'],result
                    print(json.dumps({'passed':True,'tcp':a.tcp,'tone_dbfs':f['peak_dbfs'],'tone_offset_hz':f['peak_offset_hz'],'detected_missing_samples':detected_missing,'high_rate_http':True}))
                except Exception:
                    log.flush();print((Path(temp)/'process.log').read_text(),flush=True);raise
                finally:
                    proc.terminate();proc.wait(timeout=5)
    finally:
        done.set();worker.join();sock.close()
        if connection:connection.close()
        if listener:listener.close()


if __name__=='__main__':main()
