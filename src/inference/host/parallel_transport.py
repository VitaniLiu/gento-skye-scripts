"""Byte-transparent fixed64 SSH transport; no model or robot semantics.
64路持久SSH分片传输；仅重组原始请求，不变更位姿、图像、掩码或执行逻辑。
"""
from concurrent.futures import ThreadPoolExecutor
import argparse
import hashlib
import json
import logging
import math
import os
import select
import socket
import socketserver
import struct
import subprocess
import threading
import time
import uuid
import zlib

MAGIC=b'MP64'
GREETING=b'MP64READY\n'
HEADER=struct.Struct('!4s16sQHHHIII32s')
MAX_BYTES=16*1024*1024
MAX_PACKED=MAX_BYTES+65536


def recv_exact(sock,n):
    """Read a complete TCP field / 完整读取TCP字段。"""
    out=bytearray()
    while len(out)<n:
        part=sock.recv(n-len(out))
        if not part:raise ConnectionError('transport EOF')
        out.extend(part)
    return bytes(out)


def inflate(packed,expected):
    """Bound decompression before forwarding / 限制解压大小后再转发。"""
    if not 0<expected<=MAX_BYTES:raise ValueError('raw size exceeds transport limit')
    decoder=zlib.decompressobj();raw=decoder.decompress(packed,expected+1)
    if len(raw)!=expected or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError('compressed payload size/integrity mismatch')
    return raw


class SocketStream:
    def __init__(self,sock):self.sock=sock
    def read(self,n,deadline):
        self.sock.settimeout(max(.01,deadline-time.monotonic()));return recv_exact(self.sock,n)
    def write(self,data,deadline):
        self.sock.settimeout(max(.01,deadline-time.monotonic()));self.sock.sendall(data)
    def close(self):
        try:self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:pass
        self.sock.close()


class SSHStream:
    """One independent persistent SSH TCP flow / 一条独立、持久的SSH TCP连接。"""
    def __init__(self,host,gateway_port,timeout):
        self.process=subprocess.Popen(['ssh','-T','-o','BatchMode=yes','-o','ExitOnForwardFailure=yes',
            '-o','ControlMaster=no','-o','ControlPath=none','-o','ServerAliveInterval=30',
            '-o','ServerAliveCountMax=3','-W',f'127.0.0.1:{gateway_port}',host],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,bufsize=0)
        for stream in (self.process.stdin,self.process.stdout):os.set_blocking(stream.fileno(),False)
        try:
            if self.read(len(GREETING),time.monotonic()+timeout)!=GREETING:raise ValueError('wrong gateway greeting')
        except Exception:
            self.close();raise
    def read(self,n,deadline):
        out=bytearray();fd=self.process.stdout.fileno()
        while len(out)<n:
            remaining=deadline-time.monotonic()
            if remaining<=0 or not select.select([fd],[],[],remaining)[0]:raise TimeoutError('SSH lane read timeout')
            try:part=os.read(fd,min(65536,n-len(out)))
            except BlockingIOError:continue
            if not part:raise ConnectionError('SSH lane closed')
            out.extend(part)
        return bytes(out)
    def write(self,data,deadline):
        value=memoryview(data);fd=self.process.stdin.fileno()
        while value:
            remaining=deadline-time.monotonic()
            if remaining<=0 or not select.select([],[fd],[],remaining)[1]:raise TimeoutError('SSH lane write timeout')
            try:n=os.write(fd,value[:65536])
            except BlockingIOError:continue
            value=value[n:]
    def close(self):
        if self.process.poll() is None:self.process.terminate()
        try:self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=2)
        for stream in (self.process.stdin,self.process.stdout,self.process.stderr):stream.close()


class ParallelPool:
    """One ordered model session over persistent lanes; never retries inference.
    多路上传只对应一次模型调用；保留持久会话，不自动重试推理。
    """
    def __init__(self,streams,backend_port,timeout=300):
        if not 1<=len(streams)<=64:raise ValueError('lane count must be1..64')
        self.streams=streams;self.port=backend_port;self.timeout=timeout
        self.sid=uuid.uuid4().bytes;self.sequence=0;self.lanes=len(streams);self.closed=False
        self.executor=ThreadPoolExecutor(max_workers=self.lanes);self.lock=threading.Lock()
        self.stats=dict(lanes=self.lanes,requests=0,last_shards=0)
    def rpc(self,raw):
        with self.lock:
            if self.closed:raise ConnectionError('parallel pool closed')
            if not 0<len(raw)<=MAX_BYTES:raise ValueError('invalid model message size')
            packed=zlib.compress(raw,1);digest=hashlib.sha256(packed).digest()
            count=self.lanes if len(raw)>=4096 else 1
            step=math.ceil(len(packed)/count);deadline=time.monotonic()+self.timeout
            def lane(index):
                part=packed[index*step:(index+1)*step]
                header=HEADER.pack(MAGIC,self.sid,self.sequence,self.port,index,count,len(packed),len(raw),len(part),digest)
                stream=self.streams[index];stream.write(header+part,deadline)
                flag=stream.read(1,deadline)
                if flag==b'E':
                    size=int.from_bytes(stream.read(4,deadline),'big')
                    if size>8192:raise ValueError('invalid gateway error size')
                    raise RuntimeError(stream.read(size,deadline).decode(errors='replace'))
                if index:
                    if flag!=b'A':raise ValueError('invalid shard acknowledgement')
                    return None
                if flag!=b'R':raise ValueError('invalid gateway response')
                size,raw_size=struct.unpack('!II',stream.read(8,deadline))
                if not 0<size<=MAX_PACKED:raise ValueError('invalid response size')
                sha=stream.read(32,deadline);body=stream.read(size,deadline)
                if hashlib.sha256(body).digest()!=sha:raise ValueError('response digest mismatch')
                return inflate(body,raw_size)
            futures=[self.executor.submit(lane,i) for i in range(count)]
            try:results=[f.result() for f in futures]
            except Exception:
                self.close();raise
            self.sequence+=1;self.stats.update(requests=self.sequence,last_shards=count,raw_bytes=len(raw),packed_bytes=len(packed))
            return results[0]
    def close(self):
        if self.closed:return
        self.closed=True
        # Terminate all SSH flows before waiting, so one slow lane cannot retain others.
        for stream in self.streams:
            if isinstance(stream,SSHStream) and stream.process.poll() is None:stream.process.terminate()
        for stream in self.streams:
            try:stream.close()
            except OSError:pass
        self.executor.shutdown(wait=False,cancel_futures=True)


class Assembly:
    def __init__(self,port,count,total,raw,digest):
        self.metadata=(port,count,total,raw,digest);self.parts=[None]*count;self.event=threading.Event();self.reply=None


class Session:
    def __init__(self,port):
        self.port=port;self.sequence=0;self.current=None;self.backend=None;self.connections=0;self.failed=False


class Gateway(socketserver.ThreadingTCPServer):
    """Localhost CPU gateway; backend endpoints are allowlisted / 本机CPU网关，后端限定白名单。"""
    allow_reuse_address=True;daemon_threads=True;request_queue_size=128
    def __init__(self,address,backend_host,ports,timeout=300):
        self.backend_host=backend_host;self.ports=set(ports);self.timeout=timeout
        self.sessions={};self.lock=threading.RLock()
        gateway=self
        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                sid=None;self.request.settimeout(timeout);self.request.sendall(GREETING)
                try:
                    while True:
                        header=HEADER.unpack(recv_exact(self.request,HEADER.size));current=header[1]
                        if sid is None:
                            gateway.attach(current,header[3]);sid=current
                        if sid!=current:raise ValueError('lane changed logical session')
                        size=header[8]
                        if size>MAX_PACKED:raise ValueError('shard too large')
                        part=recv_exact(self.request,size)
                        assembly=gateway.submit(header,part)
                        if header[4]==0:
                            if not assembly.event.wait(timeout):raise TimeoutError('incomplete parallel request')
                            raw=assembly.reply;body=zlib.compress(raw,1)
                            self.request.sendall(b'R'+struct.pack('!II',len(body),len(raw))+hashlib.sha256(body).digest()+body)
                        else:self.request.sendall(b'A')
                except (ConnectionError,OSError):pass
                except Exception as error:
                    body=str(error).encode()[:8192]
                    try:self.request.sendall(b'E'+len(body).to_bytes(4,'big')+body)
                    except OSError:pass
                finally:
                    if sid is not None:gateway.detach(sid)
        super().__init__(address,Handler)
    def attach(self,sid,port):
        with self.lock:
            if port not in self.ports:raise ValueError('backend port not allowed')
            if sid not in self.sessions:
                if len(self.sessions)>=16:raise ValueError('too many parallel sessions')
                self.sessions[sid]=Session(port)
            session=self.sessions[sid]
            if session.port!=port:raise ValueError('session backend changed')
            session.connections+=1
    def detach(self,sid):
        with self.lock:
            session=self.sessions.get(sid)
            if session is None:return
            session.connections-=1
            if session.connections<=0:
                if session.backend:
                    try:session.backend.shutdown(socket.SHUT_RDWR)
                    except OSError:pass
                    session.backend.close()
                if session.current:
                    session.current.reply=b'{"error":{"code":"TRANSPORT","message":"parallel lanes closed"}}'
                    session.current.event.set()
                del self.sessions[sid]
    def submit(self,header,part):
        magic,sid,seq,port,index,count,total,raw,size,digest=header
        if magic!=MAGIC or port not in self.ports:raise ValueError('magic/backend port not allowed')
        if not (1<=count<=64 and 0<=index<count and 0<total<=MAX_PACKED and 0<raw<=MAX_BYTES and size==len(part)<=total):
            raise ValueError('invalid shard bounds')
        complete=False
        with self.lock:
            session=self.sessions.get(sid)
            if session is None:
                if len(self.sessions)>=16:raise ValueError('too many parallel sessions')
                session=self.sessions.setdefault(sid,Session(port))
            if session.failed:raise ValueError('failed model session; create a new connection and reset')
            if session.port!=port or seq!=session.sequence:raise ValueError('out-of-order model request')
            if session.current is None:session.current=Assembly(port,count,total,raw,digest)
            assembly=session.current
            if assembly.metadata!=(port,count,total,raw,digest):raise ValueError('inconsistent shard metadata')
            if assembly.parts[index] is not None:raise ValueError('duplicate shard')
            if sum(len(p) for p in assembly.parts if p is not None)+size>total:
                raise ValueError('aggregate shard bytes exceed declared total')
            assembly.parts[index]=part
            complete=all(p is not None for p in assembly.parts)
        if complete:
            packed=b''.join(assembly.parts)
            if len(packed)!=total or hashlib.sha256(packed).digest()!=digest:raise ValueError('request digest mismatch')
            request=inflate(packed,raw)
            try:
                if session.backend is None:session.backend=socket.create_connection((self.backend_host,port),timeout=self.timeout)
                backend=session.backend;backend.sendall(len(request).to_bytes(4,'big')+request)
                n=int.from_bytes(recv_exact(backend,4),'big')
                if not 0<n<=MAX_BYTES:raise ValueError('invalid backend response length')
                reply=recv_exact(backend,n)
            except Exception as error:
                # Poison an uncertain byte stream; no reconnect, retry, or later stale reply.
                # 后端读写失败即废弃会话，禁止后续读到旧回复。
                session.failed=True
                if session.backend:
                    try:session.backend.shutdown(socket.SHUT_RDWR)
                    except OSError:pass
                    session.backend.close();session.backend=None
                reply=json.dumps({'error':{'code':'TRANSPORT','message':str(error)}}).encode()
            with self.lock:
                session.sequence+=1;session.current=None;assembly.reply=reply;assembly.event.set()
            logging.info('RPC sid=%s seq=%s port=%s shards=%s raw=%s packed=%s',sid.hex(),seq,port,count,raw,total)
        return assembly if index==0 else None
    def server_close(self):
        for sid in list(self.sessions):
            with self.lock:self.sessions[sid].connections=0
            self.detach(sid)
        super().server_close()


class ParallelTunnel:
    """Transparent local TCP frontend for the existing PolicyClient / 原Client透明TCP前端。"""
    def __init__(self,server):
        lanes=int(server.get('parallel_lanes',64))
        if lanes!=64:raise ValueError('this deployment requires64 lanes')
        opened=[];futures=[]
        try:
            # Bound concurrent authentication to avoid SSH MaxStartups drops.
            with ThreadPoolExecutor(max_workers=4) as executor:
                futures=[executor.submit(SSHStream,server['ssh_host'],server.get('parallel_gateway_port',19578),30) for _ in range(lanes)]
                try:
                    for f in futures:opened.append(f.result())
                except Exception:
                    for f in futures:f.cancel()
                    raise
        except Exception:
            # The executor waits for in-flight opens, so collect successes even after a failed future.
            for f in futures:
                if f.done() and not f.cancelled() and f.exception() is None:
                    stream=f.result()
                    if stream not in opened:opened.append(stream)
            for stream in opened:stream.close()
            raise
        self.pool=ParallelPool(opened,server['server_port'],server.get('timeout_s',300))
        pool=self.pool
        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                try:
                    while True:
                        n=int.from_bytes(recv_exact(self.request,4),'big')
                        if not 0<n<=MAX_BYTES:raise ValueError('invalid client message size')
                        raw=pool.rpc(recv_exact(self.request,n));self.request.sendall(len(raw).to_bytes(4,'big')+raw)
                except (ConnectionError,OSError):pass
                except Exception as error:
                    raw=json.dumps({'error':{'code':'TRANSPORT','message':str(error)}}).encode()
                    try:self.request.sendall(len(raw).to_bytes(4,'big')+raw)
                    except OSError:pass
        class Frontend(socketserver.ThreadingTCPServer):
            allow_reuse_address=True;daemon_threads=True
        try:self.server=Frontend(('127.0.0.1',server['local_port']),Handler)
        except Exception:self.pool.close();raise
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
    def close(self):
        self.pool.close();self.server.shutdown();self.server.server_close();self.thread.join(timeout=2)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--listen-port',type=int,default=19578)
    parser.add_argument('--backend-host',required=True)
    parser.add_argument('--ports',default='19300,19301,19302,19303,19304,19305,19306,19307')
    args=parser.parse_args();logging.basicConfig(level=logging.INFO,format='%(asctime)s %(message)s')
    with Gateway(('127.0.0.1',args.listen_port),args.backend_host,{int(p) for p in args.ports.split(',')}) as server:
        logging.info('LISTENING localhost:%s backend=%s fixed64',args.listen_port,args.backend_host)
        server.serve_forever()

if __name__=='__main__':main()
