"""Count actual JSON bytes carried by multiprocessing Connection.send_bytes."""
import json
class Channel:
    def __init__(self,connection):self.connection=connection;self.sent=0;self.received=0
    def send(self,value):
        raw=json.dumps(value,default=str,separators=(',',':')).encode();self.connection.send_bytes(raw);self.sent+=len(raw)
    def recv(self):
        raw=self.connection.recv_bytes();self.received+=len(raw);return json.loads(raw)
    def poll(self,timeout):return self.connection.poll(timeout)
    def close(self):self.connection.close()
