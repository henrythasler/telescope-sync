import sys
import socketserver
from struct import *
from datetime import datetime

class MyTCPHandler(socketserver.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        print("{} wrote:".format(self.client_address[0]))

        print(f'Received {len(self.data)} Bytes: {self.data.hex()}')

        if(len(self.data) >= 20):
            message_raw = unpack('<hhqIi', self.data)
            print(f'Raw Message: {message_raw}')

            message = {
                "length_bytes": message_raw[0],
                "type": message_raw[1],
                "time": datetime.fromtimestamp(message_raw[2]/1000/1000),
                "ra_h": message_raw[3] / (0x80000000 / 12.),
                "dec_deg": message_raw[4] / (0x40000000 / 90.)
                }
            print(f'Message: {message}')
        else:
            print("message too short for decoding")
            # 1400 0000 c5e3525a49d30500 fb64f828 508ed9

if __name__ == "__main__":
    HOST, PORT = "localhost", 10001

    # Create the server, binding to localhost on port 9999
    with socketserver.TCPServer((HOST, PORT), MyTCPHandler) as server:
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            server.server_close()
            sys.exit(0)