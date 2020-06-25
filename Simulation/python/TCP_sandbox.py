# -*- coding: utf-8 -*-
"""
Created on Thu Jun 25 00:04:46 2020

@author: rega0051
"""
# Legacy:
# SIL SETUP
# start_SIL_Comm.sh
# socat -d -d PTY,link=ptySimSoc,rawer PTY,link=ptySimFmu,rawer; stty sane;

# HIL SETUP
# start_HIL_CommHost.sh                                                                                                   start_HIL_CommHost.sh                                                                                                   ::::::::::::::                                                                                                          socat -d -d PTY,link=ptySimFmu,rawer tcp:192.168.7.2:8000; stty sane;                                                   ::::::::::::::                                                                                                          start_HIL_CommSOC.sh                                                                                                    ::::::::::::::                                                                                                          socat -d -d tcp-listen:8000,reuseaddr,fork PTY,link=ptySimSoc,rawer; stty sane;                                         ::::::::::::::                                                                                                          start_SIL_Comm.sh                                                                                                       ::::::::::::::                                                                                                          socat -d -d PTY,link=ptySimSoc,rawer PTY,link=ptySimFmu,rawer; stty sane;
# socat -d -d PTY,link=ptySimFmu,rawer tcp:192.168.7.2:8000; stty sane;

# start_HIL_CommSOC.sh 
# socat -d -d tcp-listen:8000,reuseaddr,fork PTY,link=ptySimSoc,rawer; stty sane;

# Issues:
# On Windows PTY isnt a things...
# Python needs to handle the socket somehow...
# SOC code is always Linux, so flight will still use: ptySimSoc
# ... which will somehow become TCP packets.
# Would like one interface that always works (SIL or HIL are the same)

import socketserver

# Create a TCP Server to run in the background (thread)
class MyTCPHandler(socketserver.StreamRequestHandler):
    def handle(self):
        # self.rfile is a file-like object created by the handler;
        # we can now use e.g. readline() instead of raw recv() calls
        self.data = self.rfile.readline().strip()
        print("{} wrote:".format(self.client_address[0]))
        print(self.data)
        # Likewise, self.wfile is a file-like object used to write back to the client
        self.wfile.write(self.data.upper())
       

HOST, PORT = "localhost", 8000
server = socketserver.ThreadingTCPServer((HOST, PORT), MyTCPHandler, bind_and_activate=True)
server_thread = socketserver.threading.Thread(target=server.serve_forever)
server_thread.daemon = True
server_thread.start()

# Create a Connection from the FMU side.
import serial
s = serial.serial_for_url('socket://' + HOST + ':' + str(PORT), do_not_open = True)
s.open()

#%% Test
# On SOC:
# socat -d -d -d PTY,link=ptySimSoc,rawer tcp-listen:8000

# cat < ptySimSoc
s.write(b'Hello')

# echo "Test" > ptySimSoc
s.readline()

#%%
server.shutdown()
server.server_close()

