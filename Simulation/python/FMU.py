#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Copyright (c) 2016 - 2020 Regents of the University of Minnesota.
MIT License; See LICENSE.md for complete details
Author: Chris Regan
'''

import serial
import fmu_messages


#%% 
class AircraftSocComms():
    def __init__ (self, port):
        
        self.serialLink = SerialLink(port)
        
    def Begin(self):
        print("Initializing communication with SOC...")
        self.serialLink.begin()
        print("done!")
        
        return True
        
    def SendMessage(self, msgID, msgAddress, msgPayload, ackReq = True):
        # msgType is used by SerialLink to determine if a SerialLink-level Ack is required
        if ackReq:
            msgType = 3
        else:
            msgType = 2
            
        # print('Message Send: ' + '\tID: ' + str(msgID) + '\tAddress: ' + str(msgAddress) + '\tPayload: ' + str(msgPayload))
        
        # Join msgData as: ID + Address + Payload
        msgData = b''
        msgData += msgID.to_bytes(1, byteorder = 'little')
        msgData += msgAddress.to_bytes(1, byteorder = 'little')
        msgData += msgPayload
        
        self.serialLink.write(msgType, msgData);
        
        return True
        
    def CheckMessage(self):
        self.serialLink.checkReceived()
        return self.serialLink.available()
        
    def ReceiveMessage(self):
        msgID = None
        msgAddress = None
        msgPayload = None
        
        msgType, msgData = self.serialLink.read()
        
        if msgType >= 2: # command type message
            # Data = ID + Address + Payload
            msgID = int.from_bytes(msgData[0:1], byteorder = 'little') # Message ID
            msgAddress = int.from_bytes(msgData[1:2], byteorder = 'little') # Address
            msgPayload = msgData[2:]
            
            # Send an SerialLink Ack - FIXIT - move to SerialLink
#            self.serialLink.sendStatus(True)

        # print('Message Recv: ' + '\tID: ' + str(msgID) + '\tAddress: ' + str(msgAddress) + '\tPayload: ' + str(msgPayload))
        
        return msgID, msgAddress, msgPayload
        
    def SendAck(self, msgID, msgSubID = 0): # This is a Config Message Ack (not a SerialLink Ack!)

        configMsgAck = fmu_messages.config_ack()
        configMsgAck.ack_id = msgID
        configMsgAck.ack_subid = msgSubID
        
        self.SendMessage(configMsgAck.id, 0, configMsgAck.pack(), ackReq = False)

        return True
    

#%% Message Framing
# Every message should be framed (start and end) with 0x7E (b'~')
# The escape byte is 0x7D
# The invert byte for escape is 0x20
frameEdge = 0x7E; frameEdgeByte = bytes.fromhex('7E');
frameEscape = 0x7D; frameEscapeByte = bytes.fromhex('7D');
frameInvert = 0x20; frameInvertByte = bytes.fromhex('20');

def UnescapeMessage(msg):    
    msgUnEsc = b''
    escPrevFlag = False
    for char in msg:
        if not escPrevFlag:
            if char == frameEscape:
                escPrevFlag = True
            else:
                msgUnEsc += char.to_bytes(1, 'little')
        else:
            msgUnEsc += (char ^ frameInvert).to_bytes(1, 'little')
            escPrevFlag = False

    if escPrevFlag:
        raise ValueError('Invalid message, ends in escape byte')

    return msgUnEsc


def EscapeMessage(msg):    
    msgEsc = b''
    for char in msg:
        if char in (frameEdgeByte, frameEscapeByte):
            msgEsc += frameEscapeByte + (char ^ frameInvert).to_bytes(1, 'little')
        else:
            msgEsc += char.to_bytes(1, 'little')
            
    return msgEsc


#%% CRC
def crc16(data, crc=0):
    
    table = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
        ]
    
    for byte in data:
        crc = ((crc << 8) & 0xff00) ^ table[((crc >> 8) & 0xff) ^ byte]
    
    return crc & 0xffff


#%% SerialLink

class SerialLink():
    
    ser = serial.Serial()
    ser.timeout = 0
    readBufList = []
    
    def __init__ (self, port):
        self.ser.port = port
    
    def begin(self):
        # Open flush, probably overboard.
        if self.ser.isOpen():
            self.ser.flush()
            self.ser.close()
        self.ser.open()
        self.ser.flush()

    # Combines: 	beginTransmission(); write(); endTransmission(); sendTransmission();
    def write(self, msgType, msgData):
       
        # Message Type as Bytes
        msgTypeByte = msgType.to_bytes(1, byteorder = 'little')
        
        # Compute CRC
        crcBytes = crc16(msgTypeByte + msgData).to_bytes(2, byteorder = 'little')
        
        # Message: Framed, Escaped, with CRC
        msgBytes = frameEdgeByte + EscapeMessage(msgTypeByte + msgData + crcBytes) + frameEdgeByte
        
        # Write to serial
        numBytes = self.ser.write(msgBytes)
        
        return numBytes
        
    
    def checkReceived(self):
        # Read everything out of the serial
        if (self.ser.in_waiting > 0):
            msgRecv = self.ser.read(self.ser.in_waiting)
        
            # multiple messages could have been read, Split on edge framing
            msgLines = msgRecv.split(frameEdgeByte)
            
             # Remove empty, append to readBufList
            [self.readBufList.append(msg) for msg in msgLines if b'' is not msg]
        
        return len(self.readBufList)
    
    
    def available(self):
        return len(self.readBufList)
    

    def read(self):
        # Pop the oldest message off front
        msgBytes = self.readBufList.pop(0)
        
        # print('SerialLink Read: ' + str(msgBytes))

        # Pull apart Serial: Start + Escaped(Type + Data + CRC) + End
        # Start and End are already removed by checkReceived()
        msgBytes = UnescapeMessage(msgBytes) # Un-Escape the Message
        
        msgType = int.from_bytes(msgBytes[0:1], byteorder = 'little') # Message Type
        msgCRC = int.from_bytes(msgBytes[-2:], byteorder = 'little') # Last 2 bytes as integer
        
        msgData = None
        # Check the CRC
        if (crc16(msgBytes[0:-2]) == msgCRC):
            # Split msgData into: ID + Address + Payload
            
            if msgType == 0: # Nack type
                # print('Nack Recv')
                pass
            elif msgType == 1: # Ack type
                # print('Ack Recv')
                pass
            elif msgType == 2: # Command type, NOACK
                msgData = msgBytes[1:-2] # Data Bytes

            elif msgType == 3: # Command type, REQACK
                msgData = msgBytes[1:-2] # Data Bytes
                self.sendStatus(True)
                
            else:
                print('Unknown Type Recv: ' + str(msgBytes))
                    
        else: # CRC Failed - Send Nack (Can't be sure if required or not with good CRC)
            print('\tCRC Fail: ' + str(msgBytes))
            self.sendStatus(False)

        return msgType, msgData
    
    def sendStatus(self, ack):
        if ack == True:
            msgType = 1
        else:
            msgType = 0
        
        self.write(msgType, b'')
        