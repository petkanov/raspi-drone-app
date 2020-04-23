
class Utils:
        
    def createNetworkMessage(msgBodyBytes):
        
        length = len(msgBodyBytes)
        lengthEncodedTo4bytes = (length).to_bytes(4, byteorder='big')
        
        return lengthEncodedTo4bytes + msgBodyBytes     
     
    def readNetworkMessage(socket):
        
        bodySize = int.from_bytes(socket.recv(4), byteorder='big')
        body = socket.recv(bodySize)
        
        return body

    def createUDP_DatagramMessage(droneId, msgBody):
        return droneId.encode() + msgBody   