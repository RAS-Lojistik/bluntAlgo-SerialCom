import serial
import time

#Lists to be filled later
pairedData = [None, None, None, None, None, None, None, None, None] 
dlNums = []
consecutiveDestinations = []

turns = [] #What we're ultimately looking for

#Sample data
dlData = ["B1","Z2","C2","Y1","B3","X2","D1","T3"] #1 x4
dlData = ["A4","X3","D2","T1","C1","Z2","B3","Y3"] #2 x4
dlData = ["A4","X3","B4","X1","C4","X2","D4","X3"] #3 x4
dlData = ["B2","Y2","A1","X3","B3","Y3","C2","X1", "B1", "Z1"] #1 x3
dlData = ["B3","X2","C2","Y2","A1","Z3","C3","X3","B1","Y3","A2","X1","A3","Y1","B2","Z1","C1","Z2"] #2 x3
dlData = ["B2","Y2","A1","X3","B3","Y3","C2","X1", "B1", "X2"] #3 x3
loadNumber = 5 #int(len(dlNums)/2)
gridSize = 3

#Table-to-coordinate information
dCoordDict = {"C3":(1,1), "C2":(2,1), "C1":(3,1), "B3":(1,2), "B2":(2,2), "B1":(3,2), "A3":(1,3), "A2":(2,3), "A1":(3,3)}
lCoordDict = {"X1":(3,0), "X2":(3,0), "X3":(3,0), "Y1":(2,0), "Y2":(2,0), "Y3":(2,0), "Z1":(1,0), "Z2":(1,0), "Z3":(1,0)}

#Direction information
direction = {"EE":"f", "EN":"l", "EW":"ll", "ES":"r", "WW":"f", "WS":"l", "WE":"ll", "WN":"r", "NN":"f", "NW":"l", "NS":"rr", "NE":"r", "SS":"f", "SE":"l", "SN":"rr", "SW":"r"}
loadInfoDict = {"1":"flfrfplflfrf", "2":"ffplff", "3":"frflfprfrflf"}

#Position and destination variables for coordinates -robot is initially at (1,0)-
position = (1,0)
destination = (None, None)

#Pointer that points towards forward (initially E)
pointer = "E"

#Delivery-load pair constructor
def pairConstructor(dlData, pairedData, loadNumber, dCoordDict, lCoordDict):
    k = -1
    for i in range(0, 2*loadNumber, 2):
        k = k + 1
        pairedData[k] = (dCoordDict[dlData[i]], lCoordDict[dlData[i+1]])

#Move in Y until destination y is reached, and append necessary steps
def yMove(pointer, position, destination):
    while not (position[1] == destination[1]):
        if (destination[1] > position[1]):
            finalPointer = "N"
            turns.append(direction[pointer+finalPointer])
            dummy = pointer
            pointer = finalPointer
            if not (finalPointer == dummy):
                continue
            position = (position[0], position[1]+1)
            
        elif (destination[1] < position[1]):
            finalPointer = "S"
            #Edge case
            if(direction[pointer+finalPointer] == "rr" or direction[pointer+finalPointer] == "ll"):
                if (position[0] == 1):
                    turns.append("l")
                    dummy = pointer
                    pointer = finalPointer
                    continue
                elif(position[0] == 3):
                    turns.append("r")
                    dummy = pointer
                    pointer = finalPointer
                    continue
            turns.append(direction[pointer+finalPointer])
            dummy = pointer
            pointer = finalPointer
            if not (finalPointer == dummy):
                continue
            position = (position[0], position[1]-1)
    return (pointer, position)

#Move in X until destination x is reached, and append necessary steps (+add extra steps for load points)
def xMove(pointer, position, destination):
    while not (position[0] == destination[0]):
        if (destination[0] > position[0]):
            finalPointer = "E"
            turns.append(direction[pointer+finalPointer])
            dummy = pointer
            pointer = finalPointer
            if not (finalPointer == dummy):
                continue
            position = (position[0]+1, position[1])
        elif (destination[0] < position[0]):
            finalPointer = "W"
            turns.append(direction[pointer+finalPointer])
            dummy = pointer
            pointer = finalPointer
            if not (finalPointer == dummy):
                continue
            position = (position[0]-1, position[1])        
    return (pointer, position)

#Construct the list of all consecutive destinations, and delivery-load numbers list
def consecConstructor(pairedData, consecutiveDestinations, gridSize, dlNums, dlData):
    for x in range(gridSize, 0, -1):
        for y in range(gridSize, 0, -1):
            for i in range(loadNumber):
                if ((x,y) == pairedData[i][0]):
                    consecutiveDestinations.append(pairedData[i][1])
                    consecutiveDestinations.append((x,y))
                    dlNums.append(dlData[2*i][1])
                    dlNums.append(dlData[2*i+1][1])
    consecutiveDestinations.append((consecutiveDestinations[-1][0],0))
    consecutiveDestinations.append((1,0))

#Finally, do everything
def solve(dlData, pairedData, loadNumber, dCoordDict, lCoordDict, pointer, position, destination, consecutiveDestinations, gridSize, loadInfoDict, dlNums):
    
    #Do the preliminary
    pairConstructor(dlData, pairedData, loadNumber, dCoordDict, lCoordDict)
    consecConstructor(pairedData, consecutiveDestinations, gridSize, dlNums, dlData)
    
    print(consecutiveDestinations)
    
    #Solve
    destination = consecutiveDestinations[0]
    (pointer, position) = xMove(pointer, position, destination)
    for i in range(1, len(consecutiveDestinations), 2):
        
        if (i < len(dlNums)):
            if not (pointer == "S"):
                turns.append(direction[pointer+"S"])
            turns.append(loadInfoDict[dlNums[i]])
            pointer = "N"
        destination = consecutiveDestinations[i]
        
        if not (position[0] == destination[0]):
            (pointer, position) = xMove(pointer, position, destination)
        if not (position[1] == destination[1]):
            (pointer, position) = yMove(pointer, position, destination)
        if not (i+1 >= len(consecutiveDestinations)):
            turns.append("d")
            destination = consecutiveDestinations[i+1]
            if not (position[1] == destination[1]):
                (pointer, position) = yMove(pointer, position, destination)
            if not (position[0] == destination[0]):
                (pointer, position) = xMove(pointer, position, destination)
                
    if not (pointer == "W"):
        turns.append(direction[pointer+"W"])
    turns.append("fls")
    print(turns)
    a = ""
    for i in turns:
        a = a + i
    return a

fin = solve(dlData, pairedData, loadNumber, dCoordDict, lCoordDict, pointer, position, destination, consecutiveDestinations, gridSize, loadInfoDict, dlNums)

path = '/dev/cu.usbserial-110' #/dev/ttyACM0 , /dev/cu.usbserial-110
if __name__ == '__main__':
    ser = serial.Serial(path, 9600, timeout=1)
    ser.reset_input_buffer()
        
    while True:
        ser.write(fin.encode('utf-8'))  
        response = ser.read(1)
        print(response)
        if response:
            if ord(response) == 31:
                print(ord(response))
                ser.close()
                break
        time.sleep(1)
