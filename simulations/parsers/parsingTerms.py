import pandas as pd
import numpy as np
import sys
from prettytable import PrettyTable
import concurrent.futures

def parse_if_number(s):
    try: return float(s)
    except: return True if s=="true" else False if s =="false" else s if s else None

def parse_ndarray(s):
    return np.fromstring(s, sep=' ') if s else None

def readAndPrint(filename):
    m3 = pd.read_csv(filename, converters = {
    'attrvalue': parse_if_number,
    'binedges': parse_ndarray,
    'binvalues': parse_ndarray,
    'vectime':parse_ndarray,
    'vecvalue': parse_ndarray
    })

    attributes = ['nodes','load','gatewaysPerNS']
    scalarsToRead = ['numSent','totalEnergyConsumed','LoRa_GWPacketReceived:count','LoRa_GW_DER','sentPackets','GW_droppedDC']
    vectorsToRead = ['NumberOfRetransmissions']

    header = []
    tabledCreated = False
    prettyTable = []
    for run in m3.run.unique():
        values = []
        for attr in attributes:
            header.append(attr)
            values.append(parse_if_number(m3.loc[(m3.attrname.astype(str) == attr) & (m3.run == run)].attrvalue))

        for scalar in scalarsToRead:
            header.extend([scalar + ' total', scalar + ' average'])
            val = 0
            count = 0.0
            for i in (m3.loc[(m3['name'] == scalar) & (m3.type == 'scalar') & (m3.run == run)].value):
                val = val + i
                count = count + 1.0
            if val == 0:
                values.extend([0.0,0.0])
            else:
                values.extend([val,val/count])

        for vector in vectorsToRead:
            header.extend([vector, vector + str(' Min'), vector + str(' Max'), vector + str(' Mean'), vector + str(' StdDev')])
            val = 0
            alldata = np.array([])
            for i in (m3.loc[(m3['name'] == vector) & (m3.run == run)].vecvalue):
                if type(i) == np.ndarray:
                    val = val + len(i)
                    alldata = np.concatenate((alldata,i),axis=0)
            if val == 0:
                values.extend([0,0,0,0,0])
            else:
                values.extend([val,np.min(alldata),np.max(alldata),np.mean(alldata),np.std(alldata)])

        if not tabledCreated:
            prettyTable = PrettyTable(header)
            tabledCreated = True
        prettyTable.add_row(values)
    print(filename)
    print(prettyTable)


if __name__ == '__main__':
    filesToParse = sys.argv[1:]
    for i in range(1,len(sys.argv)):
        readAndPrint(sys.argv[i])
