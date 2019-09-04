import pandas as pd
import numpy as np
import sys

def parse_if_number(s):
	try: return float(s)
	except: return True if s=="true" else False if s =="false" else s if s else None

def parse_ndarray(s):
	return np.fromstring(s, sep=' ') if s else None

def readAndPrint(filename):
	print(filename)
	m3 = pd.read_csv(filename, converters = {
	'attrvalue': parse_if_number,
	'binedges': parse_ndarray,
	'binvalues': parse_ndarray,
	'vectime':parse_ndarray,
	'vecvalue': parse_ndarray
	})

	print("NoOfNodes:NumberTotallySentByallNodes:E2EMean:E2EStdDev:NumberTotallyFromAppLayer:NumCollisions:NumberReceivedAtBS:minislotrequests:(SentByNodes/RecvGw+Collisions):(SentByNodes/SentByAppLayer)")
	#print(m3.name.unique())
	for run in m3.run.unique():
		cnt = 0
		alldata = np.array([])
		sentAppLayer = 0
		numCollisions = 0
		minislotrequests = 0
		noofnodes = 0
		for l in (m3.loc[(m3['name'] == 'LoRa_GWPacketReceived:count') & (m3.type == 'scalar') & (m3.run == run)].value):
			gw_recv = l
	
        	for l in (m3.loc[(m3['name'] == 'LoRa_AppPacketSent:count') & (m3.type == 'scalar') & (m3.run == run)].value):
                	sentAppLayer = sentAppLayer + l

        	for l in (m3.loc[(m3['name'] == 'numCollisions') & (m3.type == 'scalar') & (m3.run == run)].value):
                	numCollisions = numCollisions + l	

        	for l in (m3.loc[(m3['name'] == 'numSent') & (m3.type == 'scalar') & (m3.run == run)].value):
                	minislotrequests = minislotrequests + l

		for i in ((m3.loc[(m3['run'] == run) & (m3['name'] == 'End to End Delay at Node')].vecvalue)):
			cnt = cnt + (len(i))
			alldata = np.concatenate((alldata,i),axis=0)

		for l in (m3.loc[(m3.attrname.astype(str) == 'nodes') & (m3.run == run)].attrvalue):
                	noofnodes = l

		if noofnodes != 0:
			printer = [noofnodes,cnt,np.mean(alldata),np.std(alldata),sentAppLayer,numCollisions,gw_recv,minislotrequests]
			try:
				printer.append(cnt/(gw_recv+numCollisions))
			except:
				printer.append(0)
			try:
				printer.append(cnt/sentAppLayer)
			except:
				printer.append(0)
			print(':'.join(str(x) for x in printer))

for i in range(1,len(sys.argv)):
	readAndPrint(sys.argv[i])

