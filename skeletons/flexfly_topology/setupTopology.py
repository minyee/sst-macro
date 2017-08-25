import sst
import sst.macro

opticalLatency = "0ps"
smallLatency = "1ps"

def makeUniLink(linkType,srcComp,srcId,srcPort,dstComp,dstId,dstPort,outLat=None,inLat=None):
	if not outLat : outLat = inLat
	if not inLat: inLat = outLat
	if not outLat: sys.exit("must specify at least one latency for link")

	linkName = "%s%d:%d->%d:%d" % (linkType,srcId,srcPort,dstId,dstPort)
	link = sst.Link(linkName)
	portName = "output %d %d" % (srcPort, dstPort)
	srcComp.addLink(link,portName,outLat)
	portName = "input %d %d" % (srcPort, dstPort)
	dstComp.addLink(link,portName,inLat)

def makeUniNetworkLink(srcComp,srcId,srcPort,dstComp,dstId,dstPort,outLat=None,inLat=None):
	makeUniLink("network",srcComp,srcId,srcPort,dstComp,dstId,dstPort,outLat,inLat)

def makeBiLink(linkType,comp1,id1,port1,comp2,id2,port2,outLat=None,inLat=None):
	makeUniLink(linkType,comp1,id1,port1,comp2,id2,port2,outLat,inLat)
	makeUniLink(linkType,comp2,id2,port2,comp1,id1,port1,outLat,inLat)

class Interconnect:
	def __init__(self, params):
		topologiesWithOptics = [
			"flexfly",
			"flexfly_topology"
		]
		self.params = params
		self.system = sst.macro.System(params)
		self.num_nodes = self.system.numNodes()
		self.num_switches = self.system.numSwitches()
		self.switches = [0]*self.num_switches
		self.nodes = [0]*self.num_nodes #just expands the size of an initial array with 1 element 0 to size 1 * num_nodes
		topologyName = (self.params["topology"])["name"]
		if topologyName in topologiesWithOptics:
			self.containsOptics = True
			self.optical_topology_params = self.params["topology"]
			self.num_groups = int(self.optical_topology_params["groups"])
			self.switches_per_group = int(self.optical_topology_params["switches_per_group"])
			self.nodes_per_switch = int(self.optical_topology_params["nodes_per_switch"])
			self.optical_switch_radix = int(self.optical_topology_params["optical_switch_radix"])
			if self.optical_topology_params.has_key("link_multiplicity"):
				self.link_multiplicity = int(self.optical_topology_params["link_multiplicity"])
			else:
				self.link_multiplicity = 1
			# really just equals to h in Kim's literature (num of global links per switch)
			self.num_nodes = self.num_groups * self.switches_per_group * self.nodes_per_switch
			self.optical_switches_per_group = self.switches_per_group * self.link_multiplicity / self.optical_switch_radix
			if self.optical_switches_per_group % self.optical_switch_radix > 0:
				self.optical_switch_radix += 1
		else:
			self.containsOptics = False

	def buildEndpoints(self):
		nodeParams = self.params["node"]
		modelName = nodeParams["model"]
		for i in range(self.num_nodes): 
			self.nodes[i] = sst.Component("Node %d" % i, "macro.%s" % (modelName + "_node"))
			self.nodes[i].addParams(macroToCoreParams(nodeParams))
			self.nodes[i].addParam("id", i)       
		return

	def buildOpticalSwitches(self):
		switchParams = self.params["switch"]
		switchName = "logp" + "_switch"
		#switchName = switchParams["model"] + "_switch"
		for group in range(self.num_groups):
			group_index_offset = group * (self.switches_per_group + self.optical_switches_per_group) \
									+ self.switches_per_group
			for i in range(self.optical_switches_per_group):
				optical_switch_id = group_index_offset + i
				switch = sst.Component("Switch %d" % optical_switch_id, "macro.%s" % switchName)
				switch.addParam("id" , i)
				switch.addParams(macroToCoreParams(switchParams))
				switch.addParam("switch_type" , "optical")
				self.switches[group_index_offset + i] = switch
		return

	def buildElectricalSwitches(self):
		switchParams = self.params["switch"]
		switchName = "logp" + "_switch"
		#switchName = switchParams["model"] + "_switch"
		totalSwitchesPerGroup = self.optical_switches_per_group + self.switches_per_group
		for i in range(self.num_switches):
			if self.containsOptics and i % (totalSwitchesPerGroup) >= self.switches_per_group:
				continue
			switch = sst.Component("Switch %d" % i, "macro.%s" % switchName)
			switch.addParam("id" , i) 
			switch.addParams(macroToCoreParams(switchParams))
			switch.addParam("switch_type" , "electrical")
			self.switches[i] = switch  
		return

	def latency(self, params):
		if params.has_key("latency"):
			return params["latency"]
		elif params.has_key("send_latency"):
			return params["send_latency"]
		else:
			sys.exit("need link latency in parameters")

	def buildTopology(self):
		switchParams = self.params["switch"]
		for i in range(self.num_switches):
			linkParams = switchParams["link"]
			connections = self.system.switchConnections(i)
			srcSwitch = self.switches[i]
			lat = self.latency(linkParams)
			for srcId, dstId, srcOutport, dstInport in connections:
				#print "srcId: %d and dstId: %d srcOutport: %d, dstInport: %d" % (srcId, dstId, srcOutport, dstInport)
				#print self.switches

				dstSwitch = self.switches[dstId]
				linkName = "logPnetwork%d->%d" % (srcId,dstId)
        		link = sst.Link(linkName)
        		portName = "in-out %d %d" % (dstId, sst.macro.SwitchLogPNetworkPort)
        		srcSwitch.addLink(link, portName, lat)
        		portName = "in-out %d %d" % (i, sst.macro.SwitchLogPNetworkPort)
        		dstSwitch.addLink(link, portName, lat)
				#makeUniNetworkLink(srcSwitch, srcId, srcOutport,
				#					dstSwitch, dstId, dstInport, lat)
		return

	def buildNodeConnections(self):
		linkCnt = 0
		for i in range(self.num_nodes):
			switchId = self.system.nodeToLogPSwitch(i)
			linkname = "InjectionLink%d:Node%d->Switch%d" % (linkCnt, i, switchId)
			link = sst.Link(linkname)
			portName1 = "srcPort: %d -> dstPort: %d" % (linkCnt,linkCnt)
			portName2 = "srcPort: %d -> dstPort: %d" % (linkCnt,linkCnt)
			node = self.nodes[i]
			
			switch = self.switches[switchId]
			portName1 = "in-out %d %d" % (sst.macro.NICLogPInjectionPort, sst.macro.SwitchLogPInjectionPort)
			portName2 = "in-out %d %d" % (i, sst.macro.SwitchLogPInjectionPort)
			node.addLink(link, portName1, smallLatency)
			switch.addLink(link, portName2, smallLatency)
			linkCnt += 1

	def makeOneOpticalSwitch(self, switchParams, i):
		opticalSwitchName = "macro." + "flexfly_optical_switch"
		opticalSwitch = sst.Component("Switch %d" % i, opticalSwitchName)
		opticalSwitch.addParam("id" , i) 
		opticalSwitch.addParams(macroToCoreParams(switchParams))
		opticalSwitch.addParam("switch_type" , "optical")
		opticalSwitch.addParam("optical_switch_radix", self.opticalSwitchRadix)

	def build(self):
		self.buildEndpoints()
		topologyParams = self.params["topology"]
		topologyName = topologyParams["name"]
		self.opticalSwitchRadix = topologyParams["optical_switch_radix"]
		self.buildElectricalSwitches()
		if self.containsOptics:
			self.buildOpticalSwitches()
		self.buildTopology()
		self.buildNodeConnections()
		switchParams = self.params["switch"]
		self.makeOneOpticalSwitch(switchParams, 3000)

## Returns the command line argv in terms of a vector that is 0-indexed
def readCmdLineParams():
	import sys
	return sst.macro.readParams(sys.argv)

def redoSubParams_impl(nsArr, theDict, allParams):
	for key in theDict:
		val = theDict[key]
		if isinstance(val, dict):
			newNsArr = nsArr[:]
			newNsArr.append(key)
			redoSubParams_impl(newNsArr, val, allParams)
		else:
			paramArr = nsArr[:]
			paramArr.append(key)
			newParam = ".".join(paramArr)
			allParams.append((newParam, val))

def macroToCoreParams(theDict):
	allParams = []
	redoSubParams_impl([], theDict, allParams)
	newDict = {}
	for key, val in allParams:
		newDict[key] = val
	return newDict

def setupTopology():
	import sys
	params = readCmdLineParams()
	builtinApps = [
   		"apitest",
       	"global_test",
       	"hello_world",
       	"mpi_coverage",
       	"mpi_ping_all",
       	"mpi_print_nodes",
       	"mpi_topology",
       	"parsedumpi",
       	"sstmac_mpi_testall",
       	"traffic_matrix",
       	"user_app_cxx_empty_main",
       	"user_app_cxx_full_main",
	]

	for appIdx in range(15):
		appKey = "app%d" % (appIdx + 1)
		nodeParams = params["node"]
		if nodeParams.has_key(appKey):
			appParams = nodeParams[appKey]
			appName = appParams["name"]
			if not appName in builtinApps:
				cmd = "import sst.%s" % appName
				exec(cmd)
			del nodeParams[appKey]    
	interconnect = Interconnect(params)
	interconnect.build()
