import sst
import sst.macro

opticalLatency = "0ps"

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
			print "cibai"
			for srcId, dstId, srcOutport, dstInport in connections:
				print "srcId: %d and dstId: %d srcOutport: %d, dstInport: %d" % (srcId, dstId, srcOutport, dstInport)
				dstSwitch, dstParams = self.switches[dstId]
				makeUniNetworkLink(srcSwitch, srcId, srcOutport,
									dstSwitch, dstId, dstInport, 
									lat)
		return

	def build(self):
		self.buildEndpoints()
		topologyParams = self.params["topology"]
		topologyName = topologyParams["name"]
		self.buildElectricalSwitches()
		if self.containsOptics:
			self.buildOpticalSwitches()
		self.buildTopology()

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
