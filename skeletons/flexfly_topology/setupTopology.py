import sst
import sst.macro

class Interconnect:
    def __init__(self, params):
        self.params = params
        self.system = sst.macro.System(params)
        self.num_nodes = self.system.numNodes()
        self.num_switches = self.system.numSwitches()
        self.switches = [0]*self.num_switches
        self.nodes = [0]*self.num_nodes #just expands the size of an initial array with 1 element 0 to size 1 * num_nodes

    def buildEndpoints(self):
        nodeParams = self.params["node"]
        modelName = nodeParams["model"]
        for i in range(self.num_nodes): 
            self.switches[i] = sst.Component("Node %d" % i, "macro.%s" % (modelName + "_node"))
            self.switches[i].addParam("id", i)
            self.switches[i].addParam(nodeParams)
        return

    def buildOpticalSwitches(self):

        return

    def buildElectricalSwitches(self):
        return

    def buildTopology(self):
    	return

    def build(self):
        self.buildEndpoints()
        self.buildElectricalSwitches()
        topologyName = self.params["topology"]
        print(topologyName)
        if topologyName is "flexfly" or topologyName is "flexfly_topology":
        	self.buildOpticalSwitches()
        self.buildTopology()

    ## Returns the command line argv in terms of a vector that is 0-indexed
def readCmdLineParams():
	import sys
	return sst.macro.readParams(sys.argv)

def setupTopology():
	import sys
	params = readCmdLineParams()
	#print("helllloooooo")
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
	print params
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
