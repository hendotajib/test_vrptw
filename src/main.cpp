#include <stdio.h>
#include "VrptwMACS.h"
#include "InstanceData.h"

static InstanceData g_InstanceData;
int main(int argc, char** argv){
	g_InstanceData.read("D:/Projects/test_vrptw/c101_25.txt");
	SolutionLogger *logger = new SolutionLogger();
	VrptwMACS vrp(&g_InstanceData, logger);
	vrp.run(10);
	logger->write("log.txt");
	int vehicle;
	double dist;
	int *tours = g_InstanceData.getSolutionTours(&vehicle, &dist);
	return 0;
}
