#include <stdio.h>
#include "VrptwMACS.h"
#include "InstanceData.h"

static InstanceData g_InstanceData;
int main(int argc, char* argv){
	g_InstanceData.read("D:/Projects/VRPTW_C/tasks/c101.txt");
	VrptwMACS vrptw(&g_InstanceData);
	vrptw.run(10);
	return 0;
}