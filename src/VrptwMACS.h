//
// VrptwMACS.cpp
//
// Copyright (c) 2006-2007 Pascal Drecker
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//

//
//	31.12.2006  =pd=		first version
//

#ifndef _VRPTW_MACS_H_
#define _VRPTW_MACS_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


///// includes /////

#include "Vrptw.h"
#include "SolutionLogger.h"
#include "utils.h"
#include "pthread/include/pthread.h"


///// classes /////

class VrptwMACS : public Vrptw  
{
public:
	VrptwMACS();
	
	VrptwMACS(InstanceData *pInstanceData,
			  SolutionLogger *pSolutionLogger=NULL);
	
	virtual ~VrptwMACS();

	int run(int iCalcSeconds=300);

	void setParamAntsCount(short iAntsCount)
		{ if (iAntsCount >= 1) m_iAntsCount = iAntsCount; };
		
	void setParamBeta(short nBeta) { if (nBeta >= 0) m_nBeta = nBeta; };
	
	void setParamQ0(double dQ0) { if (dQ0 >= 0.0 && dQ0 <= 1.0) m_dQ0 = dQ0; };
	
	void setParamRoh(double dRho)
		{ if (dRho >= 0.0 && dRho <= 1.0) m_dRho = dRho; };

	void setParamXi(double dXi)
		{ if (dXi >= 0.0 && dXi <= 1.0) m_dXi = dXi; };

	int getParamAntsCount() { return m_iAntsCount; };
	
	short getParamBeta() { return m_nBeta; };
	
	double getParamQ0() { return m_dQ0; };
	
	double getParamRoh() { m_dRho; };

protected:
	void sortCustomersByDemand(int *piArray);
	
	void removeCustomerFromList(int iPos,
								int *piList);
	
	void insertCustomerIntoTourMatrix(int iPos,
									  int iCustomer,
									  int *piTourMatrix);
	
	void init();
	
	void cleanup();

	static void *acs_vei(void *pArg);
	
	static void *acs_time(void *pArg);
	
	static void CopyTourMatrix(int iVehicleCount,
							   int iToursMaxSize,
							   int **ppiDest,
							   int **ppiSrc);
	
	bool new_active_ant(bool bVEI,
						int iNodes,
						double dTau0,
						int iVehicleCount,
						int *piToursVehicleCount,
						double *pdToursDistance);
						
	bool insertion_procedure(bool bVEI,
							 bool *pbNodesVisited,
							 int iVehicleCount,
							 int **ppiTourMatrix,
							 double *pdTourDistance);

	// global
	bool m_bStopRunning;
	bool m_bMutexBetterSolution;
	pthread_mutex_t m_mutexBetterSolution;
	bool m_bCondBetterSolution;
	pthread_cond_t m_condBetterSolution;
	SolutionLogger *m_pSolutionLogger;

	int m_iAntsCount;
	short m_nBeta;
	double m_dQ0;
	double m_dRho;
	double m_dXi;
	int m_iToursMaxSize;

	int m_iDepotDueDate;
	int m_iCustomerCount;
	int *m_piCustomerDemand;
	int *m_piCustomerReadyTime;
	int *m_piCustomerDueDate;
	int *m_piCustomerServiceTime;

	bool m_bRWLockBestSoFar;
	pthread_rwlock_t m_rwlockBestSoFar;
	int **m_ppiTourMatrix_bestsofar;
	int m_iVehicleCount_bestsofar;
	double m_dDistance_bestsofar;
	int **m_ppiTourMatrix_acsvei;

	// acs_vei
	MTRand m_MTRand_vei;
	double **m_ppdPheromoneMatrix_vei;
	int *m_piIN_vei;
	bool *m_pbNodesVisited_vei;
	double *m_pdProbability_vei;
	int **m_ppiTourMatrix_vei;
	int *m_piCustomersToVisit_vei;
	double *m_pdLatestArrivals_vei;

	// acs_time
	MTRand m_MTRand_time;
	double **m_ppdPheromoneMatrix_time;
	bool *m_pbNodesVisited_time;
	double *m_pdProbability_time;
	int **m_ppiTourMatrix_time;
	int **m_ppiTourMatrix_newbest_time;
	int *m_piCustomersToVisit_time;
	double *m_pdLatestArrivals_time;
};

#endif // _VRPTW_MACS_H_
