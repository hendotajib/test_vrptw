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


///// includes /////

#ifdef _WIN32
	#include <windows.h>
#endif

#include "VrptwMACS.h"
#include "InstanceData.h"
#include "utils.h"
#include <stdlib.h>
#include <time.h>


///// classes /////

VrptwMACS::VrptwMACS() : Vrptw()
{
	init();
	cleanup();

	m_pSolutionLogger = NULL;
}

VrptwMACS::VrptwMACS(InstanceData *pInstanceData,
					 SolutionLogger *pSolutionLogger) : Vrptw(pInstanceData)
{
	init();
	cleanup();

	m_pSolutionLogger = pSolutionLogger;
}

VrptwMACS::~VrptwMACS()
{
	cleanup();
}

void VrptwMACS::init()
{
	m_bMutexBetterSolution = false;
	m_bCondBetterSolution = false;
	m_bRWLockBestSoFar = false;

	m_iAntsCount = 10;
	m_nBeta = 1;
	m_dQ0 = 0.9;
	m_dRho = 0.1;
	m_dXi = 0.1;
	
	m_ppiTourMatrix_bestsofar = NULL;
	m_ppiTourMatrix_acsvei = NULL;
	
	m_ppdPheromoneMatrix_vei = NULL;
	m_piIN_vei = NULL;

	m_ppdPheromoneMatrix_time = NULL;

	m_ppdPheromoneMatrix_vei = NULL;
	m_piIN_vei = NULL;
	m_pbNodesVisited_vei = NULL;
	m_pdProbability_vei = NULL;
	m_ppiTourMatrix_vei = NULL;
	m_piCustomersToVisit_vei = NULL;
	m_pdLatestArrivals_vei = NULL;

	m_ppdPheromoneMatrix_time = NULL;
	m_pbNodesVisited_time = NULL;
	m_pdProbability_time = NULL;
	m_ppiTourMatrix_time = NULL;
	m_ppiTourMatrix_newbest_time = NULL;
	m_piCustomersToVisit_time = NULL;
	m_pdLatestArrivals_time = NULL;
}

void VrptwMACS::cleanup()
{
	if (m_bMutexBetterSolution)
	{
		pthread_mutex_destroy(&m_mutexBetterSolution);
		m_bMutexBetterSolution = false;
	}

	if (m_bCondBetterSolution)
	{
		pthread_cond_destroy(&m_condBetterSolution);
		m_bCondBetterSolution = false;
	}

	if (m_bRWLockBestSoFar)
	{
		pthread_rwlock_destroy(&m_rwlockBestSoFar);
		m_bRWLockBestSoFar = false;
	}

	if (m_ppiTourMatrix_bestsofar != NULL)
	{
		free(m_ppiTourMatrix_bestsofar);
		m_ppiTourMatrix_bestsofar = NULL;
	}

	if (m_ppiTourMatrix_acsvei != NULL)
	{
		free(m_ppiTourMatrix_acsvei);
		m_ppiTourMatrix_acsvei = NULL;
	}

	if (m_ppdPheromoneMatrix_vei != NULL)
	{
		free(m_ppdPheromoneMatrix_vei);
		m_ppdPheromoneMatrix_vei = NULL;
	}

	if (m_piIN_vei != NULL)
	{
		free(m_piIN_vei);
		m_piIN_vei = NULL;
	}

	if (m_pbNodesVisited_vei != NULL)
	{
		free(m_pbNodesVisited_vei);
		m_pbNodesVisited_vei = NULL;
	}

	if (m_pdProbability_vei != NULL)
	{
		free(m_pdProbability_vei);
		m_pdProbability_vei = NULL;
	}

	if (m_ppiTourMatrix_vei != NULL)
	{
		free(m_ppiTourMatrix_vei);
		m_ppiTourMatrix_vei = NULL;
	}

	if (m_piCustomersToVisit_vei != NULL)
	{
		free(m_piCustomersToVisit_vei);
		m_piCustomersToVisit_vei = NULL;
	}

	if (m_pdLatestArrivals_vei != NULL)
	{
		free(m_pdLatestArrivals_vei);
		m_pdLatestArrivals_vei = NULL;
	}

	if (m_ppdPheromoneMatrix_time != NULL)
	{
		free(m_ppdPheromoneMatrix_time);
		m_ppdPheromoneMatrix_time = NULL;
	}

	if (m_pbNodesVisited_time != NULL)
	{
		free(m_pbNodesVisited_time);
		m_pbNodesVisited_time = NULL;
	}

	if (m_pdProbability_time != NULL)
	{
		free(m_pdProbability_time);
		m_pdProbability_time = NULL;
	}

	if (m_ppiTourMatrix_time != NULL)
	{
		free(m_ppiTourMatrix_time);
		m_ppiTourMatrix_time = NULL;
	}

	if (m_ppiTourMatrix_newbest_time != NULL)
	{
		free(m_ppiTourMatrix_newbest_time);
		m_ppiTourMatrix_newbest_time = NULL;
	}

	if (m_piCustomersToVisit_time != NULL)
	{
		free(m_piCustomersToVisit_time);
		m_piCustomersToVisit_time = NULL;
	}

	if (m_pdLatestArrivals_time != NULL)
	{
		free(m_pdLatestArrivals_time);
		m_pdLatestArrivals_time = NULL;
	}
}

int VrptwMACS::run(int iCalcSeconds)
{
	bool bError, bAcsVeiRunning, bAcsTimeRunning;
	int i, j, iCount, iMaxNodes, iVehicleCount;
	int *piNext, *piTours;

	pthread_t pthreadSelfID;
	pthread_t pthreadAcsVeiID;
	pthread_t pthreadAcsTimeID;
	void *pThreadReturn;
	struct timespec tsCancel;
	struct sched_param schedParamMain;
	struct sched_param schedParamVei;
	struct sched_param schedParamTime;

	// check params
	if (m_pInstanceData == NULL
		|| iCalcSeconds < 1)
	{
		return -1;
	}

	// cleanup
	cleanup();

	// init random number generator
#ifdef _DEBUG
	m_MTRand_vei.seed(1805017555); // everytime the same random numbers
	m_MTRand_time.seed(555180501); // everytime the same random numbers
#else
	m_MTRand_vei.seed();
	m_MTRand_time.seed();
#endif

	// init mutex, condition variable and rw-lock
	if (pthread_mutex_init(&m_mutexBetterSolution, NULL) != 0)
		return -2;

	m_bMutexBetterSolution = true;

	if (pthread_cond_init(&m_condBetterSolution, NULL) != 0)
		return -3;

	m_bCondBetterSolution = true;

	if (pthread_rwlock_init(&m_rwlockBestSoFar, NULL) != 0)
		return -4;

	m_bRWLockBestSoFar = true;

	// init vars
	bError = false;
	
	m_iCustomerCount = m_pInstanceData->getCustomerCount();
	m_iToursMaxSize = m_iCustomerCount+1;

	// solution logger
	if (m_pSolutionLogger != NULL)
	{
		m_pSolutionLogger->start("MACS-VRPTW", m_iCustomerCount*2);
		m_pSolutionLogger->addParameter("calc_seconds", iCalcSeconds);
		m_pSolutionLogger->addParameter("ants_count", m_iAntsCount);
		m_pSolutionLogger->addParameter("beta", m_nBeta);
		m_pSolutionLogger->addParameter("q0", m_dQ0);
		m_pSolutionLogger->addParameter("rho", m_dRho);
		m_pSolutionLogger->addParameter("xi", m_dXi);
	}

	// initial solution
	if (nn_gambardella1999() != 0)
		return -5;

	iVehicleCount = m_pInstanceData->getSolutionVehicleCount();
	iMaxNodes = m_iCustomerCount + iVehicleCount;

	m_iDepotDueDate = m_pInstanceData->getDepotDueDate();
	m_iCustomerCount = m_pInstanceData->getCustomerCount();
	m_piCustomerDemand = m_pInstanceData->getCustomerDemand();
	m_piCustomerReadyTime = m_pInstanceData->getCustomerReadyTime();
	m_piCustomerDueDate = m_pInstanceData->getCustomerDueDate();
	m_piCustomerServiceTime = m_pInstanceData->getCustomerServiceTime();

	// allocate memory
	m_ppiTourMatrix_bestsofar = generate_int_matrix(iVehicleCount,
													m_iToursMaxSize+1);

	m_ppiTourMatrix_acsvei = generate_int_matrix(iVehicleCount, m_iToursMaxSize+1);

	m_ppdPheromoneMatrix_vei = generate_double_matrix(iMaxNodes, iMaxNodes);

	m_piIN_vei = (int*)malloc(sizeof(int)*m_iCustomerCount);

	m_pbNodesVisited_vei = (bool*)malloc(sizeof(bool)*iMaxNodes);

	m_pdProbability_vei = (double*)malloc(sizeof(double)*iMaxNodes);

	m_ppiTourMatrix_vei = generate_int_matrix(iVehicleCount, m_iToursMaxSize+1);

	m_piCustomersToVisit_vei = (int*)malloc(sizeof(int)*(m_iCustomerCount+1));

	m_pdLatestArrivals_vei = (double*)malloc(sizeof(double)*(m_iCustomerCount+1));

	m_ppdPheromoneMatrix_time = generate_double_matrix(iMaxNodes, iMaxNodes);

	m_pbNodesVisited_time = (bool*)malloc(sizeof(bool)*iMaxNodes);

	m_pdProbability_time = (double*)malloc(sizeof(double)*iMaxNodes);

	m_ppiTourMatrix_time = generate_int_matrix(iVehicleCount, m_iToursMaxSize+1);

	m_ppiTourMatrix_newbest_time = generate_int_matrix(iVehicleCount,
													   m_iToursMaxSize+1);

	m_piCustomersToVisit_time = (int*)malloc(sizeof(int)*(m_iCustomerCount+1));

	m_pdLatestArrivals_time = (double*)malloc(sizeof(double)*(m_iCustomerCount+1));

	if (m_ppiTourMatrix_bestsofar == NULL
		|| m_ppiTourMatrix_acsvei == NULL
		|| m_ppdPheromoneMatrix_vei == NULL
		|| m_piIN_vei == NULL
		|| m_pbNodesVisited_vei == NULL
		|| m_pdProbability_vei == NULL
		|| m_ppiTourMatrix_vei == NULL
		|| m_piCustomersToVisit_vei == NULL
		|| m_pdLatestArrivals_vei == NULL
		|| m_ppdPheromoneMatrix_time == NULL
		|| m_pbNodesVisited_time == NULL
		|| m_pdProbability_time == NULL
		|| m_ppiTourMatrix_time == NULL
		|| m_ppiTourMatrix_newbest_time == NULL
		|| m_piCustomersToVisit_time == NULL
		|| m_pdLatestArrivals_time == NULL)
	{
		cleanup();
		return -6;
	}

	// copy initial solution to TourMatrix_bestsofar
	piTours = m_pInstanceData->getSolutionTours(&m_iVehicleCount_bestsofar,
												&m_dDistance_bestsofar);

	piNext = piTours;

	for (i=0; i<m_iVehicleCount_bestsofar; i++)
	{
		j = 0;

		do
		{
			m_ppiTourMatrix_bestsofar[i][++j] = *piNext;	
			piNext++;
		}
		while (*piNext != -1);

		piNext++;

		// customer count
		m_ppiTourMatrix_bestsofar[i][0] = j;

		// starting depot
		m_ppiTourMatrix_bestsofar[i][m_iToursMaxSize] = m_iCustomerCount+i;
	}

	// solution logger
	if (m_pSolutionLogger != NULL)
		m_pSolutionLogger->add(m_iVehicleCount_bestsofar, m_dDistance_bestsofar,
							   m_ppiTourMatrix_bestsofar);

	// calc cancel time
	tsCancel.tv_sec = time(NULL) + iCalcSeconds;
	tsCancel.tv_nsec = 0;

	// pthread control
	pthread_setconcurrency(3); 
	pthreadSelfID = pthread_self();

#if defined(WIN32) || defined(WIN64)
	schedParamMain.sched_priority = sched_get_priority_max(SCHED_OTHER);
	schedParamVei.sched_priority = schedParamMain.sched_priority-1;
	schedParamTime.sched_priority = schedParamMain.sched_priority-1;

	pthread_setschedparam(pthreadSelfID, SCHED_OTHER, &schedParamMain);
#else
	schedParamMain.sched_priority = 41;
	schedParamVei.sched_priority = 40;
	schedParamTime.sched_priority = 40;
	// bug in linux kernel (e.g. 2.6.18-3-amd64)
	// SCHED_RR does not work, no switch !!
//!!!	pthread_setschedparam(pthreadSelfID, SCHED_RR, &schedParamMain);
#endif

	do
	{
		bAcsVeiRunning = bAcsTimeRunning = false;
		m_bStopRunning = false;

		pthread_mutex_lock(&m_mutexBetterSolution);

		if (m_iVehicleCount_bestsofar > 1)
		{
			// create ACS_VEI thread
			if (pthread_create(&pthreadAcsVeiID, NULL, acs_vei, (void*)this) != 0)
			{
				pthread_mutex_unlock(&m_mutexBetterSolution);
				bError = true;
				break;
			}

#if defined(WIN32) || defined(WIN64)
			pthread_setschedparam(pthreadAcsVeiID, SCHED_OTHER, &schedParamVei);
#else
			// bug in linux kernel (e.g. 2.6.18-3-amd64)
			// SCHED_RR does not work, no switch !!
//!!!		pthread_setschedparam(pthreadAcsVeiID, SCHED_RR, &schedParamVei); 
#endif

			bAcsVeiRunning = true;
		}

		// create ACS_TIME thread
		if (pthread_create(&pthreadAcsTimeID, NULL, acs_time, (void*)this) != 0)
		{
			pthread_mutex_unlock(&m_mutexBetterSolution);
			bError = true;
			break;

#if defined(WIN32) || defined(WIN64)
			pthread_setschedparam(pthreadAcsTimeID, SCHED_OTHER, &schedParamTime);
#else
			// bug in linux kernel (e.g. 2.6.18-3-amd64)
			// SCHED_RR does not work, no switch !!
//!!!		pthread_setschedparam(pthreadAcsTimeID, SCHED_RR, &schedParamTime);
#endif
		}

		bAcsTimeRunning = true;

		// wait for a feasible solution with lower vehicle count
		if (pthread_cond_timedwait(&m_condBetterSolution, &m_mutexBetterSolution,
								   &tsCancel) != 0)
		{
			m_bStopRunning = true;
			pthread_mutex_unlock(&m_mutexBetterSolution);
			break; // ETIMEDOUT
		}

		m_bStopRunning = true;
		pthread_mutex_unlock(&m_mutexBetterSolution);

		// wait for exit of thread ACS_VEI
		if (bAcsVeiRunning)
			pthread_join(pthreadAcsVeiID, &pThreadReturn);

		// wait for exit of thread ACS_TIME
		if (bAcsTimeRunning)
			pthread_join(pthreadAcsTimeID, &pThreadReturn);
	}
	while (true);

	// wait for exit of thread ACS_VEI
	if (bAcsVeiRunning)
		pthread_join(pthreadAcsVeiID, &pThreadReturn);

	// wait for exit of thread ACS_TIME
	if (bAcsTimeRunning)
		pthread_join(pthreadAcsTimeID, &pThreadReturn);

	if (bError)
	{
		cleanup();
		return -7;
	}

	// convert TourMatrix to piTours
	piNext = piTours;

	for (i=0; i<m_iVehicleCount_bestsofar; i++)
	{
		iCount = m_ppiTourMatrix_bestsofar[i][0];

		for (j=1; j<=iCount; j++)
		{
			*piNext = m_ppiTourMatrix_bestsofar[i][j];
			piNext++;
		}

		*piNext = -1;
		piNext++;
	}

	m_pInstanceData->setSolutionVehicleCount(m_iVehicleCount_bestsofar);
	m_pInstanceData->setSolutionDistance(m_dDistance_bestsofar);

	cleanup();

	return 0;
}

void *VrptwMACS::acs_vei(void *pArg)
{
	bool bBetterSolutionFound;
	int i, j, iM, iN, iNodes, iAnt, iAntsCount, iToursMaxSize;
	int iCount, iCustomerCount, iNextNode, iLastNode, iVehicleCount_special;
	int iVisitedCustomers, iVisitedCustomers_acsvei, iToursVehicleCount;
	double dTau0, dToursDistance, dDistance_acsvei, dTmp1, dTmp2;
	bool *pbNodesVisited_vei;
	int *piIN_vei, *piNext;
	int **ppiTourMatrix, **ppiTourMatrix_acsvei, **ppiTourMatrix_bestsofar;
	VrptwMACS *pMACS;
	InstanceData *pInstanceData;
	double **ppdPheromoneMatrix;

	pMACS = (VrptwMACS *)pArg;

	pInstanceData = pMACS->m_pInstanceData;
	pbNodesVisited_vei = pMACS->m_pbNodesVisited_vei;
	piIN_vei = pMACS->m_piIN_vei;
	ppdPheromoneMatrix = pMACS->m_ppdPheromoneMatrix_vei;
	ppiTourMatrix_bestsofar = pMACS->m_ppiTourMatrix_bestsofar;
	ppiTourMatrix = pMACS->m_ppiTourMatrix_vei;
	ppiTourMatrix_acsvei = pMACS->m_ppiTourMatrix_acsvei;

	iCustomerCount = pMACS->m_iCustomerCount;
	iAntsCount = pMACS->m_iAntsCount;
	iToursMaxSize = pMACS->m_iToursMaxSize;

	// read shared resource
	pthread_rwlock_rdlock(&pMACS->m_rwlockBestSoFar);
	iVehicleCount_special = pMACS->m_iVehicleCount_bestsofar-1;
	pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);

	// initialize
	iNodes = iCustomerCount+iVehicleCount_special;
	
	dTau0 = 1.0;
	dTau0 /= iNodes * pInstanceData->getSolutionDistance();

	for (iM=0; iM<iNodes; iM++)
	{
		for (iN=0; iN<iNodes; iN++)
			ppdPheromoneMatrix[iM][iN] = dTau0;
	}

	for (i=0; i<iCustomerCount; i++)
		piIN_vei[i] = 0;

	bBetterSolutionFound = false;

	// copy initial solution to TourMatrix_acsvei
	iVisitedCustomers_acsvei = 0;
	dDistance_acsvei = pInstanceData->getSolutionDistance();

	piNext = pInstanceData->getSolutionTours(NULL, NULL);

	for (i=0; i<iVehicleCount_special; i++)
	{
		j = 0;

		do
		{
			iVisitedCustomers_acsvei++;
			ppiTourMatrix_acsvei[i][++j] = *piNext;	
			piNext++;
		}
		while (*piNext != -1);

		piNext++;

		// customer count
		ppiTourMatrix_acsvei[i][0] = j;

		// starting depot
		ppiTourMatrix_acsvei[i][iToursMaxSize] = iCustomerCount+i;
	}

	do
	{
		for (iAnt=0; iAnt<iAntsCount; iAnt++)
		{
			if (pMACS->m_bStopRunning)
				return NULL;

			// contruct a solution
			while (pMACS->new_active_ant(true, iNodes, dTau0, iVehicleCount_special,
										 &iToursVehicleCount, &dToursDistance))
			{
				// feasible solution found
				// -> save the solution and inform the main process

				// lock shared resource (write)
				pthread_rwlock_wrlock(&pMACS->m_rwlockBestSoFar);

				if (iToursVehicleCount >= pMACS->m_iVehicleCount_bestsofar)
				{
					// unlock shared resource (write)
					pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);

					return NULL;
				}

				pMACS->m_iVehicleCount_bestsofar = iToursVehicleCount;
				pMACS->m_dDistance_bestsofar = dToursDistance;
				CopyTourMatrix(iToursVehicleCount, iToursMaxSize,
							   pMACS->m_ppiTourMatrix_bestsofar, ppiTourMatrix);

				// solution logger
				if (pMACS->m_pSolutionLogger != NULL)
				{
					pMACS->m_pSolutionLogger->add(iToursVehicleCount,
												  dToursDistance, ppiTourMatrix);
				}

				// unlock shared resource (write)
				pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);

				// inform the main process
				pthread_mutex_lock(&pMACS->m_mutexBetterSolution);
				pthread_cond_signal(&pMACS->m_condBetterSolution);
				pthread_mutex_unlock(&pMACS->m_mutexBetterSolution);

				return NULL;
			}

			if (pMACS->m_bStopRunning)
				return NULL;

			// increment IN_vei for every not visited customers
			iVisitedCustomers = 0;

			for (i=0; i<iCustomerCount; i++)
			{
				if (pbNodesVisited_vei[i] == false)
					piIN_vei[i]++;
				else
					iVisitedCustomers++;
			}

			// better solution found?
			if (iVisitedCustomers > iVisitedCustomers_acsvei)
			{
				bBetterSolutionFound = true;
				iVisitedCustomers_acsvei = iVisitedCustomers;
				dDistance_acsvei = dToursDistance;
				CopyTourMatrix(iVehicleCount_special, iToursMaxSize,
							   ppiTourMatrix_acsvei, ppiTourMatrix);
			}
		}

		if (bBetterSolutionFound)
		{
			for (i=0; i<iCustomerCount; i++)
				piIN_vei[i] = 0;

			bBetterSolutionFound = false;
		}

		// perform global updating with TourMatrix_acsvei
		dTmp1 = 1.0-pMACS->m_dRho;
		dTmp2 = pMACS->m_dRho/dDistance_acsvei;

		iLastNode = ppiTourMatrix_acsvei[0][iToursMaxSize];

		for (i=0; i<iVehicleCount_special; i++)
		{
			// from depot to customers
			iCount = ppiTourMatrix_acsvei[i][0];

			for (j=1; j<=iCount; j++)
			{
				iNextNode = ppiTourMatrix_acsvei[i][j];
				ppdPheromoneMatrix[iLastNode][iNextNode] *= dTmp1;
				ppdPheromoneMatrix[iLastNode][iNextNode] += dTmp2;
				iLastNode = iNextNode;
			}

			// back to depot
			if (i < iVehicleCount_special-1)
				iNextNode = ppiTourMatrix_acsvei[i+1][iToursMaxSize];
			else
				iNextNode = ppiTourMatrix_acsvei[0][iToursMaxSize];

			ppdPheromoneMatrix[iLastNode][iNextNode] *= dTmp1;
			ppdPheromoneMatrix[iLastNode][iNextNode] += dTmp2;
			iLastNode = iNextNode;
		}

		if (pMACS->m_bStopRunning)
			return NULL;

		// lock shared resource (read)
		pthread_rwlock_rdlock(&pMACS->m_rwlockBestSoFar);

		// perform global updating with TourMatrix_bestsofar
		dTmp1 = 1.0-pMACS->m_dRho;
		dTmp2 = pMACS->m_dRho/pMACS->m_dDistance_bestsofar;

		iLastNode = ppiTourMatrix_bestsofar[0][iToursMaxSize];
		iToursVehicleCount = pMACS->m_iVehicleCount_bestsofar;

		for (i=0; i<iToursVehicleCount; i++)
		{
			// from depot to customers
			iCount = ppiTourMatrix_bestsofar[i][0];

			for (j=1; j<=iCount; j++)
			{
				iNextNode = ppiTourMatrix_bestsofar[i][j];
				ppdPheromoneMatrix[iLastNode][iNextNode] *= dTmp1;
				ppdPheromoneMatrix[iLastNode][iNextNode] += dTmp2;
				iLastNode = iNextNode;
			}

			// back to depot
			if (i < iToursVehicleCount-1)
				iNextNode = ppiTourMatrix_bestsofar[i+1][iToursMaxSize];
			else
				iNextNode = ppiTourMatrix_bestsofar[0][iToursMaxSize];

			ppdPheromoneMatrix[iLastNode][iNextNode] *= dTmp1;
			ppdPheromoneMatrix[iLastNode][iNextNode] += dTmp2;
			iLastNode = iNextNode;
		}

		// unlock shared resource
		pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);
	}
	while (pMACS->m_bStopRunning == false);

	return NULL;
}

void *VrptwMACS::acs_time(void *pArg)
{
	bool bNoData;
	int i, j, iCount, iLastNode, iNextNode;
	int iM, iN, iNodes, iAnt, iVehicleCount_bestsofar, iToursVehicleCount;
	int iCustomerCount, iAntsCount, iToursMaxSize;
	double dTmp1, dTmp2;
	double dTau0, dDistance_newbest, dToursDistance, dDistance_bestsofar;
	int **ppiTourMatrix, **ppiTourMatrix_newbest, **ppiTourMatrix_bestsofar;
	VrptwMACS *pMACS;
	InstanceData *pInstanceData;
	double **ppdPheromoneMatrix;

	pMACS = (VrptwMACS *)pArg;

	pInstanceData = pMACS->m_pInstanceData;
	ppdPheromoneMatrix = pMACS->m_ppdPheromoneMatrix_time;
	ppiTourMatrix_bestsofar = pMACS->m_ppiTourMatrix_bestsofar;
	ppiTourMatrix = pMACS->m_ppiTourMatrix_time;
	ppiTourMatrix_newbest = pMACS->m_ppiTourMatrix_newbest_time;

	iCustomerCount = pMACS->m_iCustomerCount;
	iAntsCount = pMACS->m_iAntsCount;
	iToursMaxSize = pMACS->m_iToursMaxSize;
	
	// read shared resource
	pthread_rwlock_rdlock(&pMACS->m_rwlockBestSoFar);
	iVehicleCount_bestsofar = pMACS->m_iVehicleCount_bestsofar;
	iVehicleCount_bestsofar = pMACS->m_iVehicleCount_bestsofar;
	pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);

	// initialize
	iNodes = iCustomerCount+iVehicleCount_bestsofar;
	
	dTau0 = 1.0;
	dTau0 /= iNodes * pInstanceData->getSolutionDistance();

	for (iM=0; iM<iNodes; iM++)
	{
		for (iN=0; iN<iNodes; iN++)
			ppdPheromoneMatrix[iM][iN] = dTau0;
	}

	do
	{
		bNoData = true;

		for (iAnt=0; iAnt<iAntsCount; iAnt++)
		{
			if (pMACS->m_bStopRunning)
				return NULL;

			// contruct a solution
			if (pMACS->new_active_ant(false, iNodes, dTau0, iVehicleCount_bestsofar,
									  &iToursVehicleCount, &dToursDistance)
									  == false)
			{
				continue; // not feasible
			}
			
			if (pMACS->m_bStopRunning)
				return NULL;

			while (iToursVehicleCount < iVehicleCount_bestsofar)
			{
				// lock shared resource (write)
				pthread_rwlock_wrlock(&pMACS->m_rwlockBestSoFar);

				if (iToursVehicleCount >= pMACS->m_iVehicleCount_bestsofar)
				{
					// unlock shared resource
					pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);
					return NULL;
				}

				pMACS->m_iVehicleCount_bestsofar = iToursVehicleCount;
				pMACS->m_dDistance_bestsofar = dToursDistance;
				CopyTourMatrix(iToursVehicleCount, iToursMaxSize,
							   ppiTourMatrix_bestsofar, ppiTourMatrix);

				// solution logger
				if (pMACS->m_pSolutionLogger != NULL)
					pMACS->m_pSolutionLogger->add(iToursVehicleCount,
												  dToursDistance, ppiTourMatrix);

				// unlock shared resource
				pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);

				// inform the main process
				pthread_mutex_lock(&pMACS->m_mutexBetterSolution);
				pthread_cond_signal(&pMACS->m_condBetterSolution);
				pthread_mutex_unlock(&pMACS->m_mutexBetterSolution);
				return NULL;
			}

			if (bNoData)
			{
				bNoData = false;
				dDistance_newbest = dToursDistance;
				CopyTourMatrix(iToursVehicleCount, iToursMaxSize,
							   ppiTourMatrix_newbest, ppiTourMatrix);
				continue;
			}

			if (dToursDistance < dDistance_newbest)
			{
				dDistance_newbest = dToursDistance;
				CopyTourMatrix(iToursVehicleCount, iToursMaxSize,
							   ppiTourMatrix_newbest, ppiTourMatrix);
			}
		}

		if (pMACS->m_bStopRunning)
			return NULL;

		// read shared resource (read)
		pthread_rwlock_rdlock(&pMACS->m_rwlockBestSoFar);
		dDistance_bestsofar = pMACS->m_dDistance_bestsofar;
		pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);

		if (bNoData == false)
		{
			if (dDistance_newbest < dDistance_bestsofar)
			{
				// lock shared resource (write)
				pthread_rwlock_wrlock(&pMACS->m_rwlockBestSoFar);

				if (iVehicleCount_bestsofar == pMACS->m_iVehicleCount_bestsofar)
				{
					pMACS->m_dDistance_bestsofar = dDistance_newbest;
					CopyTourMatrix(iVehicleCount_bestsofar, iToursMaxSize,
								   ppiTourMatrix_bestsofar, ppiTourMatrix_newbest);

					// solution logger
					if (pMACS->m_pSolutionLogger != NULL)
					{
						pMACS->m_pSolutionLogger->add(iVehicleCount_bestsofar,
													  dDistance_newbest,
													  ppiTourMatrix_newbest);
					}
				}

				// unlock shared resource (write)
				pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);
			}
		}

		// perform global updating
		dTmp1 = 1.0-pMACS->m_dRho;
		dTmp2 = pMACS->m_dRho/dDistance_bestsofar;

		// lock shared resource (read)
		pthread_rwlock_rdlock(&pMACS->m_rwlockBestSoFar);

		iLastNode = ppiTourMatrix_bestsofar[0][iToursMaxSize];
		iToursVehicleCount = pMACS->m_iVehicleCount_bestsofar;

		for (i=0; i<iToursVehicleCount; i++)
		{
			// from depot to customers
			iCount = ppiTourMatrix_bestsofar[i][0];

			for (j=1; j<=iCount; j++)
			{
				iNextNode = ppiTourMatrix_bestsofar[i][j];
				ppdPheromoneMatrix[iLastNode][iNextNode] *= dTmp1;
				ppdPheromoneMatrix[iLastNode][iNextNode] += dTmp2;
				iLastNode = iNextNode;
			}

			// back to depot
			if (i < iToursVehicleCount-1)
				iNextNode = ppiTourMatrix_bestsofar[i+1][iToursMaxSize];
			else
				iNextNode = ppiTourMatrix_bestsofar[0][iToursMaxSize];

			ppdPheromoneMatrix[iLastNode][iNextNode] *= dTmp1;
			ppdPheromoneMatrix[iLastNode][iNextNode] += dTmp2;
			iLastNode = iNextNode;
		}

		// unlock shared resource
		pthread_rwlock_unlock(&pMACS->m_rwlockBestSoFar);
	}
	while (pMACS->m_bStopRunning == false);

	return NULL;
}

bool VrptwMACS::new_active_ant(bool bVEI,
							   int iNodes,
							   double dTau0,
							   int iMaxVehicleCount,
							   int *piToursVehicleCount,
							   double *pdToursDistance)
{
	short nBeta;
	int i, iCustomer, iCapacity, iMaxCapacity;
	int iLastNode, iNextNode, iToursVehicleCount;
	double dTime, dDistance, dToursDistance, dTemp, dEta, dProbabilitySum;
	bool *pbNodesVisited;
	double *pdProbability;
	double **ppdDistanceMatrix;
	double **ppdPheromoneMatrix;
	int **ppiTourMatrix;
	MTRand *pMTRand;

	// init vars
	ppdDistanceMatrix = m_pInstanceData->getDistanceMatrix();

	if (bVEI)
	{
		pMTRand = &m_MTRand_vei;
		ppdPheromoneMatrix = m_ppdPheromoneMatrix_vei;
		pbNodesVisited = m_pbNodesVisited_vei;
		pdProbability = m_pdProbability_vei;
		ppiTourMatrix = m_ppiTourMatrix_vei;
	}
	else
	{
		pMTRand = &m_MTRand_time;
		ppdPheromoneMatrix = m_ppdPheromoneMatrix_time;
		pbNodesVisited = m_pbNodesVisited_time;
		pdProbability = m_pdProbability_time;
		ppiTourMatrix = m_ppiTourMatrix_time;
	}

	for (i=0; i<iNodes; i++)
		pbNodesVisited[i] = false;

	// put ant in a randomly selected duplicated depot
	iLastNode = m_iCustomerCount + pMTRand->randInt(iMaxVehicleCount-1);

	// save start depot
	ppiTourMatrix[0][m_iToursMaxSize] = iLastNode;
	iCustomer = 0;
	dTime = 0.0;
	iCapacity = iMaxCapacity = m_pInstanceData->getCapacity();

	iToursVehicleCount = 0;
	dToursDistance = 0.0;

	do
	{
		dProbabilitySum = 0.0;

		for (i=0; i<iNodes; i++)
		{
			if (m_bStopRunning)
				return false;

			pdProbability[i] = 0.0;

			// is served?
			if (pbNodesVisited[i] == true)
				continue;

			if (i < m_iCustomerCount) // customer
			{
				// check capacity
				if (m_piCustomerDemand[i] > iCapacity)
					continue;

				// check due date
				if (iLastNode > m_iCustomerCount)
					dDistance = ppdDistanceMatrix[m_iCustomerCount][i];
				else
					dDistance = ppdDistanceMatrix[iLastNode][i];

				dTemp = dTime + dDistance;

				if (dTemp > m_piCustomerDueDate[i])
					continue;

				// check depot due time
				if (dTemp < m_piCustomerReadyTime[i])
					dTemp = m_piCustomerReadyTime[i];

				dTemp += m_piCustomerServiceTime[i];
				dTemp += ppdDistanceMatrix[i][m_iCustomerCount];

				if (dTemp > m_iDepotDueDate)
					continue;

				// calculate attractiveness
				dTemp = __max(dTime + dDistance, m_piCustomerReadyTime[i]);
				dTemp -= dTime;
				dTemp *= (m_piCustomerDueDate[i]-dTime);

				if (bVEI)
					dTemp = __max(1.0, dTemp-m_piIN_vei[i]);
				else
					dTemp = __max(1.0, dTemp);

				dEta = 1.0 / dTemp;
			}
			else // duplicated depot
			{
				if (iLastNode >= m_iCustomerCount)
					continue;

				dDistance = ppdDistanceMatrix[iLastNode][m_iCustomerCount];

				// calculate attractiveness
				dEta = 1.0;
				dEta /= dDistance * (m_iDepotDueDate-dTime);
			}

			pdProbability[i] = ppdPheromoneMatrix[iLastNode][i];

			for (nBeta=0; nBeta<m_nBeta; nBeta++)
				pdProbability[i] *= dEta;

			dProbabilitySum += pdProbability[i];
		}

		// exploitation or exploration?
		do
		{
			// exploitation
			if (m_dQ0 > 0.0)
			{
				// exploitation
				if (pMTRand->rand() < m_dQ0)
				{
					dTemp = 0.0;

					for (i=0; i<iNodes; i++)
					{
						if (pdProbability[i] > dTemp)
						{
							dTemp = pdProbability[i];
							iNextNode = i;
						}
					}

					break;
				}
			}

			// exploration
			dTemp = pMTRand->rand() * dProbabilitySum;

			iNextNode = 0;
			dProbabilitySum = pdProbability[0];

			while (dProbabilitySum < dTemp)
				dProbabilitySum += pdProbability[++iNextNode];

			if (dTemp == 0.0)
			{
				while (dProbabilitySum < 0.0)
					dProbabilitySum += pdProbability[++iNextNode];
			}

			break;
		}
		while (false);

		// add node
		pbNodesVisited[iNextNode] = true;

		// local pheromone update
		ppdPheromoneMatrix[iLastNode][iNextNode] *= 1.0-m_dXi;
		ppdPheromoneMatrix[iLastNode][iNextNode] += m_dXi*dTau0;

		if (iNextNode < m_iCustomerCount) // customer
		{
			if (iLastNode > m_iCustomerCount)
				dDistance = ppdDistanceMatrix[m_iCustomerCount][iNextNode];
			else
				dDistance = ppdDistanceMatrix[iLastNode][iNextNode];

			dToursDistance += dDistance;
			dTime += dDistance;

			if (dTime < m_piCustomerReadyTime[iNextNode])
				dTime = m_piCustomerReadyTime[iNextNode];

			dTime += m_piCustomerServiceTime[iNextNode];
			iCapacity -= m_piCustomerDemand[iNextNode];

			ppiTourMatrix[iToursVehicleCount][++iCustomer] = iNextNode;
		}
		else // duplicated depot
		{
			if (iLastNode >= m_iCustomerCount)
				dDistance = 0.0;
			else
				dDistance = ppdDistanceMatrix[iLastNode][m_iCustomerCount];

			dToursDistance += dDistance;

			ppiTourMatrix[iToursVehicleCount][0] = iCustomer;
			iToursVehicleCount++;

			// any customer not visited yet?
			for (i=0; i<m_iCustomerCount; i++)
			{
				if (pbNodesVisited[i] == false)
					break;
			}

			if (i == m_iCustomerCount)
				break; // all customers visited

			if (iToursVehicleCount < iMaxVehicleCount)
			{
				// next tour
				ppiTourMatrix[iToursVehicleCount][m_iToursMaxSize] = iNextNode;
				iCustomer = 0;
				dTime = 0.0;
				iCapacity = iMaxCapacity;	
			}
			else
				break; // no more vehicles
		}

		iLastNode = iNextNode;
	}
	while (true);

	*piToursVehicleCount = iToursVehicleCount;
	*pdToursDistance = dToursDistance;

	// tentatively insert non visited customers
	if (insertion_procedure(bVEI, pbNodesVisited, iToursVehicleCount,
							ppiTourMatrix, pdToursDistance) == false)
	{
		return false; // not feasable
	}
	
	if (bVEI)
		return true;

	// local search
	if (ls_intra_exchange_matrix(iToursVehicleCount, pdToursDistance,
								 ppiTourMatrix, &m_bStopRunning) != 0)
	{
		return false;
	}
							 
	if (ls_cross_exchange_matrix(iToursVehicleCount, pdToursDistance,
								 ppiTourMatrix, &m_bStopRunning) != 0)
	{
		return false;	
	}

	return true;
}

bool VrptwMACS::insertion_procedure(bool bVEI,
									bool *pbNodesVisited,
									int iVehicleCount,
									int **ppiTourMatrix,
									double *pdTourDistance)
{
	bool bRestart;
	int i, j, iCustomer, iRoute, iCustomersToInsert;
	int iCapacity, iMaxCapacity, iDepotDueDate;
	int iLastCustomer, iNextCustomer;
	double dTime, dEarliestStart, dLatestArrival, dTourDistance;
	int *piCustomerReadyTime, *piCustomerDueDate, *piCustomerServiceTime;
	int *piCustomersToVisit;
	double *pdLatestArrivals;
	double **ppdDistanceMatrix;

	// init vars
	dTourDistance = *pdTourDistance;

	piCustomerReadyTime = m_pInstanceData->getCustomerReadyTime();
	piCustomerDueDate = m_pInstanceData->getCustomerDueDate();
	piCustomerServiceTime = m_pInstanceData->getCustomerServiceTime();
	ppdDistanceMatrix = m_pInstanceData->getDistanceMatrix();

	iDepotDueDate = m_pInstanceData->getDepotDueDate();
	iCapacity = iMaxCapacity = m_pInstanceData->getCapacity();

	if (bVEI)
	{
		piCustomersToVisit = m_piCustomersToVisit_vei;
		pdLatestArrivals = m_pdLatestArrivals_vei;
	}
	else
	{
		piCustomersToVisit = m_piCustomersToVisit_time;
		pdLatestArrivals = m_pdLatestArrivals_time;
	}

	j = 0;

	for (i=0; i<m_iCustomerCount; i++)
	{
		if (pbNodesVisited[i] == false)
			piCustomersToVisit[++j] = i;
	}

	iCustomersToInsert = piCustomersToVisit[0] = j;

	if (iCustomersToInsert == 0)
		return true; // all customers inserted

	// sort array by customer demand
	sortCustomersByDemand(piCustomersToVisit);

	for (iRoute=0; iRoute<iVehicleCount; iRoute++)
	{
		bRestart = false;

		// check capacity
		iCapacity = iMaxCapacity;

		for (i=1; i<=ppiTourMatrix[iRoute][0]; i++)
			iCapacity -= m_piCustomerDemand[ppiTourMatrix[iRoute][i]];

		// comp latest arrival times
		dLatestArrival = iDepotDueDate;
		iLastCustomer = m_iCustomerCount; // depot

		for (i=ppiTourMatrix[iRoute][0]; i>0; i--)
		{
			iNextCustomer = ppiTourMatrix[iRoute][i];
			dLatestArrival -= ppdDistanceMatrix[iLastCustomer][iNextCustomer];

			dLatestArrival =
				__min(dLatestArrival-piCustomerServiceTime[iNextCustomer],
					  piCustomerDueDate[iNextCustomer]);
								   
			pdLatestArrivals[i] = dLatestArrival;
			iLastCustomer = iNextCustomer;
		}

		// check all gaps, starting with the first one (depot -> customer1)
		dEarliestStart = 0.0;

		iLastCustomer = m_iCustomerCount;	// depot

		for (i=1; i<=ppiTourMatrix[iRoute][0]+1; i++)
		{
			if (i == ppiTourMatrix[iRoute][0]+1) // depot
			{
				iNextCustomer = m_iCustomerCount;
				dLatestArrival = iDepotDueDate;
			}
			else
			{
				iNextCustomer = ppiTourMatrix[iRoute][i];
				dLatestArrival = pdLatestArrivals[i];
			}

			// try to insert one of the remaining customers
			for (j=1; j<=iCustomersToInsert; j++)
			{
				if (m_bStopRunning)
					return false;

				iCustomer = piCustomersToVisit[j];

				if (m_piCustomerDemand[iCustomer] > iCapacity)
					continue;

				dTime = dEarliestStart;
				dTime += ppdDistanceMatrix[iLastCustomer][iCustomer];

				if (dTime > piCustomerDueDate[iCustomer])
					continue;

				if (dTime < piCustomerReadyTime[iCustomer])
					dTime = piCustomerReadyTime[iCustomer];

				dTime += piCustomerServiceTime[iCustomer];
				dTime += ppdDistanceMatrix[iCustomer][iNextCustomer];

				if (dTime > dLatestArrival)
					continue;

				// customer fits into tour, insert it
				insertCustomerIntoTourMatrix(i, iCustomer, ppiTourMatrix[iRoute]);

				pbNodesVisited[iCustomer] = true;
				dTourDistance += ppdDistanceMatrix[iLastCustomer][iCustomer];
				dTourDistance += ppdDistanceMatrix[iCustomer][iNextCustomer];

				// remove inserted customer from list
				removeCustomerFromList(j, piCustomersToVisit);

				if (--iCustomersToInsert == 0)
				{
					*pdTourDistance = dTourDistance;
					return true; // all customers inserted
				}

				// restart
				bRestart = true;
				iRoute--;
				break;
			}

			if (bRestart)
				break;

			// comp next gap
			dEarliestStart += ppdDistanceMatrix[iLastCustomer][iNextCustomer];

			if (dEarliestStart < piCustomerReadyTime[iNextCustomer])
				dEarliestStart = piCustomerReadyTime[iNextCustomer];

			dEarliestStart += piCustomerServiceTime[iNextCustomer];

			iLastCustomer = iNextCustomer;
		}
	}

	return false; // not feasible
}

void VrptwMACS::insertCustomerIntoTourMatrix(int iPos,
											 int iCustomer,
											 int *piTourMatrix)
{
	int i;

	for (i=piTourMatrix[0]; i>=iPos; i--)
		piTourMatrix[i+1] = piTourMatrix[i];

	piTourMatrix[i+1] = iCustomer;
	piTourMatrix[0]++;
}

void VrptwMACS::removeCustomerFromList(int iPos,
									   int *piList)
{
	int i;

	for (i=iPos; i<piList[0]; i++)
		piList[i] = piList[i+1];

	piList[0]--;
}

void VrptwMACS::sortCustomersByDemand(int *piArray)
{
	bool bSwapped;
	int i, iTmp, iMax;

	iMax = piArray[0];

	do
	{
		bSwapped = false;

		for (i=1; i<iMax; i++)
		{
			if (m_piCustomerDemand[piArray[i]] < m_piCustomerDemand[piArray[i+1]])
			{
				iTmp = piArray[i];
				piArray[i] = piArray[i+1];
				piArray[i+1] = iTmp;
				bSwapped = true;
			}
		}

		iMax--;
	}
	while (bSwapped);
}

void VrptwMACS::CopyTourMatrix(int iVehicleCount,
							   int iToursMaxSize,
							   int **ppiDest,
							   int **ppiSrc)
{
	int i;

	for (i=0; i<iVehicleCount; i++)
	{
		IntCopy(ppiDest[i], ppiSrc[i], ppiSrc[i][0]+1); // customers
		ppiDest[i][iToursMaxSize] = ppiSrc[i][iToursMaxSize]; // depot number
	}
}
