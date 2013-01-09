//
// Vrptw.cpp
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


///// includes ////

#include "Vrptw.h"
#include "InstanceData.h"
#include "utils.h"
#include <stdlib.h>
#include <math.h>


///// classes //////

Vrptw::Vrptw()
{
	m_pInstanceData = NULL;
}

Vrptw::Vrptw(InstanceData *pInstanceData)
{
	m_pInstanceData = pInstanceData;
}

Vrptw::~Vrptw()
{
}

void Vrptw::applyInstanceData(InstanceData *pInstanceData)
{
	m_pInstanceData = pInstanceData;
}

// Nearest Neighbor heuristic by Solomon
// "ALGORITHMS FOR THE VEHICLE ROUTING AND SCHEDULING PROBLEMS
// WITH TIME WINDOW CONTRAINTS" (1987)
//
int Vrptw::nn_solomon1987(double dW1,
						  double dW2,
						  double dW3)
{
	int i, iDepotDueDate, iVehicleCount, iCapacity, iCustomerCount;
	int iLastCustomer, iNextCustomer;
	double dTmp, dCosts, dTime, dDistance, dTotalDistance, dMinCosts;
	bool *pbCustomerVisited;
	int *piCustomerDemand, *piCustomerReadyTime, *piCustomerDueDate;
	int *piCustomerServiceTime, *piSolutionTours, *piNextCustomer;

	// cleanup existing solution
	m_pInstanceData->setSolution(0, 0.0, NULL);

	// check params
	if (m_pInstanceData == NULL
		|| dW1 < 0.0 
		|| dW2 < 0.0 
		|| dW3 < 0.0)
	{
		return -1;
	}

	// init vars
	dTmp = dW1 + dW2 + dW3;

	if (dTmp != 1.0)
	{
		dW1 /= dTmp;
		dW2 /= dTmp;
		dW3 /= dTmp;
	}

	iDepotDueDate = m_pInstanceData->getDepotDueDate();
	iCustomerCount = m_pInstanceData->getCustomerCount();
	piCustomerDemand = m_pInstanceData->getCustomerDemand();
	piCustomerReadyTime = m_pInstanceData->getCustomerReadyTime();
	piCustomerDueDate = m_pInstanceData->getCustomerDueDate();
	piCustomerServiceTime = m_pInstanceData->getCustomerServiceTime();
	
	iVehicleCount = 0;
	dTotalDistance = 0.0;

	// malloc memory
	pbCustomerVisited = (bool*)malloc(sizeof(bool)*iCustomerCount);

	if (pbCustomerVisited == NULL)
		return -1;

	piSolutionTours = (int*)malloc(sizeof(int)
		* m_pInstanceData->getCustomerCount() * 2);

	if (piSolutionTours == NULL)
	{
		free(pbCustomerVisited);
		return -1;
	}

	piNextCustomer = piSolutionTours;

	for (i=0; i<iCustomerCount; i++)
		pbCustomerVisited[i] = false;
 
	do
	{
		if (iVehicleCount != 0)
		{
			// customer -> depot
			dTotalDistance += m_pInstanceData->getDepotDistance(iLastCustomer);

			*piNextCustomer = -1;
			piNextCustomer++;

			// all customers served?
			for (i=0; i<iCustomerCount; i++)
			{
				if (pbCustomerVisited[i] == false)
					break; // no
			}

			if (i == iCustomerCount)
				break; // yes
		}

		dTime = 0.0;
		iCapacity = m_pInstanceData->getCapacity();
		iVehicleCount++;
		iLastCustomer = iCustomerCount; // depot

		do
		{
			iNextCustomer = -1; // no customer

			for (i=0; i<iCustomerCount; i++)
			{
				// is served?
				if (pbCustomerVisited[i] == true)
					continue;

				// check capacity
				if (piCustomerDemand[i] > iCapacity)
					continue;

				// check due date
				dDistance = m_pInstanceData->getCustomerDistance(iLastCustomer, i);
				dTmp = dTime + dDistance;

				if (dTmp > piCustomerDueDate[i])
					continue;

				// check depot due time
				if (dTmp < piCustomerReadyTime[i])
					dTmp = piCustomerReadyTime[i];

				dTmp += piCustomerServiceTime[i];
				dTmp += m_pInstanceData->getDepotDistance(i);

				if (dTmp > iDepotDueDate)
					continue;

				// distance between last and next customers
				dCosts = dW1 * dDistance;

				// difference between the completion of service at last
				// customer and beginning of service at next customer
				dTmp = piCustomerReadyTime[i]; // bj
				dTmp -= dTime; // (bi+si)

				if (dTmp > 0.0)
					dCosts += dW2 * dTmp;

				// urgency of delivery to next customer
				dTmp = piCustomerDueDate[i]; // lj
				dTmp -= dTime; // (bi+si)
				dTmp -= dDistance; // tij
				dCosts += dW3 * dTmp;

				if (iNextCustomer == -1
					|| dCosts < dMinCosts)
				{
					iNextCustomer = i;
					dMinCosts = dCosts;
				}
			}

			if (iNextCustomer == -1)
				break;

			// add customer
			dDistance = m_pInstanceData->getCustomerDistance(iLastCustomer,
				iNextCustomer);
				
			dTotalDistance += dDistance;
			dTime += dDistance;

			if (dTime < piCustomerReadyTime[iNextCustomer])
				dTime = piCustomerReadyTime[iNextCustomer];

			dTime += piCustomerServiceTime[iNextCustomer];
			iCapacity -= piCustomerDemand[iNextCustomer];

			pbCustomerVisited[iNextCustomer] = true;
			*piNextCustomer = iNextCustomer;
			piNextCustomer++;

			iLastCustomer = iNextCustomer;

			if (iCapacity == 0)
				break;
		}
		while (true);
	}
	while (true);

	m_pInstanceData->setSolution(iVehicleCount, dTotalDistance, piSolutionTours);

	free(pbCustomerVisited);
	return 0;
}

// Nearest Neighbor heuristic by Gambardella et. al.
// "MACS-VRPTW: A MULTIPLE ANT COLONY SYSTEM FOR VEHICLE ROUTING PROBLEMS
// WITH TIME WINDOW" (1999)
//
int Vrptw::nn_gambardella1999()
{
	int i, iDepotDueDate, iVehicleCount, iCapacity, iCustomerCount;
	int iLastCustomer, iNextCustomer;
	double dTmp, dCosts, dTime, dDistance, dTotalDistance, dMinCosts;
	bool *pbCustomerVisited;
	int *piCustomerDemand, *piCustomerReadyTime, *piCustomerDueDate;
	int	*piCustomerServiceTime, *piSolutionTours, *piNextCustomer;

	// cleanup existing solution
	m_pInstanceData->setSolution(0, 0.0, NULL);

	// init vars
	iDepotDueDate = m_pInstanceData->getDepotDueDate();
	iCustomerCount = m_pInstanceData->getCustomerCount();
	piCustomerDemand = m_pInstanceData->getCustomerDemand();
	piCustomerReadyTime = m_pInstanceData->getCustomerReadyTime();
	piCustomerDueDate = m_pInstanceData->getCustomerDueDate();
	piCustomerServiceTime = m_pInstanceData->getCustomerServiceTime();
	
	iVehicleCount = 0;
	dTotalDistance = 0.0;

	// malloc memory
	pbCustomerVisited = (bool*)malloc(sizeof(bool)*iCustomerCount);

	if (pbCustomerVisited == NULL)
		return -1;

	piSolutionTours = (int*)malloc(sizeof(int)
		* m_pInstanceData->getCustomerCount()*2);

	if (piSolutionTours == NULL)
	{
		free(pbCustomerVisited);
		return -1;
	}

	piNextCustomer = piSolutionTours;

	for (i=0; i<iCustomerCount; i++)
		pbCustomerVisited[i] = false;
 
	do
	{
		if (iVehicleCount != 0)
		{
			// customer -> depot
			dTotalDistance += m_pInstanceData->getDepotDistance(iLastCustomer);

			*piNextCustomer = -1;
			piNextCustomer++;

			// all customers served?
			for (i=0; i<iCustomerCount; i++)
			{
				if (pbCustomerVisited[i] == false)
					break; // no
			}

			if (i == iCustomerCount)
				break; // yes
		}

		dTime = 0.0;
		iCapacity = m_pInstanceData->getCapacity();
		iVehicleCount++;
		iLastCustomer = iCustomerCount; // depot

		do
		{
			iNextCustomer = -1; // no customer

			for (i=0; i<iCustomerCount; i++)
			{
				// is served?
				if (pbCustomerVisited[i] == true)
					continue;

				// check capacity
				if (piCustomerDemand[i] > iCapacity)
					continue;

				// check due date
				dDistance = m_pInstanceData->getCustomerDistance(iLastCustomer, i);
				dTmp = dTime + dDistance;

				if (dTmp > piCustomerDueDate[i])
					continue;

				// check depot due time
				if (dTmp < piCustomerReadyTime[i])
					dTmp = piCustomerReadyTime[i];

				dTmp += piCustomerServiceTime[i];
				dTmp += m_pInstanceData->getDepotDistance(i);

				if (dTmp > iDepotDueDate)
					continue;

				// difference between the completion of service at last
				// customer and beginning of service at next customer
				dTmp = piCustomerReadyTime[i]; // bj
				dTmp -= dTime; // (bi+si)
				dTmp = __max(dTmp, dDistance);

				dCosts = dTmp;

				// urgency of delivery to next customer
				dTmp = piCustomerDueDate[i]; // lj
				dTmp -= dTime; // (bi+si)

				dCosts *= dTmp;

				if (iNextCustomer == -1
					|| dCosts < dMinCosts)
				{
					iNextCustomer = i;
					dMinCosts = dCosts;
				}
			}

			if (iNextCustomer == -1)
				break;

			// add customer
			dDistance = m_pInstanceData->getCustomerDistance(iLastCustomer,
				iNextCustomer);
				
			dTotalDistance += dDistance;
			dTime += dDistance;

			if (dTime < piCustomerReadyTime[iNextCustomer])
				dTime = piCustomerReadyTime[iNextCustomer];

			dTime += piCustomerServiceTime[iNextCustomer];
			iCapacity -= piCustomerDemand[iNextCustomer];

			pbCustomerVisited[iNextCustomer] = true;
			*piNextCustomer = iNextCustomer;
			piNextCustomer++;

			iLastCustomer = iNextCustomer;

			if (iCapacity == 0)
				break;
		}
		while (true);
	}
	while (true);

	m_pInstanceData->setSolution(iVehicleCount, dTotalDistance, piSolutionTours);

	free(pbCustomerVisited);
	return 0;
}

// Nearest Neighbor heuristic by Ellabib et. al.
// "An Experimental Study of a Simple Ant Colony System for the
// Vehicle Routing Problem with Time Windows" (2002)
//
int Vrptw::nn_ellabib2002(double dW1,
						  double dW2)
{
	int i, iDepotDueDate, iVehicleCount, iCapacity, iCustomerCount;
	int iLastCustomer, iNextCustomer;
	double dTmp, dCosts, dTime, dDistance, dTotalDistance, dMinCosts;
	bool *pbCustomerVisited;
	int *piCustomerDemand, *piCustomerReadyTime, *piCustomerDueDate;
	int *piCustomerServiceTime, *piSolutionTours, *piNextCustomer;
	int *piCustomerXCoord, *piCustomerYCoord;

	// cleanup existing solution
	m_pInstanceData->setSolution(0, 0.0, NULL);

	// check params
	if (m_pInstanceData == NULL
		|| dW1 < 0.0 
		|| dW2 < 0.0)
	{
		return -1;
	}

	// init vars
	dTmp = dW1 + dW2;

	if (dTmp != 1.0)
	{
		dW1 /= dTmp;
		dW2 /= dTmp;
	}

	iDepotDueDate = m_pInstanceData->getDepotDueDate();
	iCustomerCount = m_pInstanceData->getCustomerCount();
	piCustomerDemand = m_pInstanceData->getCustomerDemand();
	piCustomerReadyTime = m_pInstanceData->getCustomerReadyTime();
	piCustomerDueDate = m_pInstanceData->getCustomerDueDate();
	piCustomerServiceTime = m_pInstanceData->getCustomerServiceTime();
	piCustomerXCoord = m_pInstanceData->getCustomerXCoord();
	piCustomerYCoord = m_pInstanceData->getCustomerYCoord();

	iVehicleCount = 0;
	dTotalDistance = 0.0;

	// malloc memory
	pbCustomerVisited = (bool*)malloc(sizeof(bool)*iCustomerCount);

	if (pbCustomerVisited == NULL)
		return -1;

	piSolutionTours = (int*)malloc(sizeof(int)
		* m_pInstanceData->getCustomerCount() * 2);

	if (piSolutionTours == NULL)
	{
		free(pbCustomerVisited);
		return -1;
	}

	piNextCustomer = piSolutionTours;

	for (i=0; i<iCustomerCount; i++)
		pbCustomerVisited[i] = false;
 
	do
	{
		if (iVehicleCount != 0)
		{
			// customer -> depot
			dTotalDistance += m_pInstanceData->getDepotDistance(iLastCustomer);

			*piNextCustomer = -1;
			piNextCustomer++;

			// all customers served?
			for (i=0; i<iCustomerCount; i++)
			{
				if (pbCustomerVisited[i] == false)
					break; // no
			}

			if (i == iCustomerCount)
				break; // yes
		}

		dTime = 0.0;
		iCapacity = m_pInstanceData->getCapacity();
		iVehicleCount++;
		iLastCustomer = iCustomerCount; // depot

		do
		{
			iNextCustomer = -1; // no customer

			for (i=0; i<iCustomerCount; i++)
			{
				// is served?
				if (pbCustomerVisited[i] == true)
					continue;

				// check capacity
				if (piCustomerDemand[i] > iCapacity)
					continue;

				// check due date
				dDistance = m_pInstanceData->getCustomerDistance(iLastCustomer, i);
				dTmp = dTime + dDistance;

				if (dTmp > piCustomerDueDate[i])
					continue;

				// check depot due time
				if (dTmp < piCustomerReadyTime[i])
					dTmp = piCustomerReadyTime[i];

				dTmp += piCustomerServiceTime[i];
				dTmp += m_pInstanceData->getDepotDistance(i);

				if (dTmp > iDepotDueDate)
					continue;

				// difference between the completion of service at last
				// customer and beginning of service at next customer
				dTmp = piCustomerReadyTime[i]; // bj
				dTmp -= dTime; // (bi+si)
				dTmp = __max(dTmp, dDistance);

				dCosts = dTmp;

				// urgency of delivery to next customer
				dTmp = piCustomerDueDate[i]; // lj
				dTmp -= dTime; // (bi+si)

				dCosts *= dTmp;
				dCosts *= dW1;

				// difference in the position angle between the last customer
				// and the next customer [!! to be optimized -> Matrix]
				dTmp = atan2((float)piCustomerYCoord[iLastCustomer],
					(float)piCustomerXCoord[iLastCustomer]); 
					
				dTmp -= atan2((float)piCustomerYCoord[i], (float)piCustomerXCoord[i]);

				if (dTmp < 0.0)
					dTmp *= -1.0;

				dTmp *= dW2;

				dCosts += dTmp;

				if (iNextCustomer == -1
					|| dCosts < dMinCosts)
				{
					iNextCustomer = i;
					dMinCosts = dCosts;
				}
			}

			if (iNextCustomer == -1)
				break;

			// add customer
			dDistance = m_pInstanceData->getCustomerDistance(iLastCustomer,
				iNextCustomer);
				
			dTotalDistance += dDistance;
			dTime += dDistance;

			if (dTime < piCustomerReadyTime[iNextCustomer])
				dTime = piCustomerReadyTime[iNextCustomer];

			dTime += piCustomerServiceTime[iNextCustomer];
			iCapacity -= piCustomerDemand[iNextCustomer];

			pbCustomerVisited[iNextCustomer] = true;
			*piNextCustomer = iNextCustomer;
			piNextCustomer++;

			iLastCustomer = iNextCustomer;

			if (iCapacity == 0)
				break;
		}
		while (true);
	}
	while (true);

	m_pInstanceData->setSolution(iVehicleCount, dTotalDistance, piSolutionTours);

	free(pbCustomerVisited);
	return 0;
}

void Vrptw::convertToTourMatrix(int iVehicleCount,
								int *piTours,
								int **ppiTourMatrix)
{
	int i, iVehicle;

	for (iVehicle=0; iVehicle<iVehicleCount; iVehicle++)
	{
		i = 0;

		do
		{
			ppiTourMatrix[iVehicle][++i] = *piTours;
			piTours++;
		}
		while(*piTours != -1);

		piTours++;

		ppiTourMatrix[iVehicle][0] = i;
	}
}

void Vrptw::convertFromTourMatrix(int iVehicleCount,
								  int *piTours,
								  int **ppiTourMatrix)
{
	int i, iVehicle;

	for (iVehicle=0; iVehicle<iVehicleCount; iVehicle++)
	{
		for (i=1; i<=ppiTourMatrix[iVehicle][0]; i++)
		{
			*piTours = ppiTourMatrix[iVehicle][i];
			piTours++;
		}

		*piTours = -1;
		piTours++;
	}
}

int Vrptw::ls_cross_exchange(int iVehicleCount,
							 double *pdTotalDistance,
							 int *piTours,
							 bool *pbAbort)
{
	bool bSetSolution;
	int iRet, iCustomerCount;
	double dTotalDistance;
	int **ppiTourMatrix;

	// check params
	if (m_pInstanceData == NULL)
		return -1;

	iCustomerCount = m_pInstanceData->getCustomerCount();

	// use stored solution?
	if (iVehicleCount == 0 || pdTotalDistance == NULL || piTours == NULL)
	{
		pdTotalDistance = &dTotalDistance;
		piTours = m_pInstanceData->getSolutionTours(&iVehicleCount,
			pdTotalDistance);

		if (piTours == NULL)
			return -1;

		bSetSolution = true;
	}
	else
		bSetSolution = false;

	// allocate memory
	ppiTourMatrix = generate_int_matrix(iVehicleCount, iCustomerCount+1);
	
	if (ppiTourMatrix == NULL)
		return -1;
	
	// copy tours to tour matrix
	convertToTourMatrix(iVehicleCount, piTours, ppiTourMatrix);

	// do cross exchange
	iRet = ls_cross_exchange_matrix(iVehicleCount, pdTotalDistance,
		ppiTourMatrix, pbAbort);

	if (iRet == 0)
	{
		// copy tour matrix to tours
		convertFromTourMatrix(iVehicleCount, piTours, ppiTourMatrix);

		if (bSetSolution)
		{
			m_pInstanceData->setSolutionVehicleCount(iVehicleCount);
			m_pInstanceData->setSolutionDistance(*pdTotalDistance);
		}
	}

	// free memory
	free(ppiTourMatrix);

	return iRet;
}

int Vrptw::ls_cross_exchange_matrix(int iVehicleCount,
									double *pdTotalDistance,
									int **ppiTourMatrix,
									bool *pbAbort)
{
	bool bAbort;
	int i, iCount, iDepotDueDate, iCapacity, iMaxCapacity;
	int iX1, iX2, iY1, iY2, iBestX1, iBestX2, iBestY1, iBestY2;
	int iLastCustomer, iNextCustomer, iCustomerCount;
	int iRoute1, iRoute2, iCustomerCount1, iCustomerCount2;
	int iCustomerX1_0, iCustomerX2_0, iCustomerY1_0, iCustomerY2_0;
	int iCustomerX1_1, iCustomerX2_1, iCustomerY1_1, iCustomerY2_1;
	double dTime, dDistDiff1, dDistDiff2, dBestDistDiff, dTotalDistance;
	int *piCustomerDemand, *piCustomerReadyTime, *piCustomerDueDate;
	int *piCustomerServiceTime, *piTempTour1, *piTempTour2;
	double **ppdDistanceMatrix;

	if (pbAbort == NULL)
	{
		bAbort = false;
		pbAbort = &bAbort;
	}

	// check params
	if (m_pInstanceData == NULL)
		return -1;

	dTotalDistance = *pdTotalDistance;

	iCustomerCount = m_pInstanceData->getCustomerCount();
	iDepotDueDate = m_pInstanceData->getDepotDueDate();
	iMaxCapacity = m_pInstanceData->getCapacity();
	
	if ((piTempTour1 = (int *)malloc(sizeof(int) * iCustomerCount+1)) == NULL)
		return -1;

	if ((piTempTour2 = (int *)malloc(sizeof(int) * iCustomerCount+1)) == NULL)
	{
		free(piTempTour1);
		return -1;
	}

	piCustomerDemand = m_pInstanceData->getCustomerDemand();
	piCustomerReadyTime = m_pInstanceData->getCustomerReadyTime();
	piCustomerDueDate = m_pInstanceData->getCustomerDueDate();
	piCustomerServiceTime = m_pInstanceData->getCustomerServiceTime();
	ppdDistanceMatrix = m_pInstanceData->getDistanceMatrix();

	for (iRoute1=0; iRoute1<iVehicleCount-1; iRoute1++)	// route 1
	{
		for (iRoute2=iRoute1+1; iRoute2<iVehicleCount; iRoute2++) // route 2
		{
			dBestDistDiff = 0.0;

			iCustomerCount1 = ppiTourMatrix[iRoute1][0];
			iCustomerCount2 = ppiTourMatrix[iRoute2][0];

			if (iCustomerCount1 < 2 || iCustomerCount2 < 2)
				continue;

			for (iX1 = 1; iX1 < iCustomerCount1; iX1++)
			{
				for (iX2 = 1; iX2 < iCustomerCount2; iX2++)
				{
					if (*pbAbort)
					{
						free(piTempTour1);
						free(piTempTour2);
						return -1;
					}

					iCustomerX1_0 = ppiTourMatrix[iRoute1][iX1];
					iCustomerX1_1 = ppiTourMatrix[iRoute1][iX1+1];
					iCustomerX2_0 = ppiTourMatrix[iRoute2][iX2];
					iCustomerX2_1 = ppiTourMatrix[iRoute2][iX2+1];

					// calc dist diff 1 = new1 + new2 - old1 - old2
					dDistDiff1 = ppdDistanceMatrix[iCustomerX1_0][iCustomerX2_1];
					dDistDiff1 += ppdDistanceMatrix[iCustomerX2_0][iCustomerX1_1];
					dDistDiff1 -= ppdDistanceMatrix[iCustomerX1_0][iCustomerX1_1];
					dDistDiff1 -= ppdDistanceMatrix[iCustomerX2_0][iCustomerX2_1];

					if (dDistDiff1 >= 0.0)
						continue;

					for (iY1 = iX1+1; iY1 <= iCustomerCount1; iY1++)
					{
						for (iY2 = iX2+1; iY2 <= iCustomerCount2; iY2++)
						{
							if (*pbAbort)
							{
								free(piTempTour1);
								free(piTempTour2);
								return -1;
							}

							iCustomerY1_0 = ppiTourMatrix[iRoute1][iY1];

							if (iY1 < iCustomerCount1)
								iCustomerY1_1 = ppiTourMatrix[iRoute1][iY1+1];
							else
								iCustomerY1_1 = iCustomerCount; // depot

							iCustomerY2_0 = ppiTourMatrix[iRoute2][iY2];

							if (iY2 < iCustomerCount2)
								iCustomerY2_1 = ppiTourMatrix[iRoute2][iY2+1];
							else
								iCustomerY2_1 = iCustomerCount; // depot

							// calc dist diff 2 = new1 + new2 - old1 - old2
							dDistDiff2
								= ppdDistanceMatrix[iCustomerY1_0][iCustomerY2_1];
							dDistDiff2
								+= ppdDistanceMatrix[iCustomerY2_0][iCustomerY1_1];
							dDistDiff2
								-= ppdDistanceMatrix[iCustomerY1_0][iCustomerY1_1];
							dDistDiff2
								-= ppdDistanceMatrix[iCustomerY2_0][iCustomerY2_1];

							if (dDistDiff2 > 0.0)
								continue;

							// better solution found?
							if (dBestDistDiff <= dDistDiff1+dDistDiff2)
								continue;

							// build new temp tours
							ls_cross_exchange_tour(iX1, iX2, iY1, iY2,
								iCustomerCount1, iCustomerCount2,
								ppiTourMatrix[iRoute1],	ppiTourMatrix[iRoute2],
								piTempTour1, piTempTour2);

							// is new tour 1 feasible?
							dTime = 0.0;
							iCapacity = 0;
							iLastCustomer = iCustomerCount; // depot
							iCount = piTempTour1[0];

							for (i=1; i<=iCount; i++)
							{
								iNextCustomer = piTempTour1[i];

								// check capacity
								iCapacity += piCustomerDemand[iNextCustomer];

								if (iCapacity > iMaxCapacity)
									break; // not feasible

								// check time window
								dTime +=
								ppdDistanceMatrix[iLastCustomer][iNextCustomer];

								if (dTime < piCustomerReadyTime[iNextCustomer])
									dTime = piCustomerReadyTime[iNextCustomer];
								else if (dTime > piCustomerDueDate[iNextCustomer])
									break; // not feasible

								dTime += piCustomerServiceTime[iNextCustomer];

								iLastCustomer = iNextCustomer;
							}

							if (i <= iCount)
								continue; // not feasible

							if (ppdDistanceMatrix[iLastCustomer][iCustomerCount]
								+ dTime	> iDepotDueDate)
							{
								continue; // not feasible
							}
							
							// is new tour 2 feasible?
							dTime = 0.0;
							iCapacity = 0;
							iLastCustomer = iCustomerCount; // depot
							iCount = piTempTour2[0];

							for (i=1; i<=iCount; i++)
							{
								iNextCustomer = piTempTour2[i];

								// check capacity
								iCapacity += piCustomerDemand[iNextCustomer];

								if (iCapacity > iMaxCapacity)
									break; // not feasible

								// check time window
								dTime +=
								ppdDistanceMatrix[iLastCustomer][iNextCustomer];

								if (dTime < piCustomerReadyTime[iNextCustomer])
									dTime = piCustomerReadyTime[iNextCustomer];
								else if (dTime > piCustomerDueDate[iNextCustomer])
									break; // not feasible

								dTime += piCustomerServiceTime[iNextCustomer];

								iLastCustomer = iNextCustomer;
							}

							if (i <= iCount)
								continue; // not feasible

							if (ppdDistanceMatrix[iLastCustomer][iCustomerCount]
								+ dTime > iDepotDueDate)
							{
								continue; // not feasible
							}
							
							// feasible solution
							dBestDistDiff = dDistDiff1 + dDistDiff2;
							iBestX1 = iX1;
							iBestX2 = iX2;
							iBestY1 = iY1;
							iBestY2 = iY2;
						}
					}
				}
			}

			// better solution found?
			if (dBestDistDiff < 0.0)
			{
				// build new tours
				ls_cross_exchange_tour(iBestX1, iBestX2, iBestY1, iBestY2,
					iCustomerCount1, iCustomerCount2,  ppiTourMatrix[iRoute1],
					ppiTourMatrix[iRoute2], piTempTour1, piTempTour2);

				IntCopy(ppiTourMatrix[iRoute1], piTempTour1, piTempTour1[0]+1);
				IntCopy(ppiTourMatrix[iRoute2], piTempTour2, piTempTour2[0]+1);

				dTotalDistance += dBestDistDiff;
			}
		}
	}

	*pdTotalDistance = dTotalDistance;

	// cleanup
	free(piTempTour1);
	free(piTempTour2);

	return 0;
}

void Vrptw::ls_cross_exchange_tour(int iX1,
								   int iX2,
								   int iY1,
								   int iY2,
								   int iCustomerCount1,
								   int iCustomerCount2,
								   int *piTour1,
								   int *piTour2,
								   int *piTempTour1,
								   int *piTempTour2)
{
	int i, j;

	// Tour 1
	i = 1;

	for (j=1; j<=iX1; j++)
		piTempTour1[i++] = piTour1[j];

	// cross X
	for (j=iX2+1; j<=iY2; j++)
		piTempTour1[i++] = piTour2[j];

	// cross Y
	for (j=iY1+1; j<=iCustomerCount1; j++)
		piTempTour1[i++] = piTour1[j];

	piTempTour1[0] = i - 1;

	// Tour2
	i = 1;

	for (j=1; j<=iX2; j++)
		piTempTour2[i++] = piTour2[j];

	// cross X
	for (j=iX1+1; j<=iY1; j++)
		piTempTour2[i++] = piTour1[j];

	// cross Y
	for (j=iY2+1; j<=iCustomerCount2; j++)
		piTempTour2[i++] = piTour2[j];

	piTempTour2[0] = i - 1;
}

int Vrptw::ls_intra_exchange(int iVehicleCount,
							 double *pdTotalDistance,
							 int *piTours,
							 bool *pbAbort)
{
	bool bSetSolution;
	int iRet, iCustomerCount;
	double dTotalDistance;
	int **ppiTourMatrix;

	// check params
	if (m_pInstanceData == NULL)
		return -1;

	iCustomerCount = m_pInstanceData->getCustomerCount();

	// use stored solution?
	if (iVehicleCount == 0 || pdTotalDistance == NULL || piTours == NULL)
	{
		pdTotalDistance = &dTotalDistance;
		piTours = m_pInstanceData->getSolutionTours(&iVehicleCount,
			pdTotalDistance);

		if (piTours == NULL)
			return -1;

		bSetSolution = true;
	}
	else
		bSetSolution = false;

	// allocate memory
	ppiTourMatrix = generate_int_matrix(iVehicleCount, iCustomerCount+1);
	
	if (ppiTourMatrix == NULL)
		return -1;
	
	// copy tours to tour matrix
	convertToTourMatrix(iVehicleCount, piTours, ppiTourMatrix);

	// do intra exchange
	iRet = ls_intra_exchange_matrix(iVehicleCount, pdTotalDistance,
		ppiTourMatrix, NULL);

	if (iRet == 0)
	{
		// copy tour matrix to tours
		convertFromTourMatrix(iVehicleCount, piTours, ppiTourMatrix);

		if (bSetSolution)
		{
			m_pInstanceData->setSolutionVehicleCount(iVehicleCount);
			m_pInstanceData->setSolutionDistance(*pdTotalDistance);
		}
	}

	// free memory
	free(ppiTourMatrix);

	return iRet;
}

int Vrptw::ls_intra_exchange_matrix(int iVehicleCount,
									double *pdTotalDistance,
									int **ppiTourMatrix,
									bool *pbAbort)
{
	int i, j, k;
	int iRoute, iCustomerCount, iRouteCustomerCount;
	int iCurr1, iCurr2, iPrev1, iPrev2, iNext1, iNext2;
	int *piCustomerReadyTime, *piCustomerDueDate, *piCustomerServiceTime;
	int iLastCustomer, iNextCustomer, iDepotDueDate;
	double **ppdDistanceMatrix;
	double dDistDiff, dTime, dTotalDistance;
	bool bSwapped, bAbort;

	if (pbAbort == NULL)
	{
		bAbort = false;
		pbAbort = &bAbort;
	}

	dTotalDistance = *pdTotalDistance;

	iCustomerCount = m_pInstanceData->getCustomerCount();
	iDepotDueDate = m_pInstanceData->getDepotDueDate();

	piCustomerReadyTime = m_pInstanceData->getCustomerReadyTime();
	piCustomerDueDate = m_pInstanceData->getCustomerDueDate();
	piCustomerServiceTime = m_pInstanceData->getCustomerServiceTime();
	ppdDistanceMatrix = m_pInstanceData->getDistanceMatrix();

	for (iRoute=0; iRoute<iVehicleCount; iRoute++)
	{
		iRouteCustomerCount = ppiTourMatrix[iRoute][0];

		bSwapped = true;

		while (bSwapped)
		{
			bSwapped = false;

			for (i=1; i<iRouteCustomerCount; i++)
			{
				for (j=i+1; j<=iRouteCustomerCount; j++)
				{
					if (*pbAbort)
						return -1;

					iCurr1 = ppiTourMatrix[iRoute][i];
					iCurr2 = ppiTourMatrix[iRoute][j];

					if (i == 1)
						iPrev1 = iCustomerCount; // depot
					else
						iPrev1 = ppiTourMatrix[iRoute][i-1];

					iNext1 = ppiTourMatrix[iRoute][i+1];
					iPrev2 = ppiTourMatrix[iRoute][j-1];

					if (j == iRouteCustomerCount)
						iNext2 = iCustomerCount; // depot
					else
						iNext2 = ppiTourMatrix[iRoute][j+1];

					 // new dist
					dDistDiff = ppdDistanceMatrix[iPrev1][iCurr2];
					dDistDiff += ppdDistanceMatrix[iCurr2][iNext1];
					dDistDiff += ppdDistanceMatrix[iPrev2][iCurr1];
					dDistDiff += ppdDistanceMatrix[iCurr1][iNext2];

					// old dist
					dDistDiff -= ppdDistanceMatrix[iPrev1][iCurr1];

					if (i + 1 != j) // customers are not 'neighbors'
					{
						dDistDiff -= ppdDistanceMatrix[iCurr1][iNext1];
						dDistDiff -= ppdDistanceMatrix[iPrev2][iCurr2];
					}

					dDistDiff -= ppdDistanceMatrix[iCurr2][iNext2];

					if (dDistDiff >= 0.0)
						continue;

					// check time windows
					dTime = 0.0;
					iLastCustomer = iCustomerCount; // depot

					for (k=1; k<=iRouteCustomerCount; k++)
					{
						if (k == i)
							iNextCustomer = iCurr2;
						else if (k == j)
							iNextCustomer = iCurr1;
						else
							iNextCustomer = ppiTourMatrix[iRoute][k];

						dTime += ppdDistanceMatrix[iLastCustomer][iNextCustomer];

						if (dTime < piCustomerReadyTime[iNextCustomer])
							dTime = piCustomerReadyTime[iNextCustomer];
						else if (dTime > piCustomerDueDate[iNextCustomer])
							break; // not feasible

						dTime += piCustomerServiceTime[iNextCustomer];

						iLastCustomer = iNextCustomer;
					}

					if (k <= iRouteCustomerCount)
						continue;	// not feasible

					if (dTime + ppdDistanceMatrix[iLastCustomer][iCustomerCount]
						> iDepotDueDate)
					{
						continue; // not feasible
					}
					
					// swap customers
					ppiTourMatrix[iRoute][i] = iCurr2;
					ppiTourMatrix[iRoute][j] = iCurr1;
					dTotalDistance += dDistDiff;

					bSwapped = true;
					break;
				}

				if (bSwapped)
					break;
			}
		}
	}

	*pdTotalDistance = dTotalDistance;

	return 0;
}
