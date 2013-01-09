//
// InstanceData.h
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

#if !defined(_INSTANCEDATA_H_)
#define _INSTANCEDATA_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


///// includes /////

#include <string.h>
#include "Vrptw.h"


///// classes /////

class InstanceData  
{
public:
	InstanceData();
	virtual ~InstanceData();

	void setSolution(int iVehicleCount,
					 double dDistance,
					 int *piTours);
					 
	void setSolutionVehicleCount(int iVehicleCount)
		{ m_iSolutionVehicleCount = iVehicleCount; };
			
	void setSolutionDistance(double dSolutionDistance)
		{ m_dSolutionDistance = dSolutionDistance; };
			
	int *getSolutionTours(int *piVehicleCount=NULL,
						  double *pdDistance=NULL);
						  
	int getSolutionVehicleCount() { return m_iSolutionVehicleCount; };
			
	double getSolutionDistance() { return m_dSolutionDistance; };

	bool getOptimalSolution(int *piVehicleCount,
							double *pdyDistance,
							char *szAuthors);
							
	bool getBestHeuristicSolution(int *piVehicleCount,
								  double *pdDistance,
								  char *szAuthors);

	int read(const char *szFilename);
	
	int read(const char *pData,
			 unsigned int uiDataLength);
			 
	int read(Vrptw::INSTANCE_t *pInstance);

	char * getErrorText() { return m_szError; };

	char * getName() { return m_szName; };
	
	int getVehicleCount() { return m_iVehicleCount; };
	
	int getCapacity() { return m_iCapacity; };
	
	int getCustomerCount() { return m_iCustomerCount; };

	int getMinXCoord() { return m_iMinXCoord; };
	
	int getMaxXCoord() { return m_iMaxXCoord; };
	
	int getMinYCoord() { return m_iMinYCoord; };
	
	int getMaxYCoord() { return m_iMaxYCoord; };

	int getDepotXCoord() { return m_iDepotXCoord; };
	
	int getDepotYCoord() { return m_iDepotYCoord; };
	
	int getDepotDueDate() { return m_iDepotDueDate; };

	int *getCustomerXCoord() { return m_piCustomerXCoord; };
	
	int *getCustomerYCoord() { return m_piCustomerYCoord; };
	
	int *getCustomerDemand() { return m_piCustomerDemand; };
	
	int *getCustomerReadyTime() { return m_piCustomerReadyTime; };
	
	int *getCustomerDueDate() { return m_piCustomerDueDate; };
	
	int *getCustomerServiceTime() { return m_piCustomerServiceTime; };

	double **getDistanceMatrix() { return m_ppdDistanceMatrix; };
	
	double getDepotDistance(int iCustomer)
		{ return m_ppdDistanceMatrix[iCustomer][m_iCustomerCount]; };
		
	double getCustomerDistance(int iCustomer1, int iCustomer2)
		{ return m_ppdDistanceMatrix[iCustomer1][iCustomer2]; };

	bool isDataLoaded() { return m_bDataLoaded; };
	
	bool isSolutionFeasible();

protected:
	void cleanup();
	
	int calcDistances();

	int getNextLine(int *piLineLength=NULL);
	
	int getInteger(int *piValue);
	
	int compareNoCase(char *szValue);

	// instance data
	bool m_bDataLoaded;
	char m_szName[33];
	int m_iVehicleCount;
	int m_iCapacity;
	int m_iCustomerCount;

	// solution
	int *m_piSolutionTours;
	int m_iSolutionVehicleCount;
	double m_dSolutionDistance;
	Vrptw::SOLUTION_t m_KnownSolution;

	// landscape
	int m_iMinXCoord;
	int m_iMaxXCoord;
	int m_iMinYCoord;
	int m_iMaxYCoord;

	// depot data
	int m_iDepotXCoord;
	int m_iDepotYCoord;
	int m_iDepotDueDate;

	// customer data
	int *m_pCustomerData;
	int *m_piCustomerXCoord;
	int *m_piCustomerYCoord;
	int *m_piCustomerDemand;
	int *m_piCustomerReadyTime;
	int *m_piCustomerDueDate;
	int *m_piCustomerServiceTime;

	double **m_ppdDistanceMatrix;

	// vars used to parse the input file
	char m_szError[512];
	char *m_pDataPos;
	char *m_pDataEnd;
	int m_iLine;
	char *m_pLinePos;
	char *m_pLineEnd;
};

#endif // _INSTANCEDATA_H_
