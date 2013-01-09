//
// Vrptw.h
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

#ifndef _VRPTW_H_
#define _VRPTW_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


//// classes /////

class InstanceData;


class Vrptw  
{
public:
	typedef struct 
	{
		int iXCoord;
		int iYCoord;
		int iDemand;
		int iReadyTime;
		int iDueDate;
		int iServiceTime;
	}
	CUSTOMER_t;

	typedef struct
	{
		char szName[33];
		int iOptimalVehicleCount;
		double dOptimalTotalDistance;
		char szOptimalAuthors[33];
		int iHeuristicVehicleCount;
		double dHeuristicTotalDistance;
		char szHeuristicAuthors[33];
	}
	SOLUTION_t;

	typedef struct
	{
		char szName[33];
		int iVehicleCount;
		int iCapacity;
		int iCustomer;
		int iDepotXCoord;
		int iDepotYCoord;
		int iDepotDueDate;
		CUSTOMER_t *pCustomer;
		SOLUTION_t *pSolution;
	}
	INSTANCE_t;

public:
	Vrptw();

	Vrptw(InstanceData *pInstanceData);

	virtual ~Vrptw();

	void applyInstanceData(InstanceData *pInstanceData);

	int nn_solomon1987(double dW1,
					   double dW2,
					   double dW3);

	int nn_gambardella1999();

	int nn_ellabib2002(double dW1,
					   double dW2);

	int ls_cross_exchange(int iVehicleCount,
						  double *pdTotalDistance,
						  int *piTours,
						  bool *pbAbort);

	int ls_cross_exchange_matrix(int iVehicleCount,
								 double *pdTotalDistance, 
								 int **ppiTourMatrix,
								 bool *pbAbort);

	int ls_intra_exchange(int iVehicleCount,
						  double *pdTotalDistance,
						  int *piTours,
						  bool *pbAbort);

	int ls_intra_exchange_matrix(int iVehicleCount,
								 double *pdTotalDistance,
								 int **ppiTourMatrix,
								 bool *pbAbort);

protected:
	InstanceData *m_pInstanceData;

	void convertToTourMatrix(int iVehicleCount,
							 int *piTours,
							 int **ppiTourMatrix);

	void convertFromTourMatrix(int iVehicleCount,
							   int *piTours,
							   int **ppiTourMatrix);

	void ls_cross_exchange_tour(int iX1,
								int iX2,
								int iY1,
								int iY2,
								int iCustomerCount1,
								int iCustomerCount2,
								int *piTour1,
								int *piTour2,
								int *piTempTour1,
								int *piTempTour2);
};

#endif // _VRPTW_H_
