//
// InstanceData.cpp
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
//	31.12.2006  =pd=		first version only supports VRPTW instances
//


///// includes /////

#include "InstanceData.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "utils.h"


///// defines /////

#define MAX_STRING_LENGTH 256
#define MAX_INTEGER_LENGTH 9


///// classes /////

InstanceData::InstanceData()
{
	m_pCustomerData = NULL;
	m_ppdDistanceMatrix = NULL;
	m_piSolutionTours = NULL;

	cleanup();
}

InstanceData::~InstanceData()
{
	cleanup();
}

void InstanceData::cleanup()
{
	m_bDataLoaded = false;

	if (m_pCustomerData != NULL)
	{
		free(m_pCustomerData);
		m_pCustomerData = NULL;
	}

	if (m_ppdDistanceMatrix != NULL)
	{
		free(m_ppdDistanceMatrix);
		m_ppdDistanceMatrix = NULL;
	}

	m_piCustomerXCoord = NULL;
	m_piCustomerYCoord = NULL;
	m_piCustomerDemand = NULL;
	m_piCustomerReadyTime = NULL;
	m_piCustomerDueDate = NULL;
	m_piCustomerServiceTime = NULL;

	m_szError[0] = '\0';

	m_szName[0] = '\0';
	m_iVehicleCount = 0;
	m_iCapacity = 0;
	m_iCustomerCount = 0;

	setSolution(0, 0.0, NULL);

	memset((char*)&m_KnownSolution, 0, sizeof(Vrptw::SOLUTION_t));
}

void InstanceData::setSolution(int iVehicleCount,
							   double dDistance,
							   int *piTours)
{
	if (m_piSolutionTours != NULL)
	{
		free(m_piSolutionTours);
		m_piSolutionTours = NULL;
	}

	if (iVehicleCount == 0
		|| dDistance == 0.0
		|| piTours == NULL)
	{
		m_iSolutionVehicleCount = 0;
		m_dSolutionDistance = 0;
		return;
	}

	m_iSolutionVehicleCount = iVehicleCount;
	m_dSolutionDistance = dDistance;
	m_piSolutionTours = piTours;
}

int *InstanceData::getSolutionTours(int *piVehicleCount,
									double *pdDistance)
{
	if (m_piSolutionTours == NULL)
		return NULL;

	if (piVehicleCount != NULL)
		*piVehicleCount = m_iSolutionVehicleCount;

	if (pdDistance != NULL)
		*pdDistance = m_dSolutionDistance;

	return m_piSolutionTours;
}

bool InstanceData::getOptimalSolution(int *piVehicleCount,
									  double *pdDistance,
									  char *szAuthors)
{
	if (m_KnownSolution.iOptimalVehicleCount == 0)
		return false;

	if (piVehicleCount != NULL)
		*piVehicleCount = m_KnownSolution.iOptimalVehicleCount;

	if (pdDistance != NULL)
		*pdDistance = m_KnownSolution.dOptimalTotalDistance;

	if (szAuthors != NULL)
		strcpy(szAuthors, m_KnownSolution.szOptimalAuthors);

	return true;
}

bool InstanceData::getBestHeuristicSolution(int *piVehicleCount,
											double *pdDistance,
											char *szAuthors)
{
	if (m_KnownSolution.iHeuristicVehicleCount == 0)
		return false;

	if (piVehicleCount != NULL)
		*piVehicleCount = m_KnownSolution.iHeuristicVehicleCount;

	if (pdDistance != NULL)
		*pdDistance = m_KnownSolution.dHeuristicTotalDistance;

	if (szAuthors != NULL)
		strcpy(szAuthors, m_KnownSolution.szHeuristicAuthors);

	return true;
}

bool InstanceData::isSolutionFeasible()
{
	bool bFeasible;
	int i, iCapacity, iVehicle, iNextCustomer, iLastCustomer;
	double dTime, dDistance, dTotalDistance;
	bool *pbCustomerVisited;
	int *piNext;

	if (m_bDataLoaded == false
		|| m_piSolutionTours == NULL)
	{
		return false;
	}

	// malloc memory
	pbCustomerVisited = (bool*)malloc(sizeof(bool)*m_iCustomerCount);

	if (pbCustomerVisited == NULL)
		return false;

	for (i=0; i<m_iCustomerCount; i++)
		pbCustomerVisited[i] = false;

	bFeasible = true;
	piNext = m_piSolutionTours;
	dTotalDistance = 0.0;

	for (iVehicle=0; iVehicle<m_iSolutionVehicleCount; iVehicle++)
	{
		dTime = 0.0;
		iCapacity = 0;
		iLastCustomer = m_iCustomerCount; // depot

		do
		{
			iNextCustomer = *piNext;
			piNext++;

			if (iNextCustomer == -1)
				break;

			// already visited?
			if (pbCustomerVisited[iNextCustomer])
			{
				bFeasible = false;
				break;
			}

			pbCustomerVisited[iNextCustomer] = true;

			// check capacity
			iCapacity += m_piCustomerDemand[iNextCustomer];

			if (iCapacity > m_iCapacity)
			{
				bFeasible = false;
				break;
			}

			dDistance = m_ppdDistanceMatrix[iLastCustomer][iNextCustomer];
			dTotalDistance += dDistance;

			// check time windows
			dTime += dDistance;

			if (dTime < m_piCustomerReadyTime[iNextCustomer])
				dTime = m_piCustomerReadyTime[iNextCustomer];
			else if (dTime > m_piCustomerDueDate[iNextCustomer])
			{
				bFeasible = false;
				break;
			}

			dTime += m_piCustomerServiceTime[iNextCustomer];

			iLastCustomer = iNextCustomer;
		}
		while (true);

		dDistance = m_ppdDistanceMatrix[iLastCustomer][m_iCustomerCount];
		dTotalDistance += dDistance;

		if (dTime + dDistance > m_iDepotDueDate)
		{
			bFeasible = false;
			break;
		}

		if (bFeasible == false)
			break;
	}

	// all customers visited?
	for (i=0; i<m_iCustomerCount; i++)
	{
		if (pbCustomerVisited[i] == false)
		{
			bFeasible = false;
			break;
		}
	}

	// cleanup
	free(pbCustomerVisited);

	return bFeasible;
}

int InstanceData::read(const char *szFilename)
{
	int iRet;
	unsigned int uiDataLength;
	FILE *fp;
	char *pData;

	// cleanup
	cleanup();

	// open input file
	fp = fopen(szFilename, "rb");

	if (fp == NULL)
	{
		strcpy(m_szError, "Failed to open the file.");
		return 1;
	}

	// seek to end of file
	if (fseek(fp, 0L, SEEK_END) != 0)
	{
		fclose(fp);
		strcpy(m_szError, "Failed to read the file.");
		return 1;
	}

	// determine file length
	uiDataLength = ftell(fp);

	if (uiDataLength < 50)
	{
		fclose(fp);
		strcpy(m_szError, "Invalid file format (file length too small).");
		return 1;
	}

	// seek to start of file
	if (fseek(fp, 0L, SEEK_SET) != 0)
	{
		fclose(fp);
		strcpy(m_szError, "Failed to read the file.");
		return 1;
	}


	// allocate memory
	pData = (char*)malloc(uiDataLength);

	if (pData == NULL)
	{
		fclose(fp);
		strcpy(m_szError, "Out of mem.");
		return 1;
	}

	// read file contents
	if (fread(pData, 1, uiDataLength, fp) != uiDataLength)
	{
		free(pData);
		fclose(fp);
		strcpy(m_szError, "Failed to read the file.");
		return 1;
	}

	// close input file
	fclose(fp);

	// parse data
	iRet = read(pData, uiDataLength);

	// free memory
	free(pData);

	return iRet;
}

int InstanceData::read(const char *pData,
					   unsigned int uiDataLength)
{
	int iRet, iLength, iValue, iCustomerCount;
	char *pDataPos;
	int *piCustomerXCoord;
	int *piCustomerYCoord;
	int *piCustomerDemand;
	int *piCustomerReadyTime;
	int *piCustomerDueDate;
	int *piCustomerServiceTime;

	cleanup();

	if (uiDataLength < 50)
	{
		strcpy(m_szError, "Invalid file format (file length too small).");
		return 1;
	}

	m_pDataPos = (char*)pData;
	m_pDataEnd = m_pDataPos + uiDataLength;
	m_iLine = 1;

	// get problems name
	iRet = getNextLine(&iLength);

	if (iRet != 0)
	{
		sprintf(m_szError,
			"Invalid format - problems name expected (line = %d).",	m_iLine);
				
		return iRet;
	}

	iLength = __min(32, iLength);
	memcpy(m_szName, m_pLinePos, iLength);
	m_szName[iLength] = '\0';

	// get head 'VEHICLE'
	iRet = getNextLine();

	if (iRet != 0)
	{
		sprintf(m_szError,
			"Invalid format - head 'VEHICLE' expected (line = %d).", m_iLine);
				
		return iRet;
	}

	iRet = compareNoCase("VEHICLE");

	if (iRet != 0)
	{
		sprintf(m_szError,
			"Invalid format - head 'VEHICLE' expected (line = %d).", m_iLine);
				
		return iRet;
	}

	// get head 'NUMBER   CAPACITY'
	do
	{
		iRet = getNextLine();

		if (iRet != 0)
			break;

		iRet = compareNoCase("NUMBER");

		if (iRet != 0)
			break;

		iRet = compareNoCase("CAPACITY");
		break;
	}
	while (false);

	if (iRet != 0)
	{
		sprintf(m_szError,
			"Invalid format - head 'NUMBER  CAPACITY' expected (line = %d).",
			m_iLine);
				
		return iRet;
	}

	// get vehicle number
	iRet = getNextLine();

	if (iRet != 0)
	{
		sprintf(m_szError,
			"Invalid format - vehicle number expectet (line = %d).", m_iLine);
			
		return iRet;
	}

	iRet = getInteger(&m_iVehicleCount);

	if (iRet != 0)
		return iRet;

	iRet = getInteger(&m_iCapacity);

	if (iRet != 0)
		return iRet;

	// get head 'CUSTOMER'
	iRet = getNextLine();

	if (iRet != 0)
		return iRet;

	iRet = compareNoCase("CUSTOMER");

	if (iRet != 0)
	{
		sprintf(m_szError,
			"Invalid format - head 'CUSTOMER' expected (line = %d).", 
			m_iLine);
			
		return iRet;
	}

	// get head 
	// 'CUST NO.  XCOORD.  YCOORD.  DEMAND  READY TIME  DUE DATE  SERVICE TIME'
	do
	{
		iRet = getNextLine();

		if (iRet != 0)
			break;

		iRet = compareNoCase("CUST");

		if (iRet != 0)
			break;

		iRet = compareNoCase("NO.");

		if (iRet != 0)
			break;

		iRet = compareNoCase("XCOORD.");

		if (iRet != 0)
			break;

		iRet = compareNoCase("YCOORD.");

		if (iRet != 0)
			break;

		iRet = compareNoCase("DEMAND");

		if (iRet != 0)
			break;

		iRet = compareNoCase("READY");

		if (iRet != 0)
			break;

		iRet = compareNoCase("TIME");

		if (iRet != 0)
			break;

		iRet = compareNoCase("DUE");

		if (iRet != 0)
			break;

		iRet = compareNoCase("DATE");

		if (iRet != 0)
			break;

		iRet = compareNoCase("SERVICE");

		if (iRet != 0)
			break;

		iRet = compareNoCase("TIME");
		break;
	}
	while (false);

	if (iRet != 0)
	{
		sprintf(m_szError,
			"Invalid format - head 'CUST NO.  XCOORD.  YCOORD.  DEMAND  READY"
			"TIME  DUE DATE  SERVICE TIME' expected (line = %d).",	m_iLine);
			
		return iRet;
	}

	// get depot data
	iRet = getNextLine();

	if (iRet != 0)
	{
		sprintf(m_szError,
			"Invalid format - depot data expected (line = %d).", m_iLine);
			
		return iRet;
	}

	// depot: customer number
	iRet = getInteger(&iValue);

	if (iRet != 0)
		return iRet;

	if (iValue != 0)
	{
		sprintf(m_szError,
				"Invalid format - depot data expected [CUST NO. != 0] "
				"(line = %d).", m_iLine);
		return 1;
	}

	// depot: xcoord
	iRet = getInteger(&m_iDepotXCoord);

	if (iRet != 0)
		return iRet;

	m_iMinXCoord = m_iMaxXCoord = m_iDepotXCoord;

	// depot: ycoord
	iRet = getInteger(&m_iDepotYCoord);

	if (iRet != 0)
		return iRet;

	m_iMinYCoord = m_iMaxYCoord = m_iDepotYCoord;

	// depot: demand
	iRet = getInteger(&iValue);

	if (iRet != 0)
		return iRet;

	// depot: ready time
	iRet = getInteger(&iValue);

	if (iRet != 0)
		return iRet;

	// depot: due date
	iRet = getInteger(&m_iDepotDueDate);

	if (iRet != 0)
		return iRet;

	// determine customer count
	pDataPos = m_pDataPos;

	iCustomerCount = 0;

	do
	{
		// get next customer
		iRet = getNextLine();

		if (iRet != 0)
			break;

		iCustomerCount++;

		// get customer number
		iRet = getInteger(&iValue);

		if (iRet != 0)
			return iRet;

		if (iValue != iCustomerCount)
		{
			sprintf(m_szError, 
				"Invalid format - customer number %d expected (line = %d).",
				iCustomerCount, m_iLine);
				
			return 1;
		}
	}
	while (true);

	if (iCustomerCount == 0)
	{
		sprintf(m_szError,
			"Invalid format - customer data expected (line = %d).",
			m_iLine);
			
		return 1;
	}

	m_iCustomerCount = iCustomerCount;

	// allocate memory for customer data
	m_pCustomerData = (int*)malloc(sizeof(int) * (iCustomerCount * 6));

	if (m_pCustomerData == NULL)
	{
		strcpy(m_szError, "Out of mem.");
		return 1;
	}

	m_piCustomerXCoord = m_pCustomerData;
	m_piCustomerYCoord = m_pCustomerData + iCustomerCount;
	m_piCustomerDemand = m_pCustomerData + iCustomerCount * 2;
	m_piCustomerReadyTime = m_pCustomerData + iCustomerCount * 3;
	m_piCustomerDueDate = m_pCustomerData + iCustomerCount * 4;
	m_piCustomerServiceTime = m_pCustomerData + iCustomerCount * 5;

	piCustomerXCoord = m_piCustomerXCoord;
	piCustomerYCoord = m_piCustomerYCoord;
	piCustomerDemand = m_piCustomerDemand;
	piCustomerReadyTime = m_piCustomerReadyTime;
	piCustomerDueDate = m_piCustomerDueDate;
	piCustomerServiceTime = m_piCustomerServiceTime;

	// get customer data
	m_pDataPos = pDataPos;

	do
	{
		// get next customer
		iRet = getNextLine();

		if (iRet != 0)
		{
			// no more data
			iRet = 0;
			break;
		}

		// get customer number
		iRet = getInteger(&iValue);

		if (iRet != 0)
			break;

		// get xcoord
		iRet = getInteger(piCustomerXCoord);

		if (m_iMinXCoord > *piCustomerXCoord)
			m_iMinXCoord = *piCustomerXCoord;
		else if (m_iMaxXCoord < *piCustomerXCoord)
			m_iMaxXCoord = *piCustomerXCoord;

		piCustomerXCoord++;

		if (iRet != 0)
			break;

		// get ycoord
		iRet = getInteger(piCustomerYCoord);

		if (m_iMinYCoord > *piCustomerYCoord)
			m_iMinYCoord = *piCustomerYCoord;
		else if (m_iMaxYCoord < *piCustomerYCoord)
			m_iMaxYCoord = *piCustomerYCoord;

		piCustomerYCoord++;

		if (iRet != 0)
			break;

		// get demand
		iRet = getInteger(piCustomerDemand);
		piCustomerDemand++;

		if (iRet != 0)
			break;

		// get ready time
		iRet = getInteger(piCustomerReadyTime);
		piCustomerReadyTime++;

		if (iRet != 0)
			break;

		// get due date
		iRet = getInteger(piCustomerDueDate);
		piCustomerDueDate++;

		if (iRet != 0)
			break;

		// get Service Time
		iRet = getInteger(piCustomerServiceTime);
		piCustomerServiceTime++;

		if (iRet != 0)
			break;
	}
	while (true);

	iRet = calcDistances();

	if (iRet != 0)
		return iRet;

	m_bDataLoaded = true;

	return iRet;
}

int InstanceData::read(Vrptw::INSTANCE_t *pInstance)
{
	int iRet, iCustomerCount;
	Vrptw::CUSTOMER_t *pCustomer;
	int *piCustomerXCoord;
	int *piCustomerYCoord;
	int *piCustomerDemand;
	int *piCustomerReadyTime;
	int *piCustomerDueDate;
	int *piCustomerServiceTime;

	cleanup();

	strcpy(m_szName, pInstance->szName);
	m_iVehicleCount = pInstance->iVehicleCount;
	m_iCapacity = pInstance->iCapacity;
	iCustomerCount = m_iCustomerCount = pInstance->iCustomer;
	m_iDepotXCoord = pInstance->iDepotXCoord;
	m_iDepotYCoord = pInstance->iDepotYCoord;
	m_iDepotDueDate = pInstance->iDepotDueDate;

	m_iMinXCoord = m_iMaxXCoord = m_iDepotXCoord;
	m_iMinYCoord = m_iMaxYCoord = m_iDepotYCoord;

	memcpy((char*)&m_KnownSolution, (char*)pInstance->pSolution,
		sizeof(Vrptw::SOLUTION_t));

	// allocate memory for customer data
	m_pCustomerData = (int*)malloc(sizeof(int) * iCustomerCount * 6);

	if (m_pCustomerData == NULL)
	{
		strcpy(m_szError, "Out of mem.");
		return 1;
	}

	m_piCustomerXCoord = m_pCustomerData;
	m_piCustomerYCoord = m_pCustomerData + iCustomerCount;
	m_piCustomerDemand = m_pCustomerData + iCustomerCount * 2;
	m_piCustomerReadyTime = m_pCustomerData + iCustomerCount * 3;
	m_piCustomerDueDate = m_pCustomerData + iCustomerCount * 4;
	m_piCustomerServiceTime = m_pCustomerData + iCustomerCount * 5;

	piCustomerXCoord = m_piCustomerXCoord;
	piCustomerYCoord = m_piCustomerYCoord;
	piCustomerDemand = m_piCustomerDemand;
	piCustomerReadyTime = m_piCustomerReadyTime;
	piCustomerDueDate = m_piCustomerDueDate;
	piCustomerServiceTime = m_piCustomerServiceTime;

	// get customer data
	pCustomer = pInstance->pCustomer;

	while (iCustomerCount--)
	{
		*piCustomerXCoord = pCustomer->iXCoord;
		piCustomerXCoord++;

		if (m_iMinXCoord > pCustomer->iXCoord)
			m_iMinXCoord = pCustomer->iXCoord;
		else if (m_iMaxXCoord < pCustomer->iXCoord)
			m_iMaxXCoord = pCustomer->iXCoord;

		*piCustomerYCoord = pCustomer->iYCoord;
		piCustomerYCoord++;

		if (m_iMinYCoord > pCustomer->iYCoord)
			m_iMinYCoord = pCustomer->iYCoord;
		else if (m_iMaxYCoord < pCustomer->iYCoord)
			m_iMaxYCoord = pCustomer->iYCoord;

		*piCustomerDemand = pCustomer->iDemand;
		piCustomerDemand++;

		*piCustomerReadyTime = pCustomer->iReadyTime;
		piCustomerReadyTime++;

		*piCustomerDueDate = pCustomer->iDueDate;
		piCustomerDueDate++;

		*piCustomerServiceTime = pCustomer->iServiceTime;
		piCustomerServiceTime++;

		pCustomer++;
	}

	iRet = calcDistances();

	if (iRet != 0)
		return iRet;

	m_bDataLoaded = true;

	return 0;
}

int InstanceData::calcDistances()
{
	int iM, iN, iSize;
	double dA, dB;

	iSize = m_iCustomerCount+1;

	m_ppdDistanceMatrix = generate_double_matrix(iSize, iSize);

	if (m_ppdDistanceMatrix == NULL)
	{
		strcpy(m_szError, "Out of mem.");
		return 1;
	}

	for (iM=0; iM<iSize; iM++)
	{
		for (iN=iM; iN<iSize; iN++)
		{
			if (iM == iN)
			{
				m_ppdDistanceMatrix[iM][iN] = 0.0;
				continue;
			}

			if (iN == m_iCustomerCount) // customer <-> depot
			{
				dA = m_piCustomerXCoord[iM];
				dA -= m_iDepotXCoord;

				dB = m_piCustomerYCoord[iM];
				dB -= m_iDepotYCoord;
			}
			else // customer <-> customer 
			{
				dA = m_piCustomerXCoord[iM];
				dA -= m_piCustomerXCoord[iN];

				dB = m_piCustomerYCoord[iM];
				dB -= m_piCustomerYCoord[iN];
			}

			m_ppdDistanceMatrix[iM][iN] = sqrt(dA*dA+dB*dB);
			m_ppdDistanceMatrix[iN][iM] = m_ppdDistanceMatrix[iM][iN];
		}
	}

	return 0;
}

int InstanceData::getNextLine(int *piLineLength)
{
	int iLineLength;

	do
	{
		iLineLength = 0;
		m_pLinePos = m_pDataPos;

		while (m_pDataPos < m_pDataEnd)
		{
			if (*m_pDataPos == '\r')
			{
				m_iLine++;
				m_pDataPos++;

				if (m_pDataPos < m_pDataEnd)
				{
					if (*m_pDataPos == '\n')
						m_pDataPos++;
				}

				break;
			}
			else if (*m_pDataPos == '\n')
			{
				m_iLine++;
				m_pDataPos++;
				break;
			}

			iLineLength++;
			m_pDataPos++;
		}

		// skip empty lines
		while (iLineLength)
		{
			switch (m_pLinePos[iLineLength-1])
			{
			case ' ':
			case '\t':
				iLineLength--;
				continue;
				
			default:
				break;
			}
			
			break;
		}

		if (iLineLength != 0)
		{
			m_pLineEnd = m_pLinePos + iLineLength;

			if (piLineLength != NULL)
				*piLineLength = iLineLength;

			return 0; // line found
		}

		if (m_pDataPos == m_pDataEnd)
		{
			m_pLinePos = NULL;
			return 1; // end of file
		}

		// get next line
	}
	while (true);

	return 0;
}

int InstanceData::getInteger(int *piValue)
{
	char c;
	int iLength;
	char szValue[MAX_INTEGER_LENGTH+1];

	// skip white spaces
	while (m_pLinePos < m_pLineEnd)
	{
		switch (*m_pLinePos)
		{
		case ' ':
		case '\t':
			m_pLinePos++;
			continue;

		default:
			break;
		}

		break;
	}

	// end of line?
	if (m_pLinePos == m_pLineEnd)
	{
		sprintf(m_szError, "Invalid format - integer expected (line = %d).",
			m_iLine);
			
		return 1;
	}

	// get digits
	for (iLength=0; m_pLinePos < m_pLineEnd; iLength++)
	{
		c = *m_pLinePos;

		if (c == ' ' || c == '\t')
			break;

		if (isdigit(c) == 0)
		{
			sprintf(m_szError,
				"Invalid format - integer expected (line = %d).", m_iLine);
				
			return 1;
		}

		if (iLength > MAX_INTEGER_LENGTH)
		{
			sprintf(m_szError,
				"Invalid format - integer has too many digits (line = %d).",
				m_iLine);
				
			return 1;
		}
		
		szValue[iLength] = c;
		m_pLinePos++;
	}

	if (iLength == 0)
	{
		sprintf(m_szError, "Invalid format - integer expected (line = %d).",
			m_iLine);
			
		return 1;
	}

	szValue[iLength] = '\0';

	*piValue = atoi(szValue);

	return 0;
}

int InstanceData::compareNoCase(char *szValue)
{
	int iLength;

	// skip white spaces
	while (m_pLinePos < m_pLineEnd)
	{
		switch (*m_pLinePos)
		{
		case ' ':
		case '\t':
			m_pLinePos++;
			continue;

		default:
			break;
		}

		break;
	}

	// end of line?
	if (m_pLinePos == m_pLineEnd)
		return 1;

	iLength = strlen(szValue);

	if (iLength > (m_pLineEnd-m_pLinePos))
		return 1;
	
#if defined(WIN32) || defined(WIN64)

	if (strnicmp(m_pLinePos, szValue, iLength) != 0)
		return 1;

#else

	if (strncasecmp(m_pLinePos, szValue, iLength) != 0)
		return 1;

#endif

	m_pLinePos += iLength;
	
	return 0;
}
