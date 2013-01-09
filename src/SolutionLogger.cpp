//
// SolutionLogger.cpp
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


///// include /////

#include "SolutionLogger.h"
#include "utils.h"

#if !defined(WIN32) && !defined(WIN64)
	#include <sys/time.h>
#endif
#include <string.h>
#include <stdlib.h>


///// classes /////

SolutionLogger::SolutionLogger()
{
	m_vectorLog.clear();
}

SolutionLogger::~SolutionLogger()
{
	cleanup();
}

void SolutionLogger::cleanup()
{
	int i, iSize;
	LOG_t *pLog;

	// parameter
	iSize = m_vectorParameter.size();

	for (i=0; i<iSize; i++)
		delete m_vectorParameter.at(i);

	m_vectorParameter.clear();

	// log
	iSize = m_vectorLog.size();

	for (i=0; i<iSize; i++)
	{
		pLog = m_vectorLog.at(i);
	
		if (pLog->piTours != NULL)
			free(pLog->piTours);

		delete pLog;
	}

	m_vectorLog.clear();
}

int SolutionLogger::start(char *szAlgo, int iToursMaxSize)
{
	cleanup();

	strcpy(m_szAlgo, szAlgo);
	m_iToursMaxSize = iToursMaxSize;

#if defined(WIN32) || defined(WIN64)
	time(&m_timeStart);
	m_dwStartMillis = timeGetTime();
#else
	gettimeofday(&m_timevalStart, NULL);
#endif

	return 0;
}

int SolutionLogger::addParameter(char *szName,
								 int iValue)
{
	PARAMETER_t *pParameter;

	pParameter = new PARAMETER_t;

	memset((char*)pParameter, 0, sizeof(PARAMETER_t));

	strcpy(pParameter->szName, szName);
	pParameter->bInteger = true;
	pParameter->iValue = iValue;

	m_vectorParameter.push_back(pParameter);

	return 0;
}

int SolutionLogger::addParameter(char *szName,
								 double dValue)
{
	PARAMETER_t *pParameter;

	pParameter = new PARAMETER_t;

	memset((char*)pParameter, 0, sizeof(PARAMETER_t));

	strcpy(pParameter->szName, szName);
	pParameter->bInteger = false;
	pParameter->dValue = dValue;

	m_vectorParameter.push_back(pParameter);

	return 0;
}

int SolutionLogger::add(int iVehicleCount,
						double dTotalDistance,
						int *piTours)
{
	int iMilliseconds;
	int *piToursNew;
	LOG_t *pLog;

#if defined(WIN32) || defined(WIN64)
	iMilliseconds = timeGetTime()-m_dwStartMillis;
#else
	struct timeval timevalNow;
	gettimeofday(&timevalNow, NULL);

	iMilliseconds = timevalNow.tv_sec - m_timevalStart.tv_sec;
	iMilliseconds *= 1000;
	iMilliseconds += timevalNow.tv_usec-m_timevalStart.tv_usec;
#endif

	piToursNew = (int*)malloc(sizeof(int)*m_iToursMaxSize);

	if (piToursNew == NULL)
		return -1;

	IntCopy(piToursNew, piTours, m_iToursMaxSize);
	
	pLog = new LOG_t;

	pLog->iVehicleCount = iVehicleCount;
	pLog->dTotalDistance = dTotalDistance;
	pLog->piTours = piToursNew;
	pLog->iMilliseconds = iMilliseconds;

	m_vectorLog.push_back(pLog);

	return 0;
}

int SolutionLogger::add(int iVehicleCount,
						double dTotalDistance,
						int **ppiTourMatrix)
{
	int i, j, iCount, iMilliseconds;
	int *piToursNew;
	LOG_t *pLog;
	int *piNext;

#if defined(WIN32) || defined(WIN64)
	iMilliseconds = timeGetTime()-m_dwStartMillis;
#else
	struct timeval timevalNow;
	gettimeofday(&timevalNow, NULL);

	iMilliseconds = timevalNow.tv_sec - m_timevalStart.tv_sec;
	iMilliseconds *= 1000;
	iMilliseconds += (timevalNow.tv_usec-m_timevalStart.tv_usec) / 1000;
#endif

	piToursNew = (int*)malloc(sizeof(int)*m_iToursMaxSize);

	if (piToursNew == NULL)
		return -1;
	
	piNext = piToursNew;

	for (i=0; i<iVehicleCount; i++)
	{
		iCount = ppiTourMatrix[i][0];

		for (j=1; j<=iCount; j++)
		{
			*piNext = ppiTourMatrix[i][j];
			piNext++;
		}

		*piNext = -1;
		piNext++;
	}
	
	pLog = new LOG_t;

	pLog->iVehicleCount = iVehicleCount;
	pLog->dTotalDistance = dTotalDistance;
	pLog->piTours = piToursNew;
	pLog->iMilliseconds = iMilliseconds;

	m_vectorLog.push_back(pLog);

	return 0;
}

int SolutionLogger::write(char *szFilename)
{
	int i, iSize, iVehicle, iVehicleCount;
	FILE *fp;
	PARAMETER_t *pParameter;
	LOG_t *pLog;
	int *piNext;
	struct tm *pStart;

	fp = fopen(szFilename, "wb");

	if (fp == NULL)
		return -1;

	// head
	fprintf(fp, "# %s\n", m_szAlgo);

#if defined(WIN32) || defined(WIN64)
	pStart = localtime(&m_timeStart);
#else
	time_t timeStart;
	timeStart = m_timevalStart.tv_sec;
	pStart = localtime(&timeStart);
#endif

	fprintf(fp, "# start time: %d-%02d-%02d %02d:%02d:%02d\n", pStart->tm_year+1900, pStart->tm_mon+1, pStart->tm_mday, pStart->tm_hour, pStart->tm_min, pStart->tm_sec);

	// parameters
	fprintf(fp, "# parameters\n");
	
	iSize = m_vectorParameter.size();

	for (i=0; i<iSize; i++)
	{
		pParameter = m_vectorParameter.at(i);

		if (pParameter->bInteger)
			fprintf(fp, "%s = %d\n", pParameter->szName, pParameter->iValue);
		else
			fprintf(fp, "%s = %f\n", pParameter->szName, pParameter->dValue);
	}

	// solutions
	fprintf(fp, "# solutions\n");
	
	iSize = m_vectorLog.size();

	for (i=0; i<iSize; i++)
	{
		pLog = m_vectorLog.at(i);

		iVehicleCount = pLog->iVehicleCount;
		fprintf(fp, "%d;%d;%f;", pLog->iMilliseconds, iVehicleCount,
			pLog->dTotalDistance);

		piNext = pLog->piTours;

		for (iVehicle=0; iVehicle<iVehicleCount; iVehicle++)
		{
			do
			{
				fprintf(fp, "%d;", *piNext);
				piNext++;
			}
			while (*piNext != -1);

			fprintf(fp, " -1;");
			piNext++;
		}

		fprintf(fp, "\n");
	}

	// EOF
	fprintf(fp, "# EOF\n");
	fprintf(fp, "\n");

	return 0;
}
