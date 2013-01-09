//
// SolutionLogger.h
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

#if !defined(_SOLUTIONLOGGER_H_)
#define _SOLUTIONLOGGER_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


///// includes /////

#if defined(WIN32) || defined(WIN64)
	#include <windows.h>
#endif

#include <vector>
#include <time.h>


///// classes /////

class SolutionLogger  
{
public:
	typedef struct LOG_s
	{
		int iMilliseconds;
		int iVehicleCount;
		double dTotalDistance;
		int *piTours;
	}
	LOG_t;

	typedef struct PARAMETER_s
	{
		char szName[33];
		bool bInteger;

		union
		{
			int iValue;
			double dValue;
		};
	}
	PARAMETER_t;

public:
	SolutionLogger();

	virtual ~SolutionLogger();

	int start(char *szAlgo, int iToursMaxSize);

	int addParameter(char *szName, int iValue);

	int addParameter(char *szName, double dValue);

	int add(int iVehicleCount,
			double dTotalDistance,
			int *piTours);

	int add(int iVehicleCount,
			double dTotalDistance,
			int **ppiTourMatrix);

	int write(char *szFilename);

protected:
	void cleanup();

	char m_szAlgo[33];

	int m_iCustomerCount;
	int m_iToursMaxSize;
	std::vector<PARAMETER_t*> m_vectorParameter;
	std::vector<LOG_t*> m_vectorLog;

#if defined(WIN32) || defined(WIN64)
	time_t m_timeStart;
	DWORD m_dwStartMillis;
#else
	struct timeval m_timevalStart;
#endif
};

#endif // _SOLUTIONLOGGER_H_
