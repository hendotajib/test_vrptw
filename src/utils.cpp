//
// util.cpp
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
//	31.12.2006  =pd=
//


///// includes /////

#include <stdlib.h>


///// functions /////

// m x n (rows x cols)
int **generate_int_matrix(int iM, int iN)
{
	int i;
	int **ppiMatrix;

	ppiMatrix = (int**)malloc(sizeof(int) * iN * iM + sizeof(int *) * iM);

	if (ppiMatrix == NULL)
	  return NULL;

	for (i=0; i<iM; i++)
		ppiMatrix[i] = (int*)(ppiMatrix + iM) + i * iN;

	return ppiMatrix;
}

// m x n (rows x cols)
double **generate_double_matrix(int iM, int iN)
{
	int i;
	double **ppdMatrix;

	ppdMatrix = (double**)malloc(sizeof(double) * iN * iM + sizeof(double *) * iM);

	if (ppdMatrix == NULL)
		return NULL;

	for (i=0; i<iM; i++)
		ppdMatrix[i] = (double*)(ppdMatrix + iM) + i * iN;

	return ppdMatrix;
}

void IntCopy(int *piDest, const int *piSrc, size_t iCount)
{
	while (iCount--)
		*piDest++ = *piSrc++;
}

void IntSet(int *piDest, int iValue, size_t iCount)
{
	while (iCount--)
		*piDest++ = iValue;
}

int IntCompare(const int *piSrc1, const int *piSrc2, size_t iCount)
{
	while (iCount--)
	{
		if (*piSrc1 != *piSrc2)
		{
			if (*piSrc1 < *piSrc2)
				return -1;

			return 1;
		}

		piSrc1++;
		piSrc2++;
	}

	return 0;
}

void DoubleCopy(double *pdDest, const double *pdSrc, size_t iCount)
{
	while (iCount--)
		*pdDest++ = *pdSrc++;
}

void DoubleSet(double *pdDest, double dValue, size_t iCount)
{
	while (iCount--)
		*pdDest++ = dValue;
}

int DoubleCompare(const double *pdSrc1, const double *pdSrc2, size_t iCount)
{
	while (iCount--)
	{
		if (*pdSrc1 != *pdSrc2)
		{
			if (*pdSrc1 < *pdSrc2)
				return -1;

			return 1;
		}

		pdSrc1++;
		pdSrc2++;
	}

	return 0;
}

