//
// utils.h
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

#ifndef _UTILS_H_
#define _UTILS_H_


///// defines /////

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#if !defined(WIN32) && !defined(WIN64)
	#define __min(a,b)		((a)<(b))?(a):(b)
	#define __max(a,b)		((a)<(b))?(b):(a)
#endif


///// includes //////

#include "MersenneTwister.h"


///// prototypes /////

int **generate_int_matrix(int iM, int iN);
double **generate_double_matrix(int iM, int iN);

void IntCopy(int *piDest, const int *piSrc, size_t iCount);
void IntSet(int *piDest, int iValue, size_t iCount);
int IntCompare(const int *piSrc1, const int *piSrc2, size_t iCount);

void DoubleCopy(double *pdDest, const double *pdSrc, size_t iCount);
void DoubleSet(double *pdDest, double dValue, size_t iCount);
int DoubleCompare(const double *pdSrc1, const double *pdSrc2, size_t iCount);


#endif // _UTILS_H_
