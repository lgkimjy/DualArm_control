//
//	Common Math Utilities
//
#ifndef _COMMUTIL_H_INCLUDED
#define _COMMUTIL_H_INCLUDED

#include <algorithm>
#include <iostream>
#include <list>
#include <stdarg.h>		//	va_end
#include <string.h>
#include <fstream>
#include <memory>

#include <Eigen/Dense>

using namespace std;

#ifndef _SYSREAL_
#define _SYSREAL_
typedef double sysReal;
#endif

#ifndef _REAL_
#define _REAL_
typedef double Real;
#endif


typedef unsigned int Uint;

#undef M_PI
#undef M_PI_2

///// Max. iteration for Algebraic Riccati Equation solvers
constexpr auto max_iteration = 10000;
constexpr auto epsilon = 1.0e-5;

constexpr double M_PI = 3.14159265358979323846;		//	pi
constexpr double M_PI_2 = 1.57079632679489661923;	//	pi/2
constexpr double TINY = 1.0e-40;
constexpr double MachineEps = 2.2204460492503131e-016;	// double precision machine epsilon

constexpr double tolerance=1.0e-13;
constexpr double ZCE = 1.0e-7;
constexpr double DEG = (M_PI / 180.0);
constexpr double RAD = (180.0 / M_PI);


// ============================================================================================
//						Some useful inline macro functions
// ============================================================================================
template <typename T>
inline T SQR(const T(x)) { return T(x*x); }

template <typename T>
inline T CUBE(const T(x)) { return	T(x * x * x); }

template <typename T>
inline T POW(const T(x), const int& y) { T z = 1.0; for (int i = 0; i < y; ++i) z = z * x; return	z; }

template <typename T>
inline T POW(const T(x), const T& y) { T z = 1.0; for (int i = 0; i < y; ++i) z = z * x; return	z; }

template <typename T>
int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

// template <typename T>
// inline const T & MAX(const T &a, const T &b) { return b > a ? (b) : (a); }
// inline float MAX(const double &a, const float &b) { return b > a ? (b) : float(a); }
// inline float MAX(const float &a, const double &b) { return b > a ? float(b) : (a); }
//#define	max( a, b )		(((a) > (b)) ? (a) : (b))

// template <typename T>
// inline const T & MIN(const T &a, const T &b) { return b < a ? (b) : (a); }
// inline float MIN(const double &a, const float &b) { return b < a ? (b) : float(a); }
// inline float MIN(const float &a, const double &b) { return b < a ? float(b) : (a); }
//#define	min( a, b )		(((a) < (b)) ? (a) : (b))

//template <typename T>
//inline T SIGN(const T &a, const T &b) { return (b >= 0) ? abs(a) : -abs(a); }
template <typename T>
inline T SIGN(const T &a, const T &b) { return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a); }
inline float SIGN(const float &a, const double &b) { return b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a); }
inline float SIGN(const double &a, const float &b) { return (float)(b >= 0 ? (a >= 0 ? a : -a) : (a >= 0 ? -a : a)); }

template <typename T>
inline void SWAP(T &a, T &b) { T dum = a; a = b; b = dum; }

inline double D2R(const double(x)) { return (M_PI*x) / 180.0; }
inline double R2D(const double(x)) { return (180.0*x) / M_PI; }


//	Signum Function
inline	double	SGN(const double x)
{
	//	if (abs(x) <= EPS_)	return	0;
	//	else return (x > 0) ? 1 : -1;
	return abs(x) <= MachineEps ? 0 : (x > 0 ? 1 : -1);
}


//
//	Print out Error Message
//
inline void _ErrorMsg(const char *pErrMsg)
{
	cout << endl;
	cout << " *******************************************************" << endl;
	cout << " ***" << endl;
	cout << " *** " << pErrMsg << endl;
	cout << " ***" << endl;
	cout << " *******************************************************" << endl;
	
	int system_pause = system("PAUSE");
	exit(EXIT_FAILURE);
}
#define MATH_ERROR( Error_msg )	_ErrorMsg( Error_msg )


template <typename Derived>
void print_vec(const Eigen::MatrixBase<Derived>& mat, int precision = Eigen::StreamPrecision, int transpose = 0)
{
	if (mat.cols() == 1) {
		const Eigen::IOFormat fmt(precision, 0, "   ", "\n", "[ ", " ]");
		if (transpose == 0)
			cout << mat.format(fmt) << endl;                // print vector in col style
		else
			cout << mat.transpose().format(fmt) << endl;    // print vector in row style
	}
}


template <typename Derived>
void print_mat(const Eigen::DenseBase<Derived>& mat, int precision = 6, int transpose = 0)
{
	Eigen::IOFormat CleanFmt(precision, 0, "   ", "\n", "[ ", " ]");
	if (transpose == 0)
		cout << mat.format(CleanFmt) << endl;
	else
		cout << mat.transpose().format(CleanFmt) << endl;
}

#endif


