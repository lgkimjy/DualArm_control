/////
/////	Cubic Spline
/////
/////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/////
#pragma once

#include <vector>
#include <iomanip>
#include <Eigen/Dense>
#include "ARBMLlib/CommUtil.h"

#ifndef _NO_AXIS
#define _NO_AXIS
constexpr int noAxis = 3;
#endif
constexpr double tolerance=1.0e-13;

enum SplineType
{
	NATURAL_SPLINE = 0,
	CLAMPED_SPLINE
};

namespace SPLINE {
/////
/////	CubicSpline Class
/////

class CCubicSpline
{
private:
	int N;
	Eigen::VectorXd m_knotVec;					//	Knot vec : t(i) < t(i+1) < t(N-1)
	Eigen::MatrixXd m_acc;						//	2nd derivatives at knots
	std::vector<Eigen::Vector3d> m_ctrlPoints;	//	Control points (x_i, y_i), i = 0, ... , N-1

public:
	CCubicSpline() : N(0) {}
	~CCubicSpline() {}

	///	@brief Initialize spline : Given Time limit, control points and spline type
	/// 
	///	@param T_limit : Maximum time
	///	@param ctrl_pts : Control points (x_i, y_i)
	///	@param SpType : Spline type selection (default = Clampled Spline)
	void initSpline(const double& T_limit, const std::vector<Eigen::Vector3d>& ctrl_pts, const int SpType = CLAMPED_SPLINE)
	{
		N = static_cast<int>(ctrl_pts.size());
		if (N < 2) {
			throw("Insufficient data points to interpolate.");
		}
		if (abs(T_limit) <  1e-10) {
			throw("T_limt == 0 !");
		}

		/////	Select Spline Types : Natural or Clamped cubic spline
		Eigen::Vector3d fprime_0 = Eigen::Vector3d::Zero();
		Eigen::Vector3d fprime_N = Eigen::Vector3d::Zero();

		if (SpType == NATURAL_SPLINE) {
			fprime_0 << 1e35, 1e35;
			fprime_N << 1e35, 1e35;
		}

		/////	Compute knot vector : Uniform knot t = [0, T_limit]
		Eigen::VectorXd knotVec = Eigen::VectorXd::LinSpaced(N, 0, T_limit);
		initSpline(knotVec, ctrl_pts, fprime_0, fprime_N);
	}

	///	@brief Initialize spline : Given Time limit, control points and the first derivatives at both ends
	/// 
	///	@param T_limit : Maximum time limit
	///	@param ctrl_pts : Control points (x_i, y_i)
	///	@param prime_0 : The first derivative at t_0 (0 = Clamped spline, 1e35 = Natural spline)
	///	@param prime_N : The first derivative at t_N (0 = Clamped spline, 1e35 = Natural spline)
	void initSpline(const double& T_limit, const std::vector<Eigen::Vector3d>& ctrl_pts,
					const Eigen::Vector3d& prime_0, const Eigen::Vector3d& prime_N)
	{
		N = static_cast<int>(ctrl_pts.size());
		if (N < 2) {
			throw("Insufficient data points to interpolate.");
		}
		if (abs(T_limit) <  1e-10) {
			throw("T_limt == 0 !");
		}

		/////	Compute knot vector : Uniform knot t = [0, T_limit]
		Eigen::VectorXd knotVec = Eigen::VectorXd::LinSpaced(N, 0, T_limit);
		initSpline(knotVec, ctrl_pts, prime_0, prime_N);
	}


	/// @brief Compute second derivative at each control point
	///
	///	@param knots : Knot vector, t_0 < t_1 < ... < t_N
	///	@param ctrl_pts : Control points (x_i, y_i), i = 0, ..., N
	///	@param prime_0 : First derivative at t_0
	///	@param prime_N : First derivative at t_N
	void initSpline(const Eigen::VectorXd& knots, const std::vector<Eigen::Vector3d>& ctrl_pts,
					const Eigen::Vector3d& prime_0, const Eigen::Vector3d& prime_N)
	{
		int i, k;
		double p, qn, sig, un;

		N = static_cast<int>(ctrl_pts.size());
		if (N < 2) {
			throw("Insufficient data points to interpolate.");
		}

		m_knotVec = knots;
		m_ctrlPoints = ctrl_pts;

		m_acc.resize(N, noAxis);
		m_acc.setZero();

		Eigen::VectorXd h(N - 1);
		Eigen::VectorXd u(N - 1);

		for (i = 0; i < N - 1; i++) {
			h(i) = m_knotVec(i + 1) - m_knotVec(i);
		}

		for (int axis = 0; axis < noAxis; axis++) {
			u.setZero();
			if (prime_0(axis) > 0.99e30)		//	Natural cubic spline
				m_acc(0, axis) = u(0) = 0;
			else {								//	Clamped cubic spline
				m_acc(0, axis) = -0.5;
				u(0) = (3.0 / h(0)) * ((m_ctrlPoints[1](axis) - m_ctrlPoints[0](axis)) / h(0) - prime_0(axis));
			}

			for (i = 1; i < N - 1; i++) {
				sig = h(i - 1) / (h(i) + h(i - 1));
				p = sig * m_acc(i - 1, axis) + 2;
				m_acc(i, axis) = (sig - 1.0) / p;
				u(i) = (m_ctrlPoints[i + 1](axis) - m_ctrlPoints[i](axis)) / h(i) - (m_ctrlPoints[i](axis) - m_ctrlPoints[i - 1](axis)) / h(i - 1);
				u(i) = (6.0 * u(i) / (h(i) + h(i - 1)) - sig * u(i - 1)) / p;
			}

			if (prime_N(axis) > 0.99e30)		//	Natural cubic spline
				qn = un = 0;
			else {								//	Clamped cubic spline
				qn = 0.5;
				un = (3.0 / h(N - 2)) * (prime_N(axis) - (m_ctrlPoints[N - 1](axis) - m_ctrlPoints[N - 2](axis)) / h(N - 2));
			}

			m_acc(N - 1, axis) = (un - qn * u(N - 2)) / (qn * m_acc(N - 2, axis) + 1);

			for (k = N - 2; k >= 0; k--)
				m_acc(k, axis) = m_acc(k, axis) * m_acc(k + 1, axis) + u(k);
		}


		//Eigen::Matrix<double, noCtrlPoints, noAxis> f_acc;
		//Eigen::Matrix<double, noCtrlPoints - 1, noAxis> b;
		//Eigen::Matrix<double, noCtrlPoints - 1, noAxis> u;

		//Eigen::Vector<double, noCtrlPoints - 1> h;
		//Eigen::Vector<double, noCtrlPoints> v;
		//Eigen::Vector<double, noCtrlPoints> a, c, d;

		//for (i = 0; i < N - 1; i++) {
		//	h(i) = t_knot(i + 1) - t_knot(i);

		//	for (axis = 0; axis < noAxis; axis++) {
		//		b(i, axis) = (m_ctrlPoints[i + 1](axis) - m_ctrlPoints[i](axis)) / h(i);
		//	}
		//}

		//for (i = 1; i < N - 2; i++) {
		//	v(i) = 2 * (h(i) + h(i - 1));

		//	for (axis = 0; axis < noAxis; axis++) {
		//		u(i, axis) = 6 * (b(i, axis) - b(i - 1, axis));
		//	}
		//}

		//for (axis = 0; axis < noAxis; axis++) {
		//	if (EndCondition == Natural) {
		//		acc(0, 0) = acc(0, 1) = 0;
		//		acc(N - 1, 0) = acc(N - 1, 1) = 0;
		//	}
		//	else {
		//		ypp[0] = -0.5;
		//	}

		//	for (i = 1; i < N - 1; i++) {
		//	}
		//}
	}

	/// @brief Interpolation between control points
	///
	/// @param time : evaluation input point
	/// @param (return) : interpolated 2d-vector at 'time'
	Eigen::Vector3d getPositionAt(const double& time)
	{

		   if (time > m_knotVec[N-1]) {
        // time이 스플라인의 마지막 정의된 시간을 초과하는 경우,
        // 마지막 제어점의 위치 또는 스플라인의 마지막 위치를 반환
		// GPT ver. By Tae Hyun
        return m_ctrlPoints[N-1];
    }

		int k;
		double h, b, a;
		Eigen::Vector3d sp_pos;

		//	Find the interval to which 'x' belongs
		int klo = 0;
		int khi = N - 1;

		

		while (khi - klo > 1) {
			k = (khi + klo) >> 1;
			if (m_knotVec[k] > time) khi = k;
			else klo = k;

		}

		//	Set interval
		h = m_knotVec[khi] - m_knotVec[klo];
		if (abs(h) < tolerance)		throw("Bad m_knotVec input to routine getPositionAt()");

		a = (m_knotVec[khi] - time) / h;
		b = (time - m_knotVec[klo]) / h;

		for (int axis = 0; axis < noAxis; axis++) {
			sp_pos(axis) = a * m_ctrlPoints[klo](axis) + b * m_ctrlPoints[khi](axis)
				+ ((a * a * a - a) * m_acc(klo, axis) + (b * b * b - b) * m_acc(khi, axis)) * (h * h) / 6.0;
		}
		
		return sp_pos;
	}


	/// @brief Evaluate the first derivative of cubic spline
	///
	/// @param time : t-parameter
	/// @return : The first derivative 2d-vector at 'time'
	Eigen::Vector3d getVelocityAt(const double& time) const
	{
		  if (time > m_knotVec[N-1]) {
        // time이 스플라인의 마지막 정의된 시간을 초과하는 경우,
        // 속도를 0으로 반환
		// GPT ver. By Tae Hyun

        return Eigen::Vector3d::Zero();
    }


		int k;
		double h, b, a;
		Eigen::Vector3d sp_vel;

		//	Find the interval to which 'x' belongs
		int klo = 0;
		int khi = N - 1;

		while (khi - klo > 1) {
			k = (khi + klo) >> 1;
			if (m_knotVec[k] > time) khi = k;
			else klo = k;
		}

		//	Set interval
		h = m_knotVec[khi] - m_knotVec[klo];
		if (abs(h) < tolerance)		throw("Bad m_knotVec input to routine getDerivativeAt()");

		a = (m_knotVec[khi] - time) / h;
		b = (time - m_knotVec[klo]) / h;

		for (int axis = 0; axis < noAxis; axis++) {
			sp_vel(axis) = m_ctrlPoints[khi](axis) - m_ctrlPoints[klo](axis)
				+ ((1 - 3 * a * a) * m_acc(klo, axis) + (3 * b * b - 1) * m_acc(khi, axis)) * (h * h) / 6.0;
			sp_vel(axis) = sp_vel(axis) / h;
		}
		return sp_vel;
	}


	Eigen::Vector3d getAccelerationAt(const double& time) const
	{
		int k;
		double h, b, a;
		Eigen::Vector3d sp_acc;

		//	Find the interval to which 'x' belongs
		int klo = 0;
		int khi = N - 1;

		while (khi - klo > 1) {
			k = (khi + klo) >> 1;
			if (m_knotVec[k] > time) khi = k;
			else klo = k;
		}

		//	Set interval
		h = m_knotVec[khi] - m_knotVec[klo];
		if (abs(h) < tolerance)		throw("Bad m_knotVec input to routine getDerivativeAt()");

		a = (m_knotVec[khi] - time) / h;
		b = (time - m_knotVec[klo]) / h;

		for (int axis = 0; axis < noAxis; axis++) {
			sp_acc(axis) = a * m_acc(klo, axis) + b * m_acc(khi, axis);
		}

		return sp_acc;
	}


	///	@brief Get knot vector
	///
	/// @param return : (N + p + 1) dimensional knot vector
	Eigen::VectorXd getKnotVector() { return m_knotVec; }
};


}	///// namespace SPLINE /////