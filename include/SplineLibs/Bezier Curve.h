//
//	Compute (N - 1)-th order Bezier Curve by use of the Bernstein basis polynomials
//
//		* Approximate N control points
//		* Bezier Curve & its derivative
//==============================================================================
//	Copyright by Y. Oh @ 2023
//
#pragma once

#include <vector>
#include <iomanip>
#include <Eigen/Dense>
#include "ARBMLlib/CommUtil.h"

using namespace std;
using namespace Eigen;

namespace SPLINE {
/////
/////	Bezier Curves for N control points by use of Bernstein basis polynomials.
/////
class CBezierCurve
{
private:
	int N;			//	Order of Bernstein basis polynomials = # ctrl points - 1
	double MaxT;
	double scale;
	std::vector<Eigen::Vector3d> m_ctrlPoints;

	///	@brief Compute binomial coefficient of the x^k term n >= k >= 0, (n ; k)
	///
	///	@param n : order of binomial polynomial expansion
	///	@param k : x^k term
	/// @param (return) : a coefficient of the x^k term
	double binomialCoeff(const int n, int k) const
	{
		if (k < 0 || k > n) return 0;
		if (k == 0 || k == n) return 1;

		double coeff = 1;
		for (int i = 1; i <= k; i++) {
			coeff *= static_cast<double>(n - i + 1) / i;
		}
		return coeff;
	}

	///	@brief Evaluate the Bernstein basis polynomial at 't'
	///
	/// @param order : order of bernstein basis polynomial
	/// @param k : k-th term, i.e, x^k
	/// @param t : Given t-parameter t = [0, 1]
	/// @param (return) value of the Bernstein basis polynomial at t & k
	double BernsteinPolyAt(const int& order, const int& k, const double& t) const
	{
		return (binomialCoeff(order, k) * pow((1 - t), (order - k)) * pow(t, k));
	}


public:
	CBezierCurve() : N(0), MaxT(0), scale(0) {}
	~CBezierCurve() {}

	//	The Bezier curve must be initialized with the expected number of points
	void initCurve(const double& T_limit, const std::vector<Eigen::Vector3d>& controlPoints_)
	{
		N = static_cast<int>(controlPoints_.size() - 1);
		if (N < 1) {
			throw("Order of Berbstein polynomials < 1 !");
		}
		if (abs(T_limit) < tolerance) {
			throw("T_limit == 0 !");
		}

		MaxT = T_limit;
		scale = 1 / MaxT;
		m_ctrlPoints = controlPoints_;
	}

	/// @brief Get Bezier Curve at 't'
	/// @param time_ : Current time (evaluation point)
	Eigen::Vector3d getPositionAt(const double& time_) const
	{
		double t = scale * time_;	//	Normalization, t = [0, 1]
		Eigen::Vector3d p_vec(0, 0,0);

		for (int k = 0; k <= N; k++) {
			p_vec += BernsteinPolyAt(N, k, t) * m_ctrlPoints[k];
		}

		return p_vec;
	}

	//	@brief Get 1st Derivative of Bezier Curves (Velocity)
	//	@brief Note: derivative weights/control points are not actual control points.
	//	@param time_ : Current time (evaluation point)
	Eigen::Vector3d getVelocityAt(const double& time_) const
	{
		double t = scale * time_;	//	Normalization
		Eigen::Vector3d v_vec(0, 0,0 );

		for (int k = 0; k <= N - 1; k++) {
			v_vec += BernsteinPolyAt(N - 1, k, t) * (m_ctrlPoints[k + 1] - m_ctrlPoints[k]) * N;
		}

		return (scale * v_vec);		//	Velocity scaling !!!
	}


	//	@brief Get 2nd Derivative of Bezier Curves (Acceleration)
	//	@brief Note: derivative weights/control points are not actual control points.
	//	@param time_ : Current time (evaluation point)
	Eigen::Vector3d getAccelerationAt(const double& time_) const
	{
		double t = scale * time_;	//	Normalization
		Eigen::Vector3d a_vec(0, 0, 0);

		for (int k = 0; k <= N - 2; k++) {
			a_vec += BernsteinPolyAt(N - 2, k, t) * (m_ctrlPoints[k + 2] - 2 * m_ctrlPoints[k + 1] + m_ctrlPoints[k]) * N * (N - 1);
		}

		return (SQR(scale) * a_vec);		//	Velocity scaling !!!
	}


	/// @brief Get binomial coefficients
	/// @return (N + 1) dimensional coeff. vector
	Eigen::VectorXd getBinomialCoeff(int n)
	{
		Eigen::VectorXd biCoeff(n + 1);

		for (int k = 0; k <= n; k++) {
			biCoeff(k) = binomialCoeff(n, k);
		}

		return biCoeff;
	}
};


}	///// namespace SPLINE /////