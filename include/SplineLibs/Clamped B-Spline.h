/////
/////	Clamped Cubic B-Spline & its Derivatives
/////
#pragma once

#include <vector>
#include <iomanip>
#include <Eigen/Dense>
#include "CommUtil.h"

using namespace std;
using namespace Eigen;

#ifndef _NO_AXIS
#define _NO_AXIS
constexpr int noAxis = 3;
#endif

namespace SPLINE {
/////
/////	Clamped B-spline Class 
/////
class CClampedBSpline
{
private:
	int N;
	int dimKnots;
	int order_Sp;
	double MaxT;
	double scale;
	const double T_N = 1.;

	Eigen::VectorXd m_Knots;
	Eigen::MatrixXd m_derivCoeff;	//	Coefficient matrix for derivative
	std::vector<Eigen::Vector3d> m_ctrlPoints;


	/// @brief Initialize clamped knot vector : Uniform knot vector
	void initKnotVector()
	{
		for (int i = 0; i < order_Sp; i++) {
			m_Knots(i) = 0;
			m_Knots.tail(i + 1)[0] = T_N;
		}
		m_Knots.segment(order_Sp, dimKnots - 2 * order_Sp) = Eigen::VectorXd::LinSpaced(dimKnots - 2 * order_Sp, 0.0, T_N);
	}

	///	@brief Cox-de Boor recursive basis function evaluation : N_{i, p}(t)
	///
	/// @param i : i-th polynomial (i = 0, 1, ..., N - 1)
	/// @param p : degree of spline = 3
	/// @param t : evaluation point
	/// @param return : i-th B-spline of degree p with knots t
	double Cox_deBoorBasis(const int& i, const int& p, const double& t) const
	{
		if (p == 0) {
			return (m_Knots(i) <= t && t < m_Knots(i + 1)) ? 1 : 0;
		}

		double denom1 = m_Knots(i + p) - m_Knots(i);
		double denom2 = m_Knots(i + p + 1) - m_Knots(i + 1);

		double term1 = (denom1 > 0) ? ((t - m_Knots(i)) / denom1) * Cox_deBoorBasis(i, p - 1, t) : 0;
		double term2 = (denom2 > 0) ? ((m_Knots(i + p + 1) - t) / denom2) * Cox_deBoorBasis(i + 1, p - 1, t) : 0;

		return (term1 + term2);
	}


	double Cox_deBoorBasisDerivative(const int& i, const int& p, const double& t, const int& order) const
	{
		if (order == 0) {
			return Cox_deBoorBasis(i, p, t);
		}

		double denom1 = m_Knots(i + p) - m_Knots(i);
		double denom2 = m_Knots(i + p + 1) - m_Knots(i + 1);

		double term1 = (denom1 > 0) ? p / denom1 * Cox_deBoorBasisDerivative(i, p - 1, t, order - 1) : 0.0;
		double term2 = (denom2 > 0) ? p / denom2 * Cox_deBoorBasisDerivative(i + 1, p - 1, t, order - 1) : 0.0;

		return term1 - term2;
	}

public:
	//	Constructor
	CClampedBSpline(const int& SplineOrder = 3) : N(0), MaxT(1), scale(1), dimKnots(0), order_Sp(SplineOrder) {}
	~CClampedBSpline() {}


	void initSpline(const double& T_limit, const std::vector<Eigen::Vector3d>& controlPoints)
	{
		if (abs(T_limit) < tolerance) {
			throw("T_limit == 0");
		}
		MaxT = T_limit;
		scale = 1 / MaxT;

		N = static_cast<int>(controlPoints.size());
		if (N < order_Sp + 1) {
			throw("Number of control points should be at least order of spline + 1");
		}

		m_ctrlPoints = controlPoints;

		dimKnots = N + order_Sp + 1;
		m_Knots.resize(dimKnots);
		initKnotVector();

		//	Initialize the coefficient matrix for computing derivatives
		m_derivCoeff.resize(N - 1, noAxis);
		for (int i = 0; i < N - 1; i++) {
			double denom = m_Knots(i + order_Sp + 1) - m_Knots(i + 1);

			for (int axis = 0; axis < noAxis; axis++) {
				m_derivCoeff(i, axis) = (denom > 0) ? (order_Sp * (m_ctrlPoints[i + 1](axis) - m_ctrlPoints[i](axis)) / denom) : 0.;
			}
		}
	}


	/////
	/////	Compute the B-spline curve value at 't'
	/////
	Eigen::Vector3d getPositionAt(const double& time) const
	{
		double tt = time * scale;
		Eigen::Vector3d sp_pos(0, 0);

		for (int i = 0; i < N; i++) {
			double basis = Cox_deBoorBasis(i, order_Sp, tt);
			sp_pos += basis * m_ctrlPoints[i];
		}
		return sp_pos;
	}


	///	@brief Compute n-th derivatives of B-spline curve at 't'
	///	@param order : order of derivative (defaut = 1)
	///	@param (return) : N-th derivative value of the curve at 't'
	Eigen::Vector3d getDerivativeAt(const double& time, const int deriv_order = 1) const
	{
		double tt = time * scale;
		Eigen::Vector3d result(0, 0);

		for (int i = 0; i < N; i++) {
			double basis = Cox_deBoorBasisDerivative(i, order_Sp, tt, deriv_order);
			result += basis * m_ctrlPoints[i];
		}
		return (result * pow(scale, deriv_order));
	}


	///	@brief Compute the first derivative of the B-spline curve at 't'
	///	@param (return) : Tangent value at 't'
	Eigen::Vector3d getTangentAt(const double& time) const
	{
		double tt = time * scale;
		Eigen::Vector2d result(0, 0);

		for (int i = 0; i < N - 1; i++) {
			double basis = Cox_deBoorBasis(i + 1, order_Sp - 1, tt);
			result += basis * m_derivCoeff.row(i);
		}
		return (result * scale);
	}

	///	@brief Get normalized knot vector
	/// @param return : (N + p + 1) dimensional knot vector
	Eigen::VectorXd getKnotVector() { return m_Knots; }
};

}