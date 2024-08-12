//
//	Hermite Spline
//
#pragma once

#include <vector>
#include <iomanip>
#include <Eigen/Dense>
#include "CommUtil.h"

using namespace std;
using namespace Eigen;


enum TangentChoice {
	CatmullRom_Spline = 0,
	FiniteDifference,
	Cardinal_Spline
};


namespace SPLINE {
/////
/////	Cubic Hermite Spline interpolation class
/////
class CCubicHermite
{
private:
	int N;
	Eigen::VectorXd m_knotVec;
	std::vector<Eigen::Vector3d> m_ctrlPoints;
	std::vector<Eigen::Vector3d> m_deriv;

public:
	CCubicHermite() : N(0) {}
	~CCubicHermite() {}


	///	@brief Initialize Spline : Given knot vector, position and tangent values of all control points.
	///
	///	@param ctrl_pos : n control points coordinates
	///	@param ctrl_vel : n tangent values of knots
	///	@param Knots_ : n-dimensional knot vector (t0 < t1 < ... < t_N)
	void initSpline(const std::vector<Eigen::Vector3d>& ctrl_pos, const std::vector<Eigen::Vector3d>& ctrl_vel, const Eigen::VectorXd& Knots_)
	{
		N = static_cast<int>(ctrl_pos.size());
		if (N < 3) {
			throw std::runtime_error("Insufficient data points to interpolate.");
		}

		m_deriv = ctrl_vel;
		m_knotVec = Knots_;
		m_ctrlPoints = ctrl_pos;
	}


	///	@brief Initialize Spline : Given time limit and tangent values of both ends
	///
	/// @param MaxT : Time limit
	/// @param ctrl_pos : Control points (x_i, y_i)
	/// @param vel_i : Tangent value at t_0
	/// @param vel_f : Tangent value at t_N
	/// @param TangentType : Catmull-Rom spline / finite difference / Cardinal spline
	/// @param tension : default = 0
	void initSpline(const double& MaxT, const std::vector<Eigen::Vector3d>& ctrl_pos, const Eigen::Vector3d& vel_i, const Eigen::Vector3d& vel_f,
					const int& TangentType = CatmullRom_Spline, const double& tension = 0)
	{
		if (abs(MaxT) < tolerance) {
			throw("MaxT == 0 !");
		}

		N = static_cast<int>(ctrl_pos.size());
		if (N < 3) {
			throw std::runtime_error("Insufficient data points to interpolate.");
		}

		/////	Generate knot vector : Uniform knot
		Eigen::VectorXd knot_vec = Eigen::VectorXd::LinSpaced(N, 0, MaxT);
		initSpline(ctrl_pos, vel_i, vel_f, knot_vec, TangentType, tension);
	}


	/// @brief Initialize Spline : Given control points, knots and tanget values of both ends.
	///
	/// @param ctrl_pos : Control points (x_i, y_i)
	/// @param vel_i : Tangent value at t_0
	/// @param vel_f : Tangent value at t_N
	/// @param TangentType : Catmull-Rom spline / finite difference / Cardinal spline
	/// @param tension : default = 0
	void initSpline(const std::vector<Eigen::Vector3d>& ctrl_pos, const Eigen::Vector3d& vel_i, const Eigen::Vector3d& vel_f,
					const Eigen::VectorXd& Knots_, const int& TangentType = CatmullRom_Spline, const double& tension = 0)
	{
		N = static_cast<int>(ctrl_pos.size());

		if (N < 3) {
			throw std::runtime_error("Insufficient data points to interpolate.");
		}

		m_knotVec = Knots_;
		m_ctrlPoints = ctrl_pos;

		//	Compute tangent value of internal knots : Catmull-Rom spline, Finite difference, Cardinal spline
		m_deriv.resize(N, Eigen::Vector3d::Zero());

		m_deriv[0] = vel_i;
		for (int i = 1; i < N - 1; i++) {
			if (TangentType == CatmullRom_Spline) {			//	Catmull-Rom spline
				m_deriv[i] = (1 - tension) * (m_ctrlPoints[i + 1] - m_ctrlPoints[i - 1]) / 2;
			}
			else if (TangentType == FiniteDifference) {		//	Finite difference
				Eigen::Vector3d term1 = (m_ctrlPoints[i + 1] - m_ctrlPoints[i]) / (m_knotVec(i + 1) - m_knotVec(i));
				Eigen::Vector3d term2 = (m_ctrlPoints[i] - m_ctrlPoints[i - 1]) / (m_knotVec(i) - m_knotVec(i - 1));
				m_deriv[i] = (term1 + term2) / 2;
			}
			else if (TangentType == Cardinal_Spline) {		//	Cardinal spline
				m_deriv[i] = (1 - tension) * (m_ctrlPoints[i + 1] - m_ctrlPoints[i - 1]) / (m_knotVec(i + 1) - m_knotVec(i - 1));
			}
		}
		m_deriv[N - 1] = vel_f;

		m_deriv.shrink_to_fit();
	}


	//	Get interpolate values at a given point 'tt'
	Eigen::Vector3d getPositionAt(const double& tt) const
	{
		int index = 0;
		Eigen::Vector3d p;

		//	Find the interval to which 'tt' belongs
		for (int i = 0; i < N - 1; i++) {
			if (tt >= m_knotVec[i] && tt <= m_knotVec[i + 1]) {
				index = i;
				break;
			}
		}

		double h = m_knotVec[index + 1] - m_knotVec[index];

		if (abs(h) < tolerance)		throw("Bad knots input to routine getPositionAt()");

		double t = (tt - m_knotVec[index]) / h;

		//	Hermite basis functions
		double h00 = 2 * t * t * t - 3 * t * t + 1;
		double h10 = t * t * t - 2 * t * t + t;
		double h01 = 3 * t * t - 2 * t * t * t;
		double h11 = t * t * t - t * t;

		p = h00 * m_ctrlPoints[index] + h10 * h * m_deriv[index] + h01 * m_ctrlPoints[index + 1] + h11 * h * m_deriv[index + 1];

		return p;
	}


	//	Get tangent values at a given point 'tt'
	Eigen::Vector3d getVelocityAt(const double& tt) const
	{
		int index = 0;
		Eigen::Vector3d vel;

		//	Find the interval to which 'tt' belongs
		for (int i = 0; i < N - 1; i++) {
			if (tt >= m_knotVec[i] && tt <= m_knotVec[i + 1]) {
				index = i;
				break;
			}
		}

		double h = m_knotVec[index + 1] - m_knotVec[index];

		if (abs(h) < tolerance)		throw("Bad knots input to routine getDerivativeAt()");

		double t = (tt - m_knotVec[index]) / h;

		//	Derivative of Hermite basis functions
		double hp00 = 6 * t * (t - 1);
		double hp10 = 3 * t * t - 4 * t + 1;
		double hp01 = 6 * t * (1 - t);
		double hp11 = t * (3 * t - 2);

		vel = hp00 * m_ctrlPoints[index] + hp10 * h * m_deriv[index] + hp01 * m_ctrlPoints[index + 1] + hp11 * h * m_deriv[index + 1];
		return (vel / h);
	}


	Eigen::Vector3d getAccelerationAt(const double& tt) const
	{
		int index = 0;
		Eigen::Vector3d acc;

		//	Find the interval to which 'tt' belongs
		for (int i = 0; i < N - 1; i++) {
			if (tt >= m_knotVec[i] && tt <= m_knotVec[i + 1]) {
				index = i;
				break;
			}
		}

		double h = m_knotVec[index + 1] - m_knotVec[index];

		if (abs(h) < tolerance)		throw("Bad knots input to routine getDerivativeAt()");

		double t = (tt - m_knotVec[index]) / h;

		//	Derivative of Hermite basis functions
		double hpp00 = 12 * t - 6;
		double hpp10 = 6 * t - 4;
		double hpp01 = 6 - 12 * t;
		double hpp11 = 6 * t - 2;

		acc = hpp00 * m_ctrlPoints[index] + hpp10 * h * m_deriv[index] + hpp01 * m_ctrlPoints[index + 1] + hpp11 * h * m_deriv[index + 1];
		return (acc / SQR(h));
	}

	///	@brief Get normalized knot vector
	///
	/// @param return : (N + p + 1) dimensional knot vector
	Eigen::VectorXd getKnotVector() { return m_knotVec; }
};

}	///// namespace SPLINE /////