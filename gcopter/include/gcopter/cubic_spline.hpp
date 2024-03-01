#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include "cubic_curve.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <vector>

namespace cubic_spline
{

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);

            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            return;
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        // Compulsory nullptr initialization here
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; ++k)
            {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; ++i)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; ++j)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; ++i)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solve(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; ++j)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; ++i)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; --j)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; ++i)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        template <typename EIGENMAT>
        inline void solveAdj(EIGENMAT &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; ++j)
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; ++i)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; --j)
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; ++i)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
        }
    };

    class CubicSpline
    {
    public:
        CubicSpline() = default;
        ~CubicSpline() { A.destroy(); }

    private:
        int N;
        Eigen::Vector2d headP;
        Eigen::Vector2d tailP;
        BandedSystem A;
        Eigen::MatrixX2d b;

    public:
        inline void setConditions(const Eigen::Vector2d &headPos,
                                  const Eigen::Vector2d &tailPos,
                                  const int &pieceNum)
        {
            // TODO
            headP = headPos;
            tailP = tailPos;
            N = pieceNum - 1;
            A.create(N, 1, 1);

            // 将A改称标准的矩阵形式
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < N; j++)
                {
                    if (i == j)
                    {
                        A(i, j) = 4.0;
                    }
                    if (i == (j - 1) || i == (j + 1))
                    {
                        A(i, j) = 1.0;
                    }
                }
            }
            A.factorizeLU();
            return;
        }

        inline void setInnerPoints(const Eigen::Ref<const Eigen::Matrix2Xd> &inPs)
        {
            // TODO
            // std::cout << inPs << std::endl;
            // std::cout << "***************************" << std::endl;
            b.resize(inPs.cols(), inPs.rows());
            // std::cout << "testtsettstetstetste" << std::endl;
            int col = inPs.cols();
            // 将b改成 b = [x2 - x0; x3 - x1; x4 - x2; ]
            for (int i = 0; i < col; i++)
            {
                // std::cout << "i = " << i << std::endl;
                if (i == 0)
                {
                    b.row(i) = (inPs.col(i + 1) - headP.col(0)).transpose();
                    // std::cout << inPs.col(i + 1) << std::endl;
                    // std::cout << headP.col(0) << std::endl;
                }
                else if (i == col - 1)
                {
                    b.row(i) = (tailP.col(0) - inPs.col(i - 1)).transpose();
                    // std::cout << tailP.col(0) << std::endl;
                    // std::cout << inPs.col(i - 1) << std::endl;
                }
                else
                {
                    b.row(i) = (inPs.col(i + 1) - inPs.col(i - 1)).transpose();
                    // std::cout << inPs.col(i + 1) << std::endl;
                    // std::cout << inPs.col(i - 1) << std::endl;
                }
            }
            // std::cout << "***" << std::endl;
            b = 3* b;
            // std::cout << b << std::endl;
            A.solve(b);
            // std::cout << b << std::endl;
            return;
        }

        inline void getCurve(CubicCurve &curve) const
        {
            // TODO
            return;
        }

        inline void getStretchEnergy(double &energy) const
        {
            // TODO
            return;
        }

        inline const Eigen::MatrixX2d &getCoeffs(void) const
        {
            return b;
        }

        inline void getGrad(Eigen::Ref<Eigen::MatrixX2d> gradByPoints) const
        {
            // TODO
            // 返回D的导数信息，第一列为对x求导，第二列为对y求导
            Eigen::VectorXd gradByPoint;
            gradByPoint.resize(gradByPoints.rows());
            int hahaha;
            
            for (int col_id = 0; col_id < gradByPoints.cols(); col_id++)
            {
                gradByPoint = gradByPoints.col(col_id);
                // std::cout << gradByPoint << std::endl;
                // std::cout << "****" << std::endl;
                A.solve(gradByPoint);
                gradByPoints.col(col_id) = gradByPoint;
                // std::cout << gradByPoint << std::endl;
                // std::cin >> hahaha;
            }
        }
    };
}

#endif
