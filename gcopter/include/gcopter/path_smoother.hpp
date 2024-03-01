#ifndef PATH_SMOOTHER_HPP
#define PATH_SMOOTHER_HPP

#include "cubic_spline.hpp"
#include "lbfgs.hpp"

#include <Eigen/Eigen>

#include <cmath>
#include <cfloat>
#include <iostream>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <time.h>

namespace path_smoother
{

    class PathSmoother
    {
    private:
        cubic_spline::CubicSpline cubSpline;

        int pieceN;
        Eigen::Matrix3Xd diskObstacles;
        double penaltyWeight;
        Eigen::Vector2d headP;
        Eigen::Vector2d tailP;
        Eigen::Matrix2Xd points;
        Eigen::Matrix2Xd gradByPoints;

        lbfgs::lbfgs_parameter_t lbfgs_params;

    private:
        static inline double costFunction(void *ptr,
                                          const Eigen::VectorXd &x,
                                          Eigen::VectorXd &g)
        {
            // TODO
            // x的格式为 x = {x1, x2, ..., xn-1}, 其中，x1 = {x11, x12}
            // 把x的格式改为x_cost = {x1 - x0; x2 - x1; x3 - x2; ...; xn - xn-1}
            // g的格式改为 g_1 = {D0, D2, ..., Dn-1}, g_2 = {D1, D2, ..., Dn}
            Eigen::VectorXd x_cost, x_cost1, x_cost2;
            x_cost.resize(2 * ((PathSmoother *)ptr)->pieceN);
            x_cost1.resize(2 * ((PathSmoother *)ptr)->pieceN);
            x_cost2.resize(2 * ((PathSmoother *)ptr)->pieceN);

            x_cost1 << x, ((PathSmoother *)ptr)->tailP(0), ((PathSmoother *)ptr)->tailP(1);
            x_cost2 << ((PathSmoother *)ptr)->headP(0), ((PathSmoother *)ptr)->headP(1), x;
            x_cost = x_cost1 - x_cost2;
            // std::cout << x << std::endl;
            // std::cout << "************" << std::endl;
            // std::cout << x_cost1 << std::endl;
            // std::cout << "************" << std::endl;
            // std::cout << x_cost2 << std::endl;
            // std::cout << "************" << std::endl;
            // std::cout << x_cost << std::endl;

            // 求各个D
            Eigen::Matrix2Xd current_points; // = ((PathSmoother*)ptr)->points;
            current_points.resize(2, ((PathSmoother *)ptr)->pieceN - 1);
            int current_points_temp = 0;
            for (int col_temp = 0; col_temp < (((PathSmoother *)ptr)->pieceN - 1); col_temp++)
            {
                for (int row_temp = 0; row_temp < 2; row_temp++)
                {
                    current_points(row_temp, col_temp) = x(current_points_temp);
                    current_points_temp++;
                }
            }
            // std::cout << "***" << std::endl;
            // std::cout << x << std::endl;
            // std::cout << "***" << std::endl;
            // std::cout << current_points << std::endl;
            // std::cin >> current_points_temp;
            // ((PathSmoother*)ptr)->cubSpline.setInnerPoints(((PathSmoother*)ptr)->points);
            ((PathSmoother *)ptr)->cubSpline.setInnerPoints(current_points);
            Eigen::MatrixX2d D = ((PathSmoother *)ptr)->cubSpline.getCoeffs();
            int hahaha;
            // std::cout << D << std::endl;
            Eigen::Matrix2Xd D_trans;
            D_trans.resize(2, D.rows());
            D_trans = D.transpose();
            // std::cout << D << std::endl;
            // std::cout << "***" << std::endl;
            // std::cout << D_trans << std::endl;
            Eigen::Map<Eigen::VectorXd> D_cost_temp(D_trans.data(), D_trans.size());
            // std::cout << D_cost_temp << std::endl;
            Eigen::VectorXd D_with_0, D_with_n;
            D_with_0.resize(D_cost_temp.size() + 2);
            D_with_n.resize(D_cost_temp.size() + 2);
            D_with_0 << 0, 0, D_cost_temp;
            D_with_n << D_cost_temp, 0, 0;
            // std::cout << D_with_0 << std::endl;
            // std::cout << "****" << std::endl;
            // std::cout << D_with_n << std::endl;
            // std::cout << "***" << std::endl;
            // std::cout << x_cost << std::endl;

            // 获得当前点cost
            double cost = 0.0;
            cost = 12 * x_cost.transpose() * (x_cost - D_with_0 - D_with_n);
            // std::cout << "cost = " << cost << std::endl;
            cost = cost + 4 * D_with_0.transpose() * D_with_0 + 4 * D_with_n.transpose() * D_with_0 + 4 * D_with_n.transpose() * D_with_n;
            // std::cout << "cost = " << cost << std::endl;
            // 加入和障碍物的cost
            Eigen::Matrix3Xd obstacle_message = ((PathSmoother *)ptr)->diskObstacles;

            // std::cout << obstacle_message << std::endl;
            // std::cout << current_points << std::endl;
            for (int point_id = 0; point_id < current_points.cols(); point_id++)
            {
                Eigen::VectorXd point_temp;
                point_temp.resize(current_points.rows());
                point_temp = current_points.col(point_id);
                for (int obstacle_id = 0; obstacle_id < obstacle_message.cols(); obstacle_id++)
                {
                    double distance;

                    // 取出障碍物信息
                    double radius;
                    Eigen::VectorXd obstacle_point;
                    obstacle_point.resize(2);
                    obstacle_point(0) = obstacle_message(0, obstacle_id);
                    obstacle_point(1) = obstacle_message(1, obstacle_id);
                    radius = obstacle_message(2, obstacle_id);

                    // 计算距离
                    distance = sqrt((point_temp - obstacle_point).transpose() * (point_temp - obstacle_point));

                    // 判断potential的具体数值
                    if (distance >= radius) // 如果与障碍物距离大于障碍物半径
                    {
                        cost += 0;
                    }
                    else
                    {
                        // cost += 1000 * (radius - distance);
                        cost += 10 * (radius - distance);
                    }
                }
            }
            // std::cout << "cost = " << cost << std::endl;

            // 计算当前点的梯度信息
            // std::cout << ((PathSmoother*)ptr)->points << std::endl;
            // 将x_cost改成两列的形式，第一列为x信息，第二列为y的信息
            g.resize((((PathSmoother *)ptr)->pieceN - 1) * 2);
            Eigen::MatrixX2d x_col_form;
            x_col_form.resize(((PathSmoother *)ptr)->pieceN, 2);
            int temp = 0;
            for (int row_id = 0; row_id < ((PathSmoother *)ptr)->pieceN; row_id++)
            {
                for (int col_id = 0; col_id < 2; col_id++)
                {
                    x_col_form(row_id, col_id) = x_cost(temp);
                    temp++;
                }
            }
            // std::cout << x_cost << std::endl;
            // std::cout << "******" << std::endl;
            // std::cout << x_grad << std::endl;
            // std::cin >> temp;
            temp = 0;
            double norm = 0;
            for (int col_id = 0; col_id < current_points.cols(); col_id++)
            {
                Eigen::MatrixX2d grad_D_col_i; // 第i个点D的导数, 第1列为对x求导，第2列为对y求导
                Eigen::VectorXd temp_vector;
                Eigen::MatrixX2d grad_X_col_i; // 第i个点X的导数，第1列为对x求导，第2列为对y求导
                Eigen::VectorXd temp_X_vector;

                grad_D_col_i.resize(((PathSmoother *)ptr)->pieceN - 1, 2);
                temp_vector.resize(((PathSmoother *)ptr)->pieceN - 1);
                grad_X_col_i.resize(((PathSmoother *)ptr)->pieceN, 2);
                temp_X_vector.resize(((PathSmoother *)ptr)->pieceN);
                grad_D_col_i.fill(0.0);
                temp_vector.fill(0.0);
                grad_X_col_i.fill(0.0);
                temp_X_vector.fill(0.0);
                if (col_id == 0)
                {
                    temp_vector(col_id + 1) = -1.0;
                }
                else if (col_id == (current_points.cols() - 1))
                {
                    temp_vector(col_id - 1) = 1.0;
                }
                else
                {
                    temp_vector(col_id - 1) = 1.0;
                    temp_vector(col_id + 1) = -1.0;
                }
                grad_D_col_i.col(0) = 3 * temp_vector;
                grad_D_col_i.col(1) = 3 * temp_vector;
                // std::cout << grad_D_col_i << std::endl;
                // std::cin >> hahaha;
                ((PathSmoother *)ptr)->cubSpline.getGrad(grad_D_col_i);
                // std::cout << col_id << std::endl;
                // std::cout << grad_D_col_i << std::endl;

                // 障碍物对x_cost产生的导数
                Eigen::Vector2d grad_obstacle;
                grad_obstacle.fill(0.0);
                Eigen::Vector2d x_temp;
                x_temp = current_points.col(col_id);
                for (int obstacle_id = 0; obstacle_id < obstacle_message.cols(); obstacle_id++)
                {
                    double radius;
                    Eigen::VectorXd obstacle_point;
                    double distance;

                    obstacle_point.resize(2);
                    obstacle_point(0) = obstacle_message(0, obstacle_id);
                    obstacle_point(1) = obstacle_message(1, obstacle_id);
                    radius = obstacle_message(2, obstacle_id);
                    distance = sqrt((x_temp - obstacle_point).transpose() * (x_temp - obstacle_point));
                    if (distance >= radius)
                    {
                        grad_obstacle(0) = grad_obstacle(0) + 0.0;
                        grad_obstacle(1) = grad_obstacle(1) + 0.0;
                    }
                    else
                    {
                        double grad_obstacle_0, grad_obstacle_1;
                        // grad_obstacle_0 = -1000.0 * (x_temp(0) - obstacle_point(0)) / distance;
                        // grad_obstacle_1 = -1000.0 * (x_temp(1) - obstacle_point(1)) / distance;
                        grad_obstacle_0 = -10.0 * (x_temp(0) - obstacle_point(0)) / distance;
                        grad_obstacle_1 = -10.0 * (x_temp(1) - obstacle_point(1)) / distance;
                        grad_obstacle(0) = grad_obstacle(0) + grad_obstacle_0;
                        grad_obstacle(1) = grad_obstacle(1) + grad_obstacle_1;
                    }
                }

                // x_cost对应的导数
                temp_X_vector(col_id) = 1;
                temp_X_vector(col_id + 1) = -1;
                grad_X_col_i.col(0) = temp_X_vector;
                grad_X_col_i.col(1) = temp_X_vector;
                // std::cout << "XXXXXXXXXXXXXXXXXXXXX" << std::endl;
                // std::cout << grad_X_col_i << std::endl;

                for (int row_id = 0; row_id < current_points.rows(); row_id++)
                {
                    double gradient = 0.0;
                    Eigen::VectorXd x_temp;
                    Eigen::VectorXd D_1, D_2, grad_D1, grad_D2;
                    D_1.resize(((PathSmoother *)ptr)->pieceN);
                    D_2.resize(((PathSmoother *)ptr)->pieceN);
                    grad_D1.resize(((PathSmoother *)ptr)->pieceN);
                    grad_D2.resize(((PathSmoother *)ptr)->pieceN);
                    x_temp.resize(((PathSmoother *)ptr)->pieceN);

                    if (row_id == 0) // 对x的导数信息
                    {
                        x_temp = x_col_form.col(row_id); // [x1 - x0; x2 - x1....]的原始信息
                        // std::cout << x_col_form << std::endl;
                        // std::cin >> hahaha;
                        D_1 << 0, D.col(row_id);                // [D0; D1; D2...]的原始信息
                        D_2 << D.col(row_id), 0;                // [D1; D2; D3; ...; Dn]的原始信息
                        grad_D1 << 0, grad_D_col_i.col(row_id); // D_1的导数信息
                        grad_D2 << grad_D_col_i.col(row_id), 0; // D_2的导数信息
                        gradient = 12 * temp_X_vector.transpose() * (x_temp - D_1 - D_2);
                        gradient = gradient + 12 * x_temp.transpose() * (temp_X_vector - grad_D1 - grad_D2) + 4 * grad_D1.transpose() * D_1 + 4 * D_1.transpose() * grad_D1 + 4 * grad_D2.transpose() * D_1 + 4 * D_2.transpose() * grad_D1 + 4 * grad_D2.transpose() * D_2 + 4 * D_2.transpose() * grad_D2;
                        gradient = gradient + grad_obstacle(row_id);
                        // std::cout << gradient << std::endl;
                        norm += gradient * gradient;
                    }
                    if (row_id == 1) // 对y的导数信息
                    {
                        x_temp = x_col_form.col(row_id);        // [x1 - x0; x2 - x1....]的原始信息
                        D_1 << 0, D.col(row_id);                // [D0; D1; D2...]的原始信息
                        D_2 << D.col(row_id), 0;                // [D1; D2; D3; ...; Dn]的原始信息
                        grad_D1 << 0, grad_D_col_i.col(row_id); // D_1的导数信息
                        grad_D2 << grad_D_col_i.col(row_id), 0; // D_2的导数信息
                        gradient = 12.0 * temp_X_vector.transpose() * (x_temp - D_1 - D_2);
                        gradient = gradient + 12.0 * x_temp.transpose() * (temp_X_vector - grad_D1 - grad_D2) + 4.0 * grad_D1.transpose() * D_1 + 4.0 * D_1.transpose() * grad_D1 + 4.0 * grad_D2.transpose() * D_1 + 4.0 * D_2.transpose() * grad_D1 + 4.0 * grad_D2.transpose() * D_2 + 4.0 * D_2.transpose() * grad_D2;
                        gradient = gradient + grad_obstacle(row_id);
                    }
                    g(temp) = gradient;
                    temp++;
                }
            }
            // std::cout << g << std::endl;
            // std::cout << "size" << g.rows() << std::endl;
            // std::cout << "cost: " << cost << std::endl;
            // // std::cout << "norm: " << g.norm() << std::endl;
            // std::cout << "norm: " << sqrt(norm) << std::endl;
            // // std::cout << current_points << std::endl;
            // std::cout << "point\n" << ((PathSmoother*)ptr)->points << std::endl;
            // std::cin >> hahaha;
            // std::cout << "end" << std::endl;
            return cost;
        }

    public:
        inline bool setup(const Eigen::Vector2d &initialP,
                          const Eigen::Vector2d &terminalP,
                          const int &pieceNum,
                          const Eigen::Matrix3Xd &diskObs,
                          const double penaWeight)
        {
            pieceN = pieceNum;
            diskObstacles = diskObs;
            penaltyWeight = penaWeight;
            headP = initialP;
            tailP = terminalP;

            cubSpline.setConditions(headP, tailP, pieceN);
            std::cout << "pieceN:" << pieceN << std::endl;
            // std::cout << diskObstacles << std::endl;

            points.resize(2, pieceN - 1);
            gradByPoints.resize(2, pieceN - 1);

            return true;
        }

        inline double optimize(CubicCurve &curve,
                               const Eigen::Matrix2Xd &iniInPs,
                               const double &relCostTol)
        {
            // TODO
            points = iniInPs;

            Eigen::VectorXd g;
            Eigen::VectorXd g_last;
            Eigen::Matrix2Xd mat_temp = points;
            Eigen::Map<Eigen::VectorXd> x(mat_temp.data(), mat_temp.size());
            double cost;
            clock_t start, end;
            start = clock();
            // std::ofstream ostrm("/home/wang/deep_blue/numerical_optimization/chapter_2/homework/catkin_ws/src/gcopter/src/record_g.txt", std::ios_base::out);

            // 获得初始点的cost和g
            g.resize(x.rows());
            g_last.resize(x.rows());
            g_last.fill(0.0);
            cost = costFunction(this, x, g);

            Eigen::MatrixXd B;
            B.resize(g.rows(), g.rows());
            B = Eigen::MatrixXd::Identity(g.rows(), g.rows());
            int k = 0;
            ros::Rate rate(1);
            // 开始迭代
            while (true)
            {
                // 判断是否跳出循环
                // std::cout << "k = " << k << std::endl;
                // ostrm << (g - g_last).norm() << std::endl;
                if ((g - g_last).norm() <= relCostTol || k > 50000)
                // if ((g - g_last).norm() <= 1e-3 || k > 50000)
                {
                    break;
                }

                // 获得d
                Eigen::VectorXd d;
                d.resize(g.rows());
                d = -B * g;

                // 获得步长alpha
                double l = 0.0;
                double u = INFINITY;
                double alpha = 1.0;
                double c1 = 1e-4;
                double c2 = 0.9;
                double cost_temp;
                Eigen::VectorXd g_temp;
                g_temp.resize(g.rows());
                while (true)
                {
                    Eigen::VectorXd point_temp;
                    point_temp.resize(x.rows());
                    point_temp = x + alpha * d;
                    cost_temp = costFunction(this, point_temp, g_temp);

                    if ((cost - cost_temp) < (-c1 * alpha * d.transpose() * g))
                    {
                        u = alpha;
                    }
                    else if ((d.transpose() * g_temp) < c2 * d.transpose() * g)
                    {
                        l = alpha;
                    }
                    else
                    {
                        break;
                    }

                    if (u < INFINITY)
                    {
                        alpha = (l + u) / 2;
                    }
                    else
                    {
                        alpha = 2 * l;
                    }
                }

                // 获得x_k+1与g_k+1
                Eigen::VectorXd x_new, g_new;
                x_new.resize(x.rows());
                g_new.resize(x.rows());
                x_new = x + alpha * d;
                // std::cout << "alpha: " << alpha << std::endl;
                cost = costFunction(this, x_new, g_new);

                // BFGS
                Eigen::VectorXd delta_g, delta_x;
                double epsilon = 1e-6;
                delta_g.resize(x_new.rows());
                delta_x.resize(x_new.rows());
                delta_g = g_new - g;
                delta_x = x_new - x;
                if ((delta_g.transpose() * delta_x) > (epsilon * sqrt(g.norm()) * delta_x.transpose() * delta_x)) // 更新B
                {
                    Eigen::MatrixXd mat1, mat2;
                    mat1.resize(x.rows(), x.rows());
                    mat2.resize(x.rows(), x.rows());
                    mat1 = Eigen::MatrixXd::Identity(x.rows(), x.rows()) - (delta_x * delta_g.transpose()) / (delta_g.transpose() * delta_x);
                    mat2 = Eigen::MatrixXd::Identity(x.rows(), x.rows()) - (delta_g * delta_x.transpose()) / (delta_g.transpose() * delta_x);
                    B = mat1 * B * mat2 + (delta_x * delta_x.transpose()) / (delta_g.transpose() * delta_x);
                }
                x = x_new;
                // std::cout << g_new << std::endl;
                g_last = g;
                g = g_new;
                k = k + 1;
                // std::cout << k << std::endl;
                // std::cout << g.norm() << std::endl;
                // std::cout << (g - g_last).norm() << std::endl;
                // rate.sleep();
            }
            // std::cout << x << std::endl;
            // std::cout << "end" << std::endl;

            // 将x写入curve
            std::vector<double> durs;
            std::vector<Eigen::Matrix<double, 2, 4>> cMats;

            // 先将x改为2X形式
            Eigen::Matrix2Xd x_2x;
            x_2x.resize(2, pieceN - 1);
            int temp = 0;
            for (int col_temp = 0; col_temp < (pieceN - 1); col_temp++)
            {
                for (int row_temp = 0; row_temp < 2; row_temp++)
                {
                    x_2x(row_temp, col_temp) = x(temp);
                    temp++;
                }
            }

            // 获得对应的D，并将x_2x加上tail&head的信息
            Eigen::MatrixX2d D_temp;
            Eigen::Matrix2Xd D;
            Eigen::Matrix2Xd x_with_tail;
            D_temp.resize(pieceN - 1, 2);
            D.resize(2, pieceN + 1);
            x_with_tail.resize(2, pieceN + 1);
            cubSpline.setInnerPoints(x_2x);
            D_temp = cubSpline.getCoeffs();
            for (int col_temp = 0; col_temp < D.cols(); col_temp++)
            {
                for (int row_temp = 0; row_temp < 2; row_temp++)
                {
                    if (col_temp == 0)
                    {
                        D(row_temp, col_temp) = 0;
                    }
                    else if (col_temp == D.cols() - 1)
                    {
                        D(row_temp, col_temp) = 0;
                    }
                    else
                    {
                        D(row_temp, col_temp) = D_temp(col_temp - 1, row_temp);
                    }
                }
                if (col_temp == 0)
                {
                    x_with_tail.col(col_temp) = headP;
                }
                else if (col_temp == D.cols() - 1)
                {
                    x_with_tail.col(col_temp) = tailP;
                }
                else
                {
                    x_with_tail.col(col_temp) = x_2x.col(col_temp - 1);
                }
            }
            // 将数据写入cMats
            for (int col_temp = 0; col_temp < pieceN; col_temp++)
            {
                Eigen::Matrix<double, 2, 4> cMats_temp;
                
                cMats_temp.col(0) = 2 * (x_with_tail.col(col_temp) - x_with_tail.col(col_temp + 1)) + D.col(col_temp) + D.col(col_temp + 1);
                
                cMats_temp.col(1) = 3 * (x_with_tail.col(col_temp + 1) - x_with_tail.col(col_temp)) - 2 * D.col(col_temp) - D.col(col_temp + 1);
                
                cMats_temp.col(2) = D.col(col_temp);
                
                cMats_temp.col(3) = x_with_tail.col(col_temp);
                // std::cout << 222222222222 << std::endl;
                cMats.push_back(cMats_temp);
                durs.push_back(1.0);
            }
            // curve(durs, cMats);
            CubicCurve curve_temp(durs, cMats);
            curve = curve_temp;
            double minCost = cost;
            end = clock();
            std::cout << "time = " << double(end - start) / CLOCKS_PER_SEC << std::endl;
            return minCost;
        }
    };

}

#endif
