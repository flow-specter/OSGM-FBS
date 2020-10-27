#pragma once
#include <vector>

struct LSMSummary {
    ///\brief 初始的cost
    double initial_cost;

    ///\brief 最终的cost值
    double final_cost;

    ///\brief 记录每次迭代的值
    std::vector<std::vector<double>> param_history;

    ///\brief 最终的参数
    std::vector<double> x;

    ///\brief 迭代次数
    int iterations;

    ///\brief 是否收敛
    bool is_converge;

    ///\brief 匹配时间
    double runtime;
};