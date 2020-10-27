#pragma once
#include <vector>

struct LSMSummary {
    ///\brief ��ʼ��cost
    double initial_cost;

    ///\brief ���յ�costֵ
    double final_cost;

    ///\brief ��¼ÿ�ε�����ֵ
    std::vector<std::vector<double>> param_history;

    ///\brief ���յĲ���
    std::vector<double> x;

    ///\brief ��������
    int iterations;

    ///\brief �Ƿ�����
    bool is_converge;

    ///\brief ƥ��ʱ��
    double runtime;
};