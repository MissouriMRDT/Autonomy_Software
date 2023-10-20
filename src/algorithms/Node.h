#ifndef NODE_H
#define NODE_H

#include "../../src/AutonomyLogging.h"
#include <iostream>

class Node
{
    private:
        Node* m_pParent                   = nullptr;
        std::array<double, 2> m_dposition = {0, 0};

        double m_dG                       = 0;    // distance from start
        double m_dH                       = 0;    // distance from end
        double m_dF                       = 0;    // cost = g + h

    public:
        Node()
        {
            m_dF = m_dG + m_dH;
            return;
        }

        Node(Node* pParent, std::array<double, 2> dPosition = {0, 0}, double dG = 0, double dH = 0)    // TODO: Initialization instead of assignmnet
        {
            m_pParent      = pParent;
            m_dposition[0] = dPosition[0];
            m_dposition[1] = dPosition[1];
            m_dG           = dG;
            m_dH           = dH;
            m_dF           = dG + dH;
        }

        bool operator==(const Node& node) const
        {
            if (m_dposition[0] == node.m_dposition[0] && m_dposition[1] == node.m_dposition[1])
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        bool operator<(const Node& node) const    // for min heap
        {
            return m_dF > node.m_dF;
        }

        Node* GetParent() const { return m_pParent; }

        double GetPositionFirst() const { return m_dposition[0]; }

        double GetPositionSecond() const { return m_dposition[1]; }

        void SetGValue(double G)
        {
            m_dG = G;
            m_dF = m_dG + m_dH;
        }

        double GetGValue() const { return m_dG; }

        void SetHValue(double H)
        {
            m_dH = H;
            m_dF = m_dG + m_dH;
        }

        double GetHValue() const { return m_dH; }
};

#endif
