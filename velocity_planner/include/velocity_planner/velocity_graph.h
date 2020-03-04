#ifndef VELOCITY_PLANNER_VELOCITY_GRAPH_H_INCLUDED
#define VELOCITY_PLANNER_VELOCITY_GRAPH_H_INCLUDED

namespace velocity_planner
{
    struct Node
    {
        double linear_velocity;
    };

    struct Edge
    {
        double linear_accerelation;
    };
}

#endif  //VELOCITY_PLANNER_VELOCITY_GRAPH_H_INCLUDED