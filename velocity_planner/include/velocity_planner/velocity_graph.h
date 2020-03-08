#ifndef VELOCITY_PLANNER_VELOCITY_GRAPH_H_INCLUDED
#define VELOCITY_PLANNER_VELOCITY_GRAPH_H_INCLUDED

#include <hermite_path_planner/hermite_path_generator.h>
#include <hermite_path_msgs/msg/hermite_path_stamped.hpp>
#include <hermite_path_msgs/msg/reference_velocity.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <map>

namespace velocity_planner
{
    struct Node
    {
        hermite_path_msgs::msg::ReferenceVelocity vel;
    };

    struct Edge
    {
        Node before_node;
        Node after_node;
        double linear_accerelation;
    };

    struct Graph
    {

    };

    typedef boost::adjacency_list<boost::listS,boost::vecS,
        boost::bidirectionalS,Node,Edge,Graph> VelocityGraphData;

    class VelocityGraph
    {
    public:
        VelocityGraph(hermite_path_msgs::msg::HermitePathStamped data,
            double velocity_resoluation,
            double maximum_accerelation,
            double minimum_accerelation);
    private:
        std::vector<Node> makeNodes(hermite_path_msgs::msg::ReferenceVelocity vel);
        boost::optional<std::vector<Edge> > makeEdges(std::map<double,std::vector<Node> > nodes);
        void buildVelocityGraph(std::map<double,std::vector<Node> > nodes,std::vector<Edge> edges);
        VelocityGraphData data_;
        double velocity_resoluation_;
        double maximum_accerelation_;
        double minimum_accerelation_;
        double path_length_;
        hermite_path_planner::HermitePathGenerator generator_;
        bool planning_succeed_;
    };
}

#endif  //VELOCITY_PLANNER_VELOCITY_GRAPH_H_INCLUDED