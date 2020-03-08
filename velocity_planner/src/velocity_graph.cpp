#include <velocity_planner/velocity_graph.h>

namespace velocity_planner
{
    VelocityGraph::VelocityGraph(hermite_path_msgs::msg::HermitePathStamped data,
        double velocity_resoluation,
        double maximum_accerelation,
        double minimum_accerelation) : generator_(0.0)
    {
        minimum_accerelation_ = minimum_accerelation;
        maximum_accerelation_ = maximum_accerelation;
        velocity_resoluation_ = velocity_resoluation;
        path_length_ = generator_.getLength(data.path,200);
        std::map<double,std::vector<Node> > nodes;
        for(auto itr=data.reference_velocity.begin(); itr!=data.reference_velocity.end(); itr++)
        {
            nodes[itr->t] = makeNodes(*itr);
        }
    }

    boost::optional<std::vector<Edge> > VelocityGraph::makeEdges(std::map<double,std::vector<Node> > nodes)
    {
        std::vector<Edge> edges;
        std::vector<double> t_values;
        for(auto it = nodes.begin(); it != nodes.end(); ++it)
        {
            t_values.push_back(it->first);
        }
        std::sort(t_values.begin(), t_values.end(), [](const auto &a, const auto &b){return a < b;});
        for(int i=0 ; i<((int)t_values.size()-1); i++)
        {
            bool connection_finded = false;
            std::vector<Node> before_nodes = nodes[i];
            std::vector<Node> after_nodes = nodes[i+1];
            double l = (t_values[i+1]-t_values[i])*path_length_;
            for(auto before_itr = before_nodes.begin(); before_itr != before_nodes.end(); before_itr++)
            {
                for(auto after_itr = after_nodes.begin(); after_itr != after_nodes.end(); after_itr++)
                {
                    double v0 = before_itr->vel.linear_velocity;
                    double v1 = after_itr->vel.linear_velocity;
                    double a = (v1*v1-v0*v0)/(2*l);
                    if(a > minimum_accerelation_ && a < maximum_accerelation_)
                    {
                        connection_finded = true;
                        Edge edge;
                        edge.before_node = *before_itr;
                        edge.after_node = *after_itr;
                        edge.linear_accerelation = a;
                        edges.push_back(edge);
                    }
                }
            }
            if(!connection_finded)
            {
                return boost::none;
            }
        }
        return edges;
    }

    std::vector<Node> VelocityGraph::makeNodes(hermite_path_msgs::msg::ReferenceVelocity vel)
    {
        std::vector<Node> ret;
        double v = 0.0;
        int count = 0;
        while(v>vel.linear_velocity)
        {
            v = velocity_resoluation_*count;
            Node n;
            n.vel.t = vel.t;
            n.vel.linear_velocity = vel.linear_velocity;
            ret.push_back(n);
            count++;
        }
        return ret;
    }
}