#include <velocity_planner/planner_concatenator_component.h>

namespace velocity_planner
{
    PlannerConcatenatorComponent::PlannerConcatenatorComponent(const rclcpp::NodeOptions & options)
    : Node("planner_concatenator", options)
    {
        hermite_path_pub_ = this->create_publisher<hermite_path_msgs::msg::HermitePathStamped>
            ("~/hermite_path", 1);
        declare_parameter("num_input",2);
        get_parameter("num_input",num_input_);
        assert(num_input_>=2 && num_input_<=8);
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(10));
        for(int i=0; i<num_input_; i++)
        {
            declare_parameter("input_topic"+std::to_string(i),
                get_name() + std::string("/input") + std::to_string(i));
            get_parameter("input_topic"+std::to_string(i),input_topics_[i]);
            std::shared_ptr<HermitePathSubscriber> sub_ptr = 
                std::make_shared<HermitePathSubscriber>(this,input_topics_[i]);
            sub_ptrs_[i] = sub_ptr;
        }
        switch(num_input_)
        {
            case 2:
                sync_->connectInput(*sub_ptrs_[0], *sub_ptrs_[1], 
                    nf_, nf_, nf_, nf_, nf_, nf_);
                break;
            case 3:
                sync_->connectInput(*sub_ptrs_[0], *sub_ptrs_[1], 
                    *sub_ptrs_[2], nf_, nf_, nf_, nf_, nf_);
                break;
            case 4:
                sync_->connectInput(*sub_ptrs_[0], *sub_ptrs_[1], 
                    *sub_ptrs_[2], *sub_ptrs_[3], nf_, nf_, nf_, nf_);
                break;
            case 5:
                sync_->connectInput(*sub_ptrs_[0], *sub_ptrs_[1], 
                    *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], nf_, nf_, nf_);
                break;
            case 6:
                sync_->connectInput(*sub_ptrs_[0], *sub_ptrs_[1],
                    *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], *sub_ptrs_[5], nf_, nf_);
                break;
            case 7:
                sync_->connectInput(*sub_ptrs_[0], *sub_ptrs_[1], 
                    *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], *sub_ptrs_[5], *sub_ptrs_[6], nf_);
                break;
            case 8:
                sync_->connectInput(*sub_ptrs_[0], *sub_ptrs_[1], 
                    *sub_ptrs_[2], *sub_ptrs_[3], *sub_ptrs_[4], *sub_ptrs_[5], *sub_ptrs_[6], *sub_ptrs_[7]);
                break;
        }
    }

    void PlannerConcatenatorComponent::callback(const HermitePathStamped::SharedPtr &in0, const HermitePathStamped::SharedPtr &in1,
        const HermitePathStamped::SharedPtr &in2, const HermitePathStamped::SharedPtr &in3,
        const HermitePathStamped::SharedPtr &in4, const HermitePathStamped::SharedPtr &in5,
        const HermitePathStamped::SharedPtr &in6, const HermitePathStamped::SharedPtr &in7)
        {
            assert(num_input_>=2 && num_input_<=8);
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> vels;
            /*
            switch(num_input_)
            {
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
            }
            */
        }
}