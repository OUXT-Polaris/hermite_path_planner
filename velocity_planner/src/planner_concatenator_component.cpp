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
        //sync_.reset(new message_filters::Synchronizer<SyncPolicy>(10));
        for(int i=0; i<num_input_; i++)
        {
            declare_parameter("input_topic"+std::to_string(i),
                get_name() + std::string("/input") + std::to_string(i));
            get_parameter("input_topic"+std::to_string(i),input_topics_[i]);
            std::shared_ptr<HermitePathSubscriber> sub_ptr(new HermitePathSubscriber(this,input_topics_[i]));
            sub_ptrs_[i] = sub_ptr;
        }
        /*
        sync = std::make_shared<message_filters::TimeSynchronizer<HermitePathStamped, HermitePathStamped> >
            (sub_ptrs_[0].get(),sub_ptrs_[1].get(),10);
        */
        std::shared_ptr<HermitePathSubscriber> sub0 = sub_ptrs_[0];
        std::shared_ptr<HermitePathSubscriber> sub1 = sub_ptrs_[1];
        sync = std::make_shared<message_filters::TimeSynchronizer<HermitePathStamped, HermitePathStamped> >(*sub_ptrs_[0],*sub_ptrs_[1],10);
        sync->registerCallback(std::bind(&PlannerConcatenatorComponent::test_callback, this, 
            std::placeholders::_1, std::placeholders::_2));
        /*
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
        sync_->registerCallback(std::bind(&PlannerConcatenatorComponent::callback, this, 
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8));
        */
    }

    void PlannerConcatenatorComponent::test_callback(const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1)
    {
        RCLCPP_ERROR(get_logger(),"test");
    }

    void PlannerConcatenatorComponent::callback(const HermitePathStamped::ConstSharedPtr in0, const HermitePathStamped::ConstSharedPtr in1,
        const HermitePathStamped::ConstSharedPtr in2, const HermitePathStamped::ConstSharedPtr in3,
        const HermitePathStamped::ConstSharedPtr in4, const HermitePathStamped::ConstSharedPtr in5,
        const HermitePathStamped::ConstSharedPtr in6, const HermitePathStamped::ConstSharedPtr in7)
        {
            RCLCPP_ERROR(get_logger(),"test");
            assert(num_input_>=2 && num_input_<=8);
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> target_vels;
            std::vector<hermite_path_msgs::msg::ReferenceVelocity> reference_vels;
            std::array<HermitePathStamped::SharedPtr,8> data;
            if(num_input_>=2)
            {
                std::vector<hermite_path_msgs::msg::ReferenceVelocity> v0 = in0->target_velocity;
                std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v0));
                std::vector<hermite_path_msgs::msg::ReferenceVelocity> r0 = in0->reference_velocity;
                std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r0));
                std::vector<hermite_path_msgs::msg::ReferenceVelocity> v1 = in1->target_velocity;
                std::copy(target_vels.begin(),target_vels.end(),std::back_inserter(v1));
                std::vector<hermite_path_msgs::msg::ReferenceVelocity> r1 = in1->reference_velocity;
                std::copy(reference_vels.begin(),reference_vels.end(),std::back_inserter(r1));
            }
            hermite_path_msgs::msg::HermitePathStamped path;
            path.header = in0->header;
            path.path = in0->path;
            path.target_velocity = target_vels;
            path.reference_velocity = reference_vels;
            hermite_path_pub_->publish(path);
        }
}