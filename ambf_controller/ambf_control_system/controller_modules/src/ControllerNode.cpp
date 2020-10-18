
#include "controller_modules/ControllerNode.h"



ControllerNode::ControllerNode(rigidBodyPtr _handle,
                               ros::NodeHandle* _nh,
                               const Eigen::Ref<const Eigen::MatrixXd>& kp,
                               const Eigen::Ref<const Eigen::MatrixXd>& kd): handle(_handle), n(*_nh), controller(kp, kd)
{

    running = false;
    start_controller = n.subscribe("start_controller", 1000, &ControllerNode::startControllerCallback, this);
    stop_controller = n.subscribe("stop_controller", 1000, &ControllerNode::stopControllerCallback, this);
    client_ID = n.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    have_path = false;

}


void ControllerNode::startControllerCallback(const std_msgs::Empty )
{
   if(running)
   {
       ROS_INFO("Controller is already running");
   }
   else
   {
       startController();
   }
}

void ControllerNode::stopControllerCallback(const std_msgs::Empty )
{
    if(running)
    {
        ROS_INFO("Stoping the controller");
        running=false;

    }
    else
    {
        ROS_INFO("Controller is not running");
    }
}

void ControllerNode::setGain(const Eigen::Ref<const Eigen::MatrixXd>& Kp, const Eigen::Ref<const Eigen::MatrixXd>& Kd)
{
//    controller.setKd(Kd);
//    controller.setKp(Kp);
}

void ControllerNode::updataPath(const trajectory_generator::trajectory& new_path)
{
        path = new_path;
        path_index = 0;
        have_path = true;
        path_length = new_path.pathLength;

}

bool ControllerNode::startController()
{

    if (!running)
    {
        ROS_INFO("Starting the Controller");
        //boost::thread run(boost::bind(&ControllerNode::control, this));
        run = boost::thread(boost::bind(&ControllerNode::control, this));
        running = true;
    }

    return true;

}

///
/// \brief RBDLServer::VectToEigen
/// \param msg
///
Eigen::VectorXd ControllerNode::VectToEigen(const std::vector<double> &msg)
{
    std::vector<double> vec(msg.begin(), msg.end());
    Eigen::VectorXd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
    return Q;
}


Eigen::VectorXd ControllerNode::VectToEigen(const std::vector<float> &msg)
{
    std::vector<double> vec(msg.begin(), msg.end());
    Eigen::VectorXd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
    return Q;
}


void ControllerNode::control()
{

    std::vector<double> tau;
    Eigen::VectorXd pos; //=  VectToEigen(pos_vec);
    Eigen::VectorXd vel; // =  VectToEigen(vel_vec);
    Eigen::VectorXd e, ed, aq;
    rbdl_server::RBDLInverseDynamics Invdny_msg;
    std::vector<double> curr_pos, curr_vel;
    ros::Rate loop_rate(1000);

    //set then inital desired state to the current state
    std::vector<float> pos_vec = handle->get_all_joint_pos();
    std::vector<float> vel_vec = handle->get_all_joint_vel();

    std::vector<double> pos_vec_temp(pos_vec.begin(), pos_vec.end());
    std::vector<double> vel_vec_temp(vel_vec.begin(), vel_vec.end());

    desired_pos =  VectToEigen(pos_vec_temp);
    desired_vel =  VectToEigen(vel_vec_temp);

    //run the controller in a loop
    while(ros::ok() && running)
    {

        std::vector<float> pos_vec = handle->get_all_joint_pos();
        std::vector<float> vel_vec = handle->get_all_joint_vel();

        //check if there is a new path
        if(have_path)
        {

            if(path_index == path_length)
            {
                have_path = false;
            }
            else
            {
                desired_pos = VectToEigen( path.traj[path_index].position );
                desired_vel = VectToEigen( path.traj[path_index].velocity );
                path_index++;
            }

        }

        // calculate the control input
        e = desired_pos - VectToEigen(curr_pos);
        ed = desired_vel - VectToEigen(curr_vel);
        controller.calculate(e, ed, aq);
        std::vector<double> aq_vect(&aq[0], aq.data()+aq.cols()*aq.rows());

        //build the message
        Invdny_msg.request.q = curr_pos;
        Invdny_msg.request.qd = curr_vel;
        Invdny_msg.request.qdd = aq_vect;

        //call the service
        if (client_ID.call(Invdny_msg))
        {
            //get torque
            //sent torqe to AMBF
            tau = Invdny_msg.response.tau;
            std::vector<float> float_tau(tau.begin(), tau.end());
            //handle->set_joint_efforts(float_tau);

        }
        else
        {
            ROS_ERROR("Failed to call service ID");

        }

        loop_rate.sleep();
        ros::spinOnce();
    }


}


