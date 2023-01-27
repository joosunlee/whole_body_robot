#include <gazebo/gazebo.hh>

    namespace gazebo
    {
      class MobilebasePlugin : public WorldPlugin
      {
        public: MobilebasePlugin() : WorldPlugin()
                {
                  printf("Hello World!\n");
                  gazebo::physics::ModelPtr this->model ;
                }

        public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
                {
                    this->model = world->GetModel("mobile_base");
                }
        public : void OnUpdate(const common::UpdateInfo & /*_info*/)
     {
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(math::Vector3(0, 0, 0)); // just in case needed

      // pose of the footprint
      gazebo::math::Pose mapose(this->vect3_pose_model, this->rot_pose_model);
      this->model->SetLinkWorldPose(mapose,"mobile_base");
      

    //pose of every joint 
     this.vecteur_str = ["head","base","joint3","whatever"];
      for (int i = 0 ; i < this->vecteur_str.size(); i++)
      {
         //what I used to control the joints 
        //this->model->GetJoint(vecteur_str[i])->SetAngle(0, vecteur_effort[i]);  
        // This is what you want
        
        this->model->GetLink("link_name")->SetForce(ignition::math::Vector3d(0, 0, 0.5));
        this->model->GetJoint("joint_name")->SetForce(0, torque);
      }
    }

    
  };
  GZ_REGISTER_WORLD_PLUGIN(MobilebasePlugin)
}