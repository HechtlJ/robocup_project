#include<move_object/move_object.h>

using namespace move_object;

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "move_object_grasp_only");
  // ros::NodeHandle nh;
  // MoveObject node(nh); 

  // //--------------- FIND THE BEST GRASP----------------
  // ros::Duration(1).sleep();
  // node.turnHeadDown();
  // ros::Duration(1.5).sleep();
  
  
  while(ros::ok())
  {
    bool is_object_in_hand = 0;
    while(!is_object_in_hand)  // do the grasping until grasp is successfully
    {  
      ROS_INFO("Are you ready to pick up objects?");
      system("read -p 'Press Enter to continue...' var");        
      node.callServiceToFindGrasp();
      // Waiting for the selected best grasp to arrive
      ROS_INFO("Waiting for the coming grasp pose......");
      node.pickUpSetup(); 

      while (!node.grasp_processed_)
      {
        node.queue_1.callAvailable(ros::WallDuration(1));
        ros::Duration(0.2).sleep(); 
         
        if(node.empty_grasp_)   // detect the empty table
        {
            ROS_INFO("Please put some objects on the table"); // remind the user to put something on the table
            system("read -p 'Press Enter to continue...' var");
            node.callServiceToFindGrasp();
            node.empty_grasp_ = false;
            ROS_INFO("Waiting for the coming grasp pose......");
        }//----
      }
        node.grasp_processed_ = 0;

        //--------------- PICK UP THE OBJECT----------------
    
        ROS_INFO("now start to pick up!");
        node.pickUpForGraspOnlyNode();

        // Judge whether the object is in hand 
        is_object_in_hand = node.getInhand(); 
        if (is_object_in_hand==0)  
        {
          ROS_ERROR("Failed to pick up the object, let's try it again~~!");
        }
        else
          ROS_INFO("Pick up successfully~! ");
    } //------------if not successfull, redo the pick-up progress------

    //--------------- Put the object into the container----------
    node.putDownForGraspOnlyNode();
  }

  return 0;
}


