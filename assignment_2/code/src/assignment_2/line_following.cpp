#include "line_following.h"
using namespace std;
namespace assignment_2
{

 // Conversões

  float LineFollowing::radToDeg(float angle) 
  {
     return angle * (180 / 3.14159265358979323846); 
  }

  float LineFollowing::degToRad(float angle) 
  { 
    return angle * (3.14159265358979323846 / 180); 
  }

  // Construtor 
  
  LineFollowing::LineFollowing(int argc,char **argv)
  {
    if(argc != 3)
    {
      ROS_ERROR(
        "Usage : assignment_2 robot <robot_id> <laser_id>");
      exit(0);
    }
	  
    side = None;
    firstTime = true;

    laser_topic_ = std::string("/") + std::string(argv[1]) + std::string("/") + std::string(argv[2]);
    speeds_topic_ = std::string("/") + std::string(argv[1]) + std::string("/cmd_vel");
      
    subscriber_ = n_.subscribe(
      laser_topic_.c_str(), 
      1, 
      &LineFollowing::callback,
      this);
      
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>(speeds_topic_.c_str(), 1);
  }
  
  LineFollowing::~LineFollowing(void)
  {
    
  }
  
  void LineFollowing::callback(const sensor_msgs::LaserScan& msg)
  {
    scan_ = msg;
    geometry_msgs::Twist cmd;
    
    // Começa por ter distancia maxima a tudo
	
    float distance_right = FLT_MAX; 
    float distance_left = FLT_MAX;
  
    // Parametros a aplicar ao robot
	
    float distance = 0.0;
    float angle = 0.0;
    float alpha = 0.0;
	
    // Variavel side é = None se ainda nao tiver tocado numa parede,
    // = Left se tiver tocado com o lado esquerdo
    // = Right se tiver tocado com o lado direito

    for(unsigned int i = 0; i < scan_.ranges.size(); i++)
    {
        float sensor_dist = scan_.ranges[i];
        float sensor_angle = -100.0 + radToDeg(scan_.angle_increment) * i;
    
		if(sensor_angle >= -100 && sensor_angle <= 0) 
		{
			if(sensor_dist < distance_right && side != Left) // ignora caso esteja a prioritizar o lado esquerdo
			{
				distance_right = sensor_dist;
				angle = sensor_angle;
			}
		}
		else if(sensor_angle > 0 && sensor_angle <= 100) 
		{
			if(sensor_dist < distance_left && side != Right) // ignora caso esteja a prioritizar o lado direito
			{
				distance_left = sensor_dist;
				angle = sensor_angle;
				
			}	
		} 
	}

	distance = min(distance_left, distance_right);

	// Se for a primeira vez que tocou numa parede, determina qual o lado que tocou e passa a prioritizar esse lado
	
	if(distance != FLT_MAX && firstTime && side == None)
	{
		firstTime = false;
		if(distance == distance_left)
		{
			side = Left;
		}
		else {
			side = Right;
		}
	}
	
	alpha = abs(angle);
	

	float offset = 1;
	if(side == None || side == Left)
	{
		offset = -1;
	}
	
	// DEBUG
	
	cout << "RIGHT ? " << side << endl;
	cout << "DISTANCE: " << distance << endl;
	cout << "ALPHA: " << alpha << endl;
	cout << "------------------------------------" << endl;

	
    if(distance <= scan_.range_max) // Se estiver a tocar numa parede
	{
		cmd.linear.x = 0.4;
		cmd.angular.z = (offset * 15 * (cos(degToRad(alpha)) - (distance - 1.2))) * cmd.linear.x;
   	}
   	else
	{
		cmd.linear.x = 0.4;
		cmd.angular.z = 0.1;
  	}

    	 cmd_vel_pub_.publish(cmd);
    }
}
