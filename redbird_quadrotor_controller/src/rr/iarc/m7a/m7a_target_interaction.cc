//Reference: https://dev.px4.io/en/ros/mavros_offboard.html

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <chrono>	/*for time*/
#include <thread>	/*for time*/
#include <math.h>       /* sqrt */


/**
* The mavros_msgs package contains all of the custom messages required to operate services and topics provided by the MAVROS package. 
* All services and topics as well as their corresponding message types are documented in the mavros wiki.
*/

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/**
* Create a simple callback functino which will save the current state of the autopilot. 
* This will allows checking connection, arming and Offboard flags.
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

	//need a subscriber for whatever infrastrucutre knows ground bot info
	// TO DO : Need to update these subscribers to our code setup

    /**
	* Instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change. 
	* Note that for your own system, the "mavros" prefix might be different as it will depend on the name given to the node in it's launch file.
	*/

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    /**
	* The px4 flight stack has a timeout of 500ms between two Offboard commands. 
	* If this timeout is exceeded, the commander will fall back to the last mode the vehicle was in before entering Offboard mode. 
	* This is why the publishing rate must be faster than 2 Hz to also account for possible latencies. 
	* This is also the same reason why it is recommended to enter Offboard mode from Position mode, 
	* this way if the vehicle drops out of Offboard mode it will stop in its tracks and hover.
	*/

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){ //ros::ok returning true means that the node hasn't been shut down && wait until it's current state IS connected
        ros::spinOnce(); //something to do with threading...
        rate.sleep(); //Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.
    }

    //set points here ??
    //insert call functions here?
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

	//TO DO: Figure out how to loop while hovering until maneuver command is received. 



    enum maneuver {takeOffLand, topTouch, obstruct, goTo, landNow};

    maneuver m; //Set this equal to int value received from command message (or maybe that value can be an enum?)

    if (m == 0){
    	//call takeOffLand()
    	int timeDelay = 2; //2 seconds
    	int altitude = 2; //2 meters
    	takeOffLand();
    }
    else if(m == 1){
    	//call topTouch
    	//ground robots values coming from message
    	float gx = 0;
		float gy = 0;
    	float gb = 300; //ground robot bearing in degrees needs to be imported

    	topTouch();
    }
    else if(m == 2){
    	//call obstruct
    	obstruct(); //Needs implemmenting
    }
    else if(m == 3){
    	//call goTo
    	int x; //target points - from message
    	int y;
    	int z;
    	goTo(x, y, z);

    }
    else if(m == 4){
    	//call landNow
    	landNow();
    }
    else{
    	//command not recognized
    }

    return 0;
}



/**
* The rest of the code is pretty self explanatory. We attempt to switch to Offboard mode, after which we arm the quad to allow it to fly. 
* We space out the service calls by 5 seconds so to not flood the autopilot with the requests. 
* In the same loop, we continue sending the requested pose at the appropriate rate.
*/



public static void takeOffLand(int timeDelay, altitude){

	//wait 2 seconds, takeoff to 2 meters, land 
	/*Do we need a specific function for landing vs. going to a setpoint?? I'm worried it'll try to land too quickly */

	delay(timeDelay);

	target(0, 0, altitude); //are these units in meters? - i think so

	move();

	target(0, 0, 0);

	move();

	delay(timeDelay);

	//should it disarm at this point?

}

public static void topTouch(){



//add in some kind of loop to interate until top-touch is executed

//constants:
	float gz = 0.5; //height of ground robot
	float gv = 0; //ground robot velocity should be a constant
	float dv = 0; //drone max velocity

//get current drone positions here
	float dx = 0;
	float dy = 0;

//theta corrections to gb here - needs more testing
	if(gb > 180){
		gb = 360 - gb;
	}

	pose.pose.position.x = gx + ((gv)*(std::sqrt((dx-gx)*(dx-gx)+(dy-gy)*(dy-gy)))/dv)*cos(gb);
	pose.pose.position.y = gy + ((gv)*(std::sqrt((dx-gx)*(dx-gx)+(dy-gy)*(dy-gy)))/dv)*sin(gb);
	pose.pose.position.z = gz;

}


public static void obstruct(){

	//needs implementing

}



public static void goTo(int x, int y, int z){

	//go to setpoint
	if(z == null){
		z = 0;
	}

	target(x, y, z);

	move();

}

public static void landNow(){

	//setpoint for directly below drone

	int x = 0; //drone's current position
	int y = 0; //drone's current position
	int z = 0; //target altitude

	target(x, y, z);
	move();

}


public static void delay(float t){


    //Delays for time t seconds
    //Use C++ Standard library chrono-duration class

    using namespace std::this_thread; // sleep_for, sleep_until
    using namespace std::chrono; // nanoseconds, system_clock, seconds

    //sleep_for(nanoseconds(10));
    sleep_until(system_clock::now() + seconds(t)); //wait t seconds (not sure if this takes a float)  


}

public static void target(int x, int y, int z){

    //sets message content to x,y,z input needed for target

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    //message pose now contains target coordinates

}

public static void move(){

	//send the current pose position message to the controller to be executed immediately

	local_pos_pub.publish(pose);

}

