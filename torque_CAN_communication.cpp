#include "mbed.h"
#define apps_id 104
#define current_id 1313
#define voltage_id 1314
#define rpm_id 120
#define yaw_id 112

Serial pc(USBTX, USBRX); // tx, rx

///CAN related variables 
CAN sensor_can(p9, p10);
CAN motor_controller_can(p30, p29);

CANMessage can_msg_receive;
CANMessage msg_r_BTB;
CANMessage msg_r_enable;

int enable_can_msg_receive = 0;
int enable_can_msg_send = 0;
int attach_flag = 0;

int speed_request_flag = 1;
double speed_rpm_can_receive = 0;

char t_request_BTB[8] = {0x3D, 0xE2, 0, 0, 0, 0, 0, 0};
char r_BTB[8] = {0xE2, 0x01, 0, 0, 0, 0, 0, 0};
char t_disable[8] = {0x51, 0x04, 0, 0, 0, 0, 0, 0};
char t_request_enable[8] = {0x3D, 0xE8, 0, 0, 0, 0, 0, 0};
char r_enable[8] = {0xE8, 0x01, 0, 0, 0, 0, 0, 0};
char t_no_disable[8] = {0x51, 0x00, 0, 0, 0, 0, 0, 0};

char speed_send[8]= {0x31,0,0,0,0,0,0,0};
const char speed_request[8]= {0x3D,0x30,0x1E,0,0,0,0,0}; //30 ms cyclic

Ticker mc_ticker;

///////////APPS sensor inputs
float apps1_input;
float apps2_input;

//// APPS sensor values range (max and min)
float const apps1_max = 800, apps2_max = 900;
float const apps1_min = 100, apps2_min = 200;
float const apps1_global_max = 850, apps2_global_max = 950;
float const apps1_global_min = 50, apps2_global_min = 150;

float const torque_max_for_motor = 80;

float apps1_normalise;
float apps2_normalise;
float apps_input;
float apps_normalised_difference;
Timer apps_plausibility_timer;

float torque = 0; //this is the torque value that I get from the apps

////////////Steering Sensor input
float steering_sensor_input;

//steering sensor values range (max and min) 
float const steering_right_max = 800;
float const steering_left_max = 200;
float const steering_right_global_max = 900;
float const steering_left_global_max = 100;
float const steering_right_min = 520;
float const steering_left_min = 480; 
//when steer sensor input is between right min and left min => car will not steer
int flag = 1;
int steer_flag=0;
//float const zero_steering = 500;
//}}}}}

float const turning_torque_multiplier_constant = 0.5; // range from 0 to 1

int torque_left = 0;
int torque_right = 0;

//pedal pressing will send same torque value...

//braking
float brake_sensor_input;
float const hard_brake_value = 500;
int brake_flag = 0;
Timer t;

///////////////////

float const max_battery_power_allowed = 80000;
float battery_voltage;
float battery_current;
float w_left, w_right;
float battery_to_motors_efficiency;
float battery_power;
float motor_powers_sum;
//angular velocity of both the wheels
//float power_left, power_right;`

/////////////////////////////////////////////define functions//////////////////////////////////////////////////////////

////====================================================can_initial_setup=================================
void can_initial_setup(){
	//to ensure that we can now send msgs to mc via can
	if(motor_controller_can.read(msg_r_BTB)) {
        if(msg_r_BTB.id==0x181 && msg_r_BTB.data[0]==0xE2 && msg_r_BTB.data[1]==0x01) {
            motor_controller_can.write(CANMessage(0x201, &t_disable[0], 3));
            motor_controller_can.write(CANMessage(0x201, &t_request_enable[0], 3));

            if(motor_controller_can.read(msg_r_enable)) {
                if(msg_r_enable.id==0x181 && msg_r_enable.data[0]==0xE8 && msg_r_enable.data[1]==0x01) {
                    motor_controller_can.write(CANMessage(0x201, &t_no_disable[0], 3));
                    enable_can_msg_send = 1;
                }
            }
        }
    }
	
}

////====================================================get_inputs_from_sensor_can=================================
void get_inputs_from_sensor_can(){
	if(sensor_can.read(can_msg_receive) && can_msg_receive.id == apps_id) {
        apps1_input = (can_msg_receive.data[0] << 2) | (can_msg_receive.data[4] & 3);
        apps2_input = (can_msg_receive.data[1] << 2) | ((can_msg_receive.data[4] >>2) & 3);
        brake_sensor_input = (can_msg_receive.data[2] << 2) | ((can_msg_receive.data[4] >>4) & 3);
        steering_sensor_input = (can_msg_receive.data[3] << 2) | ((can_msg_receive.data[4] >>6) & 3);
        //time_read = (can_msg_receive.data[7] << 16) | (can_msg_receive.data[6] << 8) | can_msg_receive.data[5];
    }
}

////====================================================get_inputs_from_mc_can=================================
void get_inputs_from_mc_can(){
	if (can_msg_receive.id == 181) {
		//reads right mc
        if(can_msg_receive.data[0] == 0xE8 && can_msg_receive.data[1] == 1 ) {
			//hardware enabled
            enable_can_msg_receive=1;
        }
        if(can_msg_receive.data[0] == 0x30) {
			//reading actual value of speed
			//convert received torque to rpm
			speed_rpm_can_receive = ((can_msg_receive.data[3]<<16)+(can_msg_receive.data[2]<<8)+can_msg_receive.data[1] )*7000/32676;	
		}
    }
}

////====================================================send_speed_to_mc_can=================================
void send_speed_to_mc_can(){
	//send speed to mc
	if(speed_request_flag == 1)
        if(motor_controller_can.write(CANMessage(201,&speed_request[0],8))){
            speed_request_flag = 0;
        }

    speed_send[2]=torque_right>>8;
    speed_send[1]=torque_right & 255;
    motor_controller_can.write(CANMessage(201,&speed_send[0],8));
	
	
}

////====================================================print_all_values=================================
void print_all_values()
{
    //print torque of both the wheels and other required values
    pc.printf("flag: ");
    pc.printf("%d \n",flag);
    pc.printf("steer flag: ");
    pc.printf("%d \n",steer_flag);
    pc.printf("brake flag: ");
    pc.printf("%d \n",brake_flag);
    pc.printf("t.read() : ");
    pc.printf("%f \n",t.read());
    pc.printf("apps final value: ");
    pc.printf("%f \n",apps_input);
    pc.printf("left torque: ");
    pc.printf("%d \n",torque_left);
    pc.printf("right torque: ");
    pc.printf("%d \n",torque_right);
    pc.printf("\n");
}

////====================================================set_torque_in_range=================================
void set_torque_in_range()
{
    //if torque value is out of range (0 to torque_max_for_motor) then setting the values to the extremum (0 or torque_max_for_motor)
    if(torque_left > torque_max_for_motor) torque_left = torque_max_for_motor;
    if(torque_right > torque_max_for_motor) torque_right = torque_max_for_motor;
    if(torque_left < 0) torque_left = 0;
    if(torque_right < 0) torque_right = 0;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main()
{
	pc.baud(9600);
	motor_controller_can.frequency(500000);
	sensor_can.frequency(1000000);
	
	sensor_can.attach(&get_inputs_from_sensor_can, CAN::RxIrq);
    motor_controller_can.attach(&get_inputs_from_mc_can, CAN::RxIrq);
	
    while(1) {
		
		//
		if(enable_can_msg_send==0) {
                can_initial_setup();
            }

		if(enable_can_msg_send == 1 && attach_flag == 0) {
			mc_ticker.attach(&send_speed_to_mc_can,0.010);
			attach_flag=1;
		}
		

        //normalise the value of apps1 and apps2
        apps1_normalise = (apps1_input-apps1_min)/(apps1_max-apps1_min);
        apps2_normalise = (apps2_input-apps2_min)/(apps2_max-apps2_min);
        apps_normalised_difference = apps1_normalise-apps2_normalise;
        if(apps_normalised_difference<0) {
            apps_normalised_difference = -1*apps_normalised_difference;
        }

        //check for range (for apps and steer sensor) 
        if(apps1_input>apps1_global_max || apps2_input>apps2_global_max || apps1_input<apps1_global_min || apps2_input<apps2_global_min
                || steering_sensor_input < steering_left_global_max || steering_sensor_input > steering_right_global_max) {
            torque_left = 0;
            torque_right = 0;
            flag = 2;
        } else {
            flag = 3;
            if(apps1_input > apps1_max) {
                apps1_input = apps1_max;
            }
            if(apps2_input > apps2_max) {
                apps2_input = apps2_max;
            }
            if(apps1_input < apps1_min) {
                apps1_input = apps1_min;
            }
            if(apps2_input < apps2_min) {
                apps2_input = apps2_min;
            }



            apps_input = ( apps1_normalise + apps2_normalise ) / 2;
            //APPS inputs taken and converted to range 0 to 1... then their avg value is taken

            torque = apps_input*torque_max_for_motor;
            //torque value normalised in the range for our motors

            if(steering_sensor_input > steering_right_min) {
                //turning right
                steer_flag = 1;
                if (steering_sensor_input > steering_right_max) {
                    steering_sensor_input = steering_right_max;
                }
                steering_sensor_input = (steering_sensor_input - steering_right_min)/(steering_right_max - steering_right_min);
                torque_left = torque*(1 + steering_sensor_input*turning_torque_multiplier_constant);
                torque_right = torque*(1 - steering_sensor_input*turning_torque_multiplier_constant);

            }

            else if(steering_sensor_input < steering_left_min) {
                //turning left
                steer_flag = -1;
                if (steering_sensor_input < steering_left_max) {
                    steering_sensor_input = steering_left_max;
                }
                steering_sensor_input = (steering_sensor_input - steering_left_min)/(steering_left_max - steering_left_min);
                torque_left = torque*(1 - steering_sensor_input*turning_torque_multiplier_constant);
                torque_right = torque*(1 + steering_sensor_input*turning_torque_multiplier_constant);

            }

            else {
                steer_flag = 101;
                //no steering
                torque_left = torque;
                torque_right = torque;
            }

            //if torque value is out of range (0 to torque_max_for_motor) then setting the values to the extremum (0 or torque_max_for_motor)
            set_torque_in_range();

            if(brake_sensor_input>hard_brake_value && apps_input>0.25) {
                if (brake_flag == 0) {
                    t.start();
                    brake_flag = 1;
                } else if(t.read() > 0.5) {
                    torque_left = 0;
                    torque_right = 0;
                    brake_flag = 1;
                }

            } else if(brake_sensor_input<hard_brake_value && apps_input<0.05) {
                t.reset();
                t.stop();
                brake_flag = 0;
            } else if(brake_flag == 1) {
                torque_left = 0;
                torque_right = 0;
            }
            //else if(t.read()>0.5 && brake_flag==1 && (brake_sensor_input>hard_brake_value || apps_input>0.25) ) {
            //    torque_left = 0;
            //    torque_right = 0;
            //}


        }

        //check if battery power > max allowed.... if yes.... then adjust accordingly
        motor_powers_sum = torque_right*w_right + torque_left*w_left;
        battery_power = motor_powers_sum/battery_to_motors_efficiency;
        if(battery_power > max_battery_power_allowed) {
            float torque_adjustment_factor = max_battery_power_allowed/battery_power;
            torque_left = torque_adjustment_factor*torque_left;
            torque_right = torque_adjustment_factor*torque_right;
        }

        if(apps1_input == apps2_input) {
            torque_left = 0;
            torque_right = 0;
        }

        if(apps_normalised_difference>0.1) {
            if(apps_plausibility_timer.read() == 0) {
                apps_plausibility_timer.start();
            }

            if(apps_plausibility_timer.read() > 0.1) {
                torque_left = 0;
                torque_right = 0;
            } else {
                apps_plausibility_timer.reset();
            }
        }


        //print torque of both the wheels and other required values
        print_all_values();


    }

}


