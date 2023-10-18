#include <iostream>
#include <math.h>
int sens_ele_vec [] = {3,54,7,8,90,1};
int  sens_azi_vec [] = {3,54,7,8,90,1};
float  sens_norm [] = {3,54,7,8,90,1};
float  normalized_adc_val [] = {3,54,7,8,90,1};

float azi_angle_between_sens = 72; // Azimuth angle between the sensors in the same row
float ele_angle_between_sens = 15; // Elevation angle between the sensors in the same column
float min_voltage =  1.8406; // Minimum read voltage from sensors

int azi_vec_size = sizeof(sens_azi_vec) / sizeof(sens_azi_vec[0]); // Number of elements in sens_azi_vec

int ele_vec_size = sizeof(sens_ele_vec) / sizeof(sens_ele_vec[0]); // Number of elements in sens_ele_vec
int adc_vec_size = sizeof(normalized_adc_val) / sizeof(normalized_adc_val[0]); // Number of elements in normalized_adc_val 

// Does matrix multiplication
float * matrix_multiplicator(float *matrix1, float *matrix2, int matrix1_row, int matrix1_col, int matrix2_col)
{
    float result[matrix1_row][matrix2_col];
    float * address;
   address=(float*)malloc((matrix1_row*matrix2_col*matrix1_col)*sizeof(float));
    
   
    for(int i = 0; i < matrix1_row; i++)    
    {    
        for(int j = 0; j < matrix2_col; j++)    
        {     
            for(int k = 0; k < matrix1_col; k++)    
            {    
                    result[i][j] += *(matrix1 + (i * 3) + k) * *(matrix2 + (k *3) + j); 
            }   
                
        }    
    }    
    address = *result;
    return address;
   
  
}

float function1(int matrix_num){
	float * mat_add;
	 int sens_azi = 45;
    int sens_ele = 30;
	float rotz[3][3];
    float roty[3][3];
    rotz [0][0] = cos(sens_azi * 3.141592653589793238463 / 180);
    rotz [1][0] = sin(sens_azi * 3.141592653589793238463 / 180);
    rotz [2][0] = 0;
    rotz [0][1] = -sin(sens_azi * 3.141592653589793238463 / 180);
    rotz [1][1] = cos(sens_azi * 3.141592653589793238463 / 180);
    rotz [2][1] = 0;
    rotz [0][2] = 0;
    rotz [1][2] = 0;
    rotz [2][2] = 1;
    // End of definition of rotation matrix at z axis

    // Definition of rotation matrix at y axis
    roty [0][0] = cos(-sens_ele * 3.141592653589793238463 / 180);
    roty [1][0] = 0;
    roty [2][0] = -sin(-sens_ele * 3.141592653589793238463 / 180);
    roty [0][1] = 0;
    roty [1][1] = 1;
    roty [2][1] = 0;
    roty [0][2] = sin(-sens_ele * 3.141592653589793238463 / 180);
    roty [1][2] = 0;
    roty [2][2] = cos(-sens_ele * 3.141592653589793238463 / 180);
   
 mat_add = matrix_multiplicator(*rotz, *roty, 3, 3, 3);
	printf("%f\n",*(mat_add+matrix_num));
	
	
}

// Calculates the sensor's rotation matrix for the earth
void rot_matrix_calc(float sens_azi, float sens_ele)
{
    float rotz [3][3];
    float roty [3][3];
    float * address, *rot_mat;


    // Definition of rotation matrix at z axis
    rotz [0][0] = cos(sens_azi * 3.141592653589793238463 / 180);
    rotz [1][0] = sin(sens_azi * 3.141592653589793238463 / 180);
    rotz [2][0] = 0;
    rotz [0][1] = -sin(sens_azi * 3.141592653589793238463 / 180);
    rotz [1][1] = cos(sens_azi * 3.141592653589793238463 / 180);
    rotz [2][1] = 0;
    rotz [0][2] = 0;
    rotz [1][2] = 0;
    rotz [2][2] = 1;
    // End of definition of rotation matrix at z axis

    // Definition of rotation matrix at y axis
    roty [0][0] = cos(-sens_ele * 3.141592653589793238463 / 180);
    roty [1][0] = 0;
    roty [2][0] = -sin(-sens_ele * 3.141592653589793238463 / 180);
    roty [0][1] = 0;
    roty [1][1] = 1;
    roty [2][1] = 0;
    roty [0][2] = sin(-sens_ele * 3.141592653589793238463 / 180);
    roty [1][2] = 0;
    roty [2][2] = cos(-sens_ele * 3.141592653589793238463 / 180);
    //End of definiton of rotation matrix at y axis


    address = matrix_multiplicator(*rotz,*roty,3,3,3); // Adress of the rotation array
    
    for (int i = 0; i < 3; i ++)
    {
        for (int j = 0; j < 3; j ++)
        {
        	float rot_mat[i][j]= {{0}};        	
            rot_mat[i][j] = *(address + (i*3) + j);
        }
    }
}

// Calculates sensor's normal vector according to earth
float * sens_earth_norm_vec_calc(int sens_ele_vec[], int sens_azi_vec[], float sens_norm_vec [])
{
    int i = 0;

    int sens_ele;
    int sens_azi;

    float * R_sens_add; // Address of the rotation matrix of the sensor

    float * sens_earth_norm_vec_add[adc_vec_size];

    for (int elevation_rank = 0; elevation_rank < ele_vec_size; elevation_rank ++)
    {
        sens_ele = sens_ele_vec[elevation_rank]; // Selects the sensor elevation
        
        for (int azimuth_rank = 0; azimuth_rank < azi_vec_size; azimuth_rank ++)
        {
            sens_azi = sens_azi_vec[azimuth_rank]; // Selects the sensor azimuth

            if (sens_ele == 90) // Special case for elevation = 90
            {
                sens_azi = 90; 
            }

            R_sens_add = rot_matrix_calc(sens_azi, sens_ele); // Calculates the rotation matrix

            sens_earth_norm_vec_add[i] = matrix_multiplicator(R_sens_add, sens_norm_vec,3,3,1); // Calculates the sensors normal vector according to earth and saves it's address to an array
            i ++;
            if (sens_ele == 90) // Break if all sensors are scanned
            {
                break; 
            }
        }
    }

    return *sens_earth_norm_vec_add; // Returns the first adress of the array (means first sensor's x component)
}

// Calculates the azimuth angle of the sun
float azi_calc(int  normalized_adc_val[])
{
    float total_v = 0;
    
    float current_azi_vec[3][1]; // Last calculated azimuth vector up to some time in the code
    float total_azi_vec[3][1]; // Sum of all azimuth vectors of the sensors
    
    float wa_azi_vec[3][1]; // Weighted average of the total_azi_vec
    
    float estimated_azi_rad;
    float estimated_azi_deg;
    
    float * sens_earth_norm_vec_add;

    float sens_volt;
    float sens_volt_arr [adc_vec_size];
    int sens_num = 0; // Sensor number for tracking which sensor is being read

    sens_earth_norm_vec_add = sens_earth_norm_vec_calc(sens_ele_vec, sens_azi_vec, sens_norm); // Address of sensor's normal vector according to earth

    for (int i = 0; i < adc_vec_size; i ++)
    {
        sens_volt = (normalized_adc_val[i] / 2^12) *  3.3; // Volt values obtained from ADC inputs
        sens_volt_arr[i] = sens_volt; // Array which keeps the volts values for each sensor

        total_v = total_v +  sens_volt; // Sum of the volts of all sensors

        for (int j = 0; j < 2; j ++)
        {
            current_azi_vec[j][0] = sens_volt * *(sens_earth_norm_vec_add + sens_num + j); // Azimuth vector of the sun according to current sensor
            total_azi_vec[j][0] = total_azi_vec[j][0] + current_azi_vec[j][0]; // Sum of all sun azimuth vectors 
            wa_azi_vec[j][0] = total_azi_vec[j][0] / total_v; // Weighted average of sun azimuth vectors
        }
        sens_num += 3; // Added 3 because sens_earth_norm_vec_add array keeps x y z axises
    }

    estimated_azi_rad = atan2(wa_azi_vec[1][0],wa_azi_vec[0][0]); // Sun's azimuth angle in radians
    estimated_azi_deg = estimated_azi_rad * (180.0 / 3.141592653589793238463); // Sun's azimuth angle in degrees

    if (estimated_azi_deg < 0) estimated_azi_deg += 360; // Shifts the azimuth angle 360 degrees if necessary
    
    return estimated_azi_deg;
}

// Calculates the elevation angle of the sun
float ele_calc(float estimated_azi, int normalized_adc_val [])
{
    int temp = estimated_azi/azi_angle_between_sens; // Temporary variable used for a bug fix
    int index; // Index of the selected sensor azimuth branch
    
    int selected_azi_branch;
    float sens_volt;
    float sens_volt_arr [adc_vec_size]; 
    
    int max_volt, sec_max_volt, third_max_volt = 0;
    int max_index, sec_max_index, third_max_index; // Indexes of max 3 values of the selected_branch_voltage array

    int mid_point, max3_max_mid_point, max3_mid_mid_point, max2_mid_mid_point, midmid_point, mid_sensor, mid_mid_sensor; // Middle points between selected voltage values

    int closest_sens1, closest_sens2; // Closest sensors to the sun in terms of elevation angle

    int elevation_diff;
    int estimated_ele;

    // Selection the azimuth branch
    if (temp % 1 > 0.5)
    {
        selected_azi_branch = azi_angle_between_sens * ceil(temp); // Select the upper row
    }
    else if (temp % 1 <= 0.5)
    {
        selected_azi_branch = azi_angle_between_sens * floor(temp); // Select the lower row
    }
    else
    {
        selected_azi_branch = azi_angle_between_sens * floor(temp); // Select the lower row for preventing any bugs
    }

    if (selected_azi_branch < 0)
    {
        selected_azi_branch += 360;
    }
    // End of the selection of the azimuth branch

    // Finding the index of the selected azimuth branch in the sens_azi_vec array
    for (int i = 1; i < azi_vec_size; i ++)
    {
        if (selected_azi_branch == sens_azi_vec[i])
        {
            index = i;
        }
    }
    // End of finding the index of the selected azimuth branch in the sens_azi_vec array

    // Calculation of the volts of sensors 
    for (int i = 0; i < adc_vec_size; i ++)
    {
        sens_volt = (normalized_adc_val[i] / 2^12) *  3.3; // Calculate the sensor volt
        sens_volt_arr[i] = sens_volt;
    }
    //End of the calculation of the volts of sensors

    // Insertion of the voltages of the selected sensor column to an array
    float selected_branch_voltage [] = {sens_volt_arr[index], sens_volt_arr[index + 5], sens_volt_arr[index + 10], sens_volt_arr[index + 15], sens_volt_arr[index + 20], sens_volt_arr[index + 25], sens_volt_arr[31]};
    // End of insertion of the voltages of the selected sensor column to an array

    int branch_voltage_size = sizeof(selected_branch_voltage) / sizeof(selected_branch_voltage[0]); // Number of elements in the selected_branch_voltage array

    // Finding of the maximum three values and the indexes of those values in the selected_branch_voltage array
    for (int i = 0; i < branch_voltage_size; i++)
    {
        if(selected_branch_voltage[i] > max_volt)
        {
            max_volt = selected_branch_voltage[i]; // Find the max value
            max_index = i;
        }

        if(selected_branch_voltage[i] > sec_max_volt  && selected_branch_voltage[i] != max_volt)
        {
            sec_max_volt = selected_branch_voltage[i]; // Find the second max value
            sec_max_index = i;
        }

        if(selected_branch_voltage[i] > third_max_volt && selected_branch_voltage[i] != sec_max_volt && selected_branch_voltage[i] != max_volt)
        {
            third_max_volt = selected_branch_voltage[i]; // Find the third max value
            third_max_index = i;
        }
    }
    // End of the finding of the maximum three values and the indexes of those values in the selected_branch_voltage array

    // Middle points between selected voltage values
    mid_point = sec_max_volt + (max_volt - sec_max_volt) / 2;
    max3_max_mid_point = third_max_volt + (max_volt - third_max_volt) / 2;
    max3_mid_mid_point = third_max_volt + (max3_max_mid_point + max_volt) / 2;
    max2_mid_mid_point = (sec_max_volt + mid_point) / 2;
    midmid_point = (max_volt + mid_point) / 2;

    // Determinition of the closest sensors to the sun in terms of elevation angle
    closest_sens1 = (max_index - 1) * ele_angle_between_sens;
    closest_sens2 = (sec_max_index - 1) * ele_angle_between_sens;

    mid_sensor = (closest_sens1 + closest_sens2) / 2;
    mid_mid_sensor = (mid_sensor + closest_sens1) / 2;

    if(max2_mid_mid_point < max3_mid_mid_point)
    {
        if (closest_sens1 > closest_sens2)
        {
            elevation_diff = ((max_volt - min_voltage) * ele_angle_between_sens / 4) / ((midmid_point - min_voltage) + (max_volt - min_voltage));
        }
        else
        {
            elevation_diff = ((max_volt - min_voltage) * ele_angle_between_sens / 4) / ((midmid_point - min_voltage)+(max_volt - min_voltage));
            estimated_ele = closest_sens1 + elevation_diff;
        }
    }

    else
    {
        if (closest_sens1 > closest_sens2)
        {
            elevation_diff = ((mid_point - min_voltage) * ele_angle_between_sens / 4) / ((midmid_point - min_voltage) + (max_volt - min_voltage));
            estimated_ele = mid_sensor + elevation_diff;
        }
        else
        {
            elevation_diff = ((mid_point - min_voltage) * ele_angle_between_sens / 4) / ((midmid_point - min_voltage) + (max_volt - min_voltage));
            estimated_ele = mid_sensor - elevation_diff;
        }
    }
            
    return estimated_ele;

}

int main()
{
    int sens_azi = 45;
    int sens_ele = 30;
    float rotz[3][3];
    float roty[3][3];
    float * address;
    float current_value;
    int sens_num = 0;
    rotz [0][0] = cos(sens_azi * 3.141592653589793238463 / 180);
    rotz [1][0] = sin(sens_azi * 3.141592653589793238463 / 180);
    rotz [2][0] = 0;
    rotz [0][1] = -sin(sens_azi * 3.141592653589793238463 / 180);
    rotz [1][1] = cos(sens_azi * 3.141592653589793238463 / 180);
    rotz [2][1] = 0;
    rotz [0][2] = 0;
    rotz [1][2] = 0;
    rotz [2][2] = 1;
    // End of definition of rotation matrix at z axis

    // Definition of rotation matrix at y axis
    roty [0][0] = cos(-sens_ele * 3.141592653589793238463 / 180);
    roty [1][0] = 0;
    roty [2][0] = -sin(-sens_ele * 3.141592653589793238463 / 180);
    roty [0][1] = 0;
    roty [1][1] = 1;
    roty [2][1] = 0;
    roty [0][2] = sin(-sens_ele * 3.141592653589793238463 / 180);
    roty [1][2] = 0;
    roty [2][2] = cos(-sens_ele * 3.141592653589793238463 / 180);
    
   // roty[3][3]=[cos(-sens_ele * 3.141592653589793238463 / 180),0,sin(-sens_ele * 3.141592653589793238463 / 180);0,1,0;-sin(-sens_ele * 3.141592653589793238463 / 180),0,cos(-sens_ele * 3.141592653589793238463 / 180)];
  
    // address = rot_max(*rotz, *roty, 3, 3, 3);
    rot_matrix_calc(45,30);
    /*for(int i=0; i<8; i++){
       
      printf("%f\n",i+ *(mat_add));
		 float delay(1000);
		
		  } */
    
    for(int i=0; i<3; i++){
    	
    	for(int j=0; j<3; j++){
    		
    		//float rot_mat[i][j];
			  
		printf("%f\n", rot_mat[i][j]);	  
		 }
		 
	 }
	 
	 //float a= *address+1;
	 //printf("%f\n",a);
  
    /*function1(0);
    function1(1);
    function1(2);
    function1(3);
    function1(4);
    function1(5);
    function1(6);
    function1(7);
    function1(8);  */

}