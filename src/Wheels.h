// Wheels.h
//#include <>
#include <DigitalIO.h>

#ifndef _WHEELS_h
#define _WHEELS_h

/**@enum MinMaxRange
 * @brief Generalized enum type to denote min max quantifiers
 * 		  Used as a input parameter for getter and setter methods in Wheel and Drive4Wheel classes' 		  
 * */
enum MinMaxRange : uint8_t {
	MIN, /**< enum Minimum */
	MAX	 /**< enum Maximum */
}; 

/**@class 	Wheel
 * @brief	A class to initialize the Wheels with standard three-pin motor driver
 * 
 * @detail	Can be initialized and passed to a Drive4Wheel class bu also can be a
 * 			standalone object to spin wheels with the available methods. 
 * */
class Wheel {

public:
	/**@brief Default and only constructor for Wheel class
	 *  
	 * @detail Instantiates the Wheel object. 1 object for 1 Wheel. Instantiated before the Drive4Wheel Class
	 * 
	 * @param[out] 		pin1 					Digital Pin connected to the forward spin port on the driver, default = -1
	 * @param[out] 		pin2 					Digital Pin connected to the backward spin port on the driver, default = -1  
	 * @param[out] 		pinSetSpeed 			Analog pin connected to the speed/pulse control port on the driver, default = -1
	 * @param[in, out] 	minWheelAbsoluteSpeed	Minimum programmable 8-bit integer value for wheel speed, default = 120
	 * @param[in, out]  maxWheelAbsoluteSpeed	Maximum programmable 8-bit integer value for wheel speed, default = 255
	 * */
	Wheel(int pin1 = -1, int pin2 = -1, int pinSetSpeed = -1,
		int minWheelAbsoluteSpeed = 120, int maxWheelAbsoluteSpeed = 255); 
	
	/**@brief Default deep copy constructor
	 * */
	Wheel(const Wheel& AWheel); 
	
	/**@brief Initialization method called in the constructor of Wheel class
	 * */
	void initWheel();  //initialization function, called in the constructor 
	
	/**@enum Wheel::WheelState
	 * 		 Enum type to handle the states of the Wheel
	 *  */
	enum WheelState : uint8_t {
		WHEEL_NO_SPIN, /**< Wheel is not turning*/
		WHEEL_SPIN_FORWARD, /**< Wheel is turning forward relative to its pin connection.*/
		WHEEL_SPIN_BACKWARD /**< Wheel is turning backward relative to its pin connection.*/
	}; 

	/**
	 * @brief Getter method to return current Wheel state 
	 * @retval Wheel::WheelState
	*/
	Wheel::WheelState getCurrentWheelState(); 
	
	/**@brief Turns the wheel forward and set the Wheel state to Wheel::WHEEL_SPIN_FORWARD
	 * @param 8-bit integer value between (0-255) for the turn speed of the Wheel 
	*/
	void setSpinForward(int speed); 

	/**@brief Turns the wheel backward and sets the Wheel state to Wheel::WHEEL_SPIN_BACKWARD
	 * @param 8-bit integer value between (0-255) for the turn speed of the Wheel 
	*/
	void setSpinBackward(int speed); 

	/**@brief Stops the wheel from turning and sets the Wheel state to Wheel::WHEEL_NO_SPIN
	*/
	void setSpinStop(); 

	/**@brief Getter method to return the allowable Maximum and Minimum 8-bit speed values
	 * 
	 * Returns the minimum or maximum programmable speed value based on the enum type passed 
	 * MinMaxRange::Min returns minimum value and MinMaxRange::Max returns maximum value
	 * 
	 * @param rangeValue enum type MinMaxRange 
	 * @retval int
	*/
	int getWheelAbsoluteSpeed(MinMaxRange rangeValue); 

	/**@brief Setter method to set the allowable Maximum and Minimum 8-bit speed values
	 * 
	 * Ensures that any speed values passed to the spin methods are within this programmed range
	 * 
	 * @param minSpeedAbsolute Minimum 8-bit integer programmable value to get the wheel to spin
	 * @param maxSpeedAbsolute Maximum 8-bit integer programmable value to get the wheel to spin 
	*/
	void setWheelAbsoluteSpeed(int minSpeedAbsolute, int maxSpeedAbsolute); 

	//void getNumberOfWheels(); //method to return the number of wheels initialized 
	
private:
	 
	PinIO _pinForward; /**< Pin that turns wheel forward with a High (relative to the robot)*/
	PinIO _pinBackward; /**< Pin that turns wheel backward with a High (relative to the robot)*/
	int _pinSetSpeed; /**< Pin that controls motor speed, Analog Output Signal.*/
	Wheel::WheelState _spinState; /**<  The state/direction of Wheel spin*/
	int _minWheelAbsoluteSpeed;  /**< Minimum programmable integer value for speed the wheel can turn*/ 
	int _maxWheelAbsoluteSpeed;	/**< Maximum programmable integer value for speed the wheel can turn*/

	static int _numberOfWheels; /**Number of Wheel objects instantiated*/

	/**@brief Checks if integer value for Wheel speed is within the programmable range, if not clips it to the 
	 * 		  Min and Max Wheel absolute speeds
	 * 
	 * @param wheelSpeed Integer value for speed 
	 * @retval int
	 * */
	int _limitWheelSpeed(int wheelSpeed);  	
};

/**@class 	Drive4Wheel
 * @brief	A class to initialize and control Four-wheeled Differential Drive
 * 
 * @detail	Requires Wheel class object instantiated. Has method to perform standard actions of a
 * 			differential drive system.  
 * */
class Drive4Wheel {
public:
	/**@brief Default and only constructor for Drive4Wheel class
	 * 
	 * @param WheelObjects 4 addresses of Wheel objects 
	 * @param speedToleranceRange The integer tolerance range for drive system from the programmable speed values of wheels 
	 * */
	Drive4Wheel(Wheel& LeftFrontWheel, Wheel& RightFrontWheel,
		Wheel& LeftRearWheel, Wheel& RightRearWheel, int speedToleranceRange);   
	
	/**@brief Initialization method called in the constructor of Drive4Wheel class
	 * */
	void initDrive4Wheel();  

	/**@brief Method to check and limit the speed before invoking the drive methods for users
	 * 
	 * Limits the programmable integer value for drive speed within the tolerance range from the minimum and
	 * maximum programmable integer value for Wheel speeds.
	 * 
	 * @param driveSpeed 8-bit integer value for drive speeds
	 * @retval int   
	 * */
	int limitDriveSpeed(int driveSpeed); 
	
	/**@brief Method to drive forward by calling the Wheel::setSpinForward(driveSpeed)
	 * 
	 * @param speed 8-bit integer value for drive speeds  
	 * */
	void goForward(int speed);

	/**@brief Method to drive backward by calling the Wheel::setSpinBackward(driveSpeed)
	 * 
	 * @param speed 8-bit integer value for drive speeds  
	 * */
	void goBackward(int speed);

	/**@brief Method to turn left by varying the wheel turn directions
	 * 
	 * @param wheelSpeed 8-bit integer value for wheelSpeed
	 * @param speedRatio Sets the ratio of the Right Wheel Speed to Left Wheel Speed, default = 1.0  
	 * */
	void goLeft(int wheelSpeed, float speedRatio = 1.0);

	/**@brief Method to turn right by varying the wheel turn directions
	 * 
	 * @param wheelSpeed 8-bit integer value for wheelSpeed
	 * @param speedRatio Sets the ratio of the Left Wheel Speed to Right Wheel Speed, default = 1.0  
	 * */
	void goRight(int wheelSpeed, float speedRatio = 1.0);

	/**@brief Method to slight turn left by varying the wheel turn directions and speed
	 * 
	 * @param wheelSpeed int, 8-bit integer value for wheelSpeed
	 * @param speedRatio int, Sets the ratio of the Right Wheel Speed to Left Wheel Speed, default = 0.6
	 * @param reverse 	 bool, Sets the flag for drive in reverse if True, default = false
	 * */
	void swayLeft(int wheelSpeed, float speedRatio = 0.6, bool reverse = false);

	/**@brief Method to slight turn right by varying the wheel turn directions and speed
	 * 
	 * @param wheelSpeed int, 8-bit integer value for wheelSpeed
	 * @param speedRatio int, Sets the ratio of the Right Wheel Speed to Left Wheel Speed, default = 0.6
	 * @param reverse 	 bool, Sets the flag for drive in reverse if True, default = false
	 * */
	void swayRight(int wheelSpeed, float speedRatio = 0.6, bool reverse = false);

	/**@brief Method to stop(halt) the drive
	 * 
	 * 
	 * */
	void stop();

	/**@enum DriveState
	 * 		 Enum type for drive states  
	 * */
	enum DriveState : uint8_t {
		DRIVE_STOP, /**< Driving stopped*/
		DRIVE_FORWARD, /**< Driving forward*/
		DRIVE_BACKWARD, /**< Driving backward*/
		DRIVE_LEFT, /**< Driving left*/
		DRIVE_RIGHT, /**< Driving right*/
		DRIVE_FORWARD_LEFT, /**< Swaying left forward*/
		DRIVE_FORWARD_RIGHT, /**< Swaying right forward*/
		DRIVE_BACKWARD_LEFT, /**< Swaying left backward*/
		DRIVE_BACKWARD_RIGHT /**< Swaying right backward*/
	}; 

	/**@brief Getter method to return the speed tolerance range 
	 * 
	 * @retval int
	 * */ 
	int getSpeedToleranceRange();

	/**@brief Setter method to set the speed tolerance range 
	 * 
	 * @param speedTolerance Integer value for the new speed tolerance range 
	 * */ 
	void setSpeedToleranceRange(int speedTolerance);

	/**@brief Getter method to return the allowable Maximum and Minimum 8-bit drive speed values
	 * 
	 * Returns the minimum or maximum programmable drive speed value based on the enum type passed 
	 * MinMaxRange::Min returns minimum value and MinMaxRange::Max returns maximum value
	 * 
	 * @param rangeValue enum type MinMaxRange 
	 * @retval int
	*/
	int getDriveSpeed(MinMaxRange rangeValue);

	/**@brief Getter method to return the current drive system state 
	* 
	*  @retval Drive4Wheel::DriveState
	*/
	DriveState getCurrentDriveState();

private:
	
	int _maxDriveSpeed = 0; /**< Maximum programmable drive speed value (maxWheelSpeed - speedToleranceRange)*/
	int _minDriveSpeed = 0; /**< Minimum programmable drive speed value (minWheelSpeed + speedToleranceRange)*/
	DriveState _driveState; /**< Drive system state based on the Wheel spin states*/

	/**@brief Private method to set the _minDriveSpeed and _maxDriveSpeed 
	 * 
	 * @detail The speed values are set based on the speed tolerance range member value and the programmable
	 * 		   wheel speeds from the Wheel class objects. 
	 * */
	void _setDriveSpeed(); 
	
	Wheel* _LeftFrontWheel; /**< Pointer to front left Wheel object*/
	Wheel* _RightFrontWheel; /**< Pointer to front right Wheel object*/
	Wheel* _LeftRearWheel; /**< Pointer to rear left Wheel object*/
	Wheel* _RightRearWheel; /**< Pointer to rear right Wheel object*/
	int _speedToleranceRange; /**< The tolerance between the absolute speeds of the Wheel instance and allowable drive speed*/

	
};


#endif





