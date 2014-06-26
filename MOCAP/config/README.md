NOTES ABOUT CONFIG.XML
====================================

SEPECIFICATIONS ORDER:
-----------------------------------

In general nodes in a same level can have their position exchanged, meaning that the
xml file is order-invariant. The same principle works among attributes of a node.
Although, for the "BodyParts" subnodes, the order is not totally invariant since there
are internal dependencies between some body parts.
Body parts which has one or more dependence must be specified after its dependences. For
instance, a head has to be defined after the thorax body part, but you can exchange arms
with legs without any trouble.

ATTRIBUTES NAMES AND NODES VALUES:

Points and vectors on the 3D space must be defined having 3 attributes named "X", "Y",
and "Z", meaning that they all are defined with respect to the same (arbitrary) XYZ 
referential. Exception: "Eye" which is specified by pitch, yaw and magnitude.
Colors in general must have 3 attributes named "r", "g" and "b" for red, green and blue.
An  exception occurs for the "OfflineColor" inside the "Body" node which has four 
attributes, the previous three plus one, named "a" for Alpha value.
The name of nodes and attributes has to be always the same; the parsing is case sensitive.
Theirs values have to obey a certain patter easily learned from the xml file itself and from
its comments.

CAMERA:

Sets parameters for the visualization of the 3D objects.

	*Center:
	Specifies the reference point towards which the camera will point.

	*Eye:
	Specifies the position of the camera (or eye) using pitch, yaw, and distance from the 
	center attributes.

ENVIROMENT:

Defines how the program will obtain gravity and geomagnetic field values.

	*EstimateGravityAtRunTime:
	Toggles between estimating the gravity value at run time and using the gravity value
	provided on the "Gravity" node.

	*RelativizeMagFieldForEachDevice:
	Toggles between: estimating and relativizing for each device the magnetic field
	value; and using the magnetic field value provided on the "MagField" node.

	*NumberOfSamplesUsed:
	Sets the number of IMU reading samples used for the two previous estimative 
	(in case they really occur).

	*Gravity:
	Local gravity value in mG.

	*MagField:
	Local magnetic field in mGaus.

DEVICECOMM:

Specifies some parameters for the device communi

	*RecvBoxDataWaitingTime:
	Specifies the time interval in [ms] between a data request to a Box and the
	acquisition attempt of that data.

	*RecvBoxDataTimeout:
	Amount of time in [ms] during which the program will try receive a
	successfully requested box data. After this time the box will be considered offline.

	*LargestBoxID:
	The largest Box identification number. Devices having a ID greater than this value
	will be considered an Android based device and the communication will occurs
	differently.

	*RecvSmartPhoneDataWaitingTime:
	Same as RecvBoxDataWaitingTime but for the Android based device.

	*RecvSmartPhoneDataTimeout:
	Same as RecvBoxDataTimeout but for the Android based device.

	*TCPServerBasePort:
	Specifies a base port number for the TCP communication between computer and 
	possible Android based devices.

	*TCPServerListeningTimeout:
	Specifies the timeout in [ms] for the listening stage of the TCP connection with Android
	based devices. A given Android device has to send a connection request during this
	time, otherwise it will be considered an offline device.

GENERALFILTERSETTINGS:

	*AccMagValidationGate:
	Specifies the parameters for the gate filtering of accelerometer and magnetometer 
	sensors.
	
	>epsa defines the largest difference between the gravity magnitude and the
	calibrated accelerometer output magnitude for which this output will still 
	be used for orientation correction. Given in [mG].
	
	>epsm is the same as epsa but with respect to the magnetic field and calibrated
	magnetometer output. Given in [mGaus]
	
	>epsdip defines the largest difference between the pitch angle of the magnetic field
	and the pitch angle of the calibrated magnetometer output for which this output will
	still be used for orientation correction. Given in [rad].

	*HightCovValue:
	Specifies a hight value that will be used to invalidate the sensors output effect on
	the orientation correction if need be.

BACKGROUND:

Specifies the all scene components besides the body.

	*BackGroundColor:
	Back ground color defined as RGB.

