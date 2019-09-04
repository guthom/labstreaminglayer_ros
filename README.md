# Labstreaminglayer ros synchronizer
This repository includes the source code for the 
LSL-ROS-Synchronizer. This tool can be used to exchange data from ROS based environments to LSL based environments and vice versa. 

This is work in progress and we may not use all the whole potential and options both systems have to offer. If you have any suggestions or feedback do not hesitate to get in touch with us. 

### Prerequisites 
* Basic installation of **ROS**
* Installation of **LSL**

## Installing
* Clone the package and all the dependencies in your 
ROS-Workspace and build it with catkin.

## Usage
* just launch the included *labstreaminglayer.launch* file in your ros environment. 

    ```bash
    roslaunch labstreaminglayer_ros labstreaminglayer.launch 
    ```

Please note that this the launchfile automatically loads the two parameter files *labstreaminglayer.yaml* and *lsl_maps.yaml* see the description further down for details.

## Structure of the ROS-Package

    labstreaminglayer_ros
    ├── launch                              # ROS-Launchfiles
    ├── msg                                 # ROS-message files 
    │   ├── LSLHeader.msg                   # LSLHeader with timestamp
    │   ├── LSL*.msg                        # Specific ROS-message
    │   └── ... 
    ├── pysrc
    │   ├── Converter                       # Converters
    │   │   ├── __init__.py                 
    │   │   ├── ConverterBase.py            # Converter Base-Class 
    │   │   ├── *.py
    │   │   └── ...
    │   ├── nodes                           # Nodes Source
    │   ├── Testing                         # Test Scripts 
    │   └── data                            # Mapper etc.
    │   │   ├── ...
    ├── scripts                             # ROS-Nodes to run
    │   └── labstreaminglayer_syncronizer   # synchronizer node
    ├── yaml                                # Parameter Config  
    │   ├── labstreaminglayer.yaml          # Base Parameters 
    │   └── lsl_maps.yaml                   # Mapping Parameter
    └── ...

## Main Idea of the Synchronization
ToDo

## Configuration and Parameter
To configure package you need to edit the *.yaml files within the *Config* directory of the package.

* **labstreaminglayer.yaml**
    ```yaml
    cyclicMode: True
    useLSLTypesBidirectional: False
    includeLSLTimestamps: True
    ```
    * **cyclicMode (Bool)** - *Default: True*

        Set cyclicMode to true because the planned event-based synchronisation is not implemented yet :-)

    * **useLSLTypesBidirectional(Bool)** - *Default: False*

        set True if you want to use the specific LSL-Message Types (eg. LSLBool) on both ROS and LSL side. Keep it False allow for streaming standard ROS-Messages directly to LSL
    
    * **includeLSLTimestamps(Bool)** - *Default: True*

        set False if you do not want to use the specific LSL-messages with included timestamp. If set to False the synchronizer will stream the received LSL data with standard ROS-messages if they exist.  

* **lsl_maps.yaml**
    ```yaml
    maps:
        ROStoLSL:
        - {
            commonType: "Float32", 
            rostopic: "/float",
            lslChannelInfo:
            {
                topic: "/floatFromRos",
                contentType: "Testing",
                frequency: "30",
                UID: ""
            }
        }

        LSLtoROS:
        - {
            commonType: "Float32", 
            rostopic: "/floatFromlsl",
            lslChannelInfo:
            {
                topic: "/floatFromlsl",
                contentType: "Testing",
                frequency: "30",
                UID: ""
            }
        }
    ```
    * **ROStoLSL (List)** - *Default: Empty*

        Contains all maps from ROS -> LSL.

    * **LSLtoROS (List)** - *Default: Empty*
    
        Contains all maps from LSL -> ROS.

    * **commonType (String)**

        The common base type defined in the converter.
    
    * **rostopic (String)**

        The corresponding ROS-Topic to publish OR subscribe.

    * **lslChannelInfo (String)**

        The corresponding LSL-Channel info used to push OR pull from the LSL stream. 

## Add new types and Converters
To integrate new types you need to add new converters to the 
project. In order to do so, you need to follow this steps:

1. **Create a new LSL-specific ROS-Message**:

    The specific LSL-Messages are used to include the LSL-Timestamps within each message. Just create a new LSL*.msg definition according to the following example, place it in the ./msg/ directory and build the package again (eg. catkin_make)

   **LSLFloat32.msg**
   ```
   labstreaminglayer_ros/LSLHeader header
   float32 data
   ```

   A custom LSL-ROS-Messages just contains the LSLHeader (which will include the LSL-Timestamp as a single float64) and the specific data we want to sync between the systems. Feel free to expand the messages with additional data entries but make sure to map it correctly in the specific converter class (see 2.)  


2. **Create a new ConverterClass based on the *ConverterBase*-Class**:

    Second we need to create a new ConverterClass. Place the new converter in the *./pysrc/labstreaminglayer_ros/Converter* Directory and inherit from the *ConverterBase*-Class. Implement the converter methods **__ init __**, **ToLSL**, **ToROS** and **ToROSStd** as shown in the following example.
    
    **Float32.py**
    ```python
    from ConverterBase import ConverterBase
    # import the custom created message (see 1.)
    from labstreaminglayer_ros.msg import LSLFloat32 as message
    # import the ros standard message (if it exists)
    from std_msgs.msg import Float32 as stdmessage

    #create new class and inherit from  our ConverterBase-Class
    class Float32(ConverterBase):

        #implement Init method, basically just call the constructor of the base class with the custom parameters
        def __init__(self):
            super(Float32, self).__init__(
                commonType="float32",   #custom basetype name used in the mapping configuration
                rosType=message,        # contains the LSL specific message 
                rosStdType=stdmessage,  # contains the std ROS-Message if it does not exist set it to None
                lslChannels=1,          # amount of lsls channels used by this type
                lslType="float32"
                )

        #the following three methods have been taken from the base class ConverterBase. If you stick to the design recommendation, and you are able to wrap your new type into one common ROS-Type (data in the specific message cf. 1.) you may not need to implement this methods in your new converter. 


        #contains the routine we need to convert the data from ROS to a raw LSL-Stream
        def ToLSL(self, data):
            return [data.data]
        
        #contains the routine we need to convert the data from LSL to a STANDARD ROS message without LSL-Timestamps
        def ToROSStd(self, data):
            msg = self.rosStdType()
            msg.data = data[0][0]
            return msg

        #contains the routine we need to convert the data from a raw LSL-Streams to the specific ROS-Message
        def ToROS(self, data):
            #create LSL-Specific ROS-Message
            msg = message()
            #index 0 contains the data
            msg.data = data[0][0]
            #index 1 contains the LSL-timestamp
            msg.header.timestamp = data[1]

            return msg
    ```
    You should be able to do basically what you want within the converters you just need to stick to the defined ROS-message and LSL structures at the end.

3. **Add the new converter to the _ _init_ _ file of the converter directory**:

    Finally we need to add the new created Converter-Class file to the filename *./pysrc/labstreaminglayer_ros/Converter/__ init __.py*. like shown in the following example (last line):

    **/Converter/__ init __.py**
    ```python
    import ConverterBase
    import Bool
    import Float32
    import ExoDataArray
    import Int32
    import Image
    import Transform
    import TransformStamped
    import EEGLiveAmp
    #add new created converter name: 
    import AddNewConverterName
    ```
     
    Sadly we did not find a way to avoid this step yet (please contact me if you do so).

## Open Source Acknowledgments
This work uses parts from:
* **Robot Operation System (ROS)**  https://www.ros.org/
* **Lab Streaming Layer (LSL)** https://github.com/sccn/labstreaminglayer
* **

**Thanks to ALL the people who contributed to the projects!**

## Authors
* [**Thomas Gulde**](https://github.com/guthom) - [*Cognitive Systems Research Group*](https://cogsys.reutlingen-university.de/) - Reutlingen-University 

* [**Marius Nann**](https://github.com/MariusNann) -
[*Applied Neurotechnology Lab*](https://www.medizin.uni-tuebingen.de/de/en/Press/Our+Institutions+A+_+Z/Hospitals/Psychiatry+and+Psychotherapy/General+Psychiatry+and+Psychotherapy/Research/Applied+Neurotechnology+Lab.html) -  University Hospital Tübingen 





## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

## Citation
This work is just indirectly involved in our research. See other repositories for specific research projects and citations.

