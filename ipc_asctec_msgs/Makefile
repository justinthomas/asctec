PKG= ipc_asctec_msgs
MEX_MSGS= PDCmd DMCmd HWCmd #Voltage
ROS_MSGS= PDCmd DMCmd HWCmd #Voltage

MEX_FILES := $(foreach msg, $(MEX_MSGS), asctec_$(msg))
#ROS_FILES := $(foreach msg, $(ROS_MSGS), asctec_$(msg)_publisher)
ROS_FILES += $(foreach msg, $(ROS_MSGS), asctec_$(msg)_subscriber)

include $(shell rospack find ipc_roslib)/ipc_roslib.mk
include $(shell rospack find ipc_geometry_msgs)/ipc_geometry_msgs.mk
include $(shell rospack find ipc_bridge_local)/ipc_bridge_local.mk