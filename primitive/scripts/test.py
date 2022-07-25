import xml.etree.ElementTree as ET

namespaces = {"xacro": "http://www.ros.org/wiki/xacro"}
ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")

print(ET.parse("shape.urdf.xacro").find('xacro:property[@name="static"]', namespaces))
