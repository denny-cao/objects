import xml.etree.ElementTree as ET

tree = ET.parse("shape.urdf.xacro")
x = tree.find('{http://www.ros.org/wiki/xacro}property[@name="use_cylinder"]').set('value', 'true')
