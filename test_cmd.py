import os
import xml.etree.ElementTree as ET

net_path = R'Map4_Tsinghua_Intersection\grid-map.net.xml'
trip_path = R'trips.xml'

start="gneE664"
end="zhongguancundajie_out"
tree = ET.parse(trip_path)
root = tree.getroot()
route_info = root.find('trip')
route_info.attrib['from'] = str(start)
route_info.attrib['to'] = str(end)
tree=ET.ElementTree(root)
tree.write(trip_path)
command1 = R'duarouter --route-files ' + trip_path + ' --net-file ' + net_path + ' --output-file result.rou.xml'
os.system(command1)


