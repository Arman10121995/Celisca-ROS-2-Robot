"""Native vendor hazards retain source geometry and scoped ROS bridge topics."""
import xml.etree.ElementTree as ET

from robot_lab_utils.vacuum_sensors import create3_hazard_derivative, hazard_bridge_entries


def test_source_hazards_restore_without_changing_links_or_joints():
    links = ['bumper', 'cliff_side_left', 'cliff_side_right', 'cliff_front_left', 'cliff_front_right']
    body = '<link name="base_link"><inertial><mass value="4"/></inertial></link>'
    body += ''.join(f'<link name="{n}"/><joint name="{n}_joint" type="fixed">'
                    f'<parent link="base_link"/><child link="{n}"/>'
                    '<origin xyz=".1 .05 .02" rpy="0 1.4 .2"/></joint>' for n in links)
    original = '<robot name="source">' + body + '</robot>'
    blocks = ''.join(f'<gazebo reference="{n}"><sensor name="{n}" type="gpu_lidar">'
                     '<update_rate>62</update_rate><lidar><range><min>.0001</min><max>.15</max>'
                     '</range></lidar></sensor></gazebo>' for n in links if n.startswith('cliff_'))
    blocks += '<gazebo reference="bumper"><sensor name="bumper_contact_sensor" type="contact">' \
              '<update_rate>62</update_rate><contact><collision>bumper_collision</collision></contact>' \
              '</sensor></gazebo>'
    result, sensors = create3_hazard_derivative(original, original.replace('</robot>', blocks+'</robot>'),
                                                'cleaning_variant')
    root, before = ET.fromstring(result), ET.fromstring(original)
    assert len(sensors) == 5 and root.get('name') == 'cleaning_variant'
    for tag in ('link', 'joint'):
        assert [ET.tostring(x) for x in root.findall(tag)] == [ET.tostring(x) for x in before.findall(tag)]
    assert all(s.findtext('update_rate') == '20' for s in root.findall('gazebo/sensor'))
    assert root.findtext("gazebo/sensor/lidar/range/max") == '.15'
    bridges = hazard_bridge_entries(result, 'selected_world', 'cleaning_variant')
    assert len(bridges) == 5
    bumper = next(b for b in bridges if b['ros_topic_name'] == '/bumper')
    assert bumper['gz_topic_name'] == '/world/selected_world/model/cleaning_variant/link/bumper/sensor/bumper_contact_sensor/contact'


def test_authored_vacuum_cliff_topic_does_not_gain_link_suffix():
    description = '<robot><gazebo reference="cliff_front_left_link">' \
        '<sensor name="cliff" type="gpu_lidar"><topic>/cliff/front_left</topic></sensor></gazebo></robot>'
    entry, = hazard_bridge_entries(description, 'world', 'oomwoo')
    assert entry['ros_topic_name'] == entry['gz_topic_name'] == '/cliff/front_left'
