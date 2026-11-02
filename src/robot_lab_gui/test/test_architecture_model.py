"""Source-linked teaching topology, safe exports and networking lesson contracts."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/robot_lab_gui'))
from robot_lab_gui.architecture_content import COMPONENTS,VIEWS,JOURNEYS,VIEW_BY_ID,repository_packages
from robot_lab_gui.architecture_graph import network_lesson,observed_view,svg_diagram


def test_teaching_graph_and_tours_link_real_source_files():
    assert len(VIEWS)==len(VIEW_BY_ID)==12
    covered=set()
    for view in VIEWS:
        assert view['positions']
        covered.update(view['positions'])
        assert set(view['positions'])<=set(COMPONENTS)
        for edge in view['edges']:
            assert edge['source'] in view['positions'] and edge['target'] in view['positions']
    assert covered==set(COMPONENTS)
    for row in COMPONENTS.values():
        assert row['detail'] and row['summary']
        assert all((ROOT/path).exists() for path in row['sources']+[row['guide']])
    for steps in JOURNEYS.values():
        for view,key in steps:assert key in VIEW_BY_ID[view]['positions']


@pytest.mark.parametrize('view',VIEWS,ids=lambda view:view['id'])
def test_vector_export_is_standalone_xml_with_named_components(view):
    exported=ET.fromstring(svg_diagram(view,COMPONENTS))
    text=' '.join(exported.itertext())
    assert view['title'] in text and 'not live telemetry' in text
    assert len(exported.findall('.//{http://www.w3.org/2000/svg}g'))==len(view['positions'])


def settings():
    return {role+'_'+name:value for role in ('publisher','subscriber') for name,value in
            [('domain','42'),('ns','/alpha'),('topic','cmd_vel'),('type','geometry_msgs/msg/Twist'),('reliability','reliable'),('durability','volatile')]}


@pytest.mark.parametrize('field,value,expected',[
    ('subscriber_domain','43','DDS domain'),('subscriber_ns','/beta','Resolved topic'),
    ('subscriber_type','nav_msgs/msg/Odometry','Message type'),
    ('publisher_reliability','best_effort','Reliability'),('subscriber_durability','transient_local','Durability')])
def test_network_lesson_explains_each_independent_mismatch(field,value,expected):
    data=settings();data[field]=value
    assert [name for name,ok,reason in network_lesson(data) if not ok]==[expected]


def test_network_lesson_resolves_absolute_names_and_requested_offered_direction():
    data=settings();data.update(subscriber_ns='/beta',subscriber_topic='/alpha/cmd_vel',subscriber_reliability='best_effort',publisher_durability='transient_local')
    assert all(ok for name,ok,reason in network_lesson(data))
    data['subscriber_domain']='not a domain'
    assert network_lesson(data)[0][1] is False


def test_observed_graph_has_actual_types_directions_and_no_fabricated_endpoints():
    # Explicit fixture, not a live graph or mission receipt.
    snapshot=dict(nodes=[dict(name='/alpha/source',namespace='/alpha',publishers=[dict(name='/alpha/data',types=['std_msgs/msg/String'])],subscribers=[],services=[])],
                  topics={'/alpha/data':['std_msgs/msg/String']},scope='Unit fixture')
    view,rows=observed_view(snapshot)
    assert view['edges']==[dict(source='node:/alpha/source',target='topic:/alpha/data',label='publishes')]
    assert rows['topic:/alpha/data']['summary']=='std_msgs/msg/String'
    empty,rows=observed_view(snapshot,query='unmatched')
    assert empty['edges']==[] and list(rows)==['empty']


def test_package_inventory_uses_top_level_manifests_not_vendored_parts():
    rows=repository_packages(ROOT)
    assert any(row['name']=='robot_lab_gui' and 'rclpy' in row['dependencies'] for row in rows)
    assert any(row['name']=='robot_lab_registry' for row in rows)
    assert not any('_upstream' in row['path'] or '/unitree/' in row['path'] for row in rows)
