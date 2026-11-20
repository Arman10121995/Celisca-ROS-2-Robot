"""Compile the facility graph with Humble's actual SDK and plan both lift legs."""
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

from rmf_adapter import geometry, graph, plan, vehicletraits as traits
import yaml

from robot_lab_utils.fleet_building_graph import building_graph


def run(output):
    folder = Path('src/robot_lab_maps/maps/fleet_facility')
    config = yaml.safe_load((folder/'infrastructure.yaml').read_text())
    floors = {'L0': 'map.yaml', 'L1': 'map_L1.yaml'}
    members = [dict(id='molar', model='molar_hri', radius=.49, spawn=dict(x=1.5, y=0.))]
    session = dict(map='fleet_facility', members=members,
                   floor_maps={level: str(folder/'maps'/name) for level, name in floors.items()})
    building, _ = building_graph(session, config)
    path = output/'building.yaml'
    path.write_text(yaml.safe_dump(building, sort_keys=False))
    profile = traits.Profile(geometry.make_final_convex_circle(.49), geometry.make_final_convex_circle(.64))
    vehicle = traits.VehicleTraits(traits.Limits(.25, .3), traits.Limits(.6, .8), profile)
    parsed = graph.parse_graph(str(path), vehicle)
    planner = plan.Planner(plan.Configuration(parsed, vehicle))
    result = {}
    directions = [('L0', 'L1', 0.), ('L1', 'L0', math.pi)]
    for origin, destination, yaw in directions:
        start = parsed.find_waypoint(origin+'_landing').index
        goal = parsed.find_waypoint(destination+'_landing').index
        route = planner.get_plan_waypoints(plan.Start(datetime.now(timezone.utc), start, yaw), plan.Goal(goal))
        assert route and route[-1].graph_index == goal
        assert parsed.find_waypoint('lift_'+origin+'_inside').index in {row.graph_index for row in route}
        assert parsed.find_waypoint('lift_'+destination+'_inside').index in {row.graph_index for row in route}
        assert any(row.event is not None for row in route)
        result[origin+'→'+destination] = [dict(pose=row.position.tolist(), index=row.graph_index,
                                               event=row.event is not None) for row in route]
    (output/'sdk-plan.json').write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    run(Path(sys.argv[1]))
