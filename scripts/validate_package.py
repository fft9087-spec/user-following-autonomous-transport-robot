"""Read-only static checks; does not command the robot or require ROS imports."""
from pathlib import Path
import ast, re, xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[1]
pkg=root/'src/physical_ai_robot_description'
count=0
for path in root.rglob('*.py'):
    ast.parse(path.read_text('utf-8')); count+=1
for path in list(pkg.rglob('*.xacro'))+list(pkg.rglob('*.world'))+[pkg/'package.xml']:
    ET.parse(path)
assert (pkg/'maps/corridor_map.pgm').stat().st_size>100
for path in pkg.rglob('*.xacro'):
    for rel in re.findall(r'package://physical_ai_robot_description/([^"<>]+)',path.read_text('utf-8')):
        if '${' not in rel: assert (pkg/rel).is_file(),rel
assert 'use_astar: false' in (pkg/'config/nav2_mdh100.yaml').read_text('utf-8')
assert len(list((pkg/'meshes/cad_v11').glob('*.dae')))==12
print('PASS: Python syntax, XML, fixed mesh references, map and selected Dijkstra profile;',count,'Python files')
