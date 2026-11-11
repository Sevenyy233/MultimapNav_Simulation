#!/usr/bin/env python3
"""Validate all map assets, collision coordinates, cloud payloads and spawn support."""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from stl_to_navigation_pcd import read_binary_stl
from world_to_mesh import resolve_uri, world_triangles, pose_matrix, transform, box_triangles


def read_pcd(path):
    header = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline().decode("ascii").strip()
            if not line:
                raise ValueError(f"Invalid PCD header: {path}")
            if line.startswith("#"):
                continue
            key, value = line.split(" ", 1)
            header[key] = value
            if key == "DATA":
                break
        assert header["FIELDS"] == "x y z" and header["DATA"] == "binary", path
        assert header["SIZE"] == "4 4 4" and header["TYPE"] == "F F F", path
        points = np.frombuffer(stream.read(), dtype="<f4").reshape(-1, 3)
    assert len(points) == int(header["POINTS"]) == int(header["WIDTH"]), path
    assert header["HEIGHT"] == "1" and np.isfinite(points).all(), path
    return points


def support_height(triangles, pose):
    x, y, limit, _ = pose
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    det = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    valid = det > 1e-9  # Upward-facing projected triangles.
    u, v = np.zeros(len(triangles)), np.zeros(len(triangles))
    u[valid] = (((b[:, 1] - c[:, 1]) * (x - c[:, 0]) + (c[:, 0] - b[:, 0]) * (y - c[:, 1]))[valid] / det[valid])
    v[valid] = (((c[:, 1] - a[:, 1]) * (x - c[:, 0]) + (a[:, 0] - c[:, 0]) * (y - c[:, 1]))[valid] / det[valid])
    height = u * a[:, 2] + v * b[:, 2] + (1 - u - v) * c[:, 2]
    valid &= (u >= -1e-6) & (v >= -1e-6) & (u + v <= 1 + 1e-6) & (height < limit + .01)
    if not valid.any():
        raise AssertionError(f"No collision surface supports spawn {pose}")
    return float(height[valid].max())


def validate_static_geometry(package, world, triangles):
    """Check coverage of each physical solid without requiring identical
    triangulation of Gazebo primitives and Blender meshes.

    Every exported vertex must lie on a static collision surface, and every
    solid must have its mesh vertices or primitive extremities in the export.
    This also detects accidentally baking moving actors into the static mesh.
    """
    vertices=np.unique(triangles.reshape(-1,3),axis=0)
    tree=cKDTree(vertices);residual=np.full(len(vertices),np.inf)
    root=ET.parse(world).getroot().find('world')
    for model in root.findall('model'):
        if any(p.get('filename')=='libmultimap_obstacle_motion.so'for p in model.findall('plugin')):continue
        model_pose=pose_matrix(model.findtext('pose'))
        for link in model.findall('link'):
            link_pose=model_pose@pose_matrix(link.findtext('pose'))
            for collision in link.findall('collision'):
                matrix=link_pose@pose_matrix(collision.findtext('pose'))
                local=(vertices-matrix[:3,3])@matrix[:3,:3]
                geom=collision.find('geometry');shape=list(geom)[0]
                if shape.tag=='mesh':
                    path=resolve_uri(shape.findtext('uri'),package,world.parent)
                    mesh=read_binary_stl(path)*np.fromstring(shape.findtext('scale','1 1 1'),sep=' ')
                    samples=np.unique(transform(mesh,matrix).reshape(-1,3),axis=0)
                    distances=cKDTree(samples).query(vertices)[0]
                    tolerance=3e-5
                elif shape.tag=='box':
                    size=np.fromstring(shape.findtext('size'),sep=' ')
                    samples=transform(box_triangles(size),matrix).reshape(-1,3)
                    d=np.abs(local)-size/2
                    distances=np.abs(np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(np.max(d,axis=1),0))
                    tolerance=3e-5
                elif shape.tag=='cylinder':
                    radius=float(shape.findtext('radius'));half=float(shape.findtext('length'))/2
                    angles=np.arange(32)*2*np.pi/32
                    samples=np.array([[radius*np.cos(a),radius*np.sin(a),z]for a in angles for z in [-half,half]])
                    samples=samples@matrix[:3,:3].T+matrix[:3,3]
                    d=np.column_stack((np.linalg.norm(local[:,:2],axis=1)-radius,np.abs(local[:,2])-half))
                    distances=np.abs(np.linalg.norm(np.maximum(d,0),axis=1)+np.minimum(np.max(d,axis=1),0))
                    tolerance=3e-5
                elif shape.tag=='sphere':
                    radius=float(shape.findtext('radius'))
                    samples=np.vstack([np.eye(3),-np.eye(3)])*radius
                    samples=samples@matrix[:3,:3].T+matrix[:3,3]
                    distances=np.abs(np.linalg.norm(local,axis=1)-radius)
                    tolerance=.015
                else:raise AssertionError(f'Unsupported static shape: {shape.tag}')
                assert tree.query(samples)[0].max()<tolerance,(world.stem,collision.get('name'),'missing solid')
                residual=np.minimum(residual,distances)
    assert residual.max()<3e-5,(world.stem,'export vertices outside static collision surfaces',float(residual.max()))


def validate_motion(world, record, triangles):
    actors={m.get('name'):m for m in ET.parse(world).getroot().findall('world/model')if m.find('plugin[@filename="libmultimap_obstacle_motion.so"]')is not None}
    descriptors=record.get('dynamic_obstacles',[])
    assert set(actors)=={d['name']for d in descriptors},world.stem
    for desc in descriptors:
        model=actors[desc['name']];plugin=model.find('plugin')
        assert abs(float(plugin.findtext('speed'))-desc['speed'])<1e-8
        points=np.array([np.fromstring(p.text,sep=' ')for p in plugin.findall('waypoints/point')])
        assert np.allclose(points,desc['waypoints'],atol=1e-7,rtol=0)
        assert np.linalg.norm(np.diff(points,axis=0),axis=1).sum()>2.9
        for field in ['ambient','diffuse']:
            color=np.fromstring(model.findtext(f'link/visual/material/{field}'),sep=' ')
            assert color[0]==1 and max(color[1:3])<.03
        for point in points:
            ground=support_height(triangles,[*point[:2],point[2]-.30,0])
            assert abs(point[2]-.4-ground)<.015,(world.stem,'dynamic ground support',point,ground)
    return len(descriptors)


def validate(package):
    metadata = json.loads((package / "worlds/catalog.json").read_text())
    records = metadata["maps"]
    expected = {f"{category}_{i}" for category,count in [('flat',5),('terrain',5),('floor',5)] for i in range(1,count+1)}
    expected |= {f"M{i}" for i in range(1,9)}
    assert set(records) == expected
    # Standalone robot test worlds (worlds/clearing_robot) are validated by
    # their own tests; the navigation catalog covers these three families.
    suite_worlds = {p.stem for category in ("flat", "terrain", "floor", "unseen")
                    for p in (package / "worlds" / category).rglob("*.world")}
    assert suite_worlds == expected
    assert {p.stem for p in (package / "pcd").rglob("*.pcd")} == expected
    mesh_hashes = set(); results = []
    surveyed=metadata['tracked_initial_poses']
    for name, record in records.items():
        for field in (("world", "pcd") if record.get("cohort")=="unseen" else ("world", "mesh", "pcd")):
            assert (package / record[field]).is_file(), (name, field)
        world = package / record["world"]
        for uri in ET.parse(world).getroot().findall(".//geometry/mesh/uri"):
            assert resolve_uri(uri.text, package, world.parent).is_file(), (name, uri.text)
        if record.get('cohort') == 'unseen':
            points = read_pcd(package / record['pcd'])
            assert len(points) == record['points'], name
            assert hashlib.sha256((package / record['pcd']).read_bytes()).hexdigest() == record['pcd_sha256'], name
            assert hashlib.sha256(world.read_bytes()).hexdigest() == record['world_sha256'], name
            exported = world_triangles(world, package)
            support = support_height(exported, record['spawn'])
            assert abs(record['initial_pose'][2] - support - .03) < .015, (name, support)
            results.append({'name':name,'points':len(points),'support_z':{'lower':support},'cohort':'unseen'})
            print(f"PASS {name}: {len(points)} points; support {support}", flush=True)
            continue
        triangles = read_binary_stl(package / record["mesh"])
        exported = world_triangles(package / record["world"], package)
        validate_static_geometry(package,world,triangles)
        assert len(triangles)==record['triangles'],name
        assert surveyed[name]['world_sha256']==hashlib.sha256(world.read_bytes()).hexdigest(),name
        digest = hashlib.sha256(triangles.tobytes()).hexdigest()
        assert digest not in mesh_hashes, f"Duplicate collision geometry: {name}"
        mesh_hashes.add(digest)
        points = read_pcd(package / record["pcd"])
        assert len(points) == record["points"]
        assert record['pcd_sha256']==hashlib.sha256((package/record['pcd']).read_bytes()).hexdigest(),name
        minimum, maximum = triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))
        assert (points >= minimum - 1e-4).all() and (points <= maximum + 1e-4).all(), name
        assert np.allclose(points.min(axis=0), record["bounds_min"], atol=1e-5)
        assert np.allclose(points.max(axis=0), record["bounds_max"], atol=1e-5)
        supports = {"lower": support_height(exported, record["spawn"])}
        if "clearing_spawn" in record:
            supports["clearing"] = support_height(triangles, record["clearing_spawn"])
        if "upper_spawn" in record:
            supports["upper"] = support_height(exported, record["upper_spawn"])
            assert supports["upper"] - supports["lower"] >= .9, name
        if "ramp_start" in record:
            supports["ramp"] = support_height(exported, record["ramp_start"])
        for level,pose_key in [('lower','spawn'),('upper','upper_spawn')]:
            if pose_key not in record:continue
            assert np.allclose([surveyed[name][level]['x'],surveyed[name][level]['y']],record[pose_key][:2],atol=1e-8)
            assert abs(surveyed[name][level]['base_z']-supports[level]-.03)<.015,(name,level,'survey support')
        dynamic_count=validate_motion(world,record,exported)
        results.append({"name": name, "points": len(points), "triangles": len(triangles), "support_z": supports,'dynamic_obstacles':dynamic_count})
        print(f"PASS {name}: {len(points)} points; support {supports}", flush=True)
    combos = {tuple(record["floor_types"]) for record in records.values() if record["category"] == "floor"}
    assert combos == {('flat','flat')}, 'Development floor maps 1–5 have two planar levels'
    assert {r['name']:r['dynamic_obstacles']for r in results if r.get('dynamic_obstacles')}=={'flat_3':8,'terrain_3':6,'floor_3':8}
    return {"passed": len(results), "maps": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = validate(args.package_dir.resolve())
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
