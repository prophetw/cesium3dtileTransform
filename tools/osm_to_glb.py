#!/usr/bin/env python3
import math
import struct
import json
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import osmium
except ImportError:
    osmium = None

# Constants
LEVEL_HEIGHT = 3.0  # Meters per level
EARTH_RADIUS = 6378137.0

def latlon_to_meters(lat, lon, origin_lat, origin_lon):
    """Simple Spherical Mercator projection relative to origin."""
    rad_lat = math.radians(origin_lat)
    x = math.radians(lon - origin_lon) * EARTH_RADIUS * math.cos(rad_lat)
    y = math.radians(lat - origin_lat) * EARTH_RADIUS
    # Return (East, North)
    return x, y

def normalize_vec3(v):
    l = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if l == 0: return (0,0,0)
    return (v[0]/l, v[1]/l, v[2]/l)

# ... (Math functions unchanged: cross_product, sub_vec3, is_point_in_triangle, triangulate_polygon) ...

def cross_product(a, b):
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    )

def sub_vec3(a, b):
    return (a[0]-b[0], a[1]-b[1], a[2]-b[2])

def is_point_in_triangle(p, a, b, c):
    # Barycentric technique
    v0 = sub_vec3(c, a)
    v1 = sub_vec3(b, a)
    v2 = sub_vec3(p, a)
    
    dot00 = v0[0]*v0[0] + v0[1]*v0[1] # ignore z
    dot01 = v0[0]*v1[0] + v0[1]*v1[1]
    dot02 = v0[0]*v2[0] + v0[1]*v2[1]
    dot11 = v1[0]*v1[0] + v1[1]*v1[1]
    dot12 = v1[0]*v2[0] + v1[1]*v2[1]

    denom = (dot00 * dot11 - dot01 * dot01)
    if denom == 0: return False
    invDenom = 1 / denom
    u = (dot11 * dot02 - dot01 * dot12) * invDenom
    v = (dot00 * dot12 - dot01 * dot02) * invDenom
    return (u >= 0) and (v >= 0) and (u + v < 1)

def triangulate_polygon(polygon_points):
    """
    Ear Clipping Algorithm.
    polygon_points: list of (x, z) or (x, y) tuples. Assumes simple polygon (no holes).
    Returns list of indices forming triangles.
    """
    n = len(polygon_points)
    if n < 3: return []
    
    indices = list(range(n))
    output_indices = []
    
    max_iter = n * n
    count = 0
    
    while len(indices) > 2:
        if count > max_iter:
            # print("Triangulation stuck")
            break
        count += 1
        
        ear_found = False
        for i in range(len(indices)):
            prev_idx = indices[(i - 1)]
            curr_idx = indices[i]
            next_idx = indices[(i + 1) % len(indices)]
            
            p_prev = polygon_points[prev_idx]
            p_curr = polygon_points[curr_idx]
            p_next = polygon_points[next_idx]
            
            # 1. Check if convex
            cp = (p_curr[0] - p_prev[0]) * (p_next[1] - p_prev[1]) - (p_curr[1] - p_prev[1]) * (p_next[0] - p_prev[0])
            
            if cp <= 0: # Concave or Collinear
                continue
                
            # 2. Check if any other point is inside triangle
            is_ear = True
            for k in indices:
                if k in (prev_idx, curr_idx, next_idx):
                    continue
                pk = polygon_points[k]
                
                # Barycentric point in triangle check
                ax, ay = p_prev
                bx, by = p_curr
                cx, cy = p_next
                px, py = pk
                
                v0x = cx - ax; v0y = cy - ay
                v1x = bx - ax; v1y = by - ay
                v2x = px - ax; v2y = py - ay

                dot00 = v0x*v0x + v0y*v0y
                dot01 = v0x*v1x + v0y*v1y
                dot02 = v0x*v2x + v0y*v2y
                dot11 = v1x*v1x + v1y*v1y
                dot12 = v1x*v2x + v1y*v2y

                invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01) if (dot00 * dot11 - dot01 * dot01) != 0 else 0
                u = (dot11 * dot02 - dot01 * dot12) * invDenom
                v = (dot00 * dot12 - dot01 * dot02) * invDenom

                if (u >= 0) and (v >= 0) and (u + v < 1):
                    is_ear = False
                    break
            
            if is_ear:
                output_indices.extend([prev_idx, curr_idx, next_idx])
                indices.pop(i)
                ear_found = True
                break
        
        if not ear_found:
            break
            
    return output_indices

# --- Parsing Logic ---

def parse_osm_xml(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    
    nodes_map = {} # id -> (lat, lon)
    buildings = [] # list of {tags, coords}
    
    min_lat, min_lon, max_lat, max_lon = 90, 180, -90, -180
    
    for child in root:
        if child.tag == 'node':
            lat = float(child.attrib['lat'])
            lon = float(child.attrib['lon'])
            nodes_map[child.attrib['id']] = (lat, lon)
            min_lat = min(min_lat, lat)
            min_lon = min(min_lon, lon)
            max_lat = max(max_lat, lat)
            max_lon = max(max_lon, lon)
            
        elif child.tag == 'way':
            tags = {}
            node_refs = []
            for sub in child:
                if sub.tag == 'nd':
                    node_refs.append(sub.attrib['ref'])
                elif sub.tag == 'tag':
                    tags[sub.attrib['k']] = sub.attrib['v']
            
            if 'building' in tags:
                coords = []
                valid = True
                for ref in node_refs:
                    if ref in nodes_map:
                        coords.append(nodes_map[ref])
                    else:
                        valid = False; break
                if valid and len(coords) >= 3:
                    buildings.append({'tags': tags, 'coords': coords})
                
    origin_lat = (min_lat + max_lat) / 2
    origin_lon = (min_lon + max_lon) / 2
    
    return buildings, origin_lat, origin_lon

class OsmiumBuildingHandler(osmium.SimpleHandler if osmium else object):
    def __init__(self):
        super().__init__()
        self.buildings = []
        self.bounds = [90, 180, -90, -180]

    def way(self, w):
        if 'building' not in w.tags:
            return
        
        try:
            coords = []
            for n in w.nodes:
                lat, lon = n.location.lat, n.location.lon
                coords.append((lat, lon))
                self.bounds[0] = min(self.bounds[0], lat)
                self.bounds[1] = min(self.bounds[1], lon)
                self.bounds[2] = max(self.bounds[2], lat)
                self.bounds[3] = max(self.bounds[3], lon)
            
            if len(coords) >= 3:
                # Copy tags to dict
                tags = {t.k: t.v for t in w.tags}
                self.buildings.append({'tags': tags, 'coords': coords})
        except osmium.InvalidLocationError:
            pass 

def parse_osm_pbf(file_path):
    if not osmium:
        raise ImportError("osmium library not found. Please install: pip install osmium")
        
    handler = OsmiumBuildingHandler()
    handler.apply_file(str(file_path), locations=True)
    
    min_lat, min_lon, max_lat, max_lon = handler.bounds
    if min_lat > max_lat:
        return [], 0, 0
        
    origin_lat = (min_lat + max_lat) / 2
    origin_lon = (min_lon + max_lon) / 2
    
    return handler.buildings, origin_lat, origin_lon

def build_geometry(buildings, origin_lat, origin_lon):
    # Output arrays
    positions = [] # x, y, z
    normals = []
    indices = []
    
    vertex_count = 0
    
    for b in buildings:
        coords = b['coords']
        tags = b['tags']
        
        # Determine Height
        height = 10.0 # default
        if 'height' in tags:
            try:
                height = float(tags['height'].replace('m',''))
            except: pass
        elif 'building:levels' in tags:
            try:
                height = float(tags['building:levels']) * LEVEL_HEIGHT
            except: pass
            
        # Get Footprint vertices in local meters
        footprint = [] # (x, z)
        
        # Remove duplicate last point if closed
        # Coords are (lat, lon)
        if len(coords) > 1 and coords[0] == coords[-1]:
            proc_coords = coords[:-1]
        else:
            proc_coords = coords
            
        if len(proc_coords) < 3: continue

        for lat, lon in proc_coords:
            e, n = latlon_to_meters(lat, lon, origin_lat, origin_lon)
            # Adapt to Standard GLTF Y-up (Y=Up, Z=Front/South)
            # We want final 3D Tiles (ECEF/ENU) to be (East, North, Up)
            # The tool applies x-axis rotation (usually -90) to map Y->Z and Z->-Y.
            # To get Z_out = Up (Height), we need Y_in = Height.
            # To get Y_out = North, we need -Z_in = North => Z_in = -North.
            footprint.append((e, -n))
            
        # Ensure CCW winding for Ear Clipping (and correct face culling)
        # Calculate signed area using trapezoidal formula: sum((x2-x1)*(y2+y1))
        # In footprint coords (fx, fz):
        # - Negative area = CCW winding (correct for triangulation)
        # - Positive area = CW winding (needs reversal)
        # The ear clipping algorithm requires CCW input.
        area = 0.0
        for i in range(len(footprint)):
            p1 = footprint[i]
            p2 = footprint[(i + 1) % len(footprint)]
            area += (p2[0] - p1[0]) * (p2[1] + p1[1])
        
        # Reverse if winding is CW (positive area)
        if area > 0:
            footprint.reverse()

        # Triangulate Roof
        roof_tris = triangulate_polygon(footprint)
        if not roof_tris: continue
        
        # Base Index for this building
        base_idx = vertex_count
        
        # Roof is at +Height
        roof_y = height
        
        for fx, fz in footprint:
            positions.append((fx, roof_y, fz))
            # Normal Up (0, 1, 0)
            normals.append((0, 1, 0))
        
        # Add Roof Indices
        for idx in roof_tris:
            indices.append(base_idx + idx)
            
        vertex_count += len(footprint)
        
        # Add Walls
        n_points = len(footprint)
        for i in range(n_points):
            j = (i + 1) % n_points
            
            p1 = footprint[i] # (x, z)
            p2 = footprint[j]
            
            # Wall Vector (p2 - p1)
            dx = p2[0] - p1[0]
            dz = p2[1] - p1[1]
            length = math.sqrt(dx*dx + dz*dz)
            if length == 0: continue
            
            # Normal calculation (2D in X-Z plane)
            # Tangent (dx, dz). Normal (-dz, dx) or (dz, -dx).
            # We want Outward normal.
            # If winding is CCW, Right hand rule.
            # Normal should be to the Right? No, Left is in?
            # CCW: Wall vector is counter-clockwise. Outward is Right.
            # Vector (dx, dz). Right is (dz, -dx).
            nx = dz / length
            nz = -dx / length
            
            # 4 Vertices
            # Y goes 0 to +height.
            
            # 0: p1, Ground (0)
            positions.append((p1[0], 0, p1[1]))
            normals.append((nx, 0, nz))
            # 1: p2, Ground (0)
            positions.append((p2[0], 0, p2[1]))
            normals.append((nx, 0, nz))
            # 2: p2, Roof (+height)
            positions.append((p2[0], roof_y, p2[1]))
            normals.append((nx, 0, nz))
            # 3: p1, Roof (+height)
            positions.append((p1[0], roof_y, p1[1]))
            normals.append((nx, 0, nz))
            
            # 2 Triangles
            # 0, 1, 2
            # 0, 2, 3
            indices.append(vertex_count + 0)
            indices.append(vertex_count + 1)
            indices.append(vertex_count + 2)
            
            indices.append(vertex_count + 0)
            indices.append(vertex_count + 2)
            indices.append(vertex_count + 3)
            
            vertex_count += 4

    return positions, normals, indices

def export_glb(positions, normals, indices, filename):
    # Prepare Binary Data
    # 1. Padded Buffers
    # Positions: Vec3 Float32
    pos_bytes = bytearray()
    min_x, min_y, min_z = 1e9, 1e9, 1e9
    max_x, max_y, max_z = -1e9, -1e9, -1e9
    
    for p in positions:
        pos_bytes.extend(struct.pack('<fff', p[0], p[1], p[2]))
        min_x = min(min_x, p[0]); max_x = max(max_x, p[0])
        min_y = min(min_y, p[1]); max_y = max(max_y, p[1])
        min_z = min(min_z, p[2]); max_z = max(max_z, p[2])
        
    pos_len = len(pos_bytes)
    pos_padding = (4 - (pos_len % 4)) % 4
    pos_bytes.extend(b'\x00' * pos_padding)
    
    # Normals: Vec3 Float32
    norm_bytes = bytearray()
    for n in normals:
        norm_bytes.extend(struct.pack('<fff', n[0], n[1], n[2]))
    norm_len = len(norm_bytes)
    norm_padding = (4 - (norm_len % 4)) % 4
    norm_bytes.extend(b'\x00' * norm_padding)
    
    # Indices: Scalar Uint32
    idx_bytes = bytearray()
    for i in indices:
        idx_bytes.extend(struct.pack('<I', i))
    idx_len = len(idx_bytes)
    idx_padding = (4 - (idx_len % 4)) % 4
    idx_bytes.extend(b'\x00' * idx_padding)
    
    total_len = len(pos_bytes) + len(norm_bytes) + len(idx_bytes)
    
    # JSON Header
    header = {
        "asset": {"version": "2.0", "generator": "osm_to_glb.py"},
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "materials": [{
            "pbrMetallicRoughness": {
                "baseColorFactor": [0.7, 0.7, 0.7, 1.0],
                "metallicFactor": 0.0,
                "roughnessFactor": 0.8
            },
            "doubleSided": True
        }],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1}, "indices": 2, "material": 0, "mode": 4}]}],
        "buffers": [{"byteLength": total_len}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(pos_bytes), "target": 34962},
            {"buffer": 0, "byteOffset": len(pos_bytes), "byteLength": len(norm_bytes), "target": 34962},
            {"buffer": 0, "byteOffset": len(pos_bytes) + len(norm_bytes), "byteLength": len(idx_bytes), "target": 34963}
        ],
        "accessors": [
            {
                "bufferView": 0,
                "componentType": 5126, # FLOAT
                "count": len(positions),
                "type": "VEC3",
                "min": [min_x, min_y, min_z],
                "max": [max_x, max_y, max_z]
            },
            {
                "bufferView": 1,
                "componentType": 5126, # FLOAT
                "count": len(normals),
                "type": "VEC3",
            },
            {
                "bufferView": 2,
                "componentType": 5125, # UINT32
                "count": len(indices),
                "type": "SCALAR",
            }
        ]
    }
    
    json_bytes = json.dumps(header).encode('utf-8')
    json_padding = (4 - (len(json_bytes) % 4)) % 4
    json_bytes += b' ' * json_padding
    
    # Combine
    full_binary = pos_bytes + norm_bytes + idx_bytes
    
    with open(filename, 'wb') as f:
        # GLB Header
        f.write(b'glTF')
        f.write(struct.pack('<I', 2))
        file_len = 12 + 8 + len(json_bytes) + 8 + len(full_binary)
        f.write(struct.pack('<I', file_len))
        
        # Chunk 0: JSON
        f.write(struct.pack('<I', len(json_bytes)))
        f.write(b'JSON')
        f.write(json_bytes)
        
        # Chunk 1: BIN
        f.write(struct.pack('<I', len(full_binary)))
        f.write(b'BIN\x00')
        f.write(full_binary)
        
    print(f"Exported {filename}: {len(positions)} vertices, {len(indices)//3} triangles.")

def main():
    parser = argparse.ArgumentParser(description="Convert OSM XML/PBF to GLB (Y-up)")
    parser.add_argument("input", help="Input .osm or .pbf file")
    parser.add_argument("output", help="Output .glb file")
    args = parser.parse_args()
    
    input_path = Path(args.input)
    print(f"Parsing {args.input}...")
    
    if input_path.suffix == '.pbf':
        buildings, origin_lat, origin_lon = parse_osm_pbf(input_path)
    else:
        buildings, origin_lat, origin_lon = parse_osm_xml(input_path)
        
    print(f"Found {len(buildings)} building footprints.")
    print(f"Origin: {origin_lat}, {origin_lon}")
    
    print("Building geometry...")
    pos, norm, idx = build_geometry(buildings, origin_lat, origin_lon)
    
    if not pos:
        print("No geometry generated.")
        return
        
    print("Exporting GLB...")
    export_glb(pos, norm, idx, args.output)

if __name__ == "__main__":
    main()
