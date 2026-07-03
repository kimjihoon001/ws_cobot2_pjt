import math

def generate_cylinder_triangles(radius, height, z_offset, num_segments=16):
    """
    Generates vertices and triangles for a cylinder centered along the Z-axis.
    Returns a list of triangles, where each triangle is a list of 3 vertices (each vertex is a tuple of x, y, z).
    """
    vertices_bottom = []
    vertices_top = []
    
    # Generate circle vertices
    for i in range(num_segments):
        angle = 2.0 * math.pi * i / num_segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        vertices_bottom.append((x, y, z_offset))
        vertices_top.append((x, y, z_offset + height))
        
    triangles = []
    
    # Bottom cap (fan configuration around first vertex)
    for i in range(1, num_segments - 1):
        triangles.append([vertices_bottom[0], vertices_bottom[i+1], vertices_bottom[i]])
        
    # Top cap
    for i in range(1, num_segments - 1):
        triangles.append([vertices_top[0], vertices_top[i], vertices_top[i+1]])
        
    # Side walls (quads split into 2 triangles)
    for i in range(num_segments):
        next_i = (i + 1) % num_segments
        b1, b2 = vertices_bottom[i], vertices_bottom[next_i]
        t1, t2 = vertices_top[i], vertices_top[next_i]
        
        # Triangle 1
        triangles.append([b1, b2, t2])
        # Triangle 2
        triangles.append([b1, t2, t1])
        
    return triangles

def write_ascii_stl(filename, name, triangles):
    """Writes triangles to an ASCII STL file."""
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(f"solid {name}\n")
        for tri in triangles:
            # Simple facet normal estimation (or dummy 0 0 0)
            f.write("  facet normal 0.0 0.0 0.0\n")
            f.write("    outer loop\n")
            for vertex in tri:
                f.write(f"      vertex {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {name}\n")

def main():
    # Parametric screwdriver dimensions (in meters for ROS/MoveIt consistency)
    # Handle: Radius 1.5cm, length 8.0cm
    handle_triangles = generate_cylinder_triangles(radius=0.015, height=0.08, z_offset=0.0)
    
    # Shaft/Blade: Radius 0.4cm, length 10.0cm
    shaft_triangles = generate_cylinder_triangles(radius=0.004, height=0.10, z_offset=0.08)
    
    all_triangles = handle_triangles + shaft_triangles
    
    output_path = "/home/rokey/ws_cobot2_pjt/src/cobot2_ws/m0609_rg2_bringup/meshes/screwdriver.stl"
    write_ascii_stl(output_path, "screwdriver", all_triangles)
    print(f"Parametric screwdriver STL successfully written to: {output_path}")

if __name__ == "__main__":
    main()
