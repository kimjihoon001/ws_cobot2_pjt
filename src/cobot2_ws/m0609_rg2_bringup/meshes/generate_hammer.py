import math


def generate_cylinder_triangles(radius, height, z_offset, num_segments=12):
    """
    Generates vertices and triangles for a cylinder centered along the Z-axis.
    Returns a list of triangles, where each triangle is a list of 3 vertices (each vertex is a tuple of x, y, z).
    """
    vertices_bottom = []
    vertices_top = []

    for i in range(num_segments):
        angle = 2.0 * math.pi * i / num_segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        vertices_bottom.append((x, y, z_offset))
        vertices_top.append((x, y, z_offset + height))

    triangles = []

    for i in range(1, num_segments - 1):
        triangles.append([vertices_bottom[0], vertices_bottom[i + 1], vertices_bottom[i]])

    for i in range(1, num_segments - 1):
        triangles.append([vertices_top[0], vertices_top[i], vertices_top[i + 1]])

    for i in range(num_segments):
        next_i = (i + 1) % num_segments
        b1, b2 = vertices_bottom[i], vertices_bottom[next_i]
        t1, t2 = vertices_top[i], vertices_top[next_i]
        triangles.append([b1, b2, t2])
        triangles.append([b1, t2, t1])

    return triangles


def rotate_z_axis_to_x_axis(triangles, translate=(0.0, 0.0, 0.0)):
    """Z축 기준으로 생성된 원기둥을 X축 방향으로 눕히고 이동시킨다 (Ry 90도: (x,y,z)->(z,y,-x))."""
    tx, ty, tz = translate
    out = []
    for tri in triangles:
        out.append([(z + tx, y + ty, -x + tz) for (x, y, z) in tri])
    return out


def write_ascii_stl(filename, name, triangles):
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(f"solid {name}\n")
        for tri in triangles:
            f.write("  facet normal 0.0 0.0 0.0\n")
            f.write("    outer loop\n")
            for vertex in tri:
                f.write(f"      vertex {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {name}\n")


def main():
    # 손잡이: 반지름 1.5cm, 길이 25cm, Z축 방향. 원점을 손잡이 중앙(길이 방향)에 두어
    # 그리퍼 부착 시 별도 위치 보정 없이 rg2_tcp 원점에 그대로 맞출 수 있게 함.
    handle_height = 0.25
    handle_triangles = generate_cylinder_triangles(
        radius=0.015, height=handle_height, z_offset=-handle_height / 2.0)

    # 헤드: 반지름 2cm, 길이 12cm, 손잡이 끝(z=+handle_height/2)에서 X축 방향으로 가로질러 배치 (T자 모양)
    head_length = 0.12
    head_radius = 0.02
    head_triangles_z = generate_cylinder_triangles(
        radius=head_radius, height=head_length, z_offset=-head_length / 2.0)
    head_triangles = rotate_z_axis_to_x_axis(head_triangles_z, translate=(0.0, 0.0, handle_height / 2.0))

    all_triangles = handle_triangles + head_triangles

    output_path = '/home/jihoon/ws_cobot2_pjt/ws_cobot2_pjt/src/cobot2_ws/m0609_rg2_bringup/meshes/hammer.stl'
    write_ascii_stl(output_path, 'hammer', all_triangles)
    print(f'Parametric hammer STL successfully written to: {output_path}')
    print(f'Triangle count: {len(all_triangles)}')


if __name__ == '__main__':
    main()
