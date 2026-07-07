import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
from ament_index_python.packages import get_package_share_directory
import trimesh
import time

class SceneBuilder(Node):
    def __init__(self):
        super().__init__('scene_builder')
        self.pub = self.create_publisher(
            PlanningScene,'/planning_scene',10
        )

    def add_table(self):
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'  
        obj.id = 'table'                   

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.0, 1.0, 0.05]

        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = -0.3
        pose.orientation.w = 1.0

        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True
            
        self.pub.publish(scene)
        self.get_logger().info('테이블추가')

    def add_body_mesh(self):
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'body'

        mesh_path = get_package_share_directory('m0609_rg2_bringup') + '/meshes/이름없음-몸통.stl'
        tm = trimesh.load(mesh_path, force='mesh')

        mm_to_m = 0.001
        mesh = Mesh()
        for v in tm.vertices:
            mesh.vertices.append(Point(x=float(v[0]) * mm_to_m, y=float(v[1]) * mm_to_m, z=float(v[2]) * mm_to_m))
        for f in tm.faces:
            mesh.triangles.append(MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]))

        pose = Pose()
        pose.position.x = 0.5
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0

        obj.meshes = [mesh]
        obj.mesh_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True

        self.pub.publish(scene)
        self.get_logger().info('몸통 메시 장애물 추가')

    def add_ceiling(self):
        obj = CollisionObject()
        obj.header.frame_id = 'base_link'
        obj.id = 'ceiling'

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.05]  # 2m x 2m wide, 5cm thick

        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.6  # 600mm above robot base (safety ceiling)
        pose.orientation.w = 1.0

        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        scene.is_diff = True

        self.pub.publish(scene)
        self.get_logger().info('가상 천장 장애물 추가')

def main():
    rclpy.init()
    node = SceneBuilder()
    node.add_table()
    node.add_body_mesh()
    node.add_ceiling()
    time.sleep(0.5)
    rclpy.shutdown()

if __name__ == '__main__':
    main()