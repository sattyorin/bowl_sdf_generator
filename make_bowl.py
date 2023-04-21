#!/usr/bin/env python3

import argparse
import os

import numpy as np
from stl import mesh


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument("--model_name", default="bowl", type=str, help="name of model")
    parser.add_argument(
        "-n", "--num_division", default=16, type=int, help="number of division"
    )
    parser.add_argument(
        "--radius_bottom", type=float, required=True, help="radius on bottom"
    )
    parser.add_argument(
        "--radius_top", type=float, required=True, help="radius on bottom"
    )
    parser.add_argument(
        "--height", type=float, required=True, help="height of the bowl"
    )
    parser.add_argument(
        "--thickness", type=float, required=True, help="thickness of the bowl"
    )
    parser.add_argument(
        "--out_mujoco_dir",
        default=".",
        required=True,
        help="output mujoco directory",
    )
    parser.add_argument(
        "--out_gazebo_dir",
        default=".",
        required=True,
        help="output gazebo directory",
    )

    return parser.parse_args()


class BowlGenerator:
    def __init__(
        self,
        *,
        model_name: str,
        num_division: int,
        radius_bottom: float,
        radius_top: float,
        height: float,
        thickness: float,
        out_mujoco_dir: str,
        out_gazebo_dir: str,
    ):
        self.model_name = model_name
        self.N = num_division
        self.radius_bottom = radius_bottom
        self.radius_top = radius_top
        self.height = height
        self.thickness = thickness
        self.out_mujoco_dir = out_mujoco_dir
        self.out_gazebo_dir = out_gazebo_dir

        os.makedirs(os.path.join(self.out_mujoco_dir, "xmls"), exist_ok=True)
        os.makedirs(os.path.join(self.out_mujoco_dir, "xmls", "tools"), exist_ok=True)
        os.makedirs(os.path.join(self.out_mujoco_dir, "meshes"), exist_ok=True)
        os.makedirs(self.out_gazebo_dir, exist_ok=True)
        os.makedirs(
            os.path.join(self.out_gazebo_dir, f"{self.model_name}", "meshes"),
            exist_ok=True,
        )

    def gen_bottom_plate_stl(self):
        theta = np.hstack([np.arange(0, 2.0 * np.pi, 2.0 * np.pi / self.N), 0.0])

        # center = np.zeros(2)
        circle_points = np.vstack(
            [
                self.radius_bottom * np.cos(theta),
                self.radius_bottom * np.sin(theta),
            ]
        ).transpose()

        vertices = np.vstack(
            [
                np.zeros(3),
                np.array([0.0, 0.0, self.thickness]),
                np.hstack([circle_points, np.zeros((self.N + 1, 1))]),
                np.hstack([circle_points, self.thickness * np.ones((self.N + 1, 1))]),
            ]
        )

        bottom_faces = np.vstack(
            [
                np.zeros(self.N, dtype=int),
                np.arange(3, self.N + 3, dtype=int),
                np.arange(2, self.N + 2, dtype=int),
            ]
        ).transpose()

        top_faces = np.vstack(
            [
                1 * np.ones(self.N, dtype=int),
                np.arange(3 + self.N, 2 * self.N + 3, dtype=int),
                np.arange(4 + self.N, 2 * self.N + 4, dtype=int),
            ]
        ).transpose()

        side_faces_1 = np.vstack(
            [
                np.arange(2, self.N + 2, dtype=int),
                np.arange(3, self.N + 3, dtype=int),
                np.arange(4 + self.N, 2 * self.N + 4, dtype=int),
            ]
        ).transpose()

        side_faces_2 = np.vstack(
            [
                np.arange(4 + self.N, 2 * self.N + 4, dtype=int),
                np.arange(3 + self.N, 2 * self.N + 3, dtype=int),
                np.arange(2, self.N + 2, dtype=int),
            ]
        ).transpose()

        faces = np.vstack(
            [
                bottom_faces,
                top_faces,
                side_faces_1,
                side_faces_2,
            ]
        )

        model_mesh = mesh.Mesh(np.zeros(4 * self.N, dtype=mesh.Mesh.dtype))
        model_mesh.remove_duplicate_polygons = True

        for i, f in enumerate(faces):
            for j in range(3):
                model_mesh.vectors[i][j] = vertices[f[j], :]

        model_mesh.save(
            os.path.join(self.out_mujoco_dir, "meshes", f"{self.model_name}_bottom.stl")
        )
        model_mesh.save(
            os.path.join(
                self.out_gazebo_dir,
                f"{self.model_name}",
                "meshes",
                f"{self.model_name}_bottom.stl",
            )
        )

    def gen_side_fragment_stl(self):
        theta_width = 2.0 * np.pi / self.N

        r = np.array(
            [
                self.radius_bottom,
                self.radius_bottom + self.thickness,
                self.radius_top,
                self.radius_top + self.thickness,
            ]
        )
        z = self.thickness + np.array([0.0, 0.0, self.height, self.height])

        vertices = np.vstack(
            [
                np.vstack([r, np.zeros(4), z]).transpose(),
                np.vstack(
                    [
                        r * np.cos(theta_width),
                        r * np.sin(theta_width),
                        z,
                    ]
                ).transpose(),
            ]
        )

        faces = np.array(
            [
                # side
                [0, 1, 2],
                [1, 3, 2],
                [4, 6, 5],
                [5, 6, 7],
                # inside
                [0, 2, 4],
                [2, 6, 4],
                # outside
                [1, 5, 3],
                [3, 5, 7],
                # bottom
                [0, 4, 1],
                [1, 4, 5],
                # top
                [2, 3, 7],
                [2, 7, 6],
            ]
        )

        model_mesh = mesh.Mesh(np.zeros(4 * self.N, dtype=mesh.Mesh.dtype))
        model_mesh.remove_duplicate_polygons = True

        for i, f in enumerate(faces):
            for j in range(3):
                model_mesh.vectors[i][j] = vertices[f[j], :]

        model_mesh.save(
            os.path.join(
                self.out_mujoco_dir, "meshes", f"{self.model_name}_fragment.stl"
            )
        )
        model_mesh.save(
            os.path.join(
                self.out_gazebo_dir,
                f"{self.model_name}",
                "meshes",
                f"{self.model_name}_fragment.stl",
            )
        )

    def gen_model_config(self):
        CONTENTS = f"""<?xml version = "1.0"?>
<model>
  <name>{self.model_name}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
</model>
"""
        with open(
            os.path.join(self.out_gazebo_dir, f"{self.model_name}", "model.config"),
            mode="w",
        ) as f:
            f.write(CONTENTS)

    def gen_model_sdf(self):
        fragments = [
            "\n".join(
                [
                    f'    <link name="side_fragment_{i}">',
                    "      <inertial>",
                    "        <mass>0.01</mass>",
                    "      </inertial>",
                    '      <collision name="collision">',
                    f"       <pose>0 0 0 0 0 {theta}</pose>",
                    "        <geometry>",
                    "          <mesh>",
                    f"            <uri>model://{self.model_name}/meshes/{self.model_name}_fragment.stl</uri>",
                    "          </mesh>",
                    "        </geometry>",
                    "      </collision>",
                    '      <visual name="visual">',
                    f"       <pose>0 0 0 0 0 {theta}</pose>",
                    "        <geometry>",
                    "          <mesh>",
                    f"            <uri>model://{self.model_name}/meshes/{self.model_name}_fragment.stl</uri>",
                    "          </mesh>",
                    "        </geometry>",
                    "      </visual>",
                    "    </link>",
                ]
            )
            for i, theta in enumerate(np.arange(0.0, 2.0 * np.pi, 2.0 * np.pi / self.N))
        ]

        lines = [
            '<?xml version="1.0" ?>',
            '<sdf version="1.7">',
            f'  <model name="{self.model_name}">',
            "    <pose>0 0 0 0 0 0</pose>",
            "    <static>true</static>",
            '    <link name="bottom_plate">',
            "      <inertial>",
            "        <mass>0.01</mass>",
            "      </inertial>",
            '      <collision name="collision">',
            "        <geometry>",
            "          <mesh>",
            f"            <uri>model://{self.model_name}/meshes/{self.model_name}_bottom.stl</uri>",
            "          </mesh>",
            "        </geometry>",
            "      </collision>",
            '      <visual name="visual">',
            "        <geometry>",
            "          <mesh>",
            f"            <uri>model://{self.model_name}/meshes/{self.model_name}_bottom.stl</uri>",
            "          </mesh>",
            "        </geometry>",
            "      </visual>",
            "    </link>",
            *fragments,
            "  </model>",
            "</sdf>",
        ]

        with open(
            os.path.join(self.out_gazebo_dir, f"{self.model_name}", "model.sdf"),
            mode="w",
        ) as f:
            f.write("\n".join(lines) + "\n")

    def gen_model_mujoco_xml(self):
        fragments = [
            "\n".join(
                [
                    f'    <geom name="geom_bowl_fragment{i}" type="mesh" mesh="{self.model_name}_fragment" pos="0 0 0" euler="0 0 {np.rad2deg(theta)}" />',
                ]
            )
            for i, theta in enumerate(np.arange(0.0, 2.0 * np.pi, 2.0 * np.pi / self.N))
        ]

        lines = [
            '<?xml version="1.0"?>',
            "<mujoco>",
            f'  <body name="bowl" pos="0 0 -{self.thickness}">',
            f'    <!-- <geom name="geom_bowl_bottom0" type="mesh" mesh="{self.model_name}_bottom" pos="0 0 0" euler="0 0 0" /> -->',
            f'    <!-- <geom name="geom_bowl_bottom0" type="ellipsoid" size="{self.radius_bottom} {self.radius_bottom} 0.01" pos="0 0 {self.thickness}" /> -->',
            *fragments,
            "  </body>",
            "</mujoco>",
        ]

        with open(
            os.path.join(self.out_mujoco_dir, "xmls", "tools", "bowl.xml"),
            mode="w",
        ) as f:
            f.write("\n".join(lines) + "\n")


def main():
    args = parse_args()

    generator = BowlGenerator(
        model_name=args.model_name,
        num_division=args.num_division,
        radius_bottom=args.radius_bottom,
        radius_top=args.radius_top,
        height=args.height,
        thickness=args.thickness,
        out_mujoco_dir=args.out_mujoco_dir,
        out_gazebo_dir=args.out_gazebo_dir,
    )

    generator.gen_side_fragment_stl()
    generator.gen_bottom_plate_stl()
    generator.gen_model_config()
    generator.gen_model_sdf()
    generator.gen_model_mujoco_xml()


if __name__ == "__main__":
    main()
