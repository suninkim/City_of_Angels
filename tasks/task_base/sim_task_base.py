import omni.kit.commands
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.kit import SimulationApp
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics


class SimTaskBase():
    def __init__(self, config):
        self.kit = SimulationApp({"renderer": "RayTracedLighting", "headless": config["headless"]})

        # Get stage handle
        self.stage = omni.usd.get_context().get_stage()

        # Enable physics
        self.scene = UsdPhysics.Scene.Define(self.stage, Sdf.Path("/physicsScene"))
    def make_env(self):
        # Set gravity
        self.scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        self.scene.CreateGravityMagnitudeAttr().Set(9.81)

        # Set solver settings
        PhysxSchema.PhysxSceneAPI.Apply(self.stage.GetPrimAtPath("/physicsScene"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(self.stage, "/physicsScene")
        physxSceneAPI.CreateEnableCCDAttr(True)
        physxSceneAPI.CreateEnableStabilizationAttr(True)
        physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
        physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
        physxSceneAPI.CreateSolverTypeAttr("TGS")

    def load_robot(self):
        # Setting up import configuration:
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = True
        import_config.import_inertia_tensor = True
        import_config.fix_base = True
        import_config.create_physics_scene = True
        import_config.distance_scale = 100
        
        
        status, stage_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        # urdf_path=extension_path + "/data/urdf/robots/carter/urdf/carter.urdf",
        urdf_path="/home/sunin/workspace/Sunin/Elephant_robotics/mycobot_ros2/mycobot_description/urdf/320_pi/mycobot_pro_320_pi.urdf",
        import_config=import_config,
        )

    def add_ground(self):
        # Add ground plane
        omni.kit.commands.execute(
            "AddGroundPlaneCommand",
            stage=self.stage,
            planePath="/groundPlane",
            axis="Z",
            size=1500.0,
            position=Gf.Vec3f(0, 0, 0),
            color=Gf.Vec3f(0.7),
        )

    def lighting(self):
        # Add lighting
        distantLight = UsdLux.DistantLight.Define(self.stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)



    def load_objects(self, object_list):
        a=1

    def attach_camera(self):
        a=1

    def make_env(self):
        a=1

    def make_env(self):
        a=1

    def make_env(self):
        a=1

    def make_env(self):
        a=1

    def make_env(self):
        a=1

    def make_env(self):
        a=1

    def make_env(self):
        a=1

    def make_env(self):
        a=1

    def make_env(self):
        a=1
