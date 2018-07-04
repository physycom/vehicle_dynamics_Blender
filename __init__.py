bl_info = {
    "name": "Inertial-GNSS car data to blender",
    "author": "Federico Bertani",
    "location": "View3D > Tools",
    "description": "Blender simulation generator from inertial sensor data on car",
    "category": "Object", # TODO other possible categories are Animation and Physics
    "version": (0,0,1),
    "tracker_url" : "https://github.com/physycom/inertial_to_blender/issues"
}

import bpy
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent))
from . import addon_updater_ops
import os
import urllib.request

from subprocess import call

from bpy.props import StringProperty

bpy.types.Scene.datasetPath = StringProperty(
    name="Dataset path",
    description=":",
    default="",
    maxlen=2056,
)


class InertialBlenderPanel(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "Tools"
    bl_label = "Inertial-GNSS to blender"

    # bl_context = "object"

    def draw(self, context):
        layout = self.layout

        col = layout.column(align=True)
        col.operator("physicom.load_dataset")
        col.prop(context.scene, "datasetPath")
        col.operator("physicom.animate_object")


class LoadDataset(bpy.types.Operator):
    """Object Cursor Array"""
    # TODO use more standard name
    bl_idname = "physicom.load_dataset"
    bl_label = "Load dataset"
    bl_options = {'REGISTER', 'UNDO'}

    # must be called filepath because fileselect_add() in invoke() assume this
    filepath = bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        context.scene.datasetPath = self.filepath
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class AnimateObject(bpy.types.Operator):
    """Object Cursor Array"""
    # TODO use more standard name
    bl_idname = "physicom.animate_object"
    bl_label = "Animate object"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        from src import get_trajectory_from_path
        scene = context.scene
        # get current frame per seconds value
        fps = scene.render.fps
        scene.unit_settings.system = 'METRIC'
        # get current selected object in scene
        obj = scene.objects.active
        # TODO check object is not None
        positions, times, angular_positions = get_trajectory_from_path(scene.datasetPath)
        # set animation lenght
        bpy.context.scene.frame_end = times[-1] * fps
        # create animation data
        obj.animation_data_clear()
        obj.animation_data_create()
        # create a new animation data action
        obj.animation_data.action = bpy.data.actions.new(name="MyAction")
        obj.rotation_mode = 'QUATERNION'
        positions_lenght = positions.shape[1]
        # create f-curve for each axis
        for index in range(3):
            fcurve_location = obj.animation_data.action.fcurves.new(data_path="location", index=index)
            fcurve_location.keyframe_points.add(positions_lenght)
            for i in range(0, positions_lenght):
                fcurve_location.keyframe_points[i].interpolation = 'CONSTANT'
                fcurve_location.keyframe_points[i].co = times[i] * fps, positions[index, i]
        # create f-curve for each quaternion component
        for index in range(4):
            fcurve_rotation = obj.animation_data.action.fcurves.new(data_path="rotation_quaternion", index=index)
            fcurve_rotation.keyframe_points.add(positions_lenght)
            for i in range(0, positions_lenght):
                fcurve_rotation.keyframe_points[i].interpolation = 'CONSTANT'
                fcurve_rotation.keyframe_points[i].co = times[i] * fps, angular_positions[index,i]
        print("Done adding keyframes!")
        curveData = bpy.data.curves.new('myCurve', type='CURVE')
        curveData.dimensions = '3D'
        curveData.resolution_u = 2

        # map coords to spline
        polyline = curveData.splines.new('POLY')
        polyline.points.add(positions_lenght)
        for i, location in enumerate(positions.T):
            polyline.points[i].co = (*location,1)
        curveOB = bpy.data.objects.new('myCurve', curveData)
        curveData.bevel_depth = 0.01
        # attach to scene and validate context
        scene.objects.link(curveOB)
        return {'FINISHED'}


def call_system_command(command):
    try:
        retcode = call(command, shell=True)
        if retcode < 0:
            print("Child was terminated by signal", -retcode, file=sys.stderr)
        else:
            print("Child returned", retcode, file=sys.stderr)
    except OSError as e:
        print("Execution failed:", e, file=sys.stderr)


def register():
    # register auto-update module
    # placed this on top so the plugin degenerate to a non working version
    # this can be fixed by a new relase
    # TODO uncomment on public repo
    #addon_updater_ops.register(bl_info)
    #bpy.utils.register_class(AutoUpdatePreferences)
    #bpy.utils.register_class(UpdaterPanel)

    # TODO handle permission errors
    addon_path = str(Path(__file__).parent)
    print("Addon path " + addon_path)
    blender_path = str(Path(sys.executable).parent)
    print("Blender path " + blender_path)
    # TODO detect version, make more flexible
    # _, dirs, _ = next(os.walk(blender_path))
    # blender_version_dir = dirs[0]
    blender_version_dir = "2.79"
    blender_python_dir = os.path.join(blender_path, blender_version_dir, "python")
    posix_pip_location = os.path.join(blender_python_dir, "bin", "pip")
    windows_pip_location = os.path.join(blender_python_dir, "scripts", "pip3.exe")
    if not (os.path.exists(posix_pip_location) or os.path.exists(windows_pip_location)):
        print("Downloading pip")
        # download get pip
        pip_download_location = os.path.join(addon_path, "get_pip.py")
        urllib.request.urlretrieve("https://bootstrap.pypa.io/get-pip.py",
                                   filename=pip_download_location)
        python_bin = os.path.join(blender_python_dir, "bin")
        if (os.path.exists(os.path.join(python_bin, "python3.5m"))):
            python_interpreter = os.path.join(python_bin, "python3.5m")
        elif os.path.exists(os.path.join(python_bin, "python.exe")):
            python_interpreter = os.path.join(python_bin, "python.exe")
        command = r'"{}" "{}"'.format(python_interpreter, pip_download_location)
        print("Command: " + command)
        call_system_command(command)
    try:
        import scipy, pyquaternion
    except ImportError:
        print("Installing packets")
        if (os.path.exists(posix_pip_location)):
            command = r'"{}" install -r "{}"'.format(posix_pip_location, os.path.join(addon_path, "requirements.txt"))
        elif os.path.exists(windows_pip_location):
            command = r'"{}" install -r "{}"'.format(windows_pip_location, os.path.join(addon_path, "requirements.txt"))
        call_system_command(command)
    # install
    bpy.utils.register_class(LoadDataset)
    bpy.utils.register_class(AnimateObject)
    bpy.utils.register_class(InertialBlenderPanel)
    print("Done!")


def unregister():
    # TODO uncomment on public repo
    # TODO move to implicit unregistration (module)
    #addon_updater_ops.unregister()
    #bpy.utils.unregister_class(AutoUpdatePreferences)
    #bpy.utils.unregister_class(UpdaterPanel)
    bpy.utils.unregister_class(InertialBlenderPanel)
    bpy.utils.unregister_class(AnimateObject)
    bpy.utils.unregister_class(LoadDataset)


class UpdaterPanel(bpy.types.Panel):
    """Panel to demo popup notice and ignoring functionality"""
    bl_label = "Addon-updater Panel"
    bl_idname = "OBJECT_PT_hello"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_context = "objectmode"
    bl_category = "Tools"

    def draw(self, context):
        layout = self.layout

        # Call to check for update in background
        # note: built-in checks ensure it runs at most once
        # and will run in the background thread, not blocking
        # or hanging blender
        # Internally also checks to see if auto-check enabled
        # and if the time interval has passed
        addon_updater_ops.check_for_update_background()

        layout.label("Demo Updater Addon")
        layout.label("")

        col = layout.column()
        col.scale_y = 0.7
        col.label("If an update is ready,")
        col.label("popup triggered by opening")
        col.label("this panel, plus a box ui")

        # could also use your own custom drawing
        # based on shared variables
        if addon_updater_ops.updater.update_ready == True:
            layout.label("Custom update message", icon="INFO")
        layout.label("")

        # call built-in function with draw code/checks
        addon_updater_ops.update_notice_box_ui(self, context)


# demo bare-bones preferences
class AutoUpdatePreferences(bpy.types.AddonPreferences):
    bl_idname = __package__

    # addon updater preferences

    auto_check_update = bpy.props.BoolProperty(
        name="Auto-check for Update",
        description="If enabled, auto-check for updates using an interval",
        default=False,
    )

    updater_interval_months = bpy.props.IntProperty(
        name='Months',
        description="Number of months between checking for updates",
        default=0,
        min=0
    )
    updater_interval_days = bpy.props.IntProperty(
        name='Days',
        description="Number of days between checking for updates",
        default=7,
        min=0,
        max=31
    )
    updater_interval_hours = bpy.props.IntProperty(
        name='Hours',
        description="Number of hours between checking for updates",
        default=0,
        min=0,
        max=23
    )
    updater_interval_minutes = bpy.props.IntProperty(
        name='Minutes',
        description="Number of minutes between checking for updates",
        default=0,
        min=0,
        max=59
    )


    def draw(self, context):
        layout = self.layout
        # col = layout.column() # works best if a column, or even just self.layout
        mainrow = layout.row()
        col = mainrow.column()
        # updater draw function
        # could also pass in col as third arg
        addon_updater_ops.update_settings_ui(self, context)

if __name__ == "__main__":
    register()
